#!/usr/bin/env python3
"""Expose the host X11 socket from a Docker-mountable project directory.

Snap Docker has a private /tmp, so bind-mounting /tmp/.X11-unix gives the
container an empty directory.  This proxy listens below the repository and
forwards each connection to the real host X11 socket.  The same path works
with classic Docker and WSLg, keeping the devcontainer files portable.
"""

from __future__ import annotations

import argparse
import array
import getpass
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import threading
import time

from display_scale import detect_scale


SCRIPT_PATH = Path(__file__).resolve()
DEVCONTAINER_DIR = SCRIPT_PATH.parent.parent
RUNTIME_DIR = DEVCONTAINER_DIR / ".runtime"
PROXY_DIR = RUNTIME_DIR / "x11"
PID_FILE = RUNTIME_DIR / "x11-proxy.pid"
LOG_FILE = RUNTIME_DIR / "x11-proxy.log"


def display_number(display: str) -> str:
    match = re.fullmatch(r"(?:unix)?:(\d+)(?:\.\d+)?", display)
    if not match:
        raise RuntimeError(
            f"DISPLAY={display!r} is not a local X11 display; expected :0, :1, etc."
        )
    return match.group(1)


def process_is_our_proxy(pid: int) -> bool:
    try:
        command = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ")
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return False
    return str(SCRIPT_PATH).encode() in command and b" serve " in b" " + command + b" "


def stop_previous_proxy() -> None:
    try:
        pid = int(PID_FILE.read_text(encoding="utf-8").strip())
    except (FileNotFoundError, ValueError):
        return

    if process_is_our_proxy(pid):
        os.kill(pid, signal.SIGTERM)
        for _ in range(20):
            if not process_is_our_proxy(pid):
                break
            time.sleep(0.05)
    PID_FILE.unlink(missing_ok=True)


def allow_host_user(display: str) -> None:
    try:
        subprocess.run(
            ["xhost", f"+SI:localuser:{getpass.getuser()}"],
            env={**os.environ, "DISPLAY": display},
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except FileNotFoundError:
        pass


def start_proxy(scale_override=None, output=None, watch_labels=()) -> int:
    # The desktop devcontainer always bind-mounts PROXY_DIR. Create it before
    # any X11 validation so fallback paths still have a valid mount source.
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    PROXY_DIR.mkdir(mode=0o777, parents=True, exist_ok=True)
    os.chmod(PROXY_DIR, 0o777)

    display = os.environ.get("DISPLAY", "")
    if not display:
        print(
            "X11 proxy: DISPLAY is empty. Open a graphical Linux/WSLg session first.",
            file=sys.stderr,
        )
        return 1

    try:
        number = display_number(display)
    except RuntimeError as error:
        print(f"X11 proxy: {error}", file=sys.stderr)
        return 1

    source = Path(f"/tmp/.X11-unix/X{number}")
    destination = PROXY_DIR / f"X{number}"
    if not source.is_socket():
        print(
            f"X11 proxy: host socket {source} does not exist. "
            "Check DISPLAY and XWayland/WSLg.",
            file=sys.stderr,
        )
        return 1

    try:
        scale, scale_source = detect_scale(scale_override, output)
    except ValueError as error:
        print(f"X11 proxy: {error}", file=sys.stderr)
        return 1

    (PROXY_DIR / "metro-scale").write_text(f"{scale:g}\n", encoding="utf-8")
    print(f"X11 proxy: Qt scale {scale:g} ({scale_source})")
    stop_previous_proxy()
    destination.unlink(missing_ok=True)
    allow_host_user(display)

    command = [sys.executable, str(SCRIPT_PATH), "serve", str(source), str(destination)]
    for label in watch_labels:
        command += ["--watch-label", label]
    with LOG_FILE.open("ab", buffering=0) as log:
        process = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=log,
            start_new_session=True,
        )
    PID_FILE.write_text(f"{process.pid}\n", encoding="utf-8")

    for _ in range(40):
        if destination.is_socket():
            print(f"X11 proxy: {display} is ready at {destination}")
            return 0
        if process.poll() is not None:
            break
        time.sleep(0.05)

    print(f"X11 proxy failed to start; see {LOG_FILE}", file=sys.stderr)
    return 1


MAX_ANCILLARY_FDS = 16


def close_received_fds(ancillary_data) -> None:
    """Close SCM_RIGHTS descriptors after they have been forwarded."""
    for level, kind, data in ancillary_data:
        if level != socket.SOL_SOCKET or kind != socket.SCM_RIGHTS:
            continue
        descriptors = array.array("i")
        descriptors.frombytes(data[: len(data) - (len(data) % descriptors.itemsize)])
        for descriptor in descriptors:
            try:
                os.close(descriptor)
            except OSError:
                pass


def pump(source: socket.socket, destination: socket.socket) -> None:
    ancillary_size = socket.CMSG_SPACE(MAX_ANCILLARY_FDS * array.array("i").itemsize)
    try:
        while True:
            chunk, ancillary_data, flags, _address = source.recvmsg(
                65536, ancillary_size
            )
            if not chunk:
                break
            if flags & socket.MSG_CTRUNC:
                close_received_fds(ancillary_data)
                raise RuntimeError("X11 proxy received too many file descriptors")
            try:
                sent = destination.sendmsg([chunk], ancillary_data)
                if sent < len(chunk):
                    destination.sendall(chunk[sent:])
            finally:
                close_received_fds(ancillary_data)
    except (BrokenPipeError, ConnectionResetError, OSError, RuntimeError):
        pass
    finally:
        try:
            destination.shutdown(socket.SHUT_WR)
        except OSError:
            pass


def handle_client(client: socket.socket, source_path: Path) -> None:
    upstream = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        upstream.connect(str(source_path))
        outgoing = threading.Thread(target=pump, args=(client, upstream), daemon=True)
        incoming = threading.Thread(target=pump, args=(upstream, client), daemon=True)
        outgoing.start()
        incoming.start()
        outgoing.join()
        incoming.join()
    except OSError:
        pass
    finally:
        upstream.close()
        client.close()


WATCH_INTERVAL = 2.0
# Covers a container recreate (old one gone, new one not yet running). Before the first
# container exists (an image build may take long) the watcher does not time out;
# scripts/desktop.sh stops the proxy when `compose up` fails.
WATCH_GRACE = 10.0


def containers_running(labels) -> bool | None:
    """True/False if a matching container runs; None if Docker is not installed.

    A failed or timed-out query counts as running: only a successful empty answer shows
    that the container is gone, so a briefly unavailable daemon keeps the proxy alive.
    """
    command = ["docker", "ps", "-q"]
    for label in labels:
        command += ["--filter", f"label={label}"]
    try:
        result = subprocess.run(
            command, capture_output=True, text=True, timeout=10, check=False
        )
    except FileNotFoundError:
        return None
    except subprocess.TimeoutExpired:
        return True
    if result.returncode != 0:
        return True
    return bool(result.stdout.strip())


def watch_containers(labels, stopping: threading.Event) -> None:
    """Request shutdown once a seen container with all `labels` no longer runs."""
    seen = False
    last_seen = time.monotonic()
    while not stopping.wait(WATCH_INTERVAL):
        running = containers_running(labels)
        if running is None:
            print("X11 proxy: docker not found; container watch disabled", flush=True)
            return
        now = time.monotonic()
        if running:
            seen = True
            last_seen = now
        elif seen and now - last_seen > WATCH_GRACE:
            print("X11 proxy: container is gone; stopping", flush=True)
            stopping.set()
            return


def serve(source_path: Path, destination_path: Path, watch_labels=()) -> int:
    destination_path.parent.mkdir(mode=0o777, parents=True, exist_ok=True)
    destination_path.unlink(missing_ok=True)
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    stopping = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stopping.set()

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    if watch_labels:
        threading.Thread(
            target=watch_containers, args=(watch_labels, stopping), daemon=True
        ).start()
    try:
        listener.bind(str(destination_path))
        os.chmod(destination_path, 0o777)
        listener.listen(16)
        listener.settimeout(0.5)
        while not stopping.is_set():
            try:
                client, _ = listener.accept()
            except socket.timeout:
                continue
            threading.Thread(
                target=handle_client, args=(client, source_path), daemon=True
            ).start()
    finally:
        listener.close()
        destination_path.unlink(missing_ok=True)
    return 0


def main() -> int:
    global RUNTIME_DIR, PROXY_DIR, PID_FILE, LOG_FILE
    parser = argparse.ArgumentParser()
    parser.add_argument('--runtime-dir', type=Path,
                        help='Separate proxy state directory for CLI Compose')
    subparsers = parser.add_subparsers(dest="command", required=True)
    start_parser = subparsers.add_parser("start")
    start_parser.add_argument("--scale", help="Explicit Qt scale, e.g. 1, 1.5, 2")
    start_parser.add_argument("--output", help="Host compositor output name")
    start_parser.add_argument(
        "--watch-label", action="append", default=[], metavar="KEY=VALUE",
        help="Exit when no running container has all given labels (repeatable)",
    )
    start_parser.add_argument(
        "--devcontainer", action="store_true",
        help="Write desktop.env and allow Dev Containers forwarding on proxy failure",
    )
    subparsers.add_parser("stop")
    serve_parser = subparsers.add_parser("serve")
    serve_parser.add_argument("source", type=Path)
    serve_parser.add_argument("destination", type=Path)
    serve_parser.add_argument("--watch-label", action="append", default=[])
    arguments = parser.parse_args()
    if arguments.runtime_dir is not None:
        RUNTIME_DIR = arguments.runtime_dir.resolve()
        PROXY_DIR = RUNTIME_DIR / 'x11'
        PID_FILE = RUNTIME_DIR / 'x11-proxy.pid'
        LOG_FILE = RUNTIME_DIR / 'x11-proxy.log'

    if arguments.command == "start":
        if not arguments.devcontainer:
            return start_proxy(arguments.scale, arguments.output, arguments.watch_label)

        # Docker reads this file at container creation. An absent DISPLAY lets
        # Dev Containers forward X11; a host TCP DISPLAY would suppress that
        # forwarding and point at the wrong localhost inside the container.
        RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
        env_file = RUNTIME_DIR / "desktop.env"
        env_file.write_text("# DISPLAY is provided by Dev Containers.\n", encoding="utf-8")
        result = start_proxy(arguments.scale, arguments.output, arguments.watch_label)
        if result == 0:
            env_file.write_text(f"DISPLAY={os.environ['DISPLAY']}\n", encoding="utf-8")
        else:
            print("X11 proxy unavailable; allowing Dev Containers GUI forwarding")
        return 0
    if arguments.command == "stop":
        stop_previous_proxy()
        return 0
    return serve(arguments.source, arguments.destination, arguments.watch_label)


if __name__ == "__main__":
    raise SystemExit(main())
