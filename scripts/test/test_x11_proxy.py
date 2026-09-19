"""X11 proxy tests; no X server is required."""

import array
import importlib.util
import os
from pathlib import Path
import socket
import sys
import tempfile
import threading

import pytest


script_dir = Path(__file__).resolve().parents[2] / '.devcontainer/scripts'
sys.path.insert(0, str(script_dir))
spec = importlib.util.spec_from_file_location('x11_proxy', script_dir / 'x11_proxy.py')
proxy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(proxy)


@pytest.fixture
def isolated_runtime(tmp_path, monkeypatch):
    runtime = tmp_path / '.runtime'
    monkeypatch.setattr(proxy, 'RUNTIME_DIR', runtime)
    monkeypatch.setattr(proxy, 'PROXY_DIR', runtime / 'x11')
    monkeypatch.setattr(proxy, 'PID_FILE', runtime / 'x11-proxy.pid')
    monkeypatch.setattr(proxy, 'LOG_FILE', runtime / 'x11-proxy.log')
    return runtime


@pytest.mark.parametrize('display', ['', 'localhost:10.0', ':987654'])
def test_devcontainer_fallback_drops_stale_display(isolated_runtime, monkeypatch, display):
    isolated_runtime.mkdir()
    env_file = isolated_runtime / 'desktop.env'
    env_file.write_text('DISPLAY=:0\n')
    monkeypatch.setenv('DISPLAY', display)
    monkeypatch.setattr(sys, 'argv', ['x11_proxy.py', 'start', '--devcontainer'])

    assert proxy.main() == 0
    assert (isolated_runtime / 'x11').is_dir()
    assert all(not line.startswith('DISPLAY=') for line in env_file.read_text().splitlines())


def test_devcontainer_exports_display_only_after_proxy_ready(isolated_runtime, monkeypatch):
    monkeypatch.setenv('DISPLAY', ':2.0')
    monkeypatch.setattr(sys, 'argv', ['x11_proxy.py', 'start', '--devcontainer'])

    def ready_proxy(*_args):
        assert 'DISPLAY=' not in (isolated_runtime / 'desktop.env').read_text()
        return 0

    monkeypatch.setattr(proxy, 'start_proxy', ready_proxy)
    assert proxy.main() == 0
    assert (isolated_runtime / 'desktop.env').read_text() == 'DISPLAY=:2.0\n'


def test_cli_still_reports_proxy_failure(isolated_runtime, monkeypatch):
    monkeypatch.delenv('DISPLAY', raising=False)
    monkeypatch.setattr(sys, 'argv', ['x11_proxy.py', 'start'])
    assert proxy.main() == 1
    assert not (isolated_runtime / 'desktop.env').exists()


def test_pump_forwards_file_descriptors():
    sender, source = socket.socketpair()
    destination, receiver = socket.socketpair()
    thread = threading.Thread(target=proxy.pump, args=(source, destination))
    thread.start()

    with tempfile.TemporaryFile() as transferred_file:
        transferred_file.write(b'dri3')
        transferred_file.flush()
        sender.sendmsg(
            [b'x'],
            [(socket.SOL_SOCKET, socket.SCM_RIGHTS,
              array.array('i', [transferred_file.fileno()]))],
        )
        data, ancillary, _flags, _address = receiver.recvmsg(
            1, socket.CMSG_SPACE(array.array('i').itemsize)
        )

    assert data == b'x'
    descriptors = array.array('i')
    descriptors.frombytes(ancillary[0][2])
    received_fd = descriptors[0]
    try:
        os.lseek(received_fd, 0, os.SEEK_SET)
        assert os.read(received_fd, 4) == b'dri3'
    finally:
        os.close(received_fd)
        sender.close()
        receiver.close()
        source.close()
        destination.close()
        thread.join(timeout=1)

    assert not thread.is_alive()
