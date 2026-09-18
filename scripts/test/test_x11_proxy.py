"""X11 proxy tests; no X server is required."""

import array
import importlib.util
import os
from pathlib import Path
import socket
import sys
import tempfile
import threading


script_dir = Path(__file__).resolve().parents[2] / '.devcontainer/scripts'
sys.path.insert(0, str(script_dir))
spec = importlib.util.spec_from_file_location('x11_proxy', script_dir / 'x11_proxy.py')
proxy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(proxy)


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
