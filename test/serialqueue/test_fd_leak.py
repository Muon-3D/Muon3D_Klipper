"""KAN-243: a serialqueue must give back every file descriptor it took.

`serialqueue_alloc` opens a wake-up pipe for its background thread. Klippy
allocates one serialqueue per connect attempt, and the non-critical MCU
reconnect path makes an attempt every few seconds for as long as the MCU is
absent. With the pipe never closed the process ran out of descriptors after
~500 attempts, `serialqueue_alloc` returned NULL, and the background thread
segfaulted in `serialqueue_pull` -- every 69 minutes, to the second.

This builds the real `c_helper.so` via `chelper.get_ffi()`, so it needs gcc
and cffi (both in the CI python-env). It uses a pipe in place of the serial
port so no hardware or MCU dictionary is involved.

    python3 test/serialqueue/test_fd_leak.py
"""
import os
import sys

KLIPPY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

import chelper  # noqa: E402


def open_fds():
    return len(os.listdir('/proc/self/fd'))


def alloc_exit_free(ffi_main, ffi_lib, serial_fd):
    sq = ffi_lib.serialqueue_alloc(serial_fd, b'u', 0, b'serialq test')
    assert sq != ffi_main.NULL, "serialqueue_alloc returned NULL"
    ffi_lib.serialqueue_exit(sq)
    ffi_lib.serialqueue_free(sq)


def test_alloc_free_returns_fds(ffi_main, ffi_lib, iterations=600):
    rfd, wfd = os.pipe()
    try:
        alloc_exit_free(ffi_main, ffi_lib, rfd)
        before = open_fds()
        for _ in range(iterations):
            alloc_exit_free(ffi_main, ffi_lib, rfd)
        after = open_fds()
    finally:
        os.close(rfd)
        os.close(wfd)
    assert after == before, (
        "serialqueue leaked %d fds over %d alloc/free cycles"
        % (after - before, iterations))


def test_alloc_fails_cleanly_at_fd_limit(ffi_main, ffi_lib):
    # Fill the descriptor table so pipe() fails with EMFILE, and check the
    # allocator reports NULL rather than handing back a half-built queue.
    import resource
    soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
    resource.setrlimit(resource.RLIMIT_NOFILE, (min(soft, 64), hard))
    rfd, wfd = os.pipe()
    filler = []
    try:
        while True:
            try:
                filler.append(os.dup(rfd))
            except OSError:
                break
        # Leave exactly one descriptor free: pipe() needs two.
        os.close(filler.pop())
        sq = ffi_lib.serialqueue_alloc(rfd, b'u', 0, b'serialq test')
        assert sq == ffi_main.NULL, "expected NULL at the fd limit"
        # The failed alloc must not have consumed the one free slot.
        probe = os.dup(rfd)
        os.close(probe)
    finally:
        for fd in filler:
            os.close(fd)
        os.close(rfd)
        os.close(wfd)
        resource.setrlimit(resource.RLIMIT_NOFILE, (soft, hard))


def main():
    ffi_main, ffi_lib = chelper.get_ffi()
    test_alloc_free_returns_fds(ffi_main, ffi_lib)
    print("ok: serialqueue alloc/free is fd-neutral")
    test_alloc_fails_cleanly_at_fd_limit(ffi_main, ffi_lib)
    print("ok: serialqueue_alloc returns NULL at the fd limit")


if __name__ == '__main__':
    main()
