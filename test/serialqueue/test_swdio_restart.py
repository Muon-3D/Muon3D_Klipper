"""restart_method: swdio runs openocd without blocking the reactor.

`MCURestartHelper._restart_via_swdio` resets the toolhead MCU through
openocd. The non-critical reconnect loop calls it every 2-30 s while the
toolhead is unplugged, so it must wait through `reactor.pause()` and never
block klippy. These tests run the real method against a fake `openocd`
shell script on PATH and check each way it can end:

- openocd exits 0: the reset succeeds, with the expected arguments.
- openocd exits non-zero: CalledProcessError.
- openocd hangs: TimeoutExpired, and the process is killed.
- klippy leaves during the wait: the process is killed.
- openocd is not installed: OSError.
- openocd writes more than a pipe holds: it still finishes.

Linux only (a shell script stands in for openocd).

    python3 test/serialqueue/test_swdio_restart.py
"""
import os
import shutil
import stat
import subprocess
import sys
import tempfile
import time
import unittest

KLIPPY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

import mcu  # noqa: E402


class KlippyExit(Exception):
    pass


class FakeReactor:
    def __init__(self, exit_after=None):
        self.pauses = 0
        self.exit_after = exit_after
    def monotonic(self):
        return time.monotonic()
    def pause(self, waketime):
        self.pauses += 1
        if self.exit_after is not None and self.pauses >= self.exit_after:
            raise KlippyExit()
        time.sleep(max(0., waketime - time.monotonic()))
        return time.monotonic()


def make_helper(reactor, config_path):
    helper = object.__new__(mcu.MCURestartHelper)
    helper._reactor = reactor
    helper._name = 'toolhead'
    helper._openocd_config = config_path
    return helper


def alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    # A zombie still answers kill(0); it no longer holds the adapter.
    with open('/proc/%d/stat' % pid) as f:
        return f.read().split(')')[-1].split()[0] != 'Z'


@unittest.skipUnless(sys.platform.startswith('linux'), 'Linux only')
class SwdioRestart(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.args_file = os.path.join(self.dir, 'args')
        self.pid_file = os.path.join(self.dir, 'pid')
        self.old_path = os.environ['PATH']
        os.environ['PATH'] = self.dir + os.pathsep + self.old_path
        self.old_timeout = mcu.SWDIO_OPENOCD_TIMEOUT

    def tearDown(self):
        os.environ['PATH'] = self.old_path
        mcu.SWDIO_OPENOCD_TIMEOUT = self.old_timeout
        shutil.rmtree(self.dir)

    def fake_openocd(self, body):
        path = os.path.join(self.dir, 'openocd')
        with open(path, 'w') as f:
            f.write('#!/bin/sh\n'
                    'printf "%%s\\n" "$@" > %s\n'
                    'echo $$ > %s\n%s\n' % (self.args_file, self.pid_file,
                                            body))
        os.chmod(path, os.stat(path).st_mode | stat.S_IXUSR)

    def pid(self):
        with open(self.pid_file) as f:
            return int(f.read())

    def test_success_passes_config_and_reset(self):
        self.fake_openocd('echo "Info : target reset"\nexit 0')
        reactor = FakeReactor()
        make_helper(reactor, '/etc/openocd/toolhead.cfg')._restart_via_swdio()
        with open(self.args_file) as f:
            args = f.read().splitlines()
        self.assertEqual(args, ['-f', '/etc/openocd/toolhead.cfg',
                                '-c', 'init; reset run; exit'])

    def test_waits_through_the_reactor(self):
        self.fake_openocd('sleep 0.2\nexit 0')
        reactor = FakeReactor()
        make_helper(reactor, 'cfg')._restart_via_swdio()
        self.assertGreater(reactor.pauses, 5)

    def test_nonzero_exit_raises(self):
        self.fake_openocd('echo "Error: no device found"\nexit 1')
        with self.assertRaises(subprocess.CalledProcessError) as cm:
            make_helper(FakeReactor(), 'cfg')._restart_via_swdio()
        self.assertEqual(cm.exception.returncode, 1)
        self.assertIn('no device found', cm.exception.output)

    def test_hang_times_out_and_kills_openocd(self):
        self.fake_openocd('exec sleep 60')
        mcu.SWDIO_OPENOCD_TIMEOUT = 0.3
        start = time.monotonic()
        with self.assertRaises(subprocess.TimeoutExpired):
            make_helper(FakeReactor(), 'cfg')._restart_via_swdio()
        self.assertLess(time.monotonic() - start, 5.)
        self.assertFalse(alive(self.pid()))

    def test_klippy_exit_mid_reset_kills_openocd(self):
        self.fake_openocd('exec sleep 60')
        with self.assertRaises(KlippyExit):
            make_helper(FakeReactor(exit_after=3), 'cfg')._restart_via_swdio()
        self.assertFalse(alive(self.pid()))

    def test_openocd_missing_raises_oserror(self):
        os.environ['PATH'] = self.dir   # no openocd in it
        with self.assertRaises(OSError):
            make_helper(FakeReactor(), 'cfg')._restart_via_swdio()

    def test_large_output_does_not_stall(self):
        # 256 KiB is four times a Linux pipe buffer.
        self.fake_openocd('head -c 262144 /dev/zero | tr "\\0" x\nexit 0')
        mcu.SWDIO_OPENOCD_TIMEOUT = 5.
        make_helper(FakeReactor(), 'cfg')._restart_via_swdio()


if __name__ == '__main__':
    unittest.main()
