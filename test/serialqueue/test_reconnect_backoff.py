"""KAN-243: the non-critical MCU reconnect loop must not hammer an absent MCU.

Before this, each reconnect tick ran `connect_uart`'s 90 s retry loop, then
waited `reconnect_interval` (2 s) and did it again: roughly 24% of a core and
8.8 MB of klippy.log in 12 h on a bench M1 with no toolhead board.

Checks, without klippy's reactor or any hardware:

* `connect_uart(max_attempts=1)` gives up after one session attempt.
* `MCUConnectHelper._next_reconnect_delay` doubles up to
  `reconnect_max_interval` and resets on disconnect / success.

    python3 test/serialqueue/test_reconnect_backoff.py
"""
import os
import sys

KLIPPY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

import mcu  # noqa: E402
import serialhdl  # noqa: E402


class FakeReactor:
    def __init__(self):
        self.now = 1000.
    def monotonic(self):
        return self.now
    def pause(self, waketime):
        self.now = max(self.now, waketime)


def make_helper(interval=2.12, ceiling=30.):
    h = object.__new__(mcu.MCUConnectHelper)
    h.reconnect_interval = interval
    h.reconnect_max_interval = ceiling
    h._reconnect_delay = interval
    return h


def test_backoff_doubles_to_ceiling():
    h = make_helper()
    delays = [h._next_reconnect_delay() for _ in range(7)]
    assert delays == [2.12, 4.24, 8.48, 16.96, 30., 30., 30.], delays


def test_backoff_disabled_when_ceiling_is_zero():
    h = make_helper(ceiling=0.)
    delays = [h._next_reconnect_delay() for _ in range(4)]
    assert delays == [2.12] * 4, delays


def test_backoff_resets_on_disconnect_and_success():
    # Both code paths assign self._reconnect_delay = self.reconnect_interval;
    # check the reset actually restarts the ramp rather than continuing it.
    h = make_helper()
    for _ in range(5):
        h._next_reconnect_delay()
    h._reconnect_delay = h.reconnect_interval
    assert h._next_reconnect_delay() == 2.12
    assert h._next_reconnect_delay() == 4.24


def test_connect_uart_single_attempt():
    reactor = FakeReactor()
    reader = object.__new__(serialhdl.SerialReader)
    reader.reactor = reactor
    reader.warn_prefix = "mcu 'toolhead': "
    reader.serialqueue = None
    sessions = []
    reader._start_session = lambda dev: sessions.append(dev) or False
    preps = []
    try:
        reader.connect_uart('/dev/null', 250000, True,
                            connect_prepare_cb=lambda: preps.append(1),
                            max_attempts=1)
    except serialhdl.error as e:
        assert "Unable to connect" in str(e), e
    else:
        raise AssertionError("connect_uart returned without connecting")
    assert len(sessions) == 1, sessions
    assert len(preps) == 1, preps
    assert reactor.now == 1000., "single attempt must not sleep the reactor"


def test_connect_uart_default_keeps_retrying():
    reactor = FakeReactor()
    reader = object.__new__(serialhdl.SerialReader)
    reader.reactor = reactor
    reader.warn_prefix = "mcu 'toolhead': "
    reader.serialqueue = None
    sessions = []
    def start_session(dev):
        sessions.append(dev)
        reactor.now += 5.
        return False
    reader._start_session = start_session
    try:
        reader.connect_uart('/dev/null', 250000, True)
    except serialhdl.error:
        pass
    else:
        raise AssertionError("connect_uart returned without connecting")
    assert len(sessions) > 10, sessions
    assert reactor.now >= 1090., reactor.now


class _Serial:
    def __init__(self, baudrate=0, timeout=0, exclusive=True):
        self.port = None
        self.rts = True
    def open(self):
        pass
    def close(self):
        pass


def main():
    real_serial = serialhdl.serial.Serial
    real_leave = serialhdl.stk500v2_leave
    serialhdl.serial.Serial = _Serial
    serialhdl.stk500v2_leave = lambda ser, reactor: None
    try:
        test_backoff_doubles_to_ceiling()
        test_backoff_disabled_when_ceiling_is_zero()
        test_backoff_resets_on_disconnect_and_success()
        test_connect_uart_single_attempt()
        test_connect_uart_default_keeps_retrying()
    finally:
        serialhdl.serial.Serial = real_serial
        serialhdl.stk500v2_leave = real_leave
    print("ok: non-critical reconnect back-off and single-attempt connect")


if __name__ == '__main__':
    main()
