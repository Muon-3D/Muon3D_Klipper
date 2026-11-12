# Offline tests for Heater.hold_pwm: the "off" a hold sends must reach the
# mcu before its clock, even when a PID update is still queued.
#
# On sloe (2026-11-12) a probe tap began within 0.1 s of a PID update. The
# hold's "off" got the update's own clock, the serial queue held it until the
# mcu reached that clock (minclock), and the toolhead shut down with "Timer
# too close". These tests model that queue rule.
import os, sys, threading, unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..',
                                'klippy'))
from extras import heaters  # noqa: E402

# Host to mcu transfer time, generously: a few ms on the toolhead UART.
TRANSFER = 0.005


class FakePwm:
    """Records each command as (time, value, release). A command leaves the
    host's queue when the host is at `now` and the mcu has reached the
    previous command's time (minclock), whichever is later."""
    def __init__(self):
        self.now = 0.
        self.last_time = 0.
        self.sent = []
    def set_pwm(self, print_time, value):
        release = max(self.now, self.last_time)
        self.sent.append((print_time, value, release))
        self.last_time = print_time


def make(pwm_delay=0.2):
    h = heaters.Heater.__new__(heaters.Heater)
    h.name = 'extruder'
    h.lock = threading.Lock()
    h.pwm_holds = {}
    h.last_pwm_value = h.last_pwm_time = h.next_pwm_time = 0.
    h.target_temp = 135.
    h.verify_mainthread_time = 1e9
    h.pwm_delay = pwm_delay
    h.mcu_pwm = FakePwm()
    return h


class Test(unittest.TestCase):
    def assert_on_time(self, pwm):
        for t, value, release in pwm.sent:
            self.assertLess(release + TRANSFER, t,
                            "pwm %.2f at %.3f leaves at %.3f, too late"
                            % (value, t, release))

    def test_hold_just_after_a_pid_update(self):
        # The sloe case: update at 10.0 for 10.2, tap hold at 10.05.
        h = make()
        h.mcu_pwm.now = 10.
        h.set_pwm(10., 0.5)
        h.mcu_pwm.now = 10.05
        h.hold_pwm('tap', 10.05 + 0.1)
        self.assertEqual(h.mcu_pwm.sent[-1][1], 0.)
        self.assert_on_time(h.mcu_pwm)

    def test_hold_at_every_moment_of_an_update_cycle(self):
        for k in range(31):
            h = make()
            h.mcu_pwm.now = 10.
            h.set_pwm(10., 0.5)
            now = 10. + k * 0.01
            h.mcu_pwm.now = now
            h.hold_pwm('tap', now + 0.1)
            self.assert_on_time(h.mcu_pwm)
            # The heater still goes off soon: within the pending update
            # plus the gap.
            self.assertLessEqual(h.mcu_pwm.sent[-1][0],
                                 max(now + 0.1, 10.2 + heaters.PWM_QUEUE_GAP)
                                 + 1e-9)

    def test_hold_with_nothing_queued_is_not_delayed(self):
        h = make()
        h.mcu_pwm.now = 10.
        h.set_pwm(10., 0.5)
        h.mcu_pwm.now = 11.
        h.hold_pwm('tap', 11.1)
        self.assertAlmostEqual(h.mcu_pwm.sent[-1][0], 11.1)
        self.assert_on_time(h.mcu_pwm)

    def test_hold_with_heater_already_off_sends_nothing(self):
        h = make()
        h.mcu_pwm.now = 10.
        h.hold_pwm('tap', 10.1)
        self.assertEqual(h.mcu_pwm.sent, [])
        self.assertIn('tap', h.pwm_holds)

    def test_pid_after_release_is_on_time(self):
        h = make()
        h.mcu_pwm.now = 10.
        h.set_pwm(10., 0.5)
        h.mcu_pwm.now = 10.05
        h.hold_pwm('tap', 10.15)
        h.pwm_holds.clear()
        h.mcu_pwm.now = 10.3
        h.set_pwm(10.3, 0.6)
        self.assert_on_time(h.mcu_pwm)


if __name__ == '__main__':
    unittest.main()
