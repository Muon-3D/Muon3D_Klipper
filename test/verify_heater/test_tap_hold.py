# verify_heater and heater holds (hold_pwm): time a probe tap holds the
# hotend heater off is not charged as error.
#
# On sloe (2026-11-10) a bed mesh at OOZE_TEMP held the hotend heater off for
# every tap; the hotend drifted from 130 to 113 C and verify_heater shut the
# printer down with "not heating at expected rate". These tests run
# HeaterCheck.check_event on a fake heater, once a second, as the reactor
# does.
import os, sys, unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..',
                                'klippy'))
from extras import verify_heater  # noqa: E402

NEVER = 9999999999999999.


class FakeReactor:
    NEVER = NEVER


class FakePrinter:
    def __init__(self):
        self.shutdown_msg = None
    def get_reactor(self):
        return FakeReactor()
    def register_event_handler(self, *args):
        pass
    def invoke_shutdown(self, msg):
        self.shutdown_msg = msg


class FakeConfig:
    def __init__(self, printer):
        self.printer = printer
    def get_printer(self):
        return self.printer
    def get_name(self):
        return 'verify_heater extruder'
    def getfloat(self, key, default, **kw):
        return default


class FakeHeater:
    def __init__(self):
        self.temp, self.target = 130., 130.
        self.holds = {}
        self.released = None
    def get_temp(self, eventtime):
        return self.temp, self.target
    def get_pwm_holds(self):
        return dict(self.holds)
    def get_last_hold_release(self):
        return self.released


def make():
    printer = FakePrinter()
    check = verify_heater.HeaterCheck(FakeConfig(printer))
    heater = FakeHeater()
    check.heater = heater
    return check, heater, printer


def run(check, heater, printer, seconds, temp_at, held_at, start=0.):
    """Step the check once a second. temp_at(t) is the hotend temperature,
    held_at(t) whether a tap holds the heater at t. Holds come and go
    between checks, as taps do, so each one-second step records a release
    when the step had any hold in it."""
    t = start
    while t < start + seconds:
        t += 1.
        heater.temp = temp_at(t)
        held_now = held_at(t)
        held_during = any(held_at(t - k / 10.) for k in range(10))
        heater.holds = {'tap': None} if held_now else {}
        if held_during and not held_now:
            heater.released = t - 0.1
        check.check_event(t)
        if printer.shutdown_msg:
            return t
    return None


def mesh_holds(t):
    # A tap about every 1.6 s, each holding the heater for 1.2 s.
    return (t % 1.6) < 1.2


class TestTapHold(unittest.TestCase):
    def test_a_mesh_that_cools_the_hotend_does_not_fault(self):
        check, heater, printer = make()
        # 130 C down to 113 C over a 90 s mesh, as on sloe.
        failed = run(check, heater, printer, 90.,
                     lambda t: max(113., 130. - 0.2 * t), mesh_holds)
        self.assertIsNone(failed, printer.shutdown_msg)

    def test_the_same_drift_without_holds_still_faults(self):
        check, heater, printer = make()
        failed = run(check, heater, printer, 90.,
                     lambda t: max(113., 130. - 0.2 * t), lambda t: False)
        self.assertIsNotNone(failed)
        self.assertIn('not heating at expected rate', printer.shutdown_msg)

    def test_a_heater_that_does_not_recover_after_the_mesh_faults(self):
        check, heater, printer = make()
        self.assertIsNone(run(check, heater, printer, 90.,
                              lambda t: max(113., 130. - 0.2 * t),
                              mesh_holds))
        # Taps over, heater free, but the temperature stays at 113 C.
        failed = run(check, heater, printer, 60., lambda t: 113.,
                     lambda t: False, start=90.)
        self.assertIsNotNone(failed)
        self.assertLess(failed - 90., 20.)

    def test_a_heater_that_recovers_after_the_mesh_does_not_fault(self):
        check, heater, printer = make()
        self.assertIsNone(run(check, heater, printer, 90.,
                              lambda t: max(113., 130. - 0.2 * t),
                              mesh_holds))
        # Free again: back up at 1.5 C/s, then holding target.
        failed = run(check, heater, printer, 60.,
                     lambda t: min(130., 113. + 1.5 * (t - 90.)),
                     lambda t: False, start=90.)
        self.assertIsNone(failed, printer.shutdown_msg)


if __name__ == '__main__':
    unittest.main()
