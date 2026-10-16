# Tests for klippy/extras/z_power.py: the decision, band and debounce logic,
# and the power-up / fault sequencing run against stub hardware.
import os
import sys
import unittest

KLIPPY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

from extras import z_power  # noqa: E402
from extras.z_power import (ABSENT, UNLOCKED, UNKNOWN, NO_PLATE, POWERING,
                            READY, INHIBITED, FAULT)  # noqa: E402

BANDS = [('absent', (0.90, 1.0)), ('unlocked', (0.40, 0.60)),
         ('locked', (0.0, 0.20))]


class TestDecide(unittest.TestCase):
    def test_comes_up_only_when_locked_with_a_plate(self):
        self.assertEqual(z_power.decide('locked', True, None, False)[0],
                         POWERING)
        self.assertTrue(z_power.decide('locked', True, None, False)[2])

    def test_every_other_case_is_off(self):
        cases = [(('absent', True, None, False), ABSENT),
                 (('unlocked', True, None, False), UNLOCKED),
                 ((None, True, None, False), UNKNOWN),
                 (('locked', False, None, False), NO_PLATE),
                 (('locked', None, None, False), NO_PLATE),
                 (('locked', True, None, True), INHIBITED),
                 (('locked', True, 'boom', False), FAULT)]
        for args, state in cases:
            got_state, _reason, enable = z_power.decide(*args)
            self.assertEqual(got_state, state, args)
            self.assertFalse(enable, args)

    def test_fault_outranks_everything(self):
        self.assertEqual(z_power.decide('absent', False, 'x', True)[0], FAULT)


class TestBandsAndDebounce(unittest.TestCase):
    def test_classify(self):
        self.assertEqual(z_power.classify_band(0.99, BANDS), 'absent')
        self.assertEqual(z_power.classify_band(0.5, BANDS), 'unlocked')
        self.assertEqual(z_power.classify_band(0.1, BANDS), 'locked')
        self.assertIsNone(z_power.classify_band(0.3, BANDS))

    def test_debounce_needs_consecutive_samples(self):
        d = z_power.Debounce(3)
        self.assertIsNone(d.update('locked'))
        self.assertIsNone(d.update('locked'))
        self.assertEqual(d.update('locked'), 'locked')
        # A single glitch does not change the value
        self.assertEqual(d.update('absent'), 'locked')
        self.assertEqual(d.update('locked'), 'locked')
        self.assertEqual(d.update('absent'), 'locked')
        self.assertEqual(d.update('absent'), 'locked')
        self.assertEqual(d.update('absent'), 'absent')


class CommandError(Exception):
    pass


class StubMCU:
    def estimated_print_time(self, eventtime):
        return eventtime


class StubPin:
    def __init__(self):
        self.writes = []
    def set_digital(self, print_time, value):
        self.writes.append((print_time, value))


class StubHeater:
    def __init__(self):
        self.last_temp = 25.
        self.last_temp_time = 0.
        self.target = 0.
        self.power = 0.
    def set_temp(self, degrees):
        self.target = degrees
    def get_temp(self, eventtime):
        return self.last_temp, self.target
    def get_status(self, eventtime):
        return {'power': self.power}


class StubPrinter:
    command_error = CommandError
    def is_shutdown(self):
        return False
    def lookup_object(self, name, default=None):
        return default


class StubReactor:
    NEVER = 9e99
    def register_callback(self, cb):
        cb(0.)


class Harness(unittest.TestCase):
    def make(self):
        zp = z_power.ZPower.__new__(z_power.ZPower)
        zp.printer = StubPrinter()
        zp.reactor = StubReactor()
        zp.name = 'z_power'
        zp.mcu_enable = StubPin()
        zp.enable_mcu = StubMCU()
        zp.enabled = False
        zp.last_enable_time = 0.
        zp.power_good = False
        zp.present_ratio = None
        zp.bands = BANDS
        zp.z_band = z_power.Debounce(1)
        zp.current = None
        zp.max_current, zp.max_current_time = 5.0, 0.5
        zp.idle_current, zp.idle_current_time = 1.0, 2.0
        zp.supply_voltage = 45.
        zp.over_since = zp.idle_over_since = None
        zp.heater_name = 'heater_bed'
        zp.heater = StubHeater()
        zp.plate_min_temp, zp.plate_max_temp = -10., 150.
        zp.plate = z_power.Debounce(1)
        zp.last_plate_time = -1.
        zp.power_good_timeout = 0.5
        zp.powering_since = None
        zp.state, zp.reason = UNKNOWN, "starting"
        zp.fault = None
        zp.inhibited = False
        zp.scripts = []
        zp._run_template = lambda tmpl: zp.scripts.append(tmpl)
        zp.lost_template = 'LOST'
        zp.fault_template = 'FAULT'
        # Install the interlock the way _handle_connect does
        raw = zp.heater.set_temp
        def guarded(degrees):
            if degrees and zp.state != READY:
                raise CommandError("refused")
            raw(degrees)
        zp._raw_set_temp = raw
        zp.heater.set_temp = guarded
        return zp

    def tick(self, zp, t, ratio=0.1, temp=25.):
        zp.present_ratio = ratio
        zp.heater.last_temp = temp
        zp.heater.last_temp_time = t
        zp._poll(t)

    def test_power_up_needs_pgood(self):
        zp = self.make()
        self.tick(zp, 1.0)
        self.assertEqual(zp.state, POWERING)
        self.assertTrue(zp.enabled)
        self.assertEqual(zp.mcu_enable.writes[-1][1], 1)
        zp.power_good = True
        self.tick(zp, 1.1)
        self.assertEqual(zp.state, READY)
        zp.heater.set_temp(60.)
        self.assertEqual(zp.heater.target, 60.)

    def test_heating_refused_until_ready(self):
        zp = self.make()
        self.tick(zp, 1.0, ratio=0.99)
        self.assertEqual(zp.state, ABSENT)
        with self.assertRaises(CommandError):
            zp.heater.set_temp(60.)
        zp.heater.set_temp(0.)  # turning off is always allowed

    def test_pgood_timeout_latches_fault(self):
        zp = self.make()
        self.tick(zp, 1.0)
        self.tick(zp, 1.7)
        self.assertEqual(zp.state, FAULT)
        self.assertFalse(zp.enabled)
        self.assertIn('FAULT', zp.scripts)
        # Stays latched with good inputs until reset
        zp.power_good = True
        self.tick(zp, 2.0)
        self.assertEqual(zp.state, FAULT)
        zp.fault = None
        self.tick(zp, 2.1)
        self.assertEqual(zp.state, POWERING)

    def test_pgood_drop_after_ready_latches_fault(self):
        zp = self.make()
        self.tick(zp, 1.0)
        zp.power_good = True
        self.tick(zp, 1.1)
        zp.heater.set_temp(60.)
        zp.power_good = False
        self.tick(zp, 1.2)
        self.assertEqual(zp.state, FAULT)
        self.assertFalse(zp.enabled)
        self.assertEqual(zp.heater.target, 0.)

    def test_plate_removal_drops_power_without_lost_gcode(self):
        zp = self.make()
        self.tick(zp, 1.0)
        zp.power_good = True
        self.tick(zp, 1.1)
        zp.heater.set_temp(60.)
        self.tick(zp, 1.2, temp=-24.)
        self.assertEqual(zp.state, NO_PLATE)
        self.assertFalse(zp.enabled)
        self.assertEqual(zp.heater.target, 0.)
        self.assertNotIn('LOST', zp.scripts)  # bed_removal_detector pauses

    def test_unlocking_runs_lost_gcode(self):
        zp = self.make()
        self.tick(zp, 1.0)
        zp.power_good = True
        self.tick(zp, 1.1)
        self.tick(zp, 1.2, ratio=0.5)
        self.assertEqual(zp.state, UNLOCKED)
        self.assertFalse(zp.enabled)
        self.assertIn('LOST', zp.scripts)

    def test_current_with_heater_off_latches_fault(self):
        zp = self.make()
        self.tick(zp, 1.0)
        zp.power_good = True
        self.tick(zp, 1.1)
        zp.current = 2.0
        self.tick(zp, 1.2)
        self.tick(zp, 3.3)
        self.assertEqual(zp.state, FAULT)
        self.assertIn('heater off', zp.fault)

    def test_heater_current_is_not_a_fault(self):
        zp = self.make()
        self.tick(zp, 1.0)
        zp.power_good = True
        self.tick(zp, 1.1)
        zp.heater.power = 1.0
        zp.current = 3.7
        for t in (1.2, 2.0, 3.0, 4.0):
            self.tick(zp, t)
        self.assertEqual(zp.state, READY)

    def test_overcurrent_latches_fault(self):
        zp = self.make()
        self.tick(zp, 1.0)
        zp.power_good = True
        self.tick(zp, 1.1)
        zp.heater.power = 1.0
        zp.current = 5.8
        self.tick(zp, 1.2)
        self.tick(zp, 1.8)
        self.assertEqual(zp.state, FAULT)
        self.assertIn('overcurrent', zp.fault)


if __name__ == '__main__':
    unittest.main()
