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


def config_bands():
    """The present_*_range bands from the B1 config, in decide() order."""
    path = os.path.join(KLIPPY, '..', 'core', 'M1', 'core.cfg')
    bands = {}
    for line in open(path):
        line = line.split('#')[0].strip()
        for band in ('absent', 'unlocked', 'locked'):
            key = 'present_%s_range:' % (band,)
            if line.startswith(key):
                lo, hi = line[len(key):].split(',')
                bands[band] = (float(lo), float(hi))
    return [(band, bands[band]) for band in ('absent', 'unlocked', 'locked')]


class TestConfigBands(unittest.TestCase):
    # Ratios from the backboard schematic and the gimlet bench.
    def test_each_lever_state_lands_in_its_band(self):
        bands = config_bands()
        for ratio, band in ((0.977, 'absent'), (1.0, 'absent'),
                            (0.0, 'unlocked'), (0.348, 'locked'),
                            (0.5, 'locked')):
            self.assertEqual(z_power.classify_band(ratio, bands), band,
                             ratio)

    def test_bands_do_not_overlap(self):
        spans = sorted(span for _band, span in config_bands())
        for (_lo1, hi1), (lo2, _hi2) in zip(spans, spans[1:]):
            self.assertLess(hi1, lo2)


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


class StubDetector:
    def __init__(self):
        self.plate = True
    def plate_present(self):
        return self.plate


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
        zp.raw_current = None
        zp.current_offset = None
        zp.max_current, zp.max_current_time = 5.0, 0.5
        zp.idle_current, zp.idle_current_time = 1.0, 2.0
        zp.supply_voltage = 45.
        zp.over_since = zp.idle_over_since = None
        zp.heater_name = 'heater_bed'
        zp.heater = StubHeater()
        zp.detector = StubDetector()
        zp.power_good_timeout = 0.5
        zp.powering_since = None
        zp.state, zp.reason = UNKNOWN, "starting"
        zp.fault = None
        zp.inhibited = False
        zp.scripts = []
        zp._run_template = lambda tmpl: zp.scripts.append(tmpl)
        zp.lost_template = 'LOST'
        zp.fault_template = 'FAULT'
        zp.z_change_template = 'ZCHANGE'
        zp.last_band = None
        zp.band_seen = False
        # Install the interlock the way _handle_connect does
        raw = zp.heater.set_temp
        def guarded(degrees):
            if degrees and zp.state != READY:
                raise CommandError("refused")
            raw(degrees)
        zp._raw_set_temp = raw
        zp.heater.set_temp = guarded
        return zp

    def tick(self, zp, t, ratio=0.1, plate=True):
        zp.present_ratio = ratio
        zp.detector.plate = plate
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
        self.tick(zp, 1.2, plate=False)
        self.assertEqual(zp.state, NO_PLATE)
        self.assertFalse(zp.enabled)
        self.assertEqual(zp.heater.target, 0.)
        self.assertNotIn('LOST', zp.scripts)  # bed_removal_detector pauses

    def test_unlocking_runs_the_column_hook_not_lost(self):
        zp = self.make()
        self.tick(zp, 1.0)
        zp.power_good = True
        self.tick(zp, 1.1)
        self.tick(zp, 1.2, ratio=0.5)
        self.assertEqual(zp.state, UNLOCKED)
        self.assertFalse(zp.enabled)
        self.assertNotIn('LOST', zp.scripts)
        self.assertIn('ZCHANGE', zp.scripts)
        self.assertEqual(zp.z_band.value, 'unlocked')

    def test_first_reading_is_not_a_change(self):
        zp = self.make()
        self.tick(zp, 1.0, ratio=0.99)
        self.assertEqual(zp.scripts, [])

    def test_removal_and_unknown_are_reported(self):
        zp = self.make()
        self.tick(zp, 1.0)                     # locked
        self.tick(zp, 1.1, ratio=0.99)         # pulled off
        self.assertEqual(zp.scripts.count('ZCHANGE'), 1)
        self.assertEqual(zp.z_band.value, 'absent')
        self.tick(zp, 1.2, ratio=0.3)          # matches no band
        self.assertEqual(zp.scripts.count('ZCHANGE'), 2)
        self.assertIsNone(zp.z_band.value)

    def test_inhibit_runs_lost_gcode(self):
        zp = self.make()
        self.tick(zp, 1.0)
        zp.power_good = True
        self.tick(zp, 1.1)
        zp.inhibited = True
        self.tick(zp, 1.2)
        self.assertEqual(zp.state, INHIBITED)
        self.assertIn('LOST', zp.scripts)

    def test_current_with_heater_off_latches_fault(self):
        zp = self.make()
        self.tick(zp, 1.0)
        zp.power_good = True
        self.tick(zp, 1.1)
        zp.raw_current = 2.0
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
        zp.raw_current = 3.7
        for t in (1.2, 2.0, 3.0, 4.0):
            self.tick(zp, t)
        self.assertEqual(zp.state, READY)

    def test_overcurrent_latches_fault(self):
        zp = self.make()
        self.tick(zp, 1.0)
        zp.power_good = True
        self.tick(zp, 1.1)
        zp.heater.power = 1.0
        zp.raw_current = 5.8
        self.tick(zp, 1.2)
        self.tick(zp, 1.8)
        self.assertEqual(zp.state, FAULT)
        self.assertIn('overcurrent', zp.fault)

    def test_imon_offset_is_learnt_while_off(self):
        zp = self.make()
        zp.raw_current = 0.11
        for t in (0.1, 0.2, 0.3):
            self.tick(zp, t, ratio=0.99)        # absent: feed off
        self.assertAlmostEqual(zp.current, 0., places=3)
        self.tick(zp, 1.0)                      # locked: feed comes on
        zp.power_good = True
        self.tick(zp, 1.1)
        zp.heater.power = 1.0
        zp.raw_current = 3.86
        self.tick(zp, 1.2)
        self.assertAlmostEqual(zp.current, 3.75, places=2)


    def test_unknown_plate_keeps_power_off(self):
        zp = self.make()
        self.tick(zp, 1.0, plate=None)
        self.assertEqual(zp.state, NO_PLATE)
        self.assertFalse(zp.enabled)

    def test_reasons_say_what_to_do(self):
        self.assertIn('lever', z_power.decide('unlocked', True, None,
                                              False)[1])
        self.assertIn('attach', z_power.decide('locked', False, None,
                                               False)[1])
        self.assertIn('Z_POWER_RESET', z_power.decide('locked', True, 'x',
                                                      False)[1])


class TestBedRemovalDetector(unittest.TestCase):
    """The detector is the single owner of plate presence."""
    def make(self):
        from extras import bed_removal_detector as brd
        import collections
        d = brd.BedRemovalDetector.__new__(brd.BedRemovalDetector)
        d.remove_thr, d.attach_thr = -15., -5.
        d.window = collections.deque(maxlen=2)
        d.last_sample_time = None
        d.bed_heater = StubHeater()
        d.bed_removed = False
        d.plate_known = False
        d.restore_target = None
        d.removals = []
        d.handle_bed_removal = lambda et: (d.removals.append(et),
                                           setattr(d, 'bed_removed', True))
        d.handle_bed_reconnection = lambda: setattr(d, 'bed_removed', False)
        class PR:
            is_paused = False
        d.pause_resume = PR()
        d.bed_check_interval = 0.1
        return d

    def feed(self, d, temps):
        for i, temp in enumerate(temps):
            d.bed_heater.last_temp = temp
            d.bed_heater.last_temp_time += 0.3
            d.check_bed_temperature(float(i))

    def test_unknown_until_measured(self):
        d = self.make()
        self.assertIsNone(d.plate_present())
        self.feed(d, [25.])
        self.assertIsNone(d.plate_present())    # window not full yet
        self.feed(d, [25.])
        self.assertTrue(d.plate_present())

    def test_boot_without_a_plate(self):
        d = self.make()
        self.feed(d, [-24., -24.])
        self.assertFalse(d.plate_present())
        self.assertEqual(len(d.removals), 1)

    def test_boot_between_thresholds_stays_unknown(self):
        d = self.make()
        self.feed(d, [-10., -10.])
        self.assertIsNone(d.plate_present())

    def test_repeated_polls_of_one_sample_count_once(self):
        d = self.make()
        d.bed_heater.last_temp, d.bed_heater.last_temp_time = 25., 1.
        for et in range(5):
            d.check_bed_temperature(float(et))
        self.assertEqual(len(d.window), 1)


if __name__ == '__main__':
    unittest.main()
