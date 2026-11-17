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


class TestNoDeadTests(unittest.TestCase):
    """CI runs this file as a script, so unittest.main() only sees the
    classes defined above it. Anything below it never runs (KAN-3421).
    Keep this class first in the file: it cannot catch itself below the
    guard."""
    def test_main_guard_is_the_last_statement(self):
        import ast
        with open(os.path.abspath(__file__)) as f:
            body = ast.parse(f.read()).body
        guards = [i for i, node in enumerate(body) if isinstance(node, ast.If)
                  and ast.unparse(node.test) == "__name__ == '__main__'"]
        self.assertEqual(len(guards), 1)
        below = [getattr(node, 'name', type(node).__name__)
                 for node in body[guards[0] + 1:]]
        self.assertEqual(below, [], "defined below the __main__ guard, so "
                         "they never run in CI: %s" % (below,))


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


class TestXyAllowed(unittest.TestCase):
    """X/Y may move only with the Z column plugged in (locked or not)."""
    def test_column_plugged_in(self):
        self.assertTrue(z_power.xy_allowed('locked'))
        self.assertTrue(z_power.xy_allowed('unlocked'))

    def test_absent_unknown_and_unread_refuse(self):
        # Folded, the Z axis is in the gantry's path; an unrecognised or
        # not yet read column fails closed.
        for band in ('absent', None, 'something_else'):
            self.assertFalse(z_power.xy_allowed(band), band)

    def test_development_override(self):
        for band in ('absent', None, 'locked'):
            self.assertTrue(z_power.xy_allowed(band, required=False), band)

    def test_unknown_after_a_fitted_column_allows(self):
        self.assertTrue(z_power.xy_allowed(None, was_fitted=True))

    def test_unknown_never_fitted_or_after_absent_refuses(self):
        self.assertFalse(z_power.xy_allowed(None, was_fitted=False))
        self.assertFalse(z_power.xy_allowed(None))

    def test_absent_and_unrecognised_refuse_even_after_fitted(self):
        for band in ('absent', 'something_else'):
            self.assertFalse(z_power.xy_allowed(band, was_fitted=True), band)


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
    def __init__(self):
        self.objects = {}
    def is_shutdown(self):
        return False
    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)


class StubReactor:
    NEVER = 9e99
    def register_callback(self, cb):
        cb(0.)
    def monotonic(self):
        return 0.


class StubPrintStats:
    def __init__(self):
        self.state = 'standby'
    def get_status(self, eventtime):
        return {'state': self.state}


class StubGcmd:
    error = CommandError
    def __init__(self):
        self.out = []
    def respond_info(self, msg):
        self.out.append(msg)


class HarnessFixture:
    """make() and tick() for the power-up tests. Not a TestCase, so a
    subclass with its own setup does not re-run the Harness tests."""
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
        zp._init_pgood_state(z_power.DROP_SAMPLES)
        zp.xy_requires_column = True
        zp.current = None
        zp.raw_current = None
        zp.current_offset = None
        zp.max_current, zp.max_current_time = 5.0, 0.5
        zp.idle_current, zp.idle_current_time = 1.0, 2.0
        zp.supply_voltage = 45.
        zp.over_since = zp.idle_over_since = None
        zp.start_peak = 0.
        zp.heater_name = 'heater_bed'
        zp.heater = StubHeater()
        zp.detector = StubDetector()
        # The settle has its own tests; elsewhere a plate counts at once.
        zp.plate_settle_time = 0.
        zp.column_since = None
        zp.plate = None
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
        zp.column_was_fitted = False
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


class Harness(HarnessFixture, unittest.TestCase):
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

    def test_pgood_timeout_counts_from_the_pin_change(self):
        # The enable pin changes PIN_MIN_TIME after the poll that sets it,
        # so the eFuse gets the whole timeout.
        zp = self.make()
        self.tick(zp, 1.0)
        self.assertAlmostEqual(zp.mcu_enable.writes[-1][0],
                               1.0 + z_power.PIN_MIN_TIME)
        self.tick(zp, 1.0 + z_power.PIN_MIN_TIME + 0.45)
        self.assertEqual(zp.state, POWERING)
        self.tick(zp, 1.0 + z_power.PIN_MIN_TIME + 0.55)
        self.assertEqual(zp.state, FAULT)

    def test_timeout_default_outlasts_the_efuse_start_up(self):
        # TPS16630 t(Treg_timeout) is 1.5 s max, PGOOD deglitch 13 ms max.
        self.assertGreater(z_power.POWER_GOOD_TIMEOUT, 1.5 + 0.013)

    def test_timeout_fault_names_the_start_up_current(self):
        zp = self.make()
        zp.raw_current, zp.current_offset = 0.1, 0.1
        self.tick(zp, 1.0)
        zp.raw_current = 6.1
        self.tick(zp, 1.2)
        zp.raw_current = 0.1
        self.tick(zp, 1.7)
        self.assertEqual(zp.state, FAULT)
        self.assertIn('6.00 A peak', zp.fault)

    def test_timeout_fault_without_imon(self):
        zp = self.make()
        self.tick(zp, 1.0)
        self.tick(zp, 1.7)
        self.assertIn('? A peak', zp.fault)

    def test_pgood_drop_after_ready_latches_fault(self):
        # One low poll latches with drop_samples = 1, the behaviour before
        # KAN-3334, once the start-up settle (0.5 s here) is over.
        zp = self.make()
        zp.drop_samples = 1
        self.tick(zp, 1.0)
        zp.power_good = True
        self.tick(zp, 1.1)
        zp.heater.set_temp(60.)
        zp.power_good = False
        self.tick(zp, 1.7)
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


class FakeHoming:
    """What PrinterHoming.abort_active_move records."""
    def __init__(self, raises=None):
        self.calls = []
        self.raises = raises
    def abort_active_move(self, reason, z_only=True):
        self.calls.append(reason)
        if self.raises is not None:
            raise self.raises
        return True


class HomingPrinter(StubPrinter):
    def __init__(self, homing):
        self.homing = homing
    def lookup_object(self, name, default=None):
        return self.homing if name == 'homing' else default


class TestAbortsZMove(Harness):
    """KAN-3370: losing the column or the 48 V ends a running Z move."""
    UNLOCKED_MSG = "the Z column was unlocked"
    REMOVED_MSG = "the Z column was removed"
    POWER_MSG = "the Z and bed power was lost"

    def ready(self, homing=None):
        homing = homing or FakeHoming()
        zp = self.make()
        zp.printer = HomingPrinter(homing)
        self.tick(zp, 1.0)
        zp.power_good = True
        self.tick(zp, 1.1)
        self.assertEqual(zp.state, READY)
        self.assertEqual(homing.calls, [])
        return zp, homing

    def test_unlocking_aborts_then_the_power_drop(self):
        zp, homing = self.ready()
        self.tick(zp, 1.2, ratio=0.5)
        self.assertEqual(homing.calls, [self.UNLOCKED_MSG, self.POWER_MSG])

    def test_pulling_the_column_aborts(self):
        zp, homing = self.ready()
        self.tick(zp, 1.2, ratio=0.99)
        self.assertEqual(homing.calls, [self.REMOVED_MSG, self.POWER_MSG])

    def test_pgood_drop_aborts(self):
        # KAN-3334: past the start-up settle (0.5 s here), and latching on
        # the first low poll (drop_samples = 1). The abort is on the first
        # low poll and again as the path leaves READY; this fake records
        # every call, the real abort_active_move returns False the second
        # time (see TestFirstSampleAbort).
        zp, homing = self.ready()
        zp.drop_samples = 1
        zp.power_good = False
        self.tick(zp, 1.7)
        self.assertEqual(zp.state, FAULT)
        self.assertEqual(homing.calls, [self.POWER_MSG, self.POWER_MSG])

    def test_inhibit_aborts(self):
        zp, homing = self.ready()
        zp.inhibited = True
        self.tick(zp, 1.2)
        self.assertEqual(homing.calls, [self.POWER_MSG])

    def test_plate_removal_aborts_on_the_power_drop(self):
        # bed_removal_detector aborts first on a real printer; this is the
        # second line of defence when only the power path noticed
        zp, homing = self.ready()
        self.tick(zp, 1.2, plate=False)
        self.assertEqual(homing.calls, [self.POWER_MSG])

    def test_nothing_aborts_without_a_loss(self):
        zp, homing = self.ready()
        for t in (1.2, 1.3, 1.4):
            self.tick(zp, t)
        self.assertEqual(homing.calls, [])
        # a first reading and plugging a column in are not losses
        homing2 = FakeHoming()
        zp = self.make()
        zp.printer = HomingPrinter(homing2)
        self.tick(zp, 1.0, ratio=0.99)            # absent, the first reading
        self.tick(zp, 1.1, ratio=0.1)             # plugged in and locked
        self.assertEqual(homing2.calls, [])

    def test_a_failing_abort_does_not_break_the_poll(self):
        zp, homing = self.ready(FakeHoming(raises=RuntimeError("boom")))
        self.tick(zp, 1.2, ratio=0.5)
        self.assertEqual(zp.state, UNLOCKED)
        self.assertFalse(zp.enabled)
        self.assertEqual(zp.heater.target, 0.)
        self.assertIn('ZCHANGE', zp.scripts)

    def test_no_homing_object_is_fine(self):
        zp = self.make()
        self.tick(zp, 1.0)
        zp.power_good = True
        self.tick(zp, 1.1)
        self.tick(zp, 1.2, ratio=0.5)
        self.assertEqual(zp.state, UNLOCKED)


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


class Ready(Harness):
    """A path brought up and READY at t=1.1; the start-up settle (the
    harness's power_good_timeout, 0.5 s) ends at 1.6."""
    def up(self, zp):
        self.tick(zp, 1.0)
        zp.power_good = True
        self.tick(zp, 1.1)
        self.assertEqual(zp.state, READY)

    def poll_pgood(self, zp, levels, t0=2.0, step=0.1):
        """One poll per level (True/False) from t0; returns the next time."""
        t = t0
        for level in levels:
            zp.power_good = level
            self.tick(zp, t)
            t += step
        return t

    def enable_writes(self, zp):
        return [v for _t, v in zp.mcu_enable.writes]


class TestPgoodDebounce(Ready):
    def test_defaults(self):
        self.assertEqual(z_power.DROP_SAMPLES, 3)
        self.assertEqual(self.make().drop_samples, 3)

    def test_single_low_does_not_latch_and_is_counted_and_logged(self):
        zp = self.make()
        self.up(zp)
        with self.assertLogs(level='WARNING') as logs:
            self.poll_pgood(zp, [False, True])
        self.assertEqual(zp.state, READY)
        self.assertIsNone(zp.fault)
        self.assertTrue(zp.enabled)
        self.assertEqual(self.enable_writes(zp), [1])   # never switched off
        self.assertEqual(zp.glitches, 1)
        self.assertAlmostEqual(zp.last_glitch, 2.1)
        self.assertEqual(len(logs.output), 1)
        self.assertIn('PGOOD glitch #1', logs.output[0])
        st = zp.get_status(2.6)
        self.assertEqual(st['pgood_glitches'], 1)
        self.assertEqual(st['last_pgood_glitch_age'], 0.5)
        self.assertEqual(st['state'], READY)

    def test_glitch_line_carries_the_diagnostics(self):
        zp = self.make()
        zp.heater.last_temp = 41.5
        self.up(zp)
        zp.raw_current, zp.current_offset = 2.5, 0.
        with self.assertLogs(level='WARNING') as logs:
            self.poll_pgood(zp, [True, False, True])
        line = logs.output[0]
        for text in ('peak 2.50 A', 'now 2.50 A', 'Z_PRESENT 0.100',
                     'bed 41.5 C', '0@0.1s', '1@0.0s'):
            self.assertIn(text, line)

    def test_n_consecutive_lows_latch_with_diagnostics(self):
        zp = self.make()
        self.up(zp)
        zp.heater.set_temp(60.)
        zp.raw_current, zp.current_offset = 3.0, 0.
        self.poll_pgood(zp, [True, True], t0=2.0)
        zp.raw_current = 0.
        with self.assertLogs(level='ERROR') as logs:
            self.poll_pgood(zp, [False, False], t0=2.2)
            self.assertEqual(zp.state, READY)     # two lows: not yet
            self.assertIsNone(zp.fault)
            self.tick(zp, 2.4)
        self.assertEqual(zp.state, FAULT)
        self.assertFalse(zp.enabled)
        self.assertEqual(zp.heater.target, 0.)
        self.assertIn('FAULT', zp.scripts)
        # The old first words are kept; the peak is higher than "now" (0).
        self.assertTrue(zp.fault.startswith('PGOOD dropped (eFuse tripped, '))
        self.assertIn('peak 3.00 A', zp.fault)
        self.assertNotIn('0.00 A)', zp.fault)
        self.assertEqual(len(logs.output), 1)
        self.assertIn('now 0.00 A, peak 3.00 A over 2 s', logs.output[0])
        self.assertIn('0@0.0s', logs.output[0])
        self.assertEqual(zp.glitches, 0)

    def test_n_minus_one_lows_then_a_high_does_not_latch(self):
        zp = self.make()
        self.up(zp)
        self.poll_pgood(zp, [False, False, True])
        self.assertEqual(zp.state, READY)
        self.assertIsNone(zp.fault)
        self.assertEqual(zp.glitches, 1)

    def test_lows_separated_by_highs_never_accumulate(self):
        zp = self.make()
        self.up(zp)
        self.poll_pgood(zp, [False, True] * 10)
        self.assertEqual(zp.state, READY)
        self.assertIsNone(zp.fault)
        self.assertEqual(zp.glitches, 10)
        self.assertEqual(self.enable_writes(zp), [1])

    def test_drop_samples_one_is_the_old_behaviour(self):
        zp = self.make()
        zp.drop_samples = 1
        self.up(zp)
        self.poll_pgood(zp, [False])
        self.assertEqual(zp.state, FAULT)
        self.assertEqual(zp.glitches, 0)

    def test_glitch_log_is_rate_limited_but_counted(self):
        zp = self.make()
        self.up(zp)
        with self.assertLogs(level='WARNING') as logs:
            self.poll_pgood(zp, [False, True] * 20)    # 4 s of chatter
        self.assertEqual(zp.glitches, 20)
        self.assertEqual(len(logs.output), 1)
        # After the interval the next glitch is logged, saying how many
        # were not.
        with self.assertLogs(level='WARNING') as logs:
            self.poll_pgood(zp, [False, True],
                            t0=2.0 + z_power.GLITCH_LOG_INTERVAL)
        self.assertEqual(len(logs.output), 1)
        self.assertIn('19 more not logged', logs.output[0])

    def test_config_option_defaults_and_minimum(self):
        class Cfg:
            def getint(self, name, default, minval=None):
                self.asked = (name, default, minval)
                return default
        cfg = Cfg()
        zp = self.make()
        zp._init_pgood_state(cfg.getint('power_good_drop_samples',
                                        z_power.DROP_SAMPLES, minval=1))
        self.assertEqual(cfg.asked, ('power_good_drop_samples', 3, 1))
        src = open(os.path.join(KLIPPY, 'extras', 'z_power.py')).read()
        self.assertIn("config.getint('power_good_drop_samples',", src)
        self.assertIn('minval=1', src.split('power_good_drop_samples')[1]
                      .split(')')[0])


class TestStartUpBlanking(Ready):
    def test_a_low_inside_the_settle_is_not_dropped(self):
        zp = self.make()
        zp.drop_samples = 1          # the strictest setting still holds
        self.up(zp)
        zp.heater.set_temp(60.)
        self.poll_pgood(zp, [False], t0=1.2)           # 0.1 s after READY
        self.assertIsNone(zp.fault)
        self.assertEqual(zp.state, POWERING)
        self.assertTrue(zp.enabled)
        self.assertEqual(zp.heater.target, 0.)         # heating refused
        self.assertEqual(zp.glitches, 0)

    def test_a_low_inside_the_settle_that_recovers_goes_ready(self):
        zp = self.make()
        self.up(zp)
        self.poll_pgood(zp, [False, True], t0=1.2)
        self.assertEqual(zp.state, READY)
        self.assertIsNone(zp.fault)

    def test_a_low_that_stays_latches_did_not_rise_not_dropped(self):
        zp = self.make()
        self.up(zp)
        self.poll_pgood(zp, [False], t0=1.2)
        # The start-up wait runs from the low, as from an enable
        # (power_good_timeout is 0.5 s here).
        self.tick(zp, 1.2 + 0.45)
        self.assertEqual(zp.state, POWERING)
        self.tick(zp, 1.2 + 0.55)
        self.assertEqual(zp.state, FAULT)
        self.assertIn('PGOOD did not rise within 0.50s', zp.fault)
        self.assertNotIn('dropped', zp.fault)

    def test_after_the_settle_a_low_is_judged(self):
        zp = self.make()
        zp.drop_samples = 1
        self.up(zp)
        self.poll_pgood(zp, [False], t0=1.7)           # settle ended 1.6
        self.assertEqual(zp.state, FAULT)
        self.assertIn('PGOOD dropped', zp.fault)

    def test_settle_is_not_restarted_by_a_recovered_low(self):
        zp = self.make()
        zp.drop_samples = 1
        self.up(zp)
        self.poll_pgood(zp, [False, True], t0=1.5)     # inside, recovers
        self.assertEqual(zp.state, READY)
        self.poll_pgood(zp, [False], t0=1.7)           # past the original end
        self.assertEqual(zp.state, FAULT)

    def test_power_good_timeout_still_times_the_first_rise(self):
        zp = self.make()
        self.tick(zp, 1.0)
        self.tick(zp, 1.0 + z_power.PIN_MIN_TIME + 0.45)
        self.assertEqual(zp.state, POWERING)
        self.tick(zp, 1.0 + z_power.PIN_MIN_TIME + 0.55)
        self.assertEqual(zp.state, FAULT)
        self.assertIn('PGOOD did not rise within', zp.fault)
        self.assertNotIn('after the reset', zp.fault)

    def test_re_enable_after_the_plate_flickers_waits_for_pgood(self):
        # Stale-READY check: PGOOD falls when the FET goes off, so a quick
        # off/on does not read READY from the old high.
        zp = self.make()
        self.up(zp)
        self.tick(zp, 1.2, plate=False)
        zp.power_good = False                # the FET went off
        self.tick(zp, 1.3)                   # plate back: enable again
        self.tick(zp, 1.4)
        self.assertEqual(zp.state, POWERING)
        self.assertEqual(self.enable_writes(zp), [1, 0, 1])


class TestFaultsAreExplicit(Ready):
    def latch(self, zp):
        self.up(zp)
        zp.drop_samples = 1
        self.poll_pgood(zp, [False], t0=2.0)
        self.assertEqual(zp.state, FAULT)

    def test_no_automatic_re_enable_after_a_latch(self):
        zp = self.make()
        self.latch(zp)
        writes = list(zp.mcu_enable.writes)
        self.poll_pgood(zp, [True] * 30, t0=2.1)   # PGOOD back, all good
        self.assertEqual(zp.state, FAULT)
        self.assertFalse(zp.enabled)
        self.assertEqual(zp.mcu_enable.writes, writes)
        self.assertEqual(self.enable_writes(zp)[-1], 0)

    def test_reset_clears_and_the_feed_comes_back(self):
        zp = self.make()
        self.latch(zp)
        zp.printer.objects['print_stats'] = StubPrintStats()
        gcmd = StubGcmd()
        zp.cmd_Z_POWER_RESET(gcmd)
        self.assertIsNone(zp.fault)
        self.assertIsNone(zp.fault_detail)
        self.assertIn('Cleared Z power fault', gcmd.out[0])
        zp.power_good = False
        self.tick(zp, 3.0)
        self.assertEqual(zp.state, POWERING)
        self.assertEqual(self.enable_writes(zp)[-1], 1)

    def test_reset_is_refused_while_printing(self):
        zp = self.make()
        self.latch(zp)
        stats = StubPrintStats()
        zp.printer.objects['print_stats'] = stats
        stats.state = 'printing'
        writes = list(zp.mcu_enable.writes)
        with self.assertRaises(CommandError) as ctx:
            zp.cmd_Z_POWER_RESET(StubGcmd())
        self.assertEqual(str(ctx.exception), z_power.PRINTING_RESET_ERROR)
        self.assertTrue(zp.fault)
        self.tick(zp, 3.0)
        self.assertEqual(zp.state, FAULT)
        self.assertFalse(zp.enabled)
        self.assertEqual(zp.mcu_enable.writes, writes)

    def test_reset_is_allowed_when_not_printing(self):
        for state in ('paused', 'standby', 'complete', 'cancelled', 'error'):
            zp = self.make()
            self.latch(zp)
            stats = StubPrintStats()
            stats.state = state
            zp.printer.objects['print_stats'] = stats
            zp.cmd_Z_POWER_RESET(StubGcmd())
            self.assertIsNone(zp.fault, state)

    def test_reset_without_print_stats_counts_as_not_printing(self):
        zp = self.make()
        self.latch(zp)
        zp.cmd_Z_POWER_RESET(StubGcmd())
        self.assertIsNone(zp.fault)

    def reset_and_enable(self, zp):
        self.latch(zp)
        zp.cmd_Z_POWER_RESET(StubGcmd())
        zp.power_good = False
        self.tick(zp, 3.0)                       # re-enabled
        self.assertEqual(zp.state, POWERING)

    def test_pgood_that_does_not_return_after_a_reset_says_so_once(self):
        zp = self.make()
        self.reset_and_enable(zp)
        with self.assertLogs(level='ERROR') as logs:
            self.tick(zp, 3.0 + z_power.PIN_MIN_TIME + 0.55)
            self.assertEqual(zp.state, FAULT)
            self.assertEqual(zp.reason, z_power.NO_RETURN_REASON)
            self.assertIn('PGOOD did not rise after the reset',
                          zp.fault)
            self.assertIsNotNone(zp.fault_detail)
            writes = list(zp.mcu_enable.writes)
            for t in (4.0, 4.1, 4.2, 6.0, 9.0):
                self.tick(zp, t)
        self.assertEqual(len(logs.output), 1)
        self.assertEqual(zp.state, FAULT)
        self.assertEqual(zp.reason, z_power.NO_RETURN_REASON)
        self.assertEqual(zp.mcu_enable.writes, writes)   # no second try
        self.assertEqual(zp.scripts.count('FAULT'), 2)   # the first + this

    def test_the_no_return_text_is_not_the_other_two(self):
        self.assertNotEqual(z_power.NO_RETURN_REASON, z_power.DROPPED_REASON)
        self.assertIn('did not come back', z_power.NO_RETURN_REASON)
        self.assertNotIn('did not come back', z_power.DROPPED_REASON)

    def test_pgood_that_returns_after_a_reset_goes_ready_clean(self):
        zp = self.make()
        zp.glitches = 4
        self.latch(zp)
        zp.low_polls = 2
        zp.cmd_Z_POWER_RESET(StubGcmd())
        self.assertEqual(zp.low_polls, 0)
        zp.power_good = False
        self.tick(zp, 3.0)
        zp.power_good = True
        self.tick(zp, 3.2)
        self.assertEqual(zp.state, READY)
        self.assertIsNone(zp.fault)
        self.assertEqual(zp.low_polls, 0)
        self.assertEqual(zp.glitches, 4)          # the evidence is kept
        self.poll_pgood(zp, [True], t0=4.0)       # past the settle
        self.assertFalse(zp.after_reset)
        # A later drop is "dropped", not "did not come back"
        zp.drop_samples = 1
        self.poll_pgood(zp, [False], t0=4.1)
        self.assertEqual(zp.reason, z_power.DROPPED_REASON)

    def test_dropped_reason_is_one_plain_sentence(self):
        zp = self.make()
        self.latch(zp)
        self.tick(zp, 2.1)          # the next poll's decide() keeps it
        self.assertEqual(zp.reason, z_power.DROPPED_REASON)
        self.assertEqual(zp.get_status(3.)['reason'], z_power.DROPPED_REASON)
        self.assertNotIn(' A', zp.reason)
        self.assertNotIn('PGOOD', zp.reason)
        self.assertEqual(zp.reason.count('. '), 1)
        self.assertTrue(zp.reason.endswith('.'))
        self.assertIn('PGOOD dropped', zp.fault)
        self.assertIn('peak', zp.get_status(3.)['fault_detail'])

    def test_other_faults_keep_their_old_reason(self):
        zp = self.make()
        self.up(zp)
        zp.latch_fault('boom')
        self.tick(zp, 2.0)
        self.assertIn('Z_POWER_RESET', zp.reason)
        self.assertIn('boom', zp.reason)


class TestFirstSampleAbort(Ready):
    """The agreed design (KAN-3370 with KAN-3334): an active Z move is
    aborted on the FIRST low PGOOD poll; only the latch is debounced."""
    MSG = "the Z and bed power was lost"

    def ready(self, homing=None):
        homing = homing or FakeHoming()
        zp = self.make()
        zp.printer = HomingPrinter(homing)
        self.up(zp)
        return zp, homing

    def test_first_low_aborts_once_without_a_latch(self):
        zp, homing = self.ready()
        with self.assertLogs(level='WARNING') as logs:
            self.poll_pgood(zp, [False], t0=2.0)
            self.assertEqual(homing.calls, [self.MSG])
            self.assertEqual(zp.state, READY)
            self.assertIsNone(zp.fault)
            self.assertTrue(zp.enabled)
            self.poll_pgood(zp, [True], t0=2.1)
        self.assertEqual(homing.calls, [self.MSG])
        self.assertEqual(zp.glitches, 1)          # counted
        self.assertEqual(len(logs.output), 1)     # and logged
        self.assertIn('PGOOD glitch #1', logs.output[0])
        self.assertEqual(self.enable_writes(zp), [1])

    def test_three_lows_abort_on_the_first_and_latch_on_the_third(self):
        zp, homing = self.ready()
        self.poll_pgood(zp, [False], t0=2.0)
        self.assertEqual(homing.calls, [self.MSG])
        self.poll_pgood(zp, [False], t0=2.1)
        self.assertEqual(homing.calls, [self.MSG])    # not once per poll
        self.assertIsNone(zp.fault)
        self.poll_pgood(zp, [False], t0=2.2)
        self.assertEqual(zp.state, FAULT)
        # The latch leaves READY and asks again (FakeHoming records every
        # call; the real abort_active_move returns False for a move
        # already aborted, homing.py HomingMove.abort), so two here, and
        # never one per low poll.
        self.assertEqual(homing.calls, [self.MSG, self.MSG])

    def test_a_low_in_the_start_up_settle_aborts_on_the_first_poll(self):
        zp, homing = self.ready()
        self.poll_pgood(zp, [False], t0=1.2)
        # First poll: the low sample, and the READY exit to POWERING
        self.assertEqual(homing.calls, [self.MSG, self.MSG])
        self.assertEqual(zp.state, POWERING)
        self.assertIsNone(zp.fault)
        self.poll_pgood(zp, [False], t0=1.3)
        self.assertEqual(homing.calls, [self.MSG, self.MSG])  # no more

    def test_each_new_low_run_aborts_again(self):
        zp, homing = self.ready()
        self.poll_pgood(zp, [False, True, False, True], t0=2.0)
        self.assertEqual(homing.calls, [self.MSG, self.MSG])
        self.assertEqual(zp.glitches, 2)

    def test_no_z_move_active_changes_nothing(self):
        # abort_active_move returns False with no move to stop
        class Idle(FakeHoming):
            def abort_active_move(self, reason, z_only=True):
                self.calls.append(reason)
                return False
        zp, homing = self.ready(Idle())
        self.poll_pgood(zp, [False, True], t0=2.0)
        self.assertEqual(zp.state, READY)
        self.assertIsNone(zp.fault)
        self.assertEqual(zp.glitches, 1)
        self.poll_pgood(zp, [False, False, False], t0=3.0)
        self.assertEqual(zp.state, FAULT)
        self.assertEqual(self.enable_writes(zp), [1, 0])

    def test_a_failing_abort_does_not_stop_the_debounce(self):
        zp, homing = self.ready(FakeHoming(raises=RuntimeError("boom")))
        self.poll_pgood(zp, [False, True], t0=2.0)
        self.assertEqual(zp.state, READY)
        self.assertEqual(zp.glitches, 1)


class TestColumnSettle(HarnessFixture, unittest.TestCase):
    """The bed thermistor runs through the Z column: with the column out the
    input floats and reads as a plate (sloe, 2026-11-06)."""
    def make(self):
        zp = Harness.make(self)
        zp.plate_settle_time = 2.0
        return zp

    def test_floating_plate_reading_does_not_power_a_new_column(self):
        zp = self.make()
        self.tick(zp, 1.0, ratio=0.99)          # column out, reads a plate
        self.assertFalse(zp.enabled)
        self.assertIsNone(zp.plate)
        self.tick(zp, 1.1)                      # column in and locked
        self.assertEqual(zp.state, NO_PLATE)
        self.assertFalse(zp.enabled)
        self.tick(zp, 2.9)
        self.assertFalse(zp.enabled)
        # The detector has caught up: no plate.
        self.tick(zp, 3.2, plate=False)
        self.assertEqual(zp.state, NO_PLATE)
        self.assertFalse(zp.enabled)
        self.assertEqual(zp.mcu_enable.writes, [])

    def test_plate_on_a_new_column_powers_after_the_settle(self):
        zp = self.make()
        self.tick(zp, 1.0, ratio=0.99)
        self.tick(zp, 1.1)
        self.tick(zp, 3.0)
        self.assertFalse(zp.enabled)
        self.tick(zp, 3.2)
        self.assertEqual(zp.state, POWERING)
        self.assertTrue(zp.enabled)

    def test_unlocked_column_counts_towards_the_settle(self):
        # The thermistor is wired once the column is in, lever up or down.
        zp = self.make()
        self.tick(zp, 1.0, ratio=0.5)           # in, unlocked
        self.tick(zp, 3.5)                      # locked
        self.assertEqual(zp.state, POWERING)

    def test_pulling_the_column_restarts_the_settle(self):
        zp = self.make()
        self.tick(zp, 1.0)
        self.tick(zp, 3.1)
        zp.power_good = True
        self.tick(zp, 3.2)
        self.assertEqual(zp.state, READY)
        self.tick(zp, 3.3, ratio=0.99)          # pulled
        zp.power_good = False
        self.assertFalse(zp.enabled)
        self.tick(zp, 3.4)                      # back in at once
        self.assertEqual(zp.state, NO_PLATE)
        self.assertFalse(zp.enabled)
        self.tick(zp, 5.5)
        self.assertEqual(zp.state, POWERING)


class TestB1PlateThresholds(unittest.TestCase):
    """[bed_removal_detector] thresholds in the B1 config against what the
    bed thermistor reads on sloe, 2026-11-06."""
    NO_PLATE = (-17.2, -13.3)    # column in, no plate (raw samples)

    def thresholds(self):
        path = os.path.join(KLIPPY, '..', 'core', 'M1', 'core.cfg')
        vals, section = {}, None
        for line in open(path):
            line = line.split('#')[0].strip()
            if line.startswith('['):
                section = line
            elif section == '[bed_removal_detector]' and ':' in line:
                key, val = line.split(':', 1)
                vals[key.strip()] = float(val)
        return vals['remove_threshold'], vals['attach_threshold']

    def test_no_plate_reads_removed_with_margin(self):
        remove, attach = self.thresholds()
        self.assertLess(self.NO_PLATE[1] + 10., remove)
        self.assertLess(remove, attach)

    def test_a_plate_in_a_cool_room_reads_attached(self):
        remove, attach = self.thresholds()
        self.assertLess(attach, 10.)


class TestUnknownColumnAbort(HarnessFixture, unittest.TestCase):
    """A reading in no band that outlasts the present_samples debounce
    aborts an active Z move, once per run of unknown readings."""
    MSG = "the Z column position is unknown"
    POWER_MSG = "the Z and bed power was lost"
    UNKNOWN = 0.3          # in no band of BANDS

    def make(self):
        zp = HarnessFixture.make(self)
        zp.z_band = z_power.Debounce(3)
        return zp

    def column(self, zp, ratio, polls, t):
        for _ in range(polls):
            t += 0.1
            self.tick(zp, t, ratio=ratio)
        return t

    def locked(self, homing=None):
        # Locked and powering, never READY, so only the column's own
        # aborts can be seen (READY adds the power-drop abort).
        homing = homing or FakeHoming()
        zp = self.make()
        zp.printer = HomingPrinter(homing)
        t = self.column(zp, 0.1, 3, 1.0)
        self.assertEqual(zp.z_band.value, 'locked')
        self.assertEqual(zp.state, POWERING)
        self.assertEqual(homing.calls, [])
        return zp, homing, t

    def test_unknown_for_the_debounce_aborts_once(self):
        zp, homing, t = self.locked()
        self.column(zp, self.UNKNOWN, 3, t)
        self.assertIsNone(zp.z_band.value)
        self.assertEqual(homing.calls, [self.MSG])

    def test_not_before_the_debounce_count(self):
        zp, homing, t = self.locked()
        self.column(zp, self.UNKNOWN, 2, t)
        self.assertEqual(homing.calls, [])
        self.column(zp, self.UNKNOWN, 1, t + 0.2)
        self.assertEqual(homing.calls, [self.MSG])

    def test_a_flicker_shorter_than_the_debounce_is_ignored(self):
        zp, homing, t = self.locked()
        t = self.column(zp, self.UNKNOWN, 2, t)
        self.column(zp, 0.1, 3, t)
        self.assertEqual(zp.z_band.value, 'locked')
        self.assertEqual(homing.calls, [])

    def test_a_long_unknown_run_aborts_once(self):
        zp, homing, t = self.locked()
        self.column(zp, self.UNKNOWN, 40, t)
        self.assertEqual(homing.calls, [self.MSG])

    def test_a_new_run_aborts_again(self):
        zp, homing, t = self.locked()
        t = self.column(zp, self.UNKNOWN, 3, t)
        t = self.column(zp, 0.1, 3, t)
        self.assertEqual(homing.calls, [self.MSG])
        self.column(zp, self.UNKNOWN, 3, t)
        self.assertEqual(homing.calls, [self.MSG, self.MSG])

    def test_ready_adds_the_power_drop_abort_after_it(self):
        homing = FakeHoming()
        zp = self.make()
        zp.printer = HomingPrinter(homing)
        t = self.column(zp, 0.1, 3, 1.0)
        zp.power_good = True
        t = self.column(zp, 0.1, 1, t)
        self.assertEqual(zp.state, READY)
        self.column(zp, self.UNKNOWN, 3, t)
        self.assertEqual(zp.state, UNKNOWN)
        self.assertEqual(homing.calls, [self.MSG, self.POWER_MSG])

    def test_startup_unknown_is_not_a_change(self):
        homing = FakeHoming()
        zp = self.make()
        zp.printer = HomingPrinter(homing)
        self.column(zp, self.UNKNOWN, 5, 1.0)
        self.assertEqual(homing.calls, [])

    def test_the_other_handling_is_unchanged(self):
        zp, homing, t = self.locked()
        self.column(zp, self.UNKNOWN, 3, t)
        self.assertEqual(zp.state, UNKNOWN)
        self.assertFalse(zp.enabled)
        self.assertEqual(zp.scripts.count('ZCHANGE'), 1)
        self.assertIsNone(zp.fault)

    def test_a_failing_abort_does_not_break_the_poll(self):
        zp, homing, t = self.locked(FakeHoming(raises=RuntimeError("boom")))
        self.column(zp, self.UNKNOWN, 3, t)
        self.assertEqual(homing.calls, [self.MSG])
        self.assertEqual(zp.state, UNKNOWN)
        self.assertEqual(zp.scripts.count('ZCHANGE'), 1)

    def test_no_homing_object_is_fine(self):
        zp = self.make()
        t = self.column(zp, 0.1, 3, 1.0)
        self.column(zp, self.UNKNOWN, 3, t)
        self.assertEqual(zp.state, UNKNOWN)


class StubTransform:
    def __init__(self):
        self.moves = []
    def get_position(self):
        return [0., 0., 0., 0.]
    def move(self, newpos, speed):
        self.moves.append(list(newpos))


class TestXyGateAfterAFittedColumn(HarnessFixture, unittest.TestCase):
    """XY is allowed in a reading in no band only after a fitted one with
    no absent reading since. Z_PRESENT ratios: 0.1 locked, 0.5 unlocked,
    0.99 absent, 0.3 in no band."""
    def make(self):
        zp = HarnessFixture.make(self)
        zp.next_transform = StubTransform()
        zp.last_position = [0., 0., 0., 0.]
        return zp

    def run_readings(self, zp, ratios):
        for i, ratio in enumerate(ratios):
            self.tick(zp, 1.0 + 0.1 * i, ratio=ratio)

    def xy(self, zp):
        zp.last_position = [0., 0., 0., 0.]
        zp.move([10., 10., 0., 0.], 100.)
        return zp.next_transform.moves[-1]

    def refused(self, zp):
        n = len(zp.next_transform.moves)
        with self.assertRaises(CommandError) as ctx:
            zp.last_position = [0., 0., 0., 0.]
            zp.move([10., 10., 0., 0.], 100.)
        self.assertEqual(len(zp.next_transform.moves), n)
        return str(ctx.exception)

    def test_locked_then_unknown_allows_xy(self):
        zp = self.make()
        self.run_readings(zp, [0.1, 0.3])
        self.assertIsNone(zp.z_band.value)
        self.assertEqual(self.xy(zp), [10., 10., 0., 0.])

    def test_unlocked_then_unknown_allows_xy(self):
        zp = self.make()
        self.run_readings(zp, [0.5, 0.3])
        self.assertEqual(self.xy(zp), [10., 10., 0., 0.])

    def test_absent_then_unknown_refuses(self):
        zp = self.make()
        self.run_readings(zp, [0.99, 0.3])
        self.assertIn("Cannot move X or Y", self.refused(zp))

    def test_startup_unknown_refuses_with_the_same_text(self):
        zp = self.make()
        self.run_readings(zp, [0.3, 0.3, 0.3])
        self.assertEqual(self.refused(zp), "Cannot move X or Y: %s"
                         % (z_power.REASONS[UNKNOWN],))

    def test_unread_refuses(self):
        zp = self.make()
        self.assertIn("Cannot move X or Y", self.refused(zp))

    def test_fitted_unknown_absent_refuses_again(self):
        zp = self.make()
        self.run_readings(zp, [0.1, 0.3])
        self.assertEqual(self.xy(zp), [10., 10., 0., 0.])
        self.run_readings(zp, [0.99])
        self.assertEqual(self.refused(zp), "Cannot move X or Y: %s"
                         % (z_power.REASONS[ABSENT],))

    def test_absent_stays_refused_through_unknown_until_refitted(self):
        zp = self.make()
        self.run_readings(zp, [0.1, 0.99, 0.3])
        self.assertIn("Cannot move X or Y", self.refused(zp))
        self.run_readings(zp, [0.5, 0.3])
        self.assertEqual(self.xy(zp), [10., 10., 0., 0.])

    def test_fitted_unknown_fitted_allows_xy(self):
        zp = self.make()
        self.run_readings(zp, [0.1, 0.3, 0.5])
        self.assertEqual(self.xy(zp), [10., 10., 0., 0.])
        self.run_readings(zp, [0.1])
        self.assertEqual(self.xy(zp), [10., 10., 0., 0.])

    def test_a_z_only_move_never_asked(self):
        zp = self.make()
        zp.last_position = [5., 5., 0., 0.]
        zp.move([5., 5., 10., 0.], 100.)      # no reading at all: Z is free
        self.assertEqual(zp.next_transform.moves[-1], [5., 5., 10., 0.])

    def test_a_print_whose_lever_passes_through_unknown_can_park(self):
        # Locked and READY while printing; the reading leaves every band
        # for the debounce (3 polls here) and the path drops. Today that is
        # state UNKNOWN with the heater off; the PAUSE's park (X/Y) goes
        # through instead of leaving the pause half done.
        zp = self.make()
        zp.z_band = z_power.Debounce(3)
        for t in (1.0, 1.1, 1.2):
            self.tick(zp, t, ratio=0.1)
        zp.power_good = True
        self.tick(zp, 1.3, ratio=0.1)
        self.assertEqual(zp.state, READY)
        for t in (1.4, 1.5, 1.6):
            self.tick(zp, t, ratio=0.3)
        self.assertEqual(zp.state, UNKNOWN)
        self.assertFalse(zp.enabled)
        self.assertIsNone(zp.z_band.value)
        self.assertEqual(self.xy(zp), [10., 10., 0., 0.])
        self.run_readings(zp, [0.99, 0.99, 0.99])
        self.assertIn("Cannot move X or Y", self.refused(zp))


class TestStatusAgreesWithTheGate(HarnessFixture, unittest.TestCase):
    """get_status()['xy_allowed'] is what the panel shows; it must say what
    move() does. xy_home_allowed stays strict for X/Y homing."""
    def make(self):
        zp = HarnessFixture.make(self)
        zp.next_transform = StubTransform()
        zp.last_position = [0., 0., 0., 0.]
        return zp

    def run_readings(self, zp, ratios):
        for i, ratio in enumerate(ratios):
            self.tick(zp, 1.0 + 0.1 * i, ratio=ratio)

    def gate_allows(self, zp):
        zp.last_position = [0., 0., 0., 0.]
        try:
            zp.move([10., 10., 0., 0.], 100.)
        except CommandError:
            return False
        return True

    def check(self, ratios, allowed, home_allowed):
        zp = self.make()
        self.run_readings(zp, ratios)
        st = zp.get_status(5.0)
        self.assertIs(self.gate_allows(zp), allowed)
        self.assertIs(st['xy_allowed'], allowed)
        self.assertIs(st['xy_home_allowed'], home_allowed)

    def test_locked(self):
        self.check([0.1], True, True)

    def test_unlocked(self):
        self.check([0.5], True, True)

    def test_unknown_after_fitted(self):
        self.check([0.1, 0.3], True, False)

    def test_unknown_after_unlocked(self):
        self.check([0.5, 0.3], True, False)

    def test_unknown_at_startup(self):
        self.check([0.3, 0.3, 0.3], False, False)

    def test_absent(self):
        self.check([0.99], False, False)

    def test_unknown_after_absent(self):
        self.check([0.1, 0.99, 0.3], False, False)

    def test_unread(self):
        self.check([], False, False)

    def test_not_requiring_a_column_allows_both(self):
        zp = self.make()
        zp.xy_requires_column = False
        st = zp.get_status(5.0)
        self.assertTrue(st['xy_allowed'])
        self.assertTrue(st['xy_home_allowed'])


if __name__ == '__main__':
    unittest.main()
