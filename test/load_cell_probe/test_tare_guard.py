"""load_cell_probe tare guard (KAN-3370, guard A), run without a printer.

On the M1 "Boxwood" (safety_model preloaded_max, reference_max_load_counts
-3010747) something rested on the nozzle at tare, so the tare was -3244819,
PAST the anchor.  The probe inferred its press direction from the tare, so
it flipped to +1 and waited for the load to fall; real contact pushed the
counts more negative and never triggered, and Z homing drove 164 mm.

This drives the real LoadCellProbeConfigHelper (built through its own
__init__ from a fake config), LoadCellProbingMove._pause_and_tare /
_mcu_tare / probing_test and LoadCellPrinterProbe.get_press_direction with
fakes, and asserts exact outcomes: the refusal text, the direction and
scale sign, the trigger type and the safety range ends.  The numbers are
written out literally and not imported from the module under test.

    python3 test/load_cell_probe/test_tare_guard.py

Needs numpy (for _pause_and_tare).  Needs no klippy runtime: the module is
exec'd with its klippy imports stubbed, as test/tap_path does.

Negative control (performed by hand for KAN-3370, not automated): with the
guard disabled (check_tare_headroom made a no-op and the direction
inferred again from `1. if ref_max >= tare_counts else -1.`), test_incident
fails: "AssertionError: incident tare was NOT refused (counts): None",
because the old code armed the probe with scale +1.0 and no limit in the
contact direction.  With the guard restored it passes.
"""
import os
import sys
import types

import numpy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'tap_path'))
import _paths  # noqa: E402

SENSOR_RANGE = (-8388608, 8388607)
SENSOR_MAX_USABLE = 8388606     # sensor_max - 1
SENSOR_MIN_USABLE = -8388607    # sensor_min + 1

# Boxwood
BOX_REF = -3010747
BOX_INCIDENT_TARE = -3244819
BOX_TARES = (-578000, -800000, -1750000)
# Sloe
SLOE_REF = -1585739
SLOE_TARES = (200000, 230000, 790000, -390000)

REFUSAL = ("The nozzle is already under load: check nothing is touching it")
INCIDENT_TEXT = (REFUSAL + " (tare -3244819, max-load reference -3010747, "
                 "headroom -7.8%, need 20.0%)")


def load_module(path):
    pkg = types.ModuleType('extras')
    pkg.__path__ = []
    sys.modules['extras'] = pkg
    for n in ('probe', 'trigger_analog', 'load_cell', 'hx71x', 'ads1220',
              'muon_ads1220'):
        m = types.ModuleType('extras.' + n)
        sys.modules['extras.' + n] = m
        setattr(pkg, n, m)
    sys.modules['mcu'] = types.ModuleType('mcu')
    mod = types.ModuleType('extras.load_cell_probe')
    mod.__package__ = 'extras'
    sys.modules['extras.load_cell_probe'] = mod
    exec(compile(open(path).read(), path, 'exec'), mod.__dict__)
    mod.np = numpy
    return mod


class CommandError(Exception):
    pass


class ConfigError(Exception):
    pass


class FakeConfig(object):
    def __init__(self, printer, values):
        self.printer = printer
        self.values = values
        self.error = ConfigError

    def get_printer(self):
        return self.printer

    def get_name(self):
        return 'load_cell_probe'

    def _get(self, name, default):
        return self.values.get(name, default)

    def _bounded(self, name, v, minval, maxval, above, below):
        if v is None:
            return v
        if minval is not None and v < minval:
            raise ConfigError("%s below minimum" % (name,))
        if maxval is not None and v > maxval:
            raise ConfigError("%s above maximum" % (name,))
        if above is not None and v <= above:
            raise ConfigError("%s must be above %s" % (name, above))
        if below is not None and v >= below:
            raise ConfigError("%s must be below %s" % (name, below))
        return v

    def getint(self, name, default=None, minval=None, maxval=None):
        v = self._get(name, default)
        return self._bounded(name, None if v is None else int(v),
                             minval, maxval, None, None)

    def getfloat(self, name, default=None, minval=None, maxval=None,
                 above=None, below=None):
        v = self._get(name, default)
        return self._bounded(name, None if v is None else float(v),
                             minval, maxval, above, below)

    def getchoice(self, name, choices, default=None):
        return choices[self._get(name, default)]


class FakeMcu(object):
    def estimated_print_time(self, t):
        return t


class FakeSensor(object):
    def get_range(self):
        return SENSOR_RANGE

    def get_mcu(self):
        return FakeMcu()

    def get_samples_per_second(self):
        return 660


class FakeCollector(object):
    def __init__(self, counts):
        self.counts = counts

    def start_collecting(self, min_time=None):
        self.min_time = min_time or 0.

    def collect_min(self, n=1):
        t = self.min_time
        return ([[t + i * 0.001, 0., self.counts] for i in range(max(n, 1))],
                None)


class FakeLoadCell(object):
    def __init__(self, counts_per_gram, tare_counts):
        self.sensor = FakeSensor()
        self.counts_per_gram = counts_per_gram
        self.tare_counts = tare_counts
        self.tared_to = None

    def get_sensor(self):
        return self.sensor

    def get_counts_per_gram(self):
        return self.counts_per_gram

    def get_reference_tare_counts(self):
        return None

    def get_collector(self):
        return FakeCollector(self.tare_counts)

    def tare(self, c):
        self.tared_to = c


class FakeReactor(object):
    def monotonic(self):
        return 1000.


class FakeToolhead(object):
    def get_last_move_time(self):
        return 5.0


class FakePrinter(object):
    command_error = CommandError

    def get_reactor(self):
        return FakeReactor()

    def lookup_object(self, name):
        return FakeToolhead()

    def send_event(self, event, *params):
        pass


class FakeSos(object):
    def __init__(self, log):
        self.log = log

    def set_offset_scale(self, offset=0, scale=1., scale_frac_bits=0,
                         auto_offset=False):
        self.log.append(('set_offset_scale', offset, scale, scale_frac_bits,
                         auto_offset))

    def update_offset(self, offset):
        self.log.append(('update_offset', offset))


class FakeTrigger(object):
    # records everything sent toward the mcu
    def __init__(self):
        self.log = []

    def get_sos_filter(self):
        return FakeSos(self.log)

    def set_raw_range(self, lo, hi):
        self.log.append(('set_raw_range', lo, hi))

    def set_trigger(self, t, v):
        self.log.append(('set_trigger', t, v))

    def home_start(self, *a):
        self.log.append(('home_start',))

    def home_wait(self, t):
        self.log.append(('home_wait',))
        return 1.


class FakeFilterHelper(object):
    def update_from_command(self, gcmd):
        pass


def make_helper(mod, ref, counts_per_gram=None, tare=0, **opts):
    values = {'safety_model': 'preloaded_max',
              'reference_max_load_counts': ref}
    values.update(opts)
    printer = FakePrinter()
    cfg = FakeConfig(printer, values)
    return mod.LoadCellProbeConfigHelper(cfg, FakeLoadCell(counts_per_gram,
                                                          tare))


def make_move(mod, helper, tare):
    m = object.__new__(mod.LoadCellProbingMove)
    m._printer = helper._printer
    m._config_helper = helper
    m._load_cell = helper._load_cell
    m._mcu_trigger_analog = FakeTrigger()
    m._continuous_tare_filter_helper = FakeFilterHelper()
    m._tare_counts = 0
    m._approx_tare = tare
    m._safety_min = 0
    m._safety_max = 0
    m._trigger_mode = 'grams'
    helper._load_cell.tare_counts = tare
    return m


def refused(fn, *a, **kw):
    try:
        fn(*a, **kw)
    except CommandError as e:
        return str(e)
    return None


def arm(helper, tare):
    # what _pause_and_tare does: range first, then the trigger
    lo, hi = helper.get_safety_range(tare)
    return lo, hi, helper.get_trigger_setup(tare, lo, hi)


MODES = (
    ('counts', {'trigger_counts': 20000}),
    ('percent', {'trigger_percent': 10}),
    ('grams', {'trigger_force': 75}),
)


def test_incident(mod):
    for name, opts in MODES:
        # grams needs a calibrated cell to get as far as arming
        h = make_helper(mod, BOX_REF, counts_per_gram=1000., **opts)
        msg = refused(arm, h, BOX_INCIDENT_TARE)
        assert msg == INCIDENT_TEXT, (
            'incident tare was NOT refused (%s): %r' % (name, msg))
        # uncalibrated cell: the guard itself needs no counts_per_gram
        h = make_helper(mod, BOX_REF, counts_per_gram=None, **opts)
        msg = refused(h.get_safety_range, BOX_INCIDENT_TARE)
        assert msg == INCIDENT_TEXT, (
            'incident tare was NOT refused uncalibrated (%s): %r'
            % (name, msg))
    print('ok  incident refused in counts, percent and grams (and with an '
          'uncalibrated cell)')


def test_incident_nothing_sent(mod):
    for name, opts in MODES:
        h = make_helper(mod, BOX_REF, counts_per_gram=1000.,
                        tare=BOX_INCIDENT_TARE, **opts)
        m = make_move(mod, h, BOX_INCIDENT_TARE)
        msg = refused(m._pause_and_tare, None)
        assert msg == INCIDENT_TEXT, 'not refused (%s): %r' % (name, msg)
        assert m._mcu_trigger_analog.log == [], (
            'mcu was sent %r (%s)' % (m._mcu_trigger_analog.log, name))
        # the MCUTARE path takes the same guard
        m = make_move(mod, h, BOX_INCIDENT_TARE)
        msg = refused(m._mcu_tare, None)
        assert msg == INCIDENT_TEXT, 'mcu_tare not refused (%s)' % (name,)
        assert m._load_cell.tared_to is None, (
            'mcu_tare stored the loaded zero in the load cell (%s)' % (name,))
        assert m._mcu_trigger_analog.log == [], (
            'mcu was sent %r by _mcu_tare (%s)'
            % (m._mcu_trigger_analog.log, name))
        # and probing_test, which arms then waits for a trigger
        m = make_move(mod, h, BOX_INCIDENT_TARE)
        msg = refused(m.probing_test, None, 1.)
        assert msg == INCIDENT_TEXT, 'probing_test not refused (%s)' % (name,)
        assert m._mcu_trigger_analog.log == [], m._mcu_trigger_analog.log
    print('ok  nothing sent to the mcu from _pause_and_tare, _mcu_tare or '
          'probing_test on the incident tare')


def test_good_tare_arms_through_pause_and_tare(mod):
    h = make_helper(mod, BOX_REF, counts_per_gram=1000., tare=-800000,
                    trigger_counts=20000)
    m = make_move(mod, h, -800000)
    m._pause_and_tare(None)
    log = m._mcu_trigger_analog.log
    assert log[0] == ('set_raw_range', -2900210, SENSOR_MAX_USABLE), log
    assert ('set_offset_scale', 800000, -1., 0, False) in log, log
    assert log[-1] == ('set_trigger', 'gt', 20000), log
    print('ok  a normal Boxwood tare arms: range, scale -1, trigger gt')


def test_headroom_boundaries(mod):
    for name, opts in MODES:
        h = make_helper(mod, BOX_REF, counts_per_gram=1000., **opts)
        # tare exactly at the anchor
        assert (refused(h.get_safety_range, BOX_REF) or '').startswith(
            REFUSAL), 'tare at anchor allowed (%s)' % (name,)
        # 5.0% headroom (150547 counts)
        assert (refused(h.get_safety_range, -2860200) or '').startswith(
            REFUSAL), '5%% allowed (%s)' % (name,)
        # 19.9% headroom (599138 counts)
        assert (refused(h.get_safety_range, -2411609) or '').startswith(
            REFUSAL), '19.9%% allowed (%s)' % (name,)
        # exactly the required headroom, int(0.2 * 3010747) = 602149
        assert (refused(h.get_safety_range, -2408598) or '').startswith(
            REFUSAL), 'headroom == required allowed (%s)' % (name,)
        # 20.1% headroom (605160 counts) and one count over the floor
        assert refused(h.get_safety_range, -2405587) is None, (
            '20.1%% refused (%s)' % (name,))
        assert refused(h.get_safety_range, -2408597) is None, (
            'floor + 1 refused (%s)' % (name,))
    # a margin bigger than the 20% floor wins
    h = make_helper(mod, BOX_REF, trigger_counts=20000,
                    max_load_safety_margin_counts=900000)
    msg = refused(h.get_safety_range, -2300000)    # headroom 710747
    assert msg is not None and 'need 29.9%' in msg, msg
    # the floor is configurable
    h = make_helper(mod, BOX_REF, trigger_counts=20000,
                    min_tare_headroom_percent=50)
    assert refused(h.get_safety_range, -1750000) is not None   # 42%
    assert refused(h.get_safety_range, -1400000) is None       # 53%
    print('ok  at-anchor, 5%, 19.9% and floor refused; 20.1% allowed')


def test_boxwood_normal(mod):
    expect_lo = {-578000: -2889110, -800000: -2900210, -1750000: -2947710}
    for name, opts in MODES:
        for tare in BOX_TARES:
            h = make_helper(mod, BOX_REF, counts_per_gram=1000., **opts)
            lo, hi, ts = arm(h, tare)
            assert (lo, hi) == (expect_lo[tare], SENSOR_MAX_USABLE), (
                name, tare, lo, hi)
            if name == 'grams':
                assert ts['trigger_type'] == 'abs_ge', ts
            else:
                assert ts['scale'] == -1., (name, tare, ts)
                assert ts['trigger_type'] == 'gt', (name, tare, ts)
    print('ok  Boxwood tares -578000, -800000, -1750000: scale -1, gt, '
          'range [ref + margin, sensor_max - 1]')


def test_sloe(mod):
    expect_lo = {200000: -1496453, 230000: -1494953, 790000: -1466953,
                 -390000: -1525953}
    for name, opts in MODES[:2]:
        for tare in SLOE_TARES:
            h = make_helper(mod, SLOE_REF, **opts)
            lo, hi, ts = arm(h, tare)
            assert ts['scale'] == -1. and ts['trigger_type'] == 'gt', ts
            assert (lo, hi) == (expect_lo[tare], SENSOR_MAX_USABLE), (
                name, tare, lo, hi)
        h = make_helper(mod, SLOE_REF, **opts)
        msg = refused(arm, h, -1400000)
        assert msg is not None and msg.startswith(REFUSAL), (
            'Sloe tare -1400000 allowed (%s)' % (name,))
    # uncalibrated cell: the guard allows a good tare and refuses a bad one
    h = make_helper(mod, SLOE_REF, counts_per_gram=None, trigger_force=75)
    assert refused(h.get_safety_range, 200000) is None
    assert refused(h.get_safety_range, -1400000) is not None
    print('ok  Sloe tares +200000, +230000, +790000, -390000 allowed, '
          'direction -1; -1400000 refused')


def test_mirror_and_explicit(mod):
    ref = 3010747
    for name, opts in MODES[:2]:
        h = make_helper(mod, ref, **opts)
        lo, hi, ts = arm(h, 578000)
        assert ts['scale'] == 1. and ts['trigger_type'] == 'gt', ts
        assert (lo, hi) == (SENSOR_MIN_USABLE, 2889110), (lo, hi)
        msg = refused(arm, h, 3244819)
        assert msg == ("The nozzle is already under load: check nothing is "
                       "touching it (tare 3244819, max-load reference "
                       "3010747, headroom -7.8%, need 20.0%)"), msg
    # explicit press_direction overrides the sign of the anchor
    h = make_helper(mod, BOX_REF, trigger_counts=20000, press_direction=1)
    assert h.get_press_direction() == 1.
    # the anchor is then on the unloaded side of -578000: refused
    assert refused(h.get_safety_range, -578000) is not None
    h = make_helper(mod, ref, trigger_counts=20000, press_direction=-1)
    lo, hi, ts = arm(h, 5000000)
    assert ts['scale'] == -1. and ts['trigger_type'] == 'gt', ts
    assert (lo, hi) == (3110209, SENSOR_MAX_USABLE), (lo, hi)
    # zero anchor with no option: refuse and say what to set
    h = make_helper(mod, 0, trigger_counts=20000)
    msg = refused(h.get_safety_range, 1000)
    assert msg is not None and 'set press_direction' in msg, msg
    h = make_helper(mod, 0, trigger_counts=20000, press_direction=-1)
    assert h.get_press_direction() == -1.
    # bad option value is a config error
    try:
        make_helper(mod, BOX_REF, press_direction=2)
    except ConfigError:
        pass
    else:
        raise AssertionError('press_direction=2 was accepted')
    print('ok  mirror unit +1, explicit press_direction, zero anchor '
          'needs the option, bad value refused')


def test_contact_guard_direction(mod):
    h = make_helper(mod, BOX_REF, trigger_counts=20000)
    p = object.__new__(mod.LoadCellPrinterProbe)
    p._config_helper = h
    for zero in (-1000000, -3500000, -3010747, 0, 500000):
        assert p.get_press_direction(zero) == -1., zero
    # reference_tare falls back to the sensor orientation
    h2 = make_helper(mod, BOX_REF)
    h2._safety_model = 'reference_tare'
    h2._load_cell.invert = 1.
    p2 = object.__new__(mod.LoadCellPrinterProbe)
    p2._config_helper = h2
    p2._load_cell = h2._load_cell
    assert p2.get_press_direction(-1000000) == 1.
    print('ok  get_press_direction is -1 for Boxwood either side of the '
          'anchor; reference_tare unchanged')


def test_status(mod):
    h = make_helper(mod, BOX_REF, trigger_counts=20000)
    p = object.__new__(mod.LoadCellPrinterProbe)
    p._config_helper = h
    assert p._get_status_press_direction() == -1
    assert h.get_min_tare_headroom_percent() == 20.
    print('ok  status values')


def test_no_inference_left(mod):
    src = open(_paths.patched('klippy/extras/load_cell_probe.py')).read()
    for bad in ('ref_max >= tare_counts', 'ref_max >= zero_counts',
                'tare_counts <= ref_max', 'tare_counts > ref_max'):
        assert bad not in src, bad
    print('ok  no tare-vs-anchor comparison left in the source')


def main():
    mod = load_module(_paths.patched('klippy/extras/load_cell_probe.py'))
    tests = [test_incident, test_incident_nothing_sent,
             test_good_tare_arms_through_pause_and_tare,
             test_headroom_boundaries, test_boxwood_normal, test_sloe,
             test_mirror_and_explicit, test_contact_guard_direction,
             test_status, test_no_inference_left]
    for t in tests:
        t(mod)
    print('All %d tare guard tests passed' % len(tests))


if __name__ == '__main__':
    main()
