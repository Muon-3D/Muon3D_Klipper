"""Offline tests for the MCUTARE path, run without the bench.

Two things are checked, and the second is the one that matters:

1. UNIT -- that `_mcu_tare` and `note_baseline` actually do what the patch
   claims: offset 0 with auto_offset=True, raw range and trigger derived
   from the approximate zero, and the baseline tracked from pre-contact
   samples.

2. SENSITIVITY -- the central claim of the whole change, driven through the
   REAL `LoadCellProbeConfigHelper` methods (not a re-implementation of
   them) using this machine's actual calibration read back from the live
   printer:

       tare_counts                 -913,186
       reference_max_load_counts -3,010,747
       raw_safety_min            -2,905,869
       raw_safety_max             8,388,606
       safety_model            preloaded_max
       trigger_percent                    10

   With MCUTARE the MCU supplies the zero and the host supplies only the
   envelope and the trigger MAGNITUDE. So the question that decides whether
   this is safe and accurate is: how far do the envelope and the magnitude
   move when the host's zero is stale? If they barely move, an approximate
   host zero is fine and the precision genuinely comes from the MCU.

The module is exec'd with its klippy imports stubbed, and the objects are
built with object.__new__ so that only the code under test runs.
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _paths
import sys
import types

K_EFF = 1.95e7          # counts per mm, measured on this machine
SENSOR_RANGE = (-8388608, 8388607)
REF_MAX = -3010747
TRUE_TARE = -913186


def load_module(path):
    pkg = types.ModuleType('extras')
    pkg.__path__ = []
    sys.modules['extras'] = pkg
    for n in ('probe', 'trigger_analog', 'load_cell', 'hx71x', 'ads1220'):
        m = types.ModuleType('extras.' + n)
        sys.modules['extras.' + n] = m
        setattr(pkg, n, m)
    sys.modules['mcu'] = types.ModuleType('mcu')
    mod = types.ModuleType('extras.load_cell_probe')
    mod.__package__ = 'extras'
    sys.modules['extras.load_cell_probe'] = mod
    exec(compile(open(path).read(), path, 'exec'), mod.__dict__)
    return mod


class Param(object):
    def __init__(self, v):
        self.v = v

    def get(self, gcmd=None, **kw):
        return self.v


class McuStub(object):
    def estimated_print_time(self, t):
        return t


class Sensor(object):
    def get_range(self):
        return SENSOR_RANGE

    def get_mcu(self):
        return McuStub()


class CollectorStub(object):
    # the freshness check: one sample newer than min_time, no errors
    def start_collecting(self, min_time=None):
        self.min_time = min_time

    def collect_min(self, n=1):
        return ([(self.min_time + 0.0005, 0.0, -913186)], None)


class LoadCellStub(object):
    def __init__(self):
        self.tared_to = None
        self.sensor = Sensor()

    def get_sensor(self):
        return self.sensor

    def get_collector(self):
        return CollectorStub()

    def get_counts_per_gram(self):
        return None

    def tare(self, c):
        self.tared_to = c


class ReactorStub(object):
    def monotonic(self):
        return 1000.0


class PrinterStub(object):
    command_error = RuntimeError

    def get_reactor(self):
        return ReactorStub()


def make_config_helper(mod):
    ch = object.__new__(mod.LoadCellProbeConfigHelper)
    ch._printer = PrinterStub()
    ch._load_cell = LoadCellStub()
    ch._safety_model = 'preloaded_max'
    ch._reference_max_load_counts = REF_MAX
    ch._max_load_safety_margin_counts_param = Param(0)
    ch._max_load_safety_margin_percent_param = Param(5.0)
    ch._trigger_counts_param = Param(0)
    ch._trigger_percent_param = Param(10)
    return ch


def test_matches_live_printer(mod):
    ch = make_config_helper(mod)
    lo, hi = ch.get_safety_range(TRUE_TARE)
    ts = ch.get_trigger_setup(TRUE_TARE, lo, hi)
    ok = (lo == -2905869 and hi == 8388606)
    print('  stub reproduces live printer envelope: %s  [%d, %d]'
          % ('YES' if ok else 'NO', lo, hi))
    print('  trigger: type=%s scale=%s value=%d mode=%s'
          % (ts['trigger_type'], ts['scale'], ts['trigger_value'],
             ts['trigger_mode']))
    assert ok, 'stub does NOT match the live printer -- numbers below are void'
    return lo, hi, ts


def test_sensitivity(mod):
    ch = make_config_helper(mod)
    base_lo, base_hi = ch.get_safety_range(TRUE_TARE)
    base_ts = ch.get_trigger_setup(TRUE_TARE, base_lo, base_hi)
    print('  %-12s %-14s %-14s %-16s' % ('zero error', 'safety_min',
                                         'trigger_counts', 'Z effect'))
    worst = 0.
    for err in (-5000, -2000, -500, -100, 100, 500, 2000, 5000):
        t = TRUE_TARE + err
        lo, hi = ch.get_safety_range(t)
        ts = ch.get_trigger_setup(t, lo, hi)
        dtrig = ts['trigger_value'] - base_ts['trigger_value']
        z_um = 1e3 * dtrig / K_EFF
        worst = max(worst, abs(z_um))
        print('  %-+12d %-14d %-14d %+.5f um'
              % (err, lo, ts['trigger_value'], z_um))
    print()
    print('  worst Z effect over +/-5000 counts of stale zero: %.5f um'
          % worst)
    print('  (real drift between adjacent mesh points is far under 5000;'
          ' per-sample sd is 269)')
    assert worst < 0.1, 'envelope is NOT insensitive -- claim is wrong'


def test_mcu_tare(mod):
    calls = {}

    class Sos(object):
        def set_offset_scale(self, offset=0, scale=1., scale_frac_bits=0,
                             auto_offset=False):
            calls['offset_scale'] = (offset, scale, scale_frac_bits,
                                     auto_offset)

    class Trig(object):
        def get_sos_filter(self):
            return Sos()

        def set_raw_range(self, lo, hi):
            calls['raw_range'] = (lo, hi)

        def set_trigger(self, t, v):
            calls['trigger'] = (t, v)

    class Filt(object):
        def update_from_command(self, gcmd):
            calls['filter_updated'] = True

    o = object.__new__(mod.LoadCellProbingMove)
    o._printer = PrinterStub()
    o._config_helper = make_config_helper(mod)
    o._load_cell = LoadCellStub()
    o._mcu_trigger_analog = Trig()
    o._continuous_tare_filter_helper = Filt()
    o._approx_tare = TRUE_TARE
    o._mcu_tare(None)

    off, scale, frac, auto = calls['offset_scale']
    print('  set_offset_scale(offset=%d, scale=%s, frac=%d, auto_offset=%s)'
          % (off, scale, frac, auto))
    assert auto is True, 'auto_offset NOT set -- the MCU would not self-tare'
    assert off == 0, 'offset should be a placeholder the MCU overwrites'
    assert calls['raw_range'] == (-2905869, 8388606), calls['raw_range']
    assert calls['trigger'][0] == 'gt', calls['trigger']
    assert calls.get('filter_updated'), 'filter helper not updated'
    assert o._load_cell.tared_to == TRUE_TARE
    print('  raw_range=%s trigger=%s' % (calls['raw_range'], calls['trigger']))


def test_note_baseline(mod):
    o = object.__new__(mod.LoadCellProbingMove)
    o._approx_tare = None
    # 64 pre-contact samples around -913000, then contact drives it down
    samples = [[i * 0.0005, 0.0, -913000 + (i % 7) - 3] for i in range(64)]
    samples += [[0.0, 0.0, -2500000] for _ in range(200)]
    o.note_baseline(samples)
    print('  baseline from 264 samples (200 of them contact): %d'
          % o._approx_tare)
    assert abs(o._approx_tare - (-913000)) < 50, o._approx_tare

    # too few samples must be a no-op, not a bad zero
    o2 = object.__new__(mod.LoadCellProbingMove)
    o2._approx_tare = 12345
    o2.note_baseline([[0., 0., -999999]] * 4)
    assert o2._approx_tare == 12345, 'short input must not overwrite the zero'
    print('  short input correctly left the previous zero untouched')

    # a tap that triggered instantly would put contact samples in the
    # window; that must not be allowed to drag the zero
    o3 = object.__new__(mod.LoadCellProbingMove)
    o3._approx_tare = TRUE_TARE
    o3.note_baseline([[0., 0., -2500000]] * 64)
    assert o3._approx_tare == TRUE_TARE, o3._approx_tare
    print('  implausible baseline (contact-contaminated) correctly REJECTED')

    # ordinary thermal drift must still be accepted
    o4 = object.__new__(mod.LoadCellProbingMove)
    o4._approx_tare = TRUE_TARE
    o4.note_baseline([[0., 0., TRUE_TARE + 800]] * 64)
    assert o4._approx_tare == TRUE_TARE + 800, o4._approx_tare
    print('  ordinary drift (+800 counts) correctly accepted')


def main():
    mod = load_module(_paths.patched('klippy/extras/load_cell_probe.py'))
    print('module exec: OK')
    print()
    print('[1] stub fidelity against the live printer')
    test_matches_live_printer(mod)
    print()
    print('[2] does a stale host zero move the envelope or the trigger?')
    test_sensitivity(mod)
    print()
    print('[3] _mcu_tare arms the MCU to take its own zero')
    test_mcu_tare(mod)
    print()
    print('[4] note_baseline tracks the pre-contact zero')
    test_note_baseline(mod)
    print()
    print('ALL PASS')


main()
