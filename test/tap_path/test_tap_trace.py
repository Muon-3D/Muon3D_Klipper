"""KAN-1361 diagnostic: TAP_TRACE writes one CSV row per accelerometer
sample around each tap, with the INT time, the onset time and the
positions at each, and scripts/tap_trace_summary.py reads it back.

    python3 test/tap_path/test_tap_trace.py

Covers the CSV writer (klippy/extras/tap_trace.py), the summary script,
and adxl345_probe's hook driven with a fake homing move: it must write the
tap and must not move the recorded trigger unless tap_trigger_from_samples
is on.
"""
import csv
import math
import os
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _paths  # noqa: E402

sys.argv = sys.argv[:1]
sys.path.insert(0, _paths.patched('klippy'))
sys.path.insert(0, _paths.patched('scripts'))
from extras import tap_trace  # noqa: E402
import tap_trace_summary  # noqa: E402

RATE = 3200.
DT = 1. / RATE
THRESH = 15000.
SPEED = 60.
STEP_DIST = 0.003125


def stream(t_tap, t0=0., t_end=0.5, amp=40000.):
    out = []
    for i in range(int((t_end - t0) * RATE)):
        t = t0 + i * DT
        v = 500. * math.sin(2 * math.pi * 1300. * t)
        if t >= t_tap:
            dt = t - t_tap
            v += amp * math.exp(-dt / 0.02) * math.sin(
                2 * math.pi * 77. * dt + 1.2)
        out.append((t, 10., v, -9800.))
    return out


def tap(home, n, int_time, onset, pos_int, pos_onset, phase='confirm'):
    return {'home': home, 'tap': n, 'phase': phase, 'axis': 'y',
            'speed': SPEED, 'applied': 'int', 'int_time': int_time,
            'onset_time': onset, 'thresh': THRESH, 'tap_axes': 'y',
            'start_pos_mm': pos_int - 16., 'int_pos_mm': pos_int,
            'onset_pos_mm': pos_onset, 'halt_pos_mm': pos_int + 0.05,
            'int_steps': {'stepper_x': 100, 'stepper_y': -100},
            'onset_steps': {'stepper_x': 98, 'stepper_y': -98},
            'halt_steps': {'stepper_x': 102, 'stepper_y': -102}}


def read_rows(path):
    with open(path, newline='') as fh:
        return list(csv.DictReader(fh))


def test_rows_cover_the_window_around_int():
    rows = tap_trace.tap_rows(tap(1, 1, 0.3, 0.29, 12.0, 11.4),
                              stream(0.29), 0.06, 0.03)
    ts = [float(r['t_rel_int_ms']) for r in rows]
    assert min(ts) >= -60. - 1e-6 and max(ts) <= 30. + 1e-6, (ts[0], ts[-1])
    assert len(rows) >= int(0.09 * RATE) - 1, len(rows)
    r = rows[0]
    assert set(r) == set(tap_trace.FIELDS), set(r) ^ set(tap_trace.FIELDS)
    assert r['int_minus_onset_ms'] == '10.000', r['int_minus_onset_ms']
    assert r['int_pos_mm'] == '12.00000' and r['onset_pos_mm'] == '11.40000'
    assert r['int_steps'] == 'stepper_x:100;stepper_y:-100', r['int_steps']
    assert r['triggered'] == 1
    over = [r for r in rows if r['over_thresh'] == 1]
    assert over and float(over[0]['t_rel_int_ms']) >= -10.5, over[:1]
    assert all(float(r['tap_mag']) == abs(float(r['ay'])) for r in rows)


def test_tap_without_int_keeps_one_row():
    t = tap(2, 3, None, None, 0., 0.)
    t['int_pos_mm'] = t['onset_pos_mm'] = None
    rows = tap_trace.tap_rows(t, stream(0.2), 0.06, 0.03)
    assert len(rows) == 1, len(rows)
    r = rows[0]
    assert r['triggered'] == 0 and r['int_time'] == '' and r['ay'] == ''
    assert r['start_pos_mm'] != ''


def test_writer_flushes_each_tap_and_never_overwrites():
    d = tempfile.mkdtemp()
    try:
        p1 = tap_trace.trace_path(d, now=1e9)
        open(p1, 'w').close()
        p2 = tap_trace.trace_path(d, now=1e9)
        assert p2 != p1 and p2.endswith('.csv'), (p1, p2)
        w = tap_trace.TapTraceWriter(p2, 0.06, 0.03)
        n = w.write_tap(tap(1, 1, 0.3, 0.29, 12.0, 11.4, 'seek'),
                        stream(0.29))
        # readable before close: a crash mid-series keeps what came before
        rows = read_rows(p2)
        assert len(rows) == n and w.taps == 1, (len(rows), n)
        w.write_tap(tap(1, 2, 0.9, 0.887, 12.8, 11.6), stream(0.887, 0.6, 1.))
        w.close()
        assert w.closed
        rows = read_rows(p2)
        assert len(rows) == w.rows
        assert [h for h in csv.reader(open(p2))][0] == tap_trace.FIELDS
    finally:
        shutil.rmtree(d)


def test_summary_finds_the_ratchet():
    d = tempfile.mkdtemp()
    try:
        path = os.path.join(d, 'trace.csv')
        w = tap_trace.TapTraceWriter(path, 0.06, 0.03)
        # home 1: INT positions step 0.8 mm one way, onsets drift 0.2 mm
        for i in range(4):
            t0 = 1. + i
            w.write_tap(tap(1, i + 1, t0 + 0.013, t0, 12. - 0.8 * i,
                            11.4 - 0.2 * i, 'seek' if i == 0 else 'confirm'),
                        stream(t0, t0 - 0.1, t0 + 0.1))
        # home 2: agreeing taps
        for i in range(3):
            t0 = 10. + i
            w.write_tap(tap(2, i + 1, t0 + 0.004, t0, 12. + 0.003 * (-1) ** i,
                            11.4, 'confirm'), stream(t0, t0 - 0.1, t0 + 0.1))
        w.close()
        taps = tap_trace_summary.read_taps(path)
        assert [(t['home'], t['tap']) for t in taps] == [
            (1, 1), (1, 2), (1, 3), (1, 4), (2, 1), (2, 2), (2, 3)]
        per_tap, per_home = tap_trace_summary.summarise(taps)
        assert per_tap[0]['d_int_mm'] is None
        assert abs(per_tap[1]['d_int_mm'] + 0.8) < 1e-6
        assert abs(per_tap[1]['d_onset_mm'] + 0.2) < 1e-6
        assert abs(per_tap[0]['int_minus_onset_ms'] - 13.) < 1e-3
        assert abs(per_tap[0]['overrun_mm'] - 0.05) < 1e-6
        h1, h2 = per_home
        assert abs(h1['int_spread_mm'] - 2.4) < 1e-6, h1
        assert abs(h1['onset_spread_mm'] - 0.6) < 1e-6, h1
        assert h1['int_one_way'] == '-' and h1['onset_one_way'] == '-'
        assert h2['int_one_way'] == '' and h2['onset_spread_mm'] < 1e-9
        assert per_tap[0]['first_over_ms'] is not None
        assert tap_trace_summary.main([path]) == 0
    finally:
        shutil.rmtree(d)


# adxl345_probe's hook, with a fake homing move ---------------------------

class FakeStepper:
    def __init__(self, name, sign):
        self.name = name
        self.sign = sign

    def get_name(self):
        return self.name

    def get_step_dist(self):
        return STEP_DIST

    def get_past_mcu_position(self, t):
        return int(round(self.sign * SPEED * t / STEP_DIST))

    def get_mcu_position(self):
        return self.get_past_mcu_position(1.)


class FakeKin:
    def __init__(self):
        self.steppers = [FakeStepper('stepper_x', 1), FakeStepper(
            'stepper_y', -1)]

    def get_steppers(self):
        return self.steppers

    def calc_position(self, p):
        a, b = p['stepper_x'], p['stepper_y']
        return [.5 * (a + b), .5 * (a - b), 0.]


class FakeToolhead:
    def __init__(self):
        self.kin = FakeKin()

    def get_kinematics(self):
        return self.kin


class FakePrinter:
    def __init__(self, logdir):
        self.objs = {'toolhead': FakeToolhead()}
        self.logdir = logdir

    def lookup_object(self, name, default=None):
        return self.objs.get(name, default)

    def get_start_args(self):
        return {'log_file': os.path.join(self.logdir, 'klippy.log')}


class FakeReactor:
    def monotonic(self):
        return 0.

    def pause(self, t):
        raise AssertionError("samples were already there")


class FakeClient:
    def __init__(self, samples):
        self.msgs = [{'data': samples}]
        self.is_finished = False


class FakeWrapper:
    def __init__(self, t):
        self.last_trigger_time = t

    def get_axis(self):
        return 'y'


class SP:
    def __init__(self, stepper, t_int):
        self.stepper = stepper
        self.stepper_name = stepper.get_name()
        self.endstop_name = 'y'
        self.start_pos = stepper.get_past_mcu_position(t_int - 0.3)
        self.trig_pos = stepper.get_past_mcu_position(t_int)
        self.halt_pos = stepper.get_past_mcu_position(t_int + 0.002)


class FakeHmove:
    def __init__(self, wrapper, sps):
        self.endstops = [(wrapper, 'y')]
        self.stepper_positions = sps


class FakeGcmd:
    def __init__(self, **params):
        self.params = params
        self.said = []

    def get_int(self, k, default, **kw):
        return int(self.params.get(k, default)) if (
            k in self.params or default is not None) else None

    def get(self, k, default):
        return self.params.get(k, default)

    def get_float(self, k, default, **kw):
        return float(self.params.get(k, default))

    def respond_info(self, msg):
        self.said.append(msg)

    def error(self, msg):
        return Exception(msg)


class FakeMcu:
    def is_fileoutput(self):
        return False


class FakeChip:
    axes_map = [(0, 1.), (1, 1.), (2, 1.)]


def make_probe(logdir, from_samples):
    from extras import adxl345_probe
    p = object.__new__(adxl345_probe.ADXL345Probe)
    p.printer = FakePrinter(logdir)
    p.reactor = FakeReactor()
    p.tap_trigger_from_samples = from_samples
    p.tap_onset_window = 0.06
    p.tap_profiles = {'y': {'tap_thresh': THRESH, 'tap_dur': 0.03,
                            'tap_axes_mask': 0x02}}
    p._adxl_mcu = FakeMcu()
    p.adxl345 = FakeChip()
    p._trace = None
    p._trace_axes = ''
    p._trace_pre = 0.06
    p._trace_post = 0.03
    p._trace_home = 0
    p._trace_tap = 0
    p._trace_phase = 'seek'
    p._trace_speed = SPEED
    p._trace_seek_speed = None
    return p


def run_hook(from_samples):
    d = tempfile.mkdtemp()
    try:
        p = make_probe(d, from_samples)
        g = FakeGcmd(ENABLE=1)
        p.cmd_TAP_TRACE(g)
        assert p._trace is not None and p._trace.path.startswith(d), g.said
        t_tap, t_int = 0.29, 0.303
        p._trace_home = 1
        kin = p.printer.objs['toolhead'].kin
        sps = [SP(s, t_int) for s in kin.steppers]
        before = [sp.trig_pos for sp in sps]
        w = FakeWrapper(t_int)
        p._onset_client = FakeClient(stream(t_tap))
        p._apply_tap_onset(FakeHmove(w, sps), [w])
        after = [sp.trig_pos for sp in sps]
        path = p._trace.path
        g = FakeGcmd(ENABLE=0)
        p.cmd_TAP_TRACE(g)
        assert p._trace is None and '1 taps' in g.said[0], g.said
        rows = read_rows(path)
        return rows, before, after, t_tap, t_int
    finally:
        shutil.rmtree(d)


def test_hook_writes_int_and_onset_without_moving_the_trigger():
    rows, before, after, t_tap, t_int = run_hook(False)
    assert after == before, (before, after)
    r = rows[0]
    assert r['applied'] == 'int' and r['phase'] == 'seek'
    assert abs(float(r['int_time']) - t_int) < 1e-6
    assert abs(float(r['onset_time']) - t_tap) <= DT + 1e-6, r['onset_time']
    # Y = .5 * (a - b) = SPEED * t with the fake corexy steppers
    assert abs(float(r['int_pos_mm']) - SPEED * t_int) < 2 * STEP_DIST
    assert abs(float(r['onset_pos_mm']) - SPEED * t_tap) < SPEED * DT + 0.01
    assert r['int_steps'] == 'stepper_x:%d;stepper_y:%d' % tuple(before)
    ts = [float(x['t_rel_int_ms']) for x in rows]
    assert max(ts) > 25., max(ts)   # POST samples after INT are kept


def test_hook_with_onset_trigger_records_what_was_used():
    rows, before, after, t_tap, t_int = run_hook(True)
    assert after != before
    r = rows[0]
    assert r['applied'] == 'onset'
    # the INT columns still show the hardware trigger, not the refined one
    assert r['int_steps'] == 'stepper_x:%d;stepper_y:%d' % tuple(before)


def test_hook_is_silent_when_off():
    d = tempfile.mkdtemp()
    try:
        p = make_probe(d, False)
        w = FakeWrapper(0.3)
        kin = p.printer.objs['toolhead'].kin
        sps = [SP(s, 0.3) for s in kin.steppers]
        p._onset_client = FakeClient(stream(0.29))
        p._apply_tap_onset(FakeHmove(w, sps), [w])
        assert os.listdir(d) == [], os.listdir(d)
    finally:
        shutil.rmtree(d)


# G28 TAP_TRACE=1 / =0, through the real M1 macro ------------------------

def render_g28(rawparams):
    import configparser
    import jinja2
    path = os.path.join(_paths.patched('core'), 'M1', 'macros', 'homing.cfg')
    with open(path, encoding='utf-8') as f:
        lines = ['' if ln.lstrip().startswith('#') else ln
                 for ln in f.read().splitlines()]
    cp = configparser.RawConfigParser(strict=False)
    cp.read_string('\n'.join(lines))
    env = jinja2.Environment('{%', '%}', '{', '}')
    params = dict(tok.upper().split('=', 1) for tok in rawparams.split()
                  if '=' in tok)

    class Pos(dict):
        __getattr__ = dict.get
    printer = {
        'toolhead': Pos(homed_axes='', position=Pos(x=0., y=0., z=10.)),
        'mcu toolhead': Pos(non_critical_disconnected=False),
        'bed_removal_detector': Pos(bedRemoved=False),
        'gcode_macro G28': Pos(z_lifted_once=1),
    }
    printer = Pos(printer)
    out = env.from_string(cp['gcode_macro G28']['gcode']).render(
        printer=printer, params=params, rawparams=rawparams)
    return [ln.strip() for ln in out.splitlines() if ln.strip()]


def test_g28_tap_trace_turns_the_trace_on_and_off():
    on = render_g28('Y TAP_TRACE=1')
    assert 'TAP_TRACE ENABLE=1' in on, on
    assert on.index('TAP_TRACE ENABLE=1') < on.index('_HOME_Y'), on
    assert '_HOME_X' not in on, on   # TAP_TRACE=1 is not an axis
    off = render_g28('tap_trace=0')
    assert 'TAP_TRACE ENABLE=0' in off and '_HOME_Y' in off, off
    plain = render_g28('')
    assert not [ln for ln in plain if 'TAP_TRACE' in ln], plain
    assert '_HOME_Y' in plain and '_HOME_X' in plain, plain


if __name__ == '__main__':
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            try:
                fn()
                print('PASS', name)
            except Exception as e:  # noqa: BLE001
                import traceback
                traceback.print_exc()
                fails += 1
                print('FAIL', name, repr(e))
    sys.exit(1 if fails else 0)
