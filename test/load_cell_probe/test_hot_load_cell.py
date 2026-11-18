"""A hot load cell must not abort the Z home or fail the mesh (KAN-4364).

Sloe (B1, train 17, Klipper 61caab27), three PRINT_START attempts:

 1. Homing Z aborted twice: "load_cell_probe: Z move aborted: the load cell
    moved the wrong way during the Z move (tare -121734, ceiling 366000,
    wrong way 146400)" and "(tare 1006163, ceiling 647975, wrong way
    259190)".  The tares drifted -121k -> 1.0M -> 1.85M counts as the hot
    end heated (KAN-3818: 1.7-2.4M counts hot), and the wrong-way rule
    (1.0 x the trigger delta from the tare) tripped on that drift.
 2. Attempt 3 homed, then the mesh failed: the two taps at every point
    alternated by a fixed ~0.047 mm (z=0.1585/0.2027/0.1551/0.2022...),
    samples_tolerance was 0.045: "Probe samples exceed samples_tolerance".

KAN-4364 (#148, after two reviews of a drift follower): a probing move
longer than 20 mm runs as 20 mm segments, each with its own fixed tare,
backstop and trigger.  Tests 1-3 and 5-7 drive the real
LoadCellProbingMove (probing_move, _probe_segment, _pause_and_tare) and
ZLoadBackstop through a simulated world: a nozzle descending over a bed,
the cell drifting as Sloe's did, the MCU trigger checked on every sample
and the host watch fed 20 ms batches.  Test 6 runs the real
LOAD_CELL_HOME_Z on top.  Test 4 is the shipped samples_tolerance.

    python3 test/load_cell_probe/test_hot_load_cell.py

Needs numpy.
"""
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'tap_path'))

import test_z_load_backstop as zb  # noqa: E402
import types  # noqa: E402


def _load_retract_first():
    # test_retract_first.py ends in an unguarded sys.exit(main()): take
    # its probe harness (run, load_module, descents) without running it.
    path = os.path.join(HERE, '..', 'tap_path', 'test_retract_first.py')
    src = open(path).read().replace('sys.exit(main())', 'pass')
    mod = types.ModuleType('retract_first')
    mod.__dict__['__file__'] = path
    exec(compile(src, path, 'exec'), mod.__dict__)
    return mod


rf = _load_retract_first()
tg = zb.tg

ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
SPS = zb.SPS
PRESS = -1.         # a press lowers the counts, as in the backstop tests

# The two logged limits (delta = ceiling / 2.5)
DELTA_1 = 146400    # tare -121734, ceiling 366000
DELTA_2 = 259190    # tare 1006163, ceiling 647975
TARE_1 = -121734
TARE_2 = 1006163


# --- a descent in a simulated world ---------------------------------------

SLOE_RATE = 1100000. / 23.     # Sloe's heat-up drift, counts/s
SLOE_REF = zb.SLOE_REF


def wander(i, seed=[4364]):
    # Sloe's idle wander, -4000..+7000 counts
    seed[0] = (seed[0] * 1103515245 + 12345) % (1 << 31)
    return -4000 + seed[0] % 11001


class World(object):
    """The Z axis over a bed and the load cell on the nozzle.  `real` is
    the nozzle's height over the bed (mm), the toolhead's commanded Z is
    real + off.  The cell reads tare0 + drift(seconds) + load toward the
    press side (the other way with flip, a sign error); load is
    stiff counts/mm under real 0, plus `soft` counts/mm under soft_at mm
    (a soft object on the bed).  bump=(at_s, secs, counts) adds a spike."""

    def __init__(self, tare0=TARE_1, drift=None, stiff=1000000., flip=False,
                 real=150., noise=True, bump=None, soft=0., soft_at=0.):
        self.tare0 = tare0
        self.drift = drift or (lambda s: 0.)
        self.stiff = stiff
        self.flip = flip
        self.real = real
        self.off = 0.
        self.noise = noise
        self.bump = bump
        self.soft, self.soft_at = soft, soft_at
        self.t = 10.
        self.segments = []      # (start real, end real) of each move

    @property
    def z(self):
        return self.real + self.off

    def load(self, real):
        load = max(0., -real) * self.stiff
        if self.soft:
            load += max(0., self.soft_at - real) * self.soft
        return load

    def counts(self):
        c = self.tare0 + self.drift(self.t - 10.)
        sign = -PRESS if self.flip else PRESS
        c += sign * self.load(self.real)
        if self.noise:
            c += wander(0)
        if self.bump is not None:
            at, secs, b = self.bump
            if at <= self.t - 10. < at + secs:
                c += b
        return int(c)


class Toolhead(object):
    def __init__(self, world):
        self.w = world

    def get_position(self):
        return [100., 100., self.w.z, 0.]

    def dwell(self, t):
        self.w.t += t


class Collector(object):
    """The tare: the queue drains (0.15 s, at rest), then n samples."""

    def __init__(self, world, m):
        self.w = world
        m._collect_start_time = world.t

    def start_collecting(self, min_time=None):
        pass

    def collect_min(self, n=1):
        self.w.t += 0.15
        out = []
        for _ in range(n):
            self.w.t += 1. / SPS
            out.append([self.w.t, 0., self.w.counts()])
        return out, None

    def stop_collecting(self):
        pass


class Phoming(object):
    """PrinterHoming for a probing move: moves the nozzle at `speed`, one
    sample at a time.  The MCU trigger ("gt" on the tare the move was
    armed with) is checked on every sample unless mcu is False (a dead
    trigger); the host watch gets 20 ms batches and may abort."""

    max_search_distance_z = 0.

    def __init__(self, world, m, mcu=True):
        self.w = world
        self.m = m
        self.mcu = mcu
        self.calls = []
        self.trigger_load = None
        self.trigger_delta = None

    def abort_active_move(self, reason, z_only=True):
        self.calls.append(reason)
        return True

    def probing_move(self, endstop, pos, speed):
        w, m = self.w, self.m
        target = pos[2] - w.off
        start = w.real
        zero, delta = m._tare_counts, m._trigger_delta
        press = m._trigger_press_dir
        batch = []
        cell = m._load_cell
        n = self.calls[:]
        try:
            while True:
                w.real = max(target, w.real - speed / float(SPS))
                w.t += 1. / SPS
                c = w.counts()
                batch.append([w.t, None, c, None])
                if self.mcu and (c - zero) * press > delta:
                    cell.feed(batch)
                    self.trigger_load = w.load(w.real)
                    self.trigger_delta = delta
                    return [100., 100., w.z, 0.]
                if len(batch) == 40 or w.real <= target:
                    cell.feed(batch)
                    batch = []
                    if len(self.calls) > len(n):
                        raise tg.CommandError(
                            "Homing aborted: " + self.calls[-1])
                if w.real <= target:
                    raise tg.CommandError(
                        "No trigger on probe after full movement")
        finally:
            w.segments.append((start, w.real))


class ProbeParams(object):
    def __init__(self, speed):
        self.speed = speed

    def get_probe_params(self, gcmd):
        if gcmd is not None:
            return {'probe_speed': gcmd.get_float('PROBE_SPEED',
                                                  self.speed)}
        return {'probe_speed': self.speed}


def make(mod, world, speed=15., mcu=True, ref=SLOE_REF, fixed=None):
    """The real LoadCellProbingMove (probing_move, _probe_segment and,
    unless fixed=(tare, delta), the real _pause_and_tare with its
    KAN-3370 checks) and the real ZLoadBackstop, in `world`."""
    helper = tg.make_helper(mod, ref, trigger_percent=10)
    cell = zb.Cell(SPS, counts_per_gram=1000., tare_counts=world.tare0)
    helper._load_cell = cell
    m = tg.make_move(mod, helper, world.tare0)
    m._load_cell = cell
    ph = Phoming(world, m, mcu=mcu)
    printer = zb.Printer(ph)
    printer.objects['toolhead'] = Toolhead(world)
    m._printer = printer
    helper._printer = printer
    m._z_min_position = -4.
    m._param_helper = ProbeParams(speed)
    m._nozzle_is_clear = lambda: False
    m._start_collector = lambda: Collector(world, m)
    m._z_watch = mod.ZLoadBackstop(tg.FakeConfig(printer, {}), printer,
                                   cell)
    if fixed is not None:
        # a tare the real helper would refuse (the incident's): the
        # counts at rest now, with the incident's delta
        def tare_fn(gcmd, prev_tare=None):
            col = Collector(world, m)
            rows, _ = col.collect_min(80)
            tare = int(sum(r[2] for r in rows) / len(rows))
            if prev_tare is not None:
                lim = m.SEGMENT_TARE_STEP * fixed[1]
                tare = int(max(prev_tare - lim, min(prev_tare + lim, tare)))
            m._tare_counts = tare
            m._trigger_delta = fixed[1]
            m._trigger_press_dir = PRESS
        m._pause_and_tare = tare_fn
    return m, ph


def descend(mod, world, **kw):
    """One PROBE: returns (move, phoming, error or None)."""
    m, ph = make(mod, world, **kw)
    err = None
    try:
        m.probing_move(None)
    except tg.CommandError as e:
        err = str(e)
    return m, ph, err


def sloe_drifts():
    r = SLOE_RATE
    return [
        ('heating (+1.1M in 23 s)', lambda s: r * s),
        ('heating the press way', lambda s: -r * s),
        ('stops after 4 s', lambda s: r * min(s, 4.)),
        ('decays (tau 3 s)', lambda s: 3. * r * (1. - math.exp(-s / 3.))),
        ('reverses at 5 s', lambda s: r * (s if s < 5. else 10. - s)),
        ('press way, stops', lambda s: -r * min(s, 4.)),
    ]


def test_1_hot_g28_from_the_top(mod):
    # The G28 fast approach: 150 mm at 15 mm/s (8 segments), the cell
    # drifting as Sloe's did, or stopping, decaying or reversing; the bed
    # at 1M counts/mm.  No abort, and the trigger fires at the trigger
    # delta of real force, give or take one segment of drift.
    worst = 0.
    for name, drift in sloe_drifts():
        w = World(drift=drift)
        m, ph, err = descend(mod, w)
        assert err is None and ph.calls == [], (name, err, ph.calls)
        assert len(w.segments) == 8, (name, w.segments)
        assert abs(w.real) < 0.5, (name, w.real)
        f = ph.trigger_load / ph.trigger_delta
        assert 0.5 < f < 1.5, (name, f)
        worst = max(worst, abs(f - 1.))
        # every segment ended on its own end, except the last
        assert all(abs(b - a) <= 20.0001 for a, b in w.segments), name
    # the same heat-up as ONE move (no segments) is the KAN-4364 abort
    w = World(drift=sloe_drifts()[0][1])
    m, ph = make(mod, w)
    m.SEGMENT_MM = 1000.
    try:
        m.probing_move(None)
    except tg.CommandError as e:
        assert zb.WRONG_WAY_REASON in str(e), str(e)
    else:
        raise AssertionError('one 150 mm move did not abort')
    print('ok  1 a 150 mm G28 descent in 8 segments, Sloe drift heating, '
          'stopping, decaying or reversing either way: no abort, trigger '
          'within %.2f x delta of nominal (one move aborts)' % worst)


def test_2_a_press_at_any_rate(mod):
    # Cold and hot (Sloe's drift either way), contact at 1.5M counts/s
    # (100000 counts/mm at 15 mm/s) to 15M counts/s, from far up
    # (soft-homed at Z160): the trigger fires by 1.5 x delta.
    out = []
    for drift in (None, sloe_drifts()[0][1], sloe_drifts()[1][1]):
        for stiff in (100000., 1000000.):
            w = World(drift=drift, stiff=stiff, real=60.)
            w.off = 100.
            m, ph, err = descend(mod, w)
            assert err is None, (stiff, err)
            f = ph.trigger_load / ph.trigger_delta
            assert 0.5 < f < 1.5, (stiff, f)
            out.append(f)
    # KNOWN LIMIT (reported): 150000 counts/s (10000 counts/mm, a soft
    # object, not the bed).  Cold it fires at 1 x delta; against Sloe's
    # wrong-way drift each segment nets under a delta, so the later
    # tares take in 0.75 x delta each and it fires at ~4 x delta.
    soft = []
    for drift in (None, sloe_drifts()[0][1], sloe_drifts()[1][1]):
        w = World(drift=drift, stiff=10000., real=60.)
        w.off = 100.
        m, ph, err = descend(mod, w)
        assert err is None, err
        f = ph.trigger_load / ph.trigger_delta
        assert f < 4.5, (drift, f)
        soft.append(f)
    assert soft[0] < 1.1, soft
    # With the trigger dead the ceiling stops a stiff press at 2.5 x
    # delta of the segment's tare (+ the drift since it, the 20 ms
    # confirm and one batch at that rate).
    dead = []
    for drift in (None, sloe_drifts()[0][1], sloe_drifts()[1][1]):
        for stiff in (100000., 1000000.):
            w = World(drift=drift, stiff=stiff, real=60.)
            w.off = 100.
            m, ph, err = descend(mod, w, mcu=False)
            assert err == 'Homing aborted: ' + zb.CEILING_REASON, err
            delta = m._trigger_delta
            f = w.load(w.real) / delta
            slack = (0.6 * delta + stiff * 15. * 0.042) / delta
            assert 2.4 < f < 2.5 + slack, (drift, stiff, f, slack)
            dead.append(f)
    # KNOWN LIMIT (reported, not bounded here): trigger dead AND a soft
    # press (10000 counts/mm).  Each segment presses 200000 counts, under
    # the ceiling; each later tare takes in up to 0.75 x delta of it.
    # Cold the ceiling still ends it (~4.3 x delta); against Sloe's
    # wrong-way drift the net per segment is barely over the 0.75 and the
    # descent runs to its end.
    lim = []
    for drift in (None, sloe_drifts()[0][1]):
        w = World(drift=drift, stiff=10000., real=60.)
        w.off = 100.
        m, ph, err = descend(mod, w, mcu=False)
        lim.append(((err or 'triggered').split(':')[0],
                    w.load(w.real) / m._trigger_delta))
    print('ok  2 contact at 1.5M-15M counts/s, cold or drifting either '
          'way, triggers at %.2f-%.2f x delta; trigger dead: the ceiling '
          'stops it at %.2f-%.2f x delta.  LIMIT, 150k counts/s: triggers '
          'at %.2f / %.2f / %.2f x delta (cold / drift away / in); trigger '
          'dead: %s at %.2f (cold), %s at %.2f x delta (drifting)'
          % ((min(out), max(out), min(dead), max(dead)) + tuple(soft)
             + lim[0] + lim[1]))


def test_3_wrong_way_push(mod):
    # A sign error: contact reads the wrong way, at 1.5M counts/s
    # (100000 counts/mm), cold and hot: the wrong-way rule ends it at
    # 1 x delta, give or take one segment of drift, plus what the 20 ms
    # confirm and the 20 ms batch add at that rate (63000).
    out = []
    for drift in (None, sloe_drifts()[0][1], sloe_drifts()[1][1]):
        w = World(drift=drift, flip=True, real=60., stiff=100000.)
        w.off = 100.
        m, ph, err = descend(mod, w)
        assert err == 'Homing aborted: ' + zb.WRONG_WAY_REASON, err
        delta = m._trigger_delta
        f = w.load(w.real) / delta
        lo = 1. - (0.5 if drift else 0.05)
        hi = 1. + (0.5 if drift else 0.05) + 1500000. * 0.042 / delta
        assert lo < f < hi, (f, lo, hi)
        out.append(f)
    print('ok  3 a wrong-way contact at 1.5M counts/s aborts at %.2f / '
          '%.2f / %.2f x delta (cold / drifting either way; 0.43 of it '
          'is the 40 ms of confirm and batch)' % tuple(out))


def test_4b_slow_press_across_segments(mod):
    # KNOWN LIMIT (reported): a press too slow to reach one delta inside
    # one 20 mm segment (under ~110000 counts/s at 15 mm/s: an object
    # softer than 7300 counts/mm) is partly taken into the next segment's
    # tare.  Here a soft object of 5000 counts/mm, 40 mm thick, on the
    # bed: each segment tares on top of what it pressed, and the bed
    # itself then triggers.  The headroom refusal (KAN-3370) and the raw
    # range still bound the absolute load.
    w = World(stiff=1000000., soft=5000., soft_at=40., real=150.,
              noise=False)
    m, ph, err = descend(mod, w)
    soft_load = w.soft * w.soft_at
    print('ok  4b (limit) a 5000 counts/mm object 40 mm deep: %s at '
          '%.2f x delta of total load (%.2f x of it the soft object)'
          % ('triggered' if err is None else err,
             w.load(w.real) / m._trigger_delta, soft_load / m._trigger_delta))


def test_5_sign_flipped_incident_at_4mm_s(mod):
    # The Boxwood incident (tare -3244819 past the anchor -3010747, delta
    # 23407, the trigger never fired) with the press reading the wrong
    # way, at 4 mm/s from the top.  The real tare refuses it (KAN-3370);
    # forced past that, the wrong-way rule must stop it no later than
    # 6.15 mm past contact (the old fixed-tare ceiling on the slow ramp).
    # Worst case for the segments: contact begins so that a segment ends
    # with the load just under the trigger and the next one tares on it.
    w = World(tare0=zb.INCIDENT_TARE, stiff=10000., flip=True, real=60.)
    m, ph = make(mod, w, speed=4., ref=zb.BOX_REF)
    try:
        m.probing_move(None)
    except tg.CommandError as e:
        assert 'already under load' in str(e), str(e)
    else:
        raise AssertionError('the incident tare was not refused')
    out = []
    for stiff in (100000., 10000.):
        under = 0.95 * zb.DELTA_INCIDENT / stiff    # mm of contact
        for real in (60., 60. - under):     # a segment ends at real -under
            w = World(tare0=zb.INCIDENT_TARE, stiff=stiff, flip=True,
                      real=real, noise=False)
            # soft-homed at the top, Z160: the bed is far above z_min
            w.off = 160. - real
            m, ph, err = descend(mod, w, speed=4., ref=zb.BOX_REF,
                                 fixed=(None, zb.DELTA_INCIDENT))
            assert err == 'Homing aborted: ' + zb.WRONG_WAY_REASON, err
            mm = -w.real
            assert mm <= 6.15, (stiff, real, mm)
            out.append(mm)
    print('ok  5 sign-flipped incident at 4 mm/s aborts %.2f / %.2f mm '
          'past contact (stiff), %.2f / %.2f mm (soft), the second of '
          'each with a segment ending just under the trigger'
          % tuple(out))


def load_home_z():
    path = os.path.join(ROOT, 'klippy', 'extras', 'load_cell_home_z.py')
    mod = types.ModuleType('home_z_under_test')
    exec(compile(open(path).read(), path, 'exec'), mod.__dict__)
    return mod


class Gcmd(object):
    def __init__(self, **params):
        self.params = params
        self.info = []

    def get(self, k, d=None, *a, **kw):
        return self.params.get(k, d)

    def get_float(self, k, d=None, *a, **kw):
        return self.params.get(k, d)

    def get_int(self, k, d=None, *a, **kw):
        return self.params.get(k, d)

    def respond_info(self, msg):
        self.info.append(msg)

    def error(self, msg):
        return tg.CommandError(msg)


def test_6_home_z_retries_an_early_false_touch(mod):
    # LOAD_CELL_HOME_Z on the real module, its PROBEs on the segmented
    # probing_move in the world.  A spike toward the press side (a fan
    # winding down: 1.5 x delta for 30 ms, 3 s into the first approach)
    # stops the fast approach in mid air; the confirm tap finds nothing,
    # so the home starts again from the top and finds the bed.
    hz = load_home_z()
    w = World(drift=sloe_drifts()[0][1], real=150.,
              bump=(3., 0.03, -1.5 * DELTA_1))
    m, ph = make(mod, w, speed=5.)
    last = {}

    class Gcode(object):
        error = tg.CommandError

        def __init__(self):
            self.cmds = {}
            self.relative = False

        def register_command(self, name, fn, desc=None):
            self.cmds[name] = fn

        def run_script_from_command(self, script):
            for line in script.split('\n'):
                p = line.split()
                if not p:
                    continue
                if p[0] == 'SET_KINEMATIC_POSITION':
                    w.off = float(p[1].split('=')[1]) - w.real
                elif p[0] == 'PROBE':
                    kw = dict(x.split('=') for x in p[1:])
                    g = Gcmd(**{k: float(v) for k, v in kw.items()})
                    epos, _ = m.probing_move(g)
                    last['z'] = epos[2]
                elif p[0] == '_HOME_Z_FROM_LAST_PROBE':
                    w.off -= last['z']
                elif p[0] == 'G91':
                    self.relative = True
                elif p[0] == 'G1':
                    w.real += float(p[1][1:])
                    w.t += 0.2

    class HzToolhead(Toolhead):
        def get_status(self, et):
            return {'axis_maximum': [200., 174., 160.],
                    'axis_minimum': [0., -10., -4.]}

    gcode = Gcode()

    class HzPrinter(object):
        def lookup_object(self, name):
            return {'gcode': gcode, 'toolhead': HzToolhead(w)}[name]

        def get_reactor(self):
            return zb.Reactor()

    class Cfg(object):
        def get_printer(self):
            return HzPrinter()

        def getint(self, k, d, **kw):
            return d

        def getfloat(self, k, d, **kw):
            return d
    obj = hz.load_config(Cfg())
    g = Gcmd()
    gcode.cmds['LOAD_CELL_HOME_Z'](g)
    assert obj.false_touches == 1 and obj.last_attempts == 2, (
        obj.false_touches, obj.last_attempts, g.info)
    # homed: Z0 is the bed (to the trigger's own overtravel), lifted 10
    assert 0. <= w.off < 0.3 and abs(w.real - 10.) < 0.3, (w.off, w.real)
    assert ph.calls == [], ph.calls
    print('ok  6 LOAD_CELL_HOME_Z: a mid-air false touch in a segment is '
          'retried from the top and homes on the bed (%d segments in all)'
          % len(w.segments))


def test_7_segments_and_short_moves(mod):
    m, ph = make(mod, World())
    ends = m.segment_ends
    assert ends(150., -4.) == [130., 110., 90., 70., 50., 30., 10., -4.]
    assert ends(16., -4.) == [-4.] and len(ends(16.1, -4.)) == 2
    assert ends(1.2, -4.) == [-4.]      # a confirm tap: one move
    # a tap is one move, one tare, exactly as before
    w = World(real=3.)
    m, ph, err = descend(mod, w)
    assert err is None and len(w.segments) == 1, (err, w.segments)
    # no contact at all: only the last segment says so
    w = World(real=150., stiff=0.)
    m, ph, err = descend(mod, w)
    assert err == "No trigger on probe after full movement", err
    assert len(w.segments) == 8 and abs(w.z + 4.) < 1e-6, (w.segments, w.z)
    # max_search_distance_z bounds the whole descent, with its message
    w = World(real=150., stiff=0.)
    m, ph = make(mod, w)
    ph.max_search_distance_z = 50.
    try:
        m.probing_move(None)
    except tg.CommandError as e:
        assert 'no contact within 50 mm' in str(e), str(e)
    assert abs(w.real - 100.) < 1e-6 and len(w.segments) == 3, (
        w.real, w.segments)
    # every segment's tare goes through the KAN-3370 headroom check
    w = World(real=150.)
    m, ph = make(mod, w)
    seen = []
    orig = m._config_helper.check_tare_headroom

    def check(tare, gcmd=None):
        seen.append(tare)
        return orig(tare, gcmd)
    m._config_helper.check_tare_headroom = check
    m.probing_move(None)
    assert len(w.segments) == 8 and len(seen) == 2 * 8, (seen, w.segments)
    print('ok  7 segment ends, a tap is one move, "No trigger" only from '
          'the last segment, max_search_distance_z over the whole '
          'descent, KAN-3370 on every segment tare')


def read_probe_config():
    cfg = open(os.path.join(ROOT, 'core', 'M1', 'core.cfg')).read()
    sec = re.search(r'^\[load_cell_probe\]\n(.*?)(?=^\[)', cfg,
                    re.S | re.M).group(1)

    def get(name):
        return re.search(r'^%s:\s*([0-9.]+)' % name, sec, re.M).group(1)
    return (float(get('samples_tolerance')),
            int(get('samples_tolerance_retries')), int(get('samples')))


def test_4_logged_tap_pairs_pass_the_shipped_tolerance(mod_probe):
    tol, retries, samples = read_probe_config()
    assert (tol, retries, samples) == (0.075, 3, 2), (tol, retries, samples)
    # points of the logged mesh: pairs 0.047 apart pass at once
    for pair in ([0.1585, 0.2027], [0.1551, 0.2022],
                 [0.1650, 0.1259], [0.1736, 0.1272]):
        trace, outcome = rf.run(mod_probe, pair, samples, tol, retries)
        assert outcome.startswith('ok'), (pair, outcome)
        assert len(rf.descents(trace)) == 2, (pair, trace)
    # the same taps failed under the old 0.045 (0.0471 > 0.045)
    trace, outcome = rf.run(mod_probe, [0.1551, 0.2022] * 4, samples,
                            0.045, retries)
    assert 'samples_tolerance' in outcome, outcome
    # 0.09 apart is still a bad tap: the pair retries, the next pair is
    # accepted -> 4 descents
    trace, outcome = rf.run(mod_probe, [0.1000, 0.1900, 0.1551, 0.2022],
                            samples, tol, retries)
    assert outcome.startswith('ok') and len(rf.descents(trace)) == 4, (
        outcome, trace)
    # and if every pair is 0.09 apart the mesh still fails after the
    # retries (1 try + 3 retries = 8 taps)
    trace, outcome = rf.run(mod_probe, [0.1000, 0.1900] * 4, samples, tol,
                            retries)
    assert 'samples_tolerance' in outcome, outcome
    assert len(rf.descents(trace)) == 8, trace
    print('ok  4 shipped samples_tolerance %.3f: pairs 0.047 apart pass at '
          'once, 0.09 apart retry (and fail after %d retries)'
          % (tol, retries))


def main():
    mod = tg.load_module(tg._paths.patched(
        'klippy/extras/load_cell_probe.py'))
    probe = rf.load_module(rf._paths.patched('klippy/extras/probe.py'),
                           'hot_probe')
    tests = [(test_1_hot_g28_from_the_top, mod),
             (test_2_a_press_at_any_rate, mod),
             (test_3_wrong_way_push, mod),
             (test_4_logged_tap_pairs_pass_the_shipped_tolerance, probe),
             (test_4b_slow_press_across_segments, mod),
             (test_5_sign_flipped_incident_at_4mm_s, mod),
             (test_6_home_z_retries_an_early_false_touch, mod),
             (test_7_segments_and_short_moves, mod)]
    failed = []
    for t, m in tests:
        try:
            t(m)
        except AssertionError as e:
            failed.append(t.__name__)
            print('FAIL %s: %s' % (t.__name__, e))
    if failed:
        print('%d of %d hot load cell tests FAILED: %s'
              % (len(failed), len(tests), ', '.join(failed)))
        sys.exit(1)
    print('All %d hot load cell tests passed' % (len(tests),))


if __name__ == '__main__':
    main()
