"""Z probing descents: one move, one tare (2026-11-24).

The Z home and every probe tap is a single move armed on the tare taken at
rest just before it.  The KAN-4364/4682 legs that replaced it for a while
were removed: the 1.7-2.4M count "hot drift" they rode out matched what
otter showed with something pressing on the hot end as it heated (foam, a
rubbing sock, screw heads on the heatsink; Airtable #293/#296/#297), and
with that fixed the zero drifts under 1000 counts/s over a descent.

This drives the real LoadCellProbingMove.probing_move / _measure_tare /
_apply_tare and the real ZLoadBackstop through a simulated world: a nozzle
over a bed, the cell drifting, the MCU trigger ("gt" on the tare) checked
on every sample and the host watch fed 20 ms batches.  The numbers are
otter's (B1, 23-24 Nov 2026): trigger_counts 120000, fast_speed 10,
reference_max_load_counts -1620691.

    python3 test/load_cell_probe/test_z_descent.py

Needs numpy.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import test_z_load_backstop as zb  # noqa: E402

tg = zb.tg
SPS = zb.SPS
PRESS = -1.         # a press lowers the counts, as in the backstop tests
TARE_1 = -121734    # Sloe's logged tare, delta 146400
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

    def __init__(self, world, m, ph=None):
        self.w = world
        self.ph = ph
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
        if self.ph is not None:
            self.ph.collectors_stopped += 1


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
        self.leg_dev = []           # per move: most |counts - tare|, no load
        self.active = []
        self.collectors_stopped = 0

    def note_move_active(self, hmove, active):
        if active:
            self.active.append(hmove)
        elif hmove in self.active:
            self.active.remove(hmove)

    def abort_active_move(self, reason, z_only=True):
        # as PrinterHoming: every registered move is told; the simulated
        # move itself stops on self.calls
        self.calls.append(reason)
        for h in list(self.active):
            h.abort(reason)
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
        dev = 0.
        try:
            while True:
                w.real = max(target, w.real - speed / float(SPS))
                w.t += 1. / SPS
                c = w.counts()
                if not w.load(w.real):
                    dev = max(dev, abs(c - zero))
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
            self.leg_dev.append(dev)


class ProbeParams(object):
    def __init__(self, speed):
        self.speed = speed

    def get_probe_params(self, gcmd):
        if gcmd is not None:
            return {'probe_speed': gcmd.get_float('PROBE_SPEED',
                                                  self.speed)}
        return {'probe_speed': self.speed}


def make(mod, world, speed=15., mcu=True, ref=SLOE_REF, opts=None,
         gcmd=None, trigger=None):
    """The real LoadCellProbingMove (probing_move, _measure_tare,
    _apply_tare with its KAN-3370 checks) and the real ZLoadBackstop
    (opts: its config), in `world`.  trigger: the [load_cell_probe]
    trigger options, trigger_percent 10 unless given."""
    helper = tg.make_helper(mod, ref,
                            **(trigger or {'trigger_percent': 10}))
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
    m._start_collector = lambda: Collector(world, m, ph)
    m.watch_cfg = tg.FakeConfig(printer, opts or {})
    m._z_watch = mod.ZLoadBackstop(m.watch_cfg, printer, cell)
    return m, ph


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


def descend(mod, world, **kw):
    """One PROBE: returns (move, phoming, error or None)."""
    m, ph = make(mod, world, **kw)
    err = None
    try:
        m.probing_move(kw.get('gcmd'))
    except tg.CommandError as e:
        err = str(e)
    return m, ph, err


def ramp(rate, push=0., push_at=0.):
    return lambda s: rate * s + push * max(0., s - push_at)



OTTER_REF = -1620691
M1 = {'trigger_counts': 120000}
TRIG = 120000


def m1_home(mod, world, **kw):
    """G28's descent as the M1 runs it: from the soft-homed top (real 160,
    commanded 164 above z_min) at fast_speed 10 with trigger_counts."""
    world.off = 4.
    kw.setdefault('trigger', M1)
    kw.setdefault('ref', OTTER_REF)
    kw.setdefault('speed', 10.)
    return descend(mod, world, **kw)


def test_1_clean_home_is_one_move(mod):
    # Cold, and hot with nothing touching the hot end: otter's zero drifts
    # at most ~1000 counts/s either way over a descent.  One move, one
    # tare, the trigger at 120000 whatever the tare.
    for tare0, rate in ((-90000, 0.), (180000, 1000.), (180000, -1000.)):
        w = World(tare0=tare0, drift=ramp(rate), real=160.)
        m, ph, err = m1_home(mod, w)
        assert err is None and ph.calls == [], (tare0, rate, err)
        assert len(w.segments) == 1, w.segments
        assert m._trigger_delta == TRIG, m._trigger_delta
        assert m._trigger_mode == 'counts', m._trigger_mode
        f = ph.trigger_load / TRIG
        assert 0.8 < f < 1.25, (rate, f)
        # the simulated bed gives 1000000 counts/mm: 120000 is 0.12 mm
        assert -0.15 < w.real < 0., w.real
    print('ok  1 164 mm at 10 mm/s, cold and hot (+-1000 counts/s): one move,'
          ' triggered at %.2f x 120000 of bed load' % (f,))


def test_2_trigger_does_not_follow_the_tare(mod):
    # trigger_percent 10 armed 10% of |anchor - tare|: a tare that moves
    # with heat moves the trigger.  trigger_counts does not.
    deltas = {}
    for name, trig in (('percent', {'trigger_percent': 10}), ('counts', M1)):
        for tare0 in (-90000, 180000, 2700000):
            w = World(tare0=tare0, real=10., noise=False)
            m, ph, err = descend(mod, w, trigger=trig, ref=OTTER_REF,
                                 speed=10.)
            assert err is None, (name, tare0, err)
            deltas[(name, tare0)] = m._trigger_delta
    assert deltas[('counts', -90000)] == deltas[('counts', 2700000)] == TRIG
    assert deltas[('percent', 2700000)] > 2 * deltas[('percent', -90000)]
    print('ok  2 the trigger is 120000 at tares -90k, +180k and +2.7M; with '
          'trigger_percent 10 it was %d, %d and %d'
          % tuple(deltas[('percent', t)] for t in (-90000, 180000, 2700000)))


def test_3_hot_end_pressed_by_something_is_stopped(mod):
    # otter with the unshaved sock: the zero rose 16000 counts/s just
    # after heating.  The wrong-way rule (1 x 120000) ends the home after
    # about 7.5 s, as it did on the printer (8.5 s), with its own message.
    w = World(tare0=2700000, drift=ramp(16000.), real=160., noise=False)
    m, ph, err = m1_home(mod, w)
    assert err == 'Homing aborted: ' + zb.WRONG_WAY_REASON, err
    secs = w.t - 10.
    assert 7. < secs < 9., secs
    assert len(w.segments) == 1
    print('ok  3 a hot end pressed on at 16000 counts/s: "%s" after %.1f s'
          % (zb.WRONG_WAY_REASON, secs))


def test_4_cooling_drift_toward_the_press_is_no_touch(mod):
    # a zero drifting toward the press at 3000 counts/s (three times
    # otter's worst) for the whole 16 s descent: 48000 of the 120000
    # trigger, so the first touch is the bed.
    w = World(tare0=180000, drift=ramp(-3000.), real=160.)
    m, ph, err = m1_home(mod, w)
    # on the bed (real <= 0), never in the air; the drift took some of the
    # 120000, so the bed load at the trigger is less than that
    assert err is None and -0.15 < w.real <= 0., (err, w.real)
    print('ok  4 3000 counts/s toward the press for 16 s: no early trigger'
          ' (bed load at the trigger %.0f)' % (ph.trigger_load,))


def test_5_removed_options_load_and_change_nothing(mod):
    # a dev-mode config written before the removal still loads: each old
    # option is accepted, flagged deprecated and ignored
    opts = {'z_move_leg_time': 1.0, 'z_move_leg_rate_counts': 50000,
            'z_move_fast_leg_max': 4.0, 'z_move_fast_rest_time': 0.5,
            'z_move_drift_rate_counts': 0}
    w = World(tare0=180000, real=160.)
    m, ph, err = m1_home(mod, w, opts=opts)
    assert err is None and len(w.segments) == 1, (err, w.segments)
    assert sorted(m.watch_cfg.deprecated) == sorted(opts), \
        m.watch_cfg.deprecated
    st = m._z_watch.get_status()
    assert not [k for k in st if 'leg' in k or 'fast' in k], st
    print('ok  5 z_move_leg_* / z_move_fast_* / z_move_drift_* load as '
          'deprecated and the descent is still one move')


def test_6_dead_mcu_trigger_is_stopped_by_the_ceiling(mod):
    # the MCU trigger never fires: the host ceiling (2.5 x 120000) ends
    # the move a fraction of a mm past contact
    w = World(tare0=180000, real=160.)
    m, ph, err = m1_home(mod, w, mcu=False)
    assert err == 'Homing aborted: ' + zb.CEILING_REASON, err
    # 0.3 mm to the ceiling, then 20 ms of confirm and a 20 ms batch at
    # 10 mm/s
    assert -0.8 < w.real < -0.3, w.real
    print('ok  6 no MCU trigger: the ceiling stops it %.3f mm past contact'
          % (-w.real,))


def test_7_no_bed_is_no_trigger(mod):
    # nothing under the nozzle over the whole travel: "No trigger", which
    # load_cell_home_z reads as a false first touch only on the confirm
    w = World(tare0=180000, real=160., stiff=0.)
    m, ph, err = m1_home(mod, w)
    assert err is not None and 'No trigger' in err, err
    assert len(w.segments) == 1
    print('ok  7 no bed in reach: "No trigger" after one full move')


def test_8_early_trigger_is_retried_with_a_host_tare(mod):
    # PRINT_START's fast mesh: the MCU zero was taken while the toolhead
    # rang after a 400 mm/s move, so the probe tripped before Z moved.
    # early_trigger_retries lifts, takes a host tare and tries again.
    w = World(tare0=180000, real=10.)
    m, ph = make(mod, w, trigger=M1, ref=OTTER_REF, speed=4.)
    m._early_trigger_retries = 3
    lifts = []
    th = m._printer.objects['toolhead']
    th.manual_move = lambda pos, speed: lifts.append(pos[2])
    th.wait_moves = lambda: None
    th.get_status = lambda et: {'axis_maximum': [200., 174., 160.]}
    m._param_helper.get_probe_params = lambda gcmd=None: {
        'probe_speed': 4., 'lift_speed': 10.}
    m._printer.objects['gcode'] = type('G', (), {
        'respond_info': lambda self, msg: None})()
    m._printer.get_reactor = lambda: type('R', (), {
        'monotonic': lambda self: 0.})()
    m._printer.is_shutdown = lambda: False
    real, calls = ph.probing_move, []

    def flaky(endstop, pos, speed):
        calls.append(1)
        if len(calls) < 3:
            raise tg.CommandError("Probe triggered prior to movement")
        return real(endstop, pos, speed)
    ph.probing_move = flaky
    m.probing_move(Gcmd(MCUTARE=0))
    assert len(calls) == 3 and m._early_triggers == 2, (calls, m._early_triggers)
    assert len(lifts) == 2, lifts
    assert -0.15 < w.real < 0., w.real
    # past the limit the error stands
    w = World(tare0=180000, real=10.)
    m, ph = make(mod, w, trigger=M1, ref=OTTER_REF, speed=4.)
    m._early_trigger_retries = 0

    def always(endstop, pos, speed):
        raise tg.CommandError("Probe triggered prior to movement")
    ph.probing_move = always
    try:
        m.probing_move(Gcmd())
        assert False, 'no error'
    except tg.CommandError as e:
        assert 'prior to movement' in str(e), e
    print('ok  8 a trip before the move is retried with a lift and a host '
          'tare; with retries 0 the error stands')


def main():
    mod = tg.load_module(tg._paths.patched(
        'klippy/extras/load_cell_probe.py'))
    tests = [test_1_clean_home_is_one_move,
             test_2_trigger_does_not_follow_the_tare,
             test_3_hot_end_pressed_by_something_is_stopped,
             test_4_cooling_drift_toward_the_press_is_no_touch,
             test_5_removed_options_load_and_change_nothing,
             test_6_dead_mcu_trigger_is_stopped_by_the_ceiling,
             test_7_no_bed_is_no_trigger,
             test_8_early_trigger_is_retried_with_a_host_tare]
    failed = []
    for t in tests:
        try:
            t(mod)
        except AssertionError as e:
            failed.append(t.__name__)
            print('FAIL %s: %s' % (t.__name__, e))
    if failed:
        print('%d of %d Z descent tests FAILED: %s'
              % (len(failed), len(tests), ', '.join(failed)))
        sys.exit(1)
    print('All %d Z descent tests passed' % (len(tests),))


if __name__ == '__main__':
    main()
