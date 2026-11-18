"""Z probing descents in legs (KAN-4364, the 18 Nov design note).

A hot load cell drifts 1.7-2.4M counts over a heat-up (KAN-3818); Sloe's
G28 approach aborted on that drift ("the load cell moved the wrong way").
A probing move now runs as legs of at most z_move_leg_time (1.0 s); each
leg tares afresh at rest and arms the MCU trigger and the Z load backstop
on that fixed tare; check_leg bounds what a new tare may take in.

This drives the real LoadCellProbingMove.probing_move / _measure_tare /
_apply_tare and the real ZLoadBackstop through a simulated world: a nozzle
over a bed, the cell drifting, the MCU trigger ("gt" on the leg's tare)
checked on every sample and the host watch fed 20 ms batches.

    python3 test/load_cell_probe/test_z_legs.py

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
        self.leg_dev = []           # per leg: most |counts - tare|, no load
        self.collectors_stopped = 0

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
         gcmd=None):
    """The real LoadCellProbingMove (probing_move, _measure_tare,
    _apply_tare with its KAN-3370 checks) and the real ZLoadBackstop
    (opts: its config), in `world`."""
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
    m._start_collector = lambda: Collector(world, m, ph)
    m._z_watch = mod.ZLoadBackstop(tg.FakeConfig(printer, opts or {}),
                                   printer, cell)
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


def test_1_hot_ramps_do_not_abort(mod):
    # G28's first approach, 164 mm at 15 mm/s from the soft-homed top,
    # with the cell drifting the wrong way at 20000, 30000 or 48000
    # counts/s (Sloe's worst).  No abort; in every leg the load stays
    # within 0.4 x delta of that leg's tare; nothing is followed.
    for rate in (20000., 30000., 48000.):
        w = World(drift=ramp(rate), real=160.)
        w.off = 4.          # z_min -4 is real -8: the bed at 0 is above it
        m, ph, err = descend(mod, w)
        assert err is None and ph.calls == [], (rate, err)
        assert len(w.segments) >= 10, (rate, len(w.segments))
        worst = max(ph.leg_dev[:-1]) / m._trigger_delta
        assert worst <= 0.4, (rate, worst)
        assert m._z_watch._drift == 0., m._z_watch._drift
        f = ph.trigger_load / ph.trigger_delta
        assert 0.6 < f <= 1.0 + 48000. / ph.trigger_delta + 0.05, (rate, f)
    print('ok  1 164 mm at 15 mm/s drifting 20k/30k/48k counts/s the wrong '
          'way: no abort, %d legs, each within %.2f x delta of its tare'
          % (len(w.segments), worst))


def test_2_slow_and_fast_wrong_way_pushes(mod):
    # (a) a 40000 counts/s wrong-way push from the start, on top of 20000
    # counts/s of drift: under a delta per leg, so the cumulative limit
    # since leg 1 (4 x delta) ends it, the push then at most
    # 4 x delta + delta + 48000.
    w = World(drift=ramp(20000., 40000., 0.), real=160., noise=False)
    w.off = 4.
    m, ph, err = descend(mod, w)
    assert err is not None and zb.WRONG_WAY_REASON in err, err
    assert 'since the first leg' in err, err
    delta = m._trigger_delta
    push = 40000. * (w.t - 10.)
    assert push <= 5. * delta + 48000., (push, delta)
    assert m._z_watch.last_fired == zb.WRONG_WAY_REASON
    # (b) the same push cold with legs off (a single descent, as a
    # confirm tap is): the wrong-way rule at 1 x delta
    w = World(drift=ramp(0., 40000., 0.), real=150., noise=False)
    m, ph, err = descend(mod, w, opts={'z_move_leg_time': 0.})
    assert err == 'Homing aborted: ' + zb.WRONG_WAY_REASON, err
    # (counted from 0 s: the push during the 0.22 s tare pause and the
    # confirm and batch add ~0.07)
    f = 40000. * (w.t - 10.) / m._trigger_delta
    assert 1. <= f < 1.1, f
    single = f
    # ... and with legs on, cold, it is the cumulative limit again (a
    # 300 mm descent, so it gets there before the bed)
    w = World(drift=ramp(0., 40000., 0.), real=300., noise=False)
    w.off = 4.
    m, ph, err = descend(mod, w)
    assert err is not None and 'since the first leg' in err, err
    cold = 40000. * (w.t - 10.) / m._trigger_delta
    assert cold <= 5.4, cold
    # (c) 300000 counts/s: inside one leg, by 1.33 x delta (+ what the
    # 20 ms confirm and the 20 ms batch add at that rate)
    w = World(drift=ramp(20000., 300000., 3.), real=160., noise=False)
    w.off = 4.
    m, ph, err = descend(mod, w)
    assert err == 'Homing aborted: ' + zb.WRONG_WAY_REASON, err
    d = m._trigger_delta
    f = 300000. * (w.t - 10. - 3.) / d
    assert f <= 1.33 + 300000. * 0.042 / d, f
    print('ok  2 a 40k counts/s wrong-way push: on 20k drift stopped by the '
          'cumulative limit at %.2f x delta of push, cold with legs off at '
          '%.2f x, cold with legs %.2f x; 300k counts/s inside one leg at '
          '%.2f x' % (push / delta, single, cold, f))


def test_3_press_force_after_drift(mod):
    # 10 s of 20000 counts/s wrong-way drift, then the bed (1M counts/mm
    # at 15 mm/s): the trigger fires at no more than delta + 48000 of
    # real force.  Negative control, legs off (one descent, as master):
    # the stale tare needs the 200000 of drift on top.
    def run(opts):
        w = World(drift=ramp(20000.), real=150., noise=False)
        w.off = 4.
        m, ph, err = descend(mod, w, opts=opts)
        assert err is None, err
        return ph.trigger_load, ph.trigger_delta
    load, delta = run({})
    assert load <= delta + 48000., (load, delta)
    load0, delta0 = run({'z_move_leg_time': 0., 'z_move_wrong_way_factor':
                         0., 'z_move_abort_ceiling_factor': 0.})
    assert load0 >= delta0 + 190000., (load0, delta0)
    print('ok  3 after 10 s of 20k counts/s drift the trigger fires at '
          'delta + %i (legs off: delta + %i)' % (load - delta,
                                                 load0 - delta0))


def test_4_press_side_drift_no_trigger_in_air(mod):
    # Cooling: 48000 counts/s toward the press for the whole approach.
    # No trigger in the air; the bed is found at about one delta.
    w = World(drift=ramp(-48000.), real=160.)
    w.off = 4.
    m, ph, err = descend(mod, w)
    assert err is None and abs(w.real) < 0.5, (err, w.real)
    f = ph.trigger_load / ph.trigger_delta
    assert 0.5 < f < 1.1, f
    print('ok  4 48k counts/s toward the press: no trigger in the air, the '
          'bed at %.2f x delta' % f)


def test_5_leg_boundary_in_light_contact(mod):
    # A leg ends pressing 0.6 x delta (100000 counts/mm: 0.88 mm in).  The
    # next tare jumps toward the press by more than 50000 x dt + 0.1
    # delta, so the previous tare is kept, and the trigger fires at no
    # more than delta + 2 x 48000.
    found = None
    for drift in (None, ramp(48000.)):
        w = World(drift=drift, stiff=100000., noise=False)
        w.real = 15. * 3 - 0.88   # leg 3 ends 0.88 mm into the bed
        w.off = 160. - w.real
        with zb.LogCapture() as cap:
            m, ph, err = descend(mod, w)
        assert err is None, err
        kept = [x for lv, x in cap.lines if 'keeping' in x]
        # cold the jump is held; on 48000 counts/s of wrong-way drift the
        # leg's drift offsets most of it, and the force bound still holds
        assert len(kept) == (1 if drift is None else 0), cap.lines
        load = ph.trigger_load
        assert load <= ph.trigger_delta + 2 * 48000., (load,
                                                       ph.trigger_delta)
        found = load / ph.trigger_delta
    # two such jumps in a row end the move
    w, cell, homing = zb.make_watch(mod)
    w.start_legs(-121734)
    w.check_leg(-121734 - 30000, 1.15, -121734, 146400, PRESS)
    assert w.check_leg(-121734 - 150000, 1.15, -121734, 146400,
                       PRESS) == -121734
    try:
        w.check_leg(-121734 - 300000, 1.15, -121734, 146400, PRESS)
    except tg.CommandError as e:
        assert 'load on the nozzle between Z legs' in str(e), str(e)
    else:
        raise AssertionError('a second jump in a row did not raise')
    # a jump, then a normal leg, then a jump: not in a row
    w.start_legs(0)
    assert w.check_leg(-150000, 1.15, 0, 146400, PRESS) == 0
    assert w.check_leg(-20000, 1.15, 0, 146400, PRESS) == -20000
    assert w.check_leg(-170000, 1.15, -20000, 146400, PRESS) == -20000
    print('ok  5 a leg ending 0.6 x delta into the bed: previous tare kept, '
          'trigger at %.2f x delta; two jumps in a row raise' % found)


def test_6_collectors_and_aborts(mod):
    # Every leg's collector is stopped but the last (the tap's); an abort
    # in a later leg is raised, never taken for "No trigger".
    w = World(real=60., stiff=1000000.)
    w.off = 100.
    m, ph = make(mod, w)
    m.probing_move(None)
    legs = len(w.segments)
    assert legs > 3 and ph.collectors_stopped == legs - 1, (
        legs, ph.collectors_stopped)
    # a wrong-way step during a later leg's move
    w = World(real=160., noise=False,
              bump=(2.8, 100., 2. * 146400.))
    w.off = 4.
    m, ph, err = descend(mod, w)
    assert err == 'Homing aborted: ' + zb.WRONG_WAY_REASON, err
    assert len(w.segments) >= 3, w.segments
    assert ph.collectors_stopped == len(w.segments), ph.collectors_stopped
    # no contact at all: "No trigger" from the last leg only
    w = World(real=40., stiff=0.)
    m, ph, err = descend(mod, w)
    assert err == "No trigger on probe after full movement", err
    assert abs(w.z + 4.) < 1e-6, w.z
    # max_search_distance_z bounds the whole descent, with its message
    w = World(real=150., stiff=0.)
    m, ph = make(mod, w)
    ph.max_search_distance_z = 50.
    try:
        m.probing_move(None)
    except tg.CommandError as e:
        assert 'no contact within 50 mm' in str(e), str(e)
    else:
        raise AssertionError('the search cap did not stop it')
    assert abs(w.real - 100.) < 1e-6 and len(w.segments) == 4, (
        w.real, w.segments)
    print('ok  6 every intermediate collector stopped; a later-leg abort is '
          'raised; "No trigger" only from the last leg')


def test_7_short_descents_unchanged(mod):
    # A tap, a confirm tap, a mesh point: touch inside the first leg ->
    # one tare, one arm, with or without MCUTARE.
    for mcutare in (0, 1):
        w = World(real=1.0, noise=False)
        m, ph = make(mod, w, speed=5.)
        arms = []
        orig = m._z_watch.arm

        def arm(*a, **k):
            arms.append(k.get('mcutare'))
            return orig(*a, **k)
        m._z_watch.arm = arm
        tares = []
        orig_m = m._measure_tare

        def measure(gcmd):
            r = orig_m(gcmd)
            tares.append(r[0])
            return r
        m._measure_tare = measure
        if mcutare:
            m._approx_tare = w.tare0
            m._nozzle_is_clear = lambda: True
            m._mcu_tare = lambda gcmd: m._pause_and_tare(gcmd)
        m.probing_move(Gcmd(MCUTARE=mcutare))
        assert len(arms) == 1 and len(tares) == 1, (arms, tares)
        assert len(w.segments) == 1
    print('ok  7 a tap touching in its first leg: one tare, one arm, with '
          'and without MCUTARE')


def test_8_legs_off_is_today_with_the_follower_capped(mod):
    # z_move_leg_time 0: one descent.  With the #147 follower switched on
    # (z_move_drift_rate_counts 50000) what it absorbs is capped at 0.5 x
    # delta, so 20000 counts/s of drift trips the wrong-way rule once it
    # passes 1.5 x delta (~11 s), not never.
    w = World(drift=ramp(20000.), real=300., noise=False, stiff=0.)
    w.off = 4.
    m, ph, err = descend(mod, w, opts={'z_move_leg_time': 0.,
                                       'z_move_drift_rate_counts': 50000.})
    assert err == 'Homing aborted: ' + zb.WRONG_WAY_REASON, err
    assert len(w.segments) == 1
    d = m._trigger_delta
    at = (w.t - 10.) * 20000. / d
    assert 1.5 <= at < 1.6, at
    assert m._z_watch._drift <= 0.5 * d + 1, m._z_watch._drift
    # and with the follower off (the default) at 1 x delta
    w = World(drift=ramp(20000.), real=300., noise=False, stiff=0.)
    w.off = 4.
    m, ph, err = descend(mod, w, opts={'z_move_leg_time': 0.})
    at0 = (w.t - 10.) * 20000. / m._trigger_delta
    assert err == 'Homing aborted: ' + zb.WRONG_WAY_REASON and \
        1. <= at0 < 1.05, (err, at0)
    print('ok  8 legs off: one descent; the follower, if on, is capped at '
          '0.5 x delta (trips at %.2f x), off trips at %.2f x' % (at, at0))


def test_9_note_baseline_does_not_ratchet(mod):
    # KAN-4364: a hot mesh whose drift once outran the 100000 bound used
    # to keep a stale _approx_tare for good; now the bound grows at
    # z_move_leg_rate_counts a second since the last accepted one.
    m, ph = make(mod, World())
    m._approx_tare, m._approx_tare_t = 0, 10.
    m.note_baseline([[11., 0., 150000]] * 64)      # 1 s: 150000 > 100000
    assert m._approx_tare == 0
    m.note_baseline([[14., 0., 160000]] * 64)      # 4 s: 200000 allowed
    assert m._approx_tare == 160000 and m._approx_tare_t == 14.
    print('ok  9 note_baseline accepts a change of up to 50000 counts/s '
          'since the last accepted baseline')


def main():
    mod = tg.load_module(tg._paths.patched(
        'klippy/extras/load_cell_probe.py'))
    tests = [test_1_hot_ramps_do_not_abort,
             test_2_slow_and_fast_wrong_way_pushes,
             test_3_press_force_after_drift,
             test_4_press_side_drift_no_trigger_in_air,
             test_5_leg_boundary_in_light_contact,
             test_6_collectors_and_aborts,
             test_7_short_descents_unchanged,
             test_8_legs_off_is_today_with_the_follower_capped,
             test_9_note_baseline_does_not_ratchet]
    failed = []
    for t in tests:
        try:
            t(mod)
        except AssertionError as e:
            failed.append(t.__name__)
            print('FAIL %s: %s' % (t.__name__, e))
    if failed:
        print('%d of %d Z leg tests FAILED: %s'
              % (len(failed), len(tests), ', '.join(failed)))
        sys.exit(1)
    print('All %d Z leg tests passed' % (len(tests),))


if __name__ == '__main__':
    main()
