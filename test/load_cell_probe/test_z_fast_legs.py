"""Long legs above a KNOWN plate height (KAN-4682).

A home that starts from a Z the contact guard vouches for passes
FAST_ABOVE to the probing move.  The legs that start above it may run longer
than z_move_leg_time as far as the drift measured since leg 1 allows
(ZLoadBackstop.fast_leg_time); no leg crosses FAST_ABOVE, so the last few mm
are the 1 s legs of KAN-4364 (#148) as before, and a hot cell never grows
past them.  This drives the real LoadCellProbingMove / ZLoadBackstop through
the simulated world of test_z_legs.py.

    python3 test/load_cell_probe/test_z_fast_legs.py

Needs numpy.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import test_z_legs as zl  # noqa: E402

zb, tg = zl.zb, zl.tg
SPEED = 15.
ABOVE = 15.


def run(mod, world, above=ABOVE, opts=None):
    g = zl.Gcmd(FAST_ABOVE=above) if above is not None else None
    m, ph = zl.make(mod, world, opts=opts, gcmd=g)
    err = None
    try:
        m.probing_move(g)
    except tg.CommandError as e:
        err = str(e)
    return m, ph, err


def known_world(real=140., **kw):
    # Z known: the commanded frame is the real one (off 0), the plate at 0
    return zl.World(real=real, **kw)


def leg_seconds(w):
    return [(a - b) / SPEED for a, b in w.segments]


def test_1_still_cell_grows_the_legs(mod):
    w = known_world(drift=zl.ramp(0.))
    m, ph, err = run(mod, w)
    assert err is None and ph.calls == [], (err, ph.calls)
    secs = leg_seconds(w)
    legacy = known_world(drift=zl.ramp(0.))
    zl.descend(mod, legacy)
    assert len(legacy.segments) >= 10, len(legacy.segments)
    assert len(w.segments) <= 6, (len(w.segments), secs)
    assert secs[0] <= 1.0 + 1e-6, secs
    assert max(secs) > 2.0, secs
    # no leg crosses FAST_ABOVE: one ends exactly on it
    ends = [round(b, 3) for a, b in w.segments]
    assert ABOVE in ends, ends
    for a, b in w.segments:
        assert not (a > ABOVE + 1e-6 and b < ABOVE - 1e-6), (a, b)
    # below it: 1 s legs, and the bed is found there
    below = [(a - b) / SPEED for a, b in w.segments if a <= ABOVE + 1e-6]
    assert below and max(below) <= 1.0 + 1e-6, below
    f = ph.trigger_load / ph.trigger_delta
    assert 0.6 < f <= 1.05, f
    print('ok  1 a still cell: %d legs against %d (%s s), ends on Z%g, the '
          'bed found at %.2f x delta' % (len(w.segments),
                                         len(legacy.segments),
                                         '/'.join('%.1f' % s for s in secs),
                                         ABOVE, f))


def test_2_a_hot_cell_keeps_the_one_second_legs(mod):
    # 48000 counts/s is Sloe's worst: the plan is under a second, so the
    # legs are #148's exactly.  At 30000 a leg may be a little over a
    # second: its allowance (0.5 x delta at twice the measured rate) is.
    for rate, longest in ((48000., 1.0), (30000., 1.25)):
        w = known_world(drift=zl.ramp(rate))
        m, ph, err = run(mod, w)
        assert err is None and ph.calls == [], (rate, err)
        secs = leg_seconds(w)
        assert max(secs[:-1]) <= longest + 1e-6, (rate, secs)
        assert len(w.segments) >= 9, (rate, len(w.segments))
        worst = max(ph.leg_dev[:-1]) / m._trigger_delta
        assert worst <= (0.4 if rate > 40000. else 0.5), (rate, worst)
    print('ok  2 drifting 48000 counts/s: every leg is 1 s, 30000: at most '
          '1.25 s; the legs stay within %.2f x delta of their tare' % worst)


def test_3_a_slow_drift_grows_only_as_far_as_it_allows(mod):
    out = []
    for rate in (5000., 10000., 20000.):
        w = known_world(drift=zl.ramp(rate))
        m, ph, err = run(mod, w)
        assert err is None and ph.calls == [], (rate, err)
        worst = max(ph.leg_dev[:-1]) / m._trigger_delta
        # budget 0.5 x delta of drift per leg, plus one tare's spread
        assert worst <= 0.5 + 11000. / m._trigger_delta + 0.02, (rate, worst)
        out.append((rate, len(w.segments), max(leg_seconds(w)), worst))
    assert out[0][1] <= out[1][1] <= out[2][1], out
    assert out[0][2] >= out[2][2], out
    print('ok  3 drift %s: legs %s, longest leg %s s, worst deviation %s x '
          'delta' % ('/'.join('%d' % o[0] for o in out),
                      '/'.join('%d' % o[1] for o in out),
                      '/'.join('%.1f' % o[2] for o in out),
                      '/'.join('%.2f' % o[3] for o in out)))


def test_4_without_fast_above_nothing_changes(mod):
    a = known_world(drift=zl.ramp(0.), noise=False)
    zl.descend(mod, a)
    b = known_world(drift=zl.ramp(0.), noise=False)
    run(mod, b, above=None)
    assert a.segments == b.segments, (a.segments, b.segments)
    c = known_world(drift=zl.ramp(0.), noise=False)
    run(mod, c, opts={'z_move_fast_leg_max': 0.})
    assert a.segments == c.segments
    print('ok  4 no FAST_ABOVE, or z_move_fast_leg_max 0: the same legs as '
          'before (%d)' % (len(a.segments),))


def test_5_an_obstacle_in_a_long_leg_stops_at_the_trigger(mod):
    # a part left on the plate, 60 mm tall, a soft top: found inside a long
    # leg of a still cell
    w = known_world(drift=zl.ramp(0.), stiff=1000000., soft=300000.,
                    soft_at=60.)
    m, ph, err = run(mod, w)
    assert err is None and ph.calls == [], (err, ph.calls)
    assert w.real > 55., w.real
    f = ph.trigger_load / ph.trigger_delta
    assert f <= 1.05, f
    longest = max(leg_seconds(w))
    assert longest > 2.0, leg_seconds(w)
    # the same part on a cell drifting the wrong way at a rate the plan
    # allows for: the trigger needs the drift on top, never more than the
    # planned budget (0.5 x delta) and one tare's spread
    rate = 10000.
    w = known_world(drift=zl.ramp(rate), stiff=1000000., soft=300000.,
                    soft_at=60.)
    m, ph, err = run(mod, w)
    assert err is None and ph.calls == [], (err, ph.calls)
    f2 = ph.trigger_load / ph.trigger_delta
    assert f2 <= 1.0 + 0.5 + 11000. / ph.trigger_delta + 0.05, f2
    print('ok  5 a 60 mm part on the plate: triggered at %.2f x delta on a '
          'still cell, %.2f x delta with 10000 counts/s the wrong way '
          '(limit 1.5 + spread)' % (f, f2))


def test_6_a_rate_that_jumps_inside_a_long_leg_is_stopped(mod):
    # still for 4 s (the long legs are planned), then 100000 counts/s the
    # wrong way: the wrong-way rule ends the move, at most 1 x delta + what
    # 20 ms of samples add, and the plate is never reached
    w = known_world(drift=zl.ramp(0., 100000., 4.), noise=False)
    m, ph, err = run(mod, w)
    assert err is not None and (
        err == 'Homing aborted: ' + zb.WRONG_WAY_REASON
        or 'first leg' in err), err
    assert w.real > 15., w.real
    # counted from the tare of the leg it happened in
    over = abs(w.counts() - m._tare_counts) / float(m._trigger_delta)
    assert over <= 1.0 + 100000. * 0.045 / m._trigger_delta + 0.02, over
    # and 100000 counts/s toward the press: the trigger fires early in the
    # air, far above the plate; the home treats it as a false touch
    w = known_world(drift=lambda s: -100000. * max(0., s - 4.), noise=False)
    m, ph, err = run(mod, w)
    assert err is None and w.real > 15., (err, w.real)
    print('ok  6 100000 counts/s the wrong way after 4 s: aborted at %.2f x '
          'delta of drift; toward the press: a trigger at Z%.0f, above the '
          'plate (a false touch for the home)' % (over, w.real))


def test_7_a_held_press_side_jump_gets_no_growth(mod):
    w = known_world(drift=zl.ramp(0.), noise=False)
    m, ph = zl.make(mod, w, opts=None, gcmd=zl.Gcmd(FAST_ABOVE=ABOVE))
    watch = m._z_watch
    watch.start_legs(-121734, 146400, 100.)
    watch._leg_jumps = 1
    assert watch.fast_leg_time(-121734, 101.3, 146400) == 1.0
    assert watch.last_fast_plan['why'] == 'press-side jump held'
    watch._leg_jumps = 0
    assert watch.fast_leg_time(-121734, 101.3, 146400) > 1.0
    watch.start_legs(-121734, 146400, None)
    assert watch.fast_leg_time(-121734, 101.3, 146400) == 1.0
    print('ok  7 a held jump, or no tare time: no growth')


def test_8_the_mcu_zero_ignores_fast_above(mod):
    w = known_world(drift=zl.ramp(0.), noise=False)
    g = zl.Gcmd(FAST_ABOVE=ABOVE, MCUTARE=1)
    m, ph = zl.make(mod, w, gcmd=g)
    m._approx_tare = w.tare0
    m._nozzle_is_clear = lambda: True
    try:
        m.probing_move(g)
    except tg.CommandError:
        pass
    secs = leg_seconds(w)
    assert max(secs[:-1] or [0]) <= 1.0 + 1e-6, secs
    print('ok  8 MCUTARE: legs stay 1 s')


def test_9_the_plan_is_in_the_status_and_the_log(mod):
    w = known_world(drift=zl.ramp(0.))
    with zb.LogCapture() as cap:
        m, ph, err = run(mod, w)
    lines = [x for lv, x in cap.lines if 'Z legs touched' in x]
    assert len(lines) == 1, cap.lines
    assert 'planned leg seconds [1.00,' in lines[0], lines[0]
    assert 'pauses' in lines[0], lines[0]
    st = m._z_watch.get_status()
    assert st['z_backstop_fast_leg_max'] == 4.0, st
    assert st['z_backstop_fast_drift_factor'] == 0.5, st
    try:
        zl.make(mod, w, opts={'z_move_fast_leg_max': 0.5})
    except tg.ConfigError:
        pass
    else:
        raise AssertionError('a fast leg max under the leg time was taken')
    print('ok  9 planned leg seconds and the pauses are logged, the limits '
          'are in the status, a maximum under the leg time is refused')


def main():
    mod = tg.load_module(tg._paths.patched(
        'klippy/extras/load_cell_probe.py'))
    tests = [test_1_still_cell_grows_the_legs,
             test_2_a_hot_cell_keeps_the_one_second_legs,
             test_3_a_slow_drift_grows_only_as_far_as_it_allows,
             test_4_without_fast_above_nothing_changes,
             test_5_an_obstacle_in_a_long_leg_stops_at_the_trigger,
             test_6_a_rate_that_jumps_inside_a_long_leg_is_stopped,
             test_7_a_held_press_side_jump_gets_no_growth,
             test_8_the_mcu_zero_ignores_fast_above,
             test_9_the_plan_is_in_the_status_and_the_log]
    failed = []
    for t in tests:
        try:
            t(mod)
        except AssertionError as e:
            failed.append(t.__name__)
            print('FAIL %s: %s' % (t.__name__, e))
    if failed:
        print('%d of %d fast leg tests FAILED: %s'
              % (len(failed), len(tests), ', '.join(failed)))
        sys.exit(1)
    print('All %d fast leg tests passed' % (len(tests),))


if __name__ == '__main__':
    main()
