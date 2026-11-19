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
ABOVE = 12.   # the home's fast_margin: a 1 s leg is 15 mm, so no tare on Z0


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
    m, ph, err = run(mod, w, opts={'z_move_fast_rest_time': 0.})
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
    assert 0.6 < f <= 1.1, f
    # no leg boundary (a tare at rest) within 2 mm of the plate at 0: a
    # light touch there would be taken into the next tare
    for a, b in w.segments[:-1]:
        assert abs(b) > 2., ('a leg ended at the plate', b)
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
    assert w.real >= ABOVE - 0.5, w.real
    # counted from the tare of the leg it happened in
    over = abs(w.counts() - m._tare_counts) / float(m._trigger_delta)
    assert over <= 1.0 + 100000. * 0.045 / m._trigger_delta + 0.02, over
    # and 100000 counts/s toward the press: the trigger fires early in the
    # air, far above the plate; the home treats it as a false touch
    w = known_world(drift=lambda s: -100000. * max(0., s - 4.), noise=False)
    m, ph, err = run(mod, w)
    assert err is None and w.real >= ABOVE - 0.5, (err, w.real)
    print('ok  6 100000 counts/s the wrong way after 4 s: aborted at %.2f x '
          'delta of drift; toward the press: a trigger at Z%.0f, above the '
          'plate (a false touch for the home)' % (over, w.real))


def heating(rate_at, ramp_s, peak=48000., t0=0.):
    # drift of a cell under a heating hot end: the rate grows linearly from
    # t0 to `peak` counts/s over ramp_s seconds and holds
    def drift(s):
        s = max(0., s - t0)
        if s < ramp_s:
            return 0.5 * peak * s * s / ramp_s
        return 0.5 * peak * ramp_s + peak * (s - ramp_s)
    return drift


def test_6b_a_home_at_print_start_while_the_hot_end_heats(mod):
    # Today's field case: G28 at print start with the hot end heating, Z
    # known, the drift rate rising from nothing to Sloe's 48000 counts/s
    # (wrong way) over 1 to 12 s. #148's 1 s legs finish each of these
    # homes; a long leg must not turn one into a wrong-way abort. The plan
    # follows the larger of the rate since leg 1 and the rate over the last
    # leg, and gives no growth to a rate that has risen 1.5x.
    rows = []
    for ramp in (1., 2., 4., 8., 12.):
        for onset in (0., 2., 5.):
            d = heating(None, ramp, t0=onset)
            legacy = known_world(drift=d)
            m0, ph0, err0 = zl.descend(mod, legacy)
            assert err0 is None, ('#148 itself aborted', ramp, onset, err0)
            w = known_world(drift=d)
            m, ph, err = run(mod, w)
            step = ramp <= 2. and onset > 0.
            if step:
                # (a 2 s ramp that starts inside a long leg is this case
                # too: 8 of 480 noisy homes end in the wrong-way abort,
                # a stop, never a press; #150's legs finished all 480)
                # a rate that goes from nothing to 48000 counts/s in a
                # second, in the middle of a long leg, is beyond what any
                # plan from measured drift can see: only the wrong-way
                # rule (1 x delta) is left, and it is. This is what the
                # heater gate of LOAD_CELL_HOME_Z is for (a heating hot end
                # never takes the fast path; test_load_cell_home_z).
                assert err is None or zb.WRONG_WAY_REASON in err, err
                step_worst = max(ph.leg_dev) / m._trigger_delta
                assert step_worst <= 1.1, (ramp, onset, step_worst)
                continue
            assert err is None and ph.calls == [], (ramp, onset, err)
            worst = max(ph.leg_dev[:-1]) / m._trigger_delta
            assert worst <= 0.85, (ramp, onset, worst)
            rows.append((ramp, onset, worst, len(w.segments),
                         max(leg_seconds(w))))
    print('ok  6b hot end heating (ramp 2..12 s, onset 0/2/5 s): every home '
          'finishes as #148 does, worst leg deviation %.2f x delta, longest '
          'leg %.1f s; a 1 s step in a long leg stays under 1.1 x delta '
          '(0.9 seen)' % (max(r[2] for r in rows), max(r[4] for r in rows)))


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
    # a FRESH host zero, so the tap really takes the MCU zero (KAN-4770,
    # #151: a stale one is replaced by a host tare, which may go fast)
    m._approx_tare_t = 1000.    # fresh: the fake clock reads 1000
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
        m, ph, err = run(mod, w, opts={'z_move_fast_rest_time': 0.})
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


def test_10_a_bed_removal_abort_in_a_long_leg_and_at_a_boundary(mod):
    # (a) in the middle of a long leg: the move ends with the reason, no
    # further leg runs, the plate is nowhere near
    w = known_world(drift=zl.ramp(0.), noise=False)
    m, ph = zl.make(mod, w, gcmd=zl.Gcmd(FAST_ABOVE=ABOVE),
                    opts={'z_move_fast_rest_time': 0.})
    fired = []
    counts = w.counts

    def counts_and_abort():
        # leg 1 is 1 s, leg 2 is 3 s: 6 s in is inside leg 3 (4 s)
        if not fired and w.t - 10. > 6.0:
            fired.append(w.t)
            ph.abort_active_move("the bed was removed")
        return counts()
    w.counts = counts_and_abort
    try:
        m.probing_move(zl.Gcmd(FAST_ABOVE=ABOVE))
    except tg.CommandError as e:
        assert str(e) == "Homing aborted: the bed was removed", str(e)
    else:
        raise AssertionError('the abort in a long leg was lost')
    secs = leg_seconds(w)
    assert len(w.segments) == 3 and 0. < secs[2] < 4., secs
    assert m._z_watch._leg_times[2] > 2., m._z_watch._leg_times
    assert w.real > ABOVE, w.real
    assert ph.active == [], ph.active
    leg_plan = m._z_watch._leg_times[2]
    # (b) at the boundary before the long leg 3: raised before it starts
    w = known_world(drift=zl.ramp(0.), noise=False)
    m, ph = zl.make(mod, w, gcmd=zl.Gcmd(FAST_ABOVE=ABOVE),
                    opts={'z_move_fast_rest_time': 0.})
    orig = m._measure_tare
    count = [0]

    def measure(gcmd):
        count[0] += 1
        if count[0] == 3:       # the pause before leg 3 (1: leg 1)
            assert ph.active, 'no stand-in registered'
            ph.abort_active_move("the bed was removed")
        return orig(gcmd)
    m._measure_tare = measure
    try:
        m.probing_move(zl.Gcmd(FAST_ABOVE=ABOVE))
    except tg.CommandError as e:
        assert str(e) == "Homing aborted: the bed was removed", str(e)
    else:
        raise AssertionError('the abort at a boundary was lost')
    assert len(w.segments) == 2, w.segments
    assert w.real > ABOVE and ph.active == [], (w.real, ph.active)
    print('ok  10 a bed removal in a long leg ends it (leg 3 planned %.1f s, '
          'ran %.1f s); at the boundary before it, no leg 3 runs'
          % (leg_plan, secs[2]))


def test_11_fast_above_is_clamped_to_5_mm(mod):
    # FAST_ABOVE=0 (or 2): the long legs still end 5 mm over the plate
    for above in (0., 2.):
        w = known_world(drift=zl.ramp(0.), noise=False)
        m, ph, err = run(mod, w, above=above,
                         opts={'z_move_fast_rest_time': 0.})
        assert err is None, err
        ends = [round(b, 2) for a, b in w.segments[:-1]]
        assert 5.0 in ends, (above, ends)
        for a, b in w.segments:
            assert not (a > 5. + 1e-6 and b < 5. - 1e-6), (above, a, b)
        assert max(leg_seconds(w)[-2:]) <= 1.0 + 1e-6, leg_seconds(w)
    print('ok  11 FAST_ABOVE below 5 mm is taken as 5: the legs end on Z5')


# ---- option B (KAN-4682): the drift is measured at rest before the descent,
# and no armed leg outlasts the ADS1220's temperature read -----------------

NO_REST = {'z_move_fast_rest_time': 0.}
# the rest is opt-in (default 0 until benched): the option-B tests turn it on
REST = {'z_move_fast_rest_time': 0.5}
# what #150 alone gave (noise off, 140 mm known): the golden legs
GOLDEN_STILL = [(140.0, 125.0), (125.0, 80.0), (80.0, 20.0), (20.0, 12.0),
                (12.0, -0.15)]
GOLDEN_HOT = [(140.0, 125.0), (125.0, 110.0), (110.0, 95.0), (95.0, 80.0),
              (80.0, 65.0), (65.0, 50.0), (50.0, 35.0), (35.0, 20.0),
              (20.0, 12.0), (12.0, -0.24)]


def with_ads(m, pause=4.0):
    # a muon_ads1220 load cell: it says how long a probe may stay armed
    m._load_cell.sensor.get_max_temperature_pause = lambda: pause


def run_ads(mod, world, pause=4.0, opts=None, above=ABOVE):
    g = zl.Gcmd(FAST_ABOVE=above)
    m, ph = zl.make(mod, world, opts=opts, gcmd=g)
    if pause is not None:
        with_ads(m, pause)
    err = None
    try:
        m.probing_move(g)
    except tg.CommandError as e:
        err = str(e)
    return m, ph, err


def r2(x):
    return [(round(a, 3), round(b, 3)) for a, b in x]


def test_12_rest_time_0_is_exactly_150(mod):
    for rate, gold in ((0., GOLDEN_STILL), (48000., GOLDEN_HOT)):
        w = known_world(drift=zl.ramp(rate), noise=False)
        m, ph, err = run(mod, w, opts=NO_REST)
        assert err is None and r2(w.segments) == gold, (rate, r2(w.segments))
        assert m._z_watch._rest is None
    print('ok  12 z_move_fast_rest_time 0: the legs of #150, exactly')


def test_13_the_rest_plans_leg_1(mod):
    firsts = []
    for rate in (0., 5000., 10000., 20000.):
        w = known_world(drift=zl.ramp(rate), noise=False)
        m, ph, err = run(mod, w, opts=REST)
        assert err is None and ph.calls == [], (rate, err)
        rest = m._z_watch._rest
        assert rest is not None and rest['span'] >= 0.4, rest
        assert abs(rest['rate_fit'] - rate) < 0.05 * rate + 50., (rate, rest)
        plan = m._z_watch._leg_plans[0]
        assert plan['why'].startswith('rest'), plan
        firsts.append(leg_seconds(w)[0])
    assert firsts[0] > 2.0 and firsts[0] != 1.0, firsts
    assert firsts == sorted(firsts, reverse=True), firsts
    print('ok  13 leg 1 is planned from the rest: %s s at 0/5k/10k/20k '
          'counts/s (was always 1.0)' % ('/'.join('%.2f' % f for f in firsts)))


def test_14_a_still_cell_leg_sequence(mod):
    w = known_world(drift=zl.ramp(0.), noise=False)
    m, ph, err = run(mod, w, opts=REST)
    secs = leg_seconds(w)
    assert err is None and len(secs) <= 5, (err, secs)
    assert secs[0] > 2.0, secs
    w2 = known_world(drift=zl.ramp(0.), noise=False)
    run_ads(mod, w2, opts=REST)
    secs2 = leg_seconds(w2)
    assert len(secs2) <= 5, secs2
    for a, b in w2.segments:
        assert not (a > ABOVE + 1e-6 and b < ABOVE - 1e-6), (a, b)
    assert secs2[-1] <= 1.0 + 1e-6, secs2
    # 4.0 s pause less the 0.5 s default margin
    assert secs2[0] > 2.0 and max(secs2) <= 3.5 + 1e-6, secs2
    # the shipped default (rest off, cap on): #150's legs with the cap
    w3 = known_world(drift=zl.ramp(0.), noise=False)
    run_ads(mod, w3)
    secs3 = leg_seconds(w3)
    assert secs3[0] <= 1.0 + 1e-6 and max(secs3) <= 3.5 + 1e-6, secs3
    print('ok  14 a still cell: legs %s s (no ADS1220), %s s (ADS1220 4 s '
          'pause); #150 alone: 1.0/3.0/4.0/0.53/1.0' % (
              '/'.join('%.2f' % s for s in secs),
              '/'.join('%.2f' % s for s in secs2)))


def test_15_a_hot_cell_keeps_one_second_legs_after_a_rest(mod):
    w = known_world(drift=zl.ramp(48000.))
    m, ph, err = run(mod, w, opts=REST)
    assert err is None and ph.calls == [], err
    secs = leg_seconds(w)
    assert max(secs[:-1]) <= 1.0 + 1e-6, secs
    assert len(secs) >= 9, secs
    worst = max(ph.leg_dev[:-1]) / m._trigger_delta
    assert worst <= 0.4, worst
    assert m._z_watch._leg_plans[0]['time'] == 1.0
    print('ok  15 48000 counts/s: every leg 1 s after the rest, worst '
          '%.2f x delta' % worst)


def test_16_a_drift_that_rises_in_the_rest_gets_no_growth(mod):
    # still for the first part of the rest, then 48000 counts/s
    w = known_world(drift=lambda s: 48000. * max(0., s - 0.45), noise=False)
    m, ph, err = run(mod, w, opts=REST)
    assert err is None, err
    secs = leg_seconds(w)
    assert max(secs[:-1]) <= 1.25, secs
    # the analysis itself flags an accelerating drift (quadratic) ...
    watch = m._z_watch
    n = 800
    t = [100. + i / 2000. for i in range(n)]
    c = [-121734 + 400000. * (x - 100.) ** 2 for x in t]
    info = watch.analyse_rest(99.9, -121734, t, c, 80)
    assert info['rising'], info
    flat = watch.analyse_rest(99.9, -121734, t, [-121734] * n, 80)
    assert not flat['rising'], flat
    # ... and a rising rest blocks growth for the whole descent
    watch.start_legs(-121734, 146400, 100.)
    watch.note_rest(info, -121734, 100.)
    assert watch.first_leg_time(146400) == 1.0
    assert watch._rest['why'] == 'drift rising in the rest'
    assert watch.fast_leg_time(-121734, 101.3, 146400) == 1.0
    print('ok  16 a drift that rises in the rest: leg 1 and the rest of the '
          'descent get no growth (longest leg %.2f s)' % max(secs[:-1]))


def test_17_a_press_side_jump_in_the_rest_gets_no_growth(mod):
    w = known_world(drift=zl.ramp(0.), noise=False, bump=(0.4, 0.2,
                                                           -250000.))
    m, ph, err = run(mod, w, opts=REST)
    assert err is None, err
    rest = m._z_watch._rest
    assert rest['jump'], rest
    # leg 1 armed on the tare the rest BEGAN with, not the pushed one
    assert rest['tare1'] == m._z_watch._leg_first_tare, rest
    secs = leg_seconds(w)
    assert max(secs[:-1]) <= 1.0 + 1e-6, secs
    # a load that stays: the held tare is what the trigger judges, so it
    # fires at once, in the air (a false touch for the home); taking the
    # pushed tare would have hidden the load and gone on down
    w = known_world(drift=zl.ramp(0.), noise=False, bump=(0.4, 100.,
                                                           -250000.))
    m, ph, err = run(mod, w, opts=REST)
    assert w.real > 135., (err, w.real)
    print('ok  17 a press-side step in the rest: the old tare is held, '
          'every leg 1 s; one that stays triggers at once, in the air '
          '(Z%.0f)' % w.real)


def test_17b_the_rest_plan_formula_and_its_noise_term(mod):
    w = known_world(drift=zl.ramp(0.), noise=False)
    m, ph = zl.make(mod, w, opts=REST, gcmd=zl.Gcmd(FAST_ABOVE=ABOVE))
    watch = m._z_watch
    delta = 146400.

    def plan(rate_fit, rate2, noise, span=0.5, dt2=0.25):
        watch.start_legs(-121734, delta, 100.)
        info = {'tare1': -121734, 't1': 99.5, 'tare2': -121734, 't2': 100.,
                'span': span, 'change': 0., 'rate_fit': rate_fit,
                'rate2': rate2, 'dt2': dt2, 'noise_obs': 0., 'noise': noise,
                'rising': False, 'jump': False, 'blocks': 10}
        watch.note_rest(info, -121734, 100.)
        use = watch.first_leg_time(delta)
        return use, watch.last_fast_plan
    # no drift at all: the noise over the window is the rate, and the leg
    # is 0.5 x delta / (2 x rate) (under the 3 x base cap)
    use, p = plan(0., 0., 8000., dt2=0.5)
    assert abs(p['rate'] - 8000. / 0.5) < 1e-6, p
    assert abs(use - min(3., 0.5 * delta / (2. * p['rate']))) < 1e-9, (use, p)
    # more noise, a shorter leg; the noise is never dropped
    use2, p2 = plan(0., 0., 16000., dt2=0.5)
    assert use2 < use and abs(use2 - 0.5 * delta / (2. * p2['rate'])) < 1e-9
    # a measured rate counts in full, from the whole window or the last half
    use3, p3 = plan(20000., 0., 1000., dt2=0.5)
    use4, p4 = plan(0., 20000., 1000., dt2=0.5)
    assert p3['rate'] >= 20000. and p4['rate'] >= 20000., (p3, p4)
    assert use3 < 2. and use4 < 2., (use3, use4)
    assert use < 3., use
    print('ok  17b leg 1 = 0.5 x delta / (2 x (rate + noise / window)): '
          'noise %.2f s, 2x noise %.2f s, 20000 counts/s %.2f s' % (
              use, use2, use3))


def test_18_no_armed_leg_outlasts_the_temperature_read(mod):
    # ADS1220: max_temperature_pause 4.0, margin 0.3 -> legs <= 3.7 s
    for pause, margin, cap in ((4.0, None, 3.7), (4.0, 1.0, 3.0),
                               (6.0, None, 4.0), (2.0, None, 1.7)):
        opts = dict(REST) if margin is None else dict(REST, z_move_fast_temp_margin=margin)
        w = known_world(drift=zl.ramp(0.), noise=False)
        m, ph, err = run_ads(mod, w, pause=pause, opts=opts)
        assert err is None, err
        secs = leg_seconds(w)
        assert max(secs) <= max(cap, 1.0) + 1e-6, (pause, margin, secs)
        for p in m._z_watch._leg_times:
            assert p <= max(cap, 1.0) + 1e-6, (pause, p)
    print('ok  18 every armed leg stays under max_temperature_pause - '
          'margin (3.7 s at 4.0 / 0.3)')


def test_19_no_muon_ads1220_no_cap(mod):
    w = known_world(drift=zl.ramp(0.), noise=False)
    m, ph, err = run(mod, w, opts=REST)
    assert err is None, err
    assert max(leg_seconds(w)) > 3.7 + 1e-6, leg_seconds(w)
    assert m._z_watch._temp_cap() is None
    print('ok  19 no muon_ads1220: no cap (longest leg %.2f s)'
          % max(leg_seconds(w)))


def test_20_the_log_has_the_rest_numbers(mod):
    w = known_world(drift=zl.ramp(0.))
    with zb.LogCapture() as cap:
        m, ph, err = run(mod, w, opts=REST)
    lines = [x for lv, x in cap.lines if 'Z legs touched' in x]
    assert len(lines) == 1, cap.lines
    assert 'rest ' in lines[0] and 'counts/s' in lines[0], lines[0]
    w = known_world(drift=zl.ramp(0.))
    with zb.LogCapture() as cap:
        run(mod, w, opts=NO_REST)
    lines = [x for lv, x in cap.lines if 'Z legs touched' in x]
    assert 'rest off' in lines[0], lines[0]
    print('ok  20 the Z legs line carries the rest-window numbers')


def main():
    mod = tg.load_module(tg._paths.patched(
        'klippy/extras/load_cell_probe.py'))
    tests = [test_1_still_cell_grows_the_legs,
             test_2_a_hot_cell_keeps_the_one_second_legs,
             test_3_a_slow_drift_grows_only_as_far_as_it_allows,
             test_4_without_fast_above_nothing_changes,
             test_5_an_obstacle_in_a_long_leg_stops_at_the_trigger,
             test_6_a_rate_that_jumps_inside_a_long_leg_is_stopped,
             test_6b_a_home_at_print_start_while_the_hot_end_heats,
             test_7_a_held_press_side_jump_gets_no_growth,
             test_8_the_mcu_zero_ignores_fast_above,
             test_9_the_plan_is_in_the_status_and_the_log,
             test_10_a_bed_removal_abort_in_a_long_leg_and_at_a_boundary,
             test_11_fast_above_is_clamped_to_5_mm,
             test_12_rest_time_0_is_exactly_150,
             test_13_the_rest_plans_leg_1,
             test_14_a_still_cell_leg_sequence,
             test_15_a_hot_cell_keeps_one_second_legs_after_a_rest,
             test_16_a_drift_that_rises_in_the_rest_gets_no_growth,
             test_17_a_press_side_jump_in_the_rest_gets_no_growth,
             test_17b_the_rest_plan_formula_and_its_noise_term,
             test_18_no_armed_leg_outlasts_the_temperature_read,
             test_19_no_muon_ads1220_no_cap,
             test_20_the_log_has_the_rest_numbers]
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
