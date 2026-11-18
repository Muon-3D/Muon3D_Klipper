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

This drives the real ZLoadBackstop with those numbers (2000 sps, press
direction -1 as in the other backstop tests), and the real
ProbeSessionHelper.run_probe with the samples_tolerance read from
core/M1/core.cfg.

KAN-4364 on #148 (two reviews): drift is followed only on a long blind
descent, only in the sign of the slope measured over 0.5 s before it, at
no more than 1.5 x that slope; the MCU trigger's zero moves with it.
run_mcu() models the MCU trigger beside the real watch: it fires on
(counts - zero) x press > delta, zero being the last one the host sent,
applied 20 ms after the batch that produced it.  Tests 5-10 are the
review's cases: slow pushes, a 110000 counts/s ramp, a soft surface, the
sign-flipped incident at 4 mm/s, a burst, and the excess bound.
"""
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
DRIFT_EXCESS_REASON = ("the load cell baseline moved further than its"
                       " measured drift during the Z move")


def drift_counts(tare, rate, seconds, step=None, step_at=None):
    """Counts that drift up (the wrong way) by rate counts/s, with an
    optional step (counts) added from step_at seconds on."""
    n = int(seconds * SPS)
    out = []
    for i in range(n):
        c = tare + rate * i / float(SPS)
        if step is not None and i >= int(step_at * SPS):
            c += step
        out.append(int(c))
    return out


SLOE_RATE = 1100000. / 23.     # Sloe's heat-up drift, counts/s


def run_watch(mod, tare, delta, counts, slope=0., **opts):
    """The real watch on one descent, judged in 20 ms batches.  slope is
    the drift measured before the descent (0: a tap, nothing followed)."""
    w, cell, homing = zb.make_watch(mod, **opts)
    t0 = 10.
    w.arm(t0, tare, delta, PRESS, drift_slope=slope, drift_t0=t0)
    idx, _ = zb.feed_until_abort(cell, homing, t0, counts)
    return w, homing, idx


# Sloe's idle wander (test_z_load_backstop test 2), as a fixed sequence
def wander(n, seed=4364):
    out = []
    for _ in range(n):
        seed = (seed * 1103515245 + 12345) % (1 << 31)
        out.append(-4000 + seed % 11001)
    return out


def descent(drift_rate, seconds, press_rate=None, press_at=None,
            wrong=False, tare=TARE_1, noise=True, burst=None):
    """A descent's samples as (counts, drift, load): the cell drifts by
    drift_rate counts/s (positive is the wrong way) from tare, and from
    press_at seconds on the nozzle is pressed at press_rate counts/s
    (wrong=True: a sign error, the press reads the wrong way).  load is
    the press alone, in counts toward the press side.  burst=(at, secs,
    counts) adds counts (signed) for secs."""
    n = int(seconds * SPS)
    nz = wander(n) if noise else [0] * n
    out = []
    for i in range(n):
        d = drift_rate * i / float(SPS)
        load = 0.
        if press_rate is not None and i >= int(press_at * SPS):
            load = press_rate * (i - int(press_at * SPS)) / float(SPS)
        sign = -PRESS if wrong else PRESS
        c = tare + d + sign * load + nz[i]
        if burst is not None and burst[0] <= i / float(SPS) < (
                burst[0] + burst[1]):
            c += burst[2]
        out.append((int(c), d, load))
    return out


def run_mcu(mod, delta, samples, tare=TARE_1, mcu=True, retare=True,
            latency=40, batch=40, mcutare=False, direction=PRESS, slope=0.,
            **opts):
    """The real ZLoadBackstop with a model of the MCU trigger beside it.
    The MCU sees every sample as it is converted and triggers ("gt") on
    the first with (counts - zero) x press > delta, zero being the last
    one the host sent; a zero the host sends after a 20 ms batch reaches
    the MCU `latency` samples after that batch (40 = another 20 ms).
    slope is the drift measured before the descent.
    Returns a dict: what ended the move and at which sample."""
    w, cell, homing = zb.make_watch(mod, **opts)
    sent = []
    w.arm(10., tare, delta, direction, mcutare=mcutare,
          retare=sent.append if retare else None, drift_slope=slope,
          drift_t0=10.)
    zero = tare
    pending = []
    rs = zb.rows(10., [s[0] for s in samples])
    res = {'watch': w, 'sent': sent, 'trigger': None, 'abort': None,
           'idx': None}
    for b in range(0, len(rs), batch):
        chunk = rs[b:b + batch]
        for k, r in enumerate(chunk):
            i = b + k
            while pending and pending[0][0] <= i:
                zero = pending.pop(0)[1]
            if mcu and (r[2] - zero) * PRESS > delta:
                res['trigger'], res['idx'] = i, i
                res['zero'] = zero
                return res
        n = len(sent)
        cell.feed(chunk)
        if homing.calls:
            res['abort'], res['idx'] = homing.calls[0], b + len(chunk) - 1
            return res
        for z in sent[n:]:
            pending.append((b + len(chunk) + latency, z))
    return res


def force_at(s, r):
    return s[r['idx']][2] / float(DELTA_1)


def cross(s, level, start=0):
    return next(i for i in range(start, len(s)) if s[i][2] > level)


def test_1_measured_drift_is_followed(mod):
    # Logged attempt 1 (tare -121734, wrong way 146400, ceiling 366000):
    # Sloe's heat-up, 1.1M counts in 23 s, measured before the descent:
    # past both limits, never trips them.  Logged attempt 2 (tare
    # 1006163): 30000 counts/s for 12 s.
    w, homing, idx = run_watch(mod, TARE_1, DELTA_1,
                               drift_counts(TARE_1, SLOE_RATE, 23.),
                               slope=SLOE_RATE)
    assert w.get_limits() == (366000, 146400), w.get_limits()
    assert homing.calls == [] and idx is None, (homing.calls, idx)
    w, homing, idx = run_watch(mod, TARE_2, DELTA_2,
                               drift_counts(TARE_2, 30000., 12.),
                               slope=30000.)
    assert homing.calls == [] and idx is None, (homing.calls, idx)
    # the same drift with nothing measured (every tap, mesh point and
    # confirm tap) or with the rate at 0 is judged from the fixed tare
    for slope, opts in ((0., {}), (SLOE_RATE,
                                   {'z_move_drift_rate_counts': 0})):
        w, homing, idx = run_watch(mod, TARE_1, DELTA_1,
                                   drift_counts(TARE_1, SLOE_RATE, 23.),
                                   slope=slope, **opts)
        assert homing.calls == [zb.WRONG_WAY_REASON], homing.calls
        assert abs(idx - DELTA_1 / SLOE_RATE * SPS) < 100, idx
    # a slope under 3000 counts/s is not followed
    w, homing, idx = run_watch(mod, TARE_1, DELTA_1, [TARE_1] * 100,
                               slope=2999.)
    assert w._slope == 0. and w._band == 0., (w._slope, w._band)
    print('ok  1 Sloe 1.1M counts in 23 s, measured, never trips; with '
          'nothing measured or rate 0 it trips at 1 x delta (3.06 s)')


def test_2_a_real_wrong_way_event_still_trips(mod):
    # A step of 1.2 x the delta on a cell measured and drifting 20000
    # counts/s: judged at full size, 40 samples in.
    counts = drift_counts(TARE_1, 20000., 10., step=int(1.2 * DELTA_1),
                          step_at=5.)
    w, homing, idx = run_watch(mod, TARE_1, DELTA_1, counts, slope=20000.)
    assert homing.calls == [zb.WRONG_WAY_REASON], homing.calls
    assert 10000 <= idx <= 10000 + 41, idx
    # a ramp of 300000 counts/s on that cell trips inside a second
    w, homing, idx = run_watch(mod, TARE_1, DELTA_1,
                               drift_counts(TARE_1, 300000., 3.),
                               slope=20000.)
    assert homing.calls and idx < 0.6 * SPS, (homing.calls, idx)
    # a step of 1.1 x 146400 on a cold cell
    counts = [TARE_1] * 100 + [TARE_1 + int(1.1 * DELTA_1)] * 400
    w, homing, idx = run_watch(mod, TARE_1, DELTA_1, counts)
    assert homing.calls == [zb.WRONG_WAY_REASON] and idx == 120, (
        homing.calls, idx)
    print('ok  2 a 1.2 x delta step on a drifting cell aborts 40 samples '
          'in; a 300000 counts/s ramp aborts; a 1.1 x delta step aborts')


def test_3_ceiling_from_the_current_baseline(mod):
    # a fast push to 2.6 x the delta trips at once
    counts = [TARE_1] * 100 + [TARE_1 - int(2.6 * DELTA_1)] * 400
    w, homing, idx = run_watch(mod, TARE_1, DELTA_1, counts)
    assert homing.calls == [zb.CEILING_REASON] and idx == 120, (
        homing.calls, idx)
    # the wrong-side ceiling covers a sensor that jumps the wrong way
    counts = [TARE_2] * 100 + [TARE_2 + int(2.6 * DELTA_2)] * 400
    w, homing, idx = run_watch(mod, TARE_2, DELTA_2, counts)
    assert homing.calls and idx == 120, (homing.calls, idx)
    # 1M counts of press-way drift (measured, followed), then the MCU
    # trigger is dead and the bed is pushed at 3.5M counts/s: the host
    # ceiling stops it 2.5 x delta above the CURRENT baseline
    s = descent(-50000., 21., 3500000., 20.)
    r = run_mcu(mod, DELTA_1, s, mcu=False, slope=-50000.)
    assert r['abort'] == zb.CEILING_REASON, r
    c = cross(s, 2.5 * DELTA_1)
    assert c + 39 <= r['idx'] <= c + 80, (c, r['idx'])
    # a press to 2.3 x delta, held 0.2 s, is not past it
    s = [(c_, d, min(l, 2.3 * DELTA_1)) for c_, d, l in s[:int(20.3 * SPS)]]
    s = [(int(TARE_1 + d + PRESS * l), d, l) for c_, d, l in s]
    r = run_mcu(mod, DELTA_1, s, mcu=False, slope=-50000.)
    assert r['abort'] is None, r
    print('ok  3 a 2.6 x delta push or wrong-side jump aborts; with the '
          'trigger dead the ceiling stops a press 2.5 x delta above the '
          'baseline after 1M of drift, 2.3 x does not')


def test_5_slow_push_is_never_followed_cold(mod):
    # Review of 3c5158a: a push slower than the follow rate was followed
    # to the 2.5M cap.  With nothing measured (a cold cell) nothing is
    # followed: an 80000 counts/s push is stopped at 1 x delta, by the
    # trigger on the press side and by the wrong-way rule on the other.
    s = descent(0., 10., 80000., 1., noise=False)
    r = run_mcu(mod, DELTA_1, s, slope=0.)
    assert r['trigger'] is not None, r
    assert 1. <= force_at(s, r) < 1.01, force_at(s, r)
    s = descent(0., 10., 80000., 1., wrong=True, noise=False)
    r = run_mcu(mod, DELTA_1, s, slope=0.)
    assert r['abort'] == zb.WRONG_WAY_REASON, r
    c = cross(s, DELTA_1)
    assert c + 39 <= r['idx'] <= c + 80, (c, r['idx'])
    # hot, measured drifting the wrong way: a 20000 counts/s press-side
    # push only slows the drift the load shows, and the baseline follows
    # that; the press-side reference may trail the measured drift by no
    # more than the band, so it fires at about 1.25 x delta
    s = descent(SLOE_RATE, 16., 20000., 2.)
    r = run_mcu(mod, DELTA_1, s, slope=SLOE_RATE)
    assert r['trigger'] is not None, r
    assert 1.1 <= force_at(s, r) < 1.3, force_at(s, r)
    against = force_at(s, r)
    # the same on the other side: drifting the press way, a 20000
    # counts/s sign-flipped push is caught by the wrong-way rule
    s = descent(-SLOE_RATE, 16., 20000., 2., wrong=True)
    r = run_mcu(mod, DELTA_1, s, slope=-SLOE_RATE)
    assert r['abort'] == zb.WRONG_WAY_REASON, r
    assert force_at(s, r) < 1.35, force_at(s, r)
    # and if the measured drift stops (the hot end at temperature) the
    # trailing limit fires EARLY: a false touch, never a late one
    s = descent(0., 16., noise=False)
    r = run_mcu(mod, DELTA_1, s, slope=SLOE_RATE)
    assert r['trigger'] is not None and s[r['idx']][2] == 0., r
    assert abs(r['idx'] - 1.25 * DELTA_1 / SLOE_RATE * SPS) < 0.1 * SPS, r
    print('ok  5 an 80000 counts/s push on a cold cell triggers or aborts '
          'at 1 x delta; a 20000 counts/s press against measured drift '
          'triggers at %.3f x delta; drift that stops fires early' % against)


def test_6_ramp_of_110000(mod):
    # The review's ramp: 110000 counts/s, just over the old follow rate
    # (3c5158a followed it 360000 counts before freezing).
    out = []
    for slope, wrong in ((0., False), (0., True), (SLOE_RATE, False),
                         (SLOE_RATE, True)):
        drift = slope
        s = descent(drift, 12., 110000., 2., wrong=wrong)
        r = run_mcu(mod, DELTA_1, s, slope=slope)
        if wrong:
            assert r['abort'] == zb.WRONG_WAY_REASON, (slope, r)
            # the ramp at the confirming batch: 1 x delta, + the 20 ms
            # confirm and the 20 ms batch (4400), + what was followed
            f = force_at(s, r)
            assert f < 1.15, (slope, f)
        else:
            # against drift the press side holds until the load leaves
            # the band, so it fires late by up to the drift in that time
            # (1.15 here), within the 1.3 of test 7
            assert r['trigger'] is not None, (slope, r)
            f = force_at(s, r)
            assert 0.95 <= f < 1.2, (slope, f)
        out.append(f)
    print('ok  6 a 110000 counts/s ramp, cold or on measured drift: '
          'triggers at %.3f / %.3f x delta, wrong way aborts at %.3f / '
          '%.3f x delta' % (out[0], out[2], out[1], out[3]))


def test_7_soft_surface_at_probe_speed(mod):
    # A press at 4 mm/s into a surface of 25000 counts/mm (100000
    # counts/s), cold and on a cell measured drifting either way at
    # Sloe's rate: the trigger fires at 0.75-1.3 x delta of real force.
    # Against wrong-way drift the press reference goes on with the drift
    # (~1.0); with press-way drift it holds, so the drift helps (~0.85).
    out = []
    for slope in (0., SLOE_RATE, -SLOE_RATE):
        s = descent(slope, 14., 100000., 5.)
        r = run_mcu(mod, DELTA_1, s, slope=slope)
        assert r['trigger'] is not None and r['abort'] is None, (slope, r)
        f = force_at(s, r)
        assert 0.75 <= f <= 1.3, (slope, f)
        out.append(f)
    print('ok  7 4 mm/s into 25000 counts/mm: triggers at %.3f x delta '
          'cold, %.3f drifting away, %.3f drifting in' % tuple(out))


def test_8_sign_flipped_incident_at_4mm_s(mod):
    # The Boxwood incident (tare -3244819 past the anchor, delta 23407,
    # the trigger never fires) with the press reading the wrong way, at
    # 4 mm/s, on the long G28 descent: it must stop no later than the
    # 6.15 mm the fixed-tare ceiling took at 15 mm/s on a slow ramp.
    # Cold (nothing measured) and on a cell drifting the wrong way at
    # Sloe's rate (measured), stiff and soft.
    tare, delta, speed = zb.INCIDENT_TARE, zb.DELTA_INCIDENT, 4.
    out = []
    for slope in (0., SLOE_RATE):
        for stiff in (100000., 10000.):
            s = descent(slope, 6., stiff * speed, 1., wrong=True,
                        tare=tare)
            r = run_mcu(mod, delta, s, tare=tare, mcu=False, slope=slope)
            assert r['abort'] == zb.WRONG_WAY_REASON, (slope, stiff, r)
            mm = (r['idx'] - SPS) / float(SPS) * speed
            assert mm <= 6.15, (slope, stiff, mm)
            out.append(mm)
    print('ok  8 sign-flipped incident at 4 mm/s aborts after %.2f / %.2f '
          'mm (cold) and %.2f / %.2f mm (drifting)' % tuple(out))


def test_9_burst_is_not_captured(mod):
    # Measured drift the wrong way; at 5 s a 50 ms burst of 0.6 x delta
    # each way (past the band, under every limit).  The baseline freezes
    # (one WARNING), does not move toward the burst, follows again, and
    # contact at 10 s still fires at 1 x delta.
    for b in (0.6 * DELTA_1, -0.6 * DELTA_1):
        s = descent(SLOE_RATE, 11., 3500000., 10., burst=(5., 0.05, b))
        with zb.LogCapture() as cap:
            w, cell, homing = zb.make_watch(mod)
            w.arm(10., TARE_1, DELTA_1, PRESS, drift_slope=SLOE_RATE,
                  drift_t0=10.)
            rs = zb.rows(10., [x[0] for x in s])
            before = after = None
            for i in range(0, int(10. * SPS), 40):
                cell.feed(rs[i:i + 40])
                if i + 40 == int(4.98 * SPS):
                    before = w._base
                if i + 40 == int(5.2 * SPS):
                    after = w._base
            assert homing.calls == [], homing.calls
            warns = [m for lv, m in cap.lines if lv == 'WARNING'
                     and 'baseline froze' in m]
            assert len(warns) == 1, cap.lines
        # 0.22 s of drift at Sloe's rate is 10500; the burst was 87840
        assert abs((after - before) - 0.22 * SLOE_RATE) < 6000, (
            before, after)
        r = run_mcu(mod, DELTA_1, s, slope=SLOE_RATE)
        assert r['trigger'] is not None, r
        assert 0.95 <= force_at(s, r) < 1.1, force_at(s, r)
    print('ok  9 a 0.6 x delta burst either way freezes the baseline once '
          '(WARNING), is not followed, and contact after it fires at '
          '%.3f x delta' % force_at(s, r))


def test_10_push_with_the_drift_is_bounded_by_the_excess(mod):
    # The one push that is followed: in the measured drift's own sign and
    # slower than 1.5 x the slope.  Sloe's drift the wrong way plus a
    # 20000 counts/s push the same way (68000 in all, under the 71740
    # follow rate): the baseline runs ahead of slope x elapsed by 20000 a
    # second and the move is aborted when that passes 3 x delta (439200),
    # at 22 s: the push is then 3 x delta.
    for wrong, slope in ((True, SLOE_RATE), (False, -SLOE_RATE)):
        s = descent(slope, 30., 20000., 0., wrong=wrong, noise=False)
        r = run_mcu(mod, DELTA_1, s, slope=slope)
        assert r['abort'] == DRIFT_EXCESS_REASON, (wrong, r)
        f = force_at(s, r)
        assert 3. <= f < 3.05, (wrong, f)
    # a configured factor
    s = descent(SLOE_RATE, 30., 20000., 0., wrong=True, noise=False)
    r = run_mcu(mod, DELTA_1, s, slope=SLOE_RATE,
                z_move_drift_excess_factor=1.5)
    assert r['abort'] == DRIFT_EXCESS_REASON, r
    assert 1.5 <= force_at(s, r) < 1.55, force_at(s, r)
    print('ok  10 a 20000 counts/s push in the drift\'s own sign is '
          'followed and aborted at 3 x delta (1.5 x when configured)')


def test_11_sloe_heatup_either_way_then_contact(mod):
    # Sloe: 1.1M counts in 23 s, measured, with its wander on top, the
    # wrong way (as logged) and the press way (the mid-air stops at
    # z=113.6).  No abort and no trigger for the whole descent, then the
    # bed: 3.5M counts/s (4 mm/s) or 1M counts/s.  The MCU fires within
    # 0.25 x delta of the trigger delta above the drift.
    worst = 0.
    for sign in (1., -1.):
        for press in (3500000., 1000000.):
            s = descent(sign * SLOE_RATE, 24., press, 23.)
            r = run_mcu(mod, DELTA_1, s, slope=sign * SLOE_RATE)
            assert r['abort'] is None and r['trigger'] is not None, (
                sign, press, r)
            assert r['trigger'] >= 23 * SPS, (sign, press, r['trigger'])
            f = force_at(s, r)
            assert abs(f - 1.) < 0.25, (sign, press, f)
            worst = max(worst, abs(f - 1.))
            assert len(r['sent']) > 500, len(r['sent'])
        # with the MCU zero left at the tare the press-way drift stops
        # the descent in mid air, 3 s in, and the wrong-way drift needs
        # the 1.1M drift on top of the trigger force
        s = descent(sign * SLOE_RATE, 24., 3500000., 23.)
        r = run_mcu(mod, DELTA_1, s, retare=False, slope=sign * SLOE_RATE)
        if sign < 0:
            assert r['trigger'] is not None and r['trigger'] < 4 * SPS, r
        else:
            assert r['trigger'] is None, r
    print('ok  11 Sloe 1.1M counts in 23 s either way: no abort, contact '
          'at 3.5M or 1M counts/s triggers within %.3f x delta of nominal'
          % worst)


def test_12_contact_after_2M_followed(mod):
    # 2M counts of drift at the full 100000 counts/s, measured, either
    # way, with wander, then contact at 3.5M counts/s: the trigger fires
    # at the nominal force though its zero has moved 2M counts.
    for sign in (1., -1.):
        s = descent(sign * 100000., 21., 3500000., 20.)
        r = run_mcu(mod, DELTA_1, s, slope=sign * 100000.)
        assert r['abort'] is None and r['trigger'] is not None, r
        f = force_at(s, r)
        assert abs(f - 1.) < 0.25, (sign, f)
        assert abs(abs(r['zero'] - TARE_1) - 2000000) < 30000, r['zero']
    print('ok  12 2M counts followed at 100000 counts/s, then contact: '
          'the trigger fires at %.3f x delta' % f)


def test_13_fast_wrong_way_push_after_drift(mod):
    # 1M counts of measured drift (50000 counts/s) either way, then a
    # sign-error press at 2M counts/s: the wrong-way rule aborts with the
    # push about 1 x delta past where the drift really was.
    for sign in (1., -1.):
        s = descent(sign * 50000., 22., 2000000., 20., wrong=True)
        r = run_mcu(mod, DELTA_1, s, slope=sign * 50000.)
        assert r['abort'] == zb.WRONG_WAY_REASON, (sign, r)
        c = cross(s, DELTA_1, 20 * SPS)
        assert c + 39 <= r['idx'] <= c + 80, (sign, c, r['idx'])
        c2 = cross(s, 1.03 * DELTA_1, 20 * SPS)
        assert r['idx'] <= c2 + 80, (sign, c2, r['idx'])
    print('ok  13 a 2M counts/s wrong-way push after 1M of drift aborts '
          '20 ms after it passes 1 x delta')


def test_14_who_moves_the_mcu_zero(mod):
    s = descent(-50000., 5., noise=False)
    # measured drift: the zero follows in steps of at least 1000
    r = run_mcu(mod, DELTA_1, s, slope=-50000.)
    sent = r['sent']
    assert sent and all(abs(b - a) >= 1000 for a, b in zip(
        [TARE_1] + sent, sent)), sent[:5]
    assert abs(sent[-1] - (TARE_1 - 250000)) < 3000, sent[-1]
    # nothing measured, MCUTARE, an abs_ge trigger or a rate of 0: the
    # zero stays at the tare and so does the baseline
    for kw in ({'slope': 0.}, {'slope': -50000., 'mcutare': True},
               {'slope': -50000., 'direction': None},
               {'slope': -50000., 'z_move_drift_rate_counts': 0}):
        r = run_mcu(mod, DELTA_1, descent(-20000., 2., noise=False),
                    mcu=False, **kw)
        assert r['sent'] == [] and r['watch']._base == TARE_1, (kw, r)
    # never after the descent: disarm drops the hook
    w, cell, homing = zb.make_watch(mod)
    sent = []
    w.arm(10., TARE_1, DELTA_1, PRESS, retare=sent.append,
          drift_slope=-50000., drift_t0=10.)
    w.disarm()
    cell.feed(zb.rows(10., [TARE_1 - 100000] * 400))
    assert sent == [] and w._retare is None, sent
    # a failing send aborts the move (fails safe)
    w, cell, homing = zb.make_watch(mod)

    def broken(z):
        raise IOError('mcu gone')
    w.arm(10., TARE_1, DELTA_1, PRESS, retare=broken, drift_slope=-50000.,
          drift_t0=10.)
    cell.feed(zb.rows(10., [x[0] for x in s[:400]]))
    assert homing.calls == [zb.FAILED_REASON], homing.calls
    print('ok  14 the MCU zero moves only on measured drift (not MCUTARE, '
          'grams mode or rate 0), in steps of 1000+, never after disarm; '
          'a failed send aborts')


def load_trigger_analog():
    import types
    m = types.ModuleType('mcu')

    class MCU_trsync(object):
        REASON_COMMS_TIMEOUT = 2
    m.MCU_trsync = MCU_trsync
    sys.modules['mcu'] = m
    path = os.path.join(ROOT, 'klippy', 'extras', 'trigger_analog.py')
    mod = types.ModuleType('trigger_analog_under_test')
    exec(compile(open(path).read(), path, 'exec'), mod.__dict__)
    return mod


def test_15_probing_move_long_and_short(mod):
    # The real MCU_SosFilter.update_offset: the offset only, with the
    # scale the tare set and auto_offset off, and recorded so the next
    # tare's offset is sent again.
    ta = load_trigger_analog()

    class Cmd(object):
        def __init__(self):
            self.sent = []

        def send(self, args):
            self.sent.append(tuple(args))
    f = object.__new__(ta.MCU_SosFilter)
    f._oid, f._max_sections, f._design = 7, 0, None
    f._coeff_frac_bits, f._start_value = 0, 0.
    f._last_sent_coeffs = []
    f._last_sent_offset_scale = None
    f._set_offset_scale_cmd = Cmd()
    f._set_section_cmd = f._set_state_cmd = f._set_active_cmd = Cmd()
    f.set_offset_scale(121734, -1., 0)
    f.reset_filter()
    f.update_offset(70000)
    f.set_offset_scale(121734, -1., 0)     # the next tap's tare
    f.reset_filter()
    assert f._set_offset_scale_cmd.sent == [
        (7, 121734, -1, 0, False), (7, 70000, -1, 0, False),
        (7, 121734, -1, 0, False)], f._set_offset_scale_cmd.sent
    # LoadCellProbingMove: the long G28 descent (Z160 -> z_min -4, no
    # trigger yet) fits 0.5 s of samples before the tare, arms the watch
    # with that slope and moves the trigger zero; the tare is the newest
    # tare_time of the window.
    tare = -800000
    rate = -60000.
    cell = zb.Cell(SPS, counts_per_gram=1000., tare_counts=tare)
    m, cell, fake, w = zb.make_move(mod, 'trigger', tare=tare, cell=cell)

    class Window(object):
        def __init__(self, mv):
            mv._collect_start_time = 10.

        def start_collecting(self, min_time=None):
            pass

        def collect_min(self, n=1):
            got['n'] = n
            return [[9.5 + i / 2000., 0., int(tare + rate * (i - n) / 2000.)]
                    for i in range(n)], None

        def stop_collecting(self):
            pass
    got = {}
    m._start_collector = lambda: Window(m)
    fed = descent(rate, 3., tare=tare, noise=False)

    class Ph(zb.Homing):
        def probing_move(self, endstop, pos, speed):
            seen['slope'] = w._sign * w._slope
            for b in range(0, len(fed), 40):
                cell.feed(zb.rows(10. + b / float(SPS),
                                  [x[0] for x in fed[b:b + 40]]))
            return [0., 0., 1.2, 0.]
    seen = {}
    m._printer.objects['homing'] = Ph()
    log = m._mcu_trigger_analog.log
    m.probing_move(zb.Gcmd(0))
    ntare = m._config_helper.get_tare_samples(None)
    assert got['n'] == max(ntare, 1000), (got, ntare)
    assert abs(seen['slope'] - rate) < 50, seen
    offs = [e for e in log if e[0] == 'set_offset_scale']
    assert offs[-1][1] == -m._tare_counts, (offs[-1], m._tare_counts)
    assert 0 < m._tare_counts - tare <= -rate * ntare / 2000., m._tare_counts
    moved = [e[1] for e in log if e[0] == 'update_offset']
    assert len(moved) > 100, len(moved)
    assert abs(moved[-1] - (-tare + 180000)) < 2000, moved[-1]
    assert not w.is_armed() and w._retare is None
    # a descent that starts within 20 mm of the last trigger (the confirm
    # tap, a mesh point) or that can only travel under 20 mm: no window,
    # no slope, nothing moved
    for last, z in ((1.2, 15.), (None, 15.)):
        del log[:]
        m._last_trigger_z = last
        m._printer.objects['toolhead'].get_position = (
            lambda z=z: [100., 100., z, 0.])
        m.probing_move(zb.Gcmd(0))
        assert got['n'] == ntare, (last, z, got)
        assert seen['slope'] == 0., seen
        assert [e for e in log if e[0] == 'update_offset'] == [], log
    m._last_trigger_z = 1.2
    assert not m._is_long_descent(21.)
    assert m._is_long_descent(21.3)
    m._last_trigger_z = None
    assert not m._is_long_descent(15.9) and m._is_long_descent(16.)
    # MCUTARE: the same descent moves nothing on the MCU
    del log[:]
    m._printer.objects['toolhead'].get_position = (
        lambda: [100., 100., 160., 0.])
    m._last_trigger_z = 100.
    m._mcu_tare = mod.LoadCellProbingMove._mcu_tare.__get__(m)

    class Fresh(object):
        def start_collecting(self, min_time=None):
            pass

        def collect_min(self, n=1):
            return [[10., 0., tare]], None
    cell.get_collector = lambda: Fresh()
    m._approx_tare = tare
    m.probing_move(zb.Gcmd(1))
    assert seen['slope'] == 0.
    assert [e for e in log if e[0] == 'update_offset'] == [], log
    print('ok  15 a long descent fits 0.5 s for its slope and moves the '
          'trigger zero (MCU_SosFilter.update_offset); confirm taps, short '
          'descents and MCUTARE keep a fixed tare')


def test_16_descent_log(mod):
    with zb.LogCapture() as cap:
        w, cell, homing = zb.make_watch(mod)
        w.arm(10., TARE_1, DELTA_1, PRESS, drift_slope=20000.,
              drift_t0=10.)
        zb.feed_until_abort(cell, homing, 10., drift_counts(
            TARE_1, 20000., 3.))
        w.disarm()      # the descent ended without an abort
        w.disarm()
        ended = [(lv, m) for lv, m in cap.lines if 'backstop descent' in m]
        assert len(ended) == 1 and ended[0][0] == 'INFO', cap.lines
        msg = ended[0][1]
        for part in ('ended', 'slope 20000 counts/s', 'up to 30000',
                     'froze 0 times'):
            assert part in msg, (part, msg)
        assert 'over 3.000 s' in msg or 'over 2.999 s' in msg, msg
    st = w.get_status()
    assert st['z_backstop_drift_slope'] == 20000., st
    assert st['z_backstop_drift_excess_factor'] == 3., st
    print('ok  16 one INFO line per descent with the measured slope, the '
          'follow rate and the freezes')


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
    tests = [(test_1_measured_drift_is_followed, mod),
             (test_2_a_real_wrong_way_event_still_trips, mod),
             (test_3_ceiling_from_the_current_baseline, mod),
             (test_4_logged_tap_pairs_pass_the_shipped_tolerance, probe),
             (test_5_slow_push_is_never_followed_cold, mod),
             (test_6_ramp_of_110000, mod),
             (test_7_soft_surface_at_probe_speed, mod),
             (test_8_sign_flipped_incident_at_4mm_s, mod),
             (test_9_burst_is_not_captured, mod),
             (test_10_push_with_the_drift_is_bounded_by_the_excess, mod),
             (test_11_sloe_heatup_either_way_then_contact, mod),
             (test_12_contact_after_2M_followed, mod),
             (test_13_fast_wrong_way_push_after_drift, mod),
             (test_14_who_moves_the_mcu_zero, mod),
             (test_15_probing_move_long_and_short, mod),
             (test_16_descent_log, mod)]
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
