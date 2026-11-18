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

    python3 test/load_cell_probe/test_hot_load_cell.py

Needs numpy.
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


def run_watch(mod, tare, delta, counts, **opts):
    w, cell, homing = zb.make_watch(mod, **opts)
    t0 = 10.
    w.arm(t0, tare, delta, PRESS)
    idx, _ = zb.feed_until_abort(cell, homing, t0, counts)
    return w, homing, idx


def test_1_slow_wrong_side_drift_does_not_trip(mod):
    # Logged attempt 2: tare 1006163, wrong way 259190.  A descent that
    # lasts 12 s with the cell drifting 30000 counts/s the wrong way
    # (360000 in all: past the wrong-way limit, 55% of the ceiling).
    w, homing, idx = run_watch(mod, TARE_2, DELTA_2,
                               drift_counts(TARE_2, 30000., 12.))
    assert w.get_limits() == (647975, 259190), w.get_limits()
    assert homing.calls == [] and idx is None, (homing.calls, idx)
    # Logged attempt 1: tare -121734, wrong way 146400, ceiling 366000.
    # Sloe's own heat-up: the tare moved 1.1M counts in 23 s (48000
    # counts/s, t=2176 -> 2199): past BOTH limits, never trips them.
    w, homing, idx = run_watch(mod, TARE_1, DELTA_1,
                               drift_counts(TARE_1, 1100000. / 23., 23.))
    assert w.get_limits() == (366000, 146400), w.get_limits()
    assert homing.calls == [] and idx is None, (homing.calls, idx)
    # 3M counts over 60 s (50000 counts/s) is more than one descent may
    # absorb: the cap of 2.5M stops the allowance at 50 s and the rule
    # trips once the drift is cap + 1 delta = 2646400 (52.93 s).
    w, homing, idx = run_watch(mod, TARE_1, DELTA_1,
                               drift_counts(TARE_1, 50000., 60.))
    assert homing.calls == [zb.WRONG_WAY_REASON], homing.calls
    assert abs(idx - 52.93 * SPS) < 100, idx
    # the old behaviour (z_move_drift_rate_counts 0) tripped on it
    w, homing, idx = run_watch(mod, TARE_1, DELTA_1,
                               drift_counts(TARE_1, 1100000. / 23., 23.),
                               z_move_drift_rate_counts=0)
    assert homing.calls and homing.calls[0] in (
        zb.WRONG_WAY_REASON, zb.CEILING_REASON), homing.calls
    print('ok  1 30000 counts/s for 12 s (tare 1006163) and 1.1M counts '
          'over 23 s (tare -121734) no longer abort; 3M over 60 s aborts '
          'at cap + delta; with the rate at 0 it aborts early')


def test_2_a_real_wrong_way_event_still_trips(mod):
    # A step of 1.2 x the delta (the nozzle dragging, a sign error) on a
    # cell that has already drifted 100000 counts: judged at full size.
    step = int(1.2 * DELTA_1)
    counts = drift_counts(TARE_1, 20000., 10., step=step, step_at=5.)
    w, homing, idx = run_watch(mod, TARE_1, DELTA_1, counts)
    assert homing.calls == [zb.WRONG_WAY_REASON], homing.calls
    # idx is the 40-sample batch that confirms it: the one starting at 5 s
    assert 10000 <= idx <= 10000 + 41, idx
    # a ramp of 300000 counts/s (a nozzle dragging along the bed) trips
    # inside a second, even with the drift allowance
    counts = drift_counts(TARE_1, 300000., 3.)
    w, homing, idx = run_watch(mod, TARE_1, DELTA_1, counts)
    assert homing.calls and idx < 1.5 * SPS, (homing.calls, idx)
    # a step of 1.1 x 146400 on a cold cell, no drift at all
    counts = [TARE_1] * 100 + [TARE_1 + int(1.1 * DELTA_1)] * 400
    w, homing, idx = run_watch(mod, TARE_1, DELTA_1, counts)
    assert homing.calls == [zb.WRONG_WAY_REASON] and idx == 120, (
        homing.calls, idx)
    print('ok  2 a 1.2 x delta step on a drifted cell aborts 40 samples '
          'in; a 300000 counts/s ramp aborts; a 1.1 x delta step aborts')


def test_3_the_ceiling_still_trips(mod):
    # the press side is never re-referenced: a slow push at the same
    # 20000 counts/s still reaches the ceiling (366000 after 18.3 s)
    counts = [int(TARE_1 - 20000. * i / SPS) for i in range(int(25 * SPS))]
    w, homing, idx = run_watch(mod, TARE_1, DELTA_1, counts)
    assert homing.calls == [zb.CEILING_REASON], homing.calls
    assert abs(idx - 18.3 * SPS) < 80, idx  # idx = 40-sample batch start
    # a fast push to 2.6 x the delta trips at once
    counts = [TARE_1] * 100 + [TARE_1 - int(2.6 * DELTA_1)] * 400
    w, homing, idx = run_watch(mod, TARE_1, DELTA_1, counts)
    assert homing.calls == [zb.CEILING_REASON] and idx == 120, (
        homing.calls, idx)
    # the wrong-side ceiling still covers a sensor that jumps the wrong
    # way past it (a step, not drift)
    counts = [TARE_2] * 100 + [TARE_2 + int(2.6 * DELTA_2)] * 400
    w, homing, idx = run_watch(mod, TARE_2, DELTA_2, counts)
    assert homing.calls and idx == 120, (homing.calls, idx)
    # a drifted cell that then comes back is not left with a stale
    # allowance: drift 200000 in 2 s, back to the tare for 1 s, then a
    # 1.2 x delta step trips 40 samples in
    counts = (drift_counts(TARE_1, 100000., 2.)
              + [TARE_1] * 2000 + [TARE_1 + int(1.2 * DELTA_1)] * 400)
    w, homing, idx = run_watch(mod, TARE_1, DELTA_1, counts)
    assert homing.calls == [zb.WRONG_WAY_REASON], homing.calls
    assert 6000 <= idx <= 6041, idx
    print('ok  3 a 20000 counts/s push still hits the ceiling at 18.3 s; '
          'a 2.6 x delta push or wrong-side jump aborts; a stale '
          'allowance is dropped when the load comes back')


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


def test_5_sign_error_press_after_absorbed_drift(mod):
    # 10 s of 50000 counts/s drift (500000 absorbed, under the cap of
    # the 2.5M cap), then the press itself rises the wrong way at 2M
    # counts/s (a sign error).  Judged sample by sample.
    n0 = 10 * SPS
    counts = drift_counts(TARE_1, 50000., 10.)
    base = counts[-1]
    counts += [int(base + 2000000. * i / SPS) for i in range(1, 400)]
    w, cell, homing = zb.make_watch(mod)
    w.arm(10., TARE_1, DELTA_1, PRESS)
    seen = []
    orig = w._judge

    def judge(c, t=None):
        r = orig(c, t)
        seen.append(w._drift)
        return r
    w._judge = judge
    idx, _ = zb.feed_until_abort(cell, homing, 10., counts, batch=1)
    assert homing.calls == [zb.WRONG_WAY_REASON], homing.calls
    # first sample more than one delta past the absorbed drift
    cross = next(k for k in range(n0, len(counts))
                 if counts[k] - TARE_1 - seen[k] > DELTA_1)
    # confirmed 40 samples (20 ms) later
    assert idx == cross + 39, (idx, cross)
    over = counts[cross] - TARE_1 - seen[cross]
    assert DELTA_1 < over < 1.02 * DELTA_1 + 1000, (over, DELTA_1)
    assert max(seen) <= 2500000, max(seen)
    print('ok  5 a 2M counts/s sign-error press after 10 s of absorbed '
          'drift crosses the limit at %.3f x delta and aborts 20 ms later'
          % (over / float(DELTA_1)))


def test_6_drift_that_changes_direction(mod):
    # wrong side 20000 counts/s for 5 s, then back across the tare to the
    # press side for 10 s, then wrong side again 10 s, then back 5 s:
    # a cell that wanders both ways never trips
    counts = []
    c = float(TARE_1)
    for rate, secs in ((20000., 5.), (-20000., 10.), (20000., 10.),
                       (-20000., 5.)):
        for _ in range(int(secs * SPS)):
            c += rate / SPS
            counts.append(int(c))
    w, homing, idx = run_watch(mod, TARE_1, DELTA_1, counts)
    assert homing.calls == [] and idx is None, (homing.calls, idx)
    # after a reversal the old allowance is gone: back at the tare, a
    # 1.2 x delta step trips at full size, 40 samples in
    counts = (drift_counts(TARE_1, 20000., 5.)
              + [int(TARE_1 + 100000 - 20000. * i / SPS)
                 for i in range(1, int(5 * SPS) + 1)])
    counts += [TARE_1 + int(1.2 * DELTA_1)] * 400
    w, homing, idx = run_watch(mod, TARE_1, DELTA_1, counts)
    assert homing.calls == [zb.WRONG_WAY_REASON], homing.calls
    assert 20000 <= idx <= 20000 + 41, idx
    print('ok  6 drift that reverses never trips; a step after the '
          'reversal trips at full size')


def test_7_noise_of_50000_counts_does_not_trip(mod):
    # +-50000 counts of noise (a third of the smallest logged delta),
    # alone and on top of a heat-up drift of 20000 counts/s
    seed = 4364
    for rate in (0., 20000.):
        counts = []
        for b in drift_counts(TARE_1, rate, 20.):
            seed = (seed * 1103515245 + 12345) % (1 << 31)
            counts.append(b - 50000 + seed % 100001)
        w, homing, idx = run_watch(mod, TARE_1, DELTA_1, counts)
        assert homing.calls == [] and idx is None, (rate, homing.calls)
    print('ok  7 +-50000 counts of noise, with and without 20000 counts/s '
          'of drift, never trips')


def test_8_drift_is_logged_once_per_descent(mod):
    with zb.LogCapture() as cap:
        w, cell, homing = zb.make_watch(mod)
        w.arm(10., TARE_1, DELTA_1, PRESS)
        zb.feed_until_abort(cell, homing, 10., drift_counts(
            TARE_1, 20000., 3.))
        w.disarm()      # the descent ended without an abort
        w.disarm()
        ended = [(lv, m) for lv, m in cap.lines if 'backstop descent' in m]
        assert len(ended) == 1 and ended[0][0] == 'INFO', cap.lines
        msg = ended[0][1]
        assert 'ended' in msg and 'cap 2500000' in msg, msg
        assert 'absorbed 59' in msg and 'over 3.000 s' in msg or \
            'over 2.999 s' in msg, msg
    with zb.LogCapture() as cap:
        w, cell, homing = zb.make_watch(mod)
        w.arm(10., TARE_1, DELTA_1, PRESS)
        zb.feed_until_abort(cell, homing, 10., [TARE_1] * 100
                            + [TARE_1 + 3 * DELTA_1] * 400)
        w.disarm()
        ended = [m for lv, m in cap.lines if 'backstop descent' in m]
        assert len(ended) == 1 and 'aborted' in ended[0], cap.lines
    # the cap is absolute, and configurable
    w, cell, homing = zb.make_watch(mod)
    w.arm(10., TARE_2, 400000, PRESS)
    assert w._drift_cap == 2500000, w._drift_cap
    w.arm(10., TARE_1, DELTA_1, PRESS)
    assert w._drift_cap == 2500000, w._drift_cap
    w, cell, homing = zb.make_watch(mod, z_move_drift_cap_counts=900000)
    w.arm(10., TARE_1, DELTA_1, PRESS)
    assert w._drift_cap == 900000, w._drift_cap
    assert w.get_status()['z_backstop_drift_cap_counts'] == 900000
    print('ok  8 one INFO line per descent with the drift, its peak, the '
          'cap and the time; cap absolute 2.5M')


def test_9_rate_margin_and_peak_lag(mod):
    # A hotter unit than Sloe: 60000 counts/s for 30 s (1.8M, under the
    # cap) used to trip the 50000 rate after ~15 s; at the 100000 default
    # it does not.
    w, homing, idx = run_watch(mod, TARE_1, DELTA_1,
                               drift_counts(TARE_1, 60000., 30.))
    assert homing.calls == [] and idx is None, (homing.calls, idx)
    # 150000 counts/s outruns the 100000 allowance: the lag grows 50000
    # counts/s and the rule trips when it passes one delta, 146400 / 50000
    # = 2.93 s in (+ the 20 ms confirm)
    w, homing, idx = run_watch(mod, TARE_1, DELTA_1,
                               drift_counts(TARE_1, 150000., 10.))
    assert homing.calls == [zb.WRONG_WAY_REASON], homing.calls
    assert abs(idx - 2.93 * SPS) < 120, idx
    # ... and the INFO line reports that peak lag and when it happened
    with zb.LogCapture() as cap:
        w, homing, idx = run_watch(mod, TARE_1, DELTA_1,
                                   drift_counts(TARE_1, 150000., 10.))
        lines = [m for lv, m in cap.lines if 'backstop descent' in m]
    assert len(lines) == 1 and 'aborted' in lines[0], cap.lines
    m = re.search(r'peak lag (\d+) counts at ([0-9.]+) s, over ([0-9.]+) s',
                  lines[0])
    assert m, lines[0]
    lag, at, over = int(m.group(1)), float(m.group(2)), float(m.group(3))
    assert DELTA_1 < lag < 1.02 * DELTA_1 + 2000, lag
    assert 2.9 < at < 3.0 and at <= over < at + 0.03, (at, over)
    # the press that follows long drift still trips at ~1.0 x delta: the
    # 2M counts/s sign-error press of test 5, on 60000 counts/s of drift
    counts = drift_counts(TARE_1, 60000., 10.)
    base = counts[-1]
    counts += [int(base + 2000000. * i / SPS) for i in range(1, 400)]
    w, cell, homing = zb.make_watch(mod)
    w.arm(10., TARE_1, DELTA_1, PRESS)
    seen = []
    orig = w._judge

    def judge(c, t=None):
        r = orig(c, t)
        seen.append(w._drift)
        return r
    w._judge = judge
    idx, _ = zb.feed_until_abort(cell, homing, 10., counts, batch=1)
    assert homing.calls == [zb.WRONG_WAY_REASON], homing.calls
    cross = next(k for k in range(10 * SPS, len(counts))
                 if counts[k] - TARE_1 - seen[k] > DELTA_1)
    over = counts[cross] - TARE_1 - seen[cross]
    assert idx == cross + 39 and over < 1.02 * DELTA_1 + 1000, (idx, over)
    print('ok  9 60000 counts/s for 30 s does not trip; 150000 counts/s '
          'trips by lag at %.2f s (peak lag %i); a 2M counts/s press after '
          'drift trips at %.3f x delta' % (at, lag, over / float(DELTA_1)))


def main():
    mod = tg.load_module(tg._paths.patched(
        'klippy/extras/load_cell_probe.py'))
    probe = rf.load_module(rf._paths.patched('klippy/extras/probe.py'),
                           'hot_probe')
    tests = [(test_1_slow_wrong_side_drift_does_not_trip, mod),
             (test_2_a_real_wrong_way_event_still_trips, mod),
             (test_3_the_ceiling_still_trips, mod),
             (test_4_logged_tap_pairs_pass_the_shipped_tolerance, probe),
             (test_5_sign_error_press_after_absorbed_drift, mod),
             (test_6_drift_that_changes_direction, mod),
             (test_7_noise_of_50000_counts_does_not_trip, mod),
             (test_8_drift_is_logged_once_per_descent, mod),
             (test_9_rate_margin_and_peak_lag, mod)]
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
