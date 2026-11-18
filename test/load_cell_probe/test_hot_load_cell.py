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

Tests 10-16 (the #148 review): the MCU trigger's zero follows the same
slowly tracked baseline as the host rules.  run_mcu() models the MCU
trigger beside the real watch: it fires on (counts - zero) x press >
delta, zero being the last one the host sent, applied 20 ms after the
batch that produced it.  Contact after 1.1-2M counts of drift either way
fires at the nominal force; a fast wrong-way push aborts at 1 x delta; a
push slower than the follow rate aborts at the cap; the ceiling is from
the current baseline.

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
DRIFT_CAP_REASON = ("the load cell baseline drifted more than "
                    "z_move_drift_cap_counts during the Z move")


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
    # follow: the baseline passes the 2.5M cap at 50 s and the move is
    # aborted for that, with its own reason.
    w, homing, idx = run_watch(mod, TARE_1, DELTA_1,
                               drift_counts(TARE_1, 50000., 60.))
    assert homing.calls == [DRIFT_CAP_REASON], homing.calls
    assert abs(idx - 50. * SPS) < 60, idx
    # the old behaviour (z_move_drift_rate_counts 0) tripped on it
    w, homing, idx = run_watch(mod, TARE_1, DELTA_1,
                               drift_counts(TARE_1, 1100000. / 23., 23.),
                               z_move_drift_rate_counts=0)
    assert homing.calls and homing.calls[0] in (
        zb.WRONG_WAY_REASON, zb.CEILING_REASON), homing.calls
    print('ok  1 30000 counts/s for 12 s (tare 1006163) and 1.1M counts '
          'over 23 s (tare -121734) no longer abort; 3M over 60 s aborts '
          'on the 2.5M cap at 50 s; with the rate at 0 it aborts early')


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
    # KAN-4364: a slow press-side drift is followed like any other, so
    # 20000 counts/s for 25 s (500000, past the old fixed ceiling at
    # 18.3 s) no longer aborts: test 13 bounds it with the cap instead
    counts = [int(TARE_1 - 20000. * i / SPS) for i in range(int(25 * SPS))]
    w, homing, idx = run_watch(mod, TARE_1, DELTA_1, counts)
    assert homing.calls == [] and idx is None, (homing.calls, idx)
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
    # a drifted cell that then comes back is followed back: drift 200000
    # up in 2 s, down again in 2 s, then a 1.2 x delta step trips 40
    # samples in, judged at full size from the baseline it came back to
    counts = (drift_counts(TARE_1, 100000., 2.)
              + drift_counts(TARE_1 + 200000, -100000., 2.)
              + [TARE_1 + int(1.2 * DELTA_1)] * 400)
    w, homing, idx = run_watch(mod, TARE_1, DELTA_1, counts)
    assert homing.calls == [zb.WRONG_WAY_REASON], homing.calls
    assert 8000 <= idx <= 8041, idx
    print('ok  3 a 20000 counts/s press-side drift is followed; a 2.6 x '
          'delta push or wrong-side jump aborts; a baseline that came '
          'back judges a step at full size')


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
    # 10 s of 50000 counts/s drift (500000 followed, under the 2.5M cap),
    # then the press itself rises the wrong way at 2M counts/s (a sign
    # error).  Judged sample by sample.
    n0 = 10 * SPS
    counts = drift_counts(TARE_1, 50000., 10.)
    level = counts[-1]
    counts += [int(level + 2000000. * i / SPS) for i in range(1, 400)]
    w, cell, homing = zb.make_watch(mod)
    w.arm(10., TARE_1, DELTA_1, PRESS)
    seen = []
    orig = w._judge

    def judge(c, t=None):
        r = orig(c, t)
        seen.append(w._base)
        return r
    w._judge = judge
    idx, _ = zb.feed_until_abort(cell, homing, 10., counts, batch=1)
    assert homing.calls == [zb.WRONG_WAY_REASON], homing.calls
    # first sample more than one delta past the baseline
    cross = next(k for k in range(n0, len(counts))
                 if counts[k] - seen[k] > DELTA_1)
    # confirmed 40 samples (20 ms) later
    assert idx == cross + 39, (idx, cross)
    # the push measured from where the drift really was: the baseline
    # moved 1900 counts (rate x band / press rate) before it froze
    over = counts[cross] - level
    assert DELTA_1 < over < 1.02 * DELTA_1 + 1000, (over, DELTA_1)
    assert max(seen) - TARE_1 <= 2500000, max(seen)
    print('ok  5 a 2M counts/s sign-error press after 10 s of followed '
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
    # 150000 counts/s outruns the 100000 rate: the lag grows 50000
    # counts/s until it leaves the band (0.25 x 146400 = 36600, 0.732 s,
    # baseline 73200 up) and the baseline freezes; the rule trips when
    # the load is one delta past it, (73200 + 146400) / 150000 = 1.464 s
    # in (+ the 20 ms confirm)
    w, homing, idx = run_watch(mod, TARE_1, DELTA_1,
                               drift_counts(TARE_1, 150000., 10.))
    assert homing.calls == [zb.WRONG_WAY_REASON], homing.calls
    assert abs(idx - 1.464 * SPS) < 80, idx
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
    assert 1.45 < at < 1.5 and at <= over < at + 0.03, (at, over)
    # the press that follows long drift still trips at ~1.0 x delta: the
    # 2M counts/s sign-error press of test 5, on 60000 counts/s of drift
    counts = drift_counts(TARE_1, 60000., 10.)
    level = counts[-1]
    counts += [int(level + 2000000. * i / SPS) for i in range(1, 400)]
    w, cell, homing = zb.make_watch(mod)
    w.arm(10., TARE_1, DELTA_1, PRESS)
    seen = []
    orig = w._judge

    def judge(c, t=None):
        r = orig(c, t)
        seen.append(w._base)
        return r
    w._judge = judge
    idx, _ = zb.feed_until_abort(cell, homing, 10., counts, batch=1)
    assert homing.calls == [zb.WRONG_WAY_REASON], homing.calls
    cross = next(k for k in range(10 * SPS, len(counts))
                 if counts[k] - seen[k] > DELTA_1)
    over = counts[cross] - level
    assert idx == cross + 39 and over < 1.02 * DELTA_1 + 1000, (idx, over)
    print('ok  9 60000 counts/s for 30 s does not trip; 150000 counts/s '
          'trips by lag at %.2f s (peak lag %i); a 2M counts/s press after '
          'drift trips at %.3f x delta' % (at, lag, over / float(DELTA_1)))


# --- KAN-4364: the MCU trigger follows the baseline ----------------------

# Sloe's idle wander (test_z_load_backstop test 2), as a fixed sequence
def wander(n, seed=4364):
    out = []
    for _ in range(n):
        seed = (seed * 1103515245 + 12345) % (1 << 31)
        out.append(-4000 + seed % 11001)
    return out


def descent(drift_rate, seconds, press_rate=None, press_at=None,
            wrong=False, tare=TARE_1, noise=True):
    """A descent's samples as (counts, drift, load): the cell drifts by
    drift_rate counts/s (positive is the wrong way) from tare, and from
    press_at seconds on the nozzle is pressed at press_rate counts/s
    (wrong=True: a sign error, the press reads the wrong way).  load is
    the press alone, in counts toward the press side."""
    n = int(seconds * SPS)
    nz = wander(n) if noise else [0] * n
    out = []
    for i in range(n):
        d = drift_rate * i / float(SPS)
        load = 0.
        if press_rate is not None and i >= int(press_at * SPS):
            load = press_rate * (i - int(press_at * SPS)) / float(SPS)
        sign = -PRESS if wrong else PRESS
        out.append((int(tare + d + sign * load + nz[i]), d, load))
    return out


def run_mcu(mod, delta, samples, tare=TARE_1, mcu=True, retare=True,
            latency=40, batch=40, mcutare=False, direction=PRESS, **opts):
    """The real ZLoadBackstop with a model of the MCU trigger beside it.
    The MCU sees every sample as it is converted and triggers ("gt") on
    the first with (counts - zero) x press > delta, zero being the last
    one the host sent; a zero the host sends after a 20 ms batch reaches
    the MCU `latency` samples after that batch (40 = another 20 ms).
    Returns a dict: what ended the move and at which sample."""
    w, cell, homing = zb.make_watch(mod, **opts)
    sent = []
    w.arm(10., tare, delta, direction, mcutare=mcutare,
          retare=sent.append if retare else None)
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


def test_10_sloe_heatup_either_way_then_contact(mod):
    # Sloe: 1.1M counts in 23 s, with its idle wander on top, the wrong
    # way (as logged) and the press way (the mid-air stops at z=113.6).
    # No abort and no trigger for the whole descent, then the bed: a
    # press rising 3.5M counts/s (4 mm/s) or only 1M counts/s.  The MCU
    # fires at the trigger delta above the drift, within 0.25 x delta.
    rate = 1100000. / 23.
    worst = 0.
    for sign in (1., -1.):
        for press in (3500000., 1000000.):
            s = descent(sign * rate, 24., press, 23.)
            r = run_mcu(mod, DELTA_1, s)
            assert r['abort'] is None and r['trigger'] is not None, (
                sign, press, r)
            i = r['trigger']
            assert i >= 23 * SPS, (sign, press, i)
            force = s[i][2]
            assert abs(force - DELTA_1) < 0.25 * DELTA_1, (sign, press,
                                                           force)
            worst = max(worst, abs(force - DELTA_1) / DELTA_1)
            assert len(r['sent']) > 500, len(r['sent'])
        # without the MCU zero following (the #147 trigger) the same
        # press-way drift stops the descent in mid air, 3 s in, and the
        # wrong-way drift needs the 1.1M drift on top of the trigger force
        s = descent(sign * rate, 24., 3500000., 23.)
        r = run_mcu(mod, DELTA_1, s, retare=False)
        if sign < 0:
            assert r['trigger'] is not None and r['trigger'] < 4 * SPS, r
        else:
            assert r['trigger'] is None, r
    print('ok  10 Sloe 1.1M counts in 23 s either way: no abort, contact '
          'at 3.5M or 1M counts/s triggers within %.3f x delta of nominal '
          '(fixed MCU zero: stops in mid air / misses)' % worst)


def test_11_contact_after_2M_followed(mod):
    # 2M counts of drift at the full 100000 counts/s rate, either way,
    # with wander, then contact at 3.5M counts/s: the MCU still fires at
    # the nominal force, though its zero has moved 2M counts.
    for sign in (1., -1.):
        s = descent(sign * 100000., 21., 3500000., 20.)
        r = run_mcu(mod, DELTA_1, s)
        assert r['abort'] is None and r['trigger'] is not None, r
        i = r['trigger']
        force = s[i][2]
        assert abs(force - DELTA_1) < 0.25 * DELTA_1, (sign, force)
        assert abs(abs(r['zero'] - TARE_1) - 2000000) < 30000, r['zero']
        # with the MCU left on the stale zero only the host ceiling
        # (from the current baseline) stops this press: at 2.5 x delta
        r2 = run_mcu(mod, DELTA_1, s, retare=False)
        if sign > 0:
            assert r2['abort'] == zb.CEILING_REASON, r2
            cross = next(k for k in range(len(s))
                         if s[k][2] > 2.5 * DELTA_1)
            assert cross + 39 <= r2['idx'] <= cross + 80, (cross, r2)
    print('ok  11 2M counts followed at 100000 counts/s, then contact: '
          'the trigger fires at %.3f x delta' % (force / float(DELTA_1)))


def test_12_fast_wrong_way_push_after_drift(mod):
    # 1M counts of drift (50000 counts/s) either way, then a sign-error
    # press at 2M counts/s: the wrong-way rule aborts with the push about
    # 1 x delta past where the drift really was; the MCU never fires.
    for sign in (1., -1.):
        s = descent(sign * 50000., 22., 2000000., 20., wrong=True)
        r = run_mcu(mod, DELTA_1, s)
        assert r['abort'] == zb.WRONG_WAY_REASON, (sign, r)
        # the abort is confirmed 20 ms (40 samples) after the crossing;
        # the batch ends within 40 more
        push = s[r['idx']][2]
        lo, hi = DELTA_1, DELTA_1 + 2000000. * 0.042 + 5000
        assert lo < push < hi, (sign, push)
        cross = next(i for i in range(20 * SPS, len(s))
                     if s[i][2] > DELTA_1)
        assert cross + 39 <= r['idx'] <= cross + 80, (cross, r['idx'])
    print('ok  12 a 2M counts/s wrong-way push after 1M of drift aborts '
          '20 ms after it passes 1 x delta')


def test_13_slow_push_beyond_the_cap(mod):
    # A push slower than the follow rate cannot be told from drift, so
    # it is followed, MCU zero and all, until the baseline is 2.5M from
    # the tare; then the move is aborted with its own reason.
    for wrong in (False, True):
        s = descent(0., 34., 80000., 0., wrong=wrong)
        r = run_mcu(mod, DELTA_1, s)
        assert r['abort'] == DRIFT_CAP_REASON, (wrong, r)
        assert abs(r['idx'] - 31.25 * SPS) < 0.2 * SPS, r['idx']
    # a configured cap
    s = descent(0., 15., 80000., 0.)
    with zb.LogCapture() as cap:
        r = run_mcu(mod, DELTA_1, s, z_move_drift_cap_counts=900000)
    assert r['abort'] == DRIFT_CAP_REASON, r
    assert abs(r['idx'] - 11.25 * SPS) < 0.2 * SPS, r['idx']
    warn = [m for lv, m in cap.lines if lv == 'WARNING']
    assert any('baseline drifted more than z_move_drift_cap_counts' in m
               for m in warn), cap.lines
    print('ok  13 an 80000 counts/s push either way is followed to the '
          '2.5M cap (31.25 s) and aborted; a 900000 cap at 11.25 s')


def test_14_ceiling_from_the_current_baseline(mod):
    # 1M counts of press-way drift (followed), then the MCU trigger is
    # dead (a wrong scale, a sensor fault) and the bed is pushed at 3.5M
    # counts/s: the host ceiling stops it 2.5 x delta above the CURRENT
    # baseline, not 1M counts early (a fixed tare) or late.
    s = descent(-50000., 21., 3500000., 20.)
    r = run_mcu(mod, DELTA_1, s, mcu=False)
    assert r['abort'] == zb.CEILING_REASON, r
    cross = next(i for i in range(len(s)) if s[i][2] > 2.5 * DELTA_1)
    assert cross + 39 <= r['idx'] <= cross + 80, (cross, r['idx'])
    w = r['watch']
    assert abs(w._base - TARE_1 + 1000000) < 30000, w._base
    # a press to 2.3 x delta above the current baseline, held for 0.2 s
    # (the drift goes on under it: 10000 more), is not past the ceiling
    s = [(c, d, min(l, 2.3 * DELTA_1)) for c, d, l in s[:int(20.3 * SPS)]]
    s = [(int(TARE_1 + d + PRESS * l), d, l) for c, d, l in s]
    r = run_mcu(mod, DELTA_1, s, mcu=False)
    assert r['abort'] is None, r
    print('ok  14 with the trigger dead, the ceiling stops a press 2.5 x '
          'delta above the baseline after 1M of drift; 2.3 x does not')


def test_15_who_moves_the_mcu_zero(mod):
    s = descent(-50000., 5., noise=False)
    # a directed trigger: the zero follows in steps of at least 1000
    r = run_mcu(mod, DELTA_1, s)
    sent = r['sent']
    assert sent and all(abs(b - a) >= 1000 for a, b in zip(
        [TARE_1] + sent, sent)), sent[:5]
    assert abs(sent[-1] - (TARE_1 - 250000)) < 3000, sent[-1]
    # MCUTARE: the MCU took its own zero, so it is never moved, but the
    # host rules still follow the drift
    r = run_mcu(mod, DELTA_1, s, mcutare=True, mcu=False)
    assert r['sent'] == [] and r['abort'] is None, r
    assert abs(r['watch']._base - (TARE_1 - 250000)) < 1000
    # an abs_ge trigger (grams mode) and a rate of 0 keep the fixed tare
    for opts, direction in (({}, None),
                            ({'z_move_drift_rate_counts': 0}, PRESS)):
        r = run_mcu(mod, DELTA_1, descent(-20000., 2., noise=False),
                    mcu=False, direction=direction, **opts)
        assert r['sent'] == [] and r['watch']._base == TARE_1, (opts, r)
    # never after the descent: disarm drops the hook
    w, cell, homing = zb.make_watch(mod)
    sent = []
    w.arm(10., TARE_1, DELTA_1, PRESS, retare=sent.append)
    w.disarm()
    cell.feed(zb.rows(10., [TARE_1 - 100000] * 400))
    assert sent == [] and w._retare is None, sent
    # a failing send aborts the move (fails safe)
    w, cell, homing = zb.make_watch(mod)

    def broken(z):
        raise IOError('mcu gone')
    w.arm(10., TARE_1, DELTA_1, PRESS, retare=broken)
    cell.feed(zb.rows(10., [s_[0] for s_ in s[:400]]))
    assert homing.calls == [zb.FAILED_REASON], homing.calls
    print('ok  15 the MCU zero moves only for a directed trigger without '
          'MCUTARE, in steps of 1000+, never after disarm; a failed send '
          'aborts')


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


def test_16_probing_move_moves_the_sos_offset(mod):
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
    # LoadCellProbingMove.probing_move hands the watch a hook that sends
    # -baseline to the trigger's sos filter (the trigger is armed on
    # offset -tare, scale = press direction)
    tare = -800000
    cell = zb.Cell(SPS, counts_per_gram=1000., tare_counts=tare)
    m, cell, fake, w = zb.make_move(mod, 'trigger', tare=tare, cell=cell)
    fed = descent(-60000., 3., tare=tare, noise=False)

    class Ph(zb.Homing):
        def probing_move(self, endstop, pos, speed):
            for b in range(0, len(fed), 40):
                cell.feed(zb.rows(10. + b / float(SPS),
                                  [x[0] for x in fed[b:b + 40]]))
            return [0., 0., 1.2, 0.]
    m._printer.objects['homing'] = Ph()
    log = m._mcu_trigger_analog.log
    m.probing_move(zb.Gcmd(0))
    offs = [e for e in log if e[0] == 'set_offset_scale']
    assert offs[-1][1:4] == (800000, -1., 0), offs
    moved = [e[1] for e in log if e[0] == 'update_offset']
    assert len(moved) > 100, len(moved)
    assert abs(moved[-1] - (800000 + 180000)) < 2000, moved[-1]
    assert not w.is_armed() and w._retare is None
    # MCUTARE: the same descent moves nothing on the MCU
    del log[:]
    m._mcu_tare = mod.LoadCellProbingMove._mcu_tare.__get__(m)

    class Fresh(object):
        def start_collecting(self, min_time=None):
            pass

        def collect_min(self, n=1):
            return [[10., 0., tare]], None
    cell.get_collector = lambda: Fresh()
    m._approx_tare = tare
    m.probing_move(zb.Gcmd(1))
    assert [e for e in log if e[0] == 'update_offset'] == [], log
    print('ok  16 probing_move moves the trigger zero through '
          'MCU_SosFilter.update_offset (offset only, scale kept); not '
          'under MCUTARE')

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
             (test_9_rate_margin_and_peak_lag, mod),
             (test_10_sloe_heatup_either_way_then_contact, mod),
             (test_11_contact_after_2M_followed, mod),
             (test_12_fast_wrong_way_push_after_drift, mod),
             (test_13_slow_push_beyond_the_cap, mod),
             (test_14_ceiling_from_the_current_baseline, mod),
             (test_15_who_moves_the_mcu_zero, mod),
             (test_16_probing_move_moves_the_sos_offset, mod)]
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
