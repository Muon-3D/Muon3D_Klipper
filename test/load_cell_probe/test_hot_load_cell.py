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

The Z abort (1) is now handled by descending in legs, each with its own
tare: test_z_legs.py.  This file keeps the mesh fix (2): the real
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
    probe = rf.load_module(rf._paths.patched('klippy/extras/probe.py'),
                           'hot_probe')
    tests = [(test_4_logged_tap_pairs_pass_the_shipped_tolerance, probe)]
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
