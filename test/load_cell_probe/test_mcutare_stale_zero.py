"""MCUTARE and a stale host zero (KAN-4770).

A mesh tap sent with MCUTARE=1 has the MCU zero itself on the first
descent sample; the HOST arms the load watch on the zero the previous tap
left (_approx_tare).  On Sloe a second hot mesh began after a reheat with
no tap in between; the cell had drifted by hundreds of thousands of
counts, the watch armed on that old zero and ended the first descent with
"the load on the nozzle exceeded the safety ceiling".  A false abort.

Now MCUTARE is used only while that zero is at most mcutare_max_zero_age
(5 s) old; otherwise the tap takes the host tare, which refreshes it.
The ceiling itself (2.5 x the trigger delta) is not touched.

This drives the real LoadCellProbingMove (probing_move,
_host_zero_is_fresh, _mcu_tare, _measure_tare, _apply_tare) and the real
ZLoadBackstop in the simulated world of test_z_descent.py.

    python3 test/load_cell_probe/test_mcutare_stale_zero.py

Needs numpy.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import test_z_descent as zl  # noqa: E402

zb = zl.zb
tg = zl.tg
SHIFT = 600000      # the cell sits this far off the last zero (wrong way)
CEILING = 'safety ceiling'


class WorldMcu(object):
    """estimated_print_time(monotonic) is the world's clock, the clock the
    samples (and so _approx_tare_t) are stamped on."""

    def __init__(self, world):
        self.w = world

    def estimated_print_time(self, t):
        return self.w.t


def build(mod, world, mcu=True, **kw):
    m, ph = zl.make(mod, world, mcu=mcu, **kw)
    m._nozzle_is_clear = lambda: True
    mcu_obj = WorldMcu(world)
    m._load_cell.sensor.get_mcu = lambda: mcu_obj
    m.paths = []
    host, mcutare = m._pause_and_tare, m._mcu_tare

    def pause_and_tare(gcmd):
        m.paths.append('host')
        return host(gcmd)

    def mcu_tare(gcmd):
        m.paths.append('mcu')
        return mcutare(gcmd)
    m._pause_and_tare = pause_and_tare
    m._mcu_tare = mcu_tare
    return m, ph


def tap(m, w, mcutare=1, wait=0., shift=0):
    """One tap from 10 mm up, `wait` seconds after the previous one; the
    cell has moved by `shift` counts meanwhile.  Returns the error."""
    w.real = 10.
    w.t += wait
    w.tare0 += shift
    try:
        m.probing_move(zl.Gcmd(MCUTARE=mcutare))
    except tg.CommandError as e:
        return str(e)
    return None


def first_tap(m, w):
    # a tap that leaves a fresh host zero (MCUTARE=0).  The real macro sends
    # MCUTARE=1 even on a mesh's first tap; that case (an MCUTARE tap on an
    # old zero) is test 1's second tap.
    assert tap(m, w, mcutare=0) is None
    assert m.paths == ['host'] and m._approx_tare_t is not None
    del m.paths[:]


def test_1_stale_zero_big_shift_no_press(mod):
    w = zl.World(real=10.)
    m, ph = build(mod, w)
    first_tap(m, w)
    old = m._approx_tare
    armed = []
    orig = m._z_watch.arm

    def arm(*a, **k):
        armed.append((m._tare_counts, k.get('mcutare')))
        return orig(*a, **k)
    m._z_watch.arm = arm
    with zb.LogCapture() as cap:
        err = tap(m, w, wait=60., shift=SHIFT)
    assert err is None, err
    assert m.paths == ['host'], m.paths
    assert ph.calls == [], ph.calls
    # armed on the fresh zero, not the one from 60 s ago
    assert armed and abs(armed[0][0] - (old + SHIFT)) < 20000, (armed, old)
    assert armed[0][1] is False, armed
    lines = [x for lv, x in cap.lines if 'MCUTARE asked for' in x]
    assert len(lines) == 1 and '60.' in lines[0] and '5.0 s' in lines[0], \
        cap.lines
    print('ok  1 MCUTARE with a 60 s old zero and a %d count shift: host '
          'tare, armed on the fresh zero, no false ceiling trip' % SHIFT)


def test_2_real_press_still_caught(mod):
    # a dead MCU trigger, so only the host watch can end the press
    w = zl.World(real=10.)
    m, ph = build(mod, w, mcu=False)
    err = tap(m, w, mcutare=0)
    assert err is not None and CEILING in err, err
    del m.paths[:]
    # stale zero: a host tare, and the press past the ceiling is caught
    err = tap(m, w, wait=60., shift=SHIFT)
    assert m.paths == ['host'], m.paths
    assert err is not None and CEILING in err, err
    # fresh zero: the MCUTARE path is used and still catches it
    del m.paths[:]
    err = tap(m, w, wait=1.)
    assert m.paths == ['mcu'], m.paths
    assert err is not None and CEILING in err, err
    # the ceiling is what it was: 2.5 x the trigger delta
    watch = m._z_watch
    assert watch._ceiling_factor == 2.5, watch._ceiling_factor
    assert watch.get_status()['z_backstop_ceiling_factor'] == 2.5
    assert m._trigger_delta > 0
    print('ok  2 a real press past 2.5 x delta is caught, on the host-tare '
          'path and on the MCUTARE path with a fresh zero')


def test_3_fresh_zero_uses_mcutare(mod):
    w = zl.World(real=10.)
    m, ph = build(mod, w)
    first_tap(m, w)
    with zb.LogCapture() as cap:
        assert tap(m, w, wait=1.) is None
    assert m.paths == ['mcu'], m.paths
    assert not [x for lv, x in cap.lines if 'MCUTARE asked for' in x]
    # stale -> host tare -> the next tap, 1 s later, is MCUTARE again
    del m.paths[:]
    assert tap(m, w, wait=60., shift=SHIFT) is None
    assert m.paths == ['host'], m.paths
    assert tap(m, w, wait=1.) is None
    assert m.paths == ['host', 'mcu'], m.paths
    assert tap(m, w, wait=1.) is None
    assert m.paths == ['host', 'mcu', 'mcu'], m.paths
    # a zero a hair under the limit is still fresh, a hair over is not
    m._approx_tare_t = w.t - 4.9
    assert m._host_zero_is_fresh()
    m._approx_tare_t = w.t - 5.1
    assert not m._host_zero_is_fresh()
    print('ok  3 a fresh zero keeps MCUTARE (no host tare); stale -> host '
          'tare -> the next tap is MCUTARE again')


def test_4_age_check_off_and_unknown_age(mod):
    w = zl.World(real=10.)
    m, ph = build(mod, w)
    assert m._mcutare_max_zero_age == 5.
    first_tap(m, w)
    m._mcutare_max_zero_age = 0.
    assert tap(m, w, wait=600.) is None
    assert m.paths == ['mcu'], m.paths
    m._approx_tare_t = None
    assert tap(m, w, wait=1.) is None      # off: the old behaviour
    assert m.paths == ['mcu', 'mcu'], m.paths
    m._mcutare_max_zero_age = 5.
    m._approx_tare_t = None
    del m.paths[:]
    assert tap(m, w, wait=1.) is None
    assert m.paths == ['host'], m.paths
    assert m._approx_tare_t is not None    # a host tare stamps the zero
    assert tap(m, w, wait=1.) is None
    assert m.paths == ['host', 'mcu'], m.paths
    # no MCUTARE asked for: always a host tare, as before
    del m.paths[:]
    assert tap(m, w, mcutare=0, wait=1.) is None
    assert m.paths == ['host'], m.paths
    print('ok  4 mcutare_max_zero_age 0 keeps the old behaviour; an unknown '
          'zero time takes a host tare, which stamps it')


def main():
    mod = tg.load_module(tg._paths.patched(
        'klippy/extras/load_cell_probe.py'))
    tests = [test_1_stale_zero_big_shift_no_press,
             test_2_real_press_still_caught,
             test_3_fresh_zero_uses_mcutare,
             test_4_age_check_off_and_unknown_age]
    failed = []
    for t in tests:
        try:
            t(mod)
        except AssertionError as e:
            failed.append(t.__name__)
            print('FAIL %s: %s' % (t.__name__, e))
    if failed:
        print('%d of %d MCUTARE stale zero tests FAILED: %s'
              % (len(failed), len(tests), ', '.join(failed)))
        sys.exit(1)
    print('All %d MCUTARE stale zero tests passed' % (len(tests),))


if __name__ == '__main__':
    main()
