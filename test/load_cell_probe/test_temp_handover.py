"""The ADS1220 temperature read handed over inside a long armed leg
(KAN-4835), host side.

The mcu blanks the trigger across the read (src/sensor_muon_ads1220.c,
tested in test/muon_ads1220/fw_harness.c) and reports the window.  Here:

* the #152 temperature cap is lifted only with z_move_fast_temp_handover
  on AND an mcu that reports it can blank (and only above FAST_ABOVE: the
  legs below it are 1 s legs, never touched)
* the load backstop ignores the samples of a blank window, does not carry
  a run of samples across one, and still catches a real contact right
  after it (the force bound)
* negative control: the same stream without the window information makes
  the backstop fire on the glitch

    python3 test/load_cell_probe/test_temp_handover.py

Needs numpy.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import test_z_fast_legs as fl  # noqa: E402
import test_z_load_backstop as zb  # noqa: E402

zl, tg = fl.zl, fl.tg
TARE = -800000
CEILING = 552687     # 2.5 x DELTA_BOX_800
SPS = zb.SPS


def sensor_of(cell, pause=4.0, capable=True, windows=None):
    s = cell.sensor
    s.requests = 0
    s.windows = windows if windows is not None else []

    def request():
        s.requests += 1
    s.get_max_temperature_pause = lambda: pause
    s.request_temp_handover = request
    s.get_temp_handover = lambda: capable
    s.get_blank_windows = lambda: list(s.windows)
    return s


def test_1_the_cap_is_lifted_only_with_flag_and_capable_firmware(mod):
    got = {}
    for flag in (False, True):
        for capable in (False, True):
            w, cell, homing = zb.make_watch(
                mod, z_move_fast_temp_handover=flag)
            sensor_of(cell, capable=capable)
            got[(flag, capable)] = (w._temp_cap(), w.temp_handover())
    assert got[(False, False)] == (3.5, False), got
    assert got[(False, True)] == (3.5, False), got      # flag off: the cap
    assert got[(True, False)] == (3.5, False), got      # old firmware: cap
    assert got[(True, True)] == (None, True), got
    # the flag asks the chip for the handover at config time; off does not
    for flag in (False, True):
        w, cell, homing = zb.make_watch(mod)
        s = sensor_of(cell)
        mod.ZLoadBackstop(
            tg.FakeConfig(w._printer, {'z_move_fast_temp_handover': flag}),
            w._printer, cell)
        assert s.requests == (1 if flag else 0), (flag, s.requests)
    # a load cell that is not a muon_ads1220 never had a cap
    w, cell, _ = zb.make_watch(mod, z_move_fast_temp_handover=True)
    assert w._temp_cap() is None and not w.temp_handover()
    print('ok  1 cap 3.5 s unless the flag is on and the firmware can '
          'blank (%s)' % ', '.join(
              '%s/%s: %s' % (k[0], k[1], v[0])
              for k, v in sorted(got.items())))


def descend_legs(mod, flag, capable):
    w = fl.known_world(drift=zl.ramp(0.), noise=False)
    g = zl.Gcmd(FAST_ABOVE=fl.ABOVE)
    opts = dict(fl.REST)
    opts['z_move_fast_temp_handover'] = flag
    m, ph = zl.make(mod, w, opts=opts, gcmd=g)
    fl.with_ads(m, 4.0)
    m._load_cell.sensor.get_temp_handover = lambda: capable
    err = None
    try:
        m.probing_move(g)
    except tg.CommandError as e:
        err = str(e)
    assert err is None, err
    return w, m


def test_2_long_legs_above_fast_above_only(mod):
    out = {}
    for flag, capable in ((False, True), (True, False), (True, True)):
        w, m = descend_legs(mod, flag, capable)
        above = [(a - b) / fl.SPEED for a, b in w.segments
                 if a > fl.ABOVE + 1e-6]
        below = [(a - b) / fl.SPEED for a, b in w.segments
                 if a <= fl.ABOVE + 1e-6]
        out[(flag, capable)] = (max(above), max(below), len(w.segments))
        # no leg crosses FAST_ABOVE, and below it they are 1 s legs
        for a, b in w.segments:
            assert not (a > fl.ABOVE + 1e-6 and b < fl.ABOVE - 1e-6)
        assert max(below) <= 1.0 + 1e-6, (flag, capable, below)
    assert out[(False, True)][0] <= 3.5 + 1e-6, out
    assert out[(True, False)][0] <= 3.5 + 1e-6, out
    assert 3.5 + 1e-6 < out[(True, True)][0] <= 4.0 + 1e-6, out
    assert out[(True, True)][2] <= out[(False, True)][2], out
    print('ok  2 legs above Z%g: %.2f s capped, %.2f s with the handover '
          '(%d vs %d legs); below it 1 s' % (
              fl.ABOVE, out[(False, True)][0], out[(True, True)][0],
              out[(True, True)][2], out[(False, True)][2]))


def test_2b_with_a_longer_fast_leg_max_it_is_one_long_leg(mod):
    # the handover only lifts the temperature cap; the leg length is still
    # z_move_fast_leg_max and the drift budget (a bench setting)
    legs = {}
    for flag in (False, True):
        w = fl.known_world(drift=zl.ramp(0.), noise=False)
        g = zl.Gcmd(FAST_ABOVE=fl.ABOVE)
        opts = dict(fl.REST)
        opts.update({'z_move_fast_temp_handover': flag,
                     'z_move_fast_leg_max': 10.})
        m, ph = zl.make(mod, w, opts=opts, gcmd=g)
        fl.with_ads(m, 4.0)
        m._load_cell.sensor.get_temp_handover = lambda: True
        m.probing_move(g)
        legs[flag] = fl.leg_seconds(w)
    assert max(legs[False][:-1]) <= 3.5 + 1e-6, legs
    assert max(legs[True]) > 5., legs
    assert len(legs[True]) < len(legs[False]), legs
    print('ok  2b fast_leg_max 10: legs %s s capped, %s s with the '
          'handover' % ('/'.join('%.1f' % x for x in legs[False]),
                        '/'.join('%.1f' % x for x in legs[True])))


def stream(n_rest, n_win, n_after, win_value, after_value, t0=10.):
    """Rows: rest at the tare, a window's worth of glitch, then after."""
    counts = ([TARE] * n_rest + [win_value] * n_win
              + [after_value] * n_after)
    r = zb.rows(t0, counts)
    w0 = r[n_rest][0] - 0.2 / SPS
    w1 = r[n_rest + n_win - 1][0] + 0.2 / SPS
    return r, (w0, w1)


def feed(mod, rows, window, flag=True, step=40):
    w, cell, homing = zb.make_watch(mod, z_move_fast_temp_handover=flag)
    seen = []
    orig = w._judge
    w._judge = lambda c, t=None: (seen.append(c), orig(c, t))[1]
    sensor_of(cell, windows=[(window[0], window[1], 0)] if window else [])
    w.arm(10., TARE, zb.DELTA_BOX_800, -1.)
    for i in range(0, len(rows), step):
        cell.feed(rows[i:i + step])
        if homing.calls:
            break
    return w, homing, len(seen) - 1 if homing.calls else None


def test_3_a_blank_window_is_not_load_and_negative_control(mod):
    # 30 ms (60 samples) of glitch, more than the 40 in a row that abort
    glitch = TARE - CEILING - 200000
    rows, win = stream(400, 60, 400, glitch, TARE)
    w, homing, _ = feed(mod, rows, win)
    assert homing.calls == [], homing.calls
    assert w._blank_skipped == 60, w._blank_skipped
    # negative control: without the window the same samples abort
    w, homing, _ = feed(mod, rows, None)
    assert homing.calls == [zb.CEILING_REASON], homing.calls
    # and a glitch on the wrong side is not wrong-way load either
    rows, win = stream(400, 60, 400, TARE + 400000, TARE)
    w, homing, _ = feed(mod, rows, win)
    assert homing.calls == [], homing.calls
    w, homing, _ = feed(mod, rows, None)
    assert homing.calls == [zb.WRONG_WAY_REASON], homing.calls
    print('ok  3 60 glitch samples in a blank window are ignored; the '
          'same stream with no window aborts')


def test_4_a_real_contact_is_caught_right_after_the_window(mod):
    # the contact begins inside the window and goes on: the samples after
    # it are over the ceiling, so the abort comes on the 40th of them
    # (20 ms), as for a contact anywhere else; none inside is judged
    over = TARE - CEILING - 1000
    n_rest, n_win = 400, 24         # 12 ms, the nominal window
    rows, win = stream(n_rest, n_win, 200, over, over)
    w, homing, idx = feed(mod, rows, win, step=1)
    assert homing.calls == [zb.CEILING_REASON], homing.calls
    assert idx == n_rest + 39, idx
    assert w._blank_skipped == n_win, w._blank_skipped
    print('ok  4 a contact through the window aborts on the 40th sample '
          'after it (+%.0f ms, the confirm time of any ceiling abort)'
          % (40 * 1000. / SPS))


def test_5_a_run_is_not_carried_across_a_window(mod):
    over = TARE - CEILING - 1000
    counts = ([TARE] * 400 + [over] * 30 + [over] * 24 + [over] * 30
              + [TARE] * 200)
    r = zb.rows(10., counts)
    win = (r[430][0] - 0.2 / SPS, r[453][0] + 0.2 / SPS)
    w, homing, _ = feed(mod, r, win)
    assert homing.calls == [], homing.calls
    assert w._blank_resets == 1, w._blank_resets
    # negative control: the window samples simply dropped, no window info
    w, homing, _ = feed(mod, r[:430] + r[454:], None)
    assert homing.calls == [zb.CEILING_REASON], homing.calls
    print('ok  5 30 + 30 samples over the ceiling either side of a window '
          'make no run (glued, without the window, they abort)')


def test_6_without_the_flag_windows_are_not_consulted(mod):
    glitch = TARE - CEILING - 200000
    rows, win = stream(400, 60, 400, glitch, TARE)
    w, homing, _ = feed(mod, rows, win, flag=False)
    assert homing.calls == [zb.CEILING_REASON], homing.calls
    print('ok  6 flag off: the backstop judges every sample as before')


def main():
    mod = tg.load_module(tg._paths.patched(
        'klippy/extras/load_cell_probe.py'))
    tests = [test_1_the_cap_is_lifted_only_with_flag_and_capable_firmware,
             test_2_long_legs_above_fast_above_only,
             test_2b_with_a_longer_fast_leg_max_it_is_one_long_leg,
             test_3_a_blank_window_is_not_load_and_negative_control,
             test_4_a_real_contact_is_caught_right_after_the_window,
             test_5_a_run_is_not_carried_across_a_window,
             test_6_without_the_flag_windows_are_not_consulted]
    failed = []
    for t in tests:
        try:
            t(mod)
        except AssertionError as e:
            failed.append(t.__name__)
            print('FAIL %s: %s' % (t.__name__, e))
    if failed:
        print('%d of %d temperature handover tests FAILED: %s'
              % (len(failed), len(tests), ', '.join(failed)))
        sys.exit(1)
    print('All %d temperature handover tests passed' % (len(tests),))


if __name__ == '__main__':
    main()
