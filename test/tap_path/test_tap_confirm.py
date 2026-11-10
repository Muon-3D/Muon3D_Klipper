#!/usr/bin/env python3
"""Tests for the KAN-1361 tap comparison (klippy/extras/tap_confirm.py).

    python3 test/tap_path/test_tap_confirm.py

Covers the pair classification against every Y tap pair logged on Boxwood
6-9 Nov 2026, a small model of a tap that is missed at first contact and
only trips when the motors slip a pole, and the M1 config values the fix
relies on. The confirm loop itself runs in klippy batch mode:
test/klippy/adxl345_probe_stall*.test and tap_confirm_mutation.py.
Standard library only.
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, 'klippy', 'extras'))
import tap_confirm as tc  # noqa: E402

TOL = 0.2
CYCLE = 0.8          # 4 full steps of 0.2 mm
STEP = 0.003125      # 64 microsteps
DISTS = {'stepper_x': STEP, 'stepper_y': STEP}


def test_cycle_is_four_full_steps():
    assert abs(tc.cycle_dist(40.) - CYCLE) < 1e-12
    assert abs(tc.cycle_dist(40., 400) - 0.4) < 1e-12


# Signed Y deltas of consecutive taps, from the "tap compare" lines of
# devin-briefs/assets/homing-1110 (y = (stepper_x - stepper_y) / 2).
BENCH = [
    # 6 Nov, 0.6 A: homes 1-2 pass, home 3 fails
    (0.000, tc.AGREE), (0.010, tc.AGREE),
    (0.8065, tc.STALL), (0.8035, tc.STALL), (0.803, tc.STALL),
    # 6 Nov, 3200 Hz
    (0.8035, tc.STALL), (0.8035, tc.STALL), (0.815, tc.STALL),
    # 7 Nov, confirm taps at 10 mm/s
    (0.547, tc.STALL), (0.741, tc.STALL), (0.303, tc.STALL),
    # 9 Nov, stepper_y 0.8 A: ten homes
    (-0.163, tc.AGREE), (0.0155, tc.AGREE), (0.010, tc.AGREE),
    (0.5905, tc.STALL), (0.2125, tc.STALL), (0.000, tc.AGREE),
    (0.0035, tc.AGREE), (0.016, tc.AGREE), (-0.1685, tc.AGREE),
    (0.015, tc.AGREE), (0.015, tc.AGREE), (-0.169, tc.AGREE),
    # 9 Nov evening, train 6 (spreads; all towards the stop)
    (0.703, tc.STALL), (0.266, tc.STALL), (0.800, tc.STALL),
    (0.231, tc.STALL), (0.231, tc.STALL),
]


def test_every_bench_pair_is_agree_or_stall():
    for delta, want in BENCH:
        got = tc.classify(delta, TOL, CYCLE)
        assert got == want, (delta, got, want)


def test_class_edges():
    assert tc.classify(0.2, TOL, CYCLE) == tc.AGREE
    assert tc.classify(-0.2, TOL, CYCLE) == tc.AGREE
    assert tc.classify(0.2001, TOL, CYCLE) == tc.STALL
    assert tc.classify(1.0, TOL, CYCLE) == tc.STALL
    assert tc.classify(1.0001, TOL, CYCLE) == tc.DISAGREE
    # earlier than the last tap is never a stall
    assert tc.classify(-0.2001, TOL, CYCLE) == tc.DISAGREE
    assert tc.classify(-0.8, TOL, CYCLE) == tc.DISAGREE
    # the class can be turned off
    assert tc.classify(0.8, TOL, 0.) == tc.DISAGREE
    assert tc.classify(0.8, TOL, None) == tc.DISAGREE


def test_signed_delta_follows_each_stepper_into_the_stop():
    # CoreXY Y home: the two steppers step opposite ways.
    for sx, sy in ((1, -1), (-1, 1)):
        start = {'stepper_x': 1000, 'stepper_y': -500}
        trig = {'stepper_x': 1000 + sx * 2560, 'stepper_y': -500 + sy * 2560}
        dirs = tc.tap_dirs(start, trig)
        assert dirs == {'stepper_x': sx, 'stepper_y': sy}
        further = {'stepper_x': trig['stepper_x'] + sx * 256,
                   'stepper_y': trig['stepper_y'] + sy * 256}
        d = tc.signed_delta(trig, further, dirs, DISTS)
        assert abs(d - CYCLE) < 1e-9, d
        assert abs(tc.signed_delta(further, trig, dirs, DISTS) + CYCLE) < 1e-9
        # one stepper alone still counts, with its sign
        one = dict(trig, stepper_y=trig['stepper_y'] - sy * 100)
        d = tc.signed_delta(trig, one, dirs, DISTS)
        assert abs(d + 100 * STEP) < 1e-9, d


def test_budget_keeps_stall_and_other_retries_apart():
    b = tc.Budget(2, 3)
    assert [b.take(tc.STALL) for _ in range(5)] == [True] * 5
    assert not b.take(tc.STALL) and b.used == 5
    b = tc.Budget(2, 3)
    assert [b.take(tc.DISAGREE) for _ in range(3)] == [True, True, False]
    # the stall retries are still there, but only for stalls
    assert b.take(tc.STALL) and not b.take(tc.DISAGREE)
    b = tc.Budget(2, 0)
    assert [b.take(tc.STALL) for _ in range(3)] == [True, True, False]


def test_phase_wraps_per_electrical_cycle():
    assert tc.phase(0, 256) == 0
    assert tc.phase(256, 256) == 0
    assert tc.phase(-1, 256) == 255
    assert tc.phase(300, 256, 5) == 49


# A model of the stop --------------------------------------------------------
# Positions in mm towards the stop; `slip` is how far the step count is
# ahead of the rotor (whole cycles lost so far). A tap at first contact
# triggers at the stop. A missed tap triggers `lead` past it, when the
# field has run far enough ahead of the blocked rotor for the motor to slip
# back one cycle; the count is then one more cycle ahead.

class Stop:
    def __init__(self, at=10.):
        self.at = at
        self.slip = 0.

    def tap(self, lead=None):
        if lead is None:
            return self.at + self.slip
        assert CYCLE / 2. < lead < CYCLE
        trig = self.at + self.slip + lead
        self.slip += CYCLE
        return trig

    def home_error(self, trig):
        # how far the stop really is from where this tap put it
        return self.at + self.slip - trig


def confirm(stop, taps, retries=2, stall_retries=6, samples=2,
            accept_modulo=False):
    """adxl345_probe._confirm_axis_home on the model. taps: the lead of
    each tap (None: at first contact). Returns (trig, taps used) or
    raises ValueError like the command error."""
    taps = list(taps)
    budget = tc.Budget(retries, stall_retries)
    accepted = [stop.tap(taps.pop(0))]
    used = 1
    while len(accepted) < samples:
        trig = stop.tap(taps.pop(0))
        used += 1
        delta = trig - accepted[-1]
        kind = tc.classify(delta, TOL, CYCLE)
        if accept_modulo and abs(delta - CYCLE) <= TOL:
            kind = tc.AGREE
        if kind == tc.AGREE:
            accepted.append(trig)
            continue
        if not budget.take(kind):
            raise ValueError("taps disagree by %.3f" % (abs(delta),))
        accepted = [trig]
    return accepted[-1], used


def test_model_reproduces_the_bench_deltas():
    # 6 Nov home 3: every tap missed, lead 0.48 -> +0.8 each (the ratchet)
    s = Stop()
    trigs = [s.tap(0.48) for _ in range(4)]
    deltas = [b - a for a, b in zip(trigs, trigs[1:])]
    assert all(abs(d - CYCLE) < 1e-9 for d in deltas), deltas
    # 9 Nov home 4: contact, missed (lead 0.59), contact, contact
    s = Stop()
    trigs = [s.tap(), s.tap(0.59), s.tap(), s.tap()]
    deltas = [round(b - a, 4) for a, b in zip(trigs, trigs[1:])]
    assert deltas == [0.59, 0.21, 0.0], deltas     # bench: 0.591 0.213 0.000


def test_old_budget_fails_the_bench_sequence_and_the_new_one_recovers():
    seq = [0.48, 0.48, 0.48, 0.48, None, None]
    try:
        confirm(Stop(), seq, stall_retries=0)
        raise AssertionError("2 retries cannot survive 3 stalls")
    except ValueError as e:
        assert '0.800' in str(e), e
    s = Stop()
    trig, used = confirm(s, seq)
    assert used == 6 and abs(s.home_error(trig)) < 1e-9


def test_an_accepted_home_is_always_two_taps_at_first_contact():
    # Every pattern of 8 taps, missed with any of three leads: whenever
    # the loop accepts, the home is exact, whatever was slipped before.
    leads = (None, 0.45, 0.59, 0.75)
    n_ok = n_fail = 0
    stack = [[]]
    while stack:
        seq = stack.pop()
        if len(seq) < 5:
            stack.extend(seq + [lead] for lead in leads)
            continue
        s = Stop()
        try:
            trig, used = confirm(s, seq + [0.59] * 6)
        except ValueError:
            n_fail += 1
            continue
        n_ok += 1
        assert abs(s.home_error(trig)) < 1e-9, (seq, s.home_error(trig))
    assert n_ok > 300 and n_fail > 0, (n_ok, n_fail)


def test_agreeing_modulo_one_cycle_would_home_in_the_wrong_place():
    # Why a regular +0.8 is not accepted: both taps were missed, and the
    # rotor sits 0.8 - lead off the stop when the count says it is on it.
    s = Stop()
    trig, used = confirm(s, [0.48, 0.48, 0.48], accept_modulo=True)
    assert used == 2
    assert abs(s.home_error(trig) - (CYCLE - 0.48)) < 1e-9
    assert s.home_error(trig) > TOL


# The M1 config the fix relies on ---------------------------------------------

def read_cfg(*parts):
    with open(os.path.join(ROOT, 'core', 'M1', *parts), 'rb') as fh:
        return fh.read().decode('utf-8').replace('\r\n', '\n')


def macro_body(text, name):
    m = re.search(r'^\[gcode_macro %s\]\n(.*?)(?=^\[)' % (re.escape(name),),
                  text, re.S | re.M)
    assert m, name
    return m.group(1)


def test_home_y_drives_both_motors_at_their_run_current():
    body = macro_body(read_cfg('macros', 'homing.cfg'), '_HOME_Y')
    before = body.split('G28.1 Y')[0]
    cur = dict(re.findall(
        r'^\s*SET_TMC_CURRENT STEPPER=(stepper_[xy]) CURRENT=([\d.]+)\s*$',
        before, re.M))
    assert cur == {'stepper_x': '0.8', 'stepper_y': '0.8'}, cur


def test_core_cfg_has_stall_retries_and_no_sample_trigger():
    text = read_cfg('core.cfg')
    m = re.search(r'^y_tap_confirm_stall_retries:\s*(\d+)\s*$', text, re.M)
    assert m and int(m.group(1)) >= 4, m
    # streaming the ADXL in every home caused the SPI fault of 9 Nov
    assert not re.search(r'^tap_trigger_from_samples:\s*True', text, re.M)
    # the tolerance is not what was changed
    assert re.search(r'^tap_confirm_tolerance:\s*0\.2\s*$', text, re.M)


if __name__ == '__main__':
    failed = 0
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            try:
                fn()
                print('PASS', name)
            except Exception as e:
                failed += 1
                print('FAIL', name, repr(e))
    sys.exit(1 if failed else 0)
