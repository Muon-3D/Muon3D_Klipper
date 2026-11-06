"""KAN-1361: the recorded tap position must follow the sample that crossed
the threshold, not the time the chip raised INT.

The ADXL345 INT edge can land well after the tap (a 13.3ms spread was
measured on Boxwood at 30 and 60 mm/s).  The test feeds a synthetic 3200 Hz
sample stream with the tap at many offsets inside a 13.3ms window and a
stepper that moves at the homing speed, and asserts:
  * the onset time is within one sample period of the true tap;
  * the stepper positions of two taps at different offsets agree within one
    sample period of travel, where the INT-time positions differ by up to
    the whole window (the old behaviour);
  * with no usable onset the INT positions are left alone (a missed tap
    still fails upstream: no trigger -> no refinement).
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _paths  # noqa: E402

sys.argv = sys.argv[:1]
sys.path.insert(0, os.path.dirname(_paths.patched('klippy/extras/tap_onset.py')))
import tap_onset  # noqa: E402

RATE = 3200.
DT = 1. / RATE
QUANTUM = 0.8 / 60.          # 13.33ms, measured
THRESH = 15000.              # mm/s^2, y_tap_thresh
SPEED = 60.                  # mm/s
STEP_DIST = 0.003125         # mm/step, Y
WINDOW = 0.06


def stream(t_tap, t_end=0.5, amp=40000., ring_hz=77.):
    """Quiet, then a tap at t_tap that rings (decaying) for ~50ms."""
    import math
    out = []
    n = int(t_end * RATE)
    for i in range(n):
        t = i * DT
        v = 800. * math.sin(2 * math.pi * 1300. * t)      # noise floor
        if t >= t_tap:
            dt = t - t_tap
            v += amp * math.exp(-dt / 0.02) * math.sin(
                2 * math.pi * ring_hz * dt + 1.2)
        out.append((t, 0., v, 0.))
    return out


def int_time(t_tap):
    """Hardware trigger time: the next quantum boundary after the tap."""
    k = int(t_tap / QUANTUM) + 1
    return k * QUANTUM


class FakeStepper:
    def __init__(self):
        self.asked = []

    def get_past_mcu_position(self, t):
        self.asked.append(t)
        return int(round(SPEED * t / STEP_DIST))


class SP:
    def __init__(self, hw_time):
        self.stepper = FakeStepper()
        self.trig_pos = self.stepper.get_past_mcu_position(hw_time)


def offsets():
    # tap placed at 12 positions across one quantum, far from t=0
    base = 0.2 + 0.0003
    return [base + QUANTUM * f / 12. for f in range(12)]


def test_onset_within_one_sample():
    worst = 0.
    for t_tap in offsets():
        s = stream(t_tap)
        hw = int_time(t_tap)
        onset = tap_onset.find_tap_onset(s, 'y', THRESH, hw, WINDOW)
        assert onset is not None, t_tap
        err = abs(onset - t_tap)
        worst = max(worst, err)
        assert err <= DT + 1e-9, (t_tap, onset, err)
    return worst


def test_int_time_alone_is_not_that_accurate():
    # The old behaviour: the position is read at the INT time.  Its error
    # exceeds one sample period (this is the failure the fix removes).
    errs = [int_time(t) - t for t in offsets()]
    assert max(errs) > 10 * DT, max(errs)


def test_two_taps_agree_after_refine():
    t_a, t_b = offsets()[1], offsets()[9]
    old_a, old_b = int_time(t_a), int_time(t_b)
    # taps at different physical offsets inside one quantum make the INT
    # times differ; model the head arriving at the same wall position each
    # time (same t_tap relative to a move that is shifted by the INT quantum)
    res = []
    for t_tap, hw in ((t_a, old_a), (t_b, old_b)):
        sp = SP(hw)
        onset = tap_onset.find_tap_onset(stream(t_tap), 'y', THRESH, hw,
                                         WINDOW)
        assert tap_onset.refine_stepper_positions([sp], onset, hw)
        # position of the stepper relative to where the tap really was
        res.append((sp.trig_pos * STEP_DIST - SPEED * t_tap, hw))
    for err, hw in res:
        assert abs(err) <= SPEED * DT + STEP_DIST, err
    # and the old positions were off by up to a whole quantum of travel
    old_err = max(SPEED * (hw - t) for t, hw in ((t_a, old_a), (t_b, old_b)))
    assert old_err > 5 * SPEED * DT, old_err


def test_no_onset_keeps_int_position():
    quiet = [(i * DT, 0., 100., 0.) for i in range(1600)]
    hw = 0.3
    assert tap_onset.find_tap_onset(quiet, 'y', THRESH, hw, WINDOW) is None
    sp = SP(hw)
    before = sp.trig_pos
    assert not tap_onset.refine_stepper_positions([sp], None, hw)
    assert sp.trig_pos == before


def test_already_over_threshold_is_not_an_onset():
    loud = [(i * DT, 0., 30000., 0.) for i in range(1600)]
    assert tap_onset.find_tap_onset(loud, 'y', THRESH, 0.3, WINDOW) is None


def test_onset_never_later_than_int():
    sp = SP(0.3)
    before = sp.trig_pos
    assert not tap_onset.refine_stepper_positions([sp], 0.31, 0.3)
    assert sp.trig_pos == before


def test_other_axis_ignored():
    s = [(i * DT, 30000. if i > 1200 else 0., 0., 0.) for i in range(1600)]
    assert tap_onset.find_tap_onset(s, 'y', THRESH, 0.4, WINDOW) is None
    assert tap_onset.find_tap_onset(s, 'x', THRESH, 0.4, WINDOW) is not None


if __name__ == '__main__':
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            try:
                fn()
                print('PASS', name)
            except Exception as e:  # noqa: BLE001
                fails += 1
                print('FAIL', name, repr(e))
    sys.exit(1 if fails else 0)
