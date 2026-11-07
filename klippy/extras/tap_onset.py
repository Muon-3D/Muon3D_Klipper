# Sample-accurate trigger time for ADXL345 tap homing (KAN-1361).
#
# The ADXL345 raises its INT pin when its own tap logic decides a tap has
# happened.  That decision lands after the first threshold crossing and is
# not tied to it: it is made at the end of the pulse (up to tap_dur later)
# and, when the structure rings, it can land one ring period late.  The MCU
# stops the move on that pin edge (safety path, untouched) but the stepper
# position recorded for the tap is read at the edge time, so the tap "moves"
# by speed * jitter between homes.
#
# These helpers take the time of the first accelerometer sample that crossed
# the tap threshold (linearly interpolated against the sample before it) and
# re-read the stepper positions at that time.  Pure functions: no klippy
# imports, so they can be unit tested on any host.


def find_tap_onset(samples, axes, thresh, trigger_time, window):
    """Return the print_time at which the tap signal first crossed `thresh`
    in the `window` seconds up to `trigger_time`, or None.

    samples: iterable of (print_time, x, y, z) in time order.
    axes: the sample axes the chip's tap logic watches, e.g. 'y'.
    None means "no usable onset": no sample crossed, or the signal was
    already over threshold at the start of the window (not a clean onset).
    The caller must then keep the hardware trigger time.
    """
    use = ['xyz'.index(a) + 1 for a in axes]
    if not use:
        return None
    t_lo = trigger_time - window
    prev_t = prev_v = None
    for s in samples:
        t = s[0]
        if t < t_lo:
            continue
        if t > trigger_time:
            break
        v = max(abs(s[i]) for i in use)
        if v >= thresh:
            if prev_t is None:
                return None
            if v == prev_v:
                return t
            frac = (thresh - prev_v) / (v - prev_v)
            return prev_t + frac * (t - prev_t)
        prev_t, prev_v = t, v
    return None


def refine_stepper_positions(stepper_positions, onset_time, trigger_time):
    """Re-read each stepper position at `onset_time` instead of at the
    hardware `trigger_time`.  Only moves the trigger earlier and never past
    the original trigger.  Returns True if positions were replaced."""
    if onset_time is None or not (0. < onset_time <= trigger_time):
        return False
    new = []
    for sp in stepper_positions:
        new.append(sp.stepper.get_past_mcu_position(onset_time))
    for sp, pos in zip(stepper_positions, new):
        sp.trig_pos = pos
    return True
