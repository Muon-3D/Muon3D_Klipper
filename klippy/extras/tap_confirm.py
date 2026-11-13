# Compare two X/Y home taps of adxl345_probe (KAN-1361).
#
# A tap home has no switch: the carriage runs into a hard stop and the
# ADXL345 raises INT on the impact.  When the impact is under the tap
# threshold nothing stops the move, the motors keep driving into the stop,
# and the field runs ahead of the blocked rotor until the motor slips a
# pole.  That jolt is what raises INT.  The MCU step count is then one
# electrical cycle (four full steps) ahead of the rotor, so the next tap
# reads up to one cycle further towards the stop: the "0.800mm" of
# KAN-1361.  Such a pair is a 'stall', not a disagreement about where the
# stop is.  It is still never accepted: only two taps within the tolerance
# in raw steps prove that both stopped at first contact.
# Pure module: no klippy imports, so it can be unit tested anywhere.

AGREE = 'agree'
STALL = 'stall'
DISAGREE = 'disagree'


def cycle_dist(rotation_distance, full_steps_per_rotation=200):
    """Travel of one electrical cycle (four full steps) in mm."""
    return 4. * rotation_distance / full_steps_per_rotation


def tap_dirs(start, trig):
    """Per stepper, +1/-1 for the way it stepped during the approach."""
    return dict((n, -1 if trig[n] < start[n] else 1) for n in trig)


def signed_delta(prev, new, dirs, step_dists):
    """How much further towards the stop the new tap triggered (mm).

    prev, new: {stepper: mcu step count at the trigger}.  The stepper
    with the largest move decides; negative means the new tap triggered
    earlier than the previous one.
    """
    best = 0.
    for name, sd in step_dists.items():
        d = (new[name] - prev[name]) * sd * dirs.get(name, 1)
        if abs(d) > abs(best):
            best = d
    return best


def classify(delta, tolerance, stall_dist):
    """AGREE, STALL or DISAGREE for the signed delta of two taps.

    STALL: the new tap triggered further into the stop, by no more than
    one electrical cycle (stall_dist; None or 0 turns the class off).
    """
    if abs(delta) <= tolerance:
        return AGREE
    if stall_dist and tolerance < delta <= stall_dist + tolerance:
        return STALL
    return DISAGREE


def phase(mcu_pos, phases, offset=0):
    """Microstep phase (0..phases-1) of an MCU step count."""
    return int(mcu_pos + offset) % phases


class Budget:
    """Retries left for a confirmation: stalls draw on their own budget
    first, anything else only on the general one."""

    def __init__(self, retries, stall_retries):
        self.retries = retries
        self.stall_retries = stall_retries
        self.used = 0

    def take(self, kind):
        if kind == STALL and self.stall_retries > 0:
            self.stall_retries -= 1
        elif self.retries > 0:
            self.retries -= 1
        else:
            return False
        self.used += 1
        return True
