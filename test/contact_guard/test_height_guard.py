"""contact_guard move_guard: height, the default move guard.

The guard refuses an XY move only on the KNOWN plate height: Z homed and
not invalidated since, and the move's lowest point below the contact
height.  With the height not known it says "Home first".  The load cell is
not consulted: on Boxwood (12 Nov 2026) its reading swung between -908000
and +441000 counts during plain XY moves with the plate 20 mm away, against
a contact threshold of 90000, and the old guard refused the fifth move.

    python3 -m pytest test/contact_guard/test_height_guard.py

The fakes are the ones test_contact_guard.py drives the load cell model
with, so the guard under test is the real ContactGuard.
"""
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_contact_guard as base  # noqa: E402

# Written out, not imported: these are what the tests are about.
HOME_FIRST = ("Home first: the plate height is not known, so X and Y"
              " cannot move safely. Run a home (G28).")
NEAR_START = "The plate is at the nozzle (Z "
OLD_REFUSAL = "Nozzle is touching the plate: lift Z before moving X or Y"
CONTACT_Z = 0.1
UNMESHED = 2.5


class FakeMesh:
    """A probed region, as bed_mesh's ZMesh reports it."""
    def __init__(self, x=(10., 190.), y=(10., 190.)):
        self.mesh_x_min, self.mesh_x_max = x
        self.mesh_y_min, self.mesh_y_max = y


class FakeBedMesh:
    def __init__(self):
        self.mesh = None
    def get_mesh(self):
        return self.mesh


class Rig(base.Rig):
    """The default guard (no move_guard set), homed unless told not to."""
    def __init__(self, z=20., homed=True, mesh=False, values=None):
        self.bed_mesh = FakeBedMesh()
        if mesh:
            self.bed_mesh.mesh = FakeMesh()
        values = dict(values or {})
        values.setdefault("move_guard", "height")
        orig = base.FakePrinter
        rig = self

        class Printer(orig):
            def __init__(self):
                orig.__init__(self)
                self.objects['bed_mesh'] = rig.bed_mesh
        base.FakePrinter = Printer
        try:
            base.Rig.__init__(self, values, z=z)
        finally:
            base.FakePrinter = orig
        if homed:
            self.home_z()


def test_default_mode_is_height():
    r = Rig(values={})
    r.guard = None
    printer = base.FakePrinter()
    guard = base.contact_guard.load_config(base.FakeConfig(printer, {}))
    assert guard.move_guard == 'height'


def test_no_false_refusal_with_a_wild_load_cell():
    # The Boxwood session, made worse: 30 simulated minutes of XY moves at
    # Z 20 while the load cell reads contact-level loads, swings both ways,
    # drifts and drops out
    r = Rig(z=20.)
    rng = random.Random(1112)
    refusals = []
    for i in range(1800):
        load = rng.choice([0, 441101, -908147, 679018, 150000, -149364])
        r.feed(load, 0.05, drift=rng.uniform(-400000, 400000))
        if i % 97 == 0:
            # a gap in the stream
            r.n += int(2.5 * base.SPS)
            r.printer.reactor.now = getattr(r.printer.reactor, 'now', 0.)
        err = r.refused(rng.uniform(5, 195), rng.uniform(40, 170), 20.)
        if err is not None:
            refusals.append((i, err))
    assert refusals == [], refusals[:3]
    # the load cell did read contact through this; the guard ignored it
    assert r.guard.contact or r.guard.load != 0


def test_old_guard_refuses_the_same_session():
    # Negative control for the test above: the load cell model, fed the
    # same thing, does refuse.  So that test can fail.
    r = base.Rig(z=20.)
    r.home_z()
    r.feed(0, 0.5)
    r.feed(441101, 0.1)
    assert r.refused(60., 60., 20.) == OLD_REFUSAL


def test_not_homed_says_home_first():
    r = Rig(z=20., homed=False)
    assert r.refused(60., 60., 20.) == HOME_FIRST
    # also when the move lifts: with the height unknown no XY is safe
    assert r.refused(60., 60., 30.) == HOME_FIRST
    assert r.status()['home_required'] is True
    assert r.status()['z_known'] is False
    assert r.status()['contact'] is False
    # Z alone is not the business of this guard
    assert r.refused(50., 50., 25.) is None


def test_homed_axes_without_trust_says_home_first():
    # An aborted G28 on the M1 leaves homed_axes "xyz" with placeholder
    # positions (SET_KINEMATIC_POSITION); toolhead "homed" is not enough
    r = Rig(z=20., homed=False)
    assert r.printer.toolhead.homed_axes == 'xyz'
    assert r.refused(60., 60., 20.) == HOME_FIRST


def test_position_reset_loses_the_height():
    r = Rig(z=20.)
    assert r.refused(60., 60., 20.) is None
    # SET_KINEMATIC_POSITION with a different Z, outside homing
    r.printer.toolhead.position[2] = 110.
    r.set_position()
    assert r.refused(70., 60., 110.) == HOME_FIRST
    r.home_z()
    assert r.refused(70., 60., 110.) is None


def test_z_motor_off_loses_the_height():
    r = Rig(z=20.)
    r.z_motor(False)
    r.z_motor(True)
    assert r.refused(60., 60., 20.) == HOME_FIRST


def test_force_move_of_z_loses_the_height():
    r = Rig(z=20.)
    r.printer.send_event("force_move:manual_move", base.FakeStepper('x'))
    assert r.refused(60., 60., 20.) is None
    r.printer.send_event("force_move:manual_move", base.FakeStepper('z'))
    assert r.refused(70., 60., 20.) == HOME_FIRST


def test_z_unhomed_says_home_first():
    r = Rig(z=20.)
    r.printer.toolhead.homed_axes = 'xy'
    assert r.refused(60., 60., 20.) == HOME_FIRST


def test_failed_probe_loses_the_height():
    r = Rig(z=20.)
    hmove = base.FakeHomingMove('z', 5.)
    r.printer.send_event("homing:homing_move_begin", hmove)
    r.printer.send_event("homing:homing_move_end", hmove)
    # "No trigger on probe after full movement"
    r.report(base.CommandError("No trigger on probe after full movement"))
    assert r.refused(60., 60., 20.) == HOME_FIRST


def test_unrelated_error_keeps_the_height():
    r = Rig(z=20.)
    assert r.refused(60., 60., 20.) is None
    r.report(base.CommandError("Unknown command:\"NOT_A_CMD\""))
    r.report(base.CommandError("Move out of range"), levels=3)
    assert r.refused(70., 60., 20.) is None
    # and a refusal of its own does not cost the height either
    r.move(70., 60., 0.)        # Z only, down to the plate
    assert r.refused(80., 60., 0.).startswith(NEAR_START)
    r.move(70., 60., 20.)
    assert r.refused(80., 60., 20.) is None


def test_refuses_only_below_the_contact_height():
    r = Rig(z=20., mesh=True)
    assert r.status()['min_xy_z'] == CONTACT_Z
    r.move(50., 50., CONTACT_Z)
    assert r.refused(60., 50., CONTACT_Z) is None     # at it: allowed
    assert r.status()['contact'] is False
    r.move(60., 50., 0.05)
    err = r.refused(70., 50., 0.05)
    assert err is not None and err.startswith(NEAR_START), err
    assert "lift Z before moving X or Y" in err
    assert r.status()['contact'] is True
    # a diagonal that climbs out is still dragging at its start
    assert r.refused(70., 50., 5.).startswith(NEAR_START)
    # Z alone clears it
    assert r.refused(60., 50., 5.) is None
    assert r.refused(70., 50., 5.) is None
    assert r.status()['contact'] is False
    # a diagonal that descends below it is refused on its end
    assert r.refused(80., 50., 0.).startswith(NEAR_START)


def test_without_a_mesh_keeps_the_margin():
    r = Rig(z=20., mesh=False)
    assert abs(r.status()['min_xy_z'] - (CONTACT_Z + UNMESHED)) < 1e-9
    r.move(50., 50., 2.0)       # KAN-219: the plate peaks ~2 mm up
    assert r.refused(60., 50., 2.0).startswith(NEAR_START)
    r.move(50., 50., 2.6)
    assert r.refused(60., 50., 2.6) is None
    # the mesh loads: Z is now height above the measured surface
    r.bed_mesh.mesh = FakeMesh()
    r.move(60., 50., 0.2)
    assert r.refused(70., 50., 0.2) is None


def test_boxwood_print_end_park_is_allowed():
    # 12 Nov 2026, Boxwood: "refused move [64.125, 64.601, 8.914] ->
    # [195.0, 10.0, 8.914], load 679018" and the print never parked
    r = Rig(z=8.914383453975141, mesh=True)
    r.guard.last_position = [64.125, 64.601, 8.914383453975141, 0.]
    r.feed(0, 0.5)
    r.feed(679018, 0.2)
    assert r.guard.contact, "the load cell model does call this contact"
    assert r.refused(195., 10., 8.914383453975141) is None
    assert r.status()['load_contact'] is True
    assert r.status()['contact'] is False


def test_never_refuses_in_a_job_or_while_homing():
    r = Rig(z=20., homed=False, mesh=True)
    r.printer.print_stats.state = 'printing'
    assert r.refused(60., 60., 0.0) is None
    r.printer.print_stats.state = 'standby'
    r.printer.homing_scope.in_scope = True
    assert r.refused(70., 60., 0.0) is None
    r.printer.homing_scope.in_scope = False
    r.printer.nozzle_wipe.active = True
    assert r.refused(80., 60., 0.0) is None
    r.printer.nozzle_wipe.active = False
    assert r.refused(90., 60., 0.0) == HOME_FIRST


def test_disabled_guard_does_not_refuse():
    r = Rig(z=20., homed=False)
    r.guard.enabled = False
    assert r.refused(60., 60., 20.) is None
    assert r.status()['enabled'] is False
    assert r.status()['home_required'] is False


def test_bad_mode_is_a_config_error():
    printer = base.FakePrinter()
    try:
        base.contact_guard.load_config(
            base.FakeConfig(printer, {'move_guard': 'loadcell'}))
    except base.ConfigError:
        return
    raise AssertionError("a misspelt move_guard was accepted")


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_'):
            fn()
            print("ok  " + name)


def test_out_of_range_after_a_good_probe_keeps_the_height():
    # Review of #129: "Move out of range" is raised further down the
    # chain, after the guard accepted the move.  It is not a failed probe.
    r = Rig(z=20.)
    r.homing_move('z', 5.)          # a PROBE that triggered
    th = r.printer.toolhead
    orig = th.move

    def out_of_range(newpos, speed):
        raise base.CommandError("Move out of range")
    th.move = out_of_range
    assert r.refused(999., 60., 20.) == "Move out of range"
    th.move = orig
    assert r.refused(60., 60., 20.) is None


def test_homed_flag_lost_and_put_back_is_not_a_height():
    # CLEAR_HOMED=Z (or a failed G28) and then SET_HOMED=Z on the same
    # number: the flag is back, the height is not
    r = Rig(z=20.)
    r.printer.toolhead.homed_axes = 'xy'
    assert r.status()['z_known'] is False       # a status poll sees it go
    r.printer.toolhead.homed_axes = 'xyz'
    r.set_position()                            # Z unchanged
    assert r.refused(60., 60., 20.) == HOME_FIRST
    r.home_z()
    assert r.refused(60., 60., 20.) is None


# Option A (Jack, 12 Nov): "XY homing lifts the plate first, so nothing
# scratches unless Z is moved down later".  A homing move that lifted the
# plate vouches for XY at that height until Z moves down or is let go.

LIFT = -10.     # the G28 macro's AXIS_PROTECTED_MOVE Z+10 (travel < 0 is up)


def g28_xy(r, lift=LIFT):
    """G28 X Y on the M1: the macro fakes Z to 100, lifts 10 mm as a homing
    move, clears Z's homed flag, then homes Y and X"""
    p = r.printer
    p.send_event("homing_scope:begin")
    p.homing_scope.in_scope = True
    p.toolhead.position[2] = 100.
    r.set_position()
    r.homing_move('z', lift)
    p.toolhead.position[2] = 100. - lift
    p.toolhead.homed_axes = 'xy'
    r.homing_move('y', 5.)
    r.homing_move('x', 5.)
    p.send_event("homing_scope:end")
    p.homing_scope.in_scope = False
    r.guard.get_position()


def test_lift_lets_xy_move_with_the_height_unknown():
    r = Rig(z=20., homed=False)
    assert r.refused(60., 60., 20.) == HOME_FIRST
    g28_xy(r)
    st = r.status()
    assert st['z_known'] is False
    assert st['lift_clear'] is True
    assert st['home_required'] is False
    z = r.printer.toolhead.position[2]
    assert r.refused(60., 60., z) is None          # CENTER
    assert r.refused(100., 190., z) is None        # FRONT
    assert r.refused(120., 100., z + 5.) is None   # lifting as it goes


def test_z_down_after_the_lift_needs_a_home():
    r = Rig(z=20., homed=False)
    g28_xy(r)
    z = r.printer.toolhead.position[2]
    # a diagonal that comes down is refused, and does not run
    assert r.refused(60., 60., z - 1.) == HOME_FIRST
    assert r.status()['lift_clear'] is True
    # Z alone down: allowed (not this guard's business), but it ends it
    r.move(r.guard.last_position[0], r.guard.last_position[1], z - 1.)
    assert r.status()['lift_clear'] is False
    assert r.status()['home_required'] is True
    assert r.refused(70., 60., z - 1.) == HOME_FIRST
    # back up again is not enough: the lift was the evidence, not Z
    r.move(r.guard.last_position[0], r.guard.last_position[1], z + 5.)
    assert r.refused(80., 60., z + 5.) == HOME_FIRST


def test_cut_filament_fakes_z_zero_after_the_lift():
    # CUT_FILAMENT and FILAMENT_INSERT_FULL_PURGE: G28 X Y, then
    # SET_KINEMATIC_POSITION Z=0 SET_HOMED=Z, then XY moves at that "Z 0"
    r = Rig(z=20., homed=False)
    g28_xy(r)
    th = r.printer.toolhead
    th.position[2] = 0.
    th.homed_axes = 'xyz'
    r.set_position()
    r.guard.get_position()
    assert r.refused(116., 10., 0.) is None
    assert r.refused(138., 10., 0.) is None
    assert r.refused(116., 10., 0.) is None
    # and the fake is no height: once Z goes down from there, Home first
    r.move(116., 10., -0.5)
    assert r.refused(120., 10., -0.5) == HOME_FIRST


def test_park_printer_fakes_z_fifty_after_a_z_park():
    # PARK_PRINTER with Z unhomed: Z_PARK's AXIS_HOME_MOVE lifts to the top,
    # then SET_KINEMATIC_POSITION SET_HOMED=Z Z=50 and G0 X135 Y63
    r = Rig(z=20., homed=False)
    r.homing_move('z', -60.)
    th = r.printer.toolhead
    th.position[2] = 50.
    r.set_position()
    r.guard.get_position()
    assert r.refused(135., 63., 50.) is None


def test_a_short_lift_is_not_enough():
    # Less than lift_clear_min (5 mm)
    r = Rig(z=20., homed=False)
    g28_xy(r, lift=-4.0)
    assert r.status()['lift_clear'] is False
    z = r.printer.toolhead.position[2]
    assert r.refused(60., 60., z) == HOME_FIRST


def test_motors_off_force_move_and_bed_removal_end_the_lift():
    for lose in ('motor', 'force_move', 'bed'):
        r = Rig(z=20., homed=False)
        g28_xy(r)
        z = r.printer.toolhead.position[2]
        assert r.refused(60., 60., z) is None
        if lose == 'motor':
            r.z_motor(False)
        elif lose == 'force_move':
            r.printer.send_event("force_move:manual_move",
                                 base.FakeStepper('z'))
        else:
            r.printer.send_event("bed_removal_detector:removed")
        assert r.refused(70., 60., z) == HOME_FIRST, lose
        assert r.status()['home_required'] is True, lose


def test_failed_home_after_the_lift_needs_a_home():
    # Boxwood, 12 Nov: "y home taps disagree by 0.931mm"
    r = Rig(z=20., homed=False)
    p = r.printer
    p.send_event("homing_scope:begin")
    p.homing_scope.in_scope = True
    r.homing_move('z', LIFT)
    hmove = base.FakeHomingMove('y', 5.)
    p.send_event("homing:homing_move_begin", hmove)
    r.report(base.CommandError("adxl345_probe: y home taps disagree by"
                               " 0.931mm (tolerance 0.200mm)"))
    p.homing_scope.in_scope = False
    assert r.refused(60., 60., 30.) == HOME_FIRST


def test_unrelated_error_after_a_good_g28_keeps_the_lift():
    # homing_scope:end means the G28 succeeded; a typo next is not its
    # failure
    r = Rig(z=20., homed=False)
    g28_xy(r)
    r.report(base.CommandError("Unknown command:\"CENTRE\""))
    z = r.printer.toolhead.position[2]
    assert r.refused(60., 60., z) is None


def test_unrelated_error_after_a_good_full_g28_keeps_the_height():
    r = Rig(z=20., homed=False)
    p = r.printer
    p.send_event("homing_scope:begin")
    p.homing_scope.in_scope = True
    r.home_z()
    p.send_event("homing_scope:end")
    p.homing_scope.in_scope = False
    r.report(base.CommandError("Unknown command:\"CENTRE\""))
    assert r.status()['z_known'] is True
    assert r.refused(60., 60., 20.) is None


def test_probe_down_ends_the_lift_and_a_good_home_takes_over():
    # Full G28: lift, Y, X, then Z homes by probing down
    r = Rig(z=20., homed=False)
    g28_xy(r)
    r.homing_move('z', 100.)            # a probe travelling down
    assert r.status()['lift_clear'] is False
    r.printer.toolhead.homed_axes = 'xyz'
    r.home_z()
    assert r.status()['z_known'] is True
    assert r.refused(60., 60., 20.) is None


def m1_home_z(r, second_probe_fails=False):
    """_LOAD_CELL_HOME_Z, the M1's homing_override: soft-home Z to its
    maximum, probe, set Z from the probe, G1 Z1, probe again, set Z, G1 Z10"""
    p = r.printer
    th = p.toolhead
    p.homing_override.in_script = True
    th.position[2] = 150.
    r.set_position()                    # SET_KINEMATIC_POSITION Z=max
    r.homing_move('z', 140.)            # PROBE from clear travel
    th.position[2] = 10.
    r.set_position()                    # _HOME_Z_FROM_LAST_PROBE
    th.position[2] = 11.                # G1 Z1 (relative)
    if second_probe_fails:
        hmove = base.FakeHomingMove('z', 3.)
        p.send_event("homing:homing_move_begin", hmove)
        p.homing_override.in_script = False
        r.report(base.CommandError("No trigger on probe after full"
                                   " movement"), levels=3)
        return
    r.homing_move('z', 1.)              # PROBE at standard speed
    th.position[2] = 0.
    r.set_position()
    th.position[2] = 10.                # G1 Z10
    p.homing_override.in_script = False
    r.guard.get_position()


def test_m1_z_home_through_the_override_is_trusted():
    r = Rig(z=20., homed=False)
    m1_home_z(r)
    assert r.status()['z_known'] is True
    assert r.refused(60., 60., 10.) is None
    r.move(60., 60., 1.0)
    assert r.refused(60., 70., 1.0).startswith(NEAR_START)


def test_m1_z_home_with_a_failed_second_probe_is_not():
    r = Rig(z=20., homed=False)
    m1_home_z(r, second_probe_fails=True)
    assert r.status()['z_known'] is False
    r.guard.get_position()
    assert r.refused(60., 60., 11.) == HOME_FIRST


def test_g28_x_after_a_good_home_moves_on_the_lift():
    # G28 X with Z homed: the macro's lift fakes Z to 100 and clears Z's
    # homed flag, so the height is gone; the lift still clears the plate
    r = Rig(z=20.)
    assert r.status()['z_known'] is True
    g28_xy(r)
    assert r.status()['z_known'] is False
    z = r.printer.toolhead.position[2]
    assert r.refused(60., 60., z) is None
    r.move(60., 60., z - 2.)
    assert r.refused(70., 60., z - 2.) == HOME_FIRST


def test_outside_the_mesh_keeps_the_margin():
    # Review of #129: an adaptive mesh stays loaded after its print; past
    # its region the plate can be ~2 mm higher (KAN-219)
    r = Rig(z=20., mesh=True)
    r.bed_mesh.mesh = FakeMesh(x=(60., 140.), y=(60., 140.))
    r.move(100., 100., 20.)
    r.move(100., 100., 0.5)
    assert r.refused(120., 120., 0.5) is None
    assert r.status()['min_xy_z'] == CONTACT_Z
    # leaving the region at that height is refused
    err = r.refused(150., 120., 0.5)
    assert err is not None and err.startswith(NEAR_START), err
    # and so is coming in from outside it
    r.move(120., 120., 3.)
    r.move(150., 120., 3.)
    assert abs(r.status()['min_xy_z'] - (CONTACT_Z + UNMESHED)) < 1e-9
    r.move(150., 120., 0.5)
    assert r.refused(120., 120., 0.5).startswith(NEAR_START)


def test_bed_removal_loses_the_height():
    r = Rig(z=20.)
    assert r.refused(60., 60., 20.) is None
    r.printer.send_event("bed_removal_detector:removed")
    assert r.refused(70., 60., 20.) == HOME_FIRST
    r.home_z()
    assert r.refused(70., 60., 20.) is None


def test_bed_removal_leaves_the_load_cell_model_alone():
    r = base.Rig(z=20.)
    r.home_z()
    assert r.guard.z_trusted
    r.printer.send_event("bed_removal_detector:removed")
    assert r.guard.z_trusted


# Review of 5ad0de0c: Z moves that bypass G-code, and the rest


def test_z_park_retract_after_the_lift_still_holds():
    # PARK_PRINTER with Z unhomed: Z_PARK's AXIS_HOME_MOVE lifts to the
    # top, then its manual_move retracts 10 mm; the plate is still far
    # above where the lift began
    r = Rig(z=20., homed=False)
    r.homing_move('z', -60.)
    r.z_steps(-10.)
    th = r.printer.toolhead
    th.position[2] = 50.
    r.set_position()
    r.guard.get_position()
    assert r.refused(135., 63., 50.) is None


def test_z_lowered_behind_the_guard_ends_the_lift():
    # The G28 X Y lift, then a manual_move (an AXIS_HOME_MOVE retract after
    # a park that stalled at once) takes the plate back down 10 mm: no
    # G-code move went down, but the steppers did
    r = Rig(z=20., homed=False)
    g28_xy(r)
    z = r.printer.toolhead.position[2]
    assert r.refused(60., 60., z) is None
    r.z_steps(-10.)
    waits = r.printer.toolhead.waits
    assert r.refused(70., 60., z) == HOME_FIRST
    assert r.printer.toolhead.waits > waits, "decided before the queue ran"
    assert r.status()['lift_clear'] is False
    assert r.status()['home_required'] is True


def test_z_lowered_a_little_behind_the_guard_keeps_the_lift():
    # Still lift_clear_min (5 mm) above where the 10 mm lift began
    r = Rig(z=20., homed=False)
    g28_xy(r)
    r.z_steps(-4.)
    z = r.printer.toolhead.position[2]
    assert r.refused(60., 60., z) is None
    r.z_steps(-1.5)
    assert r.refused(70., 60., z) == HOME_FIRST


def test_status_sees_a_lowered_lift_without_a_move():
    r = Rig(z=20., homed=False)
    g28_xy(r)
    assert r.status()['lift_clear'] is True
    r.z_steps(-10.)
    assert r.status()['lift_clear'] is False


def test_refused_z_down_keeps_the_lift():
    # "Must home axis first" from further down the chain: nothing moved
    r = Rig(z=20., homed=False)
    g28_xy(r)
    z = r.printer.toolhead.position[2]
    th = r.printer.toolhead
    orig = th.move

    def must_home(newpos, speed):
        raise base.CommandError("Must home axis first: 60.000 60.000 0.000")
    th.move = must_home
    x, y = r.guard.last_position[:2]
    try:
        r.move(x, y, z - 5.)
    except base.CommandError as err:
        r.report(err)
    th.move = orig
    assert r.status()['lift_clear'] is True
    assert r.refused(60., 60., z) is None


def test_error_inside_g28_drops_the_height_without_a_poll():
    # G28 fails (homing_scope clears xyz), then SET_HOMED=Z on the same Z:
    # no status poll in between
    r = Rig(z=20.)
    p = r.printer

    class Trapq:
        # The executed Z before the reset is the Z after it
        def get_trapq_position(self, print_time):
            return list(p.toolhead.position), 0.

    class MotionReport:
        dtrapqs = {'toolhead': Trapq()}
    p.objects['motion_report'] = MotionReport()
    p.send_event("homing_scope:begin")
    p.homing_scope.in_scope = True
    # _HOME_X's plain G1 back-off is a G-code move the guard accepts (it
    # clears the homing mark); the error comes after it
    r.move(50., 60., 20.)
    r.report(base.CommandError("Move out of range: 50.000 -3.000 20.000"))
    p.homing_scope.in_scope = False
    r.set_position()        # SET_KINEMATIC_POSITION SET_HOMED=Z, Z same
    assert r.refused(60., 60., 20.) == HOME_FIRST


def test_mesh_from_before_a_bed_removal_is_not_trusted():
    r = Rig(z=20., mesh=True)
    r.move(100., 100., 20.)
    r.move(100., 100., 0.5)
    assert r.refused(110., 100., 0.5) is None
    r.printer.send_event("bed_removal_detector:removed")
    r.home_z()
    assert abs(r.status()['min_xy_z'] - (CONTACT_Z + UNMESHED)) < 1e-9
    assert r.refused(120., 100., 0.5).startswith(NEAR_START)
    # a new mesh (PRINT_START re-probes) is trusted again
    r.bed_mesh.mesh = FakeMesh()
    assert r.refused(120., 100., 0.5) is None


def test_lift_clear_min_cannot_be_below_the_unmeshed_minimum():
    printer = base.FakePrinter()
    try:
        base.contact_guard.load_config(base.FakeConfig(
            printer, {'lift_clear_min': 1.0}))
    except base.ConfigError:
        return
    raise AssertionError("lift_clear_min 1.0 was accepted")
