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
UNMESHED = 1.5


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
            self.bed_mesh.mesh = object()
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
    r.move(50., 50., 1.0)
    assert r.refused(60., 50., 1.0).startswith(NEAR_START)
    r.move(50., 50., 1.6)
    assert r.refused(60., 50., 1.6) is None
    # the mesh loads: Z is now height above the measured surface
    r.bed_mesh.mesh = object()
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
