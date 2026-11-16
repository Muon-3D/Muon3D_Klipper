"""Cap on the Z search distance of a probing move (KAN-3370, guard B).

The M1 homes Z with PROBE from a soft-homed Z of 160 down to z_min -4: a
164 mm search.  On Boxwood a probe fault meant it never triggered and the
whole 164 mm lifted the bed off its mount.  A normal home travels 0.4 to
30 mm.  PrinterHoming.probing_move now limits the Z travel to
max_search_distance_z and says why it stopped.  The cap is OFF by default
(0): the M1's Z search is blind from a soft-homed Z=160 and a re-home from
the Z140 park needs about 140 mm, so a 40 mm cap broke a normal G28.  Every
test that exercises capping sets 40 explicitly.

Drives the real HomingMove and PrinterHoming with fakes (see fakes.py).
Constants are written out here, not imported from the code under test.

    python3 test/homing_abort/test_search_cap.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import fakes  # noqa: E402
from fakes import Rig, check, run  # noqa: E402

CAP_MSG = ("Z homing stopped: no contact within 40 mm. Check the bed is"
           " fitted and nothing is wrong with the nozzle")
PROBE_MSG = ("Z probing stopped: no contact within 40 mm. Check the bed is"
             " fitted and nothing is wrong with the nozzle")
STOCK_MSG = "No trigger on probe after full movement"


class FakeScope:
    def __init__(self, in_scope):
        self.in_scope = in_scope


CAP40 = {'max_search_distance_z': 40}


def homing_rig(**kw):
    kw.setdefault('options', CAP40)
    rig = Rig(**kw)
    rig.printer.objects['homing_scope'] = FakeScope(True)
    return rig


def probe(rig, target_z=-4.):
    return rig.phoming.probing_move(rig.endstops[0],
                                    [0., 0., target_z, 0.], 15.)


def test_b1_z_search_is_capped_at_40mm():
    rig = homing_rig(start=160.)
    res, err = run(lambda: probe(rig))
    check(len(rig.toolhead.drip_calls) == 1, "B1: one search move")
    movepos = rig.toolhead.drip_calls[0][0]
    check(movepos[2] == 120.0,
          "B1: commanded target Z is exactly 120 (got %r)" % (movepos[2],))
    check(movepos[2] != -4.0 and movepos[2] != 1.0,
          "B1: not the 164 mm (or 169 mm) full travel")
    check(rig.steppers[0].pos == 12000,
          "B1: steppers travelled exactly 4000 steps = 40 mm (at %d)"
          % (rig.steppers[0].pos,))
    check(rig.toolhead.travelled_mm == 40, "B1: travelled exactly 40 mm")
    check(err == CAP_MSG, "B1: error is %r (got %r)" % (CAP_MSG, err))
    check(res is None, "B1: no probe position was returned")
    check(rig.toolhead.pos[2] == 120.0,
          "B1: toolhead Z is where the steppers stopped, 120")
    # Outside a G28 scope the wording says probing, the cap still holds
    rig = Rig(start=160., options=CAP40)
    res, err = run(lambda: probe(rig))
    check(err == PROBE_MSG, "B1: outside G28 error is %r (got %r)"
          % (PROBE_MSG, err))
    check(rig.toolhead.drip_calls[0][0][2] == 120.0,
          "B1: outside G28 the move is still capped at 40 mm")


def test_b2_cap_off_uses_the_full_travel():
    rig = homing_rig(start=160., options={'max_search_distance_z': 0})
    res, err = run(lambda: probe(rig))
    check(rig.toolhead.drip_calls[0][0][2] == -4.0,
          "B2: target Z is the full -4 (got %r)"
          % (rig.toolhead.drip_calls[0][0][2],))
    check(rig.toolhead.travelled_mm == 164, "B2: travelled the full 164 mm")
    check(err == STOCK_MSG, "B2: stock error text (got %r)" % (err,))
    # The code default (no option) is off, and it runs the full travel
    rig = homing_rig(start=160., options={})
    check(rig.phoming.max_search_distance_z == 0.0,
          "B2: default max_search_distance_z is 0 (off)")
    res, err = run(lambda: probe(rig))
    check(rig.toolhead.drip_calls[0][0][2] == -4.0
          and rig.toolhead.travelled_mm == 164 and err == STOCK_MSG,
          "B2: with no option a blind 160 -> -4 search runs the full 164 mm")
    rig = homing_rig(start=140., options={})
    res, err = run(lambda: probe(rig))
    check(rig.toolhead.travelled_mm == 144 and err == STOCK_MSG,
          "B2: a re-home from the Z140 park is not capped by default")


def test_b3_x_and_y_travel_is_unchanged():
    for axis, name in ((0, 'X'), (1, 'Y')):
        rig = Rig(axis=axis, start=0.)
        target = [0., 0., 0., 0.]
        target[axis] = 200.
        res, err = run(lambda: rig.phoming.manual_home(
            rig.toolhead, [(rig.endstops[0], name.lower())], target, 60.,
            True, True))
        check(rig.toolhead.drip_calls[0][0][axis] == 200.0,
              "B3: %s homing target is the full 200" % (name,))
        check(rig.toolhead.travelled_mm == 200,
              "B3: %s homing travelled the full 200 mm" % (name,))
        check(err == "No trigger on %s after full movement"
              % (name.lower(),),
              "B3: %s stock error text (got %r)" % (name, err))
        # and a probing move that does not move Z is not capped either
        rig = Rig(axis=axis, start=0.)
        res, err = run(lambda: rig.phoming.probing_move(
            rig.endstops[0], target, 60.))
        check(rig.toolhead.drip_calls[0][0][axis] == 200.0,
              "B3: %s probing move target is the full 200" % (name,))
        check(err == STOCK_MSG, "B3: %s probing stock error (got %r)"
              % (name, err))
    # Z homed by home_rails/manual_home is not a probing move: not capped
    rig = Rig(start=160.)
    res, err = run(lambda: rig.phoming.manual_home(
        rig.toolhead, [(rig.endstops[0], 'z')], [0., 0., -4., 0.], 15.,
        True, True))
    check(rig.toolhead.travelled_mm == 164,
          "B3: manual_home on Z is not touched by the probing cap")


def test_b4_trigger_within_the_cap_homes_normally():
    for opts in ({}, {'max_search_distance_z': 0}, CAP40):
        tag = "cap %s" % (opts.get('max_search_distance_z', 'default'),)
        rig = homing_rig(start=160., trigger_after_mm=12, options=opts)
        res, err = run(lambda: probe(rig))
        check(err is None, "B4 %s: no error (got %r)" % (tag, err))
        check(res is not None and abs(res[2] - 148.0) < 1e-9,
              "B4 %s: probe position is Z 148 (got %r)" % (tag, res))
        check(rig.toolhead.travelled_mm == 12,
              "B4 %s: stopped at the 12 mm trigger" % (tag,))
        check(rig.endstops[0].triggered, "B4 %s: endstop triggered" % (tag,))
    rig = homing_rig(start=160., trigger_after_mm=39, options=CAP40)
    res, err = run(lambda: probe(rig))
    check(err is None and abs(res[2] - 121.0) < 1e-9,
          "B4: a trigger at 39 mm, just inside the cap, is fine")


def test_cap_edges():
    # Exactly at the cap is not capped and gives the stock text
    rig = homing_rig(start=160.)
    res, err = run(lambda: probe(rig, 120.))
    check(rig.toolhead.drip_calls[0][0][2] == 120.0 and err == STOCK_MSG,
          "edge: a 40 mm search is not capped (got %r)" % (err,))
    # An upward search is capped the same way
    rig = homing_rig(start=100.)
    res, err = run(lambda: probe(rig, 160.))
    check(rig.toolhead.drip_calls[0][0][2] == 140.0 and err == CAP_MSG,
          "edge: an upward 60 mm search stops at +40 (got %r)" % (err,))
    # A move with X/Y as well keeps its direction
    rig = homing_rig(start=100.)
    rig.toolhead.pos[0] = 10.
    res, err = run(lambda: rig.phoming.probing_move(
        rig.endstops[0], [60., 0., 0., 0.], 15.))
    movepos = rig.toolhead.drip_calls[0][0]
    check(movepos[2] == 60.0 and abs(movepos[0] - 30.0) < 1e-9,
          "edge: a diagonal search is scaled along its line (got %r)"
          % (movepos[:3],))
    # A different failure under the cap is not relabelled
    rig = homing_rig(start=160.,
                     wait_errors=[fakes.CommandError("Communication timeout"
                                                     " during homing")])
    res, err = run(lambda: probe(rig))
    check(err == "Error during homing probe: Communication timeout during"
          " homing", "edge: a comms error keeps its own text (got %r)"
          % (err,))


def main():
    test_b1_z_search_is_capped_at_40mm()
    test_b2_cap_off_uses_the_full_travel()
    test_b3_x_and_y_travel_is_unchanged()
    test_b4_trigger_within_the_cap_homes_normally()
    test_cap_edges()
    fakes.finish()


if __name__ == '__main__':
    main()
