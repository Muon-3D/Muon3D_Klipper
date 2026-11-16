"""KAN-1731: outside dev mode a CLIENT move may not go below the plate-side
Z floor once Z is homed, nor cross the notch (bed_mesh no_go_region_1,
x 75..125 y 0..34) below no_go_traverse_z.  Macros (homing, probing,
purge) are the printer's own and are not checked.

Drives the real GCodeDispatch and muon_guard through test_muon_guard's Rig,
with a gcode_move that tracks G90/G91/G92 the way gcode_move.py does.

    PYTHONUTF8=1 python3 test/muon_guard/test_move_limits.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_muon_guard as tmg  # noqa: E402
from extras import move_limits  # noqa: E402

FAILURES = tmg.FAILURES
check = tmg.check


class Toolhead:
    def __init__(self, homed):
        self.homed = homed

    def get_status(self, eventtime):
        return {'homed_axes': self.homed}


def rig(homed='xyz', pos=(100., 100., 10.), **values):
    r = tmg.Rig(values=values or None)
    r.printer.add_object('toolhead', Toolhead(homed))
    gm = r.gm
    gm.last_position = list(pos) + [0.]
    gm.base_position = [0., 0., 0., 0.]
    gm.absolute_coord = True

    # follow the moves the way gcode_move.py does, so a script can chain
    def g1(gcmd):
        r.calls.append(gcmd.get_commandline())
        p = gcmd.get_command_parameters()
        for i, a in enumerate('XYZ'):
            if a in p:
                v = float(p[a])
                gm.last_position[i] = (v + gm.base_position[i]
                                       if gm.absolute_coord
                                       else gm.last_position[i] + v)

    def g90(gcmd):
        gm.absolute_coord = True

    def g91(gcmd):
        gm.absolute_coord = False

    def g92(gcmd):
        p = gcmd.get_command_parameters()
        for i, a in enumerate('XYZ'):
            if a in p:
                gm.base_position[i] = gm.last_position[i] - float(p[a])
    for name, fn in (('G0', g1), ('G1', g1), ('G90', g90), ('G91', g91),
                     ('G92', g92)):
        r.gcode.ready_gcode_handlers.pop(name, None)
        r.gcode.register_command(name, fn)
    r.gcode.gcode_handlers = r.gcode.ready_gcode_handlers
    return r


def refused(r, script, origin='webhooks'):
    err = r.client(script, origin)
    return err is not None and 'Refused' in err


def test_core_cfg_matches_bed_mesh_notch():
    guard = tmg.read_core_section('muon_guard')
    mesh = tmg.read_core_section('bed_mesh')
    check(guard.get('client_no_go_traverse_z') == mesh['no_go_traverse_z'],
          "muon_guard traverse Z is bed_mesh's no_go_traverse_z")
    rect = [float(v) for v in guard.get('client_no_go_region', '').split(',')]
    want = [float(v) for v in (mesh['no_go_region_1_min'] + ','
                               + mesh['no_go_region_1_max']).split(',')]
    check(rect == want, "muon_guard notch %r is bed_mesh's %r" % (rect, want))
    check(float(guard.get('client_z_floor', 'nan')) == 0.,
          "core.cfg sets client_z_floor: 0")


def test_z_floor():
    r = rig()
    check(refused(r, 'G1 Z-0.5'), "G1 Z-0.5 refused once Z is homed")
    check(refused(r, 'G1 Z-4 F600'), "G1 Z-4 (position_min) refused")
    check(r.client('G1 Z0.2') is None, "G1 Z0.2 passes")
    check(r.client('G1 Z0') is None, "G1 Z0 passes (the floor itself)")
    check(refused(r, 'G91\nG1 Z-0.3'), "relative move below the floor refused")
    r = rig(pos=(100., 100., 1.))
    check(refused(r, 'G92 Z10\nG1 Z0'),
          "G92 cannot be used to move the floor")
    r = rig()
    r.gm.homing_position[2] = -0.1      # a babystep the offset limit allows
    r.gm.base_position[2] = -0.1
    check(r.client('G1 Z0.2') is None, "babystepped first layer passes")
    r = rig(homed='xy')
    check(r.client('G1 Z-1') is None, "Z not homed: the floor does not apply")
    r = rig(pos=(100., 100., -0.5))
    check(r.client('G1 Z-0.2') is None, "a move up toward the floor passes")
    check(r.client('G1 X120') is None, "an XY move at the current Z passes")


def test_notch():
    r = rig(pos=(100., 50., 2.))
    check(refused(r, 'G1 X100 Y10'), "into the notch below Z6 refused")
    r = rig(pos=(60., 20., 2.))
    check(refused(r, 'G1 X140 Y20'), "across the notch below Z6 refused")
    r = rig(pos=(60., 20., 2.))
    check(refused(r, 'G1 X140 Y20 Z7'), "a move that starts low is refused")
    r = rig(pos=(60., 20., 8.))
    check(r.client('G1 X140 Y20') is None, "across above Z6 passes")
    r = rig(pos=(60., 20., 2.))
    check(r.client('G1 X60 Y60') is None, "beside the notch passes")
    r = rig(pos=(75., 50., 2.))
    check(r.client('G1 X75 Y0') is None, "along the notch edge passes")
    r = rig(pos=(100., 20., 2.))
    check(r.client('G1 Z8') is None, "straight up out of the notch passes")
    r = rig(pos=(60., 20., 2.))
    check(refused(r, 'G91\nG1 X80'), "relative crossing refused")
    r = rig(homed='z', pos=(60., 20., 2.))
    check(r.client('G1 X140') is None, "XY not homed: no notch check")


def test_arc_through_the_notch():
    # half circle centred at (100, 40), r=40, from (60,40) to (140,40):
    # G3 (counter-clockwise) goes under, through (100, 0) and the notch;
    # G2 goes over the top through (100, 80)
    r = rig(pos=(60., 40., 2.))
    check(refused(r, 'G3 X140 Y40 I40 J0'), "an arc through the notch")
    r = rig(pos=(60., 40., 2.))
    check(r.client('G2 X140 Y40 I40 J0') is None,
          "the other way round it stays clear (y >= 40)")


def test_printed_file_lines_are_checked():
    r = rig()
    check(refused(r, 'G1 Z-1', origin='file'), "a file line below the floor")


def test_macros_are_not_checked():
    r = rig(pos=(100., 50., 2.))
    r.from_macro('G1 X100 Y10\nG1 Z-1')
    check(r.calls == ['G1 X100 Y10', 'G1 Z-1'],
          "homing / probe / purge macros move freely: %r" % (r.calls,))
    r = rig(pos=(100., 50., 2.))
    r.internal('G1 Z-1')
    check(r.calls == ['G1 Z-1'], "internal sources are not checked")


def test_dev_mode_allows():
    r = tmg.Rig(dev=True)
    r.printer.add_object('toolhead', Toolhead('xyz'))
    check(r.client('G1 Z-3') is None, "dev mode: no floor")


def test_geometry():
    rect = (75., 0., 125., 34.)
    hit = move_limits.segment_hits_rect
    check(hit((60., 20.), (140., 20.), rect), "straight across")
    check(not hit((60., 40.), (140., 40.), rect), "above it")
    check(not hit((75., 50.), (75., 0.), rect), "on the edge")
    check(hit((100., 20.), (100., 20.), rect), "a point inside")
    pts = move_limits.arc_points((60., 40.), (140., 40.), (40., 0.), False)
    check(min(p[1] for p in pts) < 1., "ccw arc goes through y=0: %r"
          % (min(p[1] for p in pts),))
    pts = move_limits.arc_points((60., 40.), (140., 40.), (40., 0.), True)
    check(min(p[1] for p in pts) > 39. and max(p[1] for p in pts) > 79.,
          "cw arc goes over the top")
    pts = move_limits.arc_points((60., 40.), (60., 40.), (40., 0.), True)
    check(len(pts) > 200, "start == end is a full circle")


def main():
    tests = [v for k, v in sorted(globals().items())
             if k.startswith('test_') and callable(v)]
    for t in tests:
        try:
            t()
        except Exception as e:
            import traceback
            traceback.print_exc()
            FAILURES.append("%s raised %s: %s" % (t.__name__,
                                                  type(e).__name__, e))
    if FAILURES:
        print("\n%d FAILURE(S)" % (len(FAILURES),))
        for f in FAILURES:
            print(" - " + f)
        return 1
    print("ok: %d tests" % (len(tests),))
    return 0


if __name__ == '__main__':
    sys.exit(main())
