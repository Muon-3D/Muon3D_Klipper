"""KAN-1732: outside dev mode a client's SDCARD_PRINT_FILE (or M23 + M24)
is scanned before the print starts, and refused when the file would

  * cross the notch below the traverse Z, extruding or not, G2/G3 included,
    with G90/G91/G92 tracked;
  * set a temperature above max_temp;
  * go below the Z floor;
  * go outside the XY range;
  * run a command the guard refuses a client.

The slicer fixtures are the muon3d-slicer e2e set: synthetic_square_100_90
is clear of the notch, and each cube overlaps the 75-125 x 0-34 notch (they
are the slicer's own keep-out failures).

    PYTHONUTF8=1 python3 test/muon_guard/test_preflight.py
"""
import os
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_muon_guard as tmg  # noqa: E402
from extras import print_preflight  # noqa: E402

FAILURES = tmg.FAILURES
check = tmg.check
FIXTURES = tmg.FIXTURES
CUBES = ['cube_70_15', 'cube_130_15', 'cube_69_30', 'cube_100_40']
SQUARE = 'synthetic_square_100_90'


class Toolhead:
    def get_status(self, eventtime):
        return {'homed_axes': '', 'axis_minimum': [0., -10., -4., 0.],
                'axis_maximum': [200.1, 174.1, 160., 0.]}


class VirtualSD:
    def __init__(self, dirname):
        self.sdcard_dirname = dirname
        self.current_file = None
        self.file_position = 0
        self.active = False

    def get_file_list(self, check_subdirs=False):
        out = []
        for root, dirs, files in os.walk(self.sdcard_dirname):
            for f in files:
                p = os.path.join(root, f)
                out.append((os.path.relpath(p, self.sdcard_dirname),
                            os.path.getsize(p)))
        return out

    def is_active(self):
        return self.active

    def file_path(self):
        return self.current_file.name if self.current_file else None


class Reactor(tmg.FakeReactor):
    def __init__(self):
        self.pauses = 0

    def pause(self, waketime):
        self.pauses += 1
        return waketime


def rig(**values):
    r = tmg.Rig(values=values or None)
    r.printer.add_object('toolhead', Toolhead())
    r.reactor = Reactor()
    r.printer.get_reactor = lambda: r.reactor
    r.sd = VirtualSD(tempfile.mkdtemp(prefix='preflight_gcodes_'))
    r.printer.add_object('virtual_sdcard', r.sd)
    return r


def put(r, name, text):
    path = os.path.join(r.sd.sdcard_dirname, name)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(text)
    return path


def fixture(name):
    with open(os.path.join(FIXTURES, name + '.gcode')) as f:
        return f.read()


START = "PRINT_START EXTRUDER=215 BED=55\nG90\nM83\n"


def scan_text(text):
    """Violations from the scanner alone, with core.cfg's limits."""
    lim = print_preflight.Limits(
        z_floor=0., no_go=[(75., 0., 125., 34.)], no_go_z=6.,
        axis_min=(0., -10.), axis_max=(200.1, 174.1),
        max_temp={'extruder': 265., 'heater_bed': 130.})
    return print_preflight.scan(text.split('\n'), lim)


def test_slicer_square_passes():
    v = scan_text(fixture(SQUARE))
    check(v == [], "the clear square passes: %r" % (v[:3],))
    r = rig()
    put(r, 'square.gcode', fixture(SQUARE))
    scan_file = r.guard._scan_file

    def slow(path):              # a big file: the reactor keeps running
        time.sleep(.2)
        return scan_file(path)
    r.guard._scan_file = slow
    err = r.client('SDCARD_PRINT_FILE FILENAME=square.gcode')
    check(err is None, "SDCARD_PRINT_FILE of the square runs: %r" % (err,))
    check(r.calls == ['SDCARD_PRINT_FILE FILENAME=square.gcode'],
          "and reaches virtual_sdcard: %r" % (r.calls,))
    check(r.reactor.pauses > 0, "the scan lets the reactor run")


def test_slicer_cubes_on_the_notch_are_refused():
    for name in CUBES:
        v = scan_text(fixture(name))
        check(v and 'notch' in v[0][2], "%s: notch crossing found: %r"
              % (name, v[:1]))
        r = rig()
        put(r, name + '.gcode', fixture(name))
        err = r.client('SDCARD_PRINT_FILE FILENAME=%s.gcode' % (name,))
        check(err is not None and 'notch' in err and 'line' in err,
              "%s: refused with the line: %r" % (name, err))
        check(r.calls == [], "%s: virtual_sdcard never ran" % (name,))


def one(text):
    v = scan_text(START + text)
    return v[0][2] if v else None


def test_notch():
    check('notch' in (one("G1 X60 Y20 Z2\nG1 X140 Y20 E3") or ''),
          "an extruding move across the notch")
    check('notch' in (one("G1 X60 Y20 Z2\nG0 X140") or ''),
          "a travel across it")
    check(one("G1 X60 Y20 Z7\nG1 X140") is None, "above the traverse Z")
    check('notch' in (one("G1 X60 Y40 Z2\nG3 X140 Y40 I40 J0") or ''),
          "a G3 arc under through the notch")
    check(one("G1 X60 Y40 Z2\nG2 X140 Y40 I40 J0") is None,
          "the G2 arc over the top")
    check('notch' in (one("G1 X60 Y20 Z2\nG91\nG1 X80") or ''),
          "G91 relative across it")


def test_g92_shift_is_tracked():
    # at Y60, G92 Y100 says "this is 100"; Y60 is then 40 lower: physical 20
    check('notch' in (one("G1 X60 Y60 Z2\nG92 Y100\nG1 X140 Y60") or ''),
          "G92 shift puts the move across the notch")
    check(one("G1 X60 Y60 Z2\nG92 Y60\nG1 X140 Y60") is None,
          "a G92 that changes nothing")


def test_z_floor():
    check('floor' in (one("G1 X10 Y100 Z-0.5") or ''), "Z below the floor")
    check('floor' in (one("G1 X10 Y100 Z1\nG91\nG1 Z-1.5") or ''),
          "relative Z below the floor")
    check('floor' in (one("G1 X10 Y100 Z1\nG92 Z10\nG1 Z0") or ''),
          "G92 cannot move the floor")
    check(one("G1 X10 Y100 Z0.2") is None, "first layer passes")


def test_xy_range():
    check('range' in (one("G1 X10 Y100 Z1\nG1 X220") or ''), "X too far")
    check('range' in (one("G1 X10 Y100 Z1\nG1 Y-12") or ''), "Y too low")
    check(one("G1 X200 Y-1.5 Z1") is None, "the purge lane is in range")


def test_temperatures():
    check('max_temp' in (one("M104 S300") or ''), "M104 over max_temp")
    check('max_temp' in (one("M109 S270") or ''), "M109 over max_temp")
    check('max_temp' in (one("M140 S140") or ''), "M140 over the bed max")
    check('max_temp' in (one("M190 R135") or ''), "M190 R over the bed max")
    check('max_temp' in (one(
        "SET_HEATER_TEMPERATURE HEATER=extruder TARGET=280") or ''),
        "SET_HEATER_TEMPERATURE over max_temp")
    check(one("M104 S265\nM140 S130") is None, "at max_temp passes")


def test_forbidden_commands():
    r = rig()
    put(r, 'bad.gcode', START + "G1 X10 Y100 Z1\nFORCE_MOVE STEPPER=stepper_z"
        " DISTANCE=-5 VELOCITY=5\n")
    err = r.client('SDCARD_PRINT_FILE FILENAME=bad.gcode')
    check(err is not None and 'FORCE_MOVE' in err and 'line 5' in err,
          "a forbidden command in the file: %r" % (err,))
    put(r, 'ok.gcode', START + "G1 X10 Y100 Z1\nM486 S0\n; FORCE_MOVE\n")
    check(r.client('SDCARD_PRINT_FILE FILENAME=ok.gcode') is None,
          "a comment and an unregistered M-code pass")


def test_m23_m24():
    r = rig()
    path = put(r, 'sub/Bad.gcode', START + "G1 X10 Y100 Z-1\n")
    check(r.client('SDCARD_PRINT_FILE FILENAME=/sub/bad.gcode') is not None,
          "found the way virtual_sdcard finds it (leading /, any case)")
    r.sd.current_file = open(path)
    err = r.client('M24')
    check(err is not None and 'floor' in err, "M24 of a selected bad file: %r"
          % (err,))
    r.sd.file_position = 100
    check(r.client('M24') is None, "M24 resuming a paused print is not"
          " rescanned")
    r.sd.current_file.close()


def test_missing_file_is_left_to_virtual_sdcard():
    r = rig()
    check(r.client('SDCARD_PRINT_FILE FILENAME=nothere.gcode') is None,
          "virtual_sdcard reports a missing file itself")


def test_dev_mode_and_internal_skip():
    r = tmg.Rig(dev=True)
    r.printer.add_object('toolhead', Toolhead())
    r.sd = VirtualSD(tempfile.mkdtemp())
    r.printer.add_object('virtual_sdcard', r.sd)
    put(r, 'bad.gcode', START + "G1 Z-3\n")
    check(r.client('SDCARD_PRINT_FILE FILENAME=bad.gcode') is None,
          "dev mode: no preflight")
    r = rig()
    put(r, 'bad.gcode', START + "G1 Z-3\n")
    r.from_macro('SDCARD_PRINT_FILE FILENAME=bad.gcode')
    check(r.calls == ['SDCARD_PRINT_FILE FILENAME=bad.gcode'],
          "a macro's own SDCARD_PRINT_FILE is not scanned")


def test_unknown_position_is_not_guessed():
    # after PRINT_START (a macro that moves) nothing is known until the file
    # gives absolute coordinates again
    check(one("G1 X60 Y20 Z2\nPRINT_START\nG91\nG1 X80") is None,
          "relative from an unknown position is not checked")


def test_cached_by_size_and_mtime():
    r = rig()
    put(r, 'square.gcode', fixture(SQUARE))
    r.client('SDCARD_PRINT_FILE FILENAME=square.gcode')
    n = r.guard.preflight_scans
    r.client('SDCARD_PRINT_FILE FILENAME=square.gcode')
    check(r.guard.preflight_scans == n, "the same file is not rescanned")


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
