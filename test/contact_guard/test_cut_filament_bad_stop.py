"""KAN-3813: a refused cut still parks the head, and the guard is back.

On sloe (17 Nov 2026) the cut's tap towards Y min stopped at 3.22, outside
the -9.5 to -5.0 window; the cut was refused and PRINT_END's park was
skipped with it, leaving the head over the print. Batch mode exits with an
error on any command error, even a caught one (see
test_cut_filament_error.py), so this reads the log instead: the same
simulation (test/klippy/cut_filament.cfg) with the tap told to find the stop
at 3.22, then PRINT_END. In the log, in order: the refusal, the report
(after the park), and at PRINT_END's M84 the head at X135 Y63 Z140 with
X, Y and Z still homed, PARK_PRINTER never run, and the guard enabled.

    python3 test/contact_guard/test_cut_filament_bad_stop.py <dictdir>
"""
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
GCODE = """
G28
G90
G1 X100 Y100 Z20 F6000
CONTACT_GUARD_SIMULATE LOAD=0 DURATION=0.5
_CUT_TEST STOP_AT=3.22
PRINT_END
"""
REFUSED = 'the Y back stop is at 3.22, outside -9.5 to -5.0'


def run(dictdir):
    with tempfile.TemporaryDirectory() as tmp:
        gcode = os.path.join(tmp, 'bad_stop.gcode')
        log = os.path.join(tmp, 'klippy.log')
        with open(gcode, 'w') as f:
            f.write(GCODE)
        subprocess.run(
            [sys.executable, os.path.join(ROOT, 'klippy', 'klippy.py'),
             os.path.join(ROOT, 'test', 'klippy', 'cut_filament.cfg'),
             '-i', gcode, '-o', os.path.join(tmp, 'output'), '-v',
             '-d', os.path.join(dictdir, 'atmega2560.dict'), '-l', log],
            cwd=ROOT, timeout=600, check=False)
        with open(log) as f:
            return f.read().splitlines()


def first(lines, text, start=0):
    for i in range(start, len(lines)):
        if text in lines[i]:
            return i
    return None


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    lines = run(sys.argv[1])
    skipped = first(lines, 'PRINT_END: parking skipped')
    if skipped is None or REFUSED not in lines[skipped]:
        sys.exit("FAIL: the cut was not refused for a stop at 3.22")
    report = first(lines, 'the end-of-print park stopped early', skipped)
    end = first(lines, 'cut-test: park-end', skipped)
    m84 = first(lines, 'cut-test: at M84', skipped)
    if report is None or end is None or m84 is None:
        sys.exit("FAIL: no report, or PRINT_END did not reach its M84 after"
                 " the refused cut (report %s, park end %s, M84 %s)"
                 % (report, end, m84))
    if not report < end:
        sys.exit("FAIL: the report came after the motors went off")
    if REFUSED not in lines[report]:
        sys.exit("FAIL: the report does not say why: %s" % (lines[report],))
    for line in (lines[report], lines[end], lines[m84]):
        print(line)
    expected = ('cut-test: park-end pos=135.000,63.000,140.000 homed=xyz'
                ' parks=0')
    if lines[end] != expected:
        sys.exit("FAIL: expected %r, got %r" % (expected, lines[end]))
    if lines[m84] != 'cut-test: at M84 enabled=1 active=1':
        sys.exit("FAIL: the guard is not enabled and active after the"
                 " refused cut: %s" % (lines[m84],))
    print("OK: a refused cut still parks (Z, then X135 Y63), reports it,"
          " and leaves the guard enabled")


if __name__ == '__main__':
    main()
