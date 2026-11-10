"""KAN-1940: an error part-way through CUT_FILAMENT re-enables the guard.

CUT_FILAMENT stands contact_guard aside for the cutter press. Klipper
macros have no try/finally, and PRINT_END's park catches the error, so the
print end carries on with the macro's ENABLE=1 never run. Batch mode exits
with an error on any command error, even a caught one, so test_klippy can
only see that this run failed. This runs the same simulation
(test/klippy/cut_filament.cfg) and reads the log instead: the injected
error was raised in the cutter sequence, and at PRINT_END's M84, after the
park, the guard is enabled and active again.

    python3 test/contact_guard/test_cut_filament_error.py <dictdir>
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
_CUT_TEST FAIL=1
PRINT_END
"""


def run(dictdir):
    with tempfile.TemporaryDirectory() as tmp:
        gcode = os.path.join(tmp, 'error.gcode')
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


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    lines = run(sys.argv[1])
    raised = [l for l in lines if l.startswith('PRINT_END: parking skipped')
              and 'injected error in the cutter sequence' in l]
    at_m84 = [l for l in lines if l.startswith('cut-test: at M84')]
    for line in raised[:1] + at_m84:
        print(line)
    if not raised:
        sys.exit("FAIL: the injected error did not end the park")
    if at_m84 != ['cut-test: at M84 enabled=1 active=1']:
        sys.exit("FAIL: the guard is not enabled and active after an error"
                 " in the cutter sequence: %s" % (at_m84,))
    print("OK: an error in the cutter sequence leaves the guard enabled")


if __name__ == '__main__':
    main()
