# Probe tap-path tests

Host-side tests for the changes that remove the host's standstills from
the load-cell probe tap sequence (toolhead drip priming, homing,
load_cell_probe TAIL/MCUTARE/SETTLE, probe LIFTLAST/ZLEAD).

They do not import klippy and need no MCU dictionary or hardware: each
test exec()s the module under test with small stubs, so they run on any
Python 3 with numpy (`load_cell_probe` needs it).

    test/tap_path/run.sh

`test_retract_first.py` and `test_zlead.py` compare the default behaviour
against the same file on the merge base with `origin/master` (see
`_paths.py`), so they need a clone with that ref; the CI checkout has it.

## X/Y tap confirmation (KAN-1361)

`test_tap_confirm.py` covers `klippy/extras/tap_confirm.py`: how two home
taps are compared, against every Y tap pair logged on Boxwood 6-9 Nov
2026, and a small model of a tap that is missed at first contact.

The confirm loop runs in klippy batch mode, where `ADXL_TAP_SIMULATE`
(registered there only) says where each confirm tap triggers:
`test/klippy/adxl345_probe_stall*.test`. `tap_confirm_mutation.py` runs
those, checks from the log that each ends for the right reason, then
breaks the fix one way at a time and checks that a test fails:

    python3 test/tap_path/tap_confirm_mutation.py -d <dict dir>

It needs the klippy Python environment and `linuxprocess.dict`, like
`scripts/test_klippy.py`, so `run.sh` leaves it to `scripts/ci-build.sh`.
