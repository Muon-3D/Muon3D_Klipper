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
