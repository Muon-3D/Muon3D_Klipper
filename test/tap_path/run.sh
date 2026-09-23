#!/bin/bash
# Host-side tests for the probe tap-path changes.  No klippy import, no MCU
# dictionary, no hardware: each test exec()s the module under test with stubs.
cd "$(dirname "$0")" || exit 1
rc=0
for t in test_drip_prime.py test_mcu_tare.py test_retract_first.py test_zlead.py; do
    echo "== $t"
    python3 "$t" || rc=1
done
exit $rc
