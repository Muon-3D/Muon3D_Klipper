; Printed through virtual_sdcard at the end of contact_guard.test, with the
; guard in contact: print_stats is printing, so the guard is inactive
CONTACT_GUARD_SIMULATE LOAD=200000 DURATION=0.05
ASSERT_GUARD ACTIVE=0 CONTACT=0
G90
G1 X70 Y70 F6000
G1 X50 Y50
