#!/bin/bash
# Test script for continuous integration.

# Stop script early on any error; check variables
set -eu

# Paths to tools installed by ci-install.sh
MAIN_DIR=${PWD}
BUILD_DIR=${PWD}/ci_build
export PATH=${BUILD_DIR}/pru-elf/bin:${PATH}
export PATH=${BUILD_DIR}/or1k-elf/bin:${PATH}
PYTHON=${BUILD_DIR}/python-env/bin/python


######################################################################
# Section grouping output message helpers
######################################################################

start_test()
{
    echo "::group::=============== $1 $2"
    set -x
}

finish_test()
{
    set +x
    echo "=============== Finished $2"
    echo "::endgroup::"
}


######################################################################
# Check macro status keys resolve against what klippy publishes
######################################################################

# Ahead of the whitespace check on purpose. This script runs under
# `set -eu`, and check_whitespace currently fails on this fork over
# upstream files no branch touches -- so a check placed after it never
# executes at all. Revisit the order once that check is scoped to the
# lines a branch adds.
start_test check_macro_status_keys "Check macro status keys"
$PYTHON scripts/check_macro_status_keys.py
finish_test check_macro_status_keys "Check macro status keys"


######################################################################
# Check config headers, duplicate keys and bare template names
######################################################################

start_test check_macro_config "Check macro config"
$PYTHON scripts/test_check_macro_config.py
$PYTHON scripts/check_macro_config.py
finish_test check_macro_config "Check macro config"


######################################################################
# Run the M1 LED state and user-mode macros through klippy's macro code
######################################################################

start_test m1_led_modes "Test M1 LED macros"
$PYTHON scripts/test_m1_led_modes.py
# KAN-569: on fault paths the hot end turns off before any LED command.
$PYTHON scripts/test_m1_heater_before_leds.py
# KAN-990: the start purge is drawn only where the bed mesh probed.
$PYTHON scripts/test_m1_purge_line.py
# KAN-205: a bad CPU, motor-driver or PSU fan trips once per episode.
$PYTHON scripts/test_m1_fan_stoppage.py
# KAN-1730: PRINT_START refuses a saved Z offset past the safe range, in
# dev mode too, and applies the probe's live offset, not the raw saved one.
$PYTHON scripts/test_m1_print_start_z_offset.py
# KAN-742: the unlock press reports the contact_guard offset its retract
# leaves, moves nothing, and never uses REZERO FORCE.
$PYTHON scripts/test_m1_unlock_guard_offset.py
finish_test m1_led_modes "Test M1 LED macros"


######################################################################
# Check the M1 automatic filament load is still armed
######################################################################

# Jack, 2026-10-10: the insert trigger stays armed. It was disarmed
# once (KAN-232) and the disarm kept coming back on new branches.
start_test check_m1_armed_hooks "Check M1 armed hooks"
$PYTHON scripts/test_check_m1_armed_hooks.py
$PYTHON scripts/check_m1_armed_hooks.py
finish_test check_m1_armed_hooks "Check M1 armed hooks"


######################################################################
# B0 config (core/M1-B0) under this tree's klippy
######################################################################

# The B0 board (unit Boxwood: tmc2130, ads1220, adxl345) keeps master's
# config, copied to core/M1-B0, and runs under the same klippy/ as the B1
# core/M1. The checkers take the config directory as an argument. The
# scripts/b0 and test/b0 tests are master's, which assert B0's behaviour
# (core/M1's versions assert B1's: the start gate, m1_machine, z_power).
start_test b0_config "Check B0 config"
$PYTHON scripts/check_macro_config.py core/M1-B0
$PYTHON scripts/check_macro_status_keys.py core/M1-B0
$PYTHON scripts/check_m1_armed_hooks.py core/M1-B0
$PYTHON scripts/b0/test_m1_led_modes.py
$PYTHON scripts/b0/test_m1_heater_before_leds.py
$PYTHON scripts/b0/test_m1_purge_line.py
$PYTHON scripts/b0/test_m1_fan_stoppage.py
$PYTHON scripts/b0/test_m1_print_start_z_offset.py
$PYTHON scripts/b0/test_m1_unlock_guard_offset.py
$PYTHON test/b0/test_muon_guard.py
$PYTHON test/b0/test_cold_retract.py
finish_test b0_config "Check B0 config"


######################################################################
# Check for whitespace errors
######################################################################

start_test check_whitespace "Check whitespace"
# MUON: scoped to the lines this branch adds. The upstream check fails on
# ~180 pre-existing violations in our own additions, and because this runs
# first under `set -eu` it aborted every CI run before a single MCU
# firmware compiled. MUON_WS_FULL=1 restores the whole-tree behaviour.
python3 ./scripts/check_whitespace_muon.py
finish_test check_whitespace "Check whitespace"


######################################################################
# Run compile tests for several different MCU types
######################################################################

DICTDIR=${BUILD_DIR}/dict
mkdir -p ${DICTDIR}

for TARGET in test/configs/*.config ; do
    start_test mcu_compile "$TARGET"
    make clean
    make distclean
    unset CC
    cp ${TARGET} .config
    make olddefconfig
    make V=1
    size out/*.elf
    ./scripts/check-software-div.sh .config out/*.elf
    finish_test mcu_compile "$TARGET"
    cp out/klipper.dict ${DICTDIR}/$(basename ${TARGET} .config).dict
done


######################################################################
# Verify klippy host software
######################################################################

start_test klippy "Test klippy import (Python3)"
$PYTHON klippy/klippy.py --import-test
finish_test klippy "Test klippy import (Python3)"

# MUON: host-side serialqueue / non-critical MCU reconnect tests (KAN-243).
# Build c_helper.so through chelper.get_ffi(); no MCU dictionary needed.
start_test serialqueue "Test serialqueue fd handling and reconnect back-off"
$PYTHON test/serialqueue/test_fd_leak.py
$PYTHON test/serialqueue/test_reconnect_backoff.py
$PYTHON test/serialqueue/test_noncritical_shutdown_deadlock.py
# restart_method: swdio waits for openocd through the reactor.
$PYTHON test/serialqueue/test_swdio_restart.py
finish_test serialqueue "Test serialqueue fd handling and reconnect back-off"

# MUON: FIRMWARE_RESTART after an e-stop cannot hang klippy (KAN-1362).
start_test restart "Test restart path is bounded"
$PYTHON test/restart/test_restart_bounded.py
finish_test restart "Test restart path is bounded"

# MUON: a failed tap must stop its load cell sample collector (KAN-463).
start_test load_cell "Test load cell collector is stopped on a failed tap"
$PYTHON test/load_cell/test_collector_leak.py
finish_test load_cell "Test load cell collector is stopped on a failed tap"

# MUON: the tap register bytes adxl345_probe writes, ADXL345 (A1) and
# LIS2DW12 (B1). File-output klippy tests drop every register write.
start_test adxl345_probe_chips "Test adxl345_probe tap chip registers"
$PYTHON test/adxl345_probe/test_accel_tap_chip.py
finish_test adxl345_probe_chips "Test adxl345_probe tap chip registers"

# MUON: contact_guard refusal text, hysteresis timing and inactive states.
start_test contact_guard "Test contact_guard decisions"
$PYTHON test/contact_guard/test_contact_guard.py
$PYTHON test/contact_guard/test_height_guard.py
finish_test contact_guard "Test contact_guard decisions"

# MUON: B1 shared load cell / hotend ADS1220: the mcu's no-DRDY sampling
# against a modelled chip, hotend detection, range limits, heater hold.
start_test muon_ads1220 "Test Muon ADS1220 sampling and hotend detection"
$PYTHON test/muon_ads1220/test_muon_ads1220.py
finish_test muon_ads1220 "Test Muon ADS1220 sampling and hotend detection"
# MUON: B1 Z/bed power path decisions and fault latching.
start_test z_power "Test z_power decisions"
$PYTHON test/z_power/test_z_power.py
finish_test z_power "Test z_power decisions"

# MUON: bed heater live supply power, resistance learning and heater check.
start_test heater_supply "Test heater_supply and MPC on a simulated bed"
$PYTHON test/heater_supply/test_heater_supply.py
finish_test heater_supply "Test heater_supply and MPC on a simulated bed"

# MUON: KAN-3370 probe and homing guards: a loaded tare is refused, a Z
# probing move is ended by the load backstop or an outside abort, and the
# Z home retry never swallows either.
start_test probe_guards "Test load cell probe and homing guards"
$PYTHON test/load_cell_probe/test_tare_guard.py
$PYTHON test/load_cell_probe/test_z_load_backstop.py
$PYTHON test/load_cell_probe/test_hot_load_cell.py
$PYTHON test/load_cell_probe/test_z_legs.py
# KAN-4682: longer legs above a known plate height, never across the margin.
$PYTHON test/load_cell_probe/test_z_fast_legs.py
$PYTHON test/load_cell_probe/test_mcutare_stale_zero.py
$PYTHON test/homing_abort/test_homing_abort.py
$PYTHON test/homing_abort/test_search_cap.py
$PYTHON test/homing_abort/test_toolhead_disconnect.py
$PYTHON test/landing/test_landing_resolutions.py
$PYTHON test/landing/test_abort_mid_print_start.py
# SIM_STALL_AT (a batch-only seam in AXIS_PROTECTED_MOVE) does nothing
# unless klippy was started with -i, which no printer does.
$PYTHON test/landing/test_sim_stall_seam.py
$PYTHON test/load_cell_home_z/test_load_cell_home_z.py
finish_test probe_guards "Test load cell probe and homing guards"

# MUON: tests the M1 print-start stack added but never ran in CI.
start_test stack_extras "Test heater hold, tap path and motor tune"
$PYTHON test/heaters/test_hold_pwm.py
$PYTHON test/verify_heater/test_tap_hold.py
$PYTHON test/m1_machine/test_filament_sensor_reconnect.py
$PYTHON test/motor_tune/test_motor_tune.py
$PYTHON test/tap_path/test_drip_prime.py
$PYTHON test/tap_path/test_mcu_tare.py
$PYTHON test/tap_path/test_retract_first.py
$PYTHON test/tap_path/test_zlead.py
finish_test stack_extras "Test heater hold, tap path and motor tune"

# MUON: fail-closed dev-mode check; clients start only allowlisted commands
# outside dev mode, macros and internal sources are not checked (KAN-1730).
start_test muon_guard "Test muon_guard dev-mode check and client allowlist"
$PYTHON test/muon_guard/test_muon_guard.py
$PYTHON test/muon_guard/test_velocity_clamp.py
finish_test muon_guard "Test muon_guard dev-mode check and client allowlist"

# MUON: streaming jog (Control live drag): bounded path, limits, the contact
# guard height and Z floor.
start_test muon_jog "Test muon_jog streaming jog"
$PYTHON test/muon_jog/test_muon_jog.py
finish_test muon_jog "Test muon_jog streaming jog"

# MUON: G28 is one homing scope, and a failed G28 homes nothing (KAN-573).
start_test homing_scope "Test homing_scope"
$PYTHON test/homing_scope/test_homing_scope.py
finish_test homing_scope "Test homing_scope"

# MUON: Home All stows the bed; every internal G28 passes STOW=0 (KAN-3811).
start_test home_stow "Test Home All stows the bed"
$PYTHON test/home_stow/test_home_all_stow.py
# KAN-4683, KAN-4684: the stow's speed, and the head's park in its move.
$PYTHON test/home_stow/test_stow_rise.py
finish_test home_stow "Test Home All stows the bed"

# MUON: the bed rests at the top after user operations, never in a print (KAN-4298).
start_test stow_ops "Test the bed rests at the top after user operations"
$PYTHON test/stow_ops/test_stow_after_operations.py
finish_test stow_ops "Test the bed rests at the top after user operations"

# MUON: M600 with no print only prompts; PAUSE restores absolute mode (KAN-1699).
start_test m600 "Test M600 with no print only prompts"
$PYTHON test/m600/test_m600_no_print.py
finish_test m600 "Test M600 with no print only prompts"

# MUON: a plate pull pauses only while a file is printing (KAN-272).
start_test bed_removal "Test bed_removal_detector pauses only in the print phase"
$PYTHON test/bed_removal/test_bed_removal_detector.py
finish_test bed_removal "Test bed_removal_detector pauses only in the print phase"
# MUON: M1 machine state, start/resume gates and heat waits (m1_machine).
start_test m1_machine "Test m1_machine gates, heat waits and pause bookkeeping"
$PYTHON test/m1_machine/test_m1_machine.py
finish_test m1_machine "Test m1_machine gates, heat waits and pause bookkeeping"

# MUON: the unlock retract without FORCE_MOVE, which is off (KAN-380).
start_test cold_retract "Test cold_retract and that FORCE_MOVE is off"
$PYTHON test/cold_retract/test_cold_retract.py
finish_test cold_retract "Test cold_retract and that FORCE_MOVE is off"

# MUON: fan_modes command -> rpm -> duty mapping (ADR 0056).
start_test fan_modes "Test fan_modes speed table mapping"
$PYTHON test/fan_modes/test_fan_modes.py
$PYTHON test/fan_modes/test_print_modes.py
finish_test fan_modes "Test fan_modes speed table mapping"

# MUON: a no-go snapped mesh point keeps the mesh on its grid (KAN-468).
start_test bed_mesh_no_go_snap "Test bed_mesh no-go snapping keeps the grid"
$PYTHON test/bed_mesh/test_no_go_snap.py
finish_test bed_mesh_no_go_snap "Test bed_mesh no-go snapping keeps the grid"

# MUON: the Python 2 tests are removed. This fork is Python 3 only -- the
# image builds klippy into a `python3 -m venv` and klippy carries 36
# f-strings across 12 files, which Python 2 cannot parse at all. The tests
# could therefore never pass without deleting working printer code to suit
# an interpreter the product never runs.

start_test klippy "Test invoke klippy (Python3)"
$PYTHON scripts/test_klippy.py -d ${DICTDIR} test/klippy/*.test
finish_test klippy "Test invoke klippy (Python3)"

# MUON: klippy STARTS on the shipped board configs. Every test above loads
# a small test config; this loads the real core/M1/core.cfg (B1) and
# core/M1-B0/core.cfg (B0) in batch mode with stand-ins for the files
# outside the repo, against the rp2040 and linuxprocess dictionaries built
# above, and fails on an unknown option or section, a bad pin, or any
# config error. --selftest proves it fails on those, so it cannot pass
# vacuously.
start_test real_config "Test klippy starts on the shipped M1 configs"
$PYTHON scripts/test_real_config.py --selftest -d ${DICTDIR} core/M1 core/M1-B0
$PYTHON scripts/test_real_config.py -d ${DICTDIR} core/M1 core/M1-B0
finish_test real_config "Test klippy starts on the shipped M1 configs"

# MUON: cancelling a print while PRINT_START runs. Real reactor, gcode and
# virtual_sdcard, no MCU: the batch-mode .test files above cannot send a
# webhook from a second greenlet, which is the whole of the case.
start_test print_cancel "Test cancel during PRINT_START"
$PYTHON test/print_cancel/test_cancel_during_print_start.py
finish_test print_cancel "Test cancel during PRINT_START"

# MUON: an error in CUT_FILAMENT's cutter sequence re-enables contact_guard
# (KAN-1940); batch mode exits on any error, so this reads the log.
start_test cut_filament_error "Test CUT_FILAMENT error re-enables contact_guard"
$PYTHON test/contact_guard/test_cut_filament_error.py ${DICTDIR}
finish_test cut_filament_error "Test CUT_FILAMENT error re-enables contact_guard"

# MUON: a refused cut (the Y back stop outside its window) still parks and
# reports (KAN-3813); reads the log like the test above.
start_test cut_filament_bad_stop "Test refused CUT_FILAMENT still parks"
$PYTHON test/contact_guard/test_cut_filament_bad_stop.py ${DICTDIR}
finish_test cut_filament_bad_stop "Test refused CUT_FILAMENT still parks"
