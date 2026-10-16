# KAN-558: second attended Boxwood run

Use the same four cases as the results comment from 15 October 2026.
This is a runbook for Jack. No printer was accessed for this revision.

## Before the run

- Reserve Boxwood with `machine-queue` and have Jack at the printer.
- Use matching host code, M1 macros and MCU firmware from PR #65. This
  revision changes `src/buttons.c`: a host-only install is insufficient.
  The protocol signatures are unchanged, but the MCU must emit its first
  button sample even when the pin has not changed. Wait for a matching
  firmware bundle; do not use the old four-file install script unchanged.
- Keep `insert_gcode: INSERT_FILAMENT_SENSOR` armed. Run
  `scripts/check_m1_armed_hooks.py` against the staged configuration.
- Back up the current installation, record checksums, mark the device
  hazard note, and agree a rollback with Jack before installation.
- Restart the Klipper service after host Python changes. G-code `RESTART`
  and `FIRMWARE_RESTART` do not reload Python modules. Prove a changed PID
  and matching installed checksums. Recheck these after any power cycle.
- Expect a single reconnect load at startup if filament is present. Select
  Quick Load, as on 15 October. Remain idle between cases.
- Capture `klippy.log`, the journal, and a 100 ms status poll containing
  `mcu toolhead` and `filament_switch_sensor filament_sensor_toolhead`.
  Record `non_critical_disconnected`, `filament_detected` and `sample_ready`.
  Mark each case before and after it. Do not run object-query diagnostics
  during identify.

## Four steps

1. **Connected, empty toolhead: push filament in.** Wait until
   `sample_ready=True`, then wait more than 2 seconds. Insert filament.
   Expect one insert event and one normal load. Select Quick Load.
   Confirm the sensor reports filament and the printer remains ready.

2. **Filament in: unplug, remove it, replug.** Press unlock, unplug the
   umbilical, remove filament fully, then replug. Expect no load and no
   extruder motion after reattach. The first debounced sample must publish
   `filament_detected=False` and `sample_ready=True`. An old True value
   before that sample must never start a load. Wait at least 5 seconds.

3. **Empty at unplug: feed filament while unplugged, replug.** Press
   unlock, unplug, feed filament past the switch while disconnected, then
   replug. Skip if the hardware prevents safe feeding while detached.
   Expect exactly one reconnect load after the first debounced sample,
   and no insert event from the resync edge. Select Quick Load.

4. **Filament in: unplug, replug with it still in.** Press unlock, unplug,
   leave filament in, then replug. Expect a fresh sample even though the
   switch is unchanged, followed by exactly one reconnect load. Select
   Quick Load. Repeat only while Jack agrees the connector is sound.
   There must be no `Invalid oid type` during identify or later.

## Evidence and stop conditions

- Measure from the reconnect and from the log's
  `first post-reconnect sample` line. On 15 October the debounced state
  arrived at +1.75 seconds. Readiness now follows the actual sample; the
  delayed macro checks it every 0.1 seconds. It has no fixed sample timeout.
- The 2 second insert/runout settle window starts at the debounced sample,
  so it ended at about +3.75 seconds in the measured case. During that
  window edges update status without insert/runout actions. Repeat step 1
  after the window to prove automatic loading remains armed.
- For steps 2 through 4, check for zero `Invalid sequence`, zero
  `Invalid oid type`, and zero shutdowns. The retained stepcompress clock
  calibration must hold even if no Stats tick precedes a move.
- Inspect MCU traffic from identify until configuration finishes. Permit
  handshake and configuration/init commands only. No background
  `query_ads1220_status`, TMC `spi_transfer`/`spi_send`, or other runtime
  object commands may precede completed configuration. After reconnect,
  confirm load-cell updates and TMC checks resume normally.
- If unlock remains PRESSED, press and release it once; this revision does
  not change unlock-button policy. Stop on unexpected motion, a shutdown,
  or a Pi reset. Do not retry the broken toolhead SWD reset through
  `FIRMWARE_RESTART`. A latched board needed a power cycle on 15 October.
- The Pi journal previously ended without a shutdown during a replug.
  Jack must investigate the umbilical/power hardware if that recurs. This
  code does not claim to fix the Pi power loss.
- At the end, record results on PR #65. Follow the agreed restore procedure,
  verify checksums and the running PID, clear the hazard note when safe,
  and release the lease.
