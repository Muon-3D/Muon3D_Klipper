# M1: where the bed rests (KAN-3811, KAN-4298)

The M1 stows the bed at the top (`Z_PARK`, Z140, inside the Z travel of 160)
when it is left idle, so the plate is clear of the nozzle. Z0 is the plate
touching the nozzle; a larger Z is the plate farther from it.

## Ends at the top

| Operation | Stows | Notes |
| --- | --- | --- |
| Home All (bare `G28`) | yes | `STOW=0` turns it off; `G28 X`, `Y`, `Z`, `O`, `PARK=0` never stow. Every `G28` a macro runs passes `STOW=0`. |
| End of a print and cancel | yes | `PRINT_END` parks through `CUT_FILAMENT` and `PARK_PRINTER`; a refused park still runs `Z_PARK` when Z is homed. `CANCEL_PRINT` ends through `PRINT_END`. |
| `CUT_FILAMENT` on its own | yes | ends with `PARK_PRINTER`. |
| `UNLOAD_FILAMENT` | yes, idle | via `_STOW_IF_IDLE`. |
| Insert flow: Quick Load | yes, idle | via `_STOW_IF_IDLE`. |
| Insert flow: Full Purge | yes, idle | `Z_PARK` as before, now skipped in a print. |
| A print that fails | yes | `_ON_PRINT_ERROR` (`[virtual_sdcard] on_error_gcode`) ends with `_STOW_AFTER_PRINT_ERROR`, after the heaters are off and `CLEAR_PAUSE`. It runs while `print_stats` still says `printing` (Klipper notes the error after), so it skips the print-state gate but keeps the rest. |
| `BED_MESH_CALIBRATE`, `SHAPER_CALIBRATE` | yes, idle | `rename_existing` wrappers in `calibration.cfg` call `_STOW_IF_IDLE` after the original. `STOW=0` turns it off: PRINT_START's mesh (`_BED_MESH_ENSURE`) and `FINETUNE_RESONANCE`'s sweeps pass it. The wrappers live in `stow_wrappers.cfg`. |
| `FINETUNE_RESONANCE` | yes, idle | one stow at the end; PRINT_START calls it with `STOW=0`. |
| `PID_CALIBRATE` (hot end), `CALIBRATE_HEATING_BED` (MPC) | yes, idle | stow after the calibration. |
| Probe Z offset (`CALIBRATE_PROBE`, `PROBE_CALIBRATE`) | yes, idle | `ManualProbeHelper.finalize` (`manual_probe.py`) runs `_STOW_IF_IDLE` for ACCEPT, ABORT and a failed TESTZ move, if the config defines it. The prompt's Cancel button (`_CANCEL_CALIBRATE_PROBE`) stows too. |
| A calibration that raises | yes, idle | the new `on_error_gcode` option of `[gcode_macro]` (`gcode_macro.py`) runs `_STOW_IF_IDLE` and then re-raises the error. A print cancel (`CommandAbort`) does not run it. |

`_STOW_IF_IDLE` runs `Z_PARK` only with Z homed, the Z feed ready
(`z_power`, when the board has one), Klipper not shut down, no manual probe open
(`manual_probe.is_active`: a multi-point `BED_MESH_CALIBRATE METHOD=manual` stows
only after its last point), no PRINT_START in
progress (`_PRINT_START_MARK.phase`) and no print running or paused
(`print_stats`, `pause_resume`, `virtual_sdcard`). It never homes.
`_STOW_AFTER_PRINT_ERROR` has the same gates minus the print state and the
PRINT_START phase (the failing print is the one that just ended), and still
refuses with a pause set.

Never stowed: Z not homed, `z_power` not `ready` (plate off, column unlocked,
recovering, fault), Klipper shut down (no G-code runs then anyway).

## How fast, and with what (KAN-4684, KAN-4683)

- **Speed.** With the plate height known to the contact guard (homed since
  the last bed removal, motor off, `FORCE_MOVE` or failed home) the rise runs
  at `Z_PARK`'s `known_speed` (15 mm/s), never above `[printer]
  max_z_velocity` (15); with the height unknown it is 10 mm/s as before.
  `Z_SPEED=` (mm/min) overrides either. Acceleration (`max_z_accel`, 1000) is
  not touched.
- **The head's park.** Klipper's planner runs one move at a time, so an X/Y
  move queued behind the rise waits for it, and a later `G1 X..` from a
  client cannot start alongside it. The one thing that can travel with the
  plate is a single `G0` with X, Y and Z, and Klipper holds its Z at
  `max_z_velocity`. A Home All does that: `Z_PARK_XY` parks the head at the
  back right corner in the same move as the rise when the height is known,
  X, Y and Z are homed and the plate is at or above Z10 (the height the
  filament cut and the wipe move X/Y at, above the 5 mm wipe pad and the 8 mm
  notch). Every point of that path is on the line the park always took at
  Z10, at or above Z10. Otherwise the head parks first and the plate rises
  after, as before. The park after a print (`PARK_PRINTER`) stays in that
  order on purpose: the head may be at the cutter nub and a printed part
  hangs from the plate, so the plate must be clear before X/Y moves.

## Does not stow

- A jog (`G0`/`G1` from the Move screen), `CENTER`, `FRONT`: the user put the
  bed there.
- Anything inside a print or a pause: the runout and `M600` unload and insert
  flows hand back to `RESUME`, which depends on where the head is.
- `SAVE_CONFIG` after a probe calibration: it restarts Klipper and unhomes Z,
  so the bed is wherever the stow before it left it (the top).
- The print path: PRINT_START's mesh and sweep (`STOW=0`), and
  `ANTI_OOZE_NOZZLE_DOCK` (the nozzle rests in the pad on purpose), the debug
  and bench tests.

## Not covered

- A failure that Klipper does not raise as a G-code error: a heater fault or
  MCU error shuts Klipper down (no G-code runs, no stow, by design); a
  heater that merely misbehaves without an error leaves the print running.
- A pause or `RESUME` that fails (the print is not `error`).
- Calibration commands other than the wrapped ones (`PROBE_ACCURACY`,
  `Z_ENDSTOP_CALIBRATE`, `MPC_CALIBRATE` typed directly, `TEMPERATURE_PROBE_*`):
  they have no wrapper. The manual-probe ones stow through `finalize`.
- A calibration the user stops with `M112`/`FIRMWARE_RESTART` (shutdown).
- An `on_error_gcode` stow that itself fails (for example a move error) only
  logs; the original error is the one the user sees.

Tests: `test/home_stow/` and `test/stow_ops/`.
