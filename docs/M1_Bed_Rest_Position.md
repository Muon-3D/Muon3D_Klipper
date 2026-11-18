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

`_STOW_IF_IDLE` runs `Z_PARK` only with Z homed and no print running or
paused (`print_stats`, `pause_resume`, `virtual_sdcard`). It never homes.

## Does not stow

- A jog (`G0`/`G1` from the Move screen), `CENTER`, `FRONT`: the user put the
  bed there.
- Anything inside a print or a pause: the runout and `M600` unload and insert
  flows hand back to `RESUME`, which depends on where the head is.
- Probe calibration (it ends in Klipper's own `ACCEPT`/`ABORT`, then
  `SAVE_CONFIG` restarts and unhomes), PID and MPC calibration (no motion),
  `FINETUNE_RESONANCE` and the bed mesh (print path), `ANTI_OOZE_NOZZLE_DOCK`
  (the nozzle rests in the pad on purpose), the debug and bench tests.

Tests: `test/home_stow/` and `test/stow_ops/`.
