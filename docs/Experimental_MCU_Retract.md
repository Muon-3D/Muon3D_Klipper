# Experimental MCU-triggered load-cell mesh retract

Status: reviewable prototype, **not cleared for unattended printing or bed contact**.
No printer has been flashed or moved during this implementation.

Base: Muon PR #40 commit `d4d3f7b8b980647d06001daa57f5766b8427db5c`.
Development branch: `experimental/mcu-tap-retract`, in a separate worktree.

## What changes

The host preloads a small, acceleration-limited upward movement before probing.
After a valid contact stops the descending motor, the motor controller starts
that movement after a 1 ms direction-hold delay. Python no longer has to finish
the homing round trip and schedule a fresh retract before the nozzle lifts.
The first upward step also includes the acceleration profile's first interval;
it is not promised to occur exactly 1 ms after contact.

On an M1, the sensor and Z motor are on different controllers. Contact still
travels through Klipper's existing C host dispatcher between controllers. This
is **not** direct controller-to-controller communication and is not zero-latency.

Klipper waits for the finite lift to finish, verifies its reported step count
and duration, then synchronizes its motor position. The contact coordinate and
the lifted coordinate remain separate. The ascent fit is preserved, using the
reported start clock and the exact preloaded step schedule, not the host's
descent-only step history. Collection ends at that ascent's scheduled end,
rather than a newly buffered `get_last_move_time()` timestamp.

Both firmware and configuration default to off. Only `BED_MESH_CALIBRATE` selects
the new path when enabled. G28, PROBE, PROBE_ACCURACY, Z-tilt, and other probe
commands retain the existing path. No global scheduling buffer, probe speed,
sample count, force threshold, sensor safety range, or fit acceptance rule is
relaxed. Firmware built with the option off excludes the retract state and code.

## Boundaries and safeguards

- Only Cartesian/CoreXY with exactly one independent `stepper_z` is supported.
  Multiple Z motors, delta/CoreXZ, dual carriage, and active Z input shaping are
  rejected. X/Y shaping is not bypassed.
- Z must already be homed. Available upper travel is checked conservatively
  even for a contact occurring immediately at the starting position.
- The profile respects the requested lift speed and current Z velocity and
  acceleration limits. Distance rounds upward by less than one microstep.
- Firmware bounds: 4096 microsteps, 512 segments, 0.5 seconds of profile time,
  1–10 ms initial delay, and pulse-spacing checks. Profile memory is allocated
  during configuration, not in the contact interrupt (roughly 4 KiB plus state
  per configured retract motor).
- Only the selected sensor's successful contact can authorize cross-MCU
  retract. Timeouts, sensor errors, host stop requests, and shutdown cannot
  authorize it. Late queued descent packets are discarded.
- Arming is acknowledged after the stop handler is registered and before the
  analog sensor is armed. The host checks the cycle ID, completion state,
  signed step delta, and duration before accepting a result. Ambiguous retract
  or position-sync failures request printer shutdown.
- A host disconnection after a valid contact can still leave the already
  started finite upward lift to finish. There is no new continuous force
  protection during that lift: the contact trigger is one-shot, as in the
  existing probe/ascent workflow. A shutdown reaching the MCU cancels it.

These are design constraints and software checks, not proof of mechanical
safety. Missed motor steps, wiring mistakes, physical obstructions, driver
timing, and real controller/transport behaviour still require bench validation.
The step timing reconstruction assumes normal scheduler accuracy; the final
duration check cannot detect every intermediate timing disturbance.

## What remains unchanged / not solved

Stationary tare, sensor transport/batching, fit processing, next-point travel,
and planning of the next descent remain. This is the first MCU-assisted part
of the fast-tapping architecture, not a fully pipelined mesh engine. Do not
claim a percentage speedup until an A/B measurement on the same printer.

For the M1's 3840 steps/mm, a 0.4 mm lift at 10 mm/s and 1000 mm/s² produces
1536 steps and about 50 ms of movement. That is a generated-profile calculation,
**not a measured contact-to-lift time or mesh speedup**.

## Local verification

Run from the repository root, with Python 3 and GCC:

```text
python test/mcu_retract/run_tests.py
```

The suite compiles the actual `src/stepper.c` for four scheduling configurations,
each with the feature on and off. GPIO, clock and scheduler are test doubles.
It exercises normal forward/reverse motion, both direction inversions,
interruptions during step pulses, repeated retracts, late packets, cancellation,
shutdown, profile limits, clock wrap, and ordinary motion after retract.

It also compiles the actual C trigger dispatcher and checks every 8-bit reason
from sensor and motor, feature on/off, duplicate messages, and reordered members
after reconnect. Eighteen Python tests cover generated profiles, state/position
validation, error shutdown, queue ordering, mesh-only routing, actual homing
coordinate calculations, and the real ascent fit on synthetic 2000 SPS data.
All passed in the Windows development environment, as did Python compilation.

Not run: an embedded-target firmware build, the full Linux Klippy integration
suite, physical pulse capture, fault injection on hardware, or a real mesh.
The environment has host GCC but no ARM toolchain or Linux runtime configured.

## Gated bench-validation procedure

1. Review the patch and build both the ordinary and experimental firmware for
   the exact controller configuration in a proper Klipper build environment.
   Preserve the existing known-good firmware and printer configuration.
2. Before any bed-contact test, use an isolated motor/logic-analyser bench to
   verify direction, step counts, pulse widths, direction hold, limits, contact
   routing, loss-of-signal behaviour, cancellation, and motion after a retract.
   Exercise both same-MCU and the M1's cross-MCU arrangement as applicable.
3. Only after those checks, enable the motor-controller build option
   `CONFIG_EXPERIMENTAL_LOAD_CELL_RETRACT=y` and the corresponding experimental
   host code on the supervised bench unit. The sensor controller does not need
   retract support unless it also drives Z. Its firmware must remain compatible
   with the selected host branch.
4. To select the path, add `experimental_mcu_retract: True` to the existing
   `[load_cell_probe]` section. With older/non-enabled motor firmware this should
   fail configuration, not silently fall back. Ordinary homing still uses the
   baseline path; perform and verify homing before any experimental mesh.
5. Use a cold, clean nozzle, conservative existing speeds and force limits,
   verified travel clearance, and an operator with an immediate physical stop.
   Run the smallest valid supervised mesh. Stop on any unexpected motion,
   position discrepancy, force anomaly, communication fault, or fit failure.
6. Compare feature off/on with identical mesh, speeds, samples, temperature,
   and tare settings. Check total wall time, point repeatability and fitted
   heights, MCU load, and ordinary probing/homing/printing after the test.
   `klippy.log` records `automatic_retract` entries with cycle, trigger-to-ascent,
   duration and steps. Correlate those with sensor and physical step traces.

Autonomous retract steps do **not** appear as ordinary trapq moves. A trapq-only
capture will incorrectly label this new movement as idle; account for the
reported retract intervals or use physical step traces for an A/B timeline.

## Disable / rollback

Set `experimental_mcu_retract: False` (or remove it), then restart Klipper to
return mesh probing to the ordinary host-controlled path. If a test fails,
stop first and re-home only after checking the machine. To remove all changes,
restore the known-good host revision, original controller firmware and config.
Do not change `BUFFER_TIME_START` globally as a workaround.
