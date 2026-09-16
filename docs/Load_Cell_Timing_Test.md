# Experimental load-cell timing optimization

This change is based on the ascent-fitting implementation in Muon PR #40.
It retains ascent fitting and leaves the normal motion startup margin and
homing implementation intact. `low_latency` defaults to 0. No printer
configuration needs to be saved to compare the two paths.

## What changes

With `LOW_LATENCY=1`, selecting tare samples no longer calls a timing accessor
that primes an empty motion queue. Pending moves are still scheduled normally.
Tare collection starts after the later of the queue boundary and current MCU
time, plus an explicit settling interval (40 ms by default). The configured
tare sample count, trigger thresholds, and force safety limits still apply.

The ascent's actual planned start and end are captured by a move callback,
including acceleration and any required restart delay. Sample collection ends
at the earlier of ascent completion and the end of the 300 ms fitting window.
For a long ascent, fitting can happen while the rest of the retract runs.
For the M1's roughly 50 ms retract, the entire ascent is still collected.
Late samples from already received batches are excluded from the fit.

All load-cell collectors now wake as soon as the finishing batch arrives,
instead of waiting for the next 50 ms poll. Timeout checks remain bounded.
This also applies with `LOW_LATENCY=0`, so use the unmodified PR #40 revision
for a comparison of the total change, and the command switch to isolate the
tare/window changes.

## What this does not claim

The PR does not remove the host-to-MCU restart margin or make probing continuous.
It does not defer measurement validation beyond the probe session: tolerances,
retries, and compensation callbacks still receive the fitted position before
the next point is selected. General overlap with XY travel would require a
separate change to those interfaces. Hardware speed and repeatability gains
have not yet been measured. A shorter implicit settling interval can affect
accuracy and must be checked on the actual machine.

## Bench comparison

Use the established safe mesh bounds for the particular printer. The following
commands use the previously tested M1 minimum of X20 Y50; check the configured
mesh maximum and any excluded regions before running them on another printer.
Keep the bed/nozzle state, speeds, sample count, and geometry identical.

1. On unmodified PR #40, home and record at least five uninstrumented runs of
   the same 3x3 mesh, plus `PROBE_ACCURACY SAMPLES=20` results. Record firmware
   revisions and configured sensor rate. Do not compare against the old fork
   alone, since PR #40 changes how contact height is calculated.
2. On this branch, start with the normal homing path and compare:

   ```gcode
   PROBE_ACCURACY SAMPLES=20 LOW_LATENCY=0
   PROBE_ACCURACY SAMPLES=20 LOW_LATENCY=1 SETTLING_TIME=0.04
   BED_MESH_CALIBRATE PROBE_COUNT=3,3 MESH_MIN=20,50 LOW_LATENCY=0
   BED_MESH_CALIBRATE PROBE_COUNT=3,3 MESH_MIN=20,50 LOW_LATENCY=1 SETTLING_TIME=0.04
   ```

3. Alternate the two mesh commands for at least five runs each. Time complete
   commands with the same method. Run without trapq capture or tap subscribers
   for primary timings. Compare mesh heights, accuracy range, and mean, as
   well as elapsed time. Stop on unexpected contact, sensor/fit errors, or a
   significant change in height/repeatability.
4. Separately add `PROBE_TIMING=1` for diagnostic runs. `probe` includes tare
   and descent/homing; `schedule` is host queue setup; `collect` includes real
   motion, sensor delivery and waiting; `fit` is the actual computation;
   `publish` covers tap conversion/publication. The reported `ascent` is its
   planned physical duration. These numbers do not equate collection time
   with computation or stationary time. Use one separate trapq capture for
   the physical-motion breakdown.
5. Test repeated probe-based `G28 Z`, accuracy, tolerance retry/failure, and
   mesh commands before enabling the config option. To test low-latency
   homing, temporarily use `low_latency: 1` in a backed-up configuration:
   the homing helper constructs its own probe command, so a parameter on
   `G28` may not propagate to it. Restore that configuration afterward.

Returning to `LOW_LATENCY=0` disables tare/window optimizations. Returning to
the parent revision also restores polling collector wakeups. No `SAVE_CONFIG`
is required by this experiment.

## Automated checks

Run `python -m unittest discover -s test -p test_load_cell_timing.py -v` for
host-only clock/queue/collector regressions. The normal Linux Klippy simulator
test in `test/klippy/load_cell.test` exercises multi-sample probing, accuracy,
meshing and homing. Simulator samples cannot demonstrate real fit quality,
mechanical settling, communication margins, or wall-clock speed improvements.
