# Accelerometer tap homing

The M1 homes X and Y by driving the toolhead into the frame and taking the
impact, seen by the toolhead accelerometer, as the endstop. TMC sensorless
homing was dropped because it drifts as the motors heat up.
[adxl345_probe](Config_Reference.md#adxl345_probe) implements this. The chip
wires its tap interrupt to a toolhead MCU pin, and that pin is the endstop.
The homing, arming and confirmation logic doesn't depend on the chip. The
register handling does, and lives in `klippy/extras/accel_tap_chip.py`.

| Batch | Chip | Config section | CS | Interrupt |
|-------|------|----------------|----|-----------|
| A1 (boxwood, walnut) | ADXL345 | `[adxl345]` | toolhead gpio21 | INT1 -> gpio25 |
| B1 (gimlet) | LIS2DW12TR | `[lis2dw]` | toolhead gpio25 (10k pull-up) | INT1 -> gpio21, INT2 not connected |

The CS and interrupt pins are swapped between A1 and B1. Both chips sit on
toolhead SPI0 (`spi0_gpio20_gpio23_gpio22`).

## How the settings map onto each chip

The options are the same for both chips: `tap_thresh` in mm/s^2,
`tap_dur` in seconds, `*_tap_axes`, the arm delays and `tap_confirm_*`.
What changes between chips is what the registers can represent.
`SET_ACCEL_PROBE` prints the value the chip actually uses. The status
reports it as `profiles.<axis>.tap_thresh_effective` and
`tap_dur_effective`.

| | ADXL345 (A1) | LIS2DW12 `single_tap` | LIS2DW12 `wake_up` |
|---|---|---|---|
| Threshold register | THRESH_TAP, 8 bit | TAP_THS_X/Y/Z, 5 bit | WK_THS, 6 bit |
| Threshold step | 62.5 mg (612.9 mm/s^2) | full scale / 32 | full scale / 64 |
| Step at +-4g (default) | | 125 mg (1225.8 mm/s^2) | 62.5 mg (612.9 mm/s^2) |
| Range at +-4g | | 0.125 to 3.875 g | 0.0625 to 3.94 g |
| Step at +-16g | | 500 mg | 250 mg |
| Rounding | down | nearest | nearest |
| Signal compared | acceleration | high-pass (slope) filtered acceleration | high-pass (slope) filtered acceleration |
| Duration | DUR, max time above threshold, 0.625 ms steps | SHOCK window: 4, 8, 16 or 24 samples | none: fires on the first sample over |
| Duration at 1600 Hz | | 2.5, 5, 10, 15 ms | |
| Duration at 800 Hz | | 5, 10, 20, 30 ms | |
| Per-axis enable | TAP_AXES (see below) | TAP_X/Y/Z_EN, via axes_map | no, any axis |
| Sample period | 10 ms until the first stream, then 0.31 ms (see below) | 0.625 ms at 1600 Hz | 0.625 ms at 1600 Hz |
| Interrupt pins | INT1 or INT2 | INT1 only | INT1 only |

The A1 values in `core/M1/core.cfg` map as follows:

| A1 setting | ADXL345 | LIS2DW12 single_tap +-4g | LIS2DW12 wake_up +-4g |
|---|---|---|---|
| `tap_thresh: 15000` (1.53 g) | 24 -> 1.50 g | 12 -> 1.50 g | 24 -> 1.50 g |
| `x_tap_thresh: 20000` (2.04 g) | 32 -> 2.00 g | 16 -> 2.00 g | 33 -> 2.06 g |
| `tap_dur: 0.03` | 48 -> 30 ms | refused at 1600 Hz (max 15 ms); 30 ms at 800 Hz | unused |

Matching numbers don't mean matching behaviour. The ADXL345 compares the
acceleration itself. The LIS2DW12 tap and wake-up engines compare a
high-pass (slope) filtered signal, so gravity drops out and a slow ringing
counts for less. The B1 threshold has to come from the sweep below, not
from A1.

The LIS2DW12 has two event engines. `lis2dw_tap_event` selects one:

* `single_tap` (default). Keeps the A1 semantics: per-axis enables, and
  `tap_dur` as the longest time the impact may stay above the threshold.
  The interrupt comes once the signal falls back below the threshold
  inside the SHOCK window. An impact that stays above the threshold for
  longer than the window is not a tap at all, and the home then runs into
  the frame. QUIET is set to its minimum (2 samples) and double tap is
  off.
* `wake_up`. Fires on the first sample over the threshold, with no window
  to outlast, so the latency is the lowest. It cannot be limited to one
  axis, so `x_tap_axes`/`y_tap_axes` are not enforced: any axis that
  crosses the threshold triggers.

Other LIS2DW12 details:

* The chip runs continuously at `lis2dw_odr` and `lis2dw_full_scale` from
  klippy connect onwards. It is not powered down between homes, so arming
  never follows a power-up transient.
* Interrupts are latched (CTRL3 LIR), like the ADXL345's, and released
  by reading ALL_INT_SRC.
* A resonance stream (`lis2dw.py`) switches the chip to 1600 Hz +-16g.
  The next homing move switches it back. Homing during a stream is
  refused unless the tap settings already are 1600 Hz +-16g. The first
  `ADXL_TEST_TAP` after a stream may report "tap triggered before move"
  while the full-scale change settles. Run it again.
* The probe needs the LIS2DW12 on SPI. Arming is scheduled on the MCU
  clock, and the I2C reads take no `minclock`.
* The toolhead firmware has to be built with `CONFIG_WANT_LIS2DW=y`. It
  is now on in `core/M1/firmware/config.rp2040`.

### A1 caveats found during the port (not changed here)

Both are left exactly as they were so that A1 keeps homing the way it was
tuned. Each needs an A1 bench run before anyone changes it.

1. **`x_tap_axes: x` enables the ADXL345's Z axis.** The profile mask
   (x=0x01, y=0x02, z=0x04) has been written to TAP_AXES unchanged since
   f5ca5fb63. The register's bits are TAP_X=0x04, TAP_Y=0x02, TAP_Z=0x01,
   so A1 X homing listens to the chip's Z axis and Y to Y. This may be
   behind the steady 1.60 mm X disagreement and the higher X threshold.
   On the LIS2DW12, tap axes are printer axes mapped through `axes_map`.
2. **The ADXL345 tap engine runs at the chip's reset output rate.** The
   probe never writes BW_RATE. Until the first accelerometer stream after
   power-up, the ADXL345 runs at its 100 Hz reset rate (10 ms samples,
   so `tap_dur: 0.01` is a single sample). After a stream it runs at
   3200 Hz. A1 tap timing therefore differs between the first homes after
   boot (or after a toolhead reconnect) and later ones.

## B1 configuration

A starting point for Muon-gimlet-6741. `axes_map` and the thresholds are
not validated yet. The procedure below sets them.

```
[lis2dw]
cs_pin: toolhead:gpio25
spi_bus: spi0_gpio20_gpio23_gpio22
axes_map: x, y, z    # UNVALIDATED: set from the orientation check

[adxl345_probe]
chip: lis2dw
probe_pin: toolhead:gpio21
int_pin: int1
lis2dw_tap_event: single_tap
lis2dw_odr: 1600
lis2dw_full_scale: 4
tap_thresh: 15000    # starting point only, see the sweep
tap_dur: 0.015       # longest SHOCK window at 1600 Hz
homing_tap_arm_delay: 0.1
x_tap_thresh: 20000
x_tap_axes: x
y_tap_thresh: 15000
y_tap_axes: y
tap_confirm_samples: 1
y_tap_confirm_samples: 2
tap_confirm_tolerance: 0.2
tap_confirm_retries: 2
tap_confirm_retract_dist: 8
tap_confirm_seek_retries: 1
enable_z_probe: False

[resonance_tester]
accel_chip: lis2dw
```

The endstop pins (`adxl345_probe:x_virtual_endstop`,
`adxl345_probe:y_virtual_endstop`) and the homing macros don't change.

## Bench test plan: Muon-gimlet-6741 (B1)

This needs the B1 toolhead connected and a firmware build with
`CONFIG_WANT_LIS2DW=y`. Push the Klipper change with
`./scripts/printer.sh push-klipper` from the MuonOS repository (see its
`docs/PRINTER_DEV_LOOP.md`). Record every result in the tables at the end.
Mechanical numbers below assume `rotation_distance: 40`, 200 steps and
64 microsteps, so one microstep is 0.003125 mm.

### 0. The chip answers

1. Klippy starts with no `Invalid lis2dw id` and no unknown `config_lis2dw`
   command. An unknown command means the toolhead firmware lacks
   `CONFIG_WANT_LIS2DW`.
2. `klippy.log` shows one `adxl345_probe x profile -> LIS2DW12 ...` line
   per axis, with the quantised thresholds.
3. `ACCELEROMETER_QUERY CHIP=lis2dw` with the printer at rest: the vector
   magnitude is 9800 +- 500 mm/s^2.

### 1. Axis orientation (sets `axes_map`)

Start from `axes_map: x, y, z`. The goal is to find, for each printer
axis, the chip axis that sees it.

1. **Gravity.** `ACCELEROMETER_QUERY CHIP=lis2dw` at rest. The printer
   axis that reads about +-9800 is the one the chip's vertical axis is
   mapped to. With a correct map that is printer z.
2. **Motion.** For each of X and Y:
   ```
   G28
   G1 X100 Y100 F6000
   ACCELEROMETER_MEASURE CHIP=lis2dw NAME=jogx
   G1 X150 F18000
   G1 X50 F18000
   G1 X150 F18000
   ACCELEROMETER_MEASURE CHIP=lis2dw NAME=jogx
   ```
   Then repeat with `NAME=jogy` and Y moves. On the printer, find the
   column with the most motion:
   ```
   python3 -c "import csv,sys,statistics as s; r=list(csv.reader(l for l in open(sys.argv[1]) if not l.startswith('#'))); [print(n, round(s.pstdev(float(x[i]) for x in r))) for i,n in ((1,'x'),(2,'y'),(3,'z'))]" /tmp/lis2dw-jogx.csv
   ```
   The column with by far the largest deviation is the chip axis that
   moves with printer X (or Y). Its sign is the sign of its first large
   swing during the first `G1 X150` (+X): positive means no `-`.
3. Write `axes_map` so that its first entry is the chip axis found for X,
   the second for Y, and the third the remaining (vertical) one. Restart,
   then repeat step 2 to confirm each printer axis now shows the motion on
   its own column.
4. **Tap selectivity.** `ADXL_TEST_TAP AXIS=X TAPS=3 TIMEOUT=10` and tap
   the toolhead by hand along X. Each detection prints
   `TAP_SRC=0x..: chip ? = printer x`. Then tap along Y with the same
   command: it must time out. Repeat with `AXIS=Y`. A tap reported on the
   wrong printer axis means `axes_map` is wrong. The tap sign is not used
   by homing or by shaper calibration, so it needs no fixing.

### 2. Threshold sweep (per axis, X first)

Production homing speed (60 mm/s), motors cold. For the sweep only, set
`x_tap_confirm_samples: 2`. A tap taken too early then shows up in
`klippy.log` as `tap judged spurious`, instead of passing as a short home.

1. `single_tap`, `tap_dur: 0.015`. For each threshold from 1.0 g to 3.5 g
   in 0.25 g steps (`SET_ACCEL_PROBE AXIS=X TAP_THRESH=<mm/s^2>`, which is
   every second TAP_THS step at +-4g), run `G28 X` five times. Record:
   * early triggers (spurious taps in the log, or homing done short of the
     frame),
   * misses (the motor grinds on the frame, `No trigger on x after full
     movement`),
   * the confirmation spread the log reports.
2. The working window runs from the lowest threshold with no early trigger
   to the highest with no miss. Take its middle. Report the window width
   in TAP_THS steps. Fewer than 3 steps is a fail for this configuration.
3. If every threshold misses, the impact outlasts the 15 ms window. Repeat
   with `lis2dw_odr: 800` and `tap_dur: 0.03`, and then with
   `lis2dw_tap_event: wake_up` (4g, 62.5 mg steps).
4. Repeat 1 to 3 for Y, with its production `y_tap_confirm_samples: 2`.

### 3. Repeatability (20 homes per axis)

At the chosen settings, with the production confirmation counts:

```
G28 X
G1 X100 F6000
GET_POSITION
```

Run this 20 times for X, then the equivalent 20 times for Y. From each
`GET_POSITION`, record the `mcu:` counts of stepper_x and stepper_y (the
A and B motors on the M1's corexy). The commanded position is always the
same, so any spread in the counts is homing error. Range of the counts
x 0.003125 = spread in mm.

Pass:

* X and Y spread (max - min) at or below 0.2 mm, which is
  `tap_confirm_tolerance`,
* 0 early triggers and 0 misses in 40 homes,
* Y confirmation succeeds 20 out of 20 times.

Then warm the motors (10 minutes of `G1` moves across the bed) and run 5
more homes per axis. The spread must stay inside the same limit. Warm
motors are the case sensorless homing failed.

### Results template

| Step | Setting | Result | Notes |
|------|---------|--------|-------|
| 0.3 | at rest | magnitude = | |
| 1.1 | gravity on printer axis | | |
| 1.2 | X motion on chip axis / sign | | |
| 1.2 | Y motion on chip axis / sign | | |
| 1.3 | final axes_map | | |
| 1.4 | X tap selectivity | | |
| 1.4 | Y tap selectivity | | |
| 2 | X window (g) / steps / chosen | | |
| 2 | Y window (g) / steps / chosen | | |
| 3 | X spread, 20 homes (mm) | | |
| 3 | Y spread, 20 homes (mm) | | |
| 3 | early / missed in 40 | | |
| 3 | warm X / Y spread (mm) | | |
