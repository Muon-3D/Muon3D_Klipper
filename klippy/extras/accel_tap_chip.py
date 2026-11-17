# Accelerometer tap-interrupt backends for adxl345_probe
#
# adxl345_probe wires an accelerometer's tap interrupt to an MCU endstop
# pin and uses it as X/Y virtual endstops. The homing, arming and tap
# confirmation logic there is chip independent. What is not lives here:
# the register map, how a tap profile (threshold in mm/s^2, duration in
# seconds, printer axes) becomes register values, and how the interrupt
# is armed, disarmed and cleared.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import adxl345, lis2dw

FREEFALL_ACCEL = adxl345.FREEFALL_ACCEL  # mm/s^2 per g

# Profile axis masks are printer axes; each backend maps them to its chip.
TAP_AXIS_BITS = {'x': 0x01, 'y': 0x02, 'z': 0x04}


class FileOutputRegs:
    # Stand-in for the chip register interface when klippy runs in
    # file-output (regression test) mode and SPI reads return nothing.
    def __init__(self, mcu):
        self.mcu = mcu

    def set_reg(self, reg, val, minclock=0):
        pass

    def read_reg(self, reg, minclock=0, reqclock=0):
        return 0

    def check_connected(self):
        pass


######################################################################
# ADXL345
######################################################################

ADXL_REG_THRESH_TAP = 0x1D
ADXL_REG_DUR = 0x21
ADXL_REG_TAP_AXES = 0x2A
ADXL_REG_INT_ENABLE = 0x2E
ADXL_REG_INT_MAP = 0x2F
ADXL_REG_INT_SOURCE = 0x30

ADXL_DUR_SCALE = 0.000625  # 0.625 msec / LSB
ADXL_TAP_SCALE = 0.0625 * FREEFALL_ACCEL  # 62.5 mg / LSB
ADXL_INT_SINGLE_TAP = 0x40


class ADXL345TapChip:
    name = "ADXL345"

    def __init__(self, config, chip, regs, int_pin, inverted):
        self.regs = regs
        self.inverted = inverted
        self.int_map = 0x40 if int_pin == 'int2' else 0x00
        self.thresh_min = ADXL_TAP_SCALE
        self.thresh_max = 100000.
        self.dur_above = ADXL_DUR_SCALE
        self.dur_max = 0.1

    def init(self, profile):
        regs = self.regs
        regs.set_reg(adxl345.REG_POWER_CTL, 0x00)
        regs.set_reg(adxl345.REG_DATA_FORMAT, 0x2B if self.inverted else 0x0B)
        regs.set_reg(ADXL_REG_INT_MAP, self.int_map)
        regs.set_reg(ADXL_REG_INT_ENABLE, 0x00)
        self.set_profile(profile)

    def set_profile(self, profile, minclock=0):
        # The profile mask is written to TAP_AXES unchanged, as it has been
        # since f5ca5fb63. TAP_AXES is TAP_X=0x04, TAP_Y=0x02, TAP_Z=0x01,
        # so 'x' (0x01) enables the chip's Z tap and 'z' enables X. A1
        # tuning was done that way; changing it needs an A1 bench run.
        regs = self.regs
        regs.set_reg(ADXL_REG_TAP_AXES, profile['tap_axes_mask'],
                     minclock=minclock)
        regs.set_reg(ADXL_REG_THRESH_TAP,
                     int(profile['tap_thresh'] / ADXL_TAP_SCALE),
                     minclock=minclock)
        regs.set_reg(ADXL_REG_DUR, int(profile['tap_dur'] / ADXL_DUR_SCALE),
                     minclock=minclock)

    def describe_profile(self, profile):
        return ""

    def effective(self, profile):
        return {
            'tap_thresh': (int(profile['tap_thresh'] / ADXL_TAP_SCALE)
                           * ADXL_TAP_SCALE),
            'tap_dur': int(profile['tap_dur'] / ADXL_DUR_SCALE)
                       * ADXL_DUR_SCALE,
        }

    def arm(self, minclock=0):
        self.regs.set_reg(ADXL_REG_INT_ENABLE, ADXL_INT_SINGLE_TAP,
                          minclock=minclock)

    def disarm(self, minclock=0):
        self.regs.set_reg(ADXL_REG_INT_ENABLE, 0x00, minclock=minclock)

    def read_source(self, minclock=0):
        return self.regs.read_reg(ADXL_REG_INT_SOURCE, minclock=minclock)

    def source_has_tap(self, val):
        return bool(val & ADXL_INT_SINGLE_TAP)

    def power_up(self, minclock=0):
        was_measuring = self.regs.read_reg(adxl345.REG_POWER_CTL) == 0x08
        if not was_measuring:
            self.regs.set_reg(adxl345.REG_POWER_CTL, 0x08, minclock=minclock)
        return was_measuring

    def power_restore(self, was_measuring, minclock=0):
        if not was_measuring:
            self.regs.set_reg(adxl345.REG_POWER_CTL, 0x00, minclock=minclock)

    def describe_trigger(self):
        return None


######################################################################
# LIS2DW12
######################################################################

LIS_REG_CTRL1 = lis2dw.REG_LIS2DW_CTRL_REG1_ADDR
LIS_REG_CTRL2 = lis2dw.REG_LIS2DW_CTRL_REG2_ADDR
LIS_REG_CTRL3 = lis2dw.REG_LIS2DW_CTRL_REG3_ADDR
LIS_REG_CTRL4_INT1 = lis2dw.REG_LIS2DW_CTRL_REG4_ADDR
LIS_REG_CTRL6 = lis2dw.REG_LIS2DW_CTRL_REG6_ADDR
LIS_REG_TAP_THS_X = 0x30
LIS_REG_TAP_THS_Y = 0x31
LIS_REG_TAP_THS_Z = 0x32
LIS_REG_INT_DUR = 0x33
LIS_REG_WAKE_UP_THS = 0x34
LIS_REG_WAKE_UP_DUR = 0x35
LIS_REG_WAKE_UP_SRC = 0x38
LIS_REG_TAP_SRC = 0x39
LIS_REG_ALL_INT_SRC = 0x3B
LIS_REG_CTRL7 = 0x3F

LIS_ODR_CODES = {400: 0x7, 800: 0x8, 1600: 0x9}
LIS_FS_CODES = {2: 0x0, 4: 0x1, 8: 0x2, 16: 0x3}
LIS_CTRL1_HIGH_PERF = 0x04       # MODE=01, 14-bit
LIS_CTRL2_SPI = 0x06             # IF_ADD_INC | I2C_DISABLE, as lis2dw.py
LIS_CTRL3_LIR = 0x10             # latch until the source is read
LIS_CTRL3_H_LACTIVE = 0x08       # interrupt pin active low
LIS_CTRL6_LOW_NOISE = 0x04       # BW_FILT=ODR/2, FDS=0
LIS_CTRL7_INTERRUPTS_ENABLE = 0x20
LIS_INT1_SINGLE_TAP = 0x40
LIS_INT1_WU = 0x20
LIS_ALL_INT_SINGLE_TAP = 0x04
LIS_ALL_INT_WU = 0x02
LIS_TAP_EN_BITS = (0x80, 0x40, 0x20)   # TAP_X_EN, TAP_Y_EN, TAP_Z_EN
LIS_SRC_AXIS_BITS = (0x04, 0x02, 0x01)  # X, Y, Z in TAP_SRC / WAKE_UP_SRC
LIS_TAP_SIGN = 0x08
LIS_WK_THS_MAX = 0x3F
LIS_TAP_THS_MAX = 0x1F

# What lis2dw.py's _start_measurements() writes for streaming.
LIS_STREAM_CTRL1 = 0x94          # 1600Hz high performance
LIS_STREAM_CTRL6 = 0x34          # +-16g

LIS_EVENTS = {'single_tap': 'single_tap', 'wake_up': 'wake_up'}


class LIS2DW12TapChip:
    name = "LIS2DW12"

    def __init__(self, config, chip, regs, int_pin, inverted):
        printer = config.get_printer()
        self.command_error = printer.command_error
        if int_pin != 'int1':
            raise config.error(
                "LIS2DW12 can route tap and wake-up events to INT1 only;"
                " int_pin must be int1")
        if chip.bus_type != lis2dw.SPI_SERIAL_TYPE:
            raise config.error(
                "adxl345_probe needs the LIS2DW12 on SPI: tap arming is"
                " scheduled with minclock, which the I2C reads do not take")
        self.chip = chip
        self.regs = regs
        self.inverted = inverted
        self.event = config.getchoice('lis2dw_tap_event', LIS_EVENTS,
                                      'single_tap')
        self.odr = config.getchoice('lis2dw_odr',
                                    {r: r for r in LIS_ODR_CODES}, 1600)
        self.full_scale = config.getchoice(
            'lis2dw_full_scale', {g: g for g in LIS_FS_CODES}, 4)
        # Printer axis i is chip axis chip_axes[i], from [lis2dw] axes_map.
        self.chip_axes = [pos for pos, scale in chip.axes_map]
        self.chip_signs = [1 if scale > 0. else -1
                           for pos, scale in chip.axes_map]
        if sorted(self.chip_axes) != [0, 1, 2]:
            raise config.error(
                "adxl345_probe needs the axes_map of [%s] to name each chip"
                " axis exactly once" % (chip.name,))
        self.ctrl1 = (LIS_ODR_CODES[self.odr] << 4) | LIS_CTRL1_HIGH_PERF
        self.ctrl6 = (LIS_FS_CODES[self.full_scale] << 4) | LIS_CTRL6_LOW_NOISE
        # TAP_THS is 5 bits of FS/32; WK_THS is 6 bits of FS/64.
        if self.event == 'single_tap':
            self.thresh_lsb = self.full_scale * FREEFALL_ACCEL / 32.
            self.thresh_max_code = LIS_TAP_THS_MAX
            # SHOCK window: 0 -> 4/ODR, n -> 8*n/ODR.
            self.shock_windows = [4. / self.odr] + [
                8. * n / self.odr for n in (1, 2, 3)]
            self.dur_max = self.shock_windows[-1]
        else:
            self.thresh_lsb = self.full_scale * FREEFALL_ACCEL / 64.
            self.thresh_max_code = LIS_WK_THS_MAX
            self.shock_windows = None
            # tap_dur is not used by the wake-up engine.
            self.dur_max = 0.1
        self.thresh_min = self.thresh_lsb
        self.thresh_max = self.thresh_max_code * self.thresh_lsb
        self.dur_above = 0.

    def _thresh_code(self, thresh):
        code = int(round(thresh / self.thresh_lsb))
        return max(1, min(self.thresh_max_code, code))

    def _shock_code(self, dur):
        for code, window in enumerate(self.shock_windows):
            if window >= dur - 1e-9:
                return code
        return len(self.shock_windows) - 1

    def _tap_enables(self, axes_mask):
        bits = 0
        for i, axis in enumerate('xyz'):
            if axes_mask & TAP_AXIS_BITS[axis]:
                bits |= LIS_TAP_EN_BITS[self.chip_axes[i]]
        return bits

    def init(self, profile):
        regs = self.regs
        if not isinstance(regs, FileOutputRegs):
            dev_id = regs.read_reg(lis2dw.REG_LIS2DW_WHO_AM_I_ADDR)
            if dev_id != lis2dw.LIS2DW_DEV_ID:
                raise self.command_error(
                    "Invalid lis2dw id (got %x vs %x).\n"
                    "This is generally indicative of connection problems\n"
                    "(e.g. faulty wiring) or a faulty lis2dw chip."
                    % (dev_id, lis2dw.LIS2DW_DEV_ID))
        regs.set_reg(LIS_REG_CTRL4_INT1, 0x00)
        regs.set_reg(LIS_REG_CTRL2, LIS_CTRL2_SPI)
        ctrl3 = LIS_CTRL3_LIR
        if self.inverted:
            ctrl3 |= LIS_CTRL3_H_LACTIVE
        regs.set_reg(LIS_REG_CTRL3, ctrl3)
        regs.set_reg(LIS_REG_CTRL7, LIS_CTRL7_INTERRUPTS_ENABLE)
        regs.set_reg(LIS_REG_WAKE_UP_DUR, 0x00)
        self.set_profile(profile)
        # Run continuously from here on, so arming a tap never follows a
        # power-up transient. power_up() re-applies ODR and full scale.
        regs.set_reg(LIS_REG_CTRL6, self.ctrl6)
        regs.set_reg(LIS_REG_CTRL1, self.ctrl1)

    def set_profile(self, profile, minclock=0):
        regs = self.regs
        code = self._thresh_code(profile['tap_thresh'])
        if self.event == 'single_tap':
            regs.set_reg(LIS_REG_TAP_THS_X, code, minclock=minclock)
            regs.set_reg(LIS_REG_TAP_THS_Y, code, minclock=minclock)
            regs.set_reg(LIS_REG_TAP_THS_Z,
                         self._tap_enables(profile['tap_axes_mask']) | code,
                         minclock=minclock)
            # LATENCY=0, QUIET=0 (2/ODR), SHOCK from tap_dur.
            regs.set_reg(LIS_REG_INT_DUR, self._shock_code(profile['tap_dur']),
                         minclock=minclock)
            # Single tap only; wake-up parked at its highest threshold.
            regs.set_reg(LIS_REG_WAKE_UP_THS, LIS_WK_THS_MAX,
                         minclock=minclock)
        else:
            regs.set_reg(LIS_REG_TAP_THS_Z, 0x00, minclock=minclock)
            regs.set_reg(LIS_REG_WAKE_UP_THS, code, minclock=minclock)

    def effective(self, profile):
        res = {'tap_thresh': (self._thresh_code(profile['tap_thresh'])
                              * self.thresh_lsb)}
        if self.event == 'single_tap':
            res['tap_dur'] = self.shock_windows[
                self._shock_code(profile['tap_dur'])]
        else:
            res['tap_dur'] = None
        return res

    def describe_profile(self, profile):
        eff = self.effective(profile)
        code = self._thresh_code(profile['tap_thresh'])
        if self.event == 'single_tap':
            return (" -> LIS2DW12 single_tap TAP_THS=%d (%.0f mm/s^2, %.3fg)"
                    " SHOCK=%.4fs @%dHz +-%dg"
                    % (code, eff['tap_thresh'],
                       eff['tap_thresh'] / FREEFALL_ACCEL, eff['tap_dur'],
                       self.odr, self.full_scale))
        return (" -> LIS2DW12 wake_up WK_THS=%d (%.0f mm/s^2, %.3fg)"
                " on any axis @%dHz +-%dg"
                % (code, eff['tap_thresh'], eff['tap_thresh'] / FREEFALL_ACCEL,
                   self.odr, self.full_scale))

    def arm(self, minclock=0):
        if self.event == 'single_tap':
            val = LIS_INT1_SINGLE_TAP
        else:
            val = LIS_INT1_WU
        self.regs.set_reg(LIS_REG_CTRL4_INT1, val, minclock=minclock)

    def disarm(self, minclock=0):
        self.regs.set_reg(LIS_REG_CTRL4_INT1, 0x00, minclock=minclock)

    def read_source(self, minclock=0):
        # Reading ALL_INT_SRC releases every latched (LIR) interrupt.
        return self.regs.read_reg(LIS_REG_ALL_INT_SRC, minclock=minclock)

    def source_has_tap(self, val):
        if self.event == 'single_tap':
            return bool(val & LIS_ALL_INT_SINGLE_TAP)
        return bool(val & LIS_ALL_INT_WU)

    def power_up(self, minclock=0):
        # Tap thresholds are relative to full scale, and lis2dw.py's stream
        # rewrites ODR and full scale, so restore ours. Changing them while
        # a stream runs would corrupt its samples.
        if (getattr(self.chip.batch_bulk, 'is_started', False)
                and (self.ctrl1, self.ctrl6) != (LIS_STREAM_CTRL1,
                                                 LIS_STREAM_CTRL6)):
            raise self.command_error(
                "LIS2DW12 is streaming at 1600Hz +-16g; tap detection while"
                " streaming needs lis2dw_odr: 1600 and lis2dw_full_scale: 16")
        for reg, val in ((LIS_REG_CTRL6, self.ctrl6),
                         (LIS_REG_CTRL1, self.ctrl1)):
            if self.regs.read_reg(reg) != val:
                logging.info("LIS2DW12 tap: restoring reg 0x%02x to 0x%02x",
                             reg, val)
                self.regs.set_reg(reg, val, minclock=minclock)
        return True

    def power_restore(self, was_measuring, minclock=0):
        # The chip stays powered; see init().
        pass

    def chip_to_printer(self, chip_idx):
        i = self.chip_axes.index(chip_idx)
        return "%s%s" % ('-' if self.chip_signs[i] < 0 else '', 'xyz'[i])

    def describe_trigger(self):
        if self.event == 'single_tap':
            reg, label = LIS_REG_TAP_SRC, "TAP_SRC"
        else:
            reg, label = LIS_REG_WAKE_UP_SRC, "WAKE_UP_SRC"
        val = self.regs.read_reg(reg)
        axes = ["chip %s = printer %s" % ('XYZ'[c], self.chip_to_printer(c))
                for c in range(3) if val & LIS_SRC_AXIS_BITS[c]]
        msg = "%s=0x%02x: %s" % (label, val, ", ".join(axes) or "no axis")
        if self.event == 'single_tap':
            msg += " TAP_SIGN=%d" % (1 if val & LIS_TAP_SIGN else 0,)
        return msg


def lookup_tap_chip(config, chip, regs, int_pin, inverted):
    if isinstance(chip, adxl345.ADXL345):
        return ADXL345TapChip(config, chip, regs, int_pin, inverted)
    if (isinstance(chip, lis2dw.LIS2DW)
            and chip.lis_type == lis2dw.LIS2DW_TYPE):
        return LIS2DW12TapChip(config, chip, regs, int_pin, inverted)
    raise config.error(
        "adxl345_probe chip '%s' is not an adxl345 or lis2dw section"
        % (config.get('chip', 'adxl345'),))
