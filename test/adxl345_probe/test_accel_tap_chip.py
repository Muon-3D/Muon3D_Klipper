"""adxl345_probe tap backends: the register values test/klippy cannot see.

test/klippy/adxl345_probe*.test and lis2dw_probe*.test drive the probe in
file-output mode, where every register write is dropped and every read
returns 0. This drives the real backends with a recording register fake and
asserts what reaches the chip:

* the ADXL345 bytes are exactly what A1 wrote before the LIS2DW12 port
* LIS2DW12 thresholds, shock windows and axis enables are quantised and
  mapped through [lis2dw] axes_map as documented in Config_Reference.md
* the LIS2DW12 configurations that cannot work are refused at config time

    python3 test/adxl345_probe/test_accel_tap_chip.py
"""
import os
import sys
import unittest

KLIPPY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

from extras import accel_tap_chip, adxl345, lis2dw  # noqa: E402

G = 9806.65  # mm/s^2


class ConfigError(Exception):
    pass


class CommandError(Exception):
    pass


class FakePrinter:
    command_error = CommandError


class FakeConfig:
    error = ConfigError

    def __init__(self, values=None):
        self.values = values or {}

    def get_printer(self):
        return FakePrinter()

    def get(self, name, default=None):
        return self.values.get(name, default)

    def getchoice(self, name, choices, default=None):
        c = self.values.get(name, default)
        if c not in choices:
            raise ConfigError("bad choice %r for %s" % (c, name))
        return choices[c]


class FakeRegs:
    def __init__(self, reads=None):
        self.writes = []
        self.reads = dict(reads or {})
        self.read_log = []

    def set_reg(self, reg, val, minclock=0):
        self.writes.append((reg, val))
        self.reads[reg] = val

    def read_reg(self, reg, minclock=0, reqclock=0):
        self.read_log.append(reg)
        return self.reads.get(reg, 0)

    def check_connected(self):
        pass


class FakeBulk:
    def __init__(self, started=False):
        self.is_started = started


def make_lis(axes_map=('x', 'y', 'z'), bus_type='spi', lis_type=None,
             streaming=False):
    # A real LIS2DW object without the MCU behind it.
    chip = lis2dw.LIS2DW.__new__(lis2dw.LIS2DW)
    chip.name = 'lis2dw'
    chip.bus_type = bus_type
    chip.lis_type = lis_type or lis2dw.LIS2DW_TYPE
    chip.batch_bulk = FakeBulk(streaming)
    am = {'x': (0, 1.), 'y': (1, 1.), 'z': (2, 1.),
          '-x': (0, -1.), '-y': (1, -1.), '-z': (2, -1.)}
    chip.axes_map = [am[a] for a in axes_map]
    return chip


def lis_backend(values=None, int_pin='int1', inverted=False, regs=None,
                **chip_kw):
    regs = regs if regs is not None else FakeRegs(
        {lis2dw.REG_LIS2DW_WHO_AM_I_ADDR: lis2dw.LIS2DW_DEV_ID})
    chip = make_lis(**chip_kw)
    return accel_tap_chip.lookup_tap_chip(
        FakeConfig(values), chip, regs, int_pin, inverted), regs


def profile(thresh, dur, axes='x'):
    mask = 0
    for a in axes:
        mask |= {'x': 1, 'y': 2, 'z': 4}[a]
    return {'tap_thresh': thresh, 'tap_dur': dur, 'tap_axes_mask': mask}


class ADXL345Unchanged(unittest.TestCase):
    """A1 must keep writing the bytes it wrote before this port."""

    def backend(self, int_pin='int1', inverted=False):
        chip = adxl345.ADXL345.__new__(adxl345.ADXL345)
        regs = FakeRegs()
        return accel_tap_chip.lookup_tap_chip(
            FakeConfig(), chip, regs, int_pin, inverted), regs

    def test_x_profile_bytes(self):
        # core/M1/core.cfg: x_tap_thresh 20000, tap_dur 0.03, x_tap_axes x
        b, regs = self.backend()
        b.set_profile(profile(20000., 0.03, 'x'))
        # TAP_AXES 0x01 is the chip's Z tap enable; see set_profile().
        self.assertEqual(regs.writes, [(0x2A, 0x01), (0x1D, 32), (0x21, 48)])

    def test_y_profile_bytes(self):
        b, regs = self.backend()
        b.set_profile(profile(15000., 0.03, 'y'))
        self.assertEqual(regs.writes, [(0x2A, 0x02), (0x1D, 24), (0x21, 48)])

    def test_init_and_interrupt_bytes(self):
        b, regs = self.backend(int_pin='int2', inverted=True)
        b.init(profile(5000., 0.01, 'xyz'))
        self.assertEqual(regs.writes[:4],
                         [(0x2D, 0x00), (0x31, 0x2B), (0x2F, 0x40),
                          (0x2E, 0x00)])
        regs.writes = []
        b.arm()
        b.disarm()
        self.assertEqual(regs.writes, [(0x2E, 0x40), (0x2E, 0x00)])
        self.assertTrue(b.source_has_tap(0x40))
        self.assertFalse(b.source_has_tap(0x80))

    def test_power(self):
        b, regs = self.backend()
        self.assertFalse(b.power_up())
        self.assertEqual(regs.writes, [(0x2D, 0x08)])
        b.power_restore(False)
        self.assertEqual(regs.writes[-1], (0x2D, 0x00))
        regs.writes = []
        regs.reads[0x2D] = 0x08         # already measuring (a stream)
        self.assertTrue(b.power_up())
        b.power_restore(True)
        self.assertEqual(regs.writes, [])
        self.assertIsNone(b.describe_trigger())
        self.assertEqual(b.describe_profile(profile(1., 1.)), "")


class LIS2DW12Thresholds(unittest.TestCase):

    def test_single_tap_4g(self):
        b, _ = lis_backend()
        self.assertAlmostEqual(b.thresh_min, 4 * G / 32)
        self.assertAlmostEqual(b.thresh_max, 31 * 4 * G / 32)
        # 125mg steps: 1.53g -> 12 (1.5g), 2.04g -> 16 (2.0g)
        self.assertEqual(b._thresh_code(15000.), 12)
        self.assertEqual(b._thresh_code(20000.), 16)
        self.assertAlmostEqual(b.effective(profile(20000., .01))['tap_thresh'],
                               2. * G)

    def test_single_tap_16g(self):
        b, _ = lis_backend({'lis2dw_full_scale': 16})
        self.assertEqual(b._thresh_code(15000.), 3)
        self.assertEqual(b._thresh_code(20000.), 4)

    def test_wake_up_4g(self):
        b, _ = lis_backend({'lis2dw_tap_event': 'wake_up'})
        self.assertAlmostEqual(b.thresh_min, 4 * G / 64)
        self.assertAlmostEqual(b.thresh_max, 63 * 4 * G / 64)
        self.assertEqual(b._thresh_code(15000.), 24)
        self.assertEqual(b._thresh_code(20000.), 33)
        self.assertIsNone(b.effective(profile(20000., .03))['tap_dur'])

    def test_code_is_clamped(self):
        b, _ = lis_backend()
        self.assertEqual(b._thresh_code(1.), 1)
        self.assertEqual(b._thresh_code(1e9), 31)


class LIS2DW12ShockWindow(unittest.TestCase):

    def test_1600hz(self):
        b, _ = lis_backend()
        self.assertAlmostEqual(b.dur_max, 0.015)
        for dur, code in ((0.0025, 0), (0.003, 1), (0.005, 1), (0.01, 2),
                          (0.015, 3)):
            self.assertEqual(b._shock_code(dur), code, dur)
        self.assertAlmostEqual(b.effective(profile(1e4, .003))['tap_dur'],
                               .005)

    def test_800hz_fits_a1_tap_dur(self):
        b, _ = lis_backend({'lis2dw_odr': 800})
        self.assertAlmostEqual(b.dur_max, 0.03)
        self.assertEqual(b._shock_code(0.03), 3)

    def test_wake_up_ignores_dur(self):
        b, _ = lis_backend({'lis2dw_tap_event': 'wake_up'})
        self.assertAlmostEqual(b.dur_max, 0.1)


class LIS2DW12Registers(unittest.TestCase):

    def test_single_tap_profile_bytes(self):
        b, regs = lis_backend()
        b.set_profile(profile(15000., 0.015, 'y'))
        self.assertEqual(regs.writes, [
            (0x30, 12), (0x31, 12), (0x32, 0x40 | 12),  # TAP_Y_EN
            (0x33, 0x03),                               # SHOCK=3, QUIET=0
            (0x34, 0x3F)])                              # single tap only

    def test_wake_up_profile_bytes(self):
        b, regs = lis_backend({'lis2dw_tap_event': 'wake_up'})
        b.set_profile(profile(15000., 0.03, 'x'))
        self.assertEqual(regs.writes, [(0x32, 0x00), (0x34, 24)])

    def test_axes_map_moves_the_enable(self):
        # B1 orientation is unvalidated: printer x read from chip y.
        b, regs = lis_backend(axes_map=('y', '-x', 'z'))
        b.set_profile(profile(15000., 0.01, 'x'))
        self.assertEqual(regs.writes[2], (0x32, 0x40 | 12))   # TAP_Y_EN
        regs.writes = []
        b.set_profile(profile(15000., 0.01, 'y'))
        self.assertEqual(regs.writes[2], (0x32, 0x80 | 12))   # TAP_X_EN
        regs.writes = []
        b.set_profile(profile(15000., 0.01, 'xyz'))
        self.assertEqual(regs.writes[2], (0x32, 0xE0 | 12))

    def test_init_bytes(self):
        b, regs = lis_backend()
        b.init(profile(5000., 0.01, 'xyz'))
        self.assertEqual(regs.read_log[0], 0x0F)
        self.assertEqual(regs.writes[:5], [
            (0x23, 0x00),    # INT1 routing off
            (0x21, 0x06),    # IF_ADD_INC | I2C_DISABLE
            (0x22, 0x10),    # LIR: latched until read
            (0x3F, 0x20),    # INTERRUPTS_ENABLE
            (0x35, 0x00)])
        self.assertEqual(regs.writes[-2:], [(0x25, 0x14), (0x20, 0x94)])

    def test_init_inverted(self):
        b, regs = lis_backend(inverted=True)
        b.init(profile(5000., 0.01))
        self.assertIn((0x22, 0x18), regs.writes)

    def test_init_rejects_wrong_chip(self):
        b, regs = lis_backend(regs=FakeRegs({0x0F: 0x33}))
        with self.assertRaises(CommandError):
            b.init(profile(5000., 0.01))

    def test_odr_and_fs_bytes(self):
        b, _ = lis_backend({'lis2dw_odr': 400, 'lis2dw_full_scale': 16})
        self.assertEqual((b.ctrl1, b.ctrl6), (0x74, 0x34))
        b, _ = lis_backend({'lis2dw_odr': 800, 'lis2dw_full_scale': 2})
        self.assertEqual((b.ctrl1, b.ctrl6), (0x84, 0x04))

    def test_arm_and_source(self):
        b, regs = lis_backend()
        b.arm()
        b.disarm()
        self.assertEqual(regs.writes, [(0x23, 0x40), (0x23, 0x00)])
        self.assertEqual(b.read_source(), 0)
        self.assertEqual(regs.read_log[-1], 0x3B)       # ALL_INT_SRC
        self.assertTrue(b.source_has_tap(0x04))
        self.assertFalse(b.source_has_tap(0x02))
        w, regs = lis_backend({'lis2dw_tap_event': 'wake_up'})
        w.arm()
        self.assertEqual(regs.writes, [(0x23, 0x20)])
        self.assertTrue(w.source_has_tap(0x02))
        self.assertFalse(w.source_has_tap(0x04))

    def test_power_up_restores_after_stream(self):
        # A stream left the chip at 1600Hz +-16g; tap wants +-4g back.
        b, regs = lis_backend()
        regs.reads.update({0x25: 0x34, 0x20: 0x94})
        self.assertTrue(b.power_up())
        self.assertEqual(regs.writes, [(0x25, 0x14)])
        regs.writes = []
        b.power_up()
        self.assertEqual(regs.writes, [])
        b.power_restore(True)
        self.assertEqual(regs.writes, [])

    def test_power_up_refuses_to_change_a_running_stream(self):
        b, _ = lis_backend(streaming=True)
        with self.assertRaises(CommandError):
            b.power_up()
        b, regs = lis_backend({'lis2dw_full_scale': 16}, streaming=True)
        regs.reads.update({0x25: 0x34, 0x20: 0x94})
        b.power_up()
        self.assertEqual(regs.writes, [])

    def test_describe_trigger(self):
        b, regs = lis_backend(axes_map=('y', '-x', 'z'))
        regs.reads[0x39] = 0x40 | 0x20 | 0x08 | 0x04  # single tap, X, sign
        self.assertEqual(b.describe_trigger(),
                         "TAP_SRC=0x6c: chip X = printer -y TAP_SIGN=1")
        w, regs = lis_backend({'lis2dw_tap_event': 'wake_up'})
        regs.reads[0x38] = 0x08 | 0x02 | 0x01
        self.assertEqual(w.describe_trigger(),
                         "WAKE_UP_SRC=0x0b: chip Y = printer y,"
                         " chip Z = printer z")


class LIS2DW12Refusals(unittest.TestCase):

    def test_int2(self):
        with self.assertRaises(ConfigError):
            lis_backend(int_pin='int2')

    def test_i2c(self):
        with self.assertRaises(ConfigError):
            lis_backend(bus_type='i2c')

    def test_axes_map_not_a_permutation(self):
        with self.assertRaises(ConfigError):
            lis_backend(axes_map=('x', 'x', 'z'))

    def test_bad_choices(self):
        for values in ({'lis2dw_odr': 3200}, {'lis2dw_full_scale': 6},
                       {'lis2dw_tap_event': 'double_tap'}):
            with self.assertRaises(ConfigError):
                lis_backend(values)

    def test_lis3dh_is_not_supported(self):
        with self.assertRaises(ConfigError):
            lis_backend(lis_type=lis2dw.LIS3DH_TYPE)

    def test_unknown_chip_object(self):
        with self.assertRaises(ConfigError):
            accel_tap_chip.lookup_tap_chip(FakeConfig(), object(),
                                           FakeRegs(), 'int1', False)


if __name__ == '__main__':
    unittest.main()
