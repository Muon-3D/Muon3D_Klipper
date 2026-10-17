# Tests for klippy/extras/heater_supply.py: the resistance estimator, the
# module against stub hardware, and MPC closed-loop on a simulated bed.
import os
import random
import sys
import unittest

KLIPPY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

from extras import heater_supply  # noqa: E402
from extras.heater_supply import ResistanceEstimator  # noqa: E402
from extras.control_mpc import ControlMPC  # noqa: E402

DT = 0.1
R_TRUE = 12.0


def run(est, seconds, duty, volts, amps, powered=True, start=0.):
    """Feed constant readings; return (end time, first fault or None)."""
    t, fault = start, None
    while t < start + seconds - 1e-9:
        t += DT
        a = amps() if callable(amps) else amps
        r = est.update(t, duty, volts, a, powered)
        fault = fault or r
    return t, fault


class TestEstimator(unittest.TestCase):
    def make(self, r=11.0):
        return ResistanceEstimator(r, (r / 2., r * 2.))

    def test_learns_r_when_fully_on(self):
        est = self.make()
        run(est, 30., 1.0, 45., 45. / R_TRUE)
        self.assertAlmostEqual(est.resistance, R_TRUE, delta=0.01)

    def test_ignores_the_settle_time(self):
        est = self.make()
        run(est, 0.9, 1.0, 45., 45. / 20.)
        self.assertEqual(est.resistance, 11.0)
        self.assertIsNone(est.measured)

    def test_ignores_pwm_readings(self):
        # At partial duty IMON sees either 0 or the full current depending
        # on where in the PWM cycle the ADC samples. None of it is used.
        est = self.make()
        rnd = random.Random(1)
        run(est, 60., 0.5, 45.,
            lambda: rnd.choice((0., 45. / R_TRUE)))
        self.assertEqual(est.resistance, 11.0)

    def test_subtracts_the_idle_current(self):
        est = self.make()
        t, _ = run(est, 20., 0., 45., 0.2)         # Z column draws 0.2 A
        self.assertAlmostEqual(est.idle_current, 0.2, delta=0.01)
        run(est, 30., 1.0, 45., 45. / R_TRUE + 0.2, start=t)
        self.assertAlmostEqual(est.resistance, R_TRUE, delta=0.02)

    def test_open_heater_faults_after_fault_time(self):
        est = self.make()
        t, fault = run(est, 2.9, 1.0, 45., 0.)     # 1 s settle + 1.9 s
        self.assertIsNone(fault)
        t, fault = run(est, 0.3, 1.0, 45., 0., start=t)
        self.assertIn("no current", fault)
        self.assertEqual(est.resistance, 11.0)

    def test_low_resistance_faults(self):
        est = self.make()
        _t, fault = run(est, 4., 1.0, 45., 45. / 4.)
        self.assertIn("outside", fault)

    def test_short_glitch_is_not_a_fault(self):
        est = self.make()
        t, _ = run(est, 5., 1.0, 45., 45. / R_TRUE)
        t, fault = run(est, 1., 1.0, 45., 0., start=t)
        self.assertIsNone(fault)
        _t, fault = run(est, 5., 1.0, 45., 45. / R_TRUE, start=t)
        self.assertIsNone(fault)

    def test_unpowered_resets_the_windows(self):
        est = self.make()
        t, _ = run(est, 0.8, 1.0, 45., 0.)
        t, _ = run(est, 0.5, 1.0, 45., 0., powered=False, start=t)
        _t, fault = run(est, 2.5, 1.0, 45., 0., start=t)
        self.assertIsNone(fault)   # the settle time started again


# Stubs for HeaterSupply

class FakeConfigError(Exception):
    pass


class FakeConfig:
    def __init__(self, printer, name, opts):
        self.printer, self.name, self.opts = printer, name, opts
        self.error = FakeConfigError
    def get_printer(self):
        return self.printer
    def get_name(self):
        return self.name
    def get(self, key, default=None):
        return self.opts.get(key, default)
    def getfloat(self, key, default=None, above=None):
        return float(self.opts.get(key, default))
    def getfloatlist(self, key, default=None, count=None):
        return self.opts.get(key, default)


class FakeReactor:
    NOW = 0.
    def __init__(self):
        self.time = 0.
    def monotonic(self):
        return self.time
    def register_timer(self, callback, when):
        return callback
    def unregister_timer(self, timer):
        pass


class FakeMonitor:
    def __init__(self, volts):
        self.volts = volts
    def get_status(self, eventtime):
        return {'min_supply_voltage': self.volts}


class FakeConfigfile:
    def __init__(self):
        self.saved = {}
    def set(self, section, option, value):
        self.saved[(section, option)] = value


class FakeZPower:
    def __init__(self):
        self.state = 'ready'
        self.current = 0.
        self.faults = []
    def latch_fault(self, reason):
        self.faults.append(reason)
        self.state = 'fault'


class FakeHeater:
    def __init__(self):
        self.last_pwm_value = 0.


class FakePrinter:
    def __init__(self, objects):
        self.objects = objects
        self.reactor = FakeReactor()
        self.handlers = {}
    def get_reactor(self):
        return self.reactor
    def lookup_object(self, name, default=Exception):
        if name in self.objects:
            return self.objects[name]
        if default is Exception:
            raise KeyError(name)
        return default
    def register_event_handler(self, event, cb):
        self.handlers[event] = cb


class FakeToolhead:
    def get_extruder(self):
        return object()


class FakeHeaters:
    def __init__(self, heater):
        self.heater = heater
    def lookup_heater(self, name):
        return self.heater


def make_supply(volts=45., **opts):
    heater, zp, cf = FakeHeater(), FakeZPower(), FakeConfigfile()
    objects = {'heaters': FakeHeaters(heater), 'z_power': zp,
               'configfile': cf}
    if volts is not None:
        objects['tmc_driver_monitor'] = FakeMonitor(volts)
    printer = FakePrinter(objects)
    options = {'resistance': 11.9, 'voltage': 45.0,
               'current_source': 'z_power'}
    options.update(opts)
    supply = heater_supply.HeaterSupply(
        FakeConfig(printer, 'heater_supply heater_bed', options))
    printer.handlers['klippy:connect']()
    return supply, heater, zp, cf


def drive(supply, heater, zp, seconds, duty, amps, start=0.):
    t = start
    heater.last_pwm_value = duty
    zp.current = amps
    while t < start + seconds - 1e-9:
        t += DT
        supply._poll(t)
    return t


class TestHeaterSupply(unittest.TestCase):
    def test_available_power_tracks_the_rail(self):
        supply, _h, _zp, _cf = make_supply(volts=48.)
        self.assertAlmostEqual(supply.available_power(), 48. ** 2 / 11.9)
        supply.printer.objects['tmc_driver_monitor'].volts = 36.
        self.assertAlmostEqual(supply.available_power(), 36. ** 2 / 11.9)

    def test_falls_back_to_the_nominal_voltage(self):
        supply, _h, _zp, _cf = make_supply(volts=None)
        self.assertEqual(supply.voltage(), 45.)
        supply.printer.objects['tmc_driver_monitor'] = FakeMonitor(None)
        self.assertEqual(supply.voltage(), 45.)
        supply.printer.objects['tmc_driver_monitor'] = FakeMonitor(0.2)
        self.assertEqual(supply.voltage(), 45.)

    def test_stages_a_learnt_resistance_when_the_window_ends(self):
        supply, heater, zp, cf = make_supply(resistance=11.0)
        t = drive(supply, heater, zp, 20., 1.0, 45. / R_TRUE)
        self.assertEqual(cf.saved, {})          # still fully on
        drive(supply, heater, zp, 0.2, 0.4, 1.0, start=t)
        self.assertAlmostEqual(
            float(cf.saved[('heater_supply heater_bed', 'resistance')]),
            R_TRUE, delta=0.05)

    def test_does_not_stage_a_small_change(self):
        supply, heater, zp, cf = make_supply(resistance=R_TRUE)
        t = drive(supply, heater, zp, 20., 1.0, 45. / R_TRUE)
        drive(supply, heater, zp, 0.2, 0.4, 1.0, start=t)
        self.assertEqual(cf.saved, {})

    def test_open_heater_latches_one_z_power_fault(self):
        supply, heater, zp, _cf = make_supply()
        t = drive(supply, heater, zp, 4., 1.0, 0.)
        zp.state = 'ready'                       # as if still latching
        drive(supply, heater, zp, 1., 1.0, 0., start=t)
        self.assertEqual(len(zp.faults), 1)
        self.assertIn("heater_bed: the heater draws no current",
                      zp.faults[0])

    def test_bad_range_is_a_config_error(self):
        with self.assertRaises(FakeConfigError):
            make_supply(resistance_range=(12., 20.))


# MPC closed-loop on a simulated bed

PROFILE = {
    "block_heat_capacity": 319.565, "ambient_transfer": 1.19495,
    "target_reach_time": 2.0, "heater_power": 170.0, "smoothing": 0.83,
    "sensor_responsiveness": 0.113720, "min_ambient_change": 1.0,
    "steady_state_rate": 0.5, "filament_diameter": 1.75,
    "filament_density": 1.2, "filament_heat_capacity": 0.0,
    "maximum_retract": 2.0, "filament_temp_src": ("ambient",),
    "ambient_temp_sensor": None, "cooling_fan": None,
    "fan_ambient_transfer": [],
}
REPORT_TIME = 0.3
AMBIENT = 25.


class SimBed:
    """The plant MPC models, with a heater whose R rises with temperature
    and a rail that is not the 45 V the profile assumes."""
    def __init__(self, volts, r25=11.9, alpha=0.002):
        self.volts, self.r25, self.alpha = volts, r25, alpha
        self.block = self.sensor = AMBIENT
        self.duty = 0.
        self.reactor = FakeReactor()
    def resistance(self):
        return self.r25 * (1. + self.alpha * (self.block - AMBIENT))
    def step(self, dt):
        power = self.duty * self.volts ** 2 / self.resistance()
        self.block += ((power - PROFILE["ambient_transfer"]
                        * (self.block - AMBIENT))
                       * dt / PROFILE["block_heat_capacity"])
        self.sensor += ((self.block - self.sensor)
                        * PROFILE["sensor_responsiveness"] * dt)
    # The heater interface ControlMPC uses
    def get_max_power(self):
        return 1.0
    def get_temp(self, eventtime):
        return self.sensor, 0.
    def get_name(self):
        return "heater_bed"
    def set_pwm(self, read_time, value):
        self.duty = value
        self.last_pwm_value = value


def simulate(volts, with_supply, target=60., seconds=900.):
    """Run MPC on the simulated bed. `volts` is a number or f(t)."""
    rail = volts if callable(volts) else (lambda _t: volts)
    bed = SimBed(rail(0.))
    bed.last_pwm_value = 0.
    objects = {'toolhead': FakeToolhead()}
    if with_supply:
        zp = FakeZPower()
        objects.update({'heaters': FakeHeaters(bed), 'z_power': zp,
                        'configfile': FakeConfigfile(),
                        'tmc_driver_monitor': FakeMonitor(volts)})
    printer = FakePrinter(objects)
    bed.printer = printer
    if with_supply:
        options = {'resistance': 11.9, 'voltage': 45.0,
                   'current_source': 'z_power'}
        supply = heater_supply.HeaterSupply(
            FakeConfig(printer, 'heater_supply heater_bed', options))
        printer.handlers['klippy:connect']()
        objects['heater_supply heater_bed'] = supply
    control = ControlMPC(dict(PROFILE), bed, load_clean=True,
                         register=False)
    t, temps, sub = 0., [], 0.
    rnd = random.Random(2)
    while t < seconds:
        t += DT
        bed.volts = rail(t)
        bed.step(DT)
        if with_supply:
            objects['tmc_driver_monitor'].volts = bed.volts
            # IMON as the ADC sees it: steady when fully on, a random
            # point of the PWM cycle otherwise.
            on = bed.duty >= 1. or rnd.random() < bed.duty
            zp.current = (bed.volts / bed.resistance()) if on else 0.
            supply._poll(t)
        sub += DT
        if sub >= REPORT_TIME - 1e-9:
            sub = 0.
            control.temperature_update(t, bed.sensor, target)
            temps.append((t, bed.sensor))
    return temps, control


def overshoot(temps, target):
    return max(temp for _t, temp in temps) - target


def final_error(temps, target, last=120.):
    end = temps[-1][0]
    tail = [temp for t, temp in temps if t >= end - last]
    return max(abs(temp - target) for temp in tail)


class TestClosedLoop(unittest.TestCase):
    def test_holds_target_on_any_rail(self):
        for volts in (36., 45., 50.):
            temps, control = simulate(volts, with_supply=True)
            self.assertLess(overshoot(temps, 60.), 1.0, volts)
            self.assertLess(final_error(temps, 60.), 0.3, volts)
            # MPC's idea of full power matches the plant's at the end.
            bed_power = volts ** 2 / (11.9 * (1. + 0.002 * 35.))
            self.assertAlmostEqual(control.heater_power(), bed_power,
                                   delta=0.02 * bed_power)

    def test_supply_is_no_worse_than_fixed_power(self):
        for volts in (36., 50.):
            fixed, _c = simulate(volts, with_supply=False)
            live, _c = simulate(volts, with_supply=True)
            self.assertLessEqual(overshoot(live, 60.),
                                 max(overshoot(fixed, 60.), 0.) + 0.1)

    def test_model_stays_true_through_a_voltage_step(self):
        # With a fixed heater_power MPC still holds the target after the
        # rail drops, but only by dragging its ambient estimate down to
        # explain the missing watts (to about 0 C here), which the next
        # heat-up then inherits. With the live supply it stays near 25 C.
        def rail(t):
            return 48. if t < 600. else 36.
        temps, control = simulate(rail, with_supply=True, seconds=1500.)
        self.assertLess(final_error(temps, 60.), 0.3)
        self.assertAlmostEqual(control.state_ambient_temp, AMBIENT,
                               delta=5.)
        _temps, fixed = simulate(rail, with_supply=False, seconds=1500.)
        self.assertLess(fixed.state_ambient_temp, AMBIENT - 15.)

    def test_without_a_supply_mpc_is_unchanged(self):
        bed = SimBed(45.)
        bed.printer = FakePrinter({})
        control = ControlMPC(dict(PROFILE), bed, load_clean=True,
                             register=False)
        self.assertEqual(control.heater_power(), 170.)


if __name__ == '__main__':
    unittest.main()
