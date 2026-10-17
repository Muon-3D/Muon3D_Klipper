# Live supply voltage and heater resistance for an MPC-controlled heater
#
# Copyright (C) 2026  Muon3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# MPC asks for watts and turns them into a duty cycle. On its own it divides
# by a fixed heater_power, so on a rail other than the one it was calibrated
# on it delivers the wrong power: a resistive heater's power goes with V^2.
# This module gives MPC the power the heater really delivers fully on, now:
#
#   V^2 / R
#   V  the 48 V rail as the TMC2240/TMC2241 drivers read it
#      ([tmc_driver_monitor]), or `voltage` when there is no reading.
#   R  the heater resistance, learnt from the [z_power] eFuse current and
#      kept in the calibration file (configfile.set; the next SAVE_CONFIG
#      writes it).
#
# R is only learnt once the heater has been fully on for `settle_time`. At
# any lower duty the PWM makes the IMON reading depend on where in the cycle
# the ADC happens to sample, so it is not used. A heat-up from cold is always
# fully on, so R is refreshed on every heat-up and ends at its hot value,
# which is the one steady-state control needs.
#
# The same reading checks the heater: fully on, with a resistance outside
# `resistance_range` for `fault_time` (an open heater reads as infinite),
# latches a [z_power] fault, which turns the bed power off.
import logging

POLL_INTERVAL = 0.1
FULL_DUTY = 0.999      # last_pwm_value at or above this is "fully on"
MIN_AMPS = 0.01        # below this the heater draws nothing measurable
MIN_VOLTS = 1.0        # a supply reading below this is not a reading
SAVE_CHANGE = 0.01     # stage a new R once it moves this fraction
SAVE_SAMPLES = 10      # in-band samples a window needs before R is staged


class ResistanceEstimator:
    """Learns a heater's resistance from (duty, volts, amps) readings.

    No Klipper objects, so the test suite can drive it directly. update()
    returns a fault reason, or None.
    """
    def __init__(self, resistance, resistance_range, settle_time=1.0,
                 fault_time=2.0, learn_time=3.0):
        self.resistance = resistance
        self.low, self.high = resistance_range
        self.settle_time = settle_time
        self.fault_time = fault_time
        self.learn_time = learn_time
        self.idle_current = 0.
        self.measured = None        # last fully-on resistance reading
        self.window_samples = 0     # in-band samples in this fully-on window
        self.full_since = self.off_since = self.bad_since = None
        self.last_time = None
    def _alpha(self, eventtime):
        dt = 0. if self.last_time is None else eventtime - self.last_time
        return max(0., min(1., dt / self.learn_time))
    def update(self, eventtime, duty, volts, amps, powered):
        alpha = self._alpha(eventtime)
        self.last_time = eventtime
        if not powered or volts is None or amps is None:
            self.full_since = self.off_since = self.bad_since = None
            return None
        # Which steady window are we in, if any?
        if duty >= FULL_DUTY:
            self.off_since = None
            if self.full_since is None:
                self.full_since = eventtime
                self.window_samples = 0
        elif duty <= 0.:
            self.full_since = None
            if self.off_since is None:
                self.off_since = eventtime
        else:
            self.full_since = self.off_since = None
        # Heater off: what the eFuse still carries is the Z column itself.
        if (self.off_since is not None
                and eventtime - self.off_since >= self.settle_time):
            self.idle_current += (amps - self.idle_current) * alpha
        if (self.full_since is None
                or eventtime - self.full_since < self.settle_time):
            self.bad_since = None
            return None
        # Heater fully on and settled: the reading is the heater's own.
        heater_amps = amps - self.idle_current
        r = volts / heater_amps if heater_amps > MIN_AMPS else float('inf')
        self.measured = r
        if self.low <= r <= self.high:
            self.bad_since = None
            self.resistance += (r - self.resistance) * alpha
            self.window_samples += 1
            return None
        if self.bad_since is None:
            self.bad_since = eventtime
        if eventtime - self.bad_since < self.fault_time:
            return None
        if r == float('inf'):
            return ("the heater draws no current when fully on: check the"
                    " bed plate contacts and the heater")
        return ("the heater measures %.1f ohm fully on (%.2f A at %.1f V),"
                " outside %.1f-%.1f ohm" % (r, heater_amps, volts, self.low,
                                            self.high))


class HeaterSupply:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.reactor = printer.get_reactor()
        self.name = config.get_name()
        self.heater_name = self.name.split()[-1]
        resistance = config.getfloat('resistance', above=0.)
        rng = config.getfloatlist('resistance_range',
                                  (resistance / 2., resistance * 2.), count=2)
        if not 0. < rng[0] < resistance < rng[1]:
            raise config.error("%s: resistance_range must be 'low, high'"
                               " around resistance" % (self.name,))
        self.estimator = ResistanceEstimator(
            resistance, rng,
            settle_time=config.getfloat('settle_time', 1.0, above=0.),
            fault_time=config.getfloat('fault_time', 2.0, above=0.),
            learn_time=config.getfloat('learn_time', 3.0, above=0.))
        self.saved_resistance = resistance
        self.nominal_voltage = config.getfloat('voltage', above=MIN_VOLTS)
        self.current_source_name = config.get('current_source', None)
        self.current_source = None
        self.heater = None
        self.fault = None
        self.timer = None
        printer.register_event_handler('klippy:connect', self._handle_connect)
        printer.register_event_handler('klippy:ready', self._handle_ready)
        printer.register_event_handler('klippy:shutdown', self._stop)
    def _handle_connect(self):
        pheaters = self.printer.lookup_object('heaters')
        self.heater = pheaters.lookup_heater(self.heater_name)
        if self.current_source_name is not None:
            self.current_source = self.printer.lookup_object(
                self.current_source_name)
    def _handle_ready(self):
        if self.current_source is not None:
            self.timer = self.reactor.register_timer(self._poll,
                                                     self.reactor.NOW)
    def _stop(self):
        if self.timer is not None:
            self.reactor.unregister_timer(self.timer)
            self.timer = None
    # What MPC asks for
    def voltage(self):
        monitor = self.printer.lookup_object('tmc_driver_monitor', None)
        if monitor is not None:
            volts = monitor.get_status(None).get('min_supply_voltage')
            if volts is not None and volts >= MIN_VOLTS:
                return volts
        return self.nominal_voltage
    def available_power(self):
        """Watts the heater delivers at 100% duty, right now."""
        volts = self.voltage()
        return volts * volts / self.estimator.resistance
    # Learning
    def _poll(self, eventtime):
        src = self.current_source
        powered = src.state == 'ready'
        duty = self.heater.last_pwm_value
        was_full = self.estimator.full_since is not None
        reason = self.estimator.update(eventtime, duty, self.voltage(),
                                       src.current, powered)
        if reason is not None and self.fault is None:
            self.fault = reason
            src.latch_fault("%s: %s" % (self.heater_name, reason))
        elif not powered:
            self.fault = None
        if was_full and self.estimator.full_since is None:
            self._stage_resistance()
        return eventtime + POLL_INTERVAL
    def _stage_resistance(self):
        r = self.estimator.resistance
        if self.estimator.window_samples < SAVE_SAMPLES:
            return
        if abs(r - self.saved_resistance) < SAVE_CHANGE * self.saved_resistance:
            return
        logging.info("%s: resistance %.3f -> %.3f ohm", self.name,
                     self.saved_resistance, r)
        self.saved_resistance = r
        configfile = self.printer.lookup_object('configfile')
        configfile.set(self.name, 'resistance', "%.3f" % (r,))
    def get_status(self, eventtime):
        est = self.estimator
        volts = self.voltage()
        duty = self.heater.last_pwm_value if self.heater is not None else 0.
        return {'voltage': round(volts, 2),
                'resistance': round(est.resistance, 3),
                'measured_resistance': (None if est.measured is None
                                        else round(est.measured, 3)),
                'idle_current': round(est.idle_current, 3),
                'available_power': round(self.available_power(), 1),
                'power': round(duty * self.available_power(), 1),
                'fault': self.fault}

def load_config_prefix(config):
    return HeaterSupply(config)
