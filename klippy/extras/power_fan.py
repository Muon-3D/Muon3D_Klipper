# Support fans controlled from estimated printer power draw
#
# Copyright (C) 2026  Muon3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections

from . import fan

PIN_MIN_TIME = 0.100


class PowerFan:
    cmd_SET_POWER_FAN_CONFIG_help = "Set power fan runtime configuration"
    cmd_SET_POWER_FAN_RESISTIVE_LOAD_help = "Set a power fan heater load"
    cmd_SET_POWER_FAN_FAN_LOAD_help = "Set a power fan fan load"
    cmd_SET_POWER_FAN_DYNAMIC_LOAD_help = "Set a power fan dynamic load"
    cmd_CLEAR_POWER_FAN_DYNAMIC_LOAD_help = "Clear a power fan dynamic load"
    cmd_SET_POWER_FAN_OVERRIDE_help = "Set a power fan speed override"

    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.printer.load_object(config, 'heaters')
        self.stepper_enable = self.printer.load_object(config, 'stepper_enable')
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        self.printer.register_event_handler("klippy:ready",
                                            self.handle_ready)

        self.fan = fan.Fan(config)
        self.system_voltage = config.getfloat('system_voltage', above=0.)
        self.sample_interval = config.getfloat('sample_interval', 0.5,
                                               above=0.)
        self.filter_time = config.getfloat('filter_time', 5.0, minval=0.)
        self.min_speed_delta = config.getfloat('min_speed_delta', 0.02,
                                               minval=0., maxval=1.)

        self.resistive_loads = self._parse_named_float_table(
            config, 'resistive_loads', above=0., default=())
        self.stepper_loads = self._parse_named_float_table(
            config, 'stepper_loads', above=0., default=())
        self.fan_loads = self._parse_named_float_table(
            config, 'fan_loads', minval=0., default=())
        self.fixed_loads = dict(self._parse_named_float_table(
            config, 'fixed_loads', minval=0., default=()))
        self.dynamic_loads = {}

        self.points = self._parse_points(config)
        self.heaters = {}
        self.fans = {}
        self.stepper_names = []
        self.samples = collections.deque()
        self.last_speed = 0.
        self.override_speed = None
        self.instant_power = 0.
        self.filtered_power = 0.
        self.resistive_power = 0.
        self.stepper_power = 0.
        self.fan_power = 0.
        self.fixed_power = sum(self.fixed_loads.values())
        self.dynamic_power = 0.

        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("SET_POWER_FAN_CONFIG", "FAN", self.name,
                                   self.cmd_SET_POWER_FAN_CONFIG,
                                   desc=self.cmd_SET_POWER_FAN_CONFIG_help)
        gcode.register_mux_command("SET_POWER_FAN_RESISTIVE_LOAD", "FAN",
                                   self.name,
                                   self.cmd_SET_POWER_FAN_RESISTIVE_LOAD,
                                   desc="Set a power fan heater load")
        gcode.register_mux_command("SET_POWER_FAN_FAN_LOAD", "FAN",
                                   self.name,
                                   self.cmd_SET_POWER_FAN_FAN_LOAD,
                                   desc="Set a power fan fan load")
        gcode.register_mux_command("SET_POWER_FAN_DYNAMIC_LOAD", "FAN",
                                   self.name,
                                   self.cmd_SET_POWER_FAN_DYNAMIC_LOAD,
                                   desc="Set a power fan dynamic load")
        gcode.register_mux_command("CLEAR_POWER_FAN_DYNAMIC_LOAD", "FAN",
                                   self.name,
                                   self.cmd_CLEAR_POWER_FAN_DYNAMIC_LOAD,
                                   desc="Clear a power fan dynamic load")
        gcode.register_mux_command("SET_POWER_FAN_OVERRIDE", "FAN",
                                   self.name,
                                   self.cmd_SET_POWER_FAN_OVERRIDE,
                                   desc=self.cmd_SET_POWER_FAN_OVERRIDE_help)

    def _parse_named_float_table(self, config, option, default=None,
                                 minval=None, above=None):
        raw = config.getlists(option, default, seps=(',', '\n'),
                              parser=str, count=2)
        res = []
        for name, value in raw:
            try:
                value = float(value)
            except ValueError:
                raise config.error("Unable to parse option '%s' in section"
                                   " '%s'" % (option, config.get_name()))
            if minval is not None and value < minval:
                raise config.error("Option '%s' in section '%s' must have"
                                   " minimum of %s" % (
                                       option, config.get_name(), minval))
            if above is not None and value <= above:
                raise config.error("Option '%s' in section '%s' must be"
                                   " above %s" % (
                                       option, config.get_name(), above))
            res.append((name, value))
        return res

    def _parse_points(self, config):
        points = list(config.getlists('points', seps=(',', '\n'),
                                      parser=float, count=2))
        if len(points) < 2:
            raise config.error("Option 'points' in section '%s' must contain"
                               " at least two points" % (config.get_name(),))
        points.sort(key=lambda x: x[0])
        last_power = None
        last_speed = None
        for power, speed in points:
            if power < 0.:
                raise config.error("Power fan points must have non-negative"
                                   " power values")
            if speed < 0. or speed > 1.:
                raise config.error("Power fan points must have speed values"
                                   " between 0.0 and 1.0")
            if last_power is not None:
                if power == last_power:
                    raise config.error("Power fan points may not contain"
                                       " duplicate power values")
                if speed < last_speed:
                    raise config.error("Power fan points must be"
                                       " monotonically increasing")
            last_power = power
            last_speed = speed
        return points

    def handle_connect(self):
        pheaters = self.printer.lookup_object('heaters')
        self.heaters = {}
        for heater_name, resistance in self.resistive_loads:
            self.heaters[heater_name] = {
                'heater': pheaters.lookup_heater(heater_name),
                'resistance': resistance,
            }
        self.fans = {}
        for fan_name, power in self.fan_loads:
            fan_obj = self.printer.lookup_object(fan_name, None)
            if fan_obj is None or not hasattr(fan_obj, 'get_status'):
                raise self.printer.config_error(
                    "Unknown fan '%s' in [power_fan %s]"
                    % (fan_name, self.name))
            self.fans[fan_name] = {
                'fan': fan_obj,
                'power': power,
            }
        all_steppers = self.stepper_enable.get_steppers()
        self.stepper_names = []
        for stepper_name, power in self.stepper_loads:
            if stepper_name not in all_steppers:
                raise self.printer.config_error(
                    "Unknown stepper '%s' in [power_fan %s]"
                    % (stepper_name, self.name))
            self.stepper_names.append(stepper_name)

    def handle_ready(self):
        reactor = self.printer.get_reactor()
        reactor.register_timer(self.callback, reactor.monotonic()+PIN_MIN_TIME)

    def _interp_speed(self, power):
        points = self.points
        if power <= points[0][0]:
            return points[0][1]
        if power >= points[-1][0]:
            return points[-1][1]
        for i in range(len(points) - 1):
            power1, speed1 = points[i]
            power2, speed2 = points[i + 1]
            if power <= power2:
                ratio = (power - power1) / (power2 - power1)
                return speed1 + ratio * (speed2 - speed1)
        return points[-1][1]

    def _calc_resistive_power(self, eventtime):
        voltage_sq = self.system_voltage * self.system_voltage
        total = 0.
        for load in self.heaters.values():
            heater = load['heater']
            status = heater.get_status(eventtime)
            total += status.get('power', 0.) * voltage_sq / load['resistance']
        return total

    def _calc_stepper_power(self):
        total = 0.
        powers = dict(self.stepper_loads)
        for stepper_name in self.stepper_names:
            enable = self.stepper_enable.lookup_enable(stepper_name)
            if enable.is_motor_enabled():
                total += powers[stepper_name]
        return total

    def _calc_fan_power(self, eventtime):
        total = 0.
        for load in self.fans.values():
            status = load['fan'].get_status(eventtime)
            total += status.get('speed', 0.) * load['power']
        return total

    def _record_sample(self, eventtime, power):
        self.samples.append((eventtime, power))
        if self.filter_time <= 0.:
            self.samples.clear()
            self.samples.append((eventtime, power))
            return power
        start_time = eventtime - self.filter_time
        while len(self.samples) > 1 and self.samples[1][0] < start_time:
            self.samples.popleft()
        return self._calc_filtered_power(eventtime, start_time)

    def _calc_filtered_power(self, eventtime, start_time):
        if len(self.samples) == 1:
            return self.samples[0][1]
        last_time = start_time
        last_power = self.samples[0][1]
        total = 0.
        for sample_time, sample_power in self.samples:
            if sample_time <= start_time:
                last_power = sample_power
                continue
            total += last_power * (sample_time - last_time)
            last_time = sample_time
            last_power = sample_power
        total += last_power * max(0., eventtime - last_time)
        return total / self.filter_time

    def callback(self, eventtime):
        self.resistive_power = self._calc_resistive_power(eventtime)
        self.stepper_power = self._calc_stepper_power()
        self.fan_power = self._calc_fan_power(eventtime)
        self.fixed_power = sum(self.fixed_loads.values())
        self.dynamic_power = sum(self.dynamic_loads.values())
        self.instant_power = (self.resistive_power + self.stepper_power
                              + self.fan_power + self.fixed_power
                              + self.dynamic_power)
        self.filtered_power = self._record_sample(eventtime,
                                                  self.instant_power)
        speed = self.override_speed
        if speed is None:
            speed = self._interp_speed(self.filtered_power)
        if (abs(speed - self.last_speed) >= self.min_speed_delta
            or (not speed and self.last_speed)
            or (speed and not self.last_speed)):
            self.last_speed = speed
            self.fan.set_speed(speed)
        return eventtime + self.sample_interval

    def get_status(self, eventtime):
        status = self.fan.get_status(eventtime)
        status.update({
            'instant_power': round(self.instant_power, 3),
            'filtered_power': round(self.filtered_power, 3),
            'resistive_power': round(self.resistive_power, 3),
            'stepper_power': round(self.stepper_power, 3),
            'fan_power': round(self.fan_power, 3),
            'fixed_power': round(self.fixed_power, 3),
            'dynamic_power': round(self.dynamic_power, 3),
            'system_voltage': float(self.system_voltage),
            'filter_time': float(self.filter_time),
            'override_speed': self.override_speed,
        })
        return status

    def cmd_SET_POWER_FAN_CONFIG(self, gcmd):
        voltage = gcmd.get_float('VOLTAGE', None, above=0.)
        filter_time = gcmd.get_float('FILTER_TIME', None, minval=0.)
        sample_interval = gcmd.get_float('SAMPLE_INTERVAL', None, above=0.)
        min_speed_delta = gcmd.get_float('MIN_SPEED_DELTA', None,
                                         minval=0., maxval=1.)
        if voltage is None and filter_time is None and sample_interval is None:
            if min_speed_delta is None:
                gcmd.respond_info(
                    "power_fan %s: voltage=%.3f filter_time=%.3f"
                    " sample_interval=%.3f min_speed_delta=%.3f"
                    % (self.name, self.system_voltage, self.filter_time,
                       self.sample_interval, self.min_speed_delta))
                return
        if voltage is not None:
            self.system_voltage = voltage
        if filter_time is not None:
            self.filter_time = filter_time
            self.samples.clear()
        if sample_interval is not None:
            self.sample_interval = sample_interval
        if min_speed_delta is not None:
            self.min_speed_delta = min_speed_delta

    def cmd_SET_POWER_FAN_RESISTIVE_LOAD(self, gcmd):
        heater_name = gcmd.get('HEATER')
        resistance = gcmd.get_float('RESISTANCE', above=0.)
        if heater_name not in self.heaters:
            raise gcmd.error("Unknown resistive load '%s'" % (heater_name,))
        self.heaters[heater_name]['resistance'] = resistance

    def cmd_SET_POWER_FAN_FAN_LOAD(self, gcmd):
        fan_name = gcmd.get('LOAD_FAN')
        power = gcmd.get_float('POWER', minval=0.)
        if fan_name not in self.fans:
            raise gcmd.error("Unknown fan load '%s'" % (fan_name,))
        self.fans[fan_name]['power'] = power

    def cmd_SET_POWER_FAN_DYNAMIC_LOAD(self, gcmd):
        name = gcmd.get('NAME')
        power = gcmd.get_float('POWER', minval=0.)
        self.dynamic_loads[name] = power

    def cmd_CLEAR_POWER_FAN_DYNAMIC_LOAD(self, gcmd):
        name = gcmd.get('NAME')
        if name in self.dynamic_loads:
            del self.dynamic_loads[name]

    def cmd_SET_POWER_FAN_OVERRIDE(self, gcmd):
        clear = gcmd.get_int('CLEAR', 0, minval=0, maxval=1)
        speed = gcmd.get_float('SPEED', None, minval=0., maxval=1.)
        if clear:
            if speed is not None:
                raise gcmd.error(
                    "SET_POWER_FAN_OVERRIDE cannot specify CLEAR and SPEED")
            self.override_speed = None
            return
        if speed is None:
            gcmd.respond_info(
                "power_fan %s: override_speed=%s"
                % (self.name, self.override_speed))
            return
        self.override_speed = speed
        self.last_speed = speed
        self.fan.set_speed(speed)


def load_config_prefix(config):
    return PowerFan(config)
