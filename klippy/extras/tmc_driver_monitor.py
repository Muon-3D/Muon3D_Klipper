# Driver temperature and supply voltage from TMC2240/TMC2241 registers
#
# Copyright (C) 2026  Muon3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# tmc.py reads ADC_TEMP only while a stepper is enabled, and never reads
# ADC_VSUPPLY. This module polls both for a set of drivers on its own timer,
# enabled or not, so a fan can be driven from the driver die temperature and
# the 48 V rail can be watched from every driver on it.
#
# The [tmc_driver_monitor] section must come before any section that uses
# sensor_type: tmc_driver, because it is what registers that sensor type.
#
# Example:
#   [tmc_driver_monitor]
#
#   [temperature_sensor driver_x]
#   sensor_type: tmc_driver
#   sensor_stepper: stepper_x
#
#   [temperature_fan Motor_Drivers]
#   sensor_type: tmc_driver
#   sensor_stepper: stepper_x, stepper_y, stepper_z   # reports the hottest
#
# Status object "tmc_driver_monitor":
#   {"stepper_x": {"temperature": 31.2, "supply_voltage": 45.09}, ...,
#    "max_temperature": 31.2, "min_supply_voltage": 45.06}
import logging
from . import tmc2240, tmc2241

POLL_INTERVAL = 1.
# ADC_TEMP -> degC, TMC2240/TMC2241 datasheet (same as tmc.py get_status)
TEMP_OFFSET = 2038.
TEMP_SCALE = 7.7
VSUPPLY_LSB = {tmc2241.TMC2241: tmc2241.VSUPPLY_LSB,
               tmc2240.TMC2240: 0.009732}

class DriverMonitor:
    def __init__(self, printer):
        self.printer = printer
        self.reactor = printer.get_reactor()
        self.drivers = {}
        self.readings = {}
        self.timer = None
        printer.register_event_handler("klippy:ready", self._handle_ready)
        printer.register_event_handler("klippy:shutdown", self._stop)
    def add_stepper(self, stepper_name):
        self.drivers.setdefault(stepper_name, None)
    def _find_driver(self, stepper_name):
        for name, obj in self.printer.lookup_objects():
            parts = name.split()
            if (len(parts) == 2 and parts[1] == stepper_name
                    and type(obj) in VSUPPLY_LSB):
                return obj
        raise self.printer.config_error(
            "tmc_driver: no TMC2240/TMC2241 section for '%s'"
            % (stepper_name,))
    def _handle_ready(self):
        for stepper_name in self.drivers:
            self.drivers[stepper_name] = self._find_driver(stepper_name)
        self.timer = self.reactor.register_timer(
            self._poll, self.reactor.monotonic() + POLL_INTERVAL)
    def _stop(self):
        if self.timer is not None:
            self.reactor.unregister_timer(self.timer)
            self.timer = None
    def _read(self, drv):
        fields = drv.fields
        temp_reg = fields.lookup_register("adc_temp")
        vs_reg = fields.lookup_register("adc_vsupply")
        val = drv.mcu_tmc.get_register(temp_reg)
        temp = (fields.get_field("adc_temp", val) - TEMP_OFFSET) / TEMP_SCALE
        val = drv.mcu_tmc.get_register(vs_reg)
        volts = fields.get_field("adc_vsupply", val) * VSUPPLY_LSB[type(drv)]
        return round(temp, 2), round(volts, 3)
    def _poll(self, eventtime):
        for stepper_name, drv in self.drivers.items():
            try:
                temp, volts = self._read(drv)
            except self.printer.command_error as e:
                logging.info("tmc_driver_monitor: %s read failed: %s",
                             stepper_name, e)
                self.readings[stepper_name] = None
                continue
            self.readings[stepper_name] = (temp, volts)
        return eventtime + POLL_INTERVAL
    def get_temperature(self, stepper_names):
        temps = [self.readings[n][0] for n in stepper_names
                 if self.readings.get(n) is not None]
        return max(temps) if temps else None
    def get_status(self, eventtime):
        status = {}
        temps, volts = [], []
        for name in self.drivers:
            r = self.readings.get(name)
            if r is None:
                status[name] = {"temperature": None, "supply_voltage": None}
                continue
            status[name] = {"temperature": r[0], "supply_voltage": r[1]}
            temps.append(r[0])
            volts.append(r[1])
        status["max_temperature"] = max(temps) if temps else None
        status["min_supply_voltage"] = min(volts) if volts else None
        return status

class TMCDriverTemperature:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        self.steppers = config.getlist("sensor_stepper")
        self.monitor = self.printer.lookup_object("tmc_driver_monitor")
        for s in self.steppers:
            self.monitor.add_stepper(s)
        self.temperature_callback = None
        self.min_temp = self.max_temp = None
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
    def setup_callback(self, temperature_callback):
        self.temperature_callback = temperature_callback
    def setup_minmax(self, min_temp, max_temp):
        self.min_temp, self.max_temp = min_temp, max_temp
    def get_report_time_delta(self):
        return POLL_INTERVAL
    def _handle_ready(self):
        # Offset half a period so the monitor has a fresh reading
        self.reactor.register_timer(
            self._report, self.reactor.monotonic() + POLL_INTERVAL * 1.5)
    def _report(self, eventtime):
        temp = self.monitor.get_temperature(self.steppers)
        if temp is None:
            return eventtime + POLL_INTERVAL
        if self.max_temp is not None and temp > self.max_temp:
            self.printer.invoke_shutdown(
                "TMC driver temperature %.1f above maximum %.1f on %s"
                % (temp, self.max_temp, ", ".join(self.steppers)))
            return self.reactor.NEVER
        if self.temperature_callback is not None:
            mcu = self.printer.lookup_object("mcu")
            read_time = mcu.estimated_print_time(eventtime)
            self.temperature_callback(read_time, temp)
        return eventtime + POLL_INTERVAL

def load_config(config):
    printer = config.get_printer()
    pheaters = printer.load_object(config, "heaters")
    pheaters.add_sensor_factory("tmc_driver", TMCDriverTemperature)
    return DriverMonitor(printer)
