# Muon M1 B1 Z column and bed power path (TPS16630 eFuse)
#
# Copyright (C) 2026  Muon3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# 48V_Z feeds the folding Z column and, through it, the removable bed. It is
# switched by an eFuse whose enable is held off by a pull-down. This module
# turns it on only when the Z column is present with its lever locked AND a
# plate reads on the bed thermistor, and drops it the moment any of those
# stops being true. With the plate off, the bed-side contacts are not left
# live at 48 V; with the column out, the Z connector is not either.
#
# Inputs: the Z_PRESENT lever divider (ADC, three bands), plate presence
# from [bed_removal_detector] (the single owner of that judgement), the
# eFuse PGOOD (digital) and IMON (ADC, current). Output: the eFuse enable.
# The bed heater refuses a target unless the path is READY, and a PGOOD drop
# or a current fault latches the path off until Z_POWER_RESET.
import logging

POLL_INTERVAL = 0.1
ADC_SAMPLE_TIME = 0.001
ADC_SAMPLE_COUNT = 8
ADC_REPORT_TIME = 0.100
PIN_MIN_TIME = 0.100

# States, in the order the path comes up
ABSENT = 'absent'           # no Z column on the connector
UNLOCKED = 'unlocked'       # column present, lever not locked
UNKNOWN = 'unknown'         # Z_PRESENT outside every configured band
NO_PLATE = 'no_plate'       # column locked, no plate on the bed thermistor
POWERING = 'powering'       # enable on, waiting for PGOOD
READY = 'ready'             # 48V_Z up and confirmed
INHIBITED = 'inhibited'     # held off by Z_POWER_INHIBIT
FAULT = 'fault'             # latched until Z_POWER_RESET


# What is wrong and what to do about it, as shown to the user
REASONS = {
    ABSENT: "the Z column is not connected: fold the Z axis down into"
            " the base",
    UNLOCKED: "the Z column is not locked: push the locking lever down",
    UNKNOWN: "the Z column sensor reading is not recognised (see"
             " Z_POWER_STATUS)",
    NO_PLATE: "no bed plate detected: attach the bed plate",
    POWERING: "bed power is starting up",
    INHIBITED: "bed power is held off (Z_POWER_INHIBIT ENABLE=0 releases"
               " it)",
    FAULT: "bed power fault: %s. Check the bed and its cable, then run"
           " Z_POWER_RESET",
}


def decide(z_band, plate_present, fault, inhibited):
    """Pure decision: (state before PGOOD confirmation, reason, enable).

    z_band is 'absent', 'unlocked', 'locked' or None (no matching band).
    plate_present is True, False or None (no reading yet). Returns the
    state the path should be in if PGOOD behaves; the caller turns
    'powering' into 'ready' once PGOOD confirms.
    """
    if fault:
        return FAULT, REASONS[FAULT] % (fault,), False
    if inhibited:
        return INHIBITED, REASONS[INHIBITED], False
    if z_band == 'absent':
        return ABSENT, REASONS[ABSENT], False
    if z_band == 'unlocked':
        return UNLOCKED, REASONS[UNLOCKED], False
    if z_band != 'locked':
        return UNKNOWN, REASONS[UNKNOWN], False
    if not plate_present:
        return NO_PLATE, REASONS[NO_PLATE], False
    return POWERING, REASONS[POWERING], True


def classify_band(ratio, bands):
    """Map a Z_PRESENT ADC ratio to a band name, or None."""
    for name, (lo, hi) in bands:
        if lo <= ratio <= hi:
            return name
    return None


class Debounce:
    """A value that only changes after `count` consecutive equal inputs."""
    def __init__(self, count):
        self.count = count
        self.value = None
        self.candidate = None
        self.seen = 0
    def update(self, new):
        if new == self.value:
            self.candidate, self.seen = None, 0
            return self.value
        if new != self.candidate:
            self.candidate, self.seen = new, 0
        self.seen += 1
        if self.seen >= self.count:
            self.value, self.candidate, self.seen = new, None, 0
        return self.value


class ZPower:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.reactor = printer.get_reactor()
        self.name = config.get_name()
        ppins = printer.lookup_object('pins')
        # eFuse enable: off at start and on any MCU shutdown
        self.mcu_enable = ppins.setup_pin('digital_out',
                                          config.get('enable_pin'))
        self.mcu_enable.setup_max_duration(0.)
        self.mcu_enable.setup_start_value(0., 0.)
        self.enable_mcu = self.mcu_enable.get_mcu()
        self.enabled = False
        self.last_enable_time = 0.
        # PGOOD (open drain, high = 48V_Z good)
        buttons = printer.load_object(config, 'buttons')
        buttons.register_buttons([config.get('power_good_pin')],
                                 self._power_good_callback)
        self.power_good = False
        # Z_PRESENT lever divider
        self.present_ratio = None
        self.mcu_present = ppins.setup_pin('adc', config.get('present_pin'))
        self.mcu_present.setup_adc_sample(ADC_SAMPLE_TIME, ADC_SAMPLE_COUNT)
        self.mcu_present.setup_adc_callback(ADC_REPORT_TIME,
                                            self._present_callback)
        self.bands = []
        for band in ('absent', 'unlocked', 'locked'):
            lo, hi = self._get_range(config, 'present_%s_range' % (band,))
            self.bands.append((band, (lo, hi)))
        self.z_band = Debounce(config.getint('present_samples', 3, minval=1))
        # IMON current (optional). IMON reads a small non-zero value with
        # the feed off (~0.11 A equivalent on gimlet), so the zero is learnt
        # while the enable is off and subtracted while it is on.
        self.current = None
        self.raw_current = None
        self.current_offset = None
        self.current_scale = (config.getfloat('adc_voltage', 3.3, above=0.)
                              * config.getfloat('current_per_volt', 2.24,
                                                above=0.))
        current_pin = config.get('current_pin', None)
        if current_pin is not None:
            mcu_current = ppins.setup_pin('adc', current_pin)
            mcu_current.setup_adc_sample(ADC_SAMPLE_TIME, ADC_SAMPLE_COUNT)
            mcu_current.setup_adc_callback(ADC_REPORT_TIME,
                                           self._current_callback)
        self.max_current = config.getfloat('max_current', 5.0, above=0.)
        self.max_current_time = config.getfloat('max_current_time', 0.5,
                                                minval=0.)
        self.idle_current = config.getfloat('idle_current', 1.0, above=0.)
        self.idle_current_time = config.getfloat('idle_current_time', 2.0,
                                                 minval=0.)
        self.supply_voltage = config.getfloat('supply_voltage', 45., above=0.)
        self.over_since = self.idle_over_since = None
        # Plate presence comes from [bed_removal_detector], which owns that
        # judgement (and BED_REMOVED / BED_ATTACHED and the resume target).
        self.heater_name = config.get('heater', 'heater_bed')
        self.heater = None
        self.detector = None
        # Power-up confirmation
        self.power_good_timeout = config.getfloat('power_good_timeout', 0.5,
                                                  above=0.)
        self.powering_since = None
        # State
        self.state = UNKNOWN
        self.reason = "starting"
        self.fault = None
        self.inhibited = False
        gcode_macro = printer.load_object(config, 'gcode_macro')
        self.lost_template = gcode_macro.load_template(config, 'lost_gcode',
                                                       '')
        self.fault_template = gcode_macro.load_template(config, 'fault_gcode',
                                                        '')
        # Run on every change of the Z column state after the first reading
        # (the new state is the status object's `z`). The column, not the
        # power path, decides what a print does: unlocked pauses, removed
        # cancels.
        self.z_change_template = gcode_macro.load_template(
            config, 'z_change_gcode', '')
        self.last_band = None
        self.band_seen = False
        printer.register_event_handler('klippy:connect', self._handle_connect)
        printer.register_event_handler('klippy:ready', self._handle_ready)
        printer.register_event_handler('klippy:shutdown',
                                       self._handle_shutdown)
        gcode = printer.lookup_object('gcode')
        gcode.register_command('Z_POWER_STATUS', self.cmd_Z_POWER_STATUS,
                               desc=self.cmd_Z_POWER_STATUS_help)
        gcode.register_command('Z_POWER_RESET', self.cmd_Z_POWER_RESET,
                               desc=self.cmd_Z_POWER_RESET_help)
        gcode.register_command('Z_POWER_INHIBIT', self.cmd_Z_POWER_INHIBIT,
                               desc=self.cmd_Z_POWER_INHIBIT_help)
        gcode.register_command('Z_POWER_WAIT', self.cmd_Z_POWER_WAIT,
                               desc=self.cmd_Z_POWER_WAIT_help)
        gcode.register_command('Z_POWER_REQUIRE', self.cmd_Z_POWER_REQUIRE,
                               desc=self.cmd_Z_POWER_REQUIRE_help)
    def _get_range(self, config, option):
        vals = config.getfloatlist(option, count=2)
        lo, hi = vals
        if not (0. <= lo <= hi <= 1.):
            raise config.error("%s: %s must be 'low, high' within 0..1"
                               % (self.name, option))
        return lo, hi
    # Startup
    def _handle_connect(self):
        pheaters = self.printer.lookup_object('heaters')
        self.heater = pheaters.lookup_heater(self.heater_name)
        self.detector = self.printer.lookup_object('bed_removal_detector',
                                                   None)
        if self.detector is None:
            raise self.printer.config_error(
                "%s needs [bed_removal_detector] for plate presence"
                % (self.name,))
        # Interlock: refuse a heating target unless the path is READY.
        set_temp = self.heater.set_temp
        def guarded_set_temp(degrees):
            if degrees and self.state != READY:
                raise self.printer.command_error(
                    "Cannot heat the bed: %s" % (self.reason,))
            set_temp(degrees)
        self._raw_set_temp = set_temp
        self.heater.set_temp = guarded_set_temp
    def _handle_ready(self):
        self.reactor.register_timer(self._poll, self.reactor.NOW)
    def _handle_shutdown(self):
        # The MCU drops the enable itself (shutdown value 0).
        self.enabled = False
        self.power_good = False
    # MCU callbacks
    def _power_good_callback(self, eventtime, state):
        self.power_good = bool(state)
    def _present_callback(self, read_time, read_value):
        self.present_ratio = read_value
    def _current_callback(self, read_time, read_value):
        self.raw_current = read_value * self.current_scale
    def _update_current(self):
        raw = self.raw_current
        if raw is None:
            return
        if not self.enabled:
            if self.current_offset is None:
                self.current_offset = raw
            else:
                self.current_offset += (raw - self.current_offset) * 0.1
        self.current = max(0., raw - (self.current_offset or 0.))
    # Main loop
    def _plate_present(self):
        return self.detector.plate_present()
    def _set_enable(self, value, eventtime):
        if value == self.enabled:
            return
        print_time = max(self.enable_mcu.estimated_print_time(eventtime)
                         + PIN_MIN_TIME, self.last_enable_time + 0.010)
        self.mcu_enable.set_digital(print_time, 1 if value else 0)
        self.last_enable_time = print_time
        self.enabled = value
        logging.info("%s: 48V_Z %s", self.name, "on" if value else "off")
    def _latch_fault(self, reason, eventtime):
        if self.fault:
            return
        self.fault = reason
        logging.error("%s: fault: %s", self.name, reason)
        self._run_template(self.fault_template)
    def _check_current(self, eventtime):
        if self.current is None:
            return
        if self.current > self.max_current:
            if self.over_since is None:
                self.over_since = eventtime
            elif eventtime - self.over_since >= self.max_current_time:
                self._latch_fault("overcurrent: %.2f A over %.2f A"
                                  % (self.current, self.max_current),
                                  eventtime)
        else:
            self.over_since = None
        heater_power = self.heater.get_status(eventtime).get('power', 0.)
        if (self.state == READY and heater_power <= 0.
                and self.current > self.idle_current):
            if self.idle_over_since is None:
                self.idle_over_since = eventtime
            elif eventtime - self.idle_over_since >= self.idle_current_time:
                self._latch_fault("%.2f A drawn with the bed heater off"
                                  % (self.current,), eventtime)
        else:
            self.idle_over_since = None
    def _poll(self, eventtime):
        if self.printer.is_shutdown():
            return self.reactor.NEVER
        band = None
        if self.present_ratio is not None:
            band = self.z_band.update(classify_band(self.present_ratio,
                                                    self.bands))
            self._check_z_change(band)
        plate = self._plate_present()
        self._update_current()
        if self.enabled:
            self._check_current(eventtime)
        state, reason, enable = decide(band, plate, self.fault,
                                       self.inhibited)
        if enable:
            if not self.enabled:
                self.powering_since = eventtime
            elif self.power_good:
                state, reason = READY, "bed power is on"
                self.powering_since = None
            elif self.powering_since is None:
                # PGOOD dropped after the path was up: the eFuse tripped.
                self._latch_fault("PGOOD dropped (eFuse tripped)", eventtime)
                state, reason, enable = FAULT, self.fault, False
            elif eventtime - self.powering_since > self.power_good_timeout:
                self._latch_fault("PGOOD did not rise within %.2fs"
                                  % (self.power_good_timeout,), eventtime)
                state, reason, enable = FAULT, self.fault, False
        self._set_enable(enable, eventtime)
        self._transition(state, reason, eventtime)
        return eventtime + POLL_INTERVAL
    def _check_z_change(self, band):
        if band == self.last_band or (band is None and not self.band_seen):
            return
        previous = self.last_band if self.band_seen else None
        self.last_band = band
        if not self.band_seen:
            # The first reading after startup is not a change.
            self.band_seen = True
            return
        logging.info("%s: Z column %s -> %s", self.name,
                     previous or 'unknown', band or 'unknown')
        self._run_template(self.z_change_template)
    def _transition(self, state, reason, eventtime):
        prev = self.state
        self.state, self.reason = state, reason
        if prev == state:
            return
        logging.info("%s: %s -> %s (%s)", self.name, prev, state, reason)
        if prev != READY:
            return
        # The path just went down: stop heating. What a print does about it
        # belongs to whatever went away -- the plate (bed_removal_detector),
        # the Z column (z_change_gcode) or a fault (fault_gcode) -- so
        # lost_gcode is only for the path being held off by hand.
        _temp, target = self.heater.get_temp(eventtime)
        if target:
            self._raw_set_temp(0.)
        if state == INHIBITED:
            self._run_template(self.lost_template)
    def _run_template(self, template):
        script = template.render()
        if not script.strip():
            return
        self.reactor.register_callback(
            lambda et: self._run_script(script))
    def _run_script(self, script):
        try:
            self.printer.lookup_object('gcode').run_script(script)
        except Exception:
            logging.exception("%s: gcode failed", self.name)
    # Reporting
    def _supply_voltage(self, eventtime):
        monitor = self.printer.lookup_object('tmc_driver_monitor', None)
        if monitor is not None:
            volts = monitor.get_status(eventtime).get('min_supply_voltage')
            if volts:
                return volts
        return self.supply_voltage
    def get_status(self, eventtime):
        power = None
        if self.current is not None:
            power = round(self.current * self._supply_voltage(eventtime), 1)
        return {'state': self.state,
                'reason': self.reason,
                'z': self.z_band.value,
                'plate_present': self.detector.plate_present(),
                'enabled': self.enabled,
                'power_good': self.power_good,
                'present_ratio': self.present_ratio,
                'current': (None if self.current is None
                            else round(self.current, 3)),
                'power': power,
                'fault': self.fault}
    # Commands
    cmd_Z_POWER_STATUS_help = "Report the Z column and bed power path"
    def cmd_Z_POWER_STATUS(self, gcmd):
        st = self.get_status(self.reactor.monotonic())
        ratio = st['present_ratio']
        gcmd.respond_info(
            "Z power: %s%s\n"
            "Z_PRESENT ratio %s -> %s | plate %s | enable %s | PGOOD %s\n"
            "current %s A | power %s W"
            % (st['state'], " (%s)" % st['reason'] if st['reason'] else "",
               "-" if ratio is None else "%.3f" % (ratio,), st['z'],
               st['plate_present'], int(st['enabled']),
               int(st['power_good']), st['current'], st['power']))
    cmd_Z_POWER_RESET_help = "Clear a latched Z power fault"
    def cmd_Z_POWER_RESET(self, gcmd):
        if self.fault:
            gcmd.respond_info("Cleared Z power fault: %s" % (self.fault,))
        self.fault = None
        self.over_since = self.idle_over_since = None
    cmd_Z_POWER_INHIBIT_help = "Hold 48V_Z off (ENABLE=1) or release it"
    def cmd_Z_POWER_INHIBIT(self, gcmd):
        self.inhibited = bool(gcmd.get_int('ENABLE', 1, minval=0, maxval=1))
        gcmd.respond_info("Z power %s" % ("held off" if self.inhibited
                                          else "released"))

    cmd_Z_POWER_WAIT_help = ("Wait up to TIMEOUT seconds for bed power to come"
                             " up (never an error)")
    def cmd_Z_POWER_WAIT(self, gcmd):
        timeout = gcmd.get_float('TIMEOUT', 2., minval=0., maxval=30.)
        end = self.reactor.monotonic() + timeout
        while self.state != READY and self.reactor.monotonic() < end:
            self.reactor.pause(self.reactor.monotonic() + POLL_INTERVAL)
    cmd_Z_POWER_REQUIRE_help = "Raise an error unless bed power is on"
    def cmd_Z_POWER_REQUIRE(self, gcmd):
        if self.state != READY:
            raise gcmd.error("%s: %s" % (gcmd.get('MSG', 'Bed power is off'),
                                         self.reason))

def load_config(config):
    return ZPower(config)
