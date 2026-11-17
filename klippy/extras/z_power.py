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
#
# It also gates X/Y motion on the column: folded, the Z axis sits in the
# path of the X/Y gantry, so a G-code move in X or Y is refused unless the
# column is plugged in (locked or not), or its reading is in no band after a
# fitted one with no absent reading since (the lever passing through). The
# status key xy_allowed follows the same rule as the gate, so the panel and
# the gate agree; xy_home_allowed stays strict and is what X/Y homing reads
# (a home must not start on a lever part way). Z needs
# no lock to move (the lift before an X/Y home, the park at the top); only Z
# homing, which macros gate on the lock, and anything that trusts a Z
# position need it locked.
#
# The eFuse is a TPS16630 (TI SLVSET9G). The enable output drives its SHDN#
# pin: high turns the FET on with the dVdT ramp, low turns it off and resets
# a latched fault. PGOOD is open drain and goes high once the FET's gate is
# fully enhanced, after an 11.5 ms deglitch (13 ms max). It goes low (10 ms
# deglitch) when the FET turns off: overload past the current limit, thermal
# shutdown, UVLO, OVP or SHDN#. FLT# is not wired to the MCU, so a PGOOD that
# never rises or drops is all this module can see; the IMON current at that
# moment is logged with the fault to tell a short (current at the limit) from
# an input-window or enable fault (no current).
import collections
import logging

POLL_INTERVAL = 0.1
ADC_SAMPLE_TIME = 0.001
ADC_SAMPLE_COUNT = 8
ADC_REPORT_TIME = 0.100
PIN_MIN_TIME = 0.100

# PGOOD after SHDN# goes high, from the TPS16630 datasheet: the output ramp
# (20.8e3 * Vin * C_dVdT seconds), then the 13 ms PGOOD deglitch. Into a
# large capacitance the eFuse holds the start-up in thermal regulation for up
# to t(Treg_timeout), 1.5 s max, before it gives up itself. The timeout must
# not declare a start-up failed while the eFuse is still allowed to finish it.
POWER_GOOD_TIMEOUT = 2.0
# The bed thermistor is wired through the Z column. With the column out the
# input floats (45 to 58 C on sloe, 2026-11-06), which reads as a plate. A
# plate reading counts only once the column has been connected this long,
# so the detector has fresh samples from the column (the thermistor reports
# every 0.3 s and the detector averages two).
PLATE_SETTLE_TIME = 2.0
# KAN-3334: a PGOOD low seen while READY is a trip only once it has lasted
# this many consecutive polls (POLL_INTERVAL apart). One low poll followed
# by a high one is a glitch: counted and logged, never latched. 1 is the
# old behaviour (the first low poll latches).
DROP_SAMPLES = 3
# The fault and glitch log lines carry the peak IMON current over this
# long and the last PGOOD_HISTORY PGOOD polls. The current at the moment
# of a trip is often 0.00 A (the FET is already off), so the peak is what
# says whether the eFuse was loaded.
PEAK_WINDOW = 2.0
PGOOD_HISTORY = 10
# At most one glitch line in the log per this long; the rest are counted.
GLITCH_LOG_INTERVAL = 10.0

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
# One plain sentence for the panel (m1_machine's bed_power_reason), used
# in place of REASONS[FAULT] for the faults that have one. The engineering
# detail (peak current, PGOOD history) is in fault_detail and the log.
DROPPED_REASON = ("Bed power cut out. Check the plate is seated and its"
                  " connector is clean, then reset bed power.")
NO_RETURN_REASON = ("Bed power did not come back after the reset. Check the"
                    " plate and its connector.")
PRINTING_RESET_ERROR = "Pause or stop the print before resetting bed power."


def decide(z_band, plate_present, fault, inhibited,
           fault_reason=None):
    """Pure decision: (state before PGOOD confirmation, reason, enable).

    z_band is 'absent', 'unlocked', 'locked' or None (no matching band).
    plate_present is True, False or None (no reading yet). fault_reason
    is the plain sentence for the fault, if it has one. Returns the
    state the path should be in if PGOOD behaves; the caller turns
    'powering' into 'ready' once PGOOD confirms.
    """
    if fault:
        return FAULT, fault_reason or REASONS[FAULT] % (fault,), False
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


def xy_allowed(z_band, required=True, was_fitted=False):
    """Whether X/Y may move: the Z column plugged in, locked or not. An
    unrecognised or not yet read column refuses, so the gate fails closed,
    except for a reading in no band (z_band None) that follows a fitted
    one with no absent reading since: the lever passing through, which must
    not strand a PAUSE's park or a cancel's cut half done. An absent
    column, or one never seen fitted since start-up, still refuses.
    """
    if not required or z_band in (UNLOCKED, 'locked'):
        return True
    return z_band is None and was_fitted


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
        # PGOOD drop debounce while READY (see DROP_SAMPLES)
        self._init_pgood_state(config.getint('power_good_drop_samples',
                                             DROP_SAMPLES, minval=1))
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
        # Highest current seen while waiting for PGOOD, for the fault text
        self.start_peak = 0.
        # Plate presence comes from [bed_removal_detector], which owns that
        # judgement (and BED_REMOVED / BED_ATTACHED and the resume target).
        self.heater_name = config.get('heater', 'heater_bed')
        self.heater = None
        self.detector = None
        self.plate_settle_time = config.getfloat(
            'plate_settle_time', PLATE_SETTLE_TIME, minval=0.)
        self.column_since = None
        self.plate = None
        # Power-up confirmation, timed from when the enable pin changes
        self.power_good_timeout = config.getfloat(
            'power_good_timeout', POWER_GOOD_TIMEOUT, above=0.)
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
        # The last KNOWN band was a fitted one (see xy_allowed)
        self.column_was_fitted = False
        # Production: no X/Y move without the column (see the top). A
        # development unit run without a column sets this False.
        self.xy_requires_column = config.getboolean('xy_requires_column',
                                                    True)
        self.next_transform = None
        self.last_position = [0., 0., 0., 0.]
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
    def _init_pgood_state(self, drop_samples):
        self.drop_samples = drop_samples
        self.low_polls = 0
        self.low_run = False
        self.glitches = 0
        self.last_glitch = None
        self.glitch_log_time = None
        self.glitch_unlogged = 0
        # When the start-up blanking ends (set when PGOOD first comes up
        # after an enable), and whether the enable follows a reset
        self.settle_end = None
        self.after_reset = False
        self.pgood_history = collections.deque(maxlen=PGOOD_HISTORY)
        self.current_history = collections.deque()
        self.fault_reason = None
        self.fault_detail = None
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
        # A gcode_move transform, as contact_guard is, so jogs, typed G1 and
        # macros are all checked; homing and FORCE_MOVE drive the toolhead
        # directly and are gated by the homing macros instead.
        gcode_move = self.printer.lookup_object('gcode_move')
        self.next_transform = gcode_move.set_move_transform(self, force=True)
        gcode_move.reset_last_position()
    # Move transform interface: refuse X/Y without the Z column
    def get_position(self):
        self.last_position = list(self.next_transform.get_position())
        return list(self.last_position)
    def move(self, newpos, speed):
        last = self.last_position
        moves_xy = (abs(newpos[0] - last[0]) > 1e-6
                    or abs(newpos[1] - last[1]) > 1e-6)
        if moves_xy and not xy_allowed(self.z_band.value,
                                       self.xy_requires_column,
                                       self.column_was_fitted):
            band = self.z_band.value
            reason = REASONS[ABSENT if band == ABSENT else UNKNOWN]
            raise self.printer.command_error(
                "Cannot move X or Y: %s" % (reason,))
        self.next_transform.move(newpos, speed)
        self.last_position = list(newpos)
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
    def _plate_present(self, band, eventtime):
        """The detector's plate judgement, or None while it means nothing:
        the column out (the thermistor input floats) or only just in."""
        if band not in (UNLOCKED, 'locked'):
            self.column_since = None
            return None
        if self.column_since is None:
            self.column_since = eventtime
        if eventtime - self.column_since < self.plate_settle_time:
            return None
        return self.detector.plate_present()
    def _set_enable(self, value, eventtime):
        """Schedule the enable pin; return the reactor time it changes."""
        if value == self.enabled:
            return eventtime
        est = self.enable_mcu.estimated_print_time(eventtime)
        print_time = max(est + PIN_MIN_TIME, self.last_enable_time + 0.010)
        self.mcu_enable.set_digital(print_time, 1 if value else 0)
        self.last_enable_time = print_time
        self.enabled = value
        logging.info("%s: 48V_Z %s", self.name, "on" if value else "off")
        return eventtime + (print_time - est)
    def _latch_fault(self, reason, eventtime, plain=None, detail=None):
        if self.fault:
            return
        self.fault = reason
        self.fault_reason = plain
        self.fault_detail = detail
        self.after_reset = False
        logging.error("%s: fault: %s%s", self.name, reason,
                      " | " + detail if detail else "")
        self._run_template(self.fault_template)
    def _record(self, eventtime):
        """Keep the recent PGOOD polls and IMON readings for the fault and
        glitch log lines."""
        self.pgood_history.append((eventtime, self.power_good))
        hist = self.current_history
        if self.current is not None:
            hist.append((eventtime, self.current))
        while hist and eventtime - hist[0][0] > PEAK_WINDOW:
            hist.popleft()
    def _peak(self):
        return max([c for _t, c in self.current_history] or [None])
    def _diagnostics(self, eventtime):
        """One line: what the eFuse and the bed looked like just now."""
        peak = self._peak()
        ratio = self.present_ratio
        temp = getattr(self.heater, 'last_temp', None)
        polls = " ".join("%d@%.1fs" % (v, eventtime - t)
                         for t, v in self.pgood_history)
        return ("now %s A, peak %s A over %.0f s, Z_PRESENT %s, bed %s C,"
                " PGOOD (value@age) %s"
                % (self._amps(self.current), self._amps(peak), PEAK_WINDOW,
                   "?" if ratio is None else "%.3f" % (ratio,),
                   "?" if temp is None else "%.1f" % (temp,), polls))
    def _glitch(self, eventtime):
        """PGOOD was low for fewer polls than drop_samples, then came back."""
        self.glitches += 1
        self.last_glitch = eventtime
        last = self.glitch_log_time
        if last is not None and eventtime - last < GLITCH_LOG_INTERVAL:
            self.glitch_unlogged += 1
            return
        logging.warning(
            "%s: PGOOD glitch #%d (low %d of %d polls, not latched%s) | %s",
            self.name, self.glitches, self.low_polls, self.drop_samples,
            ", %d more not logged" % (self.glitch_unlogged,)
            if self.glitch_unlogged else "", self._diagnostics(eventtime))
        self.glitch_log_time = eventtime
        self.glitch_unlogged = 0
    def _pgood_low(self, eventtime, state, reason):
        """PGOOD is low on a path that was READY: (state, reason, enable)."""
        if not self.low_run:
            # KAN-3370: an active Z move is stopped on the FIRST low poll,
            # before the debounce or the start-up settle decide anything.
            # A false abort costs one re-home; a late one drives a bed
            # that has lost its 48 V. Only the latch below is debounced.
            # abort_active_move is idempotent (False once aborted), so
            # the _transition call at the latch is a no-op.
            self.low_run = True
            self._abort_z_move("the Z and bed power was lost")
        if self.settle_end is not None and eventtime < self.settle_end:
            # The eFuse's start-up may still be settling (see
            # POWER_GOOD_TIMEOUT), so this is not a trip. Wait for PGOOD as
            # for a start-up: heating is refused meanwhile, and PGOOD
            # that never returns latches "did not rise" as usual.
            logging.info("%s: PGOOD low %.2fs before the start-up settle"
                         " ends; waiting", self.name,
                         self.settle_end - eventtime)
            self.powering_since = eventtime
            self.start_peak = 0.
            return state, reason, True
        self.low_polls += 1
        if self.low_polls < self.drop_samples:
            # Not yet a trip. The eFuse has already cut the feed if it
            # tripped, so nothing is heated meanwhile; a latch zeroes the
            # heater target through _transition.
            return READY, "bed power is on", True
        self._latch_fault("PGOOD dropped (eFuse tripped, peak %s A over"
                          " %.0f s)" % (self._amps(self._peak()),
                                        PEAK_WINDOW), eventtime,
                          DROPPED_REASON, self._diagnostics(eventtime))
        return FAULT, self.fault_reason, False
    def _pgood_high(self, eventtime):
        self.low_run = False
        if self.settle_end is None:
            self.settle_end = eventtime + self.power_good_timeout
        elif eventtime >= self.settle_end:
            self.after_reset = False
        if self.low_polls:
            self._glitch(eventtime)
            self.low_polls = 0
    def latch_fault(self, reason):
        """Latch a fault found elsewhere (e.g. [heater_supply])."""
        self._latch_fault(reason, self.reactor.monotonic())
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
            if band == ABSENT:
                self.column_was_fitted = False
            elif band in (UNLOCKED, 'locked'):
                self.column_was_fitted = True
        self.plate = plate = self._plate_present(band, eventtime)
        self._update_current()
        self._record(eventtime)
        if self.enabled:
            self._check_current(eventtime)
        state, reason, enable = decide(band, plate, self.fault,
                                       self.inhibited, self.fault_reason)
        if enable:
            if not self.enabled:
                self.powering_since = self._set_enable(True, eventtime)
                self.start_peak = 0.
                self.settle_end = None
                self.low_polls = 0
                self.low_run = False
            elif self.power_good:
                state, reason = READY, "bed power is on"
                self.powering_since = None
                self._pgood_high(eventtime)
            elif self.powering_since is None:
                # PGOOD low after the path was up: the eFuse turned its FET
                # off (overload, thermal, UVLO or OVP), or it is a glitch.
                state, reason, enable = self._pgood_low(eventtime, state,
                                                        reason)
            else:
                if self.current is not None:
                    self.start_peak = max(self.start_peak, self.current)
                if eventtime - self.powering_since > self.power_good_timeout:
                    amps = self._amps(self.start_peak
                                      if self.current is not None else None)
                    if self.after_reset:
                        self._latch_fault(
                            "PGOOD did not rise after the reset within"
                            " %.2fs (%s A peak)"
                            % (self.power_good_timeout, amps), eventtime,
                            NO_RETURN_REASON, self._diagnostics(eventtime))
                    else:
                        self._latch_fault(
                            "PGOOD did not rise within %.2fs (%s A peak)"
                            % (self.power_good_timeout, amps), eventtime,
                            None, self._diagnostics(eventtime))
                    state, reason, enable = (FAULT, self.fault_reason
                                             or self.fault, False)
        else:
            self.low_polls = 0
            self.low_run = False
            self.after_reset = False
        self._set_enable(enable, eventtime)
        self._transition(state, reason, eventtime)
        return eventtime + POLL_INTERVAL
    @staticmethod
    def _amps(current):
        return "?" if current is None else "%.2f" % (current,)
    def _abort_z_move(self, reason):
        # KAN-3370: a Z homing or probing move in progress is driving a
        # bed and column that just lost their footing or their power.
        # Stop it now; the homing macros only check the lock before a move
        # starts. A move that does not involve Z is left alone.
        try:
            homing = self.printer.lookup_object('homing', None)
            abort = getattr(homing, 'abort_active_move', None)
            if abort is not None:
                abort(reason)
        except Exception:
            logging.exception("%s: aborting the active homing move failed",
                              self.name)
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
        if band == 'absent':
            self._abort_z_move("the Z column was removed")
        elif band == 'unlocked':
            self._abort_z_move("the Z column was unlocked")
        elif band is None:
            # The reading is in no band and, being the debounced value, has
            # stayed there for present_samples polls: the lever is part way
            # (or the sensor is wrong) while Z may be moving. Once per run:
            # this branch is the change into unknown, not each poll in it.
            self._abort_z_move("the Z column position is unknown")
        self._run_template(self.z_change_template)
    def _transition(self, state, reason, eventtime):
        prev = self.state
        self.state, self.reason = state, reason
        if prev == state:
            return
        logging.info("%s: %s -> %s (%s)", self.name, prev, state, reason)
        if prev != READY:
            return
        # The path just went down: whatever Z move is running has lost the
        # column's 48 V (or the bed's), so stop it.
        self._abort_z_move("the Z and bed power was lost")
        # Stop heating. What a print does about it
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
        glitch_age = None
        if self.last_glitch is not None and eventtime is not None:
            glitch_age = round(eventtime - self.last_glitch, 1)
        power = None
        if self.current is not None:
            power = round(self.current * self._supply_voltage(eventtime), 1)
        return {'state': self.state,
                'reason': self.reason,
                'z': self.z_band.value,
                'xy_allowed': xy_allowed(self.z_band.value,
                                         self.xy_requires_column,
                                         self.column_was_fitted),
                'xy_home_allowed': xy_allowed(self.z_band.value,
                                              self.xy_requires_column),
                'plate_present': self.plate,
                'enabled': self.enabled,
                'power_good': self.power_good,
                'present_ratio': self.present_ratio,
                'current': (None if self.current is None
                            else round(self.current, 3)),
                'power': power,
                'fault': self.fault,
                'fault_detail': self.fault_detail,
                'pgood_glitches': self.glitches,
                'last_pgood_glitch': self.last_glitch,
                'last_pgood_glitch_age': glitch_age}
    # Commands
    cmd_Z_POWER_STATUS_help = "Report the Z column and bed power path"
    def cmd_Z_POWER_STATUS(self, gcmd):
        st = self.get_status(self.reactor.monotonic())
        ratio = st['present_ratio']
        gcmd.respond_info(
            "Z power: %s%s\n"
            "Z_PRESENT ratio %s -> %s | plate %s | enable %s | PGOOD %s\n"
            "current %s A | power %s W | PGOOD glitches %d"
            % (st['state'], " (%s)" % st['reason'] if st['reason'] else "",
               "-" if ratio is None else "%.3f" % (ratio,), st['z'],
               st['plate_present'], int(st['enabled']),
               int(st['power_good']), st['current'], st['power'],
               st['pgood_glitches']))
    cmd_Z_POWER_RESET_help = "Clear a latched Z power fault"
    def cmd_Z_POWER_RESET(self, gcmd):
        # Not mid-print: if the cause is still there the fault re-latches
        # within a second. A paused print is recoverable (reset, resume).
        stats = self.printer.lookup_object('print_stats', None)
        if stats is not None and stats.get_status(
                self.reactor.monotonic()).get('state') == 'printing':
            raise gcmd.error(PRINTING_RESET_ERROR)
        if self.fault:
            gcmd.respond_info("Cleared Z power fault: %s" % (self.fault,))
        # A reset that clears a fault re-enables the feed: if PGOOD does not
        # rise, say so (NO_RETURN_REASON), not "dropped" or "did not rise".
        self.after_reset = bool(self.fault)
        self.fault = self.fault_reason = self.fault_detail = None
        self.low_polls = 0
        self.low_run = False
        self.settle_end = None
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
