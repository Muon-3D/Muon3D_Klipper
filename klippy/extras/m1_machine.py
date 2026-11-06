# M1 machine state: the one owner of what the printer is doing and of what
# each action needs before it may run.
#
# Copyright (C) 2026  Muon3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# WHY THIS EXISTS. Klipper's own state (print_stats, pause_resume,
# idle_timeout) describes a generic printer. The M1 has physical states a
# generic printer does not: a Z column that folds away and has a lock lever,
# a removable plate, a hot-swappable toolhead with two hot-end types, and
# latched fan and bed-power faults. Before this module every event macro
# worked those out for itself, so they disagreed: a toolhead move at boot
# made idle_timeout "Printing" and the absent plate paused a print that did
# not exist. See the 2026-11-04 machine behaviour audit in the internal
# documentation (40-audits/reviews), whose sections this file cites.
#
# WHAT IT DOES, AND WHAT IT LEAVES TO KLIPPER.
#   * It reads the existing sources of truth -- z_power (column, bed power),
#     bed_removal_detector (plate), the toolhead MCU, muon_hotend, the
#     toolhead filament switch, hotend_fan_latch, print_stats, pause_resume
#     -- and publishes one status: the activity, the conditions, and for
#     each action the reason codes (ADR 0051) of what is missing.
#   * MACHINE_WAIT_READY is the start gate: PRINT_START calls it first, and it
#     waits, with one live checklist prompt, until everything a print needs
#     is there. It does not fail on the first missing item.
#   * MACHINE_REQUIRE is the resume gate: it refuses, with the same checklist,
#     and the checklist then follows the machine while it stays paused.
#   * MACHINE_HEAT_WAIT is a heater wait that ends with an error when the thing
#     it heats goes away (plate pulled, column unlocked, toolhead pulled,
#     target zeroed), where TEMPERATURE_WAIT and M190 would wait forever.
#   * While a print is paused it owns the part fan. A fan that was on stays
#     at the print's speed until the hot end, which PAUSE turned off, is
#     below pause_cool_temp, so the nozzle stops oozing sooner, then goes
#     off. RESUME puts back the print's own speed.
#   * MACHINE_TOOLHEAD_CONNECTED tells TOOLHEAD_CONNECT whether a connect is
#     a toolhead plugged in (load the filament it carries) or the toolhead
#     MCU coming up at a Klipper start (do nothing).
#   * It never moves anything and never starts, pauses or cancels a print
#     by itself. Pausing stays with PAUSE (pause_resume), printing with
#     virtual_sdcard, heater safety with heaters/verify_heater, the bed
#     interlock with z_power. Those are Klipper's, and they work.
#
# THE GCODE LOCK. A blocking command holds Klipper's gcode mutex, so a
# prompt button (which sends gcode) cannot run while MACHINE_WAIT_READY waits.
# The gate therefore continues by itself once everything has been present
# for settle_time, and it is stopped by the print cancel every interface
# already has (pause_resume/cancel, which aborts the running PRINT_START,
# KAN-382), by its timeout, or by a latched fault, which it refuses at once
# because the acknowledgement is itself a gcode command. Resume does not
# block for the same reason: a paused print has no file line to abort.
import logging

# --- Reason codes --------------------------------------------------------
# The UI owns the wording (ADR 0051); these are the default English texts
# Klipper's own prompt and console use. Order is display order: the
# physical things first, in the order a person fixes them.
REASONS = {
    'column_absent': "Fit the Z column: fold the Z axis down into the base",
    'column_unlocked': "Lock the Z column: push the locking lever down",
    'column_reading': "Checking the Z column",
    'column_unknown': "The Z column lever is part way: push it fully down"
                      " (or see Z_POWER_STATUS)",
    'plate_absent': "Fit the build plate",
    'plate_unknown': "Checking the build plate",
    'toolhead_disconnected': "Connect the toolhead",
    'hotend_detecting': "Detecting the hot end",
    'hotend_failed': "The hot end sensor is faulty (see MUON_HOTEND_STATUS)",
    'filament_absent': "Load filament: insert it into the toolhead",
    'bed_power_starting': "Bed power is starting",
    'bed_power_inhibited': "Bed power is held off (Z_POWER_INHIBIT ENABLE=0)",
    'bed_power_fault': "Bed power fault: check the bed and its cable, then"
                       " run Z_POWER_RESET",
    'fan_fault': "Hot-end fan fault: check the fan, then press Fan checked",
    'plate_not_cleared': "Clear the last print: take the plate off and"
                         " fit it again",
}
ORDER = list(REASONS)

# Missing items that need a person to acknowledge with a gcode command.
# A gate cannot wait for them (see THE GCODE LOCK), so it refuses at once.
REFUSE = ('bed_power_fault', 'bed_power_inhibited', 'fan_fault',
          'hotend_failed')

# What a heater wait stops for (MACHINE_HEAT_WAIT): the bed needs its plate and
# the column it is powered through, the hot end its toolhead.
HEAT_STOPS = {
    'heater_bed': ('column_absent', 'column_unlocked', 'plate_absent'),
    'extruder': ('toolhead_disconnected', 'hotend_failed', 'fan_fault'),
}

# What each action needs (audit 4.3). Each entry names a check below.
REQUIRES = {
    'print': ('column_locked', 'plate', 'toolhead', 'hotend', 'filament',
              'bed_power', 'fan', 'plate_cleared'),
    'resume': ('column_locked', 'plate', 'toolhead', 'hotend',
               'filament', 'bed_power', 'fan'),
    'home_z': ('column_locked', 'plate', 'toolhead'),
    'home_xy': ('column_fitted', 'toolhead'),
}

# Pause reasons this module shows the prompt for. The others have their
# own flow: 'user' needs none, 'filament_out' and 'fan_fault' keep the
# prompts filament.cfg and fan.cfg already show.
PROMPTED_PAUSES = ('plate_removed', 'column_unlocked',
                   'toolhead_disconnected', 'bed_power_lost')
PAUSE_TITLES = {
    'plate_removed': "Build plate removed",
    'column_unlocked': "Z column unlocked",
    'toolhead_disconnected': "Toolhead disconnected",
    'bed_power_lost': "Bed power off",
}


# --- Pure decisions (unit-tested without Klipper) --------------------------
def column_state(z):
    """z_power's column band: 'reading' before its first sample, 'unknown'
    between bands (the lever part way, or a wiring fault)."""
    if z is None:
        return 'reading'
    if z in ('absent', 'unlocked', 'locked'):
        return z
    return 'unknown'


def plate_state(present):
    """bed_removal_detector.plate_present(): True, False, or None before
    the first full window of readings."""
    if present is None:
        return 'unknown'
    return 'present' if present else 'absent'


def missing(action, c):
    """Reason codes for what `action` still needs, in display order.

    `c` is the conditions dict built by M1Machine.conditions(). Unknown
    actions need nothing."""
    out = []
    for check in REQUIRES.get(action, ()):
        col = c.get('column', 'none')
        if check == 'column_locked':
            if col in ('absent', 'unlocked', 'unknown', 'reading'):
                out.append('column_' + col)
        elif check == 'column_fitted':
            if col in ('absent', 'unknown', 'reading'):
                out.append('column_' + col)
        elif check == 'plate':
            if c.get('plate') in ('absent', 'unknown'):
                out.append('plate_' + c['plate'])
        elif check == 'toolhead':
            if c.get('toolhead') == 'disconnected':
                out.append('toolhead_disconnected')
        elif check == 'hotend':
            # Only meaningful with the toolhead there; a disconnected
            # toolhead is already on the list.
            h = c.get('hotend', 'none')
            if c.get('toolhead') == 'disconnected' or h in ('none', 'ntc',
                                                            'pt1000'):
                continue
            out.append('hotend_failed' if h == 'failed'
                       else 'hotend_detecting')
        elif check == 'filament':
            if (c.get('toolhead') != 'disconnected'
                    and c.get('filament') == 'absent'):
                out.append('filament_absent')
        elif check == 'bed_power':
            # A fault or a hold-off is the power path's own. Any other state
            # but ready is only worth listing once the column and plate are
            # right, since until then they are why the bed has no power.
            bp = c.get('bed_power', 'none')
            if bp == 'fault':
                out.append('bed_power_fault')
            elif bp == 'inhibited':
                out.append('bed_power_inhibited')
            elif bp not in ('ready', 'none') and not any(
                    m.startswith(('column_', 'plate_')) for m in out):
                out.append('bed_power_starting')
        elif check == 'fan':
            if c.get('fan_fault'):
                out.append('fan_fault')
        elif check == 'plate_cleared':
            if not c.get('plate_cleared', True):
                out.append('plate_not_cleared')
    return sorted(set(out), key=ORDER.index)


def activity(ps_state, is_paused, gate, preparing, faults, plate_cleared,
             moving):
    """The one activity (audit 4.1). A job is print_stats printing or
    paused, never toolhead motion (B-2)."""
    if ps_state in ('printing', 'paused'):
        if is_paused or ps_state == 'paused':
            return 'paused'
        if gate:
            return 'waiting'
        if preparing:
            return 'preparing'
        return 'printing'
    if faults:
        return 'fault'
    if ps_state in ('complete', 'cancelled') and not plate_cleared:
        return ps_state
    if ps_state == 'error':
        return 'error'
    if moving:
        return 'busy'
    return 'idle'


def checklist_lines(miss):
    return [REASONS.get(code, code) for code in miss]


# --- The module ------------------------------------------------------------
class M1Machine:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.reactor = printer.get_reactor()
        self.gcode = printer.lookup_object('gcode')
        self.poll_time = config.getfloat('poll_time', 0.5, above=0.05)
        # The gate continues once everything has held this long, so a lever
        # pushed half way or a plate seated twice does not start the print.
        self.settle_time = config.getfloat('settle_time', 3., minval=0.)
        self.gate_timeout = config.getfloat('gate_timeout', 1800., minval=0.)
        # A paused print's bed stays at target this long, then goes off
        # (audit 6.3). The hot end goes off at the pause itself (PAUSE).
        self.pause_bed_off_time = config.getfloat('pause_bed_off_time',
                                                  1800., minval=0.)
        # The pause cool-down (Harry, 2026-11-04): PAUSE turns the hot end
        # off, and a part fan that was on keeps the print's speed (it is not
        # spun up) until the nozzle is below pause_cool_temp, where filament
        # no longer flows out of it, or for pause_cool_max_time at most.
        # The default is the nozzle wipe's fan_off_temp, the same ooze point
        # PRINT_START's wipe uses. 0 disables it.
        self.pause_cool_temp = config.getfloat(
            'pause_cool_temp', self._wipe_fan_off_temp(config), minval=0.)
        self.pause_cool_max_time = config.getfloat('pause_cool_max_time',
                                                   600., minval=0.)
        # The toolhead MCU is non critical, so it is deferred and then
        # connects a moment after every Klipper start. A first connect later
        # than this after ready is a toolhead plugged in, not a start.
        self.toolhead_startup_grace = config.getfloat(
            'toolhead_startup_grace', 15., minval=0.)
        self.require_plate_cleared = config.getboolean(
            'require_plate_cleared', True)
        # Klipper's batch mode (an output file, as the klippy tests run)
        # has no clock to wait on; heaters.py skips its waits there too.
        self.batch = printer.get_start_args().get('debugoutput') is not None
        self.toolhead_mcu_name = config.get('toolhead_mcu', 'toolhead')
        self.filament_sensor_name = config.get(
            'filament_sensor', 'filament_sensor_toolhead')
        self.hotend_name = config.get('hotend', 'extruder')
        # Sources, looked up at connect; each is optional so the module
        # also loads on a configuration without it.
        self.print_stats = self.pause_resume = self.idle_timeout = None
        self.z_power = self.bed_detector = self.toolhead_mcu = None
        self.hotend = self.filament = self.fan_latch = None
        self.fan_modes = self.heaters = self.start_mark = None
        # Tracked state
        self.last_ps_state = None
        self.job_printed = False       # this job reached 'printing'
        self.plate_cleared = True      # see _track_plate
        self.plate_off_since_end = False
        self.last_plate = None
        self.job_targets = {'bed': 0., 'hotend': 0.}
        self.pause_reason = None
        self.paused_at = None
        self.saved_fan = 0.
        self.bed_off_done = False
        self.cooling = None            # when the pause cool-down began
        self.ready_time = None
        self.toolhead_connects = 0     # MACHINE_TOOLHEAD_CONNECTED calls
        self.toolhead_dropped = False  # disconnected since the last one
        self.toolhead_hotplug = False  # the last connect was a plug-in
        self.gate = None               # {'action', 'missing', 'since'}
        self.prompt = None             # what this module last showed
        self.resume_refused = False    # MACHINE_REQUIRE showed the checklist
        printer.register_event_handler('klippy:connect', self._connect)
        printer.register_event_handler('klippy:ready', self._ready)
        for name, func in (
                ('MACHINE_WAIT_READY', self.cmd_MACHINE_WAIT_READY),
                ('MACHINE_REQUIRE', self.cmd_MACHINE_REQUIRE),
                ('MACHINE_HEAT_WAIT', self.cmd_MACHINE_HEAT_WAIT),
                ('MACHINE_PAUSED', self.cmd_MACHINE_PAUSED),
                ('MACHINE_PAUSE_REASON', self.cmd_MACHINE_PAUSE_REASON),
                ('MACHINE_RESUMED', self.cmd_MACHINE_RESUMED),
                ('MACHINE_PLATE_CLEAR', self.cmd_MACHINE_PLATE_CLEAR),
                ('MACHINE_TOOLHEAD_CONNECTED',
                 self.cmd_MACHINE_TOOLHEAD_CONNECTED),
                ('MACHINE_STATUS', self.cmd_MACHINE_STATUS)):
            self.gcode.register_command(name, func,
                                        desc=getattr(self, 'cmd_%s_help'
                                                     % (name,)))

    @staticmethod
    def _wipe_fan_off_temp(config):
        try:
            if config.has_section('nozzle_wipe_smart'):
                return config.getsection('nozzle_wipe_smart').getfloat(
                    'fan_off_temp', 160., note_valid=False)
        except Exception:
            logging.exception("m1_machine: nozzle_wipe_smart fan_off_temp")
        return 160.

    def _connect(self):
        lo = self.printer.lookup_object
        self.print_stats = lo('print_stats', None)
        self.pause_resume = lo('pause_resume', None)
        self.idle_timeout = lo('idle_timeout', None)
        self.z_power = lo('z_power', None)
        self.bed_detector = lo('bed_removal_detector', None)
        self.toolhead_mcu = lo('mcu ' + self.toolhead_mcu_name, None)
        self.hotend = lo('muon_hotend ' + self.hotend_name, None)
        self.filament = lo('filament_switch_sensor '
                           + self.filament_sensor_name, None)
        self.fan_latch = lo('hotend_fan_latch', None)
        self.fan_modes = lo('fan_modes', None)
        self.heaters = lo('heaters', None)
        self.start_mark = lo('gcode_macro _PRINT_START_MARK', None)

    def _ready(self):
        self.ready_time = self.reactor.monotonic()
        self.reactor.register_timer(self._poll, self.reactor.NOW)

    # --- Reading the machine ------------------------------------------------
    def _status_of(self, obj, eventtime):
        try:
            return obj.get_status(eventtime)
        except Exception:
            logging.exception("m1_machine: get_status failed")
            return {}

    def _heater(self, name):
        if self.heaters is None:
            return None
        try:
            return self.heaters.lookup_heater(name)
        except self.printer.config_error:
            return None

    def _target(self, name):
        heater = self._heater(name)
        if heater is None:
            return 0.
        return heater.get_temp(self.reactor.monotonic())[1]

    def ps_state(self, eventtime):
        if self.print_stats is None:
            return 'standby'
        return self._status_of(self.print_stats, eventtime).get('state',
                                                                'standby')

    def is_paused(self):
        return bool(self.pause_resume is not None
                    and self.pause_resume.is_paused)

    def file_printing(self, eventtime):
        """A print job is running and not paused: what filament runout and
        the plate pull mean by "printing" (B-2), never toolhead motion."""
        return (self.ps_state(eventtime) == 'printing'
                and not self.is_paused())

    def conditions(self, eventtime):
        c = {}
        if self.z_power is not None:
            zs = self._status_of(self.z_power, eventtime)
            c['column'] = column_state(zs.get('z'))
            c['bed_power'] = zs.get('state') or 'unknown'
            c['bed_power_reason'] = zs.get('reason') or ''
        else:
            c['column'] = 'none'
            c['bed_power'] = 'none'
            c['bed_power_reason'] = ''
        if self.bed_detector is not None:
            c['plate'] = plate_state(self.bed_detector.plate_present())
        else:
            c['plate'] = 'none'
        if self.toolhead_mcu is not None:
            c['toolhead'] = ('disconnected' if getattr(
                self.toolhead_mcu, 'non_critical_disconnected', False)
                else 'connected')
        else:
            c['toolhead'] = 'none'
        if self.hotend is not None:
            c['hotend'] = self._status_of(self.hotend, eventtime).get(
                'state') or 'unknown'
        else:
            c['hotend'] = 'none'
        if self.filament is not None:
            st = self._status_of(self.filament, eventtime)
            c['filament'] = ('present' if st.get('filament_detected')
                             else 'absent')
        else:
            c['filament'] = 'none'
        c['fan_fault'] = bool(self.fan_latch is not None
                              and getattr(self.fan_latch, 'latched', False))
        c['plate_cleared'] = (self.plate_cleared
                              or not self.require_plate_cleared)
        return c

    def faults(self, c):
        out = []
        if c.get('fan_fault'):
            out.append('fan_fault')
        if c.get('bed_power') == 'fault':
            out.append('bed_power_fault')
        if c.get('hotend') == 'failed':
            out.append('hotend_failed')
        return out

    def _preparing(self):
        if self.start_mark is None:
            return False
        return bool(getattr(self.start_mark, 'variables', {}).get('phase'))

    def _moving(self, eventtime):
        if self.idle_timeout is None:
            return False
        return self._status_of(self.idle_timeout, eventtime).get(
            'state') == 'Printing'

    def get_activity(self, eventtime, c=None):
        if c is None:
            c = self.conditions(eventtime)
        return activity(self.ps_state(eventtime), self.is_paused(),
                        self.gate is not None, self._preparing(),
                        self.faults(c), self.plate_cleared,
                        self._moving(eventtime))

    # --- Tracking ------------------------------------------------------------
    def _poll(self, eventtime):
        try:
            self._track(eventtime)
        except Exception:
            # Status keeps flowing; the next poll tries again.
            logging.exception("m1_machine: poll failed")
        return eventtime + self.poll_time

    def _track(self, eventtime):
        ps = self.ps_state(eventtime)
        c = self.conditions(eventtime)
        paused = self.is_paused()
        if ps != self.last_ps_state:
            self._job_state_changed(self.last_ps_state, ps)
            self.last_ps_state = ps
        if ps == 'printing' and not paused and not self._preparing() \
                and self.gate is None:
            self.job_printed = True
            # The targets the print itself set, for RESUME to put back
            # after something zeroed them (z_power on a column unlock, the
            # detector on a plate pull, TOOLHEAD_DISCONNECT). Only non-zero
            # values: the zeroing itself must not overwrite them.
            for key, name in (('bed', 'heater_bed'),
                              ('hotend', self.hotend_name)):
                t = self._target(name)
                if t > 0.:
                    self.job_targets[key] = t
        self._track_plate(c['plate'])
        if c['toolhead'] == 'disconnected' and self.toolhead_connects:
            # Not before the first connect: at a Klipper start the deferred
            # MCU reads disconnected until it comes up.
            self.toolhead_dropped = True
        if paused:
            self._paused_tick(eventtime)
            self._show_pause_prompt(c)

    def _job_state_changed(self, old, new):
        if new in ('printing',) and old not in ('printing', 'paused'):
            # A new job. Its targets and pause record start empty.
            self.job_printed = False
            self.job_targets = {'bed': 0., 'hotend': 0.}
            self._clear_pause()
        elif new in ('complete', 'cancelled', 'error', 'standby') \
                and old in ('printing', 'paused'):
            if self.job_printed and new in ('complete', 'cancelled', 'error'):
                # Whatever it printed is on the plate (B-1, decision 3).
                self.plate_cleared = False
                self.plate_off_since_end = False
            self._clear_pause()
            self._end_prompt('pause')
            self._end_prompt('gate')

    def _track_plate(self, plate):
        if plate == self.last_plate:
            return
        self.last_plate = plate
        if plate == 'absent':
            self.plate_off_since_end = True
        elif plate == 'present' and self.plate_off_since_end \
                and not self.plate_cleared:
            self.plate_cleared = True
            logging.info("m1_machine: plate taken off and refitted;"
                         " the last print is cleared")

    def _paused_tick(self, eventtime):
        # Staged idle (audit 6.3): the bed goes off after a long pause. The
        # target the job set is kept, and RESUME reheats to it.
        if (self.pause_bed_off_time and not self.bed_off_done
                and self.paused_at is not None
                and eventtime - self.paused_at >= self.pause_bed_off_time):
            heater = self._heater('heater_bed')
            if heater is not None and heater.get_temp(eventtime)[1] > 0.:
                try:
                    self.heaters.set_temperature(heater, 0.)
                    self.gcode.respond_info(
                        "Paused %d min: bed off, RESUME reheats it"
                        % (self.pause_bed_off_time // 60,))
                except Exception:
                    logging.exception("m1_machine: bed off failed")
            self.bed_off_done = True
        self._cool_tick(eventtime)

    # --- The pause cool-down -------------------------------------------------
    def _cool_start(self, eventtime):
        """From MACHINE_PAUSED, inside PAUSE: a part fan that is on is left
        running to cool the nozzle PAUSE just turned off. It is never spun
        up. Off at once when it was off or there is nothing to cool."""
        keep = False
        heater = self._heater(self.hotend_name)
        if (self.pause_cool_temp and self.saved_fan > 0.
                and heater is not None
                and not getattr(self.toolhead_mcu,
                                'non_critical_disconnected', False)):
            keep = heater.get_temp(eventtime)[0] >= self.pause_cool_temp
        if keep:
            self.cooling = eventtime
        else:
            self.cooling = None
            self._script_from_command("M107")

    def _cool_tick(self, eventtime):
        if self.cooling is None:
            return
        if getattr(self.toolhead_mcu, 'non_critical_disconnected', False):
            # The fan went with the toolhead; TOOLHEAD_CONNECT leaves it off.
            self.cooling = None
            return
        heater = self._heater(self.hotend_name)
        if heater is not None:
            temp, target = heater.get_temp(eventtime)
        else:
            temp, target = 0., 0.
        # Cool, or someone heats it again (a filament load, a purge), or
        # long enough. Not otherwise.
        if (temp >= self.pause_cool_temp and target <= 0.
                and (not self.pause_cool_max_time or eventtime - self.cooling
                     < self.pause_cool_max_time)):
            return
        # A command running (a load, RESUME) holds the gcode lock: try again
        # at the next poll rather than queue behind it, as idle_timeout does.
        mutex = getattr(self.gcode, 'get_mutex', None)
        if mutex is not None and mutex().test():
            return
        self.cooling = None
        try:
            self.gcode.run_script("M107")
        except Exception:
            logging.exception("m1_machine: pause cool-down fan off failed")

    def _script_from_command(self, script):
        try:
            self.gcode.run_script_from_command(script)
        except self.printer.command_error:
            logging.exception("m1_machine: %s failed", script)

    def _clear_pause(self):
        self.pause_reason = None
        self.paused_at = None
        self.bed_off_done = False
        self.resume_refused = False
        self.cooling = None

    # --- Prompts -------------------------------------------------------------
    # One prompt at a time, owned by a reason (B-8). This module only ever
    # ends a prompt it showed itself, so it never closes another macro's.
    def _send(self, lines):
        for line in lines:
            self.gcode.respond_raw("// action:" + line)

    def _show(self, owner, title, text_lines, buttons):
        key = (owner, title, tuple(text_lines), tuple(buttons))
        if self.prompt == key:
            return
        lines = ["prompt_begin " + title]
        lines += ["prompt_text " + t for t in text_lines]
        lines += ["prompt_footer_button " + b for b in buttons]
        lines.append("prompt_show")
        self._send(lines)
        self.prompt = key

    def _end_prompt(self, owner):
        if self.prompt is not None and self.prompt[0] == owner:
            self._send(["prompt_end"])
            self.prompt = None

    def _show_pause_prompt(self, c):
        # For the pauses this module prompts for, and for any pause whose
        # RESUME was refused: from then on the checklist follows the machine
        # and turns into Resume once it is empty.
        reason = self.pause_reason
        if reason not in PROMPTED_PAUSES and not self.resume_refused:
            return
        miss = missing('resume', c)
        title = PAUSE_TITLES.get(reason, "Print paused")
        if miss:
            text = ["The print is paused. Before it can resume:"]
            text += ["- " + t for t in checklist_lines(miss)]
            buttons = ["Cancel print|CANCEL_PRINT|error"]
        else:
            text = ["Ready to resume."]
            if reason == 'plate_removed':
                text.append("Check the plate sits flat on its pins: a"
                            " plate that moved spoils the print.")
            buttons = ["Resume|RESUME|primary",
                       "Cancel print|CANCEL_PRINT|error"]
        self._show('pause', title, text, buttons)

    # --- Commands ------------------------------------------------------------
    def _action(self, gcmd):
        action = gcmd.get('ACTION', 'print').lower()
        if action not in REQUIRES:
            raise gcmd.error("ACTION must be one of %s"
                             % (", ".join(sorted(REQUIRES)),))
        return action

    cmd_MACHINE_WAIT_READY_help = (
        "Wait until the machine is ready for ACTION (print), with a live"
        " checklist")
    def cmd_MACHINE_WAIT_READY(self, gcmd):
        action = self._action(gcmd)
        timeout = gcmd.get_float('TIMEOUT', self.gate_timeout, minval=0.)
        if self.batch:
            return
        settle = gcmd.get_float('SETTLE', self.settle_time, minval=0.)
        reactor = self.reactor
        start = eventtime = reactor.monotonic()
        c = self.conditions(eventtime)
        miss = missing(action, c)
        if not miss:
            return
        self._refuse_latched(gcmd, action, miss)
        # Run by hand from the console there is no job to be cancelled.
        in_job = self.ps_state(eventtime) in ('printing', 'paused')
        self.gate = {'action': action, 'missing': miss, 'since': start}
        gcmd.respond_info("Waiting to %s: %s"
                          % (action, "; ".join(checklist_lines(miss))))
        self._run_led('LED_WARNING')
        check_abort = getattr(self.gcode, 'check_abort', None)
        met_since = None
        try:
            while True:
                if self.printer.is_shutdown():
                    raise gcmd.error("Shutdown while waiting to %s"
                                     % (action,))
                if check_abort is not None:
                    check_abort()
                if in_job and self.ps_state(eventtime) not in (
                        'printing', 'paused'):
                    # Without KAN-382's abort, a cancel ends the job under
                    # us: stop here rather than carry on into the heating.
                    raise gcmd.error("Print cancelled while waiting")
                c = self.conditions(eventtime)
                miss = missing(action, c)
                self._refuse_latched(gcmd, action, miss)
                if miss != self.gate['missing']:
                    self.gate['missing'] = miss
                    if miss:
                        gcmd.respond_info("Still waiting: %s" % (
                            "; ".join(checklist_lines(miss)),))
                if miss:
                    met_since = None
                    self._show('gate', "Waiting to start",
                               ["The print starts by itself once:"]
                               + ["- " + t for t in checklist_lines(miss)]
                               + ["Cancel the print to stop waiting."], [])
                else:
                    if met_since is None:
                        met_since = eventtime
                        self._show('gate', "Ready", [
                            "Starting in %d seconds." % (round(settle),)],
                            [])
                    if eventtime - met_since >= settle:
                        break
                if timeout and eventtime - start >= timeout:
                    raise gcmd.error(
                        "Not ready to %s after %d min: %s" % (
                            action, timeout // 60,
                            "; ".join(checklist_lines(miss))))
                eventtime = reactor.pause(eventtime + self.poll_time)
        finally:
            self.gate = None
            self._end_prompt('gate')
        gcmd.respond_info("Ready to %s" % (action,))

    def _refuse_latched(self, gcmd, action, miss):
        latched = [m for m in miss if m in REFUSE]
        if latched:
            raise gcmd.error("Cannot %s: %s" % (
                action, "; ".join(checklist_lines(latched))))

    def _run_led(self, macro):
        if self.printer.lookup_object('gcode_macro ' + macro, None) is None:
            return
        try:
            self.gcode.run_script_from_command(macro)
        except self.printer.command_error:
            logging.exception("m1_machine: %s failed", macro)

    cmd_MACHINE_REQUIRE_help = (
        "Refuse unless the machine is ready for ACTION; WAIT=s first waits"
        " that long")
    def cmd_MACHINE_REQUIRE(self, gcmd):
        action = self._action(gcmd)
        wait = gcmd.get_float('WAIT', 0., minval=0., maxval=10.)
        eventtime = self.reactor.monotonic()
        deadline = eventtime + wait
        while True:
            c = self.conditions(eventtime)
            miss = missing(action, c)
            if not miss or eventtime >= deadline \
                    or any(m in REFUSE for m in miss):
                break
            eventtime = self.reactor.pause(eventtime + 0.2)
        if not miss:
            return
        if action == 'resume' and self.is_paused():
            self.resume_refused = True
            self._show_pause_prompt(self.conditions(eventtime))
        raise gcmd.error("Cannot %s: %s" % (
            action, "; ".join(checklist_lines(miss))))

    cmd_MACHINE_HEAT_WAIT_help = (
        "Wait for HEATER to reach MINIMUM, ending with an error if what it"
        " heats goes away")
    def cmd_MACHINE_HEAT_WAIT(self, gcmd):
        name = gcmd.get('HEATER')
        minimum = gcmd.get_float('MINIMUM')
        heater = self._heater(name)
        if heater is None:
            raise gcmd.error("Unknown heater '%s'" % (name,))
        stops = HEAT_STOPS.get(name, HEAT_STOPS['extruder'])
        if self.batch:
            return
        check_abort = getattr(self.gcode, 'check_abort', None)
        eventtime = self.reactor.monotonic()
        while not self.printer.is_shutdown():
            temp, target = heater.get_temp(eventtime)
            if temp >= minimum:
                return
            # B-7: a wait whose heater was switched off, or whose plate,
            # column or toolhead went away, can never finish. End it.
            if target < minimum:
                raise gcmd.error("%s was turned off while waiting for it"
                                 % (name,))
            c = self.conditions(eventtime)
            gone = [m for m in missing('resume', c) if m in stops]
            if gone:
                raise gcmd.error("Stopped waiting for %s: %s" % (
                    name, "; ".join(checklist_lines(gone))))
            if check_abort is not None:
                check_abort()
            gcmd.respond_raw("%s: %.1f / %.1f" % (name, temp, target))
            eventtime = self.reactor.pause(eventtime + 1.)
        raise gcmd.error("Shutdown while waiting for %s" % (name,))

    cmd_MACHINE_PAUSED_help = (
        "(PAUSE) record a new pause, its REASON and the part-fan speed FAN"
        " to restore; start the cool-down")
    def cmd_MACHINE_PAUSED(self, gcmd):
        reason = gcmd.get('REASON', 'user').lower()
        if self.paused_at is None:
            self.paused_at = now = self.reactor.monotonic()
            self.bed_off_done = False
            self.saved_fan = gcmd.get_float('FAN', 0., minval=0., maxval=1.)
            self.pause_reason = reason
            self._cool_start(now)
        elif self.pause_reason in (None, 'user') and reason != 'user':
            self.pause_reason = reason
        self._after_reason()

    cmd_MACHINE_PAUSE_REASON_help = "Set why the print is paused (REASON=code)"
    def cmd_MACHINE_PAUSE_REASON(self, gcmd):
        if not self.is_paused():
            return
        self.pause_reason = gcmd.get('REASON').lower()
        if self.paused_at is None:
            self.paused_at = self.reactor.monotonic()
        self._after_reason()

    def _after_reason(self):
        self._show_pause_prompt(self.conditions(self.reactor.monotonic()))

    cmd_MACHINE_RESUMED_help = "(RESUME) the pause is over"
    def cmd_MACHINE_RESUMED(self, gcmd):
        self._end_prompt('pause')
        self._clear_pause()

    cmd_MACHINE_PLATE_CLEAR_help = (
        "Say the plate is clear of the last print (at the machine)")
    def cmd_MACHINE_PLATE_CLEAR(self, gcmd):
        self.plate_cleared = True
        gcmd.respond_info("Plate marked clear")

    cmd_MACHINE_TOOLHEAD_CONNECTED_help = (
        "(TOOLHEAD_CONNECT) note a toolhead connect: plugged in, or a"
        " Klipper start")
    def cmd_MACHINE_TOOLHEAD_CONNECTED(self, gcmd):
        # Read back as toolhead_hotplug by the macro TOOLHEAD_CONNECT runs
        # next, which loads the filament a plugged-in toolhead carries.
        now = self.reactor.monotonic()
        first = self.toolhead_connects == 0
        late = (self.ready_time is not None
                and now - self.ready_time > self.toolhead_startup_grace)
        self.toolhead_hotplug = self.toolhead_dropped or (first and late)
        self.toolhead_connects += 1
        self.toolhead_dropped = False

    cmd_MACHINE_STATUS_help = (
        "Report the machine state and what each action needs")
    def cmd_MACHINE_STATUS(self, gcmd):
        st = self.get_status(self.reactor.monotonic())
        lines = ["M1: %s%s" % (st['activity'], (" (%s)" % st['pause_reason'])
                               if st['pause_reason'] else "")]
        lines.append(", ".join("%s=%s" % (k, st['conditions'][k])
                               for k in sorted(st['conditions'])))
        for action in sorted(REQUIRES):
            miss = st['missing'][action]
            lines.append("%s: %s" % (action, "ready" if not miss
                                     else ", ".join(miss)))
        gcmd.respond_info("\n".join(lines))

    def get_status(self, eventtime):
        c = self.conditions(eventtime)
        miss = {a: missing(a, c) for a in REQUIRES}
        return {
            'activity': self.get_activity(eventtime, c),
            'conditions': c,
            'missing': miss,
            'ready': {a: not m for a, m in miss.items()},
            'faults': self.faults(c),
            'pause_reason': self.pause_reason,
            'plate_cleared': c['plate_cleared'],
            'toolhead_hotplug': self.toolhead_hotplug,
            'cooling': self.cooling is not None,
            'job_targets': dict(self.job_targets),
            'saved_fan': self.saved_fan,
            'gate': (None if self.gate is None else
                     {'action': self.gate['action'],
                      'missing': list(self.gate['missing']),
                      'since': self.gate['since']}),
        }


def load_config(config):
    return M1Machine(config)
