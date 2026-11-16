# Z homing on the load cell, with a false first touch recovered
#
# Copyright (C) 2026  Muon3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# The home is two taps: a fast approach from the top of the axis, which sets
# Z0 where it stops, then a 1 mm lift and a confirming probe at the normal
# speed. The confirming probe can only travel to position_min (a few mm), so
# when the first "touch" was not the bed -- a fan still winding down after a
# nozzle wipe, a knock on the frame -- it finds nothing, and the home used to
# fail and take PRINT_START with it.
#
# "No trigger" on the confirm proves the first touch was false: the bed is
# further down. So the axis is soft-homed at the top again and the whole home
# repeats straight away (retry_settle, default 0: the first attempt has no
# pause either, and false touches came with the hotend cold and still).
# A fast approach that finds nothing over the full travel is still an error.
#
# On sloe (2026-11-11) false touches also came on plain G28s, after 1.4-2.5 mm
# of the fast approach, three in a row after a restart and then about one in
# ten; 15 fast probes from Z40 had none. The cause is not known yet. Each
# false-touch attempt walks the head down ~6 mm (the short fast stop plus the
# 5 mm confirm), so a head far above the bed can need several.
import logging

class LoadCellHomeZ:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.attempts = config.getint('attempts', 5, minval=1)
        self.retry_settle = config.getfloat('retry_settle', 0., minval=0.)
        self.fast_speed = config.getfloat('fast_speed', 15., above=0.)
        self.fast_lift_speed = config.getfloat('fast_lift_speed', 10.,
                                               above=0.)
        self.confirm_lift = config.getfloat('confirm_lift', 1., above=0.)
        self.final_lift = config.getfloat('final_lift', 10., minval=0.)
        self.false_touches = 0
        self.last_attempts = 0
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command(
            'LOAD_CELL_HOME_Z', self.cmd_LOAD_CELL_HOME_Z,
            desc="Home Z on the load cell; retry if the first touch was"
                 " not the bed")

    def _run(self, script):
        self.gcode.run_script_from_command(script)

    def _home_from_last_probe(self):
        self._run("_HOME_Z_FROM_LAST_PROBE")

    def cmd_LOAD_CELL_HOME_Z(self, gcmd):
        tol = gcmd.get_float('SAMPLES_TOLERANCE', 0., minval=0.)
        confirm = "PROBE"
        if tol > 0.:
            confirm += " SAMPLES_TOLERANCE=%.3f" % (tol,)
        toolhead = self.printer.lookup_object('toolhead')
        eventtime = self.printer.get_reactor().monotonic()
        z_top = toolhead.get_status(eventtime)['axis_maximum'][2]
        self._run("SAVE_GCODE_STATE NAME=_LOAD_CELL_HOME_Z_STATE")
        try:
            for attempt in range(1, self.attempts + 1):
                self.last_attempts = attempt
                if attempt > 1 and self.retry_settle > 0.:
                    # G4 then M400: a queued dwell alone would not delay the
                    # tare, which reads the samples arriving now.
                    self._run("G4 P%d\nM400"
                              % (int(self.retry_settle * 1000.),))
                # Soft-home at the top so the axis can move down. After a
                # false touch the head is above the bed by an unknown amount,
                # and the full travel from here covers it.
                self._run("SET_KINEMATIC_POSITION Z=%.3f" % (z_top,))
                self._run("LOAD_CELL_TARE")
                self._run("PROBE PROBE_SPEED=%.3f LIFT_SPEED=%.3f SAMPLES=1"
                          % (self.fast_speed, self.fast_lift_speed))
                self._home_from_last_probe()
                self._run("G91\nG1 Z%.3f F600" % (self.confirm_lift,))
                try:
                    self._run(confirm)
                except self.gcode.error as e:
                    if "No trigger" not in str(e):
                        raise
                    self.false_touches += 1
                    msg = ("LOAD_CELL_HOME_Z: the confirming tap found no"
                           " bed within %.0f mm, so the first touch was not"
                           " the bed (attempt %d of %d)"
                           % (self.confirm_lift - self._z_min(eventtime),
                              attempt, self.attempts))
                    logging.info(msg)
                    if attempt >= self.attempts:
                        raise gcmd.error(
                            "%s. Z home gave up: check that nothing is"
                            " touching the nozzle and that the fan has"
                            " stopped." % (msg,))
                    gcmd.respond_info("%s. Homing Z again from the top."
                                      % (msg,))
                    continue
                self._home_from_last_probe()
                if self.final_lift > 0.:
                    self._run("G91\nG1 Z%.3f F600" % (self.final_lift,))
                return
        finally:
            self._run("RESTORE_GCODE_STATE NAME=_LOAD_CELL_HOME_Z_STATE")

    def _z_min(self, eventtime):
        toolhead = self.printer.lookup_object('toolhead')
        return toolhead.get_status(eventtime)['axis_minimum'][2]

    def get_status(self, eventtime):
        return {'false_touches': self.false_touches,
                'last_attempts': self.last_attempts}

def load_config(config):
    return LoadCellHomeZ(config)
