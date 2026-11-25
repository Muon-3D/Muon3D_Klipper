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
# "No trigger" on the confirm says the first touch was false: the bed is
# further down. The home then repeats straight away (retry_settle, default
# 0: the first attempt has no pause either, and false touches came with the
# hotend cold and still), but a repeat is BOUNDED: it lifts at most
# retry_lift mm (never above where the failed attempt started), re-tares and
# searches down at most retry_lift + retry_reach mm, so it can reach only
# retry_reach mm below where the confirm ended.  A repeat that finds nothing
# there is an error and the home stops: the bed is not where any touch said.
# Until 2026-11-25 every repeat soft-homed at the top again and searched the
# whole travel (164 mm) from wherever the head was: on sloe a repeat with no
# contact drove the carriage into its bottom stop and kept going until the
# power was cut.  A fast approach that finds nothing over the full travel is
# still an error.  MAX_TRAVEL=<mm> bounds the first attempt the same way (it
# searches only that far down from where the head is): the bench check that
# a search with no contact stops where it should.
#
# KAN-4682: a home that starts from a KNOWN Z (a stow or park at a height
# the contact guard still vouches for) used to soft-home at the top as well,
# throw the knowledge away and descend the whole travel in 1 s legs with a
# pause at every leg ("went down in small steps, like dead").  Now such a
# home keeps its frame and descends from where it is: the legs above
# fast_margin mm (the plate is expected at Z0) may run longer, as far as the
# measured drift allows (ZLoadBackstop.fast_leg_time), and the last
# fast_margin mm are the 1 s legs of #148, unchanged (12: a 1 s leg is
# 15 mm, so the boundaries fall at +12 and -3, never on the plate at 0,
# where a light touch would be taken into the next tare).  Nothing of the
# trigger, the backstop or the tares is switched off: every leg is armed.
# Z unknown, a repeat after a false touch, or the guard not saying "known":
# the home is what it was.
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
        # A repeat after a false touch: lift this far (less if the attempt
        # had not come down that far), then search this much further down
        # than the confirm reached.  See the top.
        self.retry_lift = config.getfloat('retry_lift', 10., minval=0.,
                                          maxval=20.)
        self.retry_reach = config.getfloat('retry_reach', 10., above=0.,
                                           maxval=20.)
        # KAN-4682: see the top.  fast_margin is where the plate is
        # expected, Z0, plus this much: the legs below it are the short ones.
        # Keep it clear of a multiple of the 1 s leg (15 mm) so no leg
        # boundary falls on the plate
        self.fast_approach = config.getboolean('fast_approach', True)
        self.fast_margin = config.getfloat('fast_margin', 12., minval=5.)
        # a heater this far (C) from its target is still settling
        self.heater_settle = config.getfloat('fast_heater_settle', 3.,
                                             minval=0.)
        self.false_touches = 0
        self.last_attempts = 0
        self.last_retry_travel = 0.
        self.fast_homes = 0
        self.last_fast = None
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command(
            'LOAD_CELL_HOME_Z', self.cmd_LOAD_CELL_HOME_Z,
            desc="Home Z on the load cell; retry if the first touch was"
                 " not the bed")

    def _run(self, script):
        self.gcode.run_script_from_command(script)

    def _home_from_last_probe(self):
        self._run("_HOME_Z_FROM_LAST_PROBE")

    # KAN-4682: the Z above which legs may run long, or None.  Needs Z homed
    # AND the contact guard (move_guard height) saying the height is known:
    # homed since the last bed removal, Z motor off, FORCE_MOVE or failed
    # home; Z above the margin so there is something to save.
    def _fast_above(self, th_status):
        if not self.fast_approach:
            return None, 'off'
        if 'z' not in str(th_status.get('homed_axes', '')):
            return None, 'Z not homed'
        guard = self.printer.lookup_object('contact_guard', None)
        if guard is None:
            return None, 'no contact guard'
        try:
            known = guard.get_status(
                self.printer.get_reactor().monotonic()).get('z_known')
        except Exception:
            known = False
        if not known:
            return None, 'height not known'
        # A heater still settling (a hot end heating at print start) makes
        # the cell's drift rate rise within a leg, faster than a plan from
        # measured drift can follow: 1 s legs only until it is at target.
        heaters = self.printer.lookup_object('heaters', None)
        try:
            for h in (heaters.heaters.values() if heaters else ()):
                st = h.get_status(self.printer.get_reactor().monotonic())
                if (st.get('target', 0.) > 0.
                        and abs(st['temperature'] - st['target'])
                        > self.heater_settle):
                    return None, 'heater settling'
        except Exception:
            return None, 'heaters unreadable'
        z = th_status['position'][2]
        above = self.fast_margin
        if z <= above + 1.:
            return None, 'already within the margin'
        if z > th_status['axis_maximum'][2] + 1e-6:
            return None, 'above the travel'
        return above, 'known'

    def cmd_LOAD_CELL_HOME_Z(self, gcmd):
        tol = gcmd.get_float('SAMPLES_TOLERANCE', 0., minval=0.)
        max_travel = gcmd.get_float('MAX_TRAVEL', 0., minval=0.)
        confirm = "PROBE"
        if tol > 0.:
            confirm += " SAMPLES_TOLERANCE=%.3f" % (tol,)
        toolhead = self.printer.lookup_object('toolhead')
        eventtime = self.printer.get_reactor().monotonic()
        th_status = toolhead.get_status(eventtime)
        z_top = th_status['axis_maximum'][2]
        fast_above, why = self._fast_above(th_status)
        self.last_fast = why
        self._run("SAVE_GCODE_STATE NAME=_LOAD_CELL_HOME_Z_STATE")
        try:
            for attempt in range(1, self.attempts + 1):
                self.last_attempts = attempt
                if attempt > 1 and self.retry_settle > 0.:
                    # G4 then M400: a queued dwell alone would not delay the
                    # tare, which reads the samples arriving now.
                    self._run("G4 P%d\nM400"
                              % (int(self.retry_settle * 1000.),))
                fast = (fast_above is not None and attempt == 1
                        and not max_travel)
                if attempt > 1:
                    # A repeat: bounded, from here (see the top)
                    start = self._retry_frame(start)
                elif max_travel:
                    start = self._bounded_frame(max_travel)
                elif not fast:
                    # Soft-home at the top so the axis can move down
                    self._run("SET_KINEMATIC_POSITION Z=%.3f" % (z_top,))
                    start = z_top
                else:
                    start = self._z(toolhead)
                self._run("LOAD_CELL_TARE")
                probe = ("PROBE PROBE_SPEED=%.3f LIFT_SPEED=%.3f SAMPLES=1"
                         % (self.fast_speed, self.fast_lift_speed))
                if fast:
                    # KAN-4682: from where Z is, in its own frame
                    self.fast_homes += 1
                    probe += " FAST_ABOVE=%.3f" % (fast_above,)
                    try:
                        self._run(probe)
                    except self.gcode.error as e:
                        if "No trigger" not in str(e):
                            raise
                        raise gcmd.error(
                            "LOAD_CELL_HOME_Z: no contact down to Z%.0f"
                            " although the Z height was known, so it is"
                            " not where it was believed. Home again."
                            % (self._z_min(eventtime),))
                elif attempt > 1 or max_travel:
                    self._run_bounded_probe(gcmd, probe, attempt)
                else:
                    self._run(probe)
                start = self._home_from_touch(
                    toolhead, start, attempt > 1 or bool(max_travel))
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
                    if attempt >= self.attempts or max_travel:
                        # MAX_TRAVEL is a check of one bounded search: no
                        # repeat, which would search further
                        raise gcmd.error(
                            "%s. Z home gave up: check that nothing is"
                            " touching the nozzle and that the fan has"
                            " stopped." % (msg,))
                    gcmd.respond_info(
                        "%s. Homing Z again, lifting at most %.0f mm and"
                        " searching at most %.0f mm below here."
                        % (msg, self.retry_lift, self.retry_reach))
                    continue
                self._home_from_last_probe()
                if self.final_lift > 0.:
                    self._run("G91\nG1 Z%.3f F600" % (self.final_lift,))
                return
        except Exception:
            # Z was declared, not found: a failed home must not leave it
            # homed in that frame (G28's homing_scope clears it too, but
            # LOAD_CELL_HOME_Z may be run on its own, e.g. MAX_TRAVEL)
            try:
                self._run("SET_KINEMATIC_POSITION CLEAR_HOMED=Z")
            except Exception:
                logging.exception("LOAD_CELL_HOME_Z: could not clear Z homed")
            raise
        finally:
            self._run("RESTORE_GCODE_STATE NAME=_LOAD_CELL_HOME_Z_STATE")

    def _z(self, toolhead):
        eventtime = self.printer.get_reactor().monotonic()
        return toolhead.get_status(eventtime)['position'][2]

    # _HOME_Z_FROM_LAST_PROBE re-frames Z at the touch.  Returns `start` (the
    # Z this attempt began at, the highest the head has been in it) moved
    # into the new frame, so a repeat can tell how far it may lift.  In a
    # bounded search (keep_floor) a touch within position_min of its floor
    # keeps the old frame, so the confirm cannot go below that floor either.
    def _home_from_touch(self, toolhead, start, keep_floor=False):
        before = self._z(toolhead)
        self._home_from_last_probe()
        shift = self._z(toolhead) - before
        if keep_floor and shift > 0.:
            self._run("SET_KINEMATIC_POSITION Z=%.3f" % (before,))
            shift = 0.
        start += shift
        return start

    # A repeat after a false touch.  Lift min(retry_lift, how far this
    # attempt came down), so the head never goes above where the attempt
    # began (it may have begun against the top stop), then declare Z so that
    # the probe's floor, position_min, is retry_lift + retry_reach below:
    # the probe cannot travel further than that.  Returns the new start.
    def _retry_frame(self, start):
        toolhead = self.printer.lookup_object('toolhead')
        eventtime = self.printer.get_reactor().monotonic()
        lift = max(0., min(self.retry_lift,
                           start - self._z(toolhead)))
        if lift > 0.:
            self._run("G91\nG1 Z%.3f F600" % (lift,))
        return self._bounded_frame(lift + self.retry_reach)

    # Declare Z so that position_min, the probe's floor, is `travel` mm
    # below the head.  Returns the declared Z.
    def _bounded_frame(self, travel):
        eventtime = self.printer.get_reactor().monotonic()
        z = self._z_min(eventtime) + travel
        self._run("SET_KINEMATIC_POSITION Z=%.3f" % (z,))
        self.last_retry_travel = travel
        return z

    # A bounded fast probe (a repeat, or MAX_TRAVEL=).  No trigger within it
    # means the bed is not where it can be: stop here with an error.
    def _run_bounded_probe(self, gcmd, probe, attempt):
        try:
            self._run(probe)
        except self.gcode.error as e:
            if "No trigger" not in str(e):
                raise
            if attempt == 1:
                raise gcmd.error(
                    "LOAD_CELL_HOME_Z: no contact within MAX_TRAVEL %.1f mm,"
                    " so Z stopped there." % (self.last_retry_travel,))
            raise gcmd.error(
                "LOAD_CELL_HOME_Z: repeat %d of %d found no contact within"
                " %.1f mm (%.0f mm below where the confirming tap ended), so"
                " Z stopped there. Check the plate is fitted and nothing"
                " touches the nozzle, then home again."
                % (attempt, self.attempts, self.last_retry_travel,
                   self.retry_reach))

    def _z_min(self, eventtime):
        toolhead = self.printer.lookup_object('toolhead')
        return toolhead.get_status(eventtime)['axis_minimum'][2]

    def get_status(self, eventtime):
        return {'false_touches': self.false_touches,
                'last_attempts': self.last_attempts,
                'last_retry_travel': self.last_retry_travel,
                'fast_homes': self.fast_homes,
                'last_fast': self.last_fast}

def load_config(config):
    return LoadCellHomeZ(config)
