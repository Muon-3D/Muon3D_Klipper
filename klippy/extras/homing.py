# Helper code for implementing homing operations
#
# Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math

HOMING_START_DELAY = 0.001
ENDSTOP_SAMPLE_TIME = .000015
ENDSTOP_SAMPLE_COUNT = 4
DEFAULT_MAX_SEARCH_DISTANCE_Z = 40.

# Return a completion that completes when all completions in a list complete
def multi_complete(printer, completions):
    if len(completions) == 1:
        return completions[0]
    # Build completion that waits for all completions
    reactor = printer.get_reactor()
    cp = reactor.register_callback(lambda e: [c.wait() for c in completions])
    # If any completion indicates an error, then exit main completion early
    for c in completions:
        reactor.register_callback(
            lambda e, c=c: cp.complete(1) if c.wait() else 0)
    return cp

# Tracking of stepper positions during a homing/probing move
class StepperPosition:
    def __init__(self, stepper, endstop_name):
        self.stepper = stepper
        self.endstop_name = endstop_name
        self.stepper_name = stepper.get_name()
        self.start_pos = stepper.get_mcu_position()
        self.start_cmd_pos = stepper.mcu_to_commanded_position(self.start_pos)
        self.halt_pos = self.trig_pos = None
    def note_home_end(self, trigger_time):
        self.halt_pos = self.stepper.get_mcu_position()
        self.trig_pos = self.stepper.get_past_mcu_position(trigger_time)
    def verify_no_probe_skew(self, haltpos):
        new_start_pos = self.stepper.get_mcu_position(self.start_cmd_pos)
        if new_start_pos != self.start_pos:
            logging.warning(
                "Stepper '%s' position skew after probe: pos %d now %d",
                self.stepper.get_name(), self.start_pos, new_start_pos)

# Implementation of homing/probing moves
class HomingMove:
    def __init__(self, printer, endstops, toolhead=None):
        self.printer = printer
        self.endstops = [es for es in endstops if es[0].get_steppers()]
        if toolhead is None:
            toolhead = printer.lookup_object('toolhead')
        self.toolhead = toolhead
        self.stepper_positions = []
        # Abort support (KAN-3370): the reason an outside caller ended this
        # move, the trigger completions the drip move waits on, and whether
        # the move is still at a stage an abort can reach
        self._abort_reason = None
        self._trigger_completions = []
        self._abortable = False
    def get_mcu_endstops(self):
        return [es for es, name in self.endstops]
    def _calc_endstop_rate(self, mcu_endstop, movepos, speed):
        startpos = self.toolhead.get_position()
        axes_d = [mp - sp for mp, sp in zip(movepos, startpos)]
        move_d = math.sqrt(sum([d*d for d in axes_d[:3]]))
        move_t = move_d / speed
        max_steps = max([(abs(s.calc_position_from_coord(startpos)
                              - s.calc_position_from_coord(movepos))
                          / s.get_step_dist())
                         for s in mcu_endstop.get_steppers()])
        if max_steps <= 0.:
            return .001
        return move_t / max_steps
    def calc_toolhead_pos(self, kin_spos, offsets):
        kin_spos = dict(kin_spos)
        kin = self.toolhead.get_kinematics()
        for stepper in kin.get_steppers():
            sname = stepper.get_name()
            kin_spos[sname] += offsets.get(sname, 0) * stepper.get_step_dist()
        thpos = self.toolhead.get_position()
        cpos = kin.calc_position(kin_spos)
        return [cp if cp is not None else tp
                for cp, tp in zip(cpos, thpos[:3])] + thpos[3:]
    def involves_z(self):
        # True if a stepper that this move watches an endstop for drives Z
        for es, name in self.endstops:
            for s in es.get_steppers():
                if s.is_active_axis('z'):
                    return True
        return False
    def abort(self, reason):
        # End this move now, from any context (it only completes the
        # completions the drip move waits on, so it never blocks).  The
        # greenlet inside homing_move does the cleanup and raises
        # "Homing aborted: <reason>".  False if the move is not (or no
        # longer) at a stage that can be aborted, or was already aborted.
        if not self._abortable or self._abort_reason is not None:
            return False
        self._abort_reason = reason
        self._complete_triggers()
        return True
    def _complete_triggers(self):
        # A true result is how a failed endstop ends the drip early.  These
        # are the endstops' own trigger completions, so the wait_end() in
        # home_wait() returns at once even if the MCU never answers.
        for c in self._trigger_completions:
            if not c.test():
                c.complete(True)
    def homing_move(self, movepos, speed, probe_pos=False,
                    triggered=True, check_triggered=True):
        homing = self.printer.lookup_object('homing', None)
        note_move = getattr(homing, 'note_move_active', None)
        self._abort_reason = None
        self._trigger_completions = []
        self._abortable = True
        if note_move is not None:
            note_move(self, True)
        try:
            return self._homing_move(movepos, speed, probe_pos,
                                     triggered, check_triggered)
        finally:
            self._abortable = False
            if note_move is not None:
                note_move(self, False)
    def _homing_move(self, movepos, speed, probe_pos, triggered,
                     check_triggered):
        # Notify start of homing/probing move
        self.printer.send_event("homing:homing_move_begin", self)
        # Note start location
        self.toolhead.flush_step_generation()
        kin = self.toolhead.get_kinematics()
        kin_spos = {s.get_name(): s.get_commanded_position()
                    for s in kin.get_steppers()}
        self.stepper_positions = [ StepperPosition(s, name)
                                   for es, name in self.endstops
                                   for s in es.get_steppers() ]
        # Start endstop checking.  Prime with the drip feeder's own lead
        # where the toolhead offers it; fall back to the generic lead so an
        # unpatched toolhead.py still works.
        _drip_prime = getattr(self.toolhead, 'get_drip_start_time', None)
        if _drip_prime is None:
            _drip_prime = self.toolhead.get_last_move_time
        print_time = _drip_prime()
        endstop_triggers = []
        for mcu_endstop, name in self.endstops:
            rest_time = self._calc_endstop_rate(mcu_endstop, movepos, speed)
            wait = mcu_endstop.home_start(print_time, ENDSTOP_SAMPLE_TIME,
                                          ENDSTOP_SAMPLE_COUNT, rest_time,
                                          triggered=triggered)
            endstop_triggers.append(wait)
        self._trigger_completions = list(endstop_triggers)
        if self._abort_reason is not None:
            # Aborted while the move was being set up
            self._complete_triggers()
        all_endstop_trigger = multi_complete(self.printer, endstop_triggers)
        # dwell() primes print_time BUFFER_TIME_START out; the drip-lead
        # dwell is enough here and is what the drip move itself uses.
        _dwell = getattr(self.toolhead, 'drip_dwell', None)
        if _dwell is None:
            _dwell = self.toolhead.dwell
        _dwell(HOMING_START_DELAY)
        # Issue move
        error = None
        try:
            if self._abort_reason is None:
                self.toolhead.drip_move(movepos, speed, all_endstop_trigger)
        except self.printer.command_error as e:
            error = "Error during homing move: %s" % (str(e),)
        # No abort can reach the move past this point (nothing pauses
        # between drip_move returning and this line)
        self._abortable = False
        aborted = self._abort_reason is not None
        if aborted:
            error = "Homing aborted: %s" % (self._abort_reason,)
        # Wait for endstops to trigger
        trigger_times = {}
        # The end time only has to be at or after the drip's physical end
        # (get_drip_start_time's floor is above last_step_gen_time, which in
        # the no-trigger case is past the move's end) and it is also what the
        # next queued move inherits: get_last_move_time() would prime it
        # BUFFER_TIME_START (0.25 s) out, leaving the toolhead halted on the
        # plate for that long after every trigger.
        move_end_print_time = _drip_prime()
        for mcu_endstop, name in self.endstops:
            try:
                trigger_time = mcu_endstop.home_wait(move_end_print_time)
            except self.printer.command_error as e:
                if error is None:
                    error = "Error during homing %s: %s" % (name, str(e))
                continue
            except Exception:
                # An aborted move must still reach the end of this method
                # (halt position, homing_move_end).  The usual cause is an
                # endstop MCU that has gone away.
                if not aborted:
                    raise
                logging.exception("homing: home_wait failed on %s while"
                                  " aborting", name)
                continue
            if trigger_time > 0.:
                trigger_times[name] = trigger_time
            elif check_triggered and error is None:
                error = "No trigger on %s after full movement" % (name,)
        # Determine stepper halt positions
        self.toolhead.flush_step_generation()
        for sp in self.stepper_positions:
            tt = trigger_times.get(sp.endstop_name, move_end_print_time)
            sp.note_home_end(tt)
            if aborted:
                # No trigger: the position is where the steppers stopped
                sp.trig_pos = sp.halt_pos
        if probe_pos or aborted:
            halt_steps = {sp.stepper_name: sp.halt_pos - sp.start_pos
                          for sp in self.stepper_positions}
            trig_steps = {sp.stepper_name: sp.trig_pos - sp.start_pos
                          for sp in self.stepper_positions}
            haltpos = trigpos = self.calc_toolhead_pos(kin_spos, trig_steps)
            if trig_steps != halt_steps:
                haltpos = self.calc_toolhead_pos(kin_spos, halt_steps)
            self.toolhead.set_position(haltpos)
            for sp in self.stepper_positions:
                sp.verify_no_probe_skew(haltpos)
        else:
            haltpos = trigpos = movepos
            over_steps = {sp.stepper_name: sp.halt_pos - sp.trig_pos
                          for sp in self.stepper_positions}
            if any(over_steps.values()):
                self.toolhead.set_position(movepos)
                halt_kin_spos = {s.get_name(): s.get_commanded_position()
                                 for s in kin.get_steppers()}
                haltpos = self.calc_toolhead_pos(halt_kin_spos, over_steps)
            self.toolhead.set_position(haltpos)
        # Signal homing/probing move complete
        try:
            self.printer.send_event("homing:homing_move_end", self)
        except self.printer.command_error as e:
            if error is None:
                error = str(e)
        if error is not None:
            raise self.printer.command_error(error)
        return trigpos
    def check_no_movement(self):
        if self.printer.get_start_args().get('debuginput') is not None:
            return None
        for sp in self.stepper_positions:
            if sp.start_pos == sp.trig_pos:
                return sp.endstop_name
        return None

# State tracking of homing requests
class Homing:
    def __init__(self, printer):
        self.printer = printer
        self.toolhead = printer.lookup_object('toolhead')
        self.changed_axes = []
        self.trigger_mcu_pos = {}
        self.adjust_pos = {}
    def set_axes(self, axes):
        self.changed_axes = axes
    def get_axes(self):
        return self.changed_axes
    def get_trigger_position(self, stepper_name):
        return self.trigger_mcu_pos[stepper_name]
    def set_stepper_adjustment(self, stepper_name, adjustment):
        self.adjust_pos[stepper_name] = adjustment
    def _fill_coord(self, coord):
        # Fill in any None entries in 'coord' with current toolhead position
        thcoord = list(self.toolhead.get_position())
        for i in range(len(coord)):
            if coord[i] is not None:
                thcoord[i] = coord[i]
        return thcoord
    def set_homed_position(self, pos):
        self.toolhead.set_position(self._fill_coord(pos))
    def home_rails(self, rails, forcepos, movepos):
        # Notify of upcoming homing operation
        self.printer.send_event("homing:home_rails_begin", self, rails)
        # Alter kinematics class to think printer is at forcepos
        force_axes = [axis for axis in range(3) if forcepos[axis] is not None]
        homing_axes = "".join(["xyz"[i] for i in force_axes])
        startpos = self._fill_coord(forcepos)
        homepos = self._fill_coord(movepos)
        self.toolhead.set_position(startpos, homing_axes=homing_axes)
        # Perform first home
        endstops = [es for rail in rails for es in rail.get_endstops()]
        hi = rails[0].get_homing_info()
        hmove = HomingMove(self.printer, endstops)
        hmove.homing_move(homepos, hi.speed)
        # Perform second home
        if hi.retract_dist:
            # Retract
            startpos = self._fill_coord(forcepos)
            homepos = self._fill_coord(movepos)
            axes_d = [hp - sp for hp, sp in zip(homepos, startpos)]
            move_d = math.sqrt(sum([d*d for d in axes_d[:3]]))
            retract_r = min(1., hi.retract_dist / move_d)
            retractpos = [hp - ad * retract_r
                          for hp, ad in zip(homepos, axes_d)]
            self.toolhead.move(retractpos, hi.retract_speed)
            # Home again
            startpos = [rp - ad * retract_r
                        for rp, ad in zip(retractpos, axes_d)]
            self.toolhead.set_position(startpos)
            hmove = HomingMove(self.printer, endstops)
            hmove.homing_move(homepos, hi.second_homing_speed)
            if hmove.check_no_movement() is not None:
                raise self.printer.command_error(
                    "Endstop %s still triggered after retract"
                    % (hmove.check_no_movement(),))
        # Signal home operation complete
        self.toolhead.flush_step_generation()
        self.trigger_mcu_pos = {sp.stepper_name: sp.trig_pos
                                for sp in hmove.stepper_positions}
        self.adjust_pos = {}
        self.printer.send_event("homing:home_rails_end", self, rails)
        if any(self.adjust_pos.values()):
            # Apply any homing offsets
            kin = self.toolhead.get_kinematics()
            homepos = self.toolhead.get_position()
            kin_spos = {s.get_name(): (s.get_commanded_position()
                                       + self.adjust_pos.get(s.get_name(), 0.))
                        for s in kin.get_steppers()}
            newpos = kin.calc_position(kin_spos)
            for axis in force_axes:
                if newpos[axis] is None:
                    raise self.printer.command_error(
                            "Cannot determine position of toolhead on "
                            "axis %s after homing" % "xyz"[axis])
                homepos[axis] = newpos[axis]
            self.toolhead.set_position(homepos)

class PrinterHoming:
    def __init__(self, config):
        self.printer = config.get_printer()
        # Register g-code commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('G28', self.cmd_G28)
        # KAN-3370: the furthest a Z probing move (the M1's Z homing search
        # is one) may travel in one go.  0 turns the cap off.  Z only: X/Y
        # homing travel is never limited here.
        self.max_search_distance_z = config.getfloat(
            'max_search_distance_z', DEFAULT_MAX_SEARCH_DISTANCE_Z,
            minval=0.)
        # Homing/probing moves in progress (KAN-3370 abort)
        self.active_moves = []
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
    def _handle_connect(self):
        # A non-critical MCU (the M1's toolhead) that goes away takes the
        # probe with it, so the Z move it was ending must stop at once
        for name, obj in self.printer.lookup_objects():
            if not getattr(obj, 'is_non_critical', False):
                continue
            get_name = getattr(obj, 'get_non_critical_disconnect_event_name',
                               None)
            if get_name is None:
                continue
            self.printer.register_event_handler(
                get_name(), self._handle_non_critical_disconnect)
    def _handle_non_critical_disconnect(self):
        self.abort_active_move("the toolhead disconnected")
    def note_move_active(self, hmove, active):
        # Called by HomingMove.homing_move on entry and exit
        if active:
            self.active_moves.append(hmove)
        elif hmove in self.active_moves:
            self.active_moves.remove(hmove)
    def abort_active_move(self, reason, z_only=True):
        # End any homing/probing move in progress at once.  Safe to call
        # from a reactor timer or callback: it only completes the moves'
        # trigger completions and does not wait.  The greenlet running each
        # move then stops its endstops, resynchronises the toolhead to the
        # steps actually taken and raises "Homing aborted: <reason>".
        # Moves that do not drive Z are left alone unless z_only is False.
        # Returns True if a move was aborted, False if there was none.
        aborted = False
        for hmove in list(self.active_moves):
            if z_only and not hmove.involves_z():
                continue
            if hmove.abort(reason):
                aborted = True
        if aborted:
            logging.warning("homing: aborting the active move: %s", reason)
        return aborted
    def manual_home(self, toolhead, endstops, pos, speed,
                    triggered, check_triggered):
        hmove = HomingMove(self.printer, endstops, toolhead)
        try:
            hmove.homing_move(pos, speed, triggered=triggered,
                              check_triggered=check_triggered)
        except self.printer.command_error:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Homing failed due to printer shutdown")
            raise
    def _in_homing(self):
        scope = self.printer.lookup_object('homing_scope', None)
        if getattr(scope, 'in_scope', False):
            return True
        override = self.printer.lookup_object('homing_override', None)
        return bool(getattr(override, 'in_script', False))
    def _cap_search(self, pos):
        # Limit a probing move to max_search_distance_z of Z travel.  Returns
        # (pos, capped).  The target is pulled back along the same line, so
        # the move never travels further than the cap.
        cap = self.max_search_distance_z
        if cap <= 0.:
            return pos, False
        start = self.printer.lookup_object('toolhead').get_position()
        dz = pos[2] - start[2]
        if abs(dz) <= cap:
            return pos, False
        scale = cap / abs(dz)
        capped = list(pos)
        for i in range(2):
            capped[i] = start[i] + (pos[i] - start[i]) * scale
        capped[2] = start[2] + math.copysign(cap, dz)
        return capped, True
    def probing_move(self, mcu_probe, pos, speed):
        endstops = [(mcu_probe, "probe")]
        hmove = HomingMove(self.printer, endstops)
        pos, capped = self._cap_search(pos)
        try:
            epos = hmove.homing_move(pos, speed, probe_pos=True)
        except self.printer.command_error as e:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Probing failed due to printer shutdown")
            if capped and "No trigger on" in str(e):
                logging.info("homing: %s", str(e))
                raise self.printer.command_error(
                    "%s stopped: no contact within %g mm. Check the bed is"
                    " fitted and nothing is wrong with the nozzle"
                    % ("Z homing" if self._in_homing() else "Z probing",
                       self.max_search_distance_z))
            raise
        if hmove.check_no_movement() is not None:
            raise self.printer.command_error(
                "Probe triggered prior to movement")
        return epos
    def cmd_G28(self, gcmd):
        # Move to origin
        axes = []
        for pos, axis in enumerate('XYZ'):
            if gcmd.get(axis, None) is not None:
                axes.append(pos)
        if not axes:
            axes = [0, 1, 2]
        homing_state = Homing(self.printer)
        homing_state.set_axes(axes)
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        try:
            kin.home(homing_state)
        except self.printer.command_error:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Homing failed due to printer shutdown")
            self.printer.lookup_object('stepper_enable').motor_off()
            raise

def load_config(config):
    return PrinterHoming(config)
