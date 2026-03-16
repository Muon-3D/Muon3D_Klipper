import logging

import chelper
from . import force_move
from . import homing as homing_mod

FORCE_ENABLE_SETTLE_TIME = 0.050


class ExtruderHoming:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[1]
        self.query_name = config.get_name()
        self.target_name = config.get('extruder')
        self.endstop_pin = config.get('endstop_pin')
        self.velocity = config.getfloat('velocity', 5., above=0.)
        self.accel = self.homing_accel = config.getfloat(
            'accel', 0., minval=0.)
        self.min_extrude_temp_override = config.getfloat(
            'min_extrude_temp_override', None)
        self.next_cmd_time = 0.
        self.commanded_pos = 0.
        self.last_halt_position = 0.
        self.last_move_distance = 0.
        self.last_requested_distance = 0.
        self.last_stop_on_endstop = 0
        self.last_triggered = False
        self.mcu_endstop = None
        self.target = self.printer_extruder = self.extruder_stepper = None
        self.stepper = None
        self.motion_queuing = self.printer.load_object(config, 'motion_queuing')
        self.query_endstops = self.printer.load_object(config, 'query_endstops')
        ppins = self.printer.lookup_object('pins')
        self.mcu_endstop = ppins.setup_pin('endstop', self.endstop_pin)
        self.query_endstops.register_endstop(self.mcu_endstop, self.query_name)
        self.trapq = self.motion_queuing.allocate_trapq()
        self.trapq_append = self.motion_queuing.lookup_trapq_append()
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command('EXTRUDER_HOMING_MOVE', "EXTRUDER",
                                   self.name, self.cmd_EXTRUDER_HOMING_MOVE,
                                   desc=self.cmd_EXTRUDER_HOMING_MOVE_help)

    def _handle_connect(self):
        target = self.printer.lookup_object(self.target_name, None)
        if target is None or not hasattr(target, 'extruder_stepper'):
            raise self.printer.config_error(
                "Section '%s' references unknown extruder object '%s'"
                % (self.query_name, self.target_name))
        if target.extruder_stepper is None:
            raise self.printer.config_error(
                "Extruder object '%s' does not have a stepper"
                % (self.target_name,))
        self.target = target
        self.printer_extruder = None
        if hasattr(target, 'last_position'):
            self.printer_extruder = target
        self.extruder_stepper = target.extruder_stepper
        self.stepper = self.extruder_stepper.stepper
        self.mcu_endstop.add_stepper(self.stepper)
        self.commanded_pos = self._get_target_position()

    def _get_target_position(self):
        if self.printer_extruder is not None:
            return self.printer_extruder.last_position
        return self.stepper.get_commanded_position()

    def _set_stepper_position(self, position):
        self.commanded_pos = position
        self.stepper.set_position([position, 0., 0.])

    def _sync_target_position(self, position):
        self.commanded_pos = position
        if self.printer_extruder is None:
            return
        self.printer_extruder.last_position = position
        toolhead = self.printer.lookup_object('toolhead')
        if toolhead.get_extruder() is not self.printer_extruder:
            return
        toolhead.flush_step_generation()
        toolhead.set_extruder(self.printer_extruder, position)
        gcode_move = self.printer.lookup_object('gcode_move', None)
        if gcode_move is not None:
            gcode_move.reset_last_position()

    def _force_enable(self):
        stepper_name = self.stepper.get_name()
        stepper_enable = self.printer.lookup_object('stepper_enable')
        did_enable = stepper_enable.set_motors_enable([stepper_name], True)
        if did_enable:
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.dwell(FORCE_ENABLE_SETTLE_TIME)
        return did_enable

    def _restore_enable(self, did_enable):
        if not did_enable:
            return
        stepper_name = self.stepper.get_name()
        stepper_enable = self.printer.lookup_object('stepper_enable')
        stepper_enable.set_motors_enable([stepper_name], False)

    def _begin_manual_session(self):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.wait_moves()
        self.motion_queuing.wipe_trapq(self.trapq)
        self.commanded_pos = self._get_target_position()
        self._set_stepper_position(self.commanded_pos)
        return self.stepper.set_trapq(self.trapq)

    def _end_manual_session(self, old_trapq):
        self.stepper.set_trapq(old_trapq)
        self.motion_queuing.wipe_trapq(self.trapq)
        self._sync_target_position(self.commanded_pos)
        self._sync_fileoutput_stepper()

    def _sync_fileoutput_stepper(self):
        mcu = self.stepper.get_mcu()
        if not mcu.is_fileoutput():
            return
        # File-output tests skip the MCU position query that a real home_end
        # performs, so resync the simulated step queue explicitly.
        self.sync_print_time()
        mcu_pos = self.stepper.get_mcu_position(self.commanded_pos)
        clock = mcu.print_time_to_clock(self.next_cmd_time)
        ffi_main, ffi_lib = chelper.get_ffi()
        ret = ffi_lib.stepcompress_set_last_position(
            self.stepper._stepqueue, clock, mcu_pos)
        if ret:
            raise self.printer.command_error("Internal error in stepcompress")
        self.stepper._set_mcu_position(mcu_pos)

    def _check_can_move(self, distance):
        if not distance or self.printer_extruder is None:
            return
        heater = self.printer_extruder.get_heater()
        if heater.can_extrude:
            return
        if self.min_extrude_temp_override is not None:
            eventtime = self.printer.get_reactor().monotonic()
            temp, target = heater.get_temp(eventtime)
            if temp >= self.min_extrude_temp_override:
                return
            raise self.printer.command_error(
                "Extruder temperature %.1f below"
                " extruder_homing min_extrude_temp_override %.1f"
                % (temp, self.min_extrude_temp_override))
        raise self.printer.command_error(
            "Extrude below minimum temp\n"
            "See the 'min_extrude_temp' config option for details")

    def _did_stop_early(self, requested_distance, actual_distance):
        tolerance = self.stepper.get_step_dist()
        if requested_distance > 0.:
            return actual_distance < requested_distance - tolerance
        return actual_distance > requested_distance + tolerance

    def sync_print_time(self):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        if self.next_cmd_time > print_time:
            toolhead.dwell(self.next_cmd_time - print_time)
        else:
            self.next_cmd_time = print_time

    def _submit_move(self, movetime, movepos, speed, accel):
        current_pos = self.commanded_pos
        dist = movepos - current_pos
        axis_r, accel_t, cruise_t, cruise_v = force_move.calc_move_time(
            dist, speed, accel)
        self.trapq_append(self.trapq, movetime,
                          accel_t, cruise_t, accel_t,
                          current_pos, 0., 0.,
                          axis_r, 0., 0.,
                          0., cruise_v, accel)
        self.commanded_pos = movepos
        return movetime + accel_t + cruise_t + accel_t

    def _do_homing_move(self, distance, speed, accel, stop_on_endstop):
        self.homing_accel = accel
        pos = [self.commanded_pos + distance, 0., 0., 0.]
        endstops = [(self.mcu_endstop, self.query_name)]
        hmove = homing_mod.HomingMove(self.printer, endstops, self)
        try:
            hmove.homing_move(pos, speed, probe_pos=True,
                              triggered=stop_on_endstop > 0,
                              check_triggered=abs(stop_on_endstop) == 1)
        except self.printer.command_error:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Homing failed due to printer shutdown")
            raise
        return self.commanded_pos

    cmd_EXTRUDER_HOMING_MOVE_help = (
        "Run a bounded extruder move and stop early on an endstop/stall")
    def cmd_EXTRUDER_HOMING_MOVE(self, gcmd):
        distance = gcmd.get_float('DISTANCE')
        if not distance:
            raise gcmd.error("DISTANCE must be non-zero")
        stop_on_endstop = gcmd.get_int('STOP_ON_ENDSTOP', 1)
        if stop_on_endstop not in (1, 2, -1, -2):
            raise gcmd.error("STOP_ON_ENDSTOP must be one of 1, 2, -1, -2")
        speed = gcmd.get_float('SPEED', self.velocity, above=0.)
        accel = gcmd.get_float('ACCEL', self.accel, minval=0.)
        set_position = gcmd.get_float('SET_POSITION', None)
        self._check_can_move(distance)
        logging.info("EXTRUDER_HOMING_MOVE %s distance=%.6f speed=%.6f"
                     " accel=%.6f stop=%d set_position=%s",
                     self.target_name, distance, speed, accel,
                     stop_on_endstop, set_position)
        did_enable = self._force_enable()
        old_trapq = self._begin_manual_session()
        start_pos = self.commanded_pos
        try:
            move_end_pos = self._do_homing_move(distance, speed, accel,
                                                stop_on_endstop)
            if set_position is not None:
                self._set_stepper_position(set_position)
        finally:
            move_end_pos = locals().get('move_end_pos', self.commanded_pos)
            self.last_halt_position = move_end_pos
            self.last_requested_distance = distance
            self.last_move_distance = move_end_pos - start_pos
            self.last_stop_on_endstop = stop_on_endstop
            self.last_triggered = (
                stop_on_endstop > 0
                and self._did_stop_early(distance, self.last_move_distance))
            self._end_manual_session(old_trapq)
            self._restore_enable(did_enable)
        gcmd.respond_info("Extruder '%s' moved %.6fmm to %.6f"
                          % (self.target_name, move_end_pos - start_pos,
                             self.commanded_pos), log=False)

    def get_status(self, eventtime):
        return {
            'position': self.commanded_pos,
            'last_halt_position': self.last_halt_position,
            'last_move_distance': self.last_move_distance,
            'last_requested_distance': self.last_requested_distance,
            'last_stop_on_endstop': self.last_stop_on_endstop,
            'last_triggered': self.last_triggered,
        }

    # Toolhead wrappers to support homing
    def flush_step_generation(self):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()

    def get_position(self):
        return [self.commanded_pos, 0., 0., 0.]

    def set_position(self, newpos, homing_axes=""):
        self._set_stepper_position(newpos[0])

    def get_last_move_time(self):
        self.sync_print_time()
        return self.next_cmd_time

    def dwell(self, delay):
        self.next_cmd_time += max(0., delay)

    def drip_move(self, newpos, speed, drip_completion):
        self.sync_print_time()
        start_time = self.next_cmd_time
        end_time = self._submit_move(start_time, newpos[0],
                                     speed, self.homing_accel)
        self.motion_queuing.drip_update_time(start_time, end_time,
                                             drip_completion)
        self.motion_queuing.wipe_trapq(self.trapq)
        self._set_stepper_position(self.commanded_pos)
        self.sync_print_time()

    def get_kinematics(self):
        return self

    def get_steppers(self):
        return [self.stepper]

    def calc_position(self, stepper_positions):
        return [stepper_positions[self.stepper.get_name()], 0., 0.]


def load_config_prefix(config):
    return ExtruderHoming(config)
