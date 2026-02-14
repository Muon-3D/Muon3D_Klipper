# sensorless_movement.py
#
# Axis-agnostic sensorless parking/protected-move helper using TMC virtual
# endstops.
#
# License: GPLv3 (same as Klipper)

TRINAMIC_DRIVERS = [
    "tmc2130", "tmc2208", "tmc2209", "tmc2240", "tmc2660", "tmc5160"
]
AXIS_INDEX = {'x': 0, 'y': 1, 'z': 2}
AXES = tuple(AXIS_INDEX.keys())


class AxisSensorlessPark:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.pins = self.printer.lookup_object('pins')
        self.gcode = self.printer.lookup_object('gcode')

        # Global defaults
        self.axis_default = config.getchoice('axis', list(AXES), 'z')
        self.stepper_default = config.get(
            'stepper', 'stepper_%s' % (self.axis_default,))
        self.driver_default = config.get('driver', None)
        self.dir_default = config.getchoice('direction', ['max', 'min'], 'max')
        self.speed_default = config.getfloat('speed', 6.0, above=0.)
        self.retract_default = config.getfloat('retract', 3.0, above=0.)
        self.retract_speed_default = config.getfloat(
            'retract_speed', self.speed_default, above=0.)
        self.overshoot_default = config.getfloat('overshoot', 5.0, minval=0.)

        # Optional per-axis overrides
        self.stepper_override = {
            axis: config.get('stepper_%s' % (axis,), None) for axis in AXES
        }
        self.driver_override = {
            axis: config.get('driver_%s' % (axis,), None) for axis in AXES
        }

        # Axis runtime state
        self.axis_ctx = {}      # axis -> context dict
        self.axis_errors = {}   # axis -> user-visible error string
        self.endstop_cache = {} # virtual_endstop pin -> mcu_endstop

        # Build axis contexts at config time.
        for axis in AXES:
            self._build_axis_context(
                config, axis, require=(axis == self.axis_default))

        # Generic commands
        self.gcode.register_command(
            'AXIS_PARK_SENSORLESS', self.cmd_AXIS_PARK_SENSORLESS,
            desc=self.cmd_AXIS_PARK_SENSORLESS_help)
        self.gcode.register_command(
            'AXIS_PROTECTED_MOVE', self.cmd_AXIS_PROTECTED_MOVE,
            desc=self.cmd_AXIS_PROTECTED_MOVE_help)

    def _resolve_stepper_name(self, axis):
        sname = self.stepper_override.get(axis)
        if sname:
            return sname
        if axis == self.axis_default:
            return self.stepper_default
        return 'stepper_%s' % (axis,)

    def _resolve_driver_hint(self, axis):
        dhint = self.driver_override.get(axis)
        if dhint:
            return dhint
        if axis == self.axis_default:
            return self.driver_default
        return None

    def _find_tmc_module(self, stepper_name, driver_hint):
        cand = TRINAMIC_DRIVERS
        if driver_hint:
            cand = [driver_hint] + [d for d in TRINAMIC_DRIVERS
                                    if d != driver_hint]
        for drv in cand:
            objname = "%s %s" % (drv, stepper_name)  # "tmc2209 stepper_z"
            module = self.printer.lookup_object(objname, None)
            if module is not None:
                return module, drv
        return None, None

    def _build_axis_context(self, config, axis, require=False):
        stepper_name = self._resolve_stepper_name(axis)
        driver_hint = self._resolve_driver_hint(axis)
        tmc_mod, driver_name = self._find_tmc_module(stepper_name, driver_hint)
        if tmc_mod is None:
            msg = (
                "sensorless_movement: No TMC driver found for axis '%s' "
                "stepper '%s' (looked for: %s)" % (
                    axis.upper(), stepper_name, ", ".join(TRINAMIC_DRIVERS)))
            self.axis_errors[axis] = msg
            if require:
                raise config.error(msg)
            return

        virtual_endstop_pin = "%s_%s:virtual_endstop" % (
            driver_name, stepper_name)
        mcu_endstop = self.endstop_cache.get(virtual_endstop_pin)
        if mcu_endstop is None:
            try:
                mcu_endstop = self.pins.setup_pin(
                    'endstop', virtual_endstop_pin)
            except Exception as exc:
                msg = (
                    "sensorless_movement: Could not setup axis '%s' virtual "
                    "endstop '%s': %s" % (
                        axis.upper(), virtual_endstop_pin, str(exc)))
                self.axis_errors[axis] = msg
                if require:
                    raise config.error(msg)
                return
            self.endstop_cache[virtual_endstop_pin] = mcu_endstop

        self.axis_ctx[axis] = {
            'axis': axis,
            'stepper_name': stepper_name,
            'driver_name': driver_name,
            'tmc_mod': tmc_mod,
            'virtual_endstop_pin': virtual_endstop_pin,
            'mcu_endstop': mcu_endstop,
        }
        self.axis_errors.pop(axis, None)

    def _require_axis_context(self, axis, gcmd):
        ctx = self.axis_ctx.get(axis)
        if ctx is None:
            raise gcmd.error(self.axis_errors.get(
                axis, "Sensorless axis '%s' is not configured"
                % (axis.upper(),)))
        return ctx

    def _parse_axis(self, gcmd, default_axis):
        axis = gcmd.get('AXIS', default_axis).lower()
        if axis not in AXES:
            raise gcmd.error("AXIS must be X, Y, or Z")
        return axis

    def _parse_direction(self, gcmd):
        direction = gcmd.get('DIR', self.dir_default).lower()
        if direction not in ('max', 'min'):
            raise gcmd.error("DIR must be MAX or MIN")
        sign = 1.0 if direction == 'max' else -1.0
        return direction, sign

    def _attach_axis_steppers(self, kin, axis, mcu_endstop):
        # Attach every stepper that participates in this axis move.
        for stepper in kin.get_steppers():
            if stepper.is_active_axis(axis):
                mcu_endstop.add_stepper(stepper)

    def _axis_bounds(self, kin, axis):
        ks = kin.get_status(self.printer.get_reactor().monotonic())
        axis_i = AXIS_INDEX[axis]
        axis_min = ks['axis_minimum'][axis_i]
        axis_max = ks['axis_maximum'][axis_i]
        return ks, axis_i, axis_min, axis_max

    cmd_AXIS_PARK_SENSORLESS_help = (
        "Sensorless axis parking using a TMC virtual endstop (stall). "
        "Does not set axis homed.\n"
        "Params: AXIS=<X|Y|Z> DIR=MAX|MIN SPEED=<mm/s> RETRACT=<mm> "
        "OVERSHOOT=<mm> RETRACT_SPEED=<mm/s> ON_SUCCESS=<gcode|macro> "
        "ON_FAIL=<gcode|macro>"
    )
    cmd_AXIS_PROTECTED_MOVE_help = (
        "Protected axis move using a TMC virtual endstop (stall). "
        "Stops when stall occurs or DIST is reached. Does not set axis homed.\n"
        "Params: AXIS=<X|Y|Z> DIR=MAX|MIN DIST=<mm> SPEED=<mm/s> "
        "BACKOFF=<mm> BACKOFF_SPEED=<mm/s>"
    )

    def _cmd_axis_park(self, gcmd, default_axis):
        axis = self._parse_axis(gcmd, default_axis)
        axis_i = AXIS_INDEX[axis]
        axis_u = axis.upper()
        direction, sign = self._parse_direction(gcmd)

        speed = gcmd.get_float('SPEED', self.speed_default, above=0.)
        retract = gcmd.get_float('RETRACT', self.retract_default, above=0.)
        retract_speed = gcmd.get_float(
            'RETRACT_SPEED', self.retract_speed_default, above=0.)
        overshoot = gcmd.get_float(
            'OVERSHOOT', self.overshoot_default, minval=0.)
        on_success = gcmd.get('ON_SUCCESS', None)
        on_fail = gcmd.get('ON_FAIL', None)

        toolhead = self.printer.lookup_object('toolhead')
        phoming = self.printer.lookup_object('homing')
        kin = toolhead.get_kinematics()
        ctx = self._require_axis_context(axis, gcmd)

        self._attach_axis_steppers(kin, axis, ctx['mcu_endstop'])

        ks, _, axis_min, axis_max = self._axis_bounds(kin, axis)
        axis_length = abs(axis_max - axis_min)
        end_axis = axis_max if direction == 'max' else axis_min
        travel = axis_length + overshoot
        start_axis_virtual = end_axis - sign * travel
        was_homed = axis in ks.get('homed_axes', '')

        cur = toolhead.get_position()
        start_pos = list(cur)
        start_pos[axis_i] = start_axis_virtual
        toolhead.set_position(start_pos, homing_axes=axis)

        try:
            endpos = list(cur)
            endpos[axis_i] = end_axis
            endstops = [(ctx['mcu_endstop'], "%s_park" % (axis,))]
            phoming.manual_home(
                toolhead, endstops, endpos, speed,
                triggered=True, check_triggered=True)

            pos_after = toolhead.get_position()
            final_axis = pos_after[axis_i] - sign * retract
            retract_pos = [None, None, None]
            retract_pos[axis_i] = final_axis
            toolhead.manual_move(retract_pos, retract_speed)

            gcmd.respond_info(
                "AXIS_PARK_SENSORLESS: %s stalled toward %s at %.3f, "
                "parked at %.3f"
                % (axis_u, direction.upper(), pos_after[axis_i], final_axis))
            if on_success:
                self.gcode.run_script(on_success)
        except self.printer.command_error as exc:
            reason = str(exc)
            if "No trigger" in reason or "after full movement" in reason:
                msg = (
                    "AXIS_PARK_SENSORLESS failed on %s: no stall before travel "
                    "limit. Increase OVERSHOOT, verify DIAG wiring, and ensure "
                    "a firm stop toward %s."
                    % (axis_u, direction.upper()))
            else:
                msg = "AXIS_PARK_SENSORLESS error on %s: %s" % (axis_u, reason)
            if on_fail:
                try:
                    self.gcode.run_script(on_fail)
                except Exception:
                    pass
            raise gcmd.error(msg)
        finally:
            if not was_homed:
                try:
                    kin.clear_homing_state(axis)
                except Exception:
                    pass

    def _cmd_axis_protected_move(self, gcmd, default_axis):
        axis = self._parse_axis(gcmd, default_axis)
        axis_i = AXIS_INDEX[axis]
        direction, sign = self._parse_direction(gcmd)

        dist = gcmd.get_float('DIST', None)
        if dist is None:
            raise gcmd.error("DIST is required")
        speed = gcmd.get_float('SPEED', self.speed_default, above=0.)
        backoff = gcmd.get_float('BACKOFF', 0.0, minval=0.)
        backoff_speed = gcmd.get_float('BACKOFF_SPEED', speed, above=0.)

        toolhead = self.printer.lookup_object('toolhead')
        phoming = self.printer.lookup_object('homing')
        kin = toolhead.get_kinematics()
        ctx = self._require_axis_context(axis, gcmd)

        self._attach_axis_steppers(kin, axis, ctx['mcu_endstop'])
        ks = kin.get_status(self.printer.get_reactor().monotonic())
        was_homed = axis in ks.get('homed_axes', '')

        cur = toolhead.get_position()
        target_axis = cur[axis_i] + sign * dist
        toolhead.set_position(list(cur), homing_axes=axis)

        try:
            endpos = list(cur)
            endpos[axis_i] = target_axis
            endstops = [(ctx['mcu_endstop'], "%s_protected" % (axis,))]
            phoming.manual_home(
                toolhead, endstops, endpos, speed,
                triggered=True, check_triggered=False)

            pos_after = toolhead.get_position()
            stalled = abs(pos_after[axis_i] - target_axis) > 0.001
            if stalled and backoff > 0.0:
                final_axis = pos_after[axis_i] - sign * backoff
                backoff_pos = [None, None, None]
                backoff_pos[axis_i] = final_axis
                toolhead.manual_move(backoff_pos, backoff_speed)
        finally:
            if not was_homed:
                try:
                    kin.clear_homing_state(axis)
                except Exception:
                    pass

    def cmd_AXIS_PARK_SENSORLESS(self, gcmd):
        self._cmd_axis_park(gcmd, self.axis_default)

    def cmd_AXIS_PROTECTED_MOVE(self, gcmd):
        self._cmd_axis_protected_move(gcmd, self.axis_default)


def load_config(config):
    return AxisSensorlessPark(config)
