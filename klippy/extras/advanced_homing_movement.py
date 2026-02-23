# advanced_homing_movement.py
#
# Axis-agnostic parking/protected-move helper using either:
# - TMC virtual endstops (sensorless stall), or
# - The axis' configured endstop (for example adxl345_probe:x/y_virtual_endstop)
#
# License: GPLv3 (same as Klipper)
from . import homing as homing_mod

TRINAMIC_DRIVERS = [
    "tmc2130", "tmc2208", "tmc2209", "tmc2240", "tmc2660", "tmc5160"
]
AXIS_INDEX = {'x': 0, 'y': 1, 'z': 2}
AXES = tuple(AXIS_INDEX.keys())
ENDSTOP_SOURCES = ('tmc_virtual', 'configured')
ENDSTOP_SOURCE_ALIASES = {
    'tmc': 'tmc_virtual',
    'sensorless': 'tmc_virtual',
    'virtual': 'tmc_virtual',
    'configured': 'configured',
    'rail': 'configured',
}


class AxisAdvancedHomingMovement:
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
        self.retract_default = config.getfloat('retract', 3.0, minval=0.)
        self.retract_speed_default = config.getfloat(
            'retract_speed', self.speed_default, above=0.)
        self.overshoot_default = config.getfloat('overshoot', 5.0, minval=0.)
        self.endstop_source_default = self._parse_endstop_source(
            config, 'endstop_source', 'tmc_virtual')

        # Optional per-axis overrides
        self.stepper_override = {
            axis: config.get('stepper_%s' % (axis,), None) for axis in AXES
        }
        self.driver_override = {
            axis: config.get('driver_%s' % (axis,), None) for axis in AXES
        }
        self.endstop_source_override = {
            axis: self._parse_endstop_source(
                config, 'endstop_source_%s' % (axis,), None)
            for axis in AXES
        }

        # Axis runtime state
        self.axis_ctx = {}      # axis -> context dict
        self.axis_errors = {}   # axis -> user-visible error string
        self.endstop_cache = {} # virtual_endstop pin -> mcu_endstop

        # Keep config for lazy axis initialization.
        self._config = config

        # Build only default TMC context at config time.
        if self._resolve_endstop_source(self.axis_default) == 'tmc_virtual':
            self._build_axis_context(config, self.axis_default, require=True)

        # Generic commands
        self.gcode.register_command(
            'AXIS_HOME_MOVE', self.cmd_AXIS_HOME_MOVE,
            desc=self.cmd_AXIS_HOME_MOVE_help)
        self.gcode.register_command(
            'AXIS_PROTECTED_MOVE', self.cmd_AXIS_PROTECTED_MOVE,
            desc=self.cmd_AXIS_PROTECTED_MOVE_help)

    def _parse_endstop_source(self, config, option, default):
        raw = config.get(option, default)
        if raw is None:
            return None
        source = ENDSTOP_SOURCE_ALIASES.get(raw.strip().lower(),
                                            raw.strip().lower())
        if source not in ENDSTOP_SOURCES:
            raise config.error(
                "%s must be one of: %s"
                % (option, ", ".join(ENDSTOP_SOURCES)))
        return source

    def _resolve_endstop_source(self, axis):
        source = self.endstop_source_override.get(axis)
        if source is not None:
            return source
        return self.endstop_source_default

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

    def _build_tmc_axis_context(self, config, axis, require=False):
        stepper_name = self._resolve_stepper_name(axis)
        driver_hint = self._resolve_driver_hint(axis)
        tmc_mod, driver_name = self._find_tmc_module(stepper_name, driver_hint)
        if tmc_mod is None:
            msg = (
                "advanced_homing_movement: No TMC driver found for axis '%s' "
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
                mcu_endstop = self.pins.setup_pin('endstop', virtual_endstop_pin)
            except Exception as exc:
                msg = (
                    "advanced_homing_movement: Could not setup axis '%s' "
                    "virtual endstop '%s': %s" % (
                        axis.upper(), virtual_endstop_pin, str(exc)))
                self.axis_errors[axis] = msg
                if require:
                    raise config.error(msg)
                return
            self.endstop_cache[virtual_endstop_pin] = mcu_endstop

        self.axis_ctx[axis] = {
            'axis': axis,
            'source': 'tmc_virtual',
            'stepper_name': stepper_name,
            'driver_name': driver_name,
            'tmc_mod': tmc_mod,
            'endstop_desc': "TMC virtual endstop '%s'" % (virtual_endstop_pin,),
            'mcu_endstop': mcu_endstop,
        }
        self.axis_errors.pop(axis, None)

    def _find_axis_configured_endstop(self, axis):
        toolhead = self.printer.lookup_object('toolhead', None)
        if toolhead is None:
            return None, None, "toolhead object is unavailable"
        kin = toolhead.get_kinematics()
        rails = getattr(kin, 'rails', None)
        axis_i = AXIS_INDEX[axis]
        if rails is None or axis_i >= len(rails):
            return None, None, "kinematics does not expose rail index for %s" % (
                axis.upper(),)
        rail = rails[axis_i]
        endstops = rail.get_endstops()
        if not endstops:
            return None, None, "no configured endstop on rail '%s'" % (
                rail.get_name(),)
        selected = endstops[0]
        for mcu_endstop, name in endstops:
            lname = str(name).lower()
            if axis in lname:
                selected = (mcu_endstop, name)
                break
        return selected[0], selected[1], None

    def _build_configured_axis_context(self, config, axis, require=False):
        mcu_endstop, endstop_name, err = self._find_axis_configured_endstop(axis)
        if mcu_endstop is None:
            msg = (
                "advanced_homing_movement: Could not resolve configured "
                "endstop for axis '%s': %s" % (axis.upper(), err))
            self.axis_errors[axis] = msg
            if require:
                raise config.error(msg)
            return
        self.axis_ctx[axis] = {
            'axis': axis,
            'source': 'configured',
            'endstop_name': str(endstop_name),
            'endstop_desc': "configured endstop '%s'" % (endstop_name,),
            'mcu_endstop': mcu_endstop,
        }
        self.axis_errors.pop(axis, None)

    def _build_axis_context(self, config, axis, require=False):
        source = self._resolve_endstop_source(axis)
        if source == 'configured':
            self._build_configured_axis_context(config, axis, require=require)
        else:
            self._build_tmc_axis_context(config, axis, require=require)

    def _require_axis_context(self, axis, gcmd):
        expected_source = self._resolve_endstop_source(axis)
        ctx = self.axis_ctx.get(axis)
        if ctx is None or ctx.get('source') != expected_source:
            # Lazy-init (or refresh if source changed).
            self._build_axis_context(self._config, axis, require=False)
            ctx = self.axis_ctx.get(axis)
        if ctx is None:
            raise gcmd.error(self.axis_errors.get(
                axis, "Axis '%s' is not configured" % (axis.upper(),)))
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

    def _enter_virtual_axis_frame(self, toolhead, axis, axis_i, virtual_axis_pos):
        cur = toolhead.get_position()
        coord_offset = cur[axis_i] - virtual_axis_pos
        start_pos = list(cur)
        start_pos[axis_i] = virtual_axis_pos
        toolhead.set_position(start_pos, homing_axes=axis)
        return cur, coord_offset

    def _restore_axis_frame(self, toolhead, axis, axis_i, coord_offset):
        if coord_offset is None or abs(coord_offset) < 1e-9:
            return
        corrected = list(toolhead.get_position())
        corrected[axis_i] += coord_offset
        toolhead.set_position(corrected, homing_axes=axis)

    def _restore_unhomed_axis_coordinate(self, toolhead, axis_i, original_axis):
        # Preserve the pre-move ambiguous coordinate when axis was not homed.
        pos = list(toolhead.get_position())
        pos[axis_i] = original_axis
        toolhead.set_position(pos)

    def _clamp_axis_target(self, target, axis_min, axis_max):
        if target < axis_min:
            return axis_min
        if target > axis_max:
            return axis_max
        return target

    def _run_homing_move(self, toolhead, endstops, endpos, speed,
                         triggered, check_triggered, probe_pos):
        # Use HomingMove directly so probe_pos=True can preserve actual
        # trigger travel instead of snapping to the nominal homing endpoint.
        hmove = homing_mod.HomingMove(self.printer, endstops, toolhead)
        try:
            return hmove.homing_move(
                endpos, speed, probe_pos=probe_pos, triggered=triggered,
                check_triggered=check_triggered)
        except self.printer.command_error:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Homing failed due to printer shutdown")
            raise

    cmd_AXIS_HOME_MOVE_help = (
        "Axis parking using selected endstop source "
        "(TMC virtual stall or configured endstop). "
        "Does not set axis homed.\n"
        "Params: AXIS=<X|Y|Z> DIR=MAX|MIN SPEED=<mm/s> RETRACT=<mm> "
        "OVERSHOOT=<mm> RETRACT_SPEED=<mm/s> ON_SUCCESS=<gcode|macro> "
        "ON_FAIL=<gcode|macro>"
    )
    cmd_AXIS_PROTECTED_MOVE_help = (
        "Protected axis move using selected endstop source "
        "(TMC virtual stall or configured endstop). "
        "Stops when endstop triggers or DIST is reached. "
        "Does not set axis homed.\n"
        "Params: AXIS=<X|Y|Z> DIR=MAX|MIN DIST=<mm> SPEED=<mm/s> "
        "BACKOFF=<mm> BACKOFF_SPEED=<mm/s>"
    )

    def _cmd_axis_park(self, gcmd, default_axis):
        axis = self._parse_axis(gcmd, default_axis)
        axis_i = AXIS_INDEX[axis]
        axis_u = axis.upper()
        direction, sign = self._parse_direction(gcmd)

        speed = gcmd.get_float('SPEED', self.speed_default, above=0.)
        retract = gcmd.get_float('RETRACT', self.retract_default, minval=0.)
        retract_speed = gcmd.get_float(
            'RETRACT_SPEED', self.retract_speed_default, above=0.)
        overshoot = gcmd.get_float(
            'OVERSHOOT', self.overshoot_default, minval=0.)
        on_success = gcmd.get('ON_SUCCESS', None)
        on_fail = gcmd.get('ON_FAIL', None)

        toolhead = self.printer.lookup_object('toolhead')
        kin = toolhead.get_kinematics()
        ctx = self._require_axis_context(axis, gcmd)

        self._attach_axis_steppers(kin, axis, ctx['mcu_endstop'])

        ks, _, axis_min, axis_max = self._axis_bounds(kin, axis)
        axis_length = abs(axis_max - axis_min)
        end_axis = axis_max if direction == 'max' else axis_min
        travel = axis_length + overshoot
        start_axis_virtual = end_axis - sign * travel
        was_homed = axis in ks.get('homed_axes', '')
        original_axis = toolhead.get_position()[axis_i]

        coord_offset = None
        frame_restored = False
        try:
            cur, coord_offset = self._enter_virtual_axis_frame(
                toolhead, axis, axis_i, start_axis_virtual)

            endpos = list(cur)
            endpos[axis_i] = end_axis
            endstops = [(ctx['mcu_endstop'], "%s_park" % (axis,))]
            self._run_homing_move(
                toolhead, endstops, endpos, speed,
                triggered=True, check_triggered=True, probe_pos=True)

            pos_after_virtual = toolhead.get_position()
            trigger_axis_real = pos_after_virtual[axis_i] + coord_offset
            final_axis_real = trigger_axis_real
            if was_homed:
                # Retract in real coordinates when the axis already had a
                # trusted frame.
                self._restore_axis_frame(toolhead, axis, axis_i, coord_offset)
                frame_restored = True
                if retract > 0.:
                    final_axis_real = self._clamp_axis_target(
                        trigger_axis_real - sign * retract, axis_min, axis_max)
                    retract_pos = [None, None, None]
                    retract_pos[axis_i] = final_axis_real
                    toolhead.manual_move(retract_pos, retract_speed)
            else:
                # Retract while still in virtual frame so arbitrary unhomed
                # coordinates can not cause bounds errors.
                if retract > 0.:
                    if retract > axis_length:
                        raise gcmd.error(
                            "RETRACT=%.3f exceeds %s axis span %.3f"
                            % (retract, axis_u, axis_length))
                    safe_trigger_virtual = pos_after_virtual[axis_i]
                    if sign < 0.:
                        # Moving toward MIN, retract goes positive.
                        max_trigger = axis_max - retract
                        if safe_trigger_virtual > max_trigger:
                            safe_trigger_virtual = max_trigger
                    else:
                        # Moving toward MAX, retract goes negative.
                        min_trigger = axis_min + retract
                        if safe_trigger_virtual < min_trigger:
                            safe_trigger_virtual = min_trigger
                    if abs(safe_trigger_virtual - pos_after_virtual[axis_i]) > 1e-9:
                        adjusted = list(pos_after_virtual)
                        adjusted[axis_i] = safe_trigger_virtual
                        toolhead.set_position(adjusted, homing_axes=axis)
                    final_axis_virtual = safe_trigger_virtual - sign * retract
                    retract_pos = [None, None, None]
                    retract_pos[axis_i] = final_axis_virtual
                    toolhead.manual_move(retract_pos, retract_speed)
                self._restore_axis_frame(toolhead, axis, axis_i, coord_offset)
                frame_restored = True

            if was_homed:
                gcmd.respond_info(
                    "AXIS_HOME_MOVE: %s triggered toward %s using %s at "
                    "%.3f, parked at %.3f"
                    % (axis_u, direction.upper(), ctx['endstop_desc'],
                       trigger_axis_real, final_axis_real))
            else:
                gcmd.respond_info(
                    "AXIS_HOME_MOVE: %s triggered toward %s using %s. "
                    "Axis was unhomed, so coordinate remains unchanged."
                    % (axis_u, direction.upper(), ctx['endstop_desc']))
            if on_success:
                self.gcode.run_script(on_success)
        except self.printer.command_error as exc:
            reason = str(exc)
            if not frame_restored:
                try:
                    self._restore_axis_frame(toolhead, axis, axis_i, coord_offset)
                    frame_restored = True
                except Exception:
                    pass
            if "No trigger" in reason or "after full movement" in reason:
                msg = (
                    "AXIS_HOME_MOVE failed on %s: no trigger before "
                    "travel limit while using %s. Increase OVERSHOOT and "
                    "verify endstop behavior toward %s."
                    % (axis_u, ctx['endstop_desc'], direction.upper()))
            else:
                msg = "AXIS_HOME_MOVE error on %s: %s" % (axis_u, reason)
            if on_fail:
                try:
                    self.gcode.run_script(on_fail)
                except Exception:
                    pass
            raise gcmd.error(msg)
        finally:
            if not frame_restored:
                try:
                    self._restore_axis_frame(toolhead, axis, axis_i, coord_offset)
                except Exception:
                    pass
            if not was_homed:
                try:
                    self._restore_unhomed_axis_coordinate(
                        toolhead, axis_i, original_axis)
                except Exception:
                    pass
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
        kin = toolhead.get_kinematics()
        ctx = self._require_axis_context(axis, gcmd)

        self._attach_axis_steppers(kin, axis, ctx['mcu_endstop'])
        ks = kin.get_status(self.printer.get_reactor().monotonic())
        was_homed = axis in ks.get('homed_axes', '')
        original_axis = toolhead.get_position()[axis_i]

        cur = toolhead.get_position()
        target_axis = cur[axis_i] + sign * dist
        toolhead.set_position(list(cur), homing_axes=axis)

        try:
            endpos = list(cur)
            endpos[axis_i] = target_axis
            endstops = [(ctx['mcu_endstop'], "%s_protected" % (axis,))]
            self._run_homing_move(
                toolhead, endstops, endpos, speed,
                triggered=True, check_triggered=False, probe_pos=True)

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
                    self._restore_unhomed_axis_coordinate(
                        toolhead, axis_i, original_axis)
                except Exception:
                    pass
                try:
                    kin.clear_homing_state(axis)
                except Exception:
                    pass

    def cmd_AXIS_HOME_MOVE(self, gcmd):
        self._cmd_axis_park(gcmd, self.axis_default)

    def cmd_AXIS_PROTECTED_MOVE(self, gcmd):
        self._cmd_axis_protected_move(gcmd, self.axis_default)


def load_config(config):
    return AxisAdvancedHomingMovement(config)


def load_config_prefix(config):
    return AxisAdvancedHomingMovement(config)
