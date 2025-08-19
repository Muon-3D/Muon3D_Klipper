# z_sensorless_park.py
#
# Minimal secondary "sensorless park" for Z using the TMC virtual endstop.
# - Independent from normal probe-based Z homing (does NOT mark Z homed).
# - No driver field tweaks.
# - Works whether axes are homed or not.
# - Errors if no stall within axis_length + overshoot.
#
# Command:
#   Z_PARK_SENSORLESS [DIR=MAX|MIN] [SPEED=<mm/s>] [RETRACT=<mm>] [OVERSHOOT=<mm>]
#
# Config section:
#   [z_sensorless_park]
#   stepper: stepper_z
#   driver: tmc2130
#   direction: max
#   speed: 6
#   retract: 3
#   retract_speed: 10
#   overshoot: 5
#
# License: GPLv3 (same as Klipper)

TRINAMIC_DRIVERS = ["tmc2130", "tmc2208", "tmc2209", "tmc2240", "tmc2660", "tmc5160"]

class ZSensorlessPark:
    def __init__(self, config):
        self.printer = config.get_printer()
        # Config
        self.stepper_name = config.get('stepper', 'stepper_z')
        self.driver_hint  = config.get('driver', None)
        self.dir_default  = config.getchoice('direction', ['max','min'], 'max')
        self.speed_default = config.getfloat('speed', 6.0, above=0.)
        self.retract_default = config.getfloat('retract', 3.0, above=0.)
        self.retract_speed_default = config.getfloat('retract_speed', self.speed_default, above=0.)
        self.overshoot_default = config.getfloat('overshoot', 5.0, minval=0.)

        # Find TMC driver module for this stepper and compose its virtual endstop pin
        self.tmc_mod, self.driver_name = self._find_tmc_module()
        if self.tmc_mod is None:
            raise config.error("z_sensorless_park: No TMC driver found for '%s' "
                               "(looked for: %s)" %
                               (self.stepper_name, ", ".join(TRINAMIC_DRIVERS)))
        self.virtual_endstop_pin = "%s_%s:virtual_endstop" % (self.driver_name, self.stepper_name)

        # Create the MCU endstop at CONFIG TIME
        pins = self.printer.lookup_object('pins')
        self.mcu_endstop = pins.setup_pin('endstop', self.virtual_endstop_pin)

        # Register command
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('Z_PARK_SENSORLESS', self.cmd_Z_PARK_SENSORLESS,
                               desc=self.cmd_Z_PARK_SENSORLESS_help)

    def _find_tmc_module(self):
        cand = TRINAMIC_DRIVERS
        if self.driver_hint:
            cand = [self.driver_hint] + [d for d in TRINAMIC_DRIVERS if d != self.driver_hint]
        for drv in cand:
            objname = "%s %s" % (drv, self.stepper_name)  # e.g. "tmc2130 stepper_z"
            m = self.printer.lookup_object(objname, None)
            if m is not None:
                return m, drv
        return None, None

    def _attach_z_steppers(self, kin):
        # Attach every Z-axis stepper to this endstop (safe to repeat)
        for s in kin.get_steppers():
            if s.is_active_axis('z'):
                self.mcu_endstop.add_stepper(s)

    cmd_Z_PARK_SENSORLESS_help = (
        "Sensorless Z parking using TMC virtual endstop (stall). "
        "Does not set Z homed.\n"
        "Params: DIR=MAX|MIN "
        "SPEED=<mm/s> "
        "BACKOFF=<mm> "
        "OVERSHOOT=<mm> "
        "RETRACT_SPEED=<mm/s> "
        "ON_SUCCESS=<gcode|macro> "
        "ON_FAIL=<gcode|macro>"
    )

    def cmd_Z_PARK_SENSORLESS(self, gcmd):
        # Params
        direction = gcmd.get('DIR', self.dir_default).lower()
        if direction not in ('max', 'min'):
            raise gcmd.error("DIR must be MAX or MIN")
        to_max = (direction == 'max')
        sign = 1.0 if to_max else -1.0

        speed     = gcmd.get_float('SPEED', self.speed_default, above=0.)
        retract   = gcmd.get_float('RETRACT', self.retract_default, above=0.)
        retract_speed = gcmd.get_float('RETRACT_SPEED', self.retract_speed_default, above=0.)
        overshoot = gcmd.get_float('OVERSHOOT', self.overshoot_default, minval=0.)

        #soft      = gcmd.get_int('SOFT', 0)               # 1 = don’t raise on fail
        on_success = gcmd.get('ON_SUCCESS', None)         # name of macro or single gcode
        on_fail    = gcmd.get('ON_FAIL', None)            # name of macro or single gcode

        toolhead = self.printer.lookup_object('toolhead')
        phoming  = self.printer.lookup_object('homing')  # PrinterHoming
        kin      = toolhead.get_kinematics()

        # Ensure Z steppers are associated with this endstop
        self._attach_z_steppers(kin)

        # Axis bounds from kinematics (reflect [stepper_z] position_min/max)
        ks = kin.get_status(self.printer.get_reactor().monotonic())
        z_min = ks['axis_minimum'].z
        z_max = ks['axis_maximum'].z
        axis_length = abs(z_max - z_min)

        # Choose an END position *inside* limits
        end_z = z_max if to_max else z_min

        # Choose a fake START position far on the opposite side (force a long move)
        # This START may be outside real limits; that's OK since we set homing_axes="z".
        travel = axis_length + overshoot
        start_z_virtual = end_z - sign * travel  # opposite side

        # Record whether Z was homed
        was_homed = 'z' in ks.get('homed_axes', '')

        # Set the virtual start (like G28 does with "forcepos")
        cur = toolhead.get_position()
        toolhead.set_position([cur[0], cur[1], start_z_virtual, cur[3]], homing_axes="z")

        try:
            # Run a homing-style move to an IN-RANGE end position
            endpos = [cur[0], cur[1], end_z, cur[3]]
            endstops = [(self.mcu_endstop, "z_park")]
            phoming.manual_home(toolhead, endstops, endpos,
                                speed, triggered=True, check_triggered=True)

            # Back off to a safe parked coordinate
            pos_after = toolhead.get_position()
            final_z = pos_after[2] - sign * retract
            toolhead.manual_move([None, None, final_z], retract_speed)

            gcmd.respond_info("Z_PARK_SENSORLESS: stalled toward %s at Z=%.3f, parked at Z=%.3f"
                              % (direction.upper(), pos_after[2], final_z))

            if on_success:
                self.gcode.run_script(on_success)

        except self.printer.command_error as e:
            reason = str(e)
            msg = None
            if "No trigger" in reason or "after full movement" in reason:
                msg = ("Z_PARK_SENSORLESS failed: no stall before travel limit. "
                       "Increase OVERSHOOT, verify DIAG wiring, and ensure a firm stop at Z %s."
                       % (direction.upper(),))
            else:
                msg = "Z_PARK_SENSORLESS error: %s" % reason

            # run fail-branch if configured
            if on_fail:
                try:
                    self.gcode.run_script(on_fail)
                except Exception:
                    pass

            # if soft:
            #     gcmd.respond_info(msg)   # just report it, don’t raise
            #     return

            raise gcmd.error(msg)
    
        finally:
            # Restore 'unhomed' limits if it wasn't homed before
            if not was_homed:
                try:
                    kin.clear_homing_state("z")
                except Exception:
                    pass

def load_config(config):
    return ZSensorlessPark(config)
