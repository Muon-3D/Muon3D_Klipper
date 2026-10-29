# A bounded extruder retract that works with a cold hot end
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# The M1's umbilical unlock press retracts the filament 16 mm so the
# toolhead can come off, and it has to work cold, where G1 E refuses
# (min_extrude_temp).  It used FORCE_MOVE, which is why enable_force_move
# stayed on, and with it raw motion of any stepper, any distance, for any
# client that reaches the G-code endpoint (KAN-60, KAN-380).
#
# COLD_RETRACT moves one configured extruder stepper backwards only, by at
# most max_distance, at a configured speed, through force_move's manual
# move.  With it FORCE_MOVE can be turned off.
import logging


class ColdRetract:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.stepper_name = config.get('stepper', 'extruder')
        self.max_distance = config.getfloat('max_distance', 16., above=0.)
        self.velocity = config.getfloat('velocity', 20., above=0.)
        self.accel = config.getfloat('accel', 1000., minval=0.)
        self.force_move = self.printer.load_object(config, 'force_move')
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('COLD_RETRACT', self.cmd_COLD_RETRACT,
                               desc=self.cmd_COLD_RETRACT_help)

    def _handle_connect(self):
        # Steppers register with force_move as their sections load
        self.stepper = self.force_move.lookup_stepper(self.stepper_name)

    cmd_COLD_RETRACT_help = ("Retract the extruder at most a set distance,"
                             " hot end hot or cold")
    def cmd_COLD_RETRACT(self, gcmd):
        dist = gcmd.get_float('DISTANCE', self.max_distance, above=0.,
                              maxval=self.max_distance)
        logging.info("COLD_RETRACT %s distance=%.3f velocity=%.3f",
                     self.stepper_name, dist, self.velocity)
        self.force_move._force_enable(self.stepper)
        self.force_move.manual_move(self.stepper, -dist, self.velocity,
                                    self.accel)


def load_config(config):
    return ColdRetract(config)
