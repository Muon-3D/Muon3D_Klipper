# Treat a homing orchestrator macro as one homing scope
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# The M1's G28 is a gcode_macro (core/M1/macros/homing.cfg) that runs the
# per-axis helpers _HOME_Y, _HOME_X and _HOME_Z.  Those helpers move axes
# that are not homed yet by faking them homed with SET_KINEMATIC_POSITION
# SET_HOMED=..., and clear the fake at their end.  Two things follow:
#
# * Their plain G1 back-offs are not homing moves, so a guard that stands
#   down "while homing" (contact_guard) still judges them.  After a Y tap
#   home the toolhead is still pressed against the Y end and the load cell
#   reads that as contact, so _HOME_X's Y back-off was refused (KAN-573).
# * A macro has no try/finally.  Any error between a fake and its clear
#   aborts the macro with the fake still in place, and the printer then
#   reports an axis homed that never was.
#
# This wraps the command so that `in_scope` is true for the whole of it, and
# so that a failure clears the homing state of every axis.  After a failed
# G28 no axis's homed flag can be trusted: the helpers fake unhomed axes,
# and G28's own Z lift overwrites a homed Z with a fake position.
import logging


class HomingScope:
    def __init__(self, config):
        self.printer = config.get_printer()
        # G28 is traditional G-code, so its handler takes the GCodeCommand
        # as is; an extended command's handler would need re-parsing here.
        self.command = 'G28'
        self.depth = 0
        self.orig_cmd = None
        # At ready: gcode_macro renames commands (rename_existing) at
        # klippy:connect, in config order, so only at ready is it certain
        # that the macro is the registered handler.
        self.printer.register_event_handler("klippy:ready",
                                            self._handle_ready)

    def _handle_ready(self):
        gcode = self.printer.lookup_object('gcode')
        orig = gcode.register_command(self.command, None)
        if orig is None:
            raise self.printer.config_error(
                "[homing_scope] command %s is not registered" % (
                    self.command,))
        self.orig_cmd = orig
        gcode.register_command(self.command, self._cmd_scoped)

    @property
    def in_scope(self):
        return self.depth > 0

    def _cmd_scoped(self, gcmd):
        self.depth += 1
        try:
            self.orig_cmd(gcmd)
        except Exception:
            self._clear_homing_state()
            raise
        finally:
            self.depth -= 1

    def _clear_homing_state(self):
        toolhead = self.printer.lookup_object('toolhead')
        kin = toolhead.get_kinematics()
        homed = kin.get_status(self.printer.get_reactor().monotonic())[
            'homed_axes']
        if homed:
            logging.info("homing_scope: %s failed, clearing homed axes %s",
                         self.command, homed)
        kin.clear_homing_state("xyz")

    def get_status(self, eventtime):
        return {'in_scope': self.in_scope}


def load_config(config):
    return HomingScope(config)
