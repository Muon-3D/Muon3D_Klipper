# A G-code command that stops the running script with a plain error
#
# Copyright (C) 2026  Muon3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# A macro that has to stop PRINT_START (no filament, no toolhead) can only
# do it by raising an error, and Jinja's action_raise_error surfaces as
# "Error evaluating 'gcode_macro X:gcode': gcode.CommandError: ...". This
# raises the message as given, so the console and the UI show only it.
# Raise it from its own command after any prompt has been sent: a macro
# renders in full before it runs, so an error raised while rendering stops
# the macro before its prompt goes out.

class RaiseError:
    def __init__(self, config):
        gcode = config.get_printer().lookup_object('gcode')
        gcode.register_command('RAISE_ERROR', self.cmd_RAISE_ERROR,
                               desc=self.cmd_RAISE_ERROR_help)
    cmd_RAISE_ERROR_help = "Stop the running script with the error MSG"
    def cmd_RAISE_ERROR(self, gcmd):
        raise gcmd.error(gcmd.get('MSG', 'Stopped'))

def load_config(config):
    return RaiseError(config)
