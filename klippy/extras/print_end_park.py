# Best-effort M1 end-of-print parking. Cleanup remains in PRINT_END.
#
# This file may be distributed under the terms of the GNU GPLv3 license.


class PrintEndPark:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        macro = self.printer.load_object(config, 'gcode_macro')
        self.template = macro.load_template(config, 'gcode')
        self.gcode.register_command('_PRINT_END_PARK', self.cmd_PRINT_END_PARK)

    def cmd_PRINT_END_PARK(self, gcmd):
        try:
            self.template.run_gcode_from_command()
        except self.printer.command_error as err:
            # Stop the park sequence on refusal, including CUT_FILAMENT.
            # Do not bypass the contact guard or catch unexpected errors.
            gcmd.respond_info('PRINT_END: parking skipped: %s; '
                              'cleanup will continue' % (err,))


def load_config(config):
    return PrintEndPark(config)
