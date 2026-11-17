# Best-effort M1 end-of-print parking. Cleanup remains in PRINT_END.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging


class PrintEndPark:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        macro = self.printer.load_object(config, 'gcode_macro')
        self.template = macro.load_template(config, 'gcode')
        # What to do when the park above was refused (KAN-3813): park as far
        # as is safe from whatever state the refusal left.
        self.safe_template = macro.load_template(config, 'safe_gcode', '')
        # Whether the last _PRINT_END_PARK was refused, and why.
        self.failed = False
        self.error = ''
        self.gcode.register_command('_PRINT_END_PARK', self.cmd_PRINT_END_PARK)
        self.gcode.register_command('_PRINT_END_PARK_SAFE',
                                    self.cmd_PRINT_END_PARK_SAFE)

    def get_status(self, eventtime=None):
        return {'failed': self.failed, 'error': self.error}

    def cmd_PRINT_END_PARK(self, gcmd):
        self.failed = False
        self.error = ''
        try:
            self.template.run_gcode_from_command()
        except self.printer.command_error as err:
            # Stop the park sequence on refusal, including CUT_FILAMENT.
            # Do not bypass the contact guard or catch unexpected errors.
            self.failed = True
            self.error = str(err)
            gcmd.respond_info('PRINT_END: parking skipped: %s; '
                              'cleanup will continue' % (err,))

    def cmd_PRINT_END_PARK_SAFE(self, gcmd):
        # Run after _PRINT_END_PARK. Nothing to do if the park finished.
        # Otherwise park what is safe, then say what failed: after the
        # park, and as an error line ("!!") so that it does not read as a
        # clean end. It is a message, not a raised error: raising here
        # would end PRINT_END (and a CANCEL_PRINT around it) before its
        # cleanup, or fail a print that finished.
        if not self.failed:
            return
        try:
            self.safe_template.run_gcode_from_command()
        except self.printer.command_error as err:
            gcmd.respond_info('PRINT_END: the safe park was refused too: '
                              '%s; cleanup will continue' % (err,))
        # The macro's own words: "Error evaluating 'gcode_macro X:gcode':
        # gcode.CommandError: <text>" is down to <text>.
        reason = self.error.rsplit('CommandError: ', 1)[-1].strip()
        msg = ('PRINT_END: the end-of-print park stopped early: %s'
               ' The head was parked as far as was safe and the motors'
               ' are off. Check the printer before the next print.'
               % (reason,))
        logging.warning(msg)
        gcmd.respond_raw('!! %s' % (msg,))


def load_config(config):
    return PrintEndPark(config)
