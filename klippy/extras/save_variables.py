# Save arbitrary variables so that values can be kept across restarts.
#
# Copyright (C) 2020 Dushyant Ahuja <dusht.ahuja@gmail.com>
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, logging, ast, configparser, tempfile

FAN_FAULT_VARIABLE = 'hotend_fan_fault'

class SaveVariables:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.filename = os.path.expanduser(config.get('filename'))
        self.allVariables = {}
        try:
            self.loadVariables()
        except self.printer.command_error as e:
            raise config.error(str(e))
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('SAVE_VARIABLE', self.cmd_SAVE_VARIABLE,
                               desc=self.cmd_SAVE_VARIABLE_help)
    def loadVariables(self):
        allvars = {}
        varfile = configparser.ConfigParser()
        try:
            with open(self.filename) as f:
                varfile.read_file(f)
            if varfile.has_section('Variables'):
                for name, val in varfile.items('Variables'):
                    allvars[name] = ast.literal_eval(val)
            fault = allvars.get(FAN_FAULT_VARIABLE)
            if type(fault) is not int or fault not in (0, 1):
                allvars[FAN_FAULT_VARIABLE] = 1
        except FileNotFoundError:
            allvars = {FAN_FAULT_VARIABLE: 1}
        except Exception:
            # This safety latch is the one variable that must never default
            # open.  Keep other save_variables users operational, but require
            # an explicit fan check after a missing, unreadable, or corrupt
            # file rather than silently treating it as "no fault".
            logging.exception("Unable to read existing variable file")
            allvars = {FAN_FAULT_VARIABLE: 1}
        self.allVariables = allvars
    cmd_SAVE_VARIABLE_help = "Save arbitrary variables to disk"
    def cmd_SAVE_VARIABLE(self, gcmd):
        varname = gcmd.get('VARIABLE')
        if (varname.lower() != varname):
            raise gcmd.error("VARIABLE must not contain upper case")
        value = gcmd.get('VALUE')
        try:
            value = ast.literal_eval(value)
        except ValueError as e:
            raise gcmd.error("Unable to parse '%s' as a literal" % (value,))
        newvars = dict(self.allVariables)
        newvars[varname] = value
        # Write file
        varfile = configparser.ConfigParser()
        varfile.add_section('Variables')
        for name, val in sorted(newvars.items()):
            varfile.set('Variables', name, repr(val))
        tmpname = None
        try:
            directory = os.path.dirname(os.path.abspath(self.filename))
            with tempfile.NamedTemporaryFile(
                    mode='w', dir=directory, prefix='.save_variables.',
                    delete=False) as f:
                tmpname = f.name
                varfile.write(f)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmpname, self.filename)
            tmpname = None
        except Exception:
            if tmpname is not None:
                try:
                    os.unlink(tmpname)
                except OSError:
                    pass
            msg = "Unable to save variable"
            logging.exception(msg)
            raise gcmd.error(msg)
        self.loadVariables()
    def get_status(self, eventtime):
        return {'variables': self.allVariables}

def load_config(config):
    return SaveVariables(config)
