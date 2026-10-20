# Persistent, fail-closed hot-end fan safety latch.
#
# Copyright (C) 2026 Muon 3D
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, os, stat, tempfile


class HotendFanLatch:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.filename = os.path.expanduser(config.get('state_file'))
        self.heater_name = config.get('heater', 'extruder')
        self.sensor_name = config.get(
            'heatsink_sensor', 'temperature_fan toolhead_heatsink')
        self.max_clear_temp = config.getfloat('max_clear_temp', 51.)
        self.min_clear_temp = config.getfloat('min_clear_temp', -10.)
        # D-018 is deliberately one option.  The shipped/default policy is
        # fail-closed; sites may opt into an unlatched first boot explicitly.
        self.missing_file_latched = config.getboolean(
            'missing_file_latched', True)
        self.latched = self._read_state()
        self.heater = self._set_temp = self._alter_target = None
        self.printer.register_event_handler('klippy:connect', self._connect)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('HOTEND_FAN_LATCH_SET', self.cmd_latch)
        gcode.register_command('HOTEND_FAN_LATCH_CONFIRM', self.cmd_confirm)

    def _read_state(self):
        try:
            with open(self.filename) as f:
                value = f.read().strip()
            if value == 'latched':
                return True
            if value == 'unlatched':
                return False
            raise ValueError('invalid latch state')
        except FileNotFoundError:
            logging.warning('Hot-end fan latch file is missing; %s',
                            'latched' if self.missing_file_latched
                            else 'using configured first-boot exemption')
            return self.missing_file_latched
        except Exception:
            logging.exception('Unable to read hot-end fan latch; latching')
            return True

    def _write_state(self):
        # Resolve a symlink and replace its target, not the link itself.
        path = os.path.realpath(self.filename)
        directory = os.path.dirname(os.path.abspath(path))
        old_stat = None
        try:
            old_stat = os.stat(path)
        except FileNotFoundError:
            pass
        tmpname = None
        try:
            fd, tmpname = tempfile.mkstemp(prefix='.hotend_fan_latch.',
                                           dir=directory)
            with os.fdopen(fd, 'w') as f:
                f.write('latched\n' if self.latched else 'unlatched\n')
                if old_stat is not None:
                    os.fchmod(f.fileno(), stat.S_IMODE(old_stat.st_mode))
                    os.fchown(f.fileno(), old_stat.st_uid, old_stat.st_gid)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmpname, path)
            tmpname = None
            if os.name == "posix":  # directory fsync is a POSIX notion
                dirfd = os.open(directory, os.O_RDONLY)
                try:
                    os.fsync(dirfd)
                finally:
                    os.close(dirfd)
        except Exception:
            if tmpname is not None:
                try:
                    os.unlink(tmpname)
                except OSError:
                    pass
            logging.exception('Unable to persist hot-end fan latch')
            raise self.printer.command_error(
                'Unable to persist hot-end fan latch')

    def _connect(self):
        extruder = self.printer.lookup_object(self.heater_name)
        self.heater = getattr(extruder, 'heater', extruder)
        self._set_temp = self.heater.set_temp
        self.heater.set_temp = self._guarded_set_temp
        if hasattr(self.heater, 'alter_target'):
            self._alter_target = self.heater.alter_target
            self.heater.alter_target = self._guarded_alter_target
        if self.latched:
            self._force_off()

    def _force_off(self):
        if self.heater is not None:
            self._set_temp(0.)

    def _refuse(self):
        self._force_off()
        raise self.printer.command_error(
            'Heating refused: hot-end fan fault is latched. Check the fan, '
            'then run HOTEND_FAN_FAULT_CLEAR (Fan checked).')

    def _guarded_set_temp(self, degrees):
        if degrees > 0. and self.latched:
            self._refuse()
        return self._set_temp(degrees)

    def _guarded_alter_target(self, degrees):
        if degrees > 0. and self.latched:
            self._refuse()
        return self._alter_target(degrees)

    def cmd_latch(self, gcmd):
        self.latched = True
        self._force_off()
        self._write_state()

    def cmd_confirm(self, gcmd):
        sensor = self.printer.lookup_object(self.sensor_name)
        status = sensor.get_status(self.printer.get_reactor().monotonic())
        temp = float(status.get('temperature', 0.))
        if temp >= self.max_clear_temp:
            raise gcmd.error('Fan fault not cleared: heatsink is still at '
                             '%.1fC (%.0fC or more).' %
                             (temp, self.max_clear_temp))
        if temp <= self.min_clear_temp:
            raise gcmd.error('Fan fault not cleared: heatsink sensor reads '
                             'an implausible %.1fC.' % (temp,))
        # Persist before opening the in-memory gate.  A failed write therefore
        # leaves both the current process and the next startup fail-closed.
        previous = self.latched
        self.latched = False
        try:
            self._write_state()
        except Exception:
            self.latched = previous
            raise

    def get_status(self, eventtime):
        return {'latched': self.latched, 'restored': True}


def load_config(config):
    return HotendFanLatch(config)
