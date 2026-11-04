"""Persistent, fail-closed hot-end fan safety latch.

This protects against crashes, power loss, and operator mistakes.  It is not a
security boundary against a malicious local user with shell access.  In that
threat model, the realpath lookup followed later by replace still has a TOCTOU
window; a shell user able to change the path can also change the state files.
"""
#
# Copyright (C) 2026 Muon 3D
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, os, stat, tempfile


class HotendFanLatch:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.filename = os.path.expanduser(config.get('state_file'))
        self.marker_filename = self.filename + '.trip'
        self.heater_name = config.get('heater', 'extruder')
        self.sensor_name = config.get(
            'heatsink_sensor', 'temperature_fan toolhead_heatsink')
        self.max_clear_temp = config.getfloat('max_clear_temp', 51.)
        self.min_clear_temp = config.getfloat('min_clear_temp', -10.)
        # KAN-992: the clear needs a heatsink sample at most this old.
        self.max_reading_age = config.getfloat('max_reading_age', 5.,
                                               above=0.)
        self._corrupt_reason = None
        self.latched = self._read_state()
        self._latched_write_pending = self.latched
        self.heater = self._set_temp = self._alter_target = None
        self.printer.register_event_handler('klippy:connect', self._connect)
        self.printer.register_event_handler('klippy:ready',
                                            self._announce_corrupt)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('HOTEND_FAN_LATCH_SET', self.cmd_latch)
        # Private implementation hook used only by HOTEND_FAN_FAULT_CLEAR.
        gcode.register_command('_HOTEND_FAN_LATCH_CONFIRM', self.cmd_confirm)
        reactor = self.printer.get_reactor()
        self._retry_timer = reactor.register_timer(
            self._retry_pending, reactor.monotonic() + 5.)

    def _exists(self, path):
        # True/False only when the answer is certain.  Raises on any error
        # other than "no such file", so an unreadable directory is never
        # mistaken for an absent file.  A dangling symlink is not absent.
        try:
            os.lstat(path)
            return True
        except FileNotFoundError:
            return False

    def _read_state(self):
        # D-018 (Jack, 20 Oct 2026): an ABSENT state file means no fault
        # (fresh flash, or never written) and heating is allowed.  A file that
        # exists but cannot be read or parsed (permissions, empty, garbage,
        # a directory, a dangling symlink) is corrupt and fails closed.  A
        # trip marker always wins: a trip writes it before anything else.
        try:
            # The persisted calibration directory must exist, or a later trip
            # could not be written and a restart would silently clear it.  If
            # it cannot be made, that is corrupt (latched), never "absent".
            try:
                os.makedirs(os.path.dirname(os.path.abspath(self.filename)),
                            mode=0o755, exist_ok=True)
            except Exception:
                self._corrupt_reason = ('state directory for %s cannot be '
                                        'created' % (self.filename,))
                raise
            if self._exists(self.marker_filename):
                return True
            if not self._exists(self.filename):
                logging.info('Hot-end fan latch file is absent; no fault')
                return False
            with open(self.filename) as f:
                value = f.read().strip()
            if value == 'latched':
                return True
            if value == 'unlatched':
                return False
            raise ValueError('invalid latch state %r' % (value,))
        except Exception:
            logging.exception('Hot-end fan latch file %s is unreadable or '
                              'corrupt; latching', self.filename)
            if self._corrupt_reason is None:
                self._corrupt_reason = ('state file %s is corrupt or '
                                        'unreadable' % (self.filename,))
            return True

    def _fsync_directory(self, directory):
        if os.name == "posix":  # directory fsync is a POSIX notion
            dirfd = os.open(directory, os.O_RDONLY)
            try:
                os.fsync(dirfd)
            finally:
                os.close(dirfd)

    def _create_trip_marker(self):
        path = os.path.realpath(self.marker_filename)
        directory = os.path.dirname(os.path.abspath(path))
        try:
            fd = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
        except FileExistsError:
            return
        try:
            os.write(fd, b'latched\n')
            os.fsync(fd)
        finally:
            os.close(fd)
        self._fsync_directory(directory)

    def _write_state(self, latched=None):
        if latched is None:
            latched = self.latched
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
                f.write('latched\n' if latched else 'unlatched\n')
                if old_stat is not None:
                    # fchmod and fchown are POSIX only; the printer runs
                    # Linux.
                    if hasattr(os, 'fchmod'):
                        os.fchmod(f.fileno(), stat.S_IMODE(old_stat.st_mode))
                    if hasattr(os, 'fchown'):
                        os.fchown(f.fileno(), old_stat.st_uid,
                                  old_stat.st_gid)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmpname, path)
            tmpname = None
            self._fsync_directory(directory)
            with open(path) as f:
                written = f.read().strip()
            expected = 'latched' if latched else 'unlatched'
            if written != expected:
                raise IOError('hot-end fan latch read-back did not match')
        except Exception:
            if tmpname is not None:
                try:
                    os.unlink(tmpname)
                except OSError:
                    pass
            logging.exception('Unable to persist hot-end fan latch')
            raise self.printer.command_error(
                'Unable to persist hot-end fan latch')

    def _retry_pending(self, eventtime):
        if self.latched and self._latched_write_pending:
            try:
                self._create_trip_marker()
                self._write_state(True)
                self._latched_write_pending = False
            except Exception:
                pass
        return eventtime + 5.

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

    def _announce_corrupt(self):
        if self._corrupt_reason is None:
            return
        try:
            self.printer.lookup_object('gcode').respond_info(
                'Hot-end fan fault latched: %s. Check the fan, then press '
                'Fan checked to clear it.' % (self._corrupt_reason,))
        except Exception:
            logging.exception('Unable to announce hot-end fan latch')

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
        self._latched_write_pending = True
        self._force_off()
        # The create-only marker must precede replacement of an old unlatched
        # state, so interruption at every later point remains fail-closed.
        # The latch is already closed in memory, so a persistence failure must
        # not stop the caller showing the Fan checked prompt: report it loudly
        # instead of raising, and keep retrying from the timer.
        failures = []
        for step in (self._create_trip_marker,
                     lambda: self._write_state(True)):
            try:
                step()
            except Exception as e:
                logging.exception('Unable to persist hot-end fan latch')
                failures.append(str(e) or e.__class__.__name__)
        if failures:
            gcmd.respond_info(
                '!! WARNING: the hot-end fan fault is latched in memory but '
                'could NOT be saved (%s). A restart before it saves may lose '
                'the fault. Check the fan.' % ('; '.join(failures),))
        else:
            self._latched_write_pending = False

    def _heatsink_temp(self, gcmd):
        # KAN-992: until the sensor first reports, temperature_fan says
        # 0.0C, which is inside the clear window.  A clear right after a
        # restart passed on that placeholder.  Only a real, recent sample
        # may show the heatsink is cool, so every way of not having one
        # refuses: no sample yet, a stale one, a sensor that cannot say
        # when it last read, or a status with no temperature in it.
        sensor = self.printer.lookup_object(self.sensor_name)
        eventtime = self.printer.get_reactor().monotonic()
        read_time = getattr(sensor, 'last_temp_time', None)
        if read_time is None:
            raise gcmd.error('Fan fault not cleared: %s does not report '
                             'when it last read.' % (self.sensor_name,))
        if read_time <= 0.:
            raise gcmd.error('Fan fault not cleared: heatsink temperature '
                             'not available yet. Try again in a few '
                             'seconds.')
        mcu = self.printer.lookup_object('mcu')
        age = mcu.estimated_print_time(eventtime) - read_time
        if age > self.max_reading_age:
            raise gcmd.error('Fan fault not cleared: the heatsink sensor '
                             'last reported %.0fs ago.' % (age,))
        temp = sensor.get_status(eventtime).get('temperature')
        if temp is None:
            raise gcmd.error('Fan fault not cleared: heatsink temperature '
                             'not available.')
        return float(temp)

    def cmd_confirm(self, gcmd):
        temp = self._heatsink_temp(gcmd)
        if temp >= self.max_clear_temp:
            raise gcmd.error('Fan fault not cleared: heatsink is still at '
                             '%.1fC (%.0fC or more).' %
                             (temp, self.max_clear_temp))
        if temp <= self.min_clear_temp:
            raise gcmd.error('Fan fault not cleared: heatsink sensor reads '
                             'an implausible %.1fC.' % (temp,))
        # Persist before opening the in-memory gate.  A failed write therefore
        # leaves both the current process and the next startup fail-closed.
        # Confirm the durable latched state on any clear/retry opportunity.
        if self._latched_write_pending:
            self._create_trip_marker()
            self._write_state(True)
            self._latched_write_pending = False
        self._write_state(False)
        marker = os.path.realpath(self.marker_filename)
        try:
            os.unlink(marker)
            self._fsync_directory(os.path.dirname(os.path.abspath(marker)))
        except FileNotFoundError:
            pass
        except Exception:
            # Keep the live gate closed if marker removal was not durable.
            self.latched = True
            self._latched_write_pending = True
            try:
                self._create_trip_marker()
            except Exception:
                pass
            raise self.printer.command_error(
                'Unable to clear persistent hot-end fan latch')
        self.latched = False

    def get_status(self, eventtime):
        return {'latched': self.latched}


def load_config(config):
    return HotendFanLatch(config)
