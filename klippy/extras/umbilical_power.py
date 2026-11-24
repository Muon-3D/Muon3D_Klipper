# Muon M1 B1 toolhead umbilical power (TPS16630 eFuse on the SKBOB)
#
# Copyright (C) 2026  Muon3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# 48 V reaches the toolhead through the umbilical, switched by a TPS16630
# eFuse on the SKBOB (the knob board). Its pins go to the SKBOB's RP2040, not
# to a Klipper MCU: FLT# (GPIO6), MODE (GPIO7), SHDN#/EN (GPIO8), PGOOD
# (GPIO9) and IMON (GPIO27, ADC1). That RP2040 runs the knob and the round
# panel's menu, and sk_daemon talks to it on its UART. The firmware reports
# the eFuse there, and sk_daemon relays it to this module as JSON lines on a
# Unix socket (sk_daemon src/umbilical.rs):
#
#   EFT  every 100 ms: pg flt en mode i_avg i_min i_max (raw ADC counts),
#        edges, lost, guard (latch-guard pulses), guard_ok
#   EFE  at once on a PGOOD or FLT# edge: pin lvl imon pg flt en
#   EFA  the firmware's latch guard pulsed EN (FLT# and PGOOD low 20 ms)
#   ack  the answer to a command this module sent
#
# Each line carries `mono`, sk_daemon's CLOCK_MONOTONIC at receipt: the clock
# of the reactor, so it compares with eventtime directly.
#
# This module only reports, and sends events other modules can act on:
#
#   umbilical_power:power_lost  (eventtime, amps)  PGOOD high -> low
#   umbilical_power:power_good  (eventtime,)       PGOOD low -> high
#   umbilical_power:fault       (eventtime, amps)  FLT# high -> low
#   umbilical_power:guard       (eventtime,)       the latch guard pulsed EN
#
# It takes no action itself. The one control is UMBILICAL_POWER_CYCLE, an
# eFuse restart (EN held low for MS). sk_daemon accepts nothing else on the
# socket, so nothing here can reset the knob MCU: that would also restart
# the knob and the panel menu, which work without Klipper.
#
# A missing or silent daemon makes the status `stale`, never a fault: the
# toolhead may still have power, and the toolhead MCU reports its own loss.
import collections
import errno
import json
import logging
import socket

RECONNECT_INTERVAL = 2.0
# EFT comes every 0.1 s. Silence for this long makes the status stale.
STALE_TIME = 1.0
# The peak current in the status and the log covers this long.
PEAK_WINDOW = 2.0
# Longest wait for sk_daemon's ack to a command.
ACK_TIMEOUT = 1.0
CYCLE_MIN_MS = 10
CYCLE_MAX_MS = 2000
READ_SIZE = 4096
# A line longer than this is junk, not telemetry: drop the buffer.
MAX_LINE = 4096


class UmbilicalPower:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.reactor = printer.get_reactor()
        self.name = config.get_name()
        self.socket_path = config.get(
            'socket', '/home/printer_admin/comms/umbilical.sock')
        # IMON: 27.9 uA/A into the IMON resistor (rev 006: 16 kOhm, so
        # 2.24 A/V; rev 005: 10 kOhm, 3.58 A/V), on a 12-bit ADC.
        self.amps_per_count = (
            config.getfloat('adc_voltage', 3.3, above=0.)
            / config.getint('adc_max', 4095, minval=1)
            * config.getfloat('current_per_volt', 2.24, above=0.))
        # The reading with no load (about 52 counts on rev 006).
        self.zero_counts = config.getfloat('zero_counts', 0., minval=0.)
        self.stale_time = config.getfloat('stale_time', STALE_TIME, above=0.)
        self.sock = None
        self.fd_handle = None
        self.buf = b''
        self.connect_failed_logged = False
        self.ack_waiter = None
        self.reset_state()
        printer.register_event_handler('klippy:ready', self._handle_ready)
        printer.register_event_handler('klippy:disconnect',
                                       self._handle_disconnect)
        gcode = printer.lookup_object('gcode')
        gcode.register_command('UMBILICAL_POWER_STATUS',
                               self.cmd_UMBILICAL_POWER_STATUS,
                               desc=self.cmd_UMBILICAL_POWER_STATUS_help)
        gcode.register_command('UMBILICAL_POWER_CYCLE',
                               self.cmd_UMBILICAL_POWER_CYCLE,
                               desc=self.cmd_UMBILICAL_POWER_CYCLE_help)

    def reset_state(self):
        # None is "not reported yet": no edge is ever inferred from it
        self.power_good = None
        self.fault = None
        self.enabled = None
        self.auto_retry = None
        self.current = None
        self.peaks = collections.deque()
        self.last_report = None
        self.seq = None
        self.missed = 0
        self.guard_pulses = None
        self.power_lost_count = 0
        self.fault_count = 0
        self.last_power_lost = None
        self.last_fault = None
        self.last_fault_current = None

    # --- connection -------------------------------------------------------
    def _handle_ready(self):
        self.reactor.register_timer(self._connect_timer, self.reactor.NOW)

    def _handle_disconnect(self):
        self._close(log=False)

    def _connect_timer(self, eventtime):
        if self.sock is not None:
            return eventtime + RECONNECT_INTERVAL
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            sock.connect(self.socket_path)
        except (socket.error, OSError) as e:
            sock.close()
            if not self.connect_failed_logged:
                logging.info("umbilical_power: cannot connect to %s (%s);"
                             " retrying every %.0f s", self.socket_path, e,
                             RECONNECT_INTERVAL)
                self.connect_failed_logged = True
            return eventtime + RECONNECT_INTERVAL
        sock.setblocking(False)
        self.sock = sock
        self.buf = b''
        self.connect_failed_logged = False
        self.fd_handle = self.reactor.register_fd(sock.fileno(),
                                                  self._process_read)
        logging.info("umbilical_power: connected to %s", self.socket_path)
        return eventtime + RECONNECT_INTERVAL

    def _close(self, log=True):
        if self.fd_handle is not None:
            self.reactor.unregister_fd(self.fd_handle)
            self.fd_handle = None
        if self.sock is not None:
            try:
                self.sock.close()
            except (socket.error, OSError):
                pass
            self.sock = None
            if log:
                logging.info("umbilical_power: sk_daemon went away;"
                             " reconnecting")
        self._ack(None)

    def _process_read(self, eventtime):
        try:
            data = self.sock.recv(READ_SIZE)
        except (socket.error, OSError) as e:
            if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                return
            data = b''
        if not data:
            self._close()
            return
        self.feed(data, eventtime)

    def feed(self, data, eventtime):
        """Bytes from the socket: split into lines and handle each."""
        self.buf += data
        lines = self.buf.split(b'\n')
        self.buf = lines.pop()
        if len(self.buf) > MAX_LINE:
            logging.warning("umbilical_power: dropped %d bytes with no"
                            " newline", len(self.buf))
            self.buf = b''
        for line in lines:
            line = line.strip()
            if not line:
                continue
            try:
                msg = json.loads(line)
            except ValueError:
                logging.warning("umbilical_power: not JSON: %r", line[:200])
                continue
            if isinstance(msg, dict):
                self.handle_message(msg, eventtime)

    def _send(self, line):
        if self.sock is None:
            return False
        try:
            self.sock.sendall((line + '\n').encode())
        except (socket.error, OSError) as e:
            logging.info("umbilical_power: send failed: %s", e)
            self._close()
            return False
        return True

    # --- telemetry --------------------------------------------------------
    def amps(self, counts):
        if counts is None:
            return None
        return max(0., (counts - self.zero_counts) * self.amps_per_count)

    def handle_message(self, msg, eventtime):
        kind = msg.get('type')
        if kind == 'EFT':
            self._telemetry(msg, eventtime)
        elif kind == 'EFE':
            self._edge(msg, eventtime)
        elif kind == 'EFA':
            logging.info("umbilical_power: the SKBOB latch guard pulsed EN"
                         " (held %s ms, pulse %s ms, FLT# after %s, pulse %s"
                         " in a row)", msg.get('held_ms'),
                         msg.get('pulse_ms'), msg.get('flt_after'),
                         msg.get('streak'))
            self.printer.send_event('umbilical_power:guard', eventtime)
        elif kind == 'ack':
            self._ack(msg)

    def _telemetry(self, msg, eventtime):
        seq = msg.get('seq')
        if (isinstance(seq, int) and isinstance(self.seq, int)
                and seq > self.seq + 1):
            self.missed += seq - self.seq - 1
        self.seq = seq
        self.last_report = eventtime
        self.enabled = _flag(msg.get('en'))
        mode = msg.get('mode')
        self.auto_retry = None if mode is None else mode == 0
        avg = msg.get('i_avg')
        if isinstance(avg, int):
            self.current = self.amps(avg)
        peak = msg.get('i_max')
        if isinstance(peak, int):
            self.peaks.append((eventtime, self.amps(peak)))
        while self.peaks and self.peaks[0][0] < eventtime - PEAK_WINDOW:
            self.peaks.popleft()
        if isinstance(msg.get('guard'), int):
            self.guard_pulses = msg['guard']
        # EFE reports edges at once; EFT keeps the state right if one was
        # missed (a lost line, or a reconnect)
        self._set_pgood(_flag(msg.get('pg')), eventtime, self.current)
        self._set_fault(_fault(msg.get('flt')), eventtime, self.current)

    def _edge(self, msg, eventtime):
        imon = msg.get('imon')
        amps = self.amps(imon) if isinstance(imon, int) else self.current
        pin, lvl = msg.get('pin'), _flag(msg.get('lvl'))
        if pin == 'PG':
            self._set_pgood(lvl, eventtime, amps)
        elif pin == 'FLT':
            self._set_fault(None if lvl is None else not lvl, eventtime,
                            amps)

    def peak(self):
        if not self.peaks:
            return None
        return max(a for t, a in self.peaks)

    def _set_pgood(self, value, eventtime, amps):
        if value is None:
            return
        prev, self.power_good = self.power_good, value
        if prev is None or prev == value:
            return
        if value:
            logging.info("umbilical_power: PGOOD high, toolhead power back")
            self.printer.send_event('umbilical_power:power_good', eventtime)
            return
        self.power_lost_count += 1
        self.last_power_lost = eventtime
        logging.info("umbilical_power: PGOOD low, toolhead power lost"
                     " (current %s A, peak %s A over %.0f s, FLT# %s, EN %s)",
                     _fmt(amps), _fmt(self.peak()), PEAK_WINDOW,
                     _level(self.fault, invert=True), _level(self.enabled))
        self.printer.send_event('umbilical_power:power_lost', eventtime,
                                amps)

    def _set_fault(self, value, eventtime, amps):
        if value is None:
            return
        prev, self.fault = self.fault, value
        if prev is None or prev == value or not value:
            return
        self.fault_count += 1
        self.last_fault = eventtime
        self.last_fault_current = amps
        logging.info("umbilical_power: FLT# low, the umbilical eFuse"
                     " reports a fault (current %s A, peak %s A). With"
                     " PGOOD still high this is a current limit; with"
                     " PGOOD low and little current, OVP or UVLO",
                     _fmt(amps), _fmt(self.peak()))
        self.printer.send_event('umbilical_power:fault', eventtime, amps)

    def is_stale(self, eventtime):
        return (self.last_report is None
                or eventtime - self.last_report > self.stale_time)

    # --- commands ---------------------------------------------------------
    def _ack(self, msg):
        waiter, self.ack_waiter = self.ack_waiter, None
        if waiter is not None and not waiter.test():
            waiter.complete(msg)

    def power_cycle(self, ms):
        """Asks sk_daemon for an eFuse restart. -> (ok, detail)"""
        if self.sock is None:
            return False, "sk_daemon is not connected"
        if self.ack_waiter is not None:
            return False, "a power cycle is already in progress"
        waiter = self.ack_waiter = self.reactor.completion()
        if not self._send('POWER_CYCLE %d' % (ms,)):
            self.ack_waiter = None
            return False, "send to sk_daemon failed"
        ack = waiter.wait(self.reactor.monotonic() + ACK_TIMEOUT)
        if self.ack_waiter is waiter:
            self.ack_waiter = None
        if ack is None:
            return False, "no answer from sk_daemon"
        return bool(ack.get('ok')), str(ack.get('detail', ''))

    def _printing(self, eventtime):
        stats = self.printer.lookup_object('print_stats', None)
        if stats is None:
            return False
        return stats.get_status(eventtime).get('state') in ('printing',
                                                            'paused')

    cmd_UMBILICAL_POWER_STATUS_help = (
        "Report the toolhead umbilical eFuse state and current")

    def cmd_UMBILICAL_POWER_STATUS(self, gcmd):
        eventtime = self.reactor.monotonic()
        st = self.get_status(eventtime)
        if not st['connected']:
            gcmd.respond_info("umbilical_power: sk_daemon not connected (%s)"
                              % (self.socket_path,))
            return
        gcmd.respond_info(
            "umbilical_power: %s, PGOOD %s, FLT# %s, EN %s, current %s A"
            " (peak %s A), power lost %d, faults %d, latch guard pulses %s%s"
            % (st['state'], _level(st['power_good']),
               _level(st['fault'], invert=True), _level(st['enabled']),
               _fmt(st['current']), _fmt(st['current_peak']),
               st['power_lost_count'], st['fault_count'],
               st['guard_pulses'],
               ", STALE" if st['stale'] else ""))

    cmd_UMBILICAL_POWER_CYCLE_help = (
        "Restart the toolhead umbilical eFuse (EN low for MS); never resets"
        " the knob MCU")

    def cmd_UMBILICAL_POWER_CYCLE(self, gcmd):
        ms = gcmd.get_int('MS', 100, minval=CYCLE_MIN_MS, maxval=CYCLE_MAX_MS)
        force = gcmd.get_int('FORCE', 0, minval=0, maxval=1)
        if self._printing(self.reactor.monotonic()) and not force:
            raise gcmd.error("UMBILICAL_POWER_CYCLE cuts the toolhead power;"
                             " refused while a print is running or paused"
                             " (FORCE=1 to do it anyway)")
        ok, detail = self.power_cycle(ms)
        if not ok:
            raise gcmd.error("UMBILICAL_POWER_CYCLE failed: %s" % (detail,))
        logging.info("umbilical_power: power cycle %d ms (%s)", ms, detail)
        gcmd.respond_info("umbilical_power: eFuse EN pulsed low for %d ms"
                          % (ms,))

    def get_status(self, eventtime):
        connected = self.sock is not None
        stale = self.is_stale(eventtime)
        if not connected or stale:
            state = 'unknown'
        elif self.power_good:
            state = 'fault' if self.fault else 'on'
        elif self.enabled is False:
            state = 'off'
        else:
            state = 'lost'
        return {'state': state,
                'connected': connected,
                'stale': stale,
                'power_good': self.power_good,
                'fault': self.fault,
                'enabled': self.enabled,
                'auto_retry': self.auto_retry,
                'current': _round(self.current),
                'current_peak': _round(self.peak()),
                'power_lost_count': self.power_lost_count,
                'fault_count': self.fault_count,
                'guard_pulses': self.guard_pulses,
                'last_power_lost': self.last_power_lost,
                'last_fault': self.last_fault,
                'last_fault_current': _round(self.last_fault_current),
                'missed_reports': self.missed}


def _flag(v):
    if v in (0, 1):
        return bool(v)
    return None


def _fault(flt):
    # FLT# is active low
    v = _flag(flt)
    return None if v is None else not v


def _round(v):
    return None if v is None else round(v, 3)


def _fmt(v):
    return "?" if v is None else "%.2f" % (v,)


def _level(v, invert=False):
    if v is None:
        return "?"
    return "low" if bool(v) == invert else "high"


def load_config(config):
    return UmbilicalPower(config)
