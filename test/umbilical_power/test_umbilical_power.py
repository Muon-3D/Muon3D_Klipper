# Tests for klippy/extras/umbilical_power.py: the sk_daemon line handling,
# the edge events, staleness and the eFuse power cycle, against stubs and a
# real Unix socket pair.
import json
import os
import socket
import sys
import tempfile
import unittest

KLIPPY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

from extras import umbilical_power  # noqa: E402


class TestNoDeadTests(unittest.TestCase):
    """CI runs this file as a script, so unittest.main() only sees the
    classes defined above it (KAN-3421). Keep this class first."""
    def test_main_guard_is_the_last_statement(self):
        import ast
        with open(os.path.abspath(__file__)) as f:
            body = ast.parse(f.read()).body
        guards = [i for i, node in enumerate(body) if isinstance(node, ast.If)
                  and ast.unparse(node.test) == "__name__ == '__main__'"]
        self.assertEqual(len(guards), 1)
        below = [getattr(node, 'name', type(node).__name__)
                 for node in body[guards[0] + 1:]]
        self.assertEqual(below, [])


class CommandError(Exception):
    pass


class StubConfig:
    def __init__(self, printer, values=None):
        self.printer = printer
        self.values = values or {}
    def get_printer(self):
        return self.printer
    def get_name(self):
        return 'umbilical_power'
    def get(self, key, default=None):
        return self.values.get(key, default)
    def getfloat(self, key, default=None, **kw):
        return float(self.values.get(key, default))
    def getint(self, key, default=None, **kw):
        return int(self.values.get(key, default))


class StubGcode:
    def __init__(self):
        self.commands = {}
    def register_command(self, name, func, desc=None):
        self.commands[name] = func


class StubCompletion:
    """Runs `on_wait` (the daemon's answer) when waited on."""
    def __init__(self, reactor):
        self.reactor = reactor
        self.result = None
        self.done = False
    def test(self):
        return self.done
    def complete(self, result):
        self.done = True
        self.result = result
    def wait(self, waketime=None):
        if self.reactor.on_wait is not None:
            self.reactor.on_wait()
        return self.result


class StubReactor:
    NOW = 0.
    NEVER = 9e99
    def __init__(self):
        self.now = 100.
        self.timers = []
        self.fds = {}
        self.on_wait = None
    def monotonic(self):
        return self.now
    def register_timer(self, cb, waketime):
        self.timers.append(cb)
    def register_fd(self, fd, cb):
        self.fds[fd] = cb
        return fd
    def unregister_fd(self, handle):
        del self.fds[handle]
    def completion(self):
        return StubCompletion(self)


class StubPrintStats:
    def __init__(self):
        self.state = 'standby'
    def get_status(self, eventtime):
        return {'state': self.state}


class StubPrinter:
    command_error = CommandError
    def __init__(self):
        self.reactor = StubReactor()
        self.gcode = StubGcode()
        self.objects = {'gcode': self.gcode}
        self.events = []
        self.handlers = {}
    def get_reactor(self):
        return self.reactor
    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)
    def register_event_handler(self, name, cb):
        self.handlers[name] = cb
    def send_event(self, name, *args):
        self.events.append((name,) + args)


class StubGcmd:
    error = CommandError
    def __init__(self, params=None):
        self.params = params or {}
        self.out = []
    def get_int(self, name, default=None, **kw):
        return int(self.params.get(name, default))
    def respond_info(self, msg):
        self.out.append(msg)


def eft(**kw):
    msg = {'type': 'EFT', 'seq': 1, 'pg': 1, 'flt': 1, 'en': 1, 'mode': 0,
           'i_avg': 52, 'i_min': 50, 'i_max': 55, 'guard': 0}
    msg.update(kw)
    return msg


class Base(unittest.TestCase):
    def make(self, **values):
        self.printer = StubPrinter()
        values.setdefault('zero_counts', 52)
        self.up = umbilical_power.load_config(
            StubConfig(self.printer, values))
        return self.up

    def events(self, name=None):
        return [e for e in self.printer.events
                if name is None or e[0] == name]


class TestCurrent(Base):
    def test_counts_become_amps_after_the_zero_offset(self):
        up = self.make()
        self.assertEqual(up.amps(52), 0.)
        self.assertEqual(up.amps(40), 0.)
        # rev 006: 3.3 V / 4095 * 2.24 A/V a count
        self.assertAlmostEqual(up.amps(52 + 1000), 1000 * 3.3 / 4095 * 2.24)

    def test_the_rev_005_resistor_is_a_config_value(self):
        up = self.make(current_per_volt=3.58, zero_counts=0)
        self.assertAlmostEqual(up.amps(4095), 3.3 * 3.58)

    def test_status_carries_mean_and_peak_current(self):
        up = self.make()
        up.handle_message(eft(i_avg=52 + 500, i_max=52 + 2000), 100.)
        st = up.get_status(100.)
        self.assertAlmostEqual(st['current'], round(up.amps(552), 3))
        self.assertAlmostEqual(st['current_peak'], round(up.amps(2052), 3))
        # the peak covers PEAK_WINDOW, then falls back to the newer reports
        up.handle_message(eft(seq=2, i_max=60), 100.1)
        up.handle_message(eft(seq=3, i_max=60), 103.)
        self.assertAlmostEqual(up.get_status(103.)['current_peak'],
                               round(up.amps(60), 3))


class TestEdges(Base):
    def test_the_first_report_sends_no_event(self):
        up = self.make()
        up.handle_message(eft(pg=0, flt=0), 100.)
        self.assertEqual(self.events(), [])
        self.assertFalse(up.power_good)
        self.assertTrue(up.fault)

    def test_a_pgood_edge_sends_power_lost_once_with_its_current(self):
        up = self.make()
        up.sock = object()
        up.handle_message(eft(), 100.)
        up.handle_message({'type': 'EFE', 'pin': 'PG', 'lvl': 0,
                           'imon': 52 + 100}, 100.05)
        # the next EFT agrees and must not send it again
        up.handle_message(eft(seq=2, pg=0), 100.1)
        lost = self.events('umbilical_power:power_lost')
        self.assertEqual(len(lost), 1)
        self.assertEqual(lost[0][1], 100.05)
        self.assertAlmostEqual(lost[0][2], up.amps(152))
        st = up.get_status(100.1)
        self.assertEqual(st['state'], 'lost')
        self.assertEqual(st['power_lost_count'], 1)
        self.assertEqual(st['last_power_lost'], 100.05)
        up.handle_message(eft(seq=3, pg=1), 100.2)
        self.assertEqual(len(self.events('umbilical_power:power_good')), 1)

    def test_a_missed_edge_is_caught_by_the_next_report(self):
        up = self.make()
        up.handle_message(eft(), 100.)
        up.handle_message(eft(seq=2, pg=0), 100.1)
        self.assertEqual(len(self.events('umbilical_power:power_lost')), 1)

    def test_flt_low_is_a_fault_event(self):
        up = self.make()
        up.handle_message(eft(), 100.)
        up.handle_message({'type': 'EFE', 'pin': 'FLT', 'lvl': 0,
                           'imon': 52}, 100.01)
        up.handle_message({'type': 'EFE', 'pin': 'FLT', 'lvl': 1,
                           'imon': 52}, 100.02)
        faults = self.events('umbilical_power:fault')
        self.assertEqual(len(faults), 1)
        self.assertEqual(up.get_status(100.02)['fault_count'], 1)
        self.assertEqual(up.get_status(100.02)['last_fault'], 100.01)

    def test_the_latch_guard_is_an_event(self):
        up = self.make()
        up.handle_message({'type': 'EFA', 'held_ms': 20, 'pulse_ms': 20,
                           'flt_after': 1, 'streak': 1}, 100.)
        self.assertEqual(self.events(), [('umbilical_power:guard', 100.)])

    def test_switched_off_is_off_not_lost(self):
        up = self.make()
        up.sock = object()
        up.handle_message(eft(pg=0, en=0), 100.)
        self.assertEqual(up.get_status(100.)['state'], 'off')


class TestStale(Base):
    def test_silence_is_stale_and_unknown_never_a_fault(self):
        up = self.make()
        up.sock = object()
        up.handle_message(eft(), 100.)
        self.assertEqual(up.get_status(100.5)['state'], 'on')
        st = up.get_status(101.5)
        self.assertTrue(st['stale'])
        self.assertEqual(st['state'], 'unknown')
        self.assertEqual(self.events(), [])

    def test_missed_reports_are_counted(self):
        up = self.make()
        up.handle_message(eft(seq=1), 100.)
        up.handle_message(eft(seq=5), 100.4)
        self.assertEqual(up.get_status(100.4)['missed_reports'], 3)


class TestFeed(Base):
    def test_lines_split_across_reads_and_junk_are_handled(self):
        up = self.make()
        line = json.dumps(eft(pg=1)).encode() + b'\n'
        up.feed(b'not json\n' + line[:10], 100.)
        self.assertIsNone(up.power_good)
        up.feed(line[10:] + json.dumps(eft(seq=2, pg=0)).encode()[:5], 100.1)
        self.assertTrue(up.power_good)
        self.assertEqual(self.events(), [])

    def test_an_endless_line_is_dropped(self):
        up = self.make()
        up.feed(b'x' * (umbilical_power.MAX_LINE + 1), 100.)
        self.assertEqual(up.buf, b'')


class TestSocket(Base):
    """A real AF_UNIX listener standing in for sk_daemon."""
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.path = os.path.join(self.dir, 'umb.sock')
        self.server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.server.bind(self.path)
        self.server.listen(1)
        self.peer = None

    def tearDown(self):
        if self.peer is not None:
            self.peer.close()
        self.server.close()
        os.unlink(self.path)
        os.rmdir(self.dir)

    def connect(self):
        up = self.make(socket=self.path)
        self.printer.handlers['klippy:ready']()
        self.printer.reactor.timers[0](100.)
        self.peer, _ = self.server.accept()
        self.assertIsNotNone(up.sock)
        return up

    def pump(self, up):
        """What the reactor does when the socket is readable. A local
        AF_UNIX send is in the receive buffer when sendall returns."""
        for cb in list(self.printer.reactor.fds.values()):
            cb(self.printer.reactor.now)

    def test_telemetry_arrives_over_the_socket(self):
        up = self.connect()
        self.peer.sendall(json.dumps(eft(pg=1)).encode() + b'\n')
        self.pump(up)
        self.assertTrue(up.power_good)

    def test_a_closed_daemon_is_a_reconnect_not_a_fault(self):
        up = self.connect()
        self.peer.close()
        self.peer = None
        self.pump(up)
        self.assertIsNone(up.sock)
        self.assertEqual(self.printer.reactor.fds, {})
        self.assertEqual(up.get_status(100.)['state'], 'unknown')
        self.assertEqual(self.events(), [])

    def answer(self, ok=True, detail='EFEN 200'):
        """Reads the command on the daemon side and acks it."""
        def on_wait():
            self.peer.settimeout(1.)
            self.sent = self.peer.recv(100)
            self.up.feed(json.dumps({'type': 'ack', 'ok': ok,
                                     'detail': detail}).encode() + b'\n',
                         100.)
        self.printer.reactor.on_wait = on_wait

    def test_power_cycle_sends_only_the_efuse_command(self):
        self.up = self.connect()
        self.answer()
        gcmd = StubGcmd({'MS': 200})
        self.printer.gcode.commands['UMBILICAL_POWER_CYCLE'](gcmd)
        self.assertEqual(self.sent, b'POWER_CYCLE 200\n')
        self.assertIn('200 ms', gcmd.out[-1])

    def test_a_refusal_from_the_daemon_is_an_error(self):
        self.up = self.connect()
        self.answer(ok=False, detail='rate limited, 3.0 s to go')
        with self.assertRaises(CommandError) as cm:
            self.printer.gcode.commands['UMBILICAL_POWER_CYCLE'](StubGcmd())
        self.assertIn('rate limited', str(cm.exception))

    def test_power_cycle_is_refused_while_printing_unless_forced(self):
        self.up = self.connect()
        stats = self.printer.objects['print_stats'] = StubPrintStats()
        stats.state = 'printing'
        self.answer()
        with self.assertRaises(CommandError):
            self.printer.gcode.commands['UMBILICAL_POWER_CYCLE'](StubGcmd())
        self.printer.gcode.commands['UMBILICAL_POWER_CYCLE'](
            StubGcmd({'FORCE': 1}))
        self.assertEqual(self.sent, b'POWER_CYCLE 100\n')

    def test_no_daemon_no_power_cycle(self):
        up = self.make(socket=self.path)
        with self.assertRaises(CommandError) as cm:
            self.printer.gcode.commands['UMBILICAL_POWER_CYCLE'](StubGcmd())
        self.assertIn('not connected', str(cm.exception))
        self.assertIsNone(up.ack_waiter)


class TestConfig(unittest.TestCase):
    def test_b1_core_config_has_the_section(self):
        path = os.path.join(KLIPPY, '..', 'core', 'M1', 'core.cfg')
        section, values = False, {}
        for line in open(path):
            line = line.split('#')[0].strip()
            if line.startswith('['):
                section = line == '[umbilical_power]'
                continue
            if section and ':' in line:
                key, value = line.split(':', 1)
                values[key.strip()] = value.strip()
        self.assertTrue(values, "no [umbilical_power] in core/M1/core.cfg")
        self.assertEqual(values.get('socket'),
                         '/home/printer_admin/comms/umbilical.sock')
        self.assertAlmostEqual(float(values['current_per_volt']), 2.24)


if __name__ == '__main__':
    unittest.main()
