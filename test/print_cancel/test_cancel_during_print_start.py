#!/usr/bin/env python3
"""Cancelling a print while PRINT_START is still running.

The print file runs PRINT_START as one line, and virtual_sdcard holds the
gcode mutex for the whole of it, so the CANCEL_PRINT that the pause_resume
webhook queues behind it used to wait for the macro to finish.  These tests
drive klippy's real reactor, gcode dispatcher, gcode_macro, virtual_sdcard,
print_stats and pause_resume, with the cancel sent from its own greenlet the
way Moonraker's webhook request arrives, and the real M1 CANCEL_PRINT macro
read from core/M1/macros/print.cfg.

Needs Linux (klippy's util imports pty/fcntl) and greenlet + jinja2, which
klippy's own requirements provide:

    python3 test/print_cancel/test_cancel_during_print_start.py
"""
import os
import shutil
import sys
import tempfile
import time
import types
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'klippy'))

# reactor.py only takes monotonic() from chelper; stub it so the test does
# not compile klippy's C helper into the source tree.
_chelper = types.ModuleType('chelper')
_chelper.get_ffi = lambda: (None, types.SimpleNamespace(
    get_monotonic=time.monotonic))
sys.modules.setdefault('chelper', _chelper)

import reactor  # noqa: E402
import gcode  # noqa: E402
from extras import gcode_macro, heaters, pause_resume  # noqa: E402
from extras import print_stats, virtual_sdcard  # noqa: E402

M1_PRINT_CFG = os.path.join(ROOT, 'core', 'M1', 'macros', 'print.cfg')
SENTINEL = object()


def read_macro_section(path, section):
    """The options of one [section] of a Klipper config file.  Enough of the
    format for a gcode_macro: 'key: value' lines and an indented gcode block,
    with full-line comments dropped as Klipper's parser drops them."""
    with open(path, 'r') as f:
        lines = f.read().splitlines()
    start = lines.index('[%s]' % (section,)) + 1
    opts = {}
    key = None
    for line in lines[start:]:
        if line.startswith('['):
            break
        stripped = line.strip()
        if not stripped or stripped.startswith('#'):
            continue
        if line[0] in ' \t' and key is not None:
            opts[key] += '\n' + stripped
            continue
        key, _, value = line.partition(':')
        key = key.strip()
        opts[key] = value.strip()
    return opts


class Config:
    error = Exception

    def __init__(self, printer, name, opts=None):
        self.printer = printer
        self.name = name
        self.opts = dict(opts or {})

    def get_printer(self):
        return self.printer

    def get_name(self):
        return self.name

    def get(self, option, default=SENTINEL):
        if option in self.opts:
            return self.opts[option]
        if default is SENTINEL:
            raise self.error("missing option %s in %s" % (option, self.name))
        return default

    def getint(self, option, default=SENTINEL, minval=None, maxval=None):
        return int(self.get(option, default))

    def getfloat(self, option, default=SENTINEL, minval=None, maxval=None,
                 above=None, below=None):
        return float(self.get(option, default))

    def getboolean(self, option, default=SENTINEL):
        val = self.get(option, default)
        if isinstance(val, str):
            return val.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(val)

    def get_prefix_options(self, prefix):
        return [o for o in self.opts if o.startswith(prefix)]


class Printer:
    command_error = gcode.CommandError

    def __init__(self, react):
        self.reactor = react
        self.objects = {}
        self.handlers = {}
        self.shutdown_msgs = []

    def get_reactor(self):
        return self.reactor

    def get_start_args(self):
        return {}

    def lookup_object(self, name, default=SENTINEL):
        if name in self.objects:
            return self.objects[name]
        if default is SENTINEL:
            raise Exception("unknown object %s" % (name,))
        return default

    def lookup_objects(self, module=None):
        return list(self.objects.items())

    def load_object(self, config, name):
        return self.objects[name]

    def register_event_handler(self, event, callback):
        self.handlers.setdefault(event, []).append(callback)

    def send_event(self, event, *params):
        return [cb(*params) for cb in self.handlers.get(event, [])]

    def is_shutdown(self):
        return bool(self.shutdown_msgs)

    def invoke_shutdown(self, msg):
        self.shutdown_msgs.append(msg)


class Webhooks:
    def __init__(self):
        self.endpoints = {}

    def register_endpoint(self, path, callback):
        self.endpoints[path] = callback


class GCodeMove:
    def get_status(self, eventtime=None):
        return {'position': gcode.Coord((0., 0., 0., 0.)),
                'extrude_factor': 1.}


class ConfigFile:
    def __init__(self):
        self.save_config_pending = False

    def get_status(self, eventtime):
        return {'save_config_pending': self.save_config_pending,
                'settings': {}}


class Toolhead:
    def get_last_move_time(self):
        return 0.


class ColdSensor:
    # A heater that never gets there: TEMPERATURE_WAIT on it waits forever
    def get_temp(self, eventtime):
        return 20., 0.


class Harness:
    """klippy's reactor and gcode stack with just enough printer around it.

    Records every test command in self.log, every console line in
    self.output."""

    def __init__(self, macros):
        self.reactor = reactor.Reactor()
        self.printer = p = Printer(self.reactor)
        self.log = []
        self.output = []
        self.gcode = gcode.GCodeDispatch(p)
        p.objects['gcode'] = self.gcode
        self.gcode.register_output_handler(self.output.append)
        p.objects['webhooks'] = self.webhooks = Webhooks()
        p.objects['gcode_move'] = GCodeMove()
        p.objects['toolhead'] = Toolhead()
        p.objects['configfile'] = self.configfile = ConfigFile()
        p.objects['cold_sensor'] = ColdSensor()
        p.objects['gcode_macro'] = gcode_macro.load_config(
            Config(p, 'gcode_macro'))
        p.objects['print_stats'] = self.print_stats = print_stats.PrintStats(
            Config(p, 'print_stats'))
        self.sddir = tempfile.mkdtemp(prefix='print_cancel_')
        p.objects['virtual_sdcard'] = self.vsd = virtual_sdcard.VirtualSD(
            Config(p, 'virtual_sdcard', {'path': self.sddir}))
        p.objects['pause_resume'] = self.pause_resume = \
            pause_resume.PauseResume(Config(p, 'pause_resume'))
        self._register_test_commands()
        # The real M1 CANCEL_PRINT, plus the scenario's own macros
        cancel = read_macro_section(M1_PRINT_CFG, 'gcode_macro CANCEL_PRINT')
        self._add_macro('CANCEL_PRINT', cancel)
        for name, opts in macros.items():
            self._add_macro(name, opts)
        p.send_event('klippy:connect')
        p.send_event('klippy:ready')

    def _add_macro(self, name, opts):
        section = 'gcode_macro %s' % (name,)
        cfg = Config(self.printer, section, opts)
        self.printer.objects[section] = gcode_macro.load_config_prefix(cfg)

    def _register_test_commands(self):
        g = self.gcode
        g.register_command('STEP', self._cmd_STEP)
        g.register_command('SLOW', self._cmd_SLOW)
        g.register_command('PYCMD', self._cmd_PYCMD)
        g.register_command('PRINT_END', self._cmd_PRINT_END)
        g.register_command('_SAVE_CONFIG_IF_PENDING', self._cmd_SAVE)
        g.register_command('G1', lambda gcmd: self.log.append('G1'))
        # The real TEMPERATURE_WAIT, on a PrinterHeaters with no heaters
        ph = object.__new__(heaters.PrinterHeaters)
        ph.printer = self.printer
        ph.heaters = {}
        ph._get_temp = lambda eventtime: 'T:20.0 /0.0'
        g.register_command('TEMPERATURE_WAIT', ph.cmd_TEMPERATURE_WAIT)

    def _cmd_STEP(self, gcmd):
        self.log.append('step:' + gcmd.get('NAME'))

    def _cmd_SLOW(self, gcmd):
        # A python command that takes a while, like G28 or a mesh
        self.log.append('slow:start')
        self.reactor.pause(self.reactor.monotonic() + 0.5)
        self.log.append('slow:end')

    def _cmd_PYCMD(self, gcmd):
        # A python command that runs gcode of its own, like SHAPER_CALIBRATE
        # restoring SET_VELOCITY_LIMIT: all of it must run
        self.gcode.run_script_from_command('SLOW\nSTEP NAME=py_cleanup')

    def _cmd_PRINT_END(self, gcmd):
        self.log.append('PRINT_END')

    def _cmd_SAVE(self, gcmd):
        self.log.append('SAVE_CONFIG')
        self.configfile.save_config_pending = False

    # Driving
    def run(self, file_lines, cancels=(), timers=(), timeout=15.):
        """Print a file made of file_lines.  cancels: seconds after the
        start at which to send the pause_resume/cancel webhook.  timers:
        (seconds, script) run from a timer with run_script_from_command,
        outside the gcode mutex, as some modules do."""
        with open(os.path.join(self.sddir, 't.gcode'), 'w') as f:
            f.write('\n'.join(file_lines) + '\n')
        self.cancel_returned = []
        self.error = None

        def send_cancel(eventtime):
            sent = self.reactor.monotonic()
            self.webhooks.endpoints['pause_resume/cancel'](None)
            self.cancel_returned.append(self.reactor.monotonic() - sent)

        def run_timer_script(script):
            def cb(eventtime):
                self.gcode.run_script_from_command(script)
            return cb

        def main(eventtime):
            try:
                self.gcode.run_script('SDCARD_PRINT_FILE FILENAME=t.gcode')
                start = self.reactor.monotonic()
                for delay in cancels:
                    self.reactor.register_callback(send_cancel, start + delay)
                for delay, script in timers:
                    self.reactor.register_callback(
                        run_timer_script(script), start + delay)
                deadline = start + timeout
                while self.reactor.monotonic() < deadline:
                    self.reactor.pause(self.reactor.monotonic() + 0.02)
                    idle = self.vsd.work_timer is None
                    if idle and len(self.cancel_returned) == len(cancels):
                        break
                else:
                    self.error = 'timed out'
            except Exception as e:
                self.error = repr(e)
            finally:
                self.reactor.end()

        self.reactor.register_callback(main)
        self.reactor.run()
        self.reactor.finalize()
        shutil.rmtree(self.sddir, ignore_errors=True)

    def errors(self):
        return [line for line in self.output if line.startswith('!!')]


PRINT_START = {
    'abortable': 'True',
    'gcode': '\n'.join([
        'STEP NAME=a',
        'SLOW',
        'STEP NAME=b',
        'SLOW',
        'STEP NAME=c',
    ]),
}


class CancelDuringPrintStart(unittest.TestCase):
    def test_cancel_stops_at_the_next_line_of_an_abortable_macro(self):
        h = Harness({'PRINT_START': PRINT_START})
        h.run(['PRINT_START', 'G1'], cancels=[0.2])
        self.assertIsNone(h.error)
        # The command in flight finishes; nothing after it runs
        self.assertEqual(h.log, ['step:a', 'slow:start', 'slow:end',
                                 'PRINT_END'])
        self.assertEqual(h.print_stats.state, 'cancelled')
        self.assertIsNone(h.vsd.current_file)
        self.assertFalse(h.vsd.is_cancel_pending())
        # Returned once SLOW did (~0.3 s later), not after the whole macro
        self.assertLess(h.cancel_returned[0], 0.6)
        self.assertEqual(h.errors(), [])
        self.assertEqual(h.printer.shutdown_msgs, [])

    def test_before_the_fix_the_cancel_waited_for_the_whole_macro(self):
        # The same macro, not declared abortable: the pre-fix behaviour, and
        # still the behaviour of every macro that does not opt in
        start = dict(PRINT_START)
        del start['abortable']
        h = Harness({'PRINT_START': start})
        h.run(['PRINT_START', 'G1'], cancels=[0.2])
        self.assertIsNone(h.error)
        self.assertEqual(h.log, ['step:a', 'slow:start', 'slow:end',
                                 'step:b', 'slow:start', 'slow:end',
                                 'step:c', 'PRINT_END'])
        self.assertEqual(h.print_stats.state, 'cancelled')
        self.assertGreater(h.cancel_returned[0], 0.6)

    def test_cancel_interrupts_a_temperature_wait(self):
        h = Harness({'PRINT_START': {
            'abortable': 'True',
            'gcode': 'STEP NAME=a\n'
                     'TEMPERATURE_WAIT SENSOR=cold_sensor MINIMUM=100\n'
                     'STEP NAME=b',
        }})
        h.run(['PRINT_START'], cancels=[0.3])
        self.assertIsNone(h.error)
        self.assertEqual(h.log, ['step:a', 'PRINT_END'])
        self.assertEqual(h.print_stats.state, 'cancelled')
        # TEMPERATURE_WAIT polls once a second
        self.assertLess(h.cancel_returned[0], 1.5)
        self.assertEqual(h.errors(), [])

    def test_an_ordinary_macro_called_from_print_start_runs_whole(self):
        # The M1's G28 fakes Z homed and clears it three lines later: a
        # macro that has not opted in must never be split
        h = Harness({
            'PRINT_START': {'abortable': 'True',
                            'gcode': 'INNER\nSTEP NAME=after_inner'},
            'INNER': {'gcode': 'STEP NAME=i1\nSLOW\nSTEP NAME=i2'},
        })
        h.run(['PRINT_START'], cancels=[0.2])
        self.assertIsNone(h.error)
        self.assertEqual(h.log, ['step:i1', 'slow:start', 'slow:end',
                                 'step:i2', 'PRINT_END'])
        self.assertEqual(h.print_stats.state, 'cancelled')

    def test_a_python_commands_own_gcode_runs_whole(self):
        h = Harness({'PRINT_START': {'abortable': 'True',
                                     'gcode': 'PYCMD\nSTEP NAME=after'}})
        h.run(['PRINT_START'], cancels=[0.2])
        self.assertIsNone(h.error)
        self.assertEqual(h.log, ['slow:start', 'slow:end', 'step:py_cleanup',
                                 'PRINT_END'])

    def test_an_abortable_macro_inside_an_ordinary_one_runs_whole(self):
        h = Harness({
            'PRINT_START': {'abortable': 'True', 'gcode': 'OUTER\nSTEP NAME=z'},
            'OUTER': {'gcode': 'INNER\nSTEP NAME=o2'},
            'INNER': {'abortable': 'True',
                      'gcode': 'SLOW\nSTEP NAME=i2'},
        })
        h.run(['PRINT_START'], cancels=[0.2])
        self.assertEqual(h.log, ['slow:start', 'slow:end', 'step:i2',
                                 'step:o2', 'PRINT_END'])

    def test_other_greenlets_are_not_aborted(self):
        # A timer running gcode while the abort is pending is not the print
        h = Harness({'PRINT_START': PRINT_START})
        h.run(['PRINT_START'], cancels=[0.2],
              timers=[(0.25, 'STEP NAME=timer')])
        self.assertIsNone(h.error)
        self.assertIn('step:timer', h.log)
        self.assertNotIn('step:b', h.log)
        self.assertEqual(h.errors(), [])

    def test_pressing_cancel_twice_runs_print_end_once(self):
        h = Harness({'PRINT_START': PRINT_START})
        h.run(['PRINT_START'], cancels=[0.2, 0.25])
        self.assertIsNone(h.error)
        self.assertEqual(h.log.count('PRINT_END'), 1)
        self.assertEqual(h.print_stats.state, 'cancelled')
        self.assertTrue(any('no print in progress' in line
                            for line in h.output), h.output)
        self.assertEqual(h.errors(), [])

    def test_cancel_writes_staged_calibration_before_print_end(self):
        h = Harness({'PRINT_START': PRINT_START})
        h.configfile.save_config_pending = True
        h.run(['PRINT_START'], cancels=[0.2])
        self.assertEqual(h.log[-2:], ['SAVE_CONFIG', 'PRINT_END'])

    def test_cancel_with_nothing_staged_does_not_save(self):
        h = Harness({'PRINT_START': PRINT_START})
        h.run(['PRINT_START'], cancels=[0.2])
        self.assertNotIn('SAVE_CONFIG', h.log)

    def test_cancel_never_reports_paused(self):
        # The line stops with the file still open; that must not surface as
        # a pause on its way to cancelled
        h = Harness({'PRINT_START': PRINT_START})
        seen = []
        orig = h.print_stats.note_pause
        h.print_stats.note_pause = lambda: (seen.append('pause'), orig())
        h.run(['PRINT_START'], cancels=[0.2])
        self.assertEqual(seen, [])
        self.assertEqual(h.print_stats.state, 'cancelled')

    def test_cancel_between_file_lines(self):
        # The plain case: nothing in flight that can be aborted
        h = Harness({})
        h.run(['STEP NAME=a', 'SLOW', 'STEP NAME=b', 'SLOW', 'STEP NAME=c'],
              cancels=[0.2])
        self.assertIsNone(h.error)
        self.assertNotIn('step:c', h.log)
        self.assertEqual(h.log[-1], 'PRINT_END')
        self.assertEqual(h.print_stats.state, 'cancelled')
        self.assertEqual(h.errors(), [])

    def test_a_print_nobody_cancels_is_unchanged(self):
        h = Harness({'PRINT_START': PRINT_START})
        h.run(['PRINT_START', 'G1'])
        self.assertIsNone(h.error)
        self.assertEqual(h.log, ['step:a', 'slow:start', 'slow:end',
                                 'step:b', 'slow:start', 'slow:end',
                                 'step:c', 'G1'])
        self.assertEqual(h.print_stats.state, 'complete')
        self.assertEqual(h.errors(), [])

    def test_a_real_error_is_still_an_error(self):
        h = Harness({'PRINT_START': {'abortable': 'True',
                                     'gcode': 'STEP NAME=a\nNO_SUCH_CMD\n'
                                              'STEP NAME=b'}})
        h.gcode.register_command(
            'NO_SUCH_CMD', lambda gcmd: (_ for _ in ()).throw(
                gcmd.error('boom')))
        h.run(['PRINT_START'])
        self.assertEqual(h.print_stats.state, 'error')
        self.assertIn('!! boom', h.output)


if __name__ == '__main__':
    unittest.main(verbosity=2)
