#!/usr/bin/env python3
"""The merge resolutions of the M1 print-start landing (KAN-2245), pinned.

land/m1-print-start-on-master is feat/m1-print-start merged with master.
Six places needed a hand resolution (the Opus demo-risk review of 16 Nov
2026); a later bad merge or rebase could undo any of them without a test
failing. One test class per resolution:

  B1  print.cfg PRINT_START: the start gate first, then master's KAN-1730
      refusal of an invalid saved z_offset, and the hand-over applies the
      probe's LIVE offset, never the saved one in configfile.
  B2  contact_guard: the connect looks up BOTH bed_removal_detector and
      bed_mesh, and the Z stepper state callbacks, so _may_refuse never
      meets a missing attribute.
  B3  gcode.py + virtual_sdcard.py: a print file's line runs through
      run_client_script(line, 'file', abortable=True): the muon_guard check
      for each line AND the cancel abort, and the file loop treats the abort
      as "stop here", not as an error.
  B4  homing.cfg: both motors home at 0.6 A (KAN-3813), both put back.
  B5  toolhead.cfg / core.cfg: nothing calls FORCE_MOVE (it is off); the
      umbilical unlock retract is COLD_RETRACT DISTANCE=16, the limit of
      [cold_retract].
  B6  muon_guard's allowlist: test/muon_guard/test_muon_guard.py (not here).
  KAN-3387: the start gate shows no button-less macro prompt (the Z power
      fault prompt ending on a reset is tested in test/m1_machine).

Needs greenlet and jinja2 (klippy's own). Offline, no hardware.

    python3 test/landing/test_landing_resolutions.py
"""
import ast
import io
import os
import re
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
sys.path.insert(0, os.path.join(ROOT, 'test', 'contact_guard'))

# These put klippy on sys.path and bring klippy's own GCodeDispatch and
# GCodeMacro, as the M1 macro tests do.
import test_m1_heater_before_leds as th  # noqa: E402
from test_m1_heater_before_leds import _M1TestCase  # noqa: E402
from test_m1_led_modes import Status  # noqa: E402
from test_m1_purge_line import RECORDED  # noqa: E402
import gcode  # noqa: E402
import jinja2  # noqa: E402

MACROS = os.path.join(ROOT, 'core', 'M1', 'macros')
CORE = os.path.join(ROOT, 'core', 'M1', 'core.cfg')


# --- reading the shipped config ------------------------------------------
def read_cfg(path):
    """[(section, [raw lines])] of a Klipper config file."""
    out = []
    with io.open(path, 'r', encoding='utf-8', newline=None) as f:
        for line in f.read().split('\n'):
            m = re.match(r'^\[([^\]]+)\]', line)
            if m:
                out.append((m.group(1).strip(), []))
            elif out:
                out[-1][1].append(line)
    return out


def options(lines):
    """The 'key: value' options of a section: top-level, not indented, and
    the indented lines under a key joined by newlines. Whole-line comments
    go, as Klipper's parser drops them."""
    opts = {}
    key = None
    for line in lines:
        stripped = line.strip()
        if not stripped or stripped.startswith(('#', ';')):
            continue
        if line[0] in ' \t' and key is not None:
            opts[key] += '\n' + stripped
            continue
        m = re.match(r'^([^:=\s]+)\s*[:=]\s*(.*)$', line)
        if m:
            key = m.group(1).strip()
            opts[key] = m.group(2).strip()
    return opts


def section(path, name):
    for sect, lines in read_cfg(path):
        if sect == name:
            return options(lines)
    raise KeyError("%s: no [%s]" % (path, name))


def macro_lines(path, name):
    """The gcode of a macro as command lines: comments and blanks gone."""
    text = section(path, 'gcode_macro ' + name)['gcode']
    out = []
    for line in text.split('\n'):
        line = line.strip()
        if line and not line.startswith(('#', ';')):
            out.append(line)
    return out


def cfg_files():
    out = [CORE]
    for name in sorted(os.listdir(MACROS)):
        if name.endswith('.cfg'):
            out.append(os.path.join(MACROS, name))
    return out


def command_lines(path):
    """Every non-comment line of a cfg file, with its section."""
    for sect, lines in read_cfg(path):
        for line in lines:
            s = line.strip()
            if s and not s.startswith(('#', ';')):
                yield sect, s


def render(path, name, printer=None, params=None):
    """Render a macro's gcode as klippy's GCodeMacro would (its own
    variable_* options are the template's variables)."""
    opts = section(path, 'gcode_macro ' + name)
    ctx = {'printer': printer or {}, 'params': params or {}}
    for key, value in opts.items():
        if key.startswith('variable_'):
            ctx[key[len('variable_'):]] = ast.literal_eval(value)
    text = '\n'.join(l.strip() for l in opts['gcode'].split('\n'))
    # klippy's gcode_macro uses single braces for expressions
    env = jinja2.Environment('{%', '%}', '{', '}')
    return [l.strip() for l in env.from_string(text).render(**ctx)
            .split('\n') if l.strip()]


# --- B1 -------------------------------------------------------------------
START = 'PRINT_START BED=65 EXTRUDER=215 MATERIAL=PLA'


class B1PrintStartGate(_M1TestCase):
    """scripts/test_m1_print_start_z_offset.py already covers: a healthy
    probe's live offset is applied (with a different value saved in
    configfile), an invalid one refuses in dev mode, the raw saved value is
    never applied, and the override must be explicit. What it does not pin,
    and this does: the gate is written before the refusal and the refusal
    before any move, and the hand-over text reads the live offset only."""

    def setUp(self):
        super().setUp()
        for name in RECORDED:
            self.m1.gcode.register_command(name, None)
            self.m1.gcode.register_command(name, self.m1._record)
        self.status = {'z_offset': 0.0, 'z_offset_invalid': False,
                       'z_offset_saved': None}
        self.m1.printer.add_object('load_cell_probe', Status(self.status))
        settings = self.m1.printer.lookup_object('configfile') \
            .get_status(0)['settings']
        settings['load_cell_probe']['z_offset'] = -5.0
        self.expected_error = False

    def tearDown(self):
        if self.expected_error:
            self.m1.responses[:] = [r for r in self.m1.responses
                                    if not r.startswith('!!')]
            self.log.records.clear()
        self.m1.responses[:] = [r for r in self.m1.responses
                                if 'PURGE_LINE: no purge lane' not in r]
        super().tearDown()

    def test_an_invalid_offset_refuses_before_anything_runs(self):
        # The macro's template renders in full before its first line runs,
        # so the refusal (written after the gate in the text) is raised at
        # render time: not even the gate runs, nothing heats or moves, and
        # the operator is told at once rather than after a wait for the
        # plate. That is the behaviour pinned here; the text order, which
        # is what a merge can silently swap, is test_macro_text_order.
        self.status.update({'z_offset': 0.0, 'z_offset_invalid': True,
                            'z_offset_saved': -5.0})
        self.m1.timeline.clear()
        self.expected_error = True
        with self.assertRaises(gcode.CommandError) as ctx:
            self.m1.run(START)
        self.assertIn('Saved Z offset -5.000 mm', str(ctx.exception))
        self.assertEqual(self.m1.names(), [])

    def test_a_valid_offset_runs_the_gate_first(self):
        self.m1.timeline.clear()
        self.m1.run(START)
        names = self.m1.names()
        self.assertIn('MACHINE_WAIT_READY', names)
        heat = [i for i, n in enumerate(names) if n in ('M140', 'G28', 'G0')]
        self.assertTrue(heat, names)
        self.assertLess(names.index('MACHINE_WAIT_READY'), heat[0])

    def test_macro_text_order(self):
        lines = macro_lines(os.path.join(MACROS, 'print.cfg'), 'PRINT_START')
        gate = [i for i, l in enumerate(lines)
                if l.startswith('MACHINE_WAIT_READY')]
        refuse = [i for i, l in enumerate(lines) if 'z_offset_invalid' in l]
        raise_ = [i for i, l in enumerate(lines)
                  if l.startswith('{action_raise_error(')
                  and 'Saved Z offset' in l]
        first_move = [i for i, l in enumerate(lines)
                      if re.match(r'^(G28|G0|G1|M104|M140)\b', l)][0]
        self.assertEqual(len(gate), 1, lines)
        self.assertEqual(len(refuse), 1, lines)
        self.assertEqual(len(raise_), 1, lines)
        self.assertLess(gate[0], refuse[0])
        self.assertLess(refuse[0], raise_[0])
        self.assertLess(raise_[0], first_move)
        # the refusal can only be passed with IGNORE_Z_OFFSET
        self.assertIn('IGNORE_Z_OFFSET', lines[refuse[0] + 1])

    def test_hand_over_reads_the_live_offset_only(self):
        lines = macro_lines(os.path.join(MACROS, 'print.cfg'), 'PRINT_START')
        reads = [l for l in lines if 'load_cell_probe' in l
                 and 'z_offset_invalid' not in l and 'z_offset_saved' not in l]
        self.assertEqual(len(reads), 1, reads)
        self.assertIn('printer.load_cell_probe.z_offset|', reads[0])
        for l in lines:
            self.assertNotRegex(l, r'settings\.load_cell_probe',
                                'PRINT_START reads a saved z_offset')
        sets = [l for l in lines if l.startswith('SET_GCODE_OFFSET')]
        self.assertEqual(sets, ['SET_GCODE_OFFSET Z={zoff}'])

    def test_live_offset_wins_over_a_different_saved_one(self):
        self.status['z_offset'] = 0.35
        self.m1.printer.lookup_object('configfile').get_status(0)[
            'settings']['load_cell_probe']['z_offset'] = 1.9
        self.m1.timeline.clear()
        self.m1.run(START)
        self.assertEqual([l for l in self.m1.timeline
                          if l.startswith('SET_GCODE_OFFSET')],
                         ['SET_GCODE_OFFSET Z=0.35'])


# --- B2 -------------------------------------------------------------------
class B2ContactGuardConnect(unittest.TestCase):
    def build(self, detector=True, mesh=True):
        import test_contact_guard as tcg
        from extras import contact_guard
        self.tcg = tcg
        printer = tcg.FakePrinter()
        self.det = type('Det', (), {})()
        self.det.present = True
        self.det.plate_present = lambda: self.det.present
        if detector:
            printer.objects['bed_removal_detector'] = self.det
        self.mesh = object()
        if mesh:
            printer.objects['bed_mesh'] = self.mesh
        guard = contact_guard.load_config(tcg.FakeConfig(
            printer, {'move_guard': 'load_cell'}))
        printer.send_event('klippy:connect')
        printer.send_event('klippy:ready')
        return printer, guard

    def test_both_lookups_and_the_z_callbacks_are_set_at_connect(self):
        printer, guard = self.build()
        self.assertIs(guard.bed_detector, self.det)
        self.assertIs(guard.bed_mesh, self.mesh)
        self.assertEqual(list(guard.z_steppers), ['stepper_z'])
        line = printer.objects['stepper_enable'].lines['stepper_z']
        self.assertEqual(line.callbacks, [guard._handle_z_motor])
        # X and Y have no callback: only the Z motors can be turned by hand
        self.assertEqual(sorted(printer.objects['stepper_enable'].lines),
                         ['stepper_z'])

    def test_may_refuse_with_the_plate_present_and_absent(self):
        printer, guard = self.build()
        now = printer.reactor.monotonic()
        self.det.present = True
        self.assertTrue(guard._may_refuse(now))
        self.det.present = False
        self.assertFalse(guard._may_refuse(now))
        self.det.present = None      # startup: unknown keeps the guard on
        self.assertTrue(guard._may_refuse(now))

    def test_missing_detector_and_mesh_do_not_raise(self):
        printer, guard = self.build(detector=False, mesh=False)
        self.assertIsNone(guard.bed_detector)
        self.assertIsNone(guard.bed_mesh)
        self.assertTrue(guard._may_refuse(printer.reactor.monotonic()))

    def test_constructed_but_not_connected_has_no_detector(self):
        import test_contact_guard as tcg
        from extras import contact_guard
        printer = tcg.FakePrinter()
        guard = contact_guard.load_config(tcg.FakeConfig(
            printer, {'move_guard': 'load_cell'}))
        self.assertIsNone(guard.bed_detector)
        self.assertIsNone(guard.bed_mesh)
        # a probe set but no connect yet: the detector read must not raise
        guard.probe = object()
        guard._may_refuse(printer.reactor.monotonic())


# --- B3 -------------------------------------------------------------------
class Reactor:
    NOW = 0.
    NEVER = 9999999999999999.

    def __init__(self):
        self.now = 0.

    def monotonic(self):
        return self.now

    def pause(self, waketime):
        self.now = max(self.now, waketime)
        return self.now

    def mutex(self):
        class Mutex:
            locked = False

            def test(self):
                return self.locked

            def __enter__(self):
                self.locked = True

            def __exit__(self, *exc):
                self.locked = False
        return Mutex()

    def unregister_timer(self, timer):
        pass


class Printer:
    config_error = Exception
    command_error = gcode.CommandError

    def __init__(self):
        self.reactor = Reactor()
        self.shutdowns = []
        self.handlers = {}

    def get_reactor(self):
        return self.reactor

    def get_state_message(self):
        return ("Printer is ready", "ready")

    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)

    def send_event(self, event, *args):
        return [cb(*args) for cb in self.handlers.get(event, [])]

    def invoke_shutdown(self, msg):
        self.shutdowns.append(msg)

    def get_start_args(self):
        return {}


class SDStats:
    def __init__(self):
        self.calls = []

    def note_start(self):
        self.calls.append('start')

    def note_pause(self):
        self.calls.append('pause')

    def note_complete(self):
        self.calls.append('complete')

    def note_error(self, msg):
        self.calls.append('error: ' + msg)

    def note_cancel(self):
        self.calls.append('cancel')


class OnError:
    def __init__(self, sd):
        self.sd = sd

    def render(self):
        return 'ON_ERROR'


class B3FileLines(unittest.TestCase):
    def setUp(self):
        from extras import virtual_sdcard
        self.vs_mod = virtual_sdcard
        self.printer = Printer()
        self.gcode = gcode.GCodeDispatch(self.printer)
        self.ran = []
        self.gate_calls = []
        self.out = []
        self.gcode.register_output_handler(self.out.append)
        for name in ('ONE', 'TWO', 'THREE', 'ON_ERROR'):
            self.gcode.register_command(name, self._record(name))
        self.gcode.register_command('BAD', self._record('BAD'))
        self.gcode.register_command('FAIL', self._fail)
        self.gcode.register_command('ABORT_NOW', self._abort_now)
        self.gcode.register_command('REQUEST_ABORT', self._request_abort)
        self.gcode.register_client_gate(self._gate)
        self.printer.send_event('klippy:ready')

    def _record(self, name):
        def cmd(gcmd):
            self.ran.append(name)
        return cmd

    def _gate(self, origin, gcmd):
        self.gate_calls.append((origin, gcmd.get_command()))
        if gcmd.get_command() == 'BAD':
            raise self.gcode.error("Refused: BAD is not allowed")

    def _fail(self, gcmd):
        self.ran.append('FAIL')
        raise gcmd.error("a command failed")

    def _request_abort(self, gcmd):
        self.ran.append('REQUEST_ABORT')
        self.gcode.request_abort("Print cancelled")

    def _abort_now(self, gcmd):
        # What TEMPERATURE_WAIT or NOZZLE_WIPE_SMART does inside a line:
        # a cancel arrives while it waits, and its next check_abort() ends it
        self.ran.append('ABORT_NOW')
        self.sd.request_cancel()
        self.gcode.check_abort()
        self.ran.append('ABORT_NOW survived')

    # -- the gcode dispatcher, the real one
    def test_each_line_is_checked_as_a_file_line(self):
        with self.assertRaises(gcode.CommandError) as e:
            self.gcode.run_client_script('ONE\nBAD\nTWO', 'file',
                                         abortable=True)
        self.assertNotIsInstance(e.exception, gcode.CommandAbort)
        self.assertIn('BAD is not allowed', str(e.exception))
        self.assertEqual(self.gate_calls, [('file', 'ONE'), ('file', 'BAD')])
        self.assertEqual(self.ran, ['ONE'])

    def test_a_requested_abort_raises_the_abort_error(self):
        with self.assertRaises(self.gcode.abort_error) as e:
            self.gcode.run_client_script('ONE\nREQUEST_ABORT\nTWO\nTHREE',
                                         'file', abortable=True)
        self.assertIs(self.gcode.abort_error, gcode.CommandAbort)
        self.assertEqual(str(e.exception), 'Print cancelled')
        self.assertEqual(self.ran, ['ONE', 'REQUEST_ABORT'])
        # and the checks ran for the lines before it, not after
        self.assertEqual([c for _, c in self.gate_calls],
                         ['ONE', 'REQUEST_ABORT'])

    def test_without_abortable_nothing_can_be_aborted(self):
        self.gcode.run_client_script('ONE\nREQUEST_ABORT\nTWO', 'file')
        self.assertEqual(self.ran, ['ONE', 'REQUEST_ABORT', 'TWO'])
        self.assertEqual([o for o, _ in self.gate_calls], ['file'] * 3)

    # -- virtual_sdcard's file loop, the real one
    def make_sd(self, text):
        vs = self.vs_mod.VirtualSD.__new__(self.vs_mod.VirtualSD)
        self.sd = vs
        vs.printer = self.printer
        vs.reactor = self.printer.reactor
        vs.gcode = self.gcode
        vs.print_stats = self.stats = SDStats()
        vs.on_error_gcode = OnError(vs)
        vs.current_file = io.StringIO(text)
        vs.file_position = 0
        vs.file_size = len(text)
        vs.must_pause_work = vs.cmd_from_sd = False
        vs.next_file_position = 0
        vs.cancel_pending = False
        vs.work_timer = object()
        return vs

    def run_sd(self, text):
        vs = self.make_sd(text)
        vs.work_handler(0.)
        return vs

    def test_the_file_loop_stops_at_a_cancel_without_an_error(self):
        vs = self.run_sd('ONE\nABORT_NOW\nTWO\n')
        self.assertEqual(self.ran, ['ONE', 'ABORT_NOW'])
        # an abort is not an error: no on_error_gcode, no note_error, and
        # no "paused" flash before the CANCEL_PRINT that follows
        self.assertEqual(self.stats.calls, ['start'])
        self.assertTrue(vs.cancel_pending)
        self.assertIsNone(vs.work_timer)
        self.assertEqual(vs.file_position, len('ONE\n'))
        self.assertEqual(self.printer.shutdowns, [])

    def test_the_file_loop_checks_each_line_as_the_file_door(self):
        self.run_sd('ONE\nTWO\n')
        self.assertEqual(self.gate_calls, [('file', 'ONE'), ('file', 'TWO')])
        self.assertEqual(self.stats.calls, ['start', 'complete'])

    def test_a_refused_file_line_is_an_error_that_stops_the_print(self):
        vs = self.run_sd('ONE\nBAD\nTWO\n')
        self.assertEqual(self.ran, ['ONE', 'ON_ERROR'])
        self.assertEqual(self.stats.calls[0], 'start')
        self.assertTrue(self.stats.calls[1].startswith('error: Refused'))
        self.assertEqual(vs.file_position, len('ONE\n'))

    def test_a_failing_line_runs_the_error_gcode(self):
        self.run_sd('ONE\nFAIL\nTWO\n')
        self.assertEqual(self.ran, ['ONE', 'FAIL', 'ON_ERROR'])
        self.assertEqual(self.stats.calls,
                         ['start', 'error: a command failed'])


# --- B4 -------------------------------------------------------------------
HOMING = os.path.join(MACROS, 'homing.cfg')


def tmc_settings():
    return {'configfile': {'settings': {
        'tmc2241 stepper_x': {'run_current': 1.0},
        'tmc2241 stepper_y': {'run_current': 1.1}}}}


class B4HomingCurrent(unittest.TestCase):
    def test_homing_currents_are_x_06_and_y_06(self):
        lines = render(HOMING, '_HOMING_CURRENT', tmc_settings(), {})
        self.assertEqual(lines, [
            'SET_TMC_CURRENT STEPPER=stepper_x CURRENT=0.6',
            'SET_TMC_CURRENT STEPPER=stepper_y CURRENT=0.6'])

    def test_homing_currents_are_equal_on_b1(self):
        # KAN-3813: unequal X/Y homing currents shifted the Y frame about
        # 2 mm on sloe (CoreXY). Keep them equal if this is retuned.
        home = section(HOMING, 'gcode_macro _HOMING_CURRENT')
        self.assertEqual(float(home['variable_current']),
                         float(home['variable_current_y']))

    def test_the_cut_stop_find_taps_at_06_on_y(self):
        # KAN-3813: one tap towards Y min with no confirming tap. At the
        # 0.8 A it tapped 0.8 mm after arming on sloe (17 Nov).
        # Y= is what reaches stepper_y, not current_y: 0.5 here, so the
        # tap's value can be told from the home's (both are 0.6 now).
        lines = render(HOMING, '_HOMING_CURRENT', tmc_settings(),
                       {'Y': '0.5'})
        self.assertEqual(lines, [
            'SET_TMC_CURRENT STEPPER=stepper_x CURRENT=0.6',
            'SET_TMC_CURRENT STEPPER=stepper_y CURRENT=0.5'])
        find = macro_lines(os.path.join(MACROS, 'filament.cfg'),
                           '_CUT_FILAMENT_FIND_STOP')
        calls = [l.strip() for l in find
                 if l.strip().startswith('_HOMING_CURRENT')]
        self.assertEqual(calls, [
            '_HOMING_CURRENT Y={c.tap_current_y}',
            '_HOMING_CURRENT RESTORE=1'])
        # The tap's current is CUT_FILAMENT's own, not _HOMING_CURRENT's
        # (retuning homing must not move it) and not the Y home's
        cut = section(os.path.join(MACROS, 'filament.cfg'),
                      'gcode_macro CUT_FILAMENT')
        self.assertEqual(cut['variable_tap_current_y'], '0.6')
        home = section(HOMING, 'gcode_macro _HOMING_CURRENT')
        self.assertEqual(home['variable_current_y'], '0.6')

    def test_restore_puts_both_run_currents_back(self):
        lines = render(HOMING, '_HOMING_CURRENT', tmc_settings(),
                       {'RESTORE': '1'})
        self.assertEqual(lines, [
            'SET_TMC_CURRENT STEPPER=stepper_x CURRENT=1.0',
            'SET_TMC_CURRENT STEPPER=stepper_y CURRENT=1.1'])

    def test_no_other_homing_macro_sets_a_motor_current(self):
        for sect, lines in read_cfg(HOMING):
            if not sect.startswith('gcode_macro '):
                continue
            name = sect.split(' ', 1)[1]
            text = options(lines).get('gcode', '')
            tmc = [l for l in text.split('\n')
                   if 'SET_TMC_CURRENT' in l and not l.startswith('#')]
            if name == '_HOMING_CURRENT':
                self.assertEqual(len(tmc), 4)
            else:
                self.assertEqual(tmc, [], name)
        # and the axis helpers go through it, once per G28
        for name in ('_HOME_X', '_HOME_Y'):
            text = '\n'.join(macro_lines(HOMING, name))
            self.assertIn('_HOMING_CURRENT\n', text + '\n')
            self.assertIn('_HOMING_CURRENT RESTORE=1', text)
        g28 = '\n'.join(macro_lines(HOMING, 'G28'))
        self.assertIn('_HOMING_CURRENT RESTORE=1', g28)

    def test_x_and_y_homing_are_the_only_homing_current_users(self):
        # The only other X/Y current change in the macros is CUT_FILAMENT's
        # stroke (its own cut_current), and debug.cfg's generic loop.
        users = set()
        for path in cfg_files():
            for sect, line in command_lines(path):
                if (line.startswith('SET_TMC_CURRENT')
                        and re.search(r'stepper_[xy]', line)):
                    users.add((os.path.basename(path), sect))
        self.assertEqual(users, {
            ('homing.cfg', 'gcode_macro _HOMING_CURRENT'),
            ('filament.cfg', 'gcode_macro _CUT_FILAMENT_CUT')})


# --- B5 -------------------------------------------------------------------
class B5ForceMove(unittest.TestCase):
    def test_no_macro_calls_force_move(self):
        for path in cfg_files():
            for sect, line in command_lines(path):
                self.assertNotRegex(
                    line, r'(^|[\s"\'{|;])FORCE_MOVE\b',
                    '%s [%s] calls FORCE_MOVE: %s'
                    % (os.path.basename(path), sect, line))

    def test_force_move_is_off_and_only_declared_once(self):
        self.assertEqual(section(CORE, 'force_move')['enable_force_move'],
                         'false')
        for path in cfg_files():
            sects = [s for s, _ in read_cfg(path) if s == 'force_move']
            self.assertEqual(len(sects), 1 if path == CORE else 0, path)

    def test_the_unlock_press_retracts_with_cold_retract_16(self):
        lines = macro_lines(os.path.join(MACROS, 'toolhead.cfg'),
                            'TOOLHEAD_UMBILICAL_UNLOCK_BUTTON_PRESSED')
        retracts = [l for l in lines if l.startswith('COLD_RETRACT')]
        self.assertEqual(retracts, ['COLD_RETRACT DISTANCE=16'])
        limit = float(section(CORE, 'cold_retract')['max_distance'])
        self.assertEqual(limit, 16.)
        self.assertEqual(section(CORE, 'cold_retract')['stepper'],
                         'extruder')
        # the retract is in the branch that touches the hot end, after the
        # hot end is off (KAN-569) and before the stepper is disabled
        text = '\n'.join(lines)
        self.assertLess(text.index('M104 S0'), text.index('COLD_RETRACT'))
        self.assertLess(text.index('COLD_RETRACT'),
                        text.index('SET_STEPPER_ENABLE'))


# --- z_power: an unknown column reading pauses a print -------------------
class ColumnChangedMacro(unittest.TestCase):
    """printer.z_power.z is None when the reading matches no band.  A plain
    default("unknown") leaves None alone, so the macro took its "locked"
    branch and a print carried on with the bed unpowered."""
    BED = os.path.join(MACROS, 'bed.cfg')

    def lines(self, z, job):
        opts = section(self.BED, 'gcode_macro _Z_COLUMN_CHANGED')
        text = '\n'.join(l.strip() for l in opts['gcode'].split('\n'))
        env = jinja2.Environment('{%', '%}', '{', '}')
        out = env.from_string(text).render(
            printer={'z_power': {'z': z}, 'print_stats': {'state': job}},
            params={}, action_respond_info=lambda msg: '')
        return [l.strip() for l in out.split('\n') if l.strip()]

    def test_unknown_pauses_a_running_print(self):
        self.assertEqual(self.lines(None, 'printing'),
                         ['PAUSE REASON=column_unlocked'])

    def test_unlocked_pauses_a_running_print(self):
        self.assertEqual(self.lines('unlocked', 'printing'),
                         ['PAUSE REASON=column_unlocked'])

    def test_locked_does_not_pause(self):
        self.assertNotIn('PAUSE REASON=column_unlocked',
                         self.lines('locked', 'printing'))


# --- KAN-3387: the start gate shows no macro prompt ---------------------
class StartGateHasNoMacroPrompt(unittest.TestCase):
    def test_m1_machine_shows_no_gate_prompt(self):
        path = os.path.join(ROOT, 'klippy', 'extras', 'm1_machine.py')
        with io.open(path, 'r', encoding='utf-8') as f:
            tree = ast.parse(f.read())
        shown = []
        for node in ast.walk(tree):
            if (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and node.func.attr == '_show' and node.args
                    and isinstance(node.args[0], ast.Constant)):
                shown.append(node.args[0].value)
        # the pause prompt (Resume and Cancel buttons) is the only one left
        self.assertEqual(shown, ['pause'])
        with io.open(path, 'r', encoding='utf-8') as f:
            text = f.read()
        for title in ('Waiting to start', '"Ready"'):
            self.assertNotIn(title, text)


if __name__ == '__main__':
    unittest.main()
