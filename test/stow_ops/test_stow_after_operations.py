#!/usr/bin/env python3
"""The bed rests at the top after a user operation (KAN-4298).

Z_PARK raises the bed to the stow position (Z140). After a full user home
(KAN-3811) and after a print it already did; this covers the user-facing
filament operations that end idle: UNLOAD_FILAMENT, the insert flow's Quick
Load and Full Purge, a failed print (_ON_PRINT_ERROR), the calibrations (bed
mesh, resonance, PID and bed MPC, the probe's Z offset) when they end, are
cancelled or fail. They stow (through _STOW_IF_IDLE) only when no print is
running or paused (a RESUME depends on where the head is left), Z is homed
and the Z feed is ready. A plain jog is not a macro and never
stows, and nothing on the print path gained a stow. This renders the real
macros of core/M1 and core/M1-B0 with jinja2: no klippy, no hardware.

    python3 test/stow_ops/test_stow_after_operations.py
"""
import os
import re
import sys
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
import check_macro_config as cmc  # noqa: E402

BOARDS = ('M1', 'M1-B0')
STATES = (  # (print_stats.state, is_paused, sd active, label)
    ('standby', False, False, 'idle'),
    ('complete', False, False, 'complete'),
    ('cancelled', False, False, 'cancelled'),
    ('error', False, False, 'error'),
    ('printing', False, False, 'printing'),
    ('printing', False, True, 'printing from file'),
    ('paused', True, False, 'paused'),
    ('standby', True, False, 'pause flag alone'),
    ('standby', False, True, 'file active alone'),
)
JOB = ('printing', 'printing from file', 'paused', 'pause flag alone',
       'file active alone')
PAUSED = STATES[6]


def read(path):
    with open(path, 'rb') as f:
        return f.read().decode('utf-8')


_CACHE = {}


def macros_of(board):
    """{section header: {option: value}} for every cfg of the board."""
    if board not in _CACHE:
        out = {}
        base = os.path.join(ROOT, 'core', board)
        for path in cmc.config_files(base):
            sections, _ = cmc.parse(read(path))
            for _n, header, options in sections:
                out.setdefault(header, {}).update(
                    {k: cmc._join_value(v) for _l, k, v in options})
        _CACHE[board] = out
    return _CACHE[board]


def printer_for(state, paused, sd, homed, z_power=None, webhooks=None,
                phase=None):
    p = {
        'toolhead': {'homed_axes': homed, 'position': {'z': 20.},
                     'axis_maximum': {'x': 200.1, 'y': 174.1, 'z': 160.}},
        'print_stats': {'state': state},
        'pause_resume': {'is_paused': paused},
        'virtual_sdcard': {'is_active': sd},
        'extruder': {'target': 0., 'can_extrude': 'true'},
        'gcode_macro FINETUNE_RESONANCE': {
            'sweep_span_hz': 10, 'swept_this_session': 1, 'last_swept': 0},
        'configfile': {'settings': {'input_shaper': {}}},
        'save_variables': {'variables': {}},
    }
    if z_power is not None:
        p['z_power'] = z_power
    if webhooks is not None:
        p['webhooks'] = {'state': webhooks}
    if phase is not None:
        p['gcode_macro _PRINT_START_MARK'] = {'phase': phase}
    return p


def render(board, name, st, homed='xyz', z_power=None, params=None,
           option='gcode', webhooks=None, phase=None):
    macro = macros_of(board)['gcode_macro ' + name][option]
    text = cmc._jinja_env().from_string(macro).render(
        rawparams='', params=params or {},
        action_respond_info=lambda m: '',
        printer=printer_for(*st[:3], homed, z_power, webhooks, phase))
    return [ln.strip() for ln in text.split('\n')
            if ln.strip() and not ln.strip().startswith('#')]


def zparks(board, name, st, homed='xyz', z_power=None, params=None,
           option='gcode', webhooks=None, phase=None):
    """How many Z_PARK run when the macro (and the helper it calls) does."""
    n = 0
    kw = dict(webhooks=webhooks, phase=phase)
    for ln in render(board, name, st, homed, z_power, params, option, **kw):
        if ln.startswith('Z_PARK'):
            n += 1
        elif ln.startswith('_STOW_IF_IDLE'):
            n += sum(x.startswith('Z_PARK') for x in
                     render(board, '_STOW_IF_IDLE', st, homed, z_power, **kw))
        elif ln.startswith('_STOW_AFTER_PRINT_ERROR'):
            n += sum(x.startswith('Z_PARK') for x in
                     render(board, '_STOW_AFTER_PRINT_ERROR', st, homed,
                            z_power, **kw))
    return n


class Helper(unittest.TestCase):
    def test_stows_only_when_idle_and_z_homed(self):
        for b in BOARDS:
            for st in STATES:
                for homed in ('xyz', 'z', 'xy', ''):
                    want = (st[3] not in JOB) and 'z' in homed
                    self.assertEqual(
                        zparks(b, '_STOW_IF_IDLE', st, homed),
                        1 if want else 0, (b, st, homed))

    def test_stows_only_with_the_z_feed_ready(self):
        # A plate off, the column lever unlocked, a recovery or a fault can
        # leave the 48V_Z feed off while Z still reads homed.
        idle = STATES[0]
        for b in BOARDS:
            for state, want in (('ready', 1), ('no_plate', 0),
                                ('recovering', 0), ('fault', 0),
                                ('unknown', 0), ('', 0)):
                for col in ('locked', 'unlocked'):
                    zp = {'state': state, 'z': col}
                    self.assertEqual(
                        zparks(b, '_STOW_IF_IDLE', idle, 'xyz', zp),
                        want, (b, zp))
            # a column left unlocked reports a non-ready state
            self.assertEqual(zparks(b, '_STOW_IF_IDLE', idle, 'xyz',
                                    {'state': 'no_plate', 'z': 'unlocked'}),
                             0, b)
            # no state key at all: not ready
            self.assertEqual(zparks(b, '_STOW_IF_IDLE', idle, 'xyz',
                                    {'z': 'unlocked'}), 0, b)
            # a ready feed still never stows in a job or with Z unhomed
            ready = {'state': 'ready', 'z': 'locked'}
            for st in STATES:
                self.assertEqual(zparks(b, '_STOW_IF_IDLE', st, 'xyz', ready),
                                 0 if st[3] in JOB else 1, (b, st))
            self.assertEqual(zparks(b, '_STOW_IF_IDLE', idle, 'xy', ready), 0)

    def test_every_operation_honours_the_z_feed_gate(self):
        idle = STATES[0]
        for b in BOARDS:
            for name in ('UNLOAD_FILAMENT', 'FILAMENT_INSERT_QUICK_LOAD',
                         'FILAMENT_INSERT_FULL_PURGE'):
                for state in ('no_plate', 'recovering', 'fault', 'unknown'):
                    self.assertEqual(
                        zparks(b, name, idle, 'xyz', {'state': state}), 0,
                        (b, name, state))
                self.assertEqual(
                    zparks(b, name, idle, 'xyz', {'state': 'ready'}), 1,
                    (b, name))

    def test_never_runs_the_unhomed_parking_routine(self):
        # No AXIS_HOME_MOVE, no Z_PARK: nothing at all with Z unhomed.
        for b in BOARDS:
            for st in STATES:
                self.assertEqual(render(b, '_STOW_IF_IDLE', st, 'xy'), [],
                                 (b, st))


class Operations(unittest.TestCase):
    def test_unload_and_quick_load_stow_standalone_only(self):
        for b in BOARDS:
            for name in ('UNLOAD_FILAMENT', 'FILAMENT_INSERT_QUICK_LOAD'):
                for st in STATES:
                    self.assertEqual(zparks(b, name, st),
                                     0 if st[3] in JOB else 1, (b, name, st))
                    self.assertEqual(zparks(b, name, st, 'xy'), 0,
                                     (b, name, st, 'z unhomed'))

    def test_unload_stows_after_the_unload_itself(self):
        for b in BOARDS:
            lines = render(b, 'UNLOAD_FILAMENT', STATES[0])
            stow = lines.index('_STOW_IF_IDLE')
            m400 = max(n for n, ln in enumerate(lines)
                       if ln.startswith('M400'))
            self.assertGreater(stow, m400)
            self.assertGreater(stow, lines.index(
                'RESTORE_GCODE_STATE NAME=UNLOAD_FILAMENT'))

    def test_full_purge_stows_when_not_in_a_print(self):
        for b in BOARDS:
            for st in STATES:
                for homed in ('xyz', 'xy'):
                    # Z unhomed: the purge sets Z only to move X and Y; it
                    # ends with no Z motion at all.
                    want = 'z' in homed and st[3] not in JOB
                    self.assertEqual(
                        zparks(b, 'FILAMENT_INSERT_FULL_PURGE', st, homed),
                        1 if want else 0, (b, st, homed))

    def test_the_purge_stow_is_last_before_restoring_the_state(self):
        for b in BOARDS:
            lines = render(b, 'FILAMENT_INSERT_FULL_PURGE', STATES[0])
            i = max(n for n, ln in enumerate(lines)
                    if ln == '_STOW_IF_IDLE')
            self.assertEqual(lines[i + 1], 'M400', b)
            self.assertTrue(lines[i + 2].startswith('RESTORE_GCODE_STATE'), b)


class PrintPathAndPositioning(unittest.TestCase):
    def text(self, b, name):
        return macros_of(b)['gcode_macro ' + name]['gcode']

    def test_print_path_callers_do_not_stow(self):
        # They end the print elsewhere (PRINT_END's park) or hand back to a
        # RESUME; none calls the stow helper.
        for b in BOARDS:
            for name in ('PAUSE', 'RESUME', 'M600', '_CANCEL_M600',
                         '_DO_UNLOAD_M600', '_RUNOUT_UNLOAD',
                         '_FILAMENT_RUNOUT', 'INSERT_FILAMENT_SENSOR',
                         'FILAMENT_INSERT_FLOW', 'FILAMENT_INSERT_CANCEL',
                         'PRINT_START', 'CANCEL_PRINT', '_BED_MESH_ENSURE',
                         'PURGE_LINE'):
                if 'gcode_macro ' + name in macros_of(b):
                    self.assertNotIn('_STOW_IF_IDLE', self.text(b, name),
                                     (b, name))

    def test_a_paused_print_flow_does_not_stow(self):
        # A runout pause: unload, insert, purge, then RESUME.
        for b in BOARDS:
            for name in ('UNLOAD_FILAMENT', 'FILAMENT_INSERT_QUICK_LOAD',
                         'FILAMENT_INSERT_FULL_PURGE'):
                self.assertEqual(zparks(b, name, PAUSED), 0, (b, name))

    def test_moves_and_jogs_do_not_stow(self):
        # A jog is G0/G1 from the Move screen, not a macro. The positioning
        # macros are moves of the same kind: the user put the bed there.
        for b in BOARDS:
            for name in ('CENTER', 'FRONT', 'ANTI_OOZE_NOZZLE_DOCK'):
                t = self.text(b, name)
                self.assertNotIn('Z_PARK', t, (b, name))
                self.assertNotIn('_STOW_IF_IDLE', t, (b, name))

    def test_only_the_listed_macros_stow(self):
        # Scans every gcode / *_gcode option, any case: a macro call or a
        # raw G0/G1 to Z140.
        pat = r'(?i)(\{%[^}]*%\}\s*)?NAME\b'
        raw = re.compile(r'(?i)(\{%[^}]*%\}\s*)?G0?[01]\b[^;]*\bZ\s*140\b')
        for b in BOARDS:
            helper, direct = set(), set()
            for header, opts in macros_of(b).items():
                for key, val in opts.items():
                    if key != 'gcode' and not key.endswith('_gcode'):
                        continue
                    lines = [ln.strip() for ln in val.split('\n')]
                    if any(re.match(pat.replace('NAME', '_STOW_IF_IDLE'), ln)
                           for ln in lines):
                        helper.add(header)
                    if any(re.match(pat.replace('NAME', 'Z_PARK'), ln)
                           or raw.match(ln) for ln in lines):
                        direct.add(header)
            self.assertEqual(
                helper, {'gcode_macro UNLOAD_FILAMENT',
                         'gcode_macro FILAMENT_INSERT_QUICK_LOAD',
                         'gcode_macro FILAMENT_INSERT_FULL_PURGE',
                         'gcode_macro BED_MESH_CALIBRATE',
                         'gcode_macro SHAPER_CALIBRATE',
                         'gcode_macro FINETUNE_RESONANCE',
                         'gcode_macro PID_CALIBRATE',
                         'gcode_macro CALIBRATE_HEATING_BED',
                         'gcode_macro CALIBRATE_PROBE',
                         'gcode_macro _DO_CALIBRATE_PROBE',
                         'gcode_macro _CANCEL_CALIBRATE_PROBE'}, b)
            expect = {'gcode_macro _STOW_IF_IDLE',
                      'gcode_macro _STOW_AFTER_PRINT_ERROR',
                      'gcode_macro PARK_PRINTER', 'gcode_macro G28',
                      'gcode_macro Z_PARK'}  # Z_PARK is the raw G0 Z140
            if b == 'M1':  # B0 has no safe park
                expect.add('print_end_park')
            self.assertEqual(direct, expect, b)


class NewGates(unittest.TestCase):
    def test_no_stow_when_klipper_is_shut_down(self):
        for b in BOARDS:
            for name in ('_STOW_IF_IDLE', '_STOW_AFTER_PRINT_ERROR'):
                for wh, want in (('ready', 1), ('shutdown', 0),
                                 ('error', 0), ('startup', 0)):
                    self.assertEqual(
                        zparks(b, name, STATES[0], webhooks=wh), want,
                        (b, name, wh))

    def test_no_stow_during_a_console_print_start(self):
        for b in BOARDS:
            self.assertEqual(zparks(b, '_STOW_IF_IDLE', STATES[0],
                                    phase='mesh'), 0, b)
            self.assertEqual(zparks(b, '_STOW_IF_IDLE', STATES[0],
                                    phase=''), 1, b)


class PrintError(unittest.TestCase):
    """A failed print (_ON_PRINT_ERROR) rests the bed at the top."""
    ERR = ('printing', False, True, 'failing print')  # note_error comes later

    def test_gates(self):
        for b in BOARDS:
            name = '_STOW_AFTER_PRINT_ERROR'
            # job state does not matter, a homed Z and a ready feed do
            for st in STATES[:5] + (self.ERR,):
                self.assertEqual(zparks(b, name, st), 1, (b, st))
            self.assertEqual(zparks(b, name, self.ERR, 'xy'), 0, b)
            self.assertEqual(zparks(b, name, self.ERR, ''), 0, b)
            for state in ('no_plate', 'recovering', 'fault', 'unknown', ''):
                self.assertEqual(
                    zparks(b, name, self.ERR, 'xyz', {'state': state}), 0,
                    (b, state))
            self.assertEqual(zparks(b, name, self.ERR, 'xyz',
                                    {'state': 'ready'}), 1, b)
            # a pause that is still set (CLEAR_PAUSE did not clear it)
            self.assertEqual(zparks(b, name, PAUSED), 0, b)

    def test_on_print_error_stows_last_after_clear_pause(self):
        for b in BOARDS:
            lines = [ln.strip() for ln in macros_of(b)[
                'gcode_macro _ON_PRINT_ERROR']['gcode'].split('\n')]
            lines = [ln for ln in lines if ln]
            self.assertEqual(lines[-1], '_STOW_AFTER_PRINT_ERROR', b)
            self.assertLess(lines.index('CLEAR_PAUSE'),
                            lines.index('_STOW_AFTER_PRINT_ERROR'), b)
            self.assertEqual(zparks(b, '_ON_PRINT_ERROR', self.ERR), 1, b)
            self.assertEqual(zparks(b, '_ON_PRINT_ERROR', self.ERR, 'xy'),
                             0, b)
            self.assertEqual(zparks(b, '_ON_PRINT_ERROR', self.ERR, 'xyz',
                                    {'state': 'fault'}), 0, b)

    def test_the_gate_helper_would_refuse_inside_the_print(self):
        # why a separate helper exists
        for b in BOARDS:
            self.assertEqual(zparks(b, '_STOW_IF_IDLE', self.ERR), 0, b)


class Calibration(unittest.TestCase):
    WRAPPED = ('BED_MESH_CALIBRATE', 'SHAPER_CALIBRATE')
    ENDS = WRAPPED + ('FINETUNE_RESONANCE', 'CALIBRATE_HEATING_BED',
                      'PID_CALIBRATE')

    def test_wrappers_keep_the_original_and_pass_params(self):
        for b in BOARDS:
            for name in self.WRAPPED:
                opts = macros_of(b)['gcode_macro ' + name]
                self.assertEqual(opts['rename_existing'], name + '_BASE')
                self.assertIn(name + '_BASE {rawparams}', opts['gcode'])

    def test_end_stows_under_the_gates_and_stow_0_skips(self):
        for b in BOARDS:
            for name in self.ENDS:
                p = {'HEATER': 'heater_bed'} if name == 'PID_CALIBRATE' else {}
                for st in STATES:
                    self.assertEqual(zparks(b, name, st, params=p),
                                     0 if st[3] in JOB else 1, (b, name, st))
                self.assertEqual(zparks(b, name, STATES[0], 'xy', params=p),
                                 0, (b, name))
                self.assertEqual(
                    zparks(b, name, STATES[0], params=p,
                           z_power={'state': 'fault'}), 0, (b, name))
                self.assertEqual(
                    zparks(b, name, STATES[0], params=p, webhooks='shutdown'),
                    0, (b, name))
                self.assertEqual(
                    zparks(b, name, STATES[0], params=p, phase='mesh'),
                    0, (b, name))
            self.assertEqual(
                zparks(b, 'FINETUNE_RESONANCE', STATES[0],
                       params={'STOW': '0'}), 0, b)

    def test_failure_stows_under_the_gates(self):
        for b in BOARDS:
            for name in self.WRAPPED + ('FINETUNE_RESONANCE',
                                        'CALIBRATE_HEATING_BED',
                                        'CALIBRATE_PROBE',
                                        '_DO_CALIBRATE_PROBE'):
                kw = dict(option='on_error_gcode')
                for st in STATES:
                    self.assertEqual(
                        zparks(b, name, st, **kw),
                        0 if st[3] in JOB else 1, (b, name, st))
                self.assertEqual(zparks(b, name, STATES[0], 'xy', **kw), 0)
                self.assertEqual(zparks(b, name, STATES[0],
                                        z_power={'state': 'no_plate'}, **kw),
                                 0, (b, name))
                self.assertEqual(zparks(b, name, STATES[0],
                                        webhooks='shutdown', **kw), 0)
            self.assertEqual(
                zparks(b, 'FINETUNE_RESONANCE', STATES[0],
                       params={'STOW': '0'}, option='on_error_gcode'), 0, b)

    def test_pid_stows_after_the_base_command(self):
        for b in BOARDS:
            g = render(b, 'PID_CALIBRATE', STATES[0],
                       params={'HEATER': 'heater_bed'})
            self.assertEqual(g[-1], '_STOW_IF_IDLE', b)

    def test_cancel_button_stows(self):
        for b in BOARDS:
            self.assertEqual(zparks(b, '_CANCEL_CALIBRATE_PROBE', STATES[0]),
                             1, b)
            self.assertEqual(zparks(b, '_CANCEL_CALIBRATE_PROBE', PAUSED), 0)
            self.assertEqual(zparks(b, '_CLOSE_CALIBRATE_PROBE', STATES[0]), 0)
            self.assertEqual(zparks(b, '_DO_CALIBRATE_PROBE', STATES[0]), 0)
            for n in ('CALIBRATE_PROBE', '_HELP_CALIBRATE_PROBE'):
                t = macros_of(b)['gcode_macro ' + n]['gcode']
                self.assertNotIn('|_CLOSE_CALIBRATE_PROBE|', t, (b, n))

    def test_print_path_calls_pass_stow_0(self):
        for b in BOARDS:
            ps = macros_of(b)['gcode_macro PRINT_START']['gcode']
            self.assertRegex(ps, r'(?m)^\s*FINETUNE_RESONANCE STOW=0\s*$')
            cal = macros_of(b)['gcode_macro FINETUNE_RESONANCE']['gcode']
            self.assertEqual(len(re.findall(
                r'(?m)^\s*SHAPER_CALIBRATE AXIS=\w .*STOW=0$', cal)), 4, b)
            ens = macros_of(b)['gcode_macro _BED_MESH_ENSURE']['gcode']
            calls = [ln for ln in ens.splitlines()
                     if ln.strip().startswith('BED_MESH_CALIBRATE')]
            self.assertEqual(len(calls), 2, b)
            for ln in calls:
                self.assertTrue(ln.rstrip().endswith('STOW=0'), (b, ln))

    def test_wrappers_are_not_in_calibration_cfg(self):
        # the klippy regression tests include calibration.cfg and stub the
        # commands the wrappers rename
        for b in BOARDS:
            text = read(os.path.join(ROOT, 'core', b, 'macros',
                                     'calibration.cfg'))
            self.assertNotIn('rename_existing: BED_MESH', text)
            self.assertNotIn('rename_existing: SHAPER', text)
            inc = read(os.path.join(ROOT, 'core', b, 'macros', 'macros.cfg'))
            self.assertIn('[include stow_wrappers.cfg]', inc)



class PythonHooks(unittest.TestCase):
    def setUp(self):
        for sub in ('klippy', os.path.join('klippy', 'extras')):
            sys.path.insert(0, os.path.join(ROOT, sub))

    def test_manual_probe_finalize_stows_on_accept_abort_and_failure(self):
        import manual_probe as mp

        class Err(Exception):
            pass

        class Gcode:
            def __init__(self):
                self.ran = []

            def register_command(self, *a, **k):
                pass

            def run_script_from_command(self, s):
                self.ran.append(s)

        class Printer:
            command_error = Err

            def __init__(self, macro=True):
                self.gcode = Gcode()
                self.macro = macro

            def lookup_object(self, name, default=None):
                if name == 'gcode':
                    return self.gcode
                if name == 'gcode_macro _STOW_IF_IDLE' and self.macro:
                    return object()
                return default

        class MP:
            def reset_status(self):
                pass

        def helper(pr, cb):
            h = mp.ManualProbeHelper.__new__(mp.ManualProbeHelper)
            h.printer, h.gcode, h.manual_probe = pr, pr.gcode, MP()
            h.finalize_callback = cb
            h.get_kinematics_pos = lambda: [1., 2., 3.]
            return h

        pr = Printer()
        helper(pr, lambda r: None).finalize(False)      # ABORT
        helper(pr, lambda r: None).finalize(True)       # ACCEPT
        self.assertEqual(pr.gcode.ran, ['_STOW_IF_IDLE'] * 2)

        def boom(r):
            raise Err('callback failed')
        pr = Printer()
        with self.assertRaises(Err):                    # the error survives
            helper(pr, boom).finalize(True)
        self.assertEqual(pr.gcode.ran, ['_STOW_IF_IDLE'])

        pr = Printer(macro=False)                       # not an M1 config
        helper(pr, lambda r: None).finalize(False)
        self.assertEqual(pr.gcode.ran, [])

    def test_macro_on_error_gcode_runs_then_reraises(self):
        import gcode_macro as gm

        class Err(Exception):
            pass

        class Abort(Err):
            pass

        ran = []

        class Tpl:
            def __init__(self, name, fail=None):
                self.name, self.fail = name, fail

            def run_gcode_from_command(self, ctx=None, abortable=False):
                ran.append(self.name)
                if self.fail:
                    raise self.fail

        class Ctx:
            def create_template_context(self):
                return {}

        class G:
            error, abort_error = Err, Abort

        class Cmd:
            def get_command_parameters(self):
                return {}

            def get_raw_command_parameters(self):
                return ''

        def macro(fail, on_error=True, err_fail=None):
            m = gm.GCodeMacro.__new__(gm.GCodeMacro)
            m.recursion_limit, m.script_depth, m.alias = 1, 0, 'X'
            m.variables, m.abortable, m.gcode = {}, False, G()
            m.template = Tpl('gcode', fail)
            m.template.create_template_context = lambda: {}
            m.on_error = Tpl('on_error', err_fail)
            m.has_on_error = on_error
            return m

        def run(m):
            del ran[:]
            try:
                m.cmd(Cmd())
            except Err as e:
                return type(e), list(ran)
            return None, list(ran)

        self.assertEqual(run(macro(None)), (None, ['gcode']))
        self.assertEqual(run(macro(Err('x'))), (Err, ['gcode', 'on_error']))
        self.assertEqual(run(macro(Err('x'), on_error=False)),
                         (Err, ['gcode']))
        # a cancel is not an error: no stow
        self.assertEqual(run(macro(Abort('x'))), (Abort, ['gcode']))
        # a failing on_error does not replace the original error
        self.assertEqual(run(macro(Err('x'), err_fail=Err('y'))),
                         (Err, ['gcode', 'on_error']))


class PrintEndStillStows(unittest.TestCase):
    """A print ends at Z140, normally and by cancel (KAN-4298 confirms)."""

    def test_print_end_park_chain_reaches_z_park(self):
        m = macros_of('M1')
        end = m['gcode_macro PRINT_END']['gcode']
        lines = [ln.strip() for ln in end.split('\n')]
        self.assertLess(lines.index('_PRINT_END_PARK'),
                        lines.index('_PRINT_END_PARK_SAFE'))
        park = m['print_end_park']
        self.assertRegex(park['gcode'].rstrip(), r'CUT_FILAMENT$')
        # the cut ends with the park, whose first move is Z_PARK
        self.assertRegex(m['gcode_macro _CUT_FILAMENT_CUT']['gcode'],
                         r'(?m)^\s*PARK_PRINTER\s*$')
        self.assertRegex(m['gcode_macro PARK_PRINTER']['gcode'],
                         r'(?m)^\s*Z_PARK\s*$')
        # a refused park still stows when Z is homed
        self.assertRegex(park['safe_gcode'],
                         r"if 'z' in HA %\}\s*\n\s*Z_PARK\b")

    def test_park_printer_stows_whatever_the_state(self):
        # It runs inside PRINT_END, where the print is still "printing".
        for b in BOARDS:
            for st in STATES:
                self.assertEqual(render(b, 'PARK_PRINTER', st)[0], 'Z_PARK',
                                 (b, st))

    def test_cancel_print_ends_through_print_end(self):
        cancel = macros_of('M1')['gcode_macro CANCEL_PRINT']['gcode']
        self.assertRegex(cancel, r'(?m)^\s*PRINT_END\s*$')

    def test_the_stow_position_is_140(self):
        for b in BOARDS:
            z = re.search(r'G0 Z([\d.]+)',
                          macros_of(b)['gcode_macro Z_PARK']['gcode'])
            self.assertEqual(float(z.group(1)), 140.0, b)


if __name__ == '__main__':
    unittest.main()
