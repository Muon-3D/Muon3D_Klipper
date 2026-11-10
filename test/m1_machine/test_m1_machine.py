# Tests for klippy/extras/m1_machine.py: the requirement and activity
# decisions, and the gates, heat waits and pause bookkeeping run against
# stub sources (z_power, the plate detector, the toolhead MCU, the hot end,
# the filament switch, the fan latch, print_stats, pause_resume, heaters).
#
#   python3 test/m1_machine/test_m1_machine.py
#
# Offline, no hardware. The reactor is a fake clock: pause() advances it and
# runs whatever a test scheduled for that time, so a gate that waits for a
# plate is exercised by fitting the plate at t+5 s.
import os
import sys
import unittest

KLIPPY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

from extras import m1_machine  # noqa: E402
from extras.m1_machine import missing, activity  # noqa: E402

READY = {'column': 'locked', 'bed_power': 'ready', 'plate': 'present',
         'toolhead': 'connected', 'hotend': 'ntc', 'filament': 'present',
         'fan_fault': False, 'plate_cleared': True}


def cond(**kw):
    c = dict(READY)
    c.update(kw)
    return c


class TestMissing(unittest.TestCase):
    def test_ready_machine_needs_nothing(self):
        for action in m1_machine.REQUIRES:
            self.assertEqual(missing(action, cond()), [], action)

    def test_everything_missing_is_listed_at_once_in_order(self):
        # Audit R3/F14: not the first missing item, all of them.
        c = cond(column='unlocked', plate='absent', toolhead='disconnected',
                 bed_power='unlocked', plate_cleared=False)
        self.assertEqual(missing('print', c), [
            'column_unlocked', 'plate_absent', 'toolhead_disconnected',
            'plate_not_cleared'])

    def test_bed_power_only_once_column_and_plate_are_right(self):
        self.assertEqual(missing('print', cond(plate='absent',
                                               bed_power='no_plate')),
                         ['plate_absent'])
        self.assertEqual(missing('print', cond(bed_power='powering')),
                         ['bed_power_starting'])
        self.assertEqual(missing('print', cond(bed_power='no_plate')),
                         ['bed_power_starting'])
        # Its own faults are always listed.
        self.assertEqual(missing('print', cond(plate='absent',
                                               bed_power='fault')),
                         ['plate_absent', 'bed_power_fault'])

    def test_unread_column_is_waited_for(self):
        self.assertEqual(missing('print', cond(column='reading')),
                         ['column_reading'])
        self.assertNotIn('column_reading', m1_machine.REFUSE)
        self.assertNotIn('column_unknown', m1_machine.REFUSE)

    def test_hotend_and_filament_only_with_a_toolhead(self):
        c = cond(toolhead='disconnected', hotend='disconnected',
                 filament='absent')
        self.assertEqual(missing('print', c), ['toolhead_disconnected'])
        self.assertEqual(missing('print', cond(hotend='settling')),
                         ['hotend_detecting'])
        self.assertEqual(missing('print', cond(hotend='failed')),
                         ['hotend_failed'])

    def test_resume_takes_either_hotend_type(self):
        # Harry, 2026-11-04: nobody changes a hot end mid-print, and
        # muon_hotend re-detects the type itself. No same-type check.
        self.assertEqual(missing('resume', cond(hotend='pt1000')), [])

    def test_plate_cleared_is_for_a_new_print_only(self):
        c = cond(plate_cleared=False)
        self.assertEqual(missing('print', c), ['plate_not_cleared'])
        self.assertEqual(missing('resume', c), [])

    def test_xy_home_needs_the_column_fitted_not_locked(self):
        self.assertEqual(missing('home_xy', cond(column='unlocked',
                                                 plate='absent')), [])
        self.assertEqual(missing('home_xy', cond(column='absent')),
                         ['column_absent'])
        self.assertEqual(missing('home_z', cond(column='unlocked')),
                         ['column_unlocked'])

    def test_machine_without_the_b1_sources(self):
        c = cond(column='none', bed_power='none', plate='none',
                 toolhead='none', hotend='none', filament='none')
        self.assertEqual(missing('print', c), [])


class TestActivity(unittest.TestCase):
    def test_motion_is_never_a_job(self):
        # Audit R1/F1: the boot auto-load made idle_timeout "Printing".
        self.assertEqual(activity('standby', False, False, False, [], True,
                                  True), 'busy')
        self.assertEqual(activity('standby', False, False, False, [], True,
                                  False), 'idle')

    def test_job_states(self):
        self.assertEqual(activity('printing', False, True, True, [], True,
                                  True), 'waiting')
        self.assertEqual(activity('printing', False, False, True, [], True,
                                  True), 'preparing')
        self.assertEqual(activity('printing', False, False, False, [], True,
                                  True), 'printing')
        self.assertEqual(activity('printing', True, False, False,
                                  ['fan_fault'], True, False), 'paused')
        self.assertEqual(activity('paused', False, False, False, [], True,
                                  False), 'paused')

    def test_complete_until_the_plate_is_cleared(self):
        self.assertEqual(activity('complete', False, False, False, [], False,
                                  False), 'complete')
        self.assertEqual(activity('complete', False, False, False, [], True,
                                  False), 'idle')
        self.assertEqual(activity('cancelled', False, False, False, [],
                                  False, False), 'cancelled')

    def test_fault_without_a_job(self):
        self.assertEqual(activity('complete', False, False, False,
                                  ['fan_fault'], False, False), 'fault')


# --- Stubs ---------------------------------------------------------------
class CommandError(Exception):
    pass


class ConfigError(Exception):
    pass


class Shutdown(Exception):
    pass


class FakeReactor:
    NOW = 0.

    def __init__(self):
        self.now = 1000.
        self.at = []        # (time, fn) run when the clock passes time
        self.timers = []
        self.pauses = 0

    def monotonic(self):
        return self.now

    def schedule(self, delay, fn):
        self.at.append((self.now + delay, fn))

    def pause(self, waketime):
        self.pauses += 1
        if self.pauses > 100000:
            raise AssertionError("wait never ended")
        self.now = max(self.now, waketime)
        for item in sorted(self.at, key=lambda x: x[0]):
            if item[0] <= self.now:
                self.at.remove(item)
                item[1]()
        return self.now

    def register_timer(self, fn, when):
        self.timers.append(fn)


class FakeGcode:
    def __init__(self):
        self.commands = {}
        self.out = []
        self.scripts = []      # run_script_from_command (inside a command)
        self.run = []          # (time, script) from run_script (a timer)
        self.abort = None
        self.busy = False      # another command holds the gcode lock
        self.reactor = None

    def register_command(self, name, fn, desc=None):
        self.commands[name] = fn

    def respond_raw(self, msg):
        self.out.append(msg)

    def respond_info(self, msg):
        self.out.append('// ' + msg)

    def run_script_from_command(self, script):
        self.scripts.append(script)

    def run_script(self, script):
        self.run.append((self.reactor.now, script))

    def get_mutex(self):
        gcode = self

        class Mutex:
            def test(self):
                return gcode.busy
        return Mutex()

    def check_abort(self):
        if self.abort:
            raise CommandError(self.abort)


class FakeGcmd:
    def __init__(self, gcode, params):
        self.gcode = gcode
        self.params = params
        self.error = CommandError

    def get(self, name, default=None):
        if name in self.params:
            return str(self.params[name])
        if default is None:
            raise CommandError("missing %s" % (name,))
        return default

    def get_float(self, name, default=None, minval=None, maxval=None):
        return float(self.get(name, default if default is None
                              else str(default)))

    def respond_info(self, msg):
        self.gcode.respond_info(msg)

    def respond_raw(self, msg):
        self.gcode.respond_raw(msg)


class Status:
    def __init__(self, **status):
        self.status = status

    def get_status(self, eventtime):
        return self.status


class FakeDetector:
    def __init__(self):
        self.present = True

    def plate_present(self):
        return self.present


class FakeHeater:
    def __init__(self, temp=25., target=0.):
        self.temp = temp
        self.target = target

    def get_temp(self, eventtime):
        return self.temp, self.target


class FakeHeaters:
    def __init__(self):
        self.heaters = {'heater_bed': FakeHeater(), 'extruder': FakeHeater()}

    def lookup_heater(self, name):
        if name not in self.heaters:
            raise ConfigError(name)
        return self.heaters[name]

    def set_temperature(self, heater, temp, wait=False):
        heater.target = temp


class FakePrinter:
    command_error = CommandError
    config_error = ConfigError

    def __init__(self, reactor, gcode):
        self.reactor = reactor
        self.objects = {'gcode': gcode}
        self.handlers = {}
        self.shutdown = False
        self.start_args = {}

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=Shutdown):
        if name in self.objects:
            return self.objects[name]
        if default is Shutdown:
            raise ConfigError(name)
        return default

    def register_event_handler(self, event, fn):
        self.handlers[event] = fn

    def get_start_args(self):
        return self.start_args

    def is_shutdown(self):
        return self.shutdown


class FakeConfig:
    def __init__(self, printer, values=None):
        self.printer = printer
        self.values = values or {}

    def get_printer(self):
        return self.printer

    def get(self, name, default=None):
        return self.values.get(name, default)

    def getfloat(self, name, default=None, **kw):
        return float(self.values.get(name, default))

    def getboolean(self, name, default=None):
        return bool(self.values.get(name, default))

    def has_section(self, name):
        return name in self.values.get('sections', {})

    def getsection(self, name):
        return FakeConfig(self.printer, self.values['sections'][name])


class Machine:
    """An M1 that is ready to print, with every source m1_machine reads."""

    def __init__(self, **cfg):
        self.reactor = FakeReactor()
        self.gcode = FakeGcode()
        self.gcode.reactor = self.reactor
        self.printer = p = FakePrinter(self.reactor, self.gcode)
        self.zp = Status(z='locked', state='ready', reason='')
        self.detector = FakeDetector()
        self.mcu = type('MCU', (), {'non_critical_disconnected': False})()
        self.hotend = Status(state='ntc')
        self.filament = Status(filament_detected=True)
        self.latch = type('Latch', (), {'latched': False})()
        self.ps = Status(state='standby')
        self.pr = type('PR', (), {'is_paused': False})()
        self.idle = Status(state='Idle')
        self.heaters = FakeHeaters()
        for name, obj in (
                ('z_power', self.zp), ('bed_removal_detector', self.detector),
                ('mcu toolhead', self.mcu), ('muon_hotend extruder',
                                             self.hotend),
                ('filament_switch_sensor filament_sensor_toolhead',
                 self.filament), ('hotend_fan_latch', self.latch),
                ('print_stats', self.ps), ('pause_resume', self.pr),
                ('idle_timeout', self.idle), ('heaters', self.heaters),
                ('gcode_macro LED_WARNING', object())):
            p.objects[name] = obj
        self.m1 = m1_machine.M1Machine(FakeConfig(p, cfg))
        p.objects['m1_machine'] = self.m1
        p.handlers['klippy:connect']()
        p.handlers['klippy:ready']()

    def run(self, name, **params):
        return self.gcode.commands[name](FakeGcmd(self.gcode, params))

    def poll(self):
        self.m1._track(self.reactor.now)

    def prompts(self):
        return [o for o in self.gcode.out if o.startswith('// action:')]

    def last_prompt_text(self):
        """The text lines of the last prompt shown, or None if it ended."""
        out = self.prompts()
        for i in range(len(out) - 1, -1, -1):
            if out[i] == '// action:prompt_end':
                return None
            if out[i].startswith('// action:prompt_begin'):
                return [o[len('// action:'):] for o in out[i:]]
        return None

    def start_print(self):
        self.ps.status['state'] = 'printing'
        self.poll()


class TestStartGate(unittest.TestCase):
    def test_ready_machine_starts_at_once(self):
        m = Machine()
        m.start_print()
        m.run('MACHINE_WAIT_READY', ACTION='print')
        self.assertEqual(m.reactor.pauses, 0)
        self.assertEqual(m.prompts(), [])

    def test_waits_with_the_checklist_then_continues_by_itself(self):
        m = Machine(settle_time=3.)
        m.start_print()
        m.detector.present = False
        m.zp.status.update(z='unlocked', state='unlocked')
        t0 = m.reactor.now

        def fit_plate():
            m.detector.present = True

        def lock():
            m.zp.status.update(z='locked', state='ready')
        m.reactor.schedule(5., fit_plate)
        m.reactor.schedule(8., lock)
        m.run('MACHINE_WAIT_READY', ACTION='print')
        # Locked at +8, held for the 3 s settle time.
        self.assertGreaterEqual(m.reactor.now - t0, 11.)
        self.assertLess(m.reactor.now - t0, 12.)
        text = ' '.join(o for o in m.prompts())
        self.assertIn('Lock the Z column', text)
        self.assertIn('Fit the build plate', text)
        self.assertEqual(m.prompts()[-1], '// action:prompt_end')
        self.assertIn('LED_WARNING', m.gcode.scripts)
        self.assertIsNone(m.m1.gate)

    def test_a_condition_lost_during_the_settle_restarts_it(self):
        m = Machine(settle_time=3.)
        m.start_print()
        m.detector.present = False
        t0 = m.reactor.now
        m.reactor.schedule(2., lambda: setattr(m.detector, 'present', True))
        m.reactor.schedule(3., lambda: setattr(m.detector, 'present', False))
        m.reactor.schedule(4., lambda: setattr(m.detector, 'present', True))
        m.run('MACHINE_WAIT_READY', ACTION='print')
        self.assertGreaterEqual(m.reactor.now - t0, 7.)

    def _grab_machine(self, grabbed):
        m = Machine(settle_time=1.)
        m.printer.objects['gcode_macro _GATE_FILAMENT_GRAB'] = object()
        m.printer.objects['extruder_homing sensorless_extruder'] = Status(
            last_triggered=grabbed)
        m.start_print()
        m.filament.status['filament_detected'] = False
        return m

    def test_filament_inserted_while_waiting_is_grabbed(self):
        # The sensor's insert_gcode does not run during a job, so the gate
        # grabs it itself before the print goes on (Harry, 2026-11-10).
        m = self._grab_machine(grabbed=True)
        m.reactor.schedule(
            3., lambda: m.filament.status.update(filament_detected=True))
        m.run('MACHINE_WAIT_READY', ACTION='print')
        self.assertEqual(m.gcode.scripts.count('_GATE_FILAMENT_GRAB'), 1)
        self.assertIn('// Filament grabbed', m.gcode.out)

    def test_filament_not_grabbed_keeps_the_gate_waiting(self):
        m = self._grab_machine(grabbed=False)
        homing = m.printer.objects['extruder_homing sensorless_extruder']
        m.reactor.schedule(
            3., lambda: m.filament.status.update(filament_detected=True))
        # Pulled out and inserted again; this time it is grabbed.
        m.reactor.schedule(
            10., lambda: m.filament.status.update(filament_detected=False))

        def reinsert():
            homing.status['last_triggered'] = True
            m.filament.status['filament_detected'] = True
        m.reactor.schedule(14., reinsert)
        t0 = m.reactor.now
        m.run('MACHINE_WAIT_READY', ACTION='print')
        self.assertGreaterEqual(m.reactor.now - t0, 14.)
        self.assertEqual(m.gcode.scripts.count('_GATE_FILAMENT_GRAB'), 2)
        text = ' '.join(m.prompts())
        self.assertIn('Filament not grabbed', text)

    def test_filament_already_in_is_not_grabbed(self):
        m = self._grab_machine(grabbed=True)
        m.filament.status['filament_detected'] = True
        m.detector.present = False
        m.reactor.schedule(3., lambda: setattr(m.detector, 'present', True))
        m.run('MACHINE_WAIT_READY', ACTION='print')
        self.assertNotIn('_GATE_FILAMENT_GRAB', m.gcode.scripts)

    def test_latched_fault_refuses_at_once(self):
        # Its acknowledgement is a gcode command, which cannot run while
        # the gate holds the gcode lock.
        m = Machine()
        m.start_print()
        m.latch.latched = True
        m.detector.present = False
        with self.assertRaises(CommandError) as e:
            m.run('MACHINE_WAIT_READY', ACTION='print')
        self.assertIn('fan', str(e.exception))
        self.assertEqual(m.reactor.pauses, 0)

    def test_fault_appearing_while_waiting_ends_the_wait(self):
        m = Machine()
        m.start_print()
        m.detector.present = False
        m.reactor.schedule(5., lambda: m.zp.status.update(state='fault'))
        with self.assertRaises(CommandError):
            m.run('MACHINE_WAIT_READY', ACTION='print')
        self.assertIsNone(m.m1.gate)
        self.assertEqual(m.prompts()[-1], '// action:prompt_end')

    def test_times_out(self):
        m = Machine(gate_timeout=60.)
        m.start_print()
        m.filament.status['filament_detected'] = False
        with self.assertRaises(CommandError) as e:
            m.run('MACHINE_WAIT_READY', ACTION='print')
        self.assertIn('Load filament', str(e.exception))

    def test_print_cancel_aborts_it(self):
        # KAN-382: pause_resume/cancel requests an abort, which every wait
        # checks for.
        m = Machine()
        m.start_print()
        m.detector.present = False
        m.reactor.schedule(5., lambda: setattr(m.gcode, 'abort',
                                               'Print cancelled'))
        with self.assertRaises(CommandError) as e:
            m.run('MACHINE_WAIT_READY', ACTION='print')
        self.assertIn('cancelled', str(e.exception))

    def test_job_ended_under_it_without_abort_support(self):
        m = Machine()
        m.start_print()
        m.detector.present = False
        m.reactor.schedule(5., lambda: m.ps.status.update(state='cancelled'))
        with self.assertRaises(CommandError):
            m.run('MACHINE_WAIT_READY', ACTION='print')

    def test_by_hand_outside_a_job_still_waits(self):
        m = Machine(settle_time=0.)
        m.detector.present = False
        m.reactor.schedule(5., lambda: setattr(m.detector, 'present', True))
        m.run('MACHINE_WAIT_READY', ACTION='print')

    def test_batch_mode_does_not_wait(self):
        m = Machine()
        m.printer.start_args['debugoutput'] = 'out'
        m.m1.batch = True
        m.detector.present = False
        m.run('MACHINE_WAIT_READY', ACTION='print')
        self.assertEqual(m.reactor.pauses, 0)


class TestPlateCleared(unittest.TestCase):
    def finish(self, m, state='complete'):
        m.ps.status['state'] = state
        m.poll()

    def test_a_finished_print_must_come_off_before_the_next(self):
        m = Machine()
        m.start_print()
        m.poll()            # printing, past PRINT_START
        self.finish(m)
        self.assertEqual(m.m1.get_status(0)['activity'], 'complete')
        self.assertEqual(m.m1.get_status(0)['missing']['print'],
                         ['plate_not_cleared'])
        m.detector.present = False
        m.poll()
        self.assertFalse(m.m1.plate_cleared)
        m.detector.present = True
        m.poll()
        self.assertTrue(m.m1.plate_cleared)
        self.assertEqual(m.m1.get_status(0)['activity'], 'idle')
        self.assertEqual(m.m1.get_status(0)['missing']['print'], [])

    def test_plate_off_before_the_end_does_not_count(self):
        m = Machine()
        m.start_print()
        m.poll()
        m.detector.present = False
        m.poll()
        m.detector.present = True
        m.poll()
        self.finish(m)
        self.assertFalse(m.m1.plate_cleared)

    def test_cancel_during_print_start_prints_nothing(self):
        m = Machine()
        m.m1.start_mark = type('Mark', (), {'variables': {'phase': 'home'}})()
        m.start_print()
        m.poll()
        self.finish(m, 'cancelled')
        self.assertTrue(m.m1.plate_cleared)

    def test_operator_override(self):
        m = Machine()
        m.start_print()
        m.poll()
        self.finish(m)
        m.run('MACHINE_PLATE_CLEAR')
        self.assertEqual(m.m1.get_status(0)['missing']['print'], [])

    def test_can_be_switched_off(self):
        m = Machine(require_plate_cleared=False)
        m.start_print()
        m.poll()
        self.finish(m)
        self.assertEqual(m.m1.get_status(0)['missing']['print'], [])


class TestPauseAndResume(unittest.TestCase):
    def paused(self, reason, **kw):
        m = Machine(**kw)
        m.heaters.heaters['heater_bed'].target = 60.
        m.heaters.heaters['extruder'].target = 215.
        m.start_print()
        m.poll()
        m.pr.is_paused = True
        m.ps.status['state'] = 'paused'
        m.run('MACHINE_PAUSED', REASON=reason, FAN=0.6)
        return m

    def test_job_targets_survive_the_zeroing(self):
        m = self.paused('plate_removed')
        m.heaters.heaters['heater_bed'].target = 0.
        m.detector.present = False
        m.poll()
        self.assertEqual(m.m1.job_targets, {'bed': 60., 'hotend': 215.})
        self.assertEqual(m.m1.saved_fan, 0.6)

    def test_plate_pause_prompt_follows_the_machine(self):
        m = self.paused('plate_removed')
        m.detector.present = False
        m.poll()
        text = m.last_prompt_text()
        self.assertIn('prompt_begin Build plate removed', text)
        self.assertIn('prompt_text - Fit the build plate', text)
        self.assertNotIn('prompt_footer_button Resume|RESUME|primary', text)
        m.detector.present = True
        m.poll()
        text = m.last_prompt_text()
        self.assertIn('prompt_footer_button Resume|RESUME|primary', text)
        # Unchanged state, no new prompt.
        n = len(m.prompts())
        m.poll()
        self.assertEqual(len(m.prompts()), n)

    def test_user_pause_has_no_prompt_until_resume_is_refused(self):
        m = self.paused('user')
        m.zp.status.update(z='unlocked', state='unlocked')
        m.poll()
        self.assertEqual(m.prompts(), [])
        with self.assertRaises(CommandError) as e:
            m.run('MACHINE_REQUIRE', ACTION='resume')
        self.assertIn('Lock the Z column', str(e.exception))
        self.assertIn('prompt_text - Lock the Z column: push the locking'
                      ' lever down', m.last_prompt_text())
        m.zp.status.update(z='locked', state='ready')
        m.poll()
        self.assertIn('prompt_footer_button Resume|RESUME|primary',
                      m.last_prompt_text())

    def test_require_rides_out_a_starting_bed_power_path(self):
        m = self.paused('column_unlocked')
        m.zp.status.update(state='powering')
        m.reactor.schedule(1., lambda: m.zp.status.update(state='ready'))
        m.run('MACHINE_REQUIRE', ACTION='resume', WAIT=3)

    def test_resume_after_a_toolhead_swap(self):
        m = Machine()
        m.start_print()
        m.run('MACHINE_WAIT_READY', ACTION='print')
        m.poll()
        m.pr.is_paused = True
        m.run('MACHINE_PAUSED', REASON='toolhead_disconnected')
        m.hotend.status['state'] = 'pt1000'
        m.run('MACHINE_REQUIRE', ACTION='resume')

    def test_a_second_pause_keeps_the_first_reason_and_fan(self):
        m = self.paused('plate_removed')
        m.run('MACHINE_PAUSED', REASON='toolhead_disconnected', FAN=0.)
        self.assertEqual(m.m1.pause_reason, 'plate_removed')
        self.assertEqual(m.m1.saved_fan, 0.6)

    def test_an_event_reason_replaces_user(self):
        m = self.paused('user')
        m.run('MACHINE_PAUSED', REASON='plate_removed')
        self.assertEqual(m.m1.pause_reason, 'plate_removed')

    def test_resumed_ends_only_its_own_prompt(self):
        m = self.paused('plate_removed')
        m.run('MACHINE_RESUMED')
        self.assertEqual(m.prompts()[-1], '// action:prompt_end')
        self.assertIsNone(m.m1.pause_reason)
        n = len(m.prompts())
        m.run('MACHINE_RESUMED')
        self.assertEqual(len(m.prompts()), n)

    def test_bed_goes_off_after_a_long_pause(self):
        m = self.paused('user', pause_bed_off_time=1800.)
        bed = m.heaters.heaters['heater_bed']
        m.reactor.now += 1799.
        m.poll()
        self.assertEqual(bed.target, 60.)
        m.reactor.now += 2.
        m.poll()
        self.assertEqual(bed.target, 0.)
        self.assertEqual(m.m1.job_targets['bed'], 60.)

    def test_job_end_clears_the_pause(self):
        m = self.paused('plate_removed')
        m.pr.is_paused = False
        m.ps.status['state'] = 'cancelled'
        m.poll()
        self.assertIsNone(m.m1.pause_reason)
        self.assertEqual(m.prompts()[-1], '// action:prompt_end')


class TestPauseCoolDown(unittest.TestCase):
    """Harry, 2026-11-04: PAUSE turns the hot end off. A part fan that was
    on stays at the print's speed (never spun up) until the nozzle is below
    the ooze point, then stops."""

    def paused(self, temp=215., fan=0.4, **kw):
        m = Machine(**kw)
        he = m.heaters.heaters['extruder']
        he.temp, he.target = temp, 215.
        m.start_print()
        m.poll()
        m.pr.is_paused = True
        m.ps.status['state'] = 'paused'
        he.target = 0.          # PAUSE's M104 S0
        m.run('MACHINE_PAUSED', REASON='user', FAN=fan)
        return m, he

    def fan_off_times(self, m):
        return [t for t, s in m.gcode.run if s == 'M107']

    def test_fan_keeps_its_speed_until_the_nozzle_is_cool(self):
        m, he = self.paused()
        # Nothing sent: the fan carries on at the print's speed.
        self.assertEqual(m.gcode.scripts, [])
        self.assertTrue(m.m1.get_status(0)['cooling'])
        he.temp = 161.
        m.poll()
        self.assertEqual(self.fan_off_times(m), [])
        he.temp = 139.
        m.poll()
        self.assertEqual(len(self.fan_off_times(m)), 1)
        self.assertFalse(m.m1.get_status(0)['cooling'])
        m.poll()
        self.assertEqual(len(self.fan_off_times(m)), 1)
        # The print's own speed is still what RESUME puts back.
        self.assertEqual(m.m1.saved_fan, 0.4)

    def test_a_fan_that_was_off_is_not_turned_on(self):
        m, he = self.paused(fan=0.)
        self.assertEqual(m.gcode.scripts, ['M107'])
        self.assertFalse(m.m1.get_status(0)['cooling'])

    def test_the_ooze_point_is_the_nozzle_wipes(self):
        # One number for "the nozzle no longer oozes": the wipe's.
        m, he = self.paused(sections={'nozzle_wipe_smart':
                                      {'fan_off_temp': 150.}})
        self.assertEqual(m.m1.pause_cool_temp, 150.)
        self.assertEqual(Machine().m1.pause_cool_temp, 160.)

    def test_a_cool_nozzle_turns_the_fan_off_at_once(self):
        m, he = self.paused(temp=100.)
        self.assertEqual(m.gcode.scripts[-1], 'M107')
        self.assertFalse(m.m1.get_status(0)['cooling'])

    def test_no_fan_with_the_toolhead_gone(self):
        m = Machine()
        m.heaters.heaters['extruder'].temp = 215.
        m.start_print()
        m.pr.is_paused = True
        m.mcu.non_critical_disconnected = True
        m.run('MACHINE_PAUSED', REASON='toolhead_disconnected', FAN=0.4)
        self.assertFalse(m.m1.get_status(0)['cooling'])

    def test_waits_for_the_gcode_lock(self):
        m, he = self.paused()
        he.temp = 100.
        m.gcode.busy = True
        m.poll()
        self.assertEqual(self.fan_off_times(m), [])
        m.gcode.busy = False
        m.poll()
        self.assertEqual(len(self.fan_off_times(m)), 1)

    def test_ends_when_the_hot_end_is_heated_again(self):
        # A filament load or a purge while paused: the fan must not fight it.
        m, he = self.paused()
        he.target = 230.
        m.poll()
        self.assertEqual(len(self.fan_off_times(m)), 1)

    def test_ends_after_the_longest_time(self):
        m, he = self.paused(pause_cool_max_time=600.)
        m.reactor.now += 599.
        m.poll()
        self.assertEqual(self.fan_off_times(m), [])
        m.reactor.now += 2.
        m.poll()
        self.assertEqual(len(self.fan_off_times(m)), 1)

    def test_resume_takes_the_fan_back(self):
        # RESUME sets the print's speed itself; a later poll must not stop it.
        m, he = self.paused()
        m.run('MACHINE_RESUMED')
        m.pr.is_paused = False
        he.temp = 100.
        m.poll()
        self.assertEqual(self.fan_off_times(m), [])

    def test_can_be_disabled(self):
        m, he = self.paused(pause_cool_temp=0.)
        self.assertEqual(m.gcode.scripts[-1], 'M107')

    def test_a_second_pause_does_not_restart_it(self):
        m, he = self.paused()
        n = len(m.gcode.scripts)
        m.run('MACHINE_PAUSED', REASON='plate_removed')
        self.assertEqual(len(m.gcode.scripts), n)


class TestToolheadHotplug(unittest.TestCase):
    """Is a toolhead connect a toolhead plugged in (load its filament) or
    the toolhead MCU coming up at a Klipper start (do nothing)?"""

    def connect(self, m):
        m.run('MACHINE_TOOLHEAD_CONNECTED')
        return m.m1.get_status(0)['toolhead_hotplug']

    def test_the_connect_at_a_klipper_start(self):
        m = Machine()
        m.mcu.non_critical_disconnected = True      # deferred
        m.poll()
        m.reactor.now += 2.
        m.mcu.non_critical_disconnected = False
        self.assertFalse(self.connect(m))

    def test_unplugged_and_plugged_back_in(self):
        m = Machine()
        self.assertFalse(self.connect(m))
        m.reactor.now += 100.
        m.mcu.non_critical_disconnected = True
        m.poll()
        m.reactor.now += 30.
        m.mcu.non_critical_disconnected = False
        self.assertTrue(self.connect(m))

    def test_first_plugged_in_after_the_start(self):
        # Booted with no toolhead; it is plugged in a minute later.
        m = Machine(toolhead_startup_grace=15.)
        m.mcu.non_critical_disconnected = True
        m.poll()
        m.reactor.now += 60.
        m.mcu.non_critical_disconnected = False
        self.assertTrue(self.connect(m))

    def test_once_per_plug_in(self):
        m = Machine()
        self.connect(m)
        m.mcu.non_critical_disconnected = True
        m.poll()
        m.mcu.non_critical_disconnected = False
        self.assertTrue(self.connect(m))
        self.assertFalse(self.connect(m))


class TestHeatWait(unittest.TestCase):
    def test_returns_at_temperature(self):
        m = Machine()
        bed = m.heaters.heaters['heater_bed']
        bed.target = 60.
        m.reactor.schedule(10., lambda: setattr(bed, 'temp', 59.5))
        m.run('MACHINE_HEAT_WAIT', HEATER='heater_bed', MINIMUM=59)

    def test_plate_pulled_ends_the_bed_wait(self):
        # Audit F12: TEMPERATURE_WAIT on a bed whose plate was pulled (the
        # detector zeroes the target too) never finishes.
        m = Machine()
        bed = m.heaters.heaters['heater_bed']
        bed.target = 60.
        m.reactor.schedule(5., lambda: setattr(m.detector, 'present', False))
        with self.assertRaises(CommandError) as e:
            m.run('MACHINE_HEAT_WAIT', HEATER='heater_bed', MINIMUM=59)
        self.assertIn('Fit the build plate', str(e.exception))

    def test_target_zeroed_ends_it(self):
        m = Machine()
        bed = m.heaters.heaters['heater_bed']
        bed.target = 60.
        m.reactor.schedule(5., lambda: setattr(bed, 'target', 0.))
        with self.assertRaises(CommandError):
            m.run('MACHINE_HEAT_WAIT', HEATER='heater_bed', MINIMUM=59)

    def test_toolhead_pulled_ends_the_hotend_wait(self):
        m = Machine()
        he = m.heaters.heaters['extruder']
        he.target = 215.
        m.reactor.schedule(5., lambda: setattr(
            m.mcu, 'non_critical_disconnected', True))
        with self.assertRaises(CommandError) as e:
            m.run('MACHINE_HEAT_WAIT', HEATER='extruder', MINIMUM=214)
        self.assertIn('Connect the toolhead', str(e.exception))

    def test_the_bed_does_not_need_the_toolhead(self):
        m = Machine()
        bed = m.heaters.heaters['heater_bed']
        bed.target = 60.
        m.mcu.non_critical_disconnected = True
        m.reactor.schedule(5., lambda: setattr(bed, 'temp', 60.))
        m.run('MACHINE_HEAT_WAIT', HEATER='heater_bed', MINIMUM=59)

    def test_abortable(self):
        m = Machine()
        he = m.heaters.heaters['extruder']
        he.target = 215.
        m.reactor.schedule(3., lambda: setattr(m.gcode, 'abort', 'cancel'))
        with self.assertRaises(CommandError):
            m.run('MACHINE_HEAT_WAIT', HEATER='extruder', MINIMUM=214)


class TestFilePrinting(unittest.TestCase):
    """What filament_switch_sensor asks: is a print file printing?"""
    def test_cases(self):
        m = Machine()
        m.idle.status['state'] = 'Printing'      # toolhead motion
        self.assertFalse(m.m1.file_printing(0))
        m.ps.status['state'] = 'printing'
        self.assertTrue(m.m1.file_printing(0))
        m.pr.is_paused = True
        self.assertFalse(m.m1.file_printing(0))


class TestStatus(unittest.TestCase):
    def test_status_shape(self):
        m = Machine()
        st = m.m1.get_status(0)
        self.assertEqual(st['activity'], 'idle')
        self.assertTrue(all(st['ready'].values()))
        self.assertEqual(set(st['missing']), set(m1_machine.REQUIRES))
        m.run('MACHINE_STATUS')
        self.assertIn('print: ready', m.gcode.out[-1])

    def test_reasons_cover_every_code(self):
        # Every code missing() can return has a default text.
        for code in ('column_absent', 'column_unlocked', 'column_unknown',
                     'column_reading', 'plate_absent', 'plate_unknown',
                     'toolhead_disconnected', 'hotend_detecting',
                     'hotend_failed', 'filament_absent',
                     'bed_power_starting', 'bed_power_inhibited',
                     'bed_power_fault', 'fan_fault', 'plate_not_cleared'):
            self.assertIn(code, m1_machine.REASONS)


if __name__ == '__main__':
    unittest.main()
