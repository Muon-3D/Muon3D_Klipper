#!/usr/bin/env python3
"""The bed is pulled while PRINT_START homes Z: what state is left behind.

Scenario (KAN-3370 and the landing review, item 5). A print file runs
PRINT_START as its first line. Its G28 Z is descending when the plate comes
off. bed_removal_detector (its own timer) zeroes the bed heater, aborts the
homing move, sends the pause half of pause_resume (the file is active) and
queues the BED_REMOVED macro; G28 raises "Homing aborted: the bed was
removed" inside PRINT_START. The test then asks what is left:

* which state print_stats ends in, and whether a pause flag outlives it;
* whether the heaters are off;
* whether the queued BED_REMOVED (or a PAUSE) lands later, in the next job;
* whether a new job can start, with no Klipper restart, once the plate is
  refitted.

It drives klippy's real reactor, GCodeDispatch, GCodeMacro, virtual_sdcard,
print_stats, pause_resume, bed_removal_detector and m1_machine, and the
shipped PAUSE, CANCEL_PRINT, BED_REMOVED, BED_ATTACHED and _ON_PRINT_ERROR
macros (core/M1/macros), with fakes for what is hardware: the heaters, the
toolhead, the homing move (which ends on abort_active_move() exactly as
PrinterHoming's does, and raises the same error text) and the LEDs.
PRINT_START itself is a short stand-in (the shipped one needs the whole
machine): the M1 start gate, then the home that gets aborted.

Needs greenlet and jinja2. On a machine without epoll or poll (Windows) a
reactor that only sleeps stands in for the select one; the dispatch,
mutexes and greenlets are klippy's own.

    python3 test/landing/test_abort_mid_print_start.py
"""
import os
import sys
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'test', 'print_cancel'))

# Importing this puts klippy on sys.path and stubs chelper
import test_cancel_during_print_start as base  # noqa: E402
import reactor  # noqa: E402
import gcode  # noqa: E402
from extras import (bed_removal_detector, gcode_macro, m1_machine,  # noqa
                    pause_resume, print_stats, virtual_sdcard)

BED_CFG = os.path.join(ROOT, 'core', 'M1', 'macros', 'bed.cfg')
PRINT_CFG = os.path.join(ROOT, 'core', 'M1', 'macros', 'print.cfg')
HOMING_ABORTED = "Homing aborted: the bed was removed"
PULLED = -83.76
CLEANUP = 0.05
NORMAL = 25.


class PortableReactor(reactor.SelectReactor):
    """klippy's SelectReactor with the select() and the wake-up pipe
    replaced by a sleep, for a platform that has neither."""

    def __init__(self, *args, **kw):
        reactor.SelectReactor.__init__(self, *args, **kw)
        self._pipe_fds = ()

    def _dispatch_loop(self):
        import greenlet
        self._g_dispatch = greenlet.getcurrent()
        busy = True
        eventtime = self.monotonic()
        while self._process:
            timeout = self._check_timers(eventtime, busy)
            busy = False
            if timeout > 0.:
                time.sleep(min(timeout, 0.005))
            eventtime = self.monotonic()
        self._g_dispatch = None

    def finalize(self):
        self._pipe_fds = None
        reactor.SelectReactor.finalize(self)


def _needs_portable_reactor():
    import select
    return not (hasattr(select, 'epoll') or hasattr(select, 'poll'))


class Config(base.Config):
    def has_section(self, name):
        return False


class Status:
    def __init__(self, status):
        self.status = status

    def get_status(self, eventtime):
        return self.status


class Heater:
    """What bed_removal_detector and m1_machine read of a heater."""

    def __init__(self, temp, target):
        self.last_temp = temp
        self.last_temp_time = 0.
        self.target = target

    def get_temp(self, eventtime):
        return self.last_temp, self.target

    def set_temp(self, degrees):
        self.target = degrees


class HeaterObject:
    def __init__(self, heater):
        self.heater = heater

    def get_status(self, eventtime):
        return {'target': self.heater.target,
                'temperature': self.heater.last_temp}


class Heaters:
    def __init__(self, heaters):
        self.heaters = heaters

    def lookup_heater(self, name):
        return self.heaters[name]

    def set_temperature(self, heater, temp, wait=False):
        heater.target = temp

    def get_status(self, eventtime):
        return {}


class Extruder:
    def __init__(self, heater):
        self.heater = heater

    def get_status(self, eventtime):
        return {'target': self.heater.target, 'can_extrude': True,
                'temperature': self.heater.last_temp}


class Homing:
    """PrinterHoming's contract, which is all the rest of the stack sees:
    abort_active_move() ends a Z move at once, and the G28 that was running
    it raises "Homing aborted: <reason>"."""

    def __init__(self, react):
        self.reactor = react
        self.completion = None
        self.reason = None
        self.calls = []

    def abort_active_move(self, reason, z_only=True):
        self.calls.append(reason)
        if self.completion is None or self.completion.test():
            return False
        self.reason = reason
        self.completion.complete(True)
        return True


class StartMark:
    """_PRINT_START_MARK's `phase`, which m1_machine reads as preparing."""
    variables = {'phase': ''}


class Rig:
    def __init__(self):
        if _needs_portable_reactor():
            self.reactor = PortableReactor()
        else:
            self.reactor = reactor.Reactor()
        react = self.reactor
        self.printer = p = base.Printer(react)
        self.log = []          # (what, print_stats state at that time)
        self.output = []
        self.gcode = g = gcode.GCodeDispatch(p)
        p.objects['gcode'] = g
        g.register_output_handler(self.output.append)
        p.objects['webhooks'] = base.Webhooks()
        p.objects['gcode_move'] = base.GCodeMove()
        p.objects['configfile'] = base.ConfigFile()
        self.toolhead = {'homed_axes': 'xyz', 'position': {'z': 10.},
                         'axis_maximum': {'x': 200., 'y': 175., 'z': 180.}}
        p.objects['toolhead'] = Status(self.toolhead)
        self.bed = Heater(60., 65.)
        self.hotend = Heater(135., 135.)
        p.objects['heaters'] = Heaters({'heater_bed': self.bed,
                                        'extruder': self.hotend})
        p.objects['heater_bed'] = HeaterObject(self.bed)
        p.objects['extruder'] = Extruder(self.hotend)
        p.objects['gcode_macro _PRINT_START_MARK'] = self.mark = StartMark()
        self.mark.variables = {'phase': ''}
        self.homing = p.objects['homing'] = Homing(react)
        p.objects['gcode_macro'] = gcode_macro.load_config(
            Config(p, 'gcode_macro'))
        p.objects['print_stats'] = self.stats = print_stats.PrintStats(
            Config(p, 'print_stats'))
        self.sddir = base.tempfile.mkdtemp(prefix='landing_abort_')
        p.objects['virtual_sdcard'] = self.vsd = virtual_sdcard.VirtualSD(
            Config(p, 'virtual_sdcard', {'path': self.sddir,
                                         'on_error_gcode': '_ON_PRINT_ERROR'}))
        p.objects['pause_resume'] = self.pr = pause_resume.PauseResume(
            Config(p, 'pause_resume'))
        p.objects['m1_machine'] = self.m1 = m1_machine.M1Machine(
            Config(p, 'm1_machine', {'settle_time': 0.05, 'poll_time': 0.1}))
        p.objects['bed_removal_detector'] = self.det = \
            bed_removal_detector.BedRemovalDetector(
                Config(p, 'bed_removal_detector'))
        self._commands()
        # The shipped macros that matter; PRINT_START is the stand-in
        for section, name in (('gcode_macro PAUSE', 'PAUSE'),
                              ('gcode_macro CANCEL_PRINT', 'CANCEL_PRINT'),
                              ('gcode_macro _ON_PRINT_ERROR',
                               '_ON_PRINT_ERROR')):
            self._macro(name, base.read_macro_section(PRINT_CFG, section))
        for section, name in (('gcode_macro BED_REMOVED', 'BED_REMOVED'),
                              ('gcode_macro BED_ATTACHED', 'BED_ATTACHED')):
            self._macro(name, base.read_macro_section(BED_CFG, section))
        self._macro('PRINT_START', {'abortable': 'True', 'gcode': '\n'.join([
            'MACHINE_WAIT_READY ACTION=print',
            'MARK_PHASE',
            'M140 S65',
            'M104 S135',
            'STEP NAME=before_home',
            'G28 Z',
            'STEP NAME=after_home'])})
        p.send_event('klippy:connect')
        p.send_event('klippy:ready')
        # The plate, as thermistor samples: normal until pulled
        self.pulled = False
        react.register_timer(self._feed, react.NOW)

    def _macro(self, name, opts):
        section = 'gcode_macro %s' % (name,)
        cfg = Config(self.printer, section, opts)
        self.printer.objects[section] = gcode_macro.load_config_prefix(cfg)

    def _feed(self, eventtime):
        self.bed.last_temp = PULLED if self.pulled else NORMAL
        self.bed.last_temp_time += 1.
        return eventtime + 0.05

    def note(self, what):
        self.log.append((what, self.stats.state))

    def _commands(self):
        g = self.gcode

        def noop(gcmd):
            pass
        for name in ('SAVE_GCODE_STATE', 'RESTORE_GCODE_STATE', 'G1', 'G90',
                     'G91', 'M117', 'LED_INFO', 'LED_WARNING', 'M107',
                     'SET_PRINT_STATS_INFO_'):
            g.register_command(name, noop)
        g.register_command('LED_CONNECTED',
                           lambda gcmd: self.note('LED_CONNECTED'))
        g.register_command('STEP', lambda gcmd: self.note(
            'step:' + gcmd.get('NAME')))
        g.register_command('MARK_PHASE', lambda gcmd: self.mark.variables
                           .update(phase='home'))
        g.register_command('_PRINT_START_PHASE_CLEAR', self._phase_clear)
        g.register_command('_SAVE_CONFIG_IF_PENDING', noop)
        g.register_command('PRINT_END', self._print_end)
        g.register_command('M104', self._set(self.hotend))
        g.register_command('M140', self._set(self.bed))
        g.register_command('TURN_OFF_HEATERS', self._heaters_off)
        g.register_command('G28', self._g28)

    def _set(self, heater):
        def cmd(gcmd):
            self.note(gcmd.get_command() + ' S%g' % (gcmd.get_float('S'),))
            heater.target = gcmd.get_float('S', 0.)
        return cmd

    def _heaters_off(self, gcmd):
        self.note('TURN_OFF_HEATERS')
        self.bed.target = self.hotend.target = 0.

    def _phase_clear(self, gcmd):
        self.note('PHASE_CLEAR')
        self.mark.variables = {'phase': ''}

    def _print_end(self, gcmd):
        self.note('PRINT_END')
        self.bed.target = self.hotend.target = 0.

    def _g28(self, gcmd):
        # A Z home that waits for the plate, or for an abort
        self.note('G28 start')
        self.homing.completion = self.reactor.completion()
        self.homing.reason = None
        self.homing.completion.wait(self.reactor.monotonic() + 5.)
        reason, self.homing.reason = self.homing.reason, None
        self.homing.completion = None
        if reason is not None:
            # An aborted HomingMove still has to wait_moves(), stop the
            # endstops (home_wait: an MCU round trip each) and resync the
            # toolhead before it raises, so the reactor runs other greenlets
            # meanwhile. CLEANUP is the time that takes, 50 ms here.
            self.reactor.pause(self.reactor.monotonic() + CLEANUP)
            # homing_scope: a failed G28 leaves no axis homed
            self.toolhead['homed_axes'] = ''
            self.note('G28 aborted')
            raise gcmd.error("Homing aborted: %s" % (reason,))
        self.note('G28 done')

    # Driving
    def start_file(self, name, lines):
        with open(os.path.join(self.sddir, name), 'w') as f:
            f.write('\n'.join(lines) + '\n')
        self.gcode.run_script('SDCARD_PRINT_FILE FILENAME=%s' % (name,))

    def wait(self, seconds):
        self.reactor.pause(self.reactor.monotonic() + seconds)

    def wait_until(self, test, timeout=6.):
        end = self.reactor.monotonic() + timeout
        while self.reactor.monotonic() < end:
            if test():
                return True
            self.reactor.pause(self.reactor.monotonic() + 0.02)
        return False

    def run(self, scenario):
        result = {}

        def main(eventtime):
            try:
                scenario(self)
            except BaseException as e:      # reported by the test
                import traceback
                result['error'] = traceback.format_exc()
                result['exc'] = e
            finally:
                self.reactor.end()
        self.reactor.register_callback(main)
        self.reactor.run()
        self.reactor.finalize()
        base.shutil.rmtree(self.sddir, ignore_errors=True)
        if 'error' in result:
            raise AssertionError(result['error'])

    def names(self):
        return [w for w, _ in self.log]

    def errors(self):
        return [o for o in self.output if o.startswith('!!')]


def pull_during_home(rig):
    """The shared first half: a file whose first line is PRINT_START, the
    plate pulled while G28 Z is descending."""
    rig.start_file('one.gcode', ['PRINT_START', 'STEP NAME=body',
                                 'PRINT_END'])
    assert rig.wait_until(lambda: 'G28 start' in rig.names()), rig.log
    rig.wait(0.3)
    rig.pulled = True
    assert rig.wait_until(lambda: 'LED_CONNECTED' in rig.names()), rig.log
    # BED_REMOVED has run; let the work handler and m1_machine finish
    assert rig.wait_until(lambda: rig.vsd.work_timer is None), rig.log
    rig.wait(0.5)


class AbortMidPrintStart(unittest.TestCase):
    def after_failed_start(self):
        rig = Rig()
        rig.run(pull_during_home)
        return rig

    def test_print_ends_in_error_at_the_homing_line(self):
        rig = self.after_failed_start()
        self.assertEqual(rig.homing.calls, ['the bed was removed'])
        st = rig.stats.get_status(0.)
        self.assertEqual(st['state'], 'error')
        self.assertIn(HOMING_ABORTED, st['message'])
        # nothing after the failed line ran; the file stopped there
        self.assertNotIn('step:after_home', rig.names())
        self.assertNotIn('step:body', rig.names())
        self.assertNotIn('PRINT_END', rig.names())
        self.assertIsNone(rig.vsd.work_timer)
        # the failed line was not consumed
        self.assertEqual(rig.vsd.file_position, 0)

    def test_heaters_are_off(self):
        rig = self.after_failed_start()
        self.assertEqual(rig.bed.target, 0.)
        self.assertEqual(rig.hotend.target, 0.)
        # the print's own bed target is remembered for a resume
        self.assertEqual(rig.det.restore_target, 65.)

    def test_start_phase_is_cleared(self):
        rig = self.after_failed_start()
        self.assertEqual(rig.mark.variables['phase'], '')
        self.assertIn('PHASE_CLEAR', rig.names())

    def test_no_axis_is_left_homed(self):
        rig = self.after_failed_start()
        self.assertEqual(rig.toolhead['homed_axes'], '')

    def test_no_pause_stands_on_the_failed_print(self):
        # The finding of item 5. BED_REMOVED was queued on the gcode mutex
        # while PRINT_START held it, so it runs when the line ends, before
        # the work handler's on_error_gcode and while print_stats still says
        # "printing": its PAUSE marked pause_resume paused on a print that
        # was about to be "error". _ON_PRINT_ERROR now clears it (CLEAR_PAUSE),
        # as CANCEL_PRINT does.
        rig = self.after_failed_start()
        names = rig.names()
        pause = names.index('M104 S0')        # PAUSE's hot-end-off line
        err = names.index('TURN_OFF_HEATERS')
        self.assertLess(pause, err, "BED_REMOVED's PAUSE ran first")
        self.assertEqual(rig.log[pause][1], 'printing')
        self.assertEqual(rig.log[err][1], 'printing')
        self.assertFalse(rig.pr.is_paused, "pause_resume.is_paused")
        self.assertFalse(rig.pr.pause_command_sent)
        self.assertIsNone(rig.m1.pause_reason)

    def test_resume_does_not_restart_the_failed_start(self):
        # A stale pause would let RESUME re-run the file from PRINT_START
        # (the failed line was not consumed), reheating the bed on the way.
        rig = Rig()

        def scenario(r):
            pull_during_home(r)
            r.pulled = False
            r.wait(0.3)
            r.gcode.run_script('RESUME')
            r.wait(0.5)
        rig.run(scenario)
        self.assertEqual(rig.names().count('G28 start'), 1)
        self.assertIsNone(rig.vsd.work_timer)
        self.assertEqual(rig.stats.get_status(0.)['state'], 'error')
        self.assertIn('// Print is not paused, resume aborted', rig.output)
        self.assertEqual(rig.bed.target, 0.)

    def test_machine_reports_error_and_a_clean_gate(self):
        rig = self.after_failed_start()
        st = rig.m1.get_status(rig.reactor.monotonic())
        self.assertEqual(st['activity'], 'error')
        self.assertIsNone(st['gate'])
        # a job that never got past PRINT_START leaves nothing on the plate
        self.assertTrue(st['plate_cleared'])
        self.assertFalse(rig.m1.cooling is not None)

    def test_nothing_lands_later_and_a_new_job_starts(self):
        rig = Rig()

        def scenario(r):
            pull_during_home(r)
            # The plate is refitted. BED_ATTACHED is the detector's own.
            r.pulled = False
            assert r.wait_until(lambda: not r.det.bed_removed), r.log
            r.wait(0.3)
            before = len(r.log)
            r.start_file('two.gcode', ['PRINT_START', 'STEP NAME=body',
                                       'PRINT_END'])
            assert r.wait_until(lambda: r.vsd.work_timer is None), r.log
            r.wait(0.8)
            r.second = r.log[before:]
        rig.run(scenario)
        second = [w for w, _ in rig.second]
        self.assertEqual(rig.stats.get_status(0.)['state'], 'complete')
        self.assertEqual(set(rig.errors()), {'!! ' + HOMING_ABORTED})
        # job two ran start to end: homed, the body, the end
        for want in ('G28 start', 'G28 done', 'step:after_home',
                     'step:body', 'PRINT_END'):
            self.assertIn(want, second)
        # and no PAUSE or BED_REMOVED of job one came with it
        self.assertNotIn('LED_CONNECTED', [w for w in second
                                           if w == 'LED_CONNECTED'
                                           and rig.pulled])
        self.assertFalse(rig.pr.is_paused)
        self.assertFalse(rig.pr.sd_paused)
        self.assertIsNone(rig.m1.pause_reason)
        st = rig.m1.get_status(rig.reactor.monotonic())
        self.assertEqual(st['missing']['print'], [])

    def test_cancel_after_the_failure_is_clean(self):
        rig = Rig()

        def scenario(r):
            pull_during_home(r)
            r.pulled = False
            r.wait(0.3)
            r.gcode.run_script('CANCEL_PRINT')
            r.wait(0.3)
        rig.run(scenario)
        self.assertFalse(rig.pr.is_paused)
        self.assertEqual(set(rig.errors()), {'!! ' + HOMING_ABORTED})
        self.assertIn('PRINT_END', rig.names())


if __name__ == '__main__':
    unittest.main()
