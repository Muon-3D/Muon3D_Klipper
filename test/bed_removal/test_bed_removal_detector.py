"""bed_removal_detector: when pulling the plate pauses the print (KAN-272).

Drives the real BedRemovalDetector with fakes for the heater, reactor,
print_stats, virtual_sdcard, idle_timeout and pause_resume, and asserts the
one decision that matters for safety: whether a plate removal sends the
pause half of pause_resume.

It must when a file is really printing (KAN-79, Klipper #29). It must not
when nothing is printing but idle_timeout reads "Printing" because startup
gcode moved the toolhead (walnut, 1 Oct 2026: the stray pause disabled
every later pause-based fault path for the whole next print), nor in the
PRINT_END tail, nor once the file has finished, been cancelled or paused.
In every case the bed heater still goes off and BED_REMOVED still runs.

    python3 test/bed_removal/test_bed_removal_detector.py
"""
import os
import sys
import unittest

KLIPPY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

from extras import bed_removal_detector  # noqa: E402

REMOVED = -83.76    # what the open bed thermistor computes to
ATTACHED = 60.


class CommandError(Exception):
    pass


class FakeReactor:
    NOW = 0.
    NEVER = 9999999999999999.

    def __init__(self):
        self.now = 100.
        self.callbacks = []
        self.timers = []

    def monotonic(self):
        return self.now

    def register_timer(self, callback, waketime):
        self.timers.append(callback)
        return callback

    def register_callback(self, callback):
        self.callbacks.append(callback)


class FakeHeater:
    def __init__(self):
        self.temp = ATTACHED
        self.target = ATTACHED

    def get_temp(self, eventtime):
        return self.temp, self.target

    def set_temp(self, degrees):
        self.target = degrees


class FakeBed:
    def __init__(self):
        self.heater = FakeHeater()


class FakeGCodeCommand:
    def __init__(self, params):
        self.params = params

    def get_int(self, name, default=None, minval=None, maxval=None):
        value = int(self.params.get(name, default))
        if minval is not None and value < minval:
            raise CommandError('%s below %s' % (name, minval))
        if maxval is not None and value > maxval:
            raise CommandError('%s above %s' % (name, maxval))
        return value

    def respond_info(self, msg, log=True):
        pass


class FakeGCode:
    error = CommandError

    def __init__(self):
        self.scripts = []
        self.commands = {}

    def run_script(self, script):
        self.scripts.append(script)

    def register_command(self, name, func, desc=None):
        self.commands[name] = func

    def run(self, name, **params):
        self.commands[name](FakeGCodeCommand(params))


class FakePauseResume:
    def __init__(self):
        self.is_paused = False
        self.pauses_sent = 0

    def send_pause_command(self):
        self.pauses_sent += 1


class FakePrintStats:
    """What print_stats holds that the detector may read."""

    def __init__(self):
        self.state = 'standby'
        self.print_start_time = None

    def get_status(self, eventtime):
        return {'state': self.state}


class FakeVirtualSD:
    def __init__(self):
        self.active = False

    def is_active(self):
        return self.active


class FakeIdleTimeout:
    def __init__(self):
        self.state = 'Idle'

    def get_status(self, eventtime):
        return {'state': self.state}


class FakePrinter:
    def __init__(self, reactor, with_sdcard=True):
        self.reactor = reactor
        self.objects = {}
        self.handlers = {}
        self.with_sdcard = with_sdcard

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=KeyError):
        if name in self.objects:
            return self.objects[name]
        if default is KeyError:
            raise KeyError(name)
        return default

    def load_object(self, config, name):
        return self.objects[name]

    def register_event_handler(self, event, callback):
        self.handlers.setdefault(event, []).append(callback)


class FakeConfig:
    def __init__(self, printer):
        self.printer = printer

    def get_printer(self):
        return self.printer

    def getfloat(self, name, default=None, **kw):
        return default

    def getint(self, name, default=None, **kw):
        return default


class Machine:
    def __init__(self, with_sdcard=True):
        self.reactor = FakeReactor()
        self.printer = printer = FakePrinter(self.reactor)
        self.bed = FakeBed()
        self.gcode = FakeGCode()
        self.pause_resume = FakePauseResume()
        self.print_stats = FakePrintStats()
        self.sdcard = FakeVirtualSD()
        self.idle_timeout = FakeIdleTimeout()
        printer.objects.update({
            'heater_bed': self.bed, 'gcode': self.gcode,
            'pause_resume': self.pause_resume,
            'print_stats': self.print_stats,
            'idle_timeout': self.idle_timeout})
        if with_sdcard:
            printer.objects['virtual_sdcard'] = self.sdcard
        self.detector = bed_removal_detector.BedRemovalDetector(
            FakeConfig(printer))
        for handler in printer.handlers.get('klippy:ready', []):
            handler()

    # States the printer can be in.
    def idle_after_startup_moves(self):
        # walnut, 1 Oct: no file, but startup gcode advanced print_time.
        self.idle_timeout.state = 'Printing'
        self.print_stats.state = 'standby'
        self.sdcard.active = False

    def start_file(self):
        self.reactor.now += 10.
        self.idle_timeout.state = 'Printing'
        self.print_stats.state = 'printing'
        self.print_stats.print_start_time = self.reactor.now
        self.sdcard.active = True

    def finish_file(self, state='complete'):
        # print_stats._note_finish: the state, and no start time.
        self.print_stats.state = state
        self.print_stats.print_start_time = None
        self.sdcard.active = False

    def print_end(self):
        """What PRINT_END sends once the heaters are off."""
        self.gcode.run('SET_BED_REMOVAL_DETECTOR', PRINT_ENDING=1)

    def sample(self, temp, count=5):
        self.bed.heater.temp = temp
        for _ in range(count):
            self.reactor.now += 0.1
            self.detector.check_bed_temperature(self.reactor.now)

    def pull_plate(self):
        self.sample(REMOVED)
        self.run_callbacks()

    def refit_plate(self):
        self.sample(ATTACHED)
        self.run_callbacks()

    def run_callbacks(self):
        callbacks, self.reactor.callbacks = self.reactor.callbacks, []
        for callback in callbacks:
            callback(self.reactor.now)

    def status(self):
        return self.detector.get_status(self.reactor.now)


class PausesOnlyWhileAFileIsPrinting(unittest.TestCase):
    def setUp(self):
        self.m = Machine()

    def assert_paused(self):
        self.assertEqual(self.m.pause_resume.pauses_sent, 1)
        self.assertTrue(self.status()['removedWhilePrinting'])

    def assert_not_paused(self):
        self.assertEqual(self.m.pause_resume.pauses_sent, 0)
        self.assertFalse(self.status()['removedWhilePrinting'])

    def status(self):
        return self.m.status()

    def assert_safety_actions(self):
        # Whatever the phase: heater off, flag up, BED_REMOVED run.
        self.assertEqual(self.m.bed.heater.target, 0.)
        self.assertTrue(self.status()['bedRemoved'])
        self.assertEqual(self.m.gcode.scripts, ['BED_REMOVED'])

    def test_mid_print_removal_pauses(self):
        # KAN-79: the behaviour that must not regress.
        self.m.start_file()
        self.m.pull_plate()
        self.assert_paused()
        self.assert_safety_actions()

    def test_removal_during_print_start_pauses(self):
        # PRINT_START is a line of the file, so it is the print phase.
        self.m.start_file()
        self.m.idle_timeout.state = 'Ready'   # parked in a heat wait
        self.m.pull_plate()
        self.assert_paused()

    def test_idle_with_startup_moves_does_not_pause(self):
        # The walnut boot of 1 Oct.
        self.m.idle_after_startup_moves()
        self.m.pull_plate()
        self.assert_not_paused()
        self.assert_safety_actions()

    def test_print_end_tail_does_not_pause(self):
        self.m.start_file()
        self.m.print_end()
        self.m.pull_plate()
        self.assert_not_paused()
        self.assert_safety_actions()

    def test_after_the_file_completes_does_not_pause(self):
        # idle_timeout still reads "Printing" for ~0.5 s after the file.
        self.m.start_file()
        self.m.finish_file('complete')
        self.m.pull_plate()
        self.assert_not_paused()

    def test_after_a_cancel_does_not_pause(self):
        # CANCEL_PRINT runs PRINT_END after CANCEL_PRINT_BASE.
        self.m.start_file()
        self.m.finish_file('cancelled')
        self.m.print_end()
        self.m.pull_plate()
        self.assert_not_paused()

    def test_while_paused_does_not_pause_again(self):
        self.m.start_file()
        self.m.print_stats.state = 'paused'
        self.m.sdcard.active = False
        self.m.pause_resume.is_paused = True
        self.m.pull_plate()
        self.assertEqual(self.m.pause_resume.pauses_sent, 0)

    def test_print_stats_printing_without_an_active_file(self):
        # Belt and braces: both have to agree a file is being read.
        self.m.start_file()
        self.m.sdcard.active = False
        self.m.pull_plate()
        self.assert_not_paused()

    def test_without_virtual_sdcard_nothing_is_a_file_print(self):
        m = Machine(with_sdcard=False)
        m.start_file()
        m.pull_plate()
        self.assertEqual(m.pause_resume.pauses_sent, 0)
        self.assertFalse(m.status()['removedWhilePrinting'])


class PrintEndingBelongsToOnePrint(unittest.TestCase):
    """The PRINT_END mark must never outlive the print it was set in: a
    stale mark would silently suppress the KAN-79 pause on the next one."""

    def setUp(self):
        self.m = Machine()

    def test_next_print_pauses_again(self):
        self.m.start_file()
        self.m.print_end()
        self.m.finish_file('complete')
        self.m.start_file()
        self.m.pull_plate()
        self.assertEqual(self.m.pause_resume.pauses_sent, 1)

    def test_next_print_pauses_even_if_never_sampled_in_between(self):
        # SDCARD_PRINT_FILE resets print_stats and starts the next file in
        # one go, so the detector may never see a non-printing state.
        self.m.start_file()
        self.m.print_end()
        self.m.start_file()
        self.m.pull_plate()
        self.assertEqual(self.m.pause_resume.pauses_sent, 1)

    def test_mark_outside_a_print_is_ignored(self):
        # PRINT_END from the console, or CANCEL_PRINT's.
        self.m.idle_after_startup_moves()
        self.m.print_end()
        self.m.start_file()
        self.m.pull_plate()
        self.assertEqual(self.m.pause_resume.pauses_sent, 1)

    def test_mark_survives_a_pause_and_resume_in_the_same_print(self):
        self.m.start_file()
        self.m.print_end()
        self.m.print_stats.state = 'paused'
        self.m.print_stats.state = 'printing'   # same print_start_time
        self.m.pull_plate()
        self.assertEqual(self.m.pause_resume.pauses_sent, 0)

    def test_mark_can_be_cleared(self):
        self.m.start_file()
        self.m.print_end()
        self.m.gcode.run('SET_BED_REMOVAL_DETECTOR', PRINT_ENDING=0)
        self.m.pull_plate()
        self.assertEqual(self.m.pause_resume.pauses_sent, 1)

    def test_status_reports_the_mark(self):
        self.m.start_file()
        self.assertFalse(self.m.status()['printEnding'])
        self.m.print_end()
        self.assertTrue(self.m.status()['printEnding'])
        self.m.finish_file('complete')
        self.assertFalse(self.m.status()['printEnding'])


class DecisionIsLatchedPerRemoval(unittest.TestCase):
    def setUp(self):
        self.m = Machine()

    def test_a_later_idle_removal_overwrites_it(self):
        self.m.start_file()
        self.m.pull_plate()
        self.assertTrue(self.m.status()['removedWhilePrinting'])
        self.m.refit_plate()
        # The pause was resolved by a cancel; next removal is idle.
        self.m.pause_resume.is_paused = False
        self.m.finish_file('cancelled')
        self.m.idle_after_startup_moves()
        self.m.pull_plate()
        self.assertFalse(self.m.status()['removedWhilePrinting'])
        self.assertEqual(self.m.pause_resume.pauses_sent, 1)

    def test_reattach_leaves_it_for_bed_attached(self):
        # BED_REMOVED reads it when it renders, which can be after the
        # plate is back if the gcode mutex was busy.
        self.m.start_file()
        self.m.sample(REMOVED)
        self.m.sample(ATTACHED)
        self.assertTrue(self.m.status()['removedWhilePrinting'])


if __name__ == '__main__':
    unittest.main()
