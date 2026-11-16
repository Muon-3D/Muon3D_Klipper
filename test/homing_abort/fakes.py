"""Fakes shared by the KAN-3370 homing tests (abort and search cap).

They stand in for the reactor, toolhead, steppers, endstops and printer so
the real HomingMove, PrinterHoming and BedRemovalDetector run unmodified.
A move is simulated in 1 mm ticks of the moving axis (100 steps of 0.01 mm
each); a test hooks `toolhead.on_tick` to do something mid-move.
"""
import os
import sys

KLIPPY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

from extras import homing  # noqa: E402

STEP_DIST = 0.01
STEPS_PER_MM = 100       # written out, not derived from STEP_DIST
AXES = 'xyz'


class CommandError(Exception):
    pass


class Pending(Exception):
    pass


class FakeCompletion:
    """Like ReactorCompletion, but wait() raises Pending instead of
    blocking, so pump() can retry a callback that is not ready."""

    def __init__(self):
        self.done = False
        self.result = None

    def test(self):
        return self.done

    def complete(self, result):
        self.done = True
        self.result = result

    def wait(self, *args):
        if not self.done:
            raise Pending()
        return self.result


class FakeReactor:
    NOW = 0.
    NEVER = 9999999999999999.

    def __init__(self):
        self.pending = []

    def monotonic(self):
        return 100.

    def completion(self):
        return FakeCompletion()

    def register_callback(self, callback, waketime=0.):
        comp = FakeCompletion()
        self.pending.append((callback, comp))
        return comp

    def register_timer(self, callback, waketime):
        return callback

    def pump(self):
        # Run callbacks whose waits can now be satisfied
        for item in list(self.pending):
            callback, comp = item
            try:
                comp.complete(callback(None))
            except Pending:
                continue
            self.pending.remove(item)


class FakeStepper:
    def __init__(self, name, axis, start_mm):
        self.name = name
        self.axis = axis                 # 0, 1 or 2
        self.pos = int(round(start_mm * STEPS_PER_MM))   # MCU steps
        self.cmd_mm = start_mm
        self.target_steps = self.pos
        self.trig_snapshot = None

    def get_name(self):
        return self.name

    def get_step_dist(self):
        return STEP_DIST

    def is_active_axis(self, axis):
        return AXES.index(axis) == self.axis

    def get_mcu_position(self, cmd_pos=None):
        if cmd_pos is not None:
            return int(round(cmd_pos * STEPS_PER_MM))
        return self.pos

    def mcu_to_commanded_position(self, mcu_pos):
        return mcu_pos / float(STEPS_PER_MM)

    def get_commanded_position(self):
        return self.cmd_mm

    def get_past_mcu_position(self, print_time):
        # What the step history says: the trigger point if there was one,
        # otherwise the end of the move, whether or not it got there
        if self.trig_snapshot is not None:
            return self.trig_snapshot
        return self.target_steps

    def calc_position_from_coord(self, coord):
        return coord[self.axis]


class FakeEndstop:
    def __init__(self, reactor, steppers, trigger_after_mm=None,
                 wait_error=None):
        self.reactor = reactor
        self.steppers = steppers
        self.trigger_after_mm = trigger_after_mm
        self.wait_error = wait_error
        self.completion = None
        self.started = 0
        self.waited = 0
        self.triggered = False

    def get_steppers(self):
        return list(self.steppers)

    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        self.started += 1
        self.triggered = False
        self.completion = self.reactor.completion()
        return self.completion

    def home_wait(self, home_end_time):
        self.waited += 1
        if self.wait_error is not None:
            raise self.wait_error
        return 20. if self.triggered else 0.

    def trigger(self):
        self.triggered = True
        for s in self.steppers:
            s.trig_snapshot = s.pos
        self.completion.complete(False)


class FakeKinematics:
    def __init__(self, steppers):
        self.steppers = steppers

    def get_steppers(self):
        return list(self.steppers)

    def calc_position(self, kin_spos):
        pos = [None, None, None]
        for s in self.steppers:
            pos[s.axis] = kin_spos[s.name]
        return pos


class FakeToolhead:
    def __init__(self, reactor, steppers, pos):
        self.reactor = reactor
        self.steppers = steppers
        self.pos = list(pos)
        self.endstops = []
        self.kin = FakeKinematics(steppers)
        self.drip_calls = []        # (movepos, speed)
        self.set_positions = []
        self.travelled_mm = 0
        self.on_tick = None
        self.flushes = 0

    def get_position(self):
        return list(self.pos)

    def set_position(self, newpos, homing_axes=()):
        self.pos = list(newpos)
        self.set_positions.append(list(newpos))
        for s in self.steppers:
            s.cmd_mm = newpos[s.axis]

    def get_kinematics(self):
        return self.kin

    def flush_step_generation(self):
        self.flushes += 1

    def get_last_move_time(self):
        return 10.

    def dwell(self, delay):
        pass

    def drip_move(self, newpos, speed, completion):
        self.drip_calls.append((list(newpos), speed))
        deltas = [n - p for n, p in zip(newpos[:3], self.pos[:3])]
        axis = max(range(3), key=lambda i: abs(deltas[i]))
        sign = 1 if deltas[axis] > 0 else -1
        moving = [s for s in self.steppers if s.axis == axis]
        for s in moving:
            s.target_steps = s.pos + int(round(
                deltas[axis] * STEPS_PER_MM))
        self.travelled_mm = 0
        for _ in range(int(round(abs(deltas[axis])))):
            self.reactor.pump()
            if completion.test():
                break
            for s in moving:
                s.pos += sign * STEPS_PER_MM
            self.travelled_mm += 1
            for es in self.endstops:
                if (es.trigger_after_mm == self.travelled_mm
                        and not es.triggered):
                    es.trigger()
            if self.on_tick is not None:
                self.on_tick(self.travelled_mm)
        self.pos = list(newpos[:3]) + self.pos[3:]


class FakeGCode:
    def register_command(self, name, func, desc=None):
        pass


class FakeMcuObject:
    def __init__(self, name, non_critical):
        self.name = name
        self.is_non_critical = non_critical

    def get_non_critical_disconnect_event_name(self):
        return "danger:non_critical_mcu_%s:disconnected" % (self.name,)


class FakePrinter:
    command_error = CommandError

    def __init__(self, reactor):
        self.reactor = reactor
        self.objects = {}
        self.handlers = {}
        self.events = []

    def get_reactor(self):
        return self.reactor

    def get_start_args(self):
        return {}

    def is_shutdown(self):
        return False

    def lookup_object(self, name, default=KeyError):
        if name in self.objects:
            return self.objects[name]
        if default is KeyError:
            raise KeyError(name)
        return default

    def load_object(self, config, name):
        return self.objects[name]

    def lookup_objects(self, module=None):
        return list(self.objects.items())

    def register_event_handler(self, event, callback):
        self.handlers.setdefault(event, []).append(callback)

    def send_event(self, event, *params):
        self.events.append(event)
        return [cb(*params) for cb in self.handlers.get(event, [])]


class FakeConfig:
    def __init__(self, printer, values=None):
        self.printer = printer
        self.values = values or {}

    def get_printer(self):
        return self.printer

    def getfloat(self, name, default=None, **kw):
        return float(self.values.get(name, default))


class Rig:
    """A printer with one move axis, real PrinterHoming and fake motion."""

    def __init__(self, axis=2, start=160., trigger_after_mm=None,
                 n_endstops=1, options=None, wait_errors=None):
        self.reactor = FakeReactor()
        self.printer = FakePrinter(self.reactor)
        self.steppers = [FakeStepper('stepper_%s%d' % (AXES[axis], i), axis,
                                     start) for i in range(n_endstops)]
        pos = [0., 0., 0., 0.]
        pos[axis] = start
        self.toolhead = FakeToolhead(self.reactor, self.steppers, pos)
        errors = wait_errors or [None] * n_endstops
        self.endstops = [FakeEndstop(self.reactor, [s], trigger_after_mm,
                                     errors[i])
                         for i, s in enumerate(self.steppers)]
        self.toolhead.endstops = self.endstops
        self.printer.objects['toolhead'] = self.toolhead
        self.printer.objects['gcode'] = FakeGCode()
        self.phoming = homing.PrinterHoming(
            FakeConfig(self.printer, options))
        self.printer.objects['homing'] = self.phoming

    def hmove(self):
        es = [(e, 'es%d' % (i,)) for i, e in enumerate(self.endstops)]
        return homing.HomingMove(self.printer, es, self.toolhead)


def run(rig_move):
    """Call rig_move(); return (result, CommandError text or None)."""
    try:
        return rig_move(), None
    except CommandError as e:
        return None, str(e)


FAILED = []
CHECKS = []


def check(cond, what):
    print("%s  %s" % ("ok " if cond else "FAIL", what))
    CHECKS.append(what)
    if not cond:
        FAILED.append(what)


def finish():
    if FAILED:
        print("%d of %d checks FAILED" % (len(FAILED), len(CHECKS)))
        sys.exit(1)
    print("all %d checks passed" % (len(CHECKS),))
