"""contact_guard: the decisions test/klippy can only see as an exit code.

test/klippy/contact_guard*.test drive the guard inside klippy.  test_klippy
reports only whether klippy exited with an error, so this drives the real
ContactGuard with fakes and asserts what that cannot: the refusal's exact
text, the timing boundaries, and the states that make the guard inactive.

    python3 test/contact_guard/test_contact_guard.py

What test/klippy batch mode cannot cover, and so is tested only here:
CONTACT_GUARD_SIMULATE feeds samples only when it is called, starting
after the queued motion, and _sync returns at once in batch mode.  So no
batch test can see samples during a Z move (positive clearance, the lift
re-zero after an upward Z homing move), a stream that stops or lags (fail
closed), the wait after a probe trigger or a gap, or samples arriving
inside homing_override between its tares unless the test config feeds
them (contact_guard_m1home.cfg does).  The trapq fake follows
trapq_extract_old's boundary rules, which batch mode found the hard way.
"""
import os
import sys

KLIPPY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

from extras import contact_guard  # noqa: E402

# Written out, not imported: these are what the tests are about.
REFUSAL = "Nozzle is touching the plate: lift Z before moving X or Y"
THRESHOLD = 90000
SPS = 2000.
ZERO = -1000000   # raw counts with nothing touching
PRESS = -1.       # pressing lowers the raw counts on the M1


class CommandError(Exception):
    pass


class ConfigError(Exception):
    pass


class FakeConfig:
    def __init__(self, printer, values=None):
        self.printer = printer
        self.values = values or {}
    def get_printer(self):
        return self.printer
    def _get(self, name, default):
        return self.values.get(name, default)
    def getint(self, name, default=None, minval=None, maxval=None):
        return int(self._get(name, default))
    def getfloat(self, name, default=None, minval=None, maxval=None,
                 above=None):
        return float(self._get(name, default))


class FakeReactor:
    def __init__(self):
        self.now = 100.
        self.on_pause = []      # called as the guard waits, like the stream
        self.pauses = 0
    def monotonic(self):
        return self.now
    def pause(self, waketime):
        self.pauses += 1
        self.now = max(self.now, waketime)
        for cb in list(self.on_pause):
            cb()
        return self.now


class FakeGCode:
    error = CommandError
    def __init__(self):
        self.commands = {}
    def register_command(self, name, func, desc=None):
        self.commands[name] = func


class FakeMcu:
    def is_fileoutput(self):
        return False
    def estimated_print_time(self, eventtime):
        return eventtime - 90.  # reactor 100 s is print_time 10 s


class FakeSensor:
    def get_mcu(self):
        return FakeMcu()
    def get_range(self):
        return -0x800000, 0x7FFFFF      # the ADS1220's 24 bits


class FakeLoadCell:
    def __init__(self):
        self.clients = []
        self.sensor = FakeSensor()
    def add_client(self, cb):
        self.clients.append(cb)


class FakeProbe:
    def __init__(self):
        self.load_cell = FakeLoadCell()
        self.trigger_time = 0.
        self.reference_max = None
    def get_load_cell(self):
        return self.load_cell
    def get_reference_max_load_counts(self):
        return self.reference_max
    def get_last_trigger_time(self):
        return self.trigger_time
    def get_press_direction(self, zero_counts):
        return PRESS


class FakePrintStats:
    def __init__(self):
        self.state = 'standby'
    def get_status(self, eventtime):
        return {'state': self.state}


class FakeToolhead:
    def __init__(self):
        self.position = [50., 50., 5., 0.]
        self.moves = []
        self.homed_axes = 'xyz'
        self.print_time = 0.    # end of the queued motion
        self.waits = 0
    def get_kinematics(self):
        return self
    def get_steppers(self):
        return [FakeStepper('x', 'stepper_x'), FakeStepper('y', 'stepper_y'),
                FakeStepper('z', 'stepper_z')]
    def get_status(self, eventtime):
        return {'homed_axes': self.homed_axes}
    def wait_moves(self):
        self.waits += 1
    def get_position(self):
        return list(self.position)
    def move(self, newpos, speed):
        self.moves.append(list(newpos))
        self.position = list(newpos)


class FakeGCodeIO:
    def __init__(self):
        self.is_processing_data = False
        self.is_fileinput = False


class FakeGCodeMove:
    def __init__(self, toolhead):
        self.transform = toolhead
    def set_move_transform(self, transform, force=False):
        old, self.transform = self.transform, transform
        return old
    def reset_last_position(self):
        # As gcode_move does: read the position back through the chain
        self.last_position = self.transform.get_position()


class Flag:
    def __init__(self, name):
        setattr(self, name, False)


class FakePrinter:
    config_error = ConfigError
    command_error = CommandError
    def __init__(self):
        self.reactor = FakeReactor()
        self.gcode = FakeGCode()
        self.probe = FakeProbe()
        self.print_stats = FakePrintStats()
        self.toolhead = FakeToolhead()
        self.homing_override = Flag('in_script')
        self.nozzle_wipe = Flag('active')
        self.gcode_io = FakeGCodeIO()
        self.objects = {
            'gcode': self.gcode, 'load_cell_probe': self.probe,
            'print_stats': self.print_stats, 'toolhead': self.toolhead,
            'gcode_move': FakeGCodeMove(self.toolhead),
            'homing_override': self.homing_override,
            'nozzle_wipe_smart': self.nozzle_wipe,
            'gcode_io': self.gcode_io,
            'stepper_enable': FakeStepperEnable(),
        }
        self.handlers = {}
    def get_reactor(self):
        return self.reactor
    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *args):
        for cb in self.handlers.get(event, []):
            cb(*args)
    def get_start_args(self):
        return {}


class FakeStepper:
    def __init__(self, axis, name=None):
        self.axis = axis
        self.name = name or 'stepper_' + axis
    def get_name(self):
        return self.name
    def is_active_axis(self, axis):
        return axis == self.axis
    def get_step_dist(self):
        return 0.0025
    def mcu_to_commanded_position(self, mcu_pos):
        return mcu_pos * 0.0025


class FakeEnableTracking:
    def __init__(self):
        self.callbacks = []
    def register_state_callback(self, cb):
        self.callbacks.append(cb)


class FakeStepperEnable:
    def __init__(self):
        self.lines = {}
    def lookup_enable(self, name):
        return self.lines.setdefault(name, FakeEnableTracking())


class FakeStepperPosition:
    def __init__(self, axis, travel):
        # travel > 0 is down (toward the plate), < 0 up
        self.stepper = FakeStepper(axis)
        self.start_pos = 100000
        self.trig_pos = self.start_pos - int(round(travel / 0.0025))


class FakeHomingMove:
    """What homing_move_end sees: how far each stepper went to trigger."""
    def __init__(self, axis='z', travel=5.):
        self.stepper_positions = [FakeStepperPosition(axis, travel)]


class FakeHomingState:
    def __init__(self, axes):
        self.axes = axes
    def get_axes(self):
        return self.axes


class Rig:
    """A guard wired to fakes, with a sample clock."""
    def __init__(self, values=None, z=5.):
        self.printer = FakePrinter()
        self.printer.toolhead.position[2] = z
        self.guard = contact_guard.load_config(
            FakeConfig(self.printer, values))
        self.printer.send_event("klippy:connect")
        self.printer.send_event("klippy:ready")
        self.n = 0              # samples fed; t = 10 + n / SPS
        self.drift = 0.
    @property
    def t(self):
        return 10. + self.n / SPS
    def feed(self, load, duration, drift=0.):
        """Feed `duration` s of samples at SPS, pressing `load` counts."""
        rows = []
        for i in range(int(round(duration * SPS))):
            self.drift += drift / SPS
            rows.append([self.t, None,
                         int(round(ZERO + self.drift + PRESS * load)), None])
            self.n += 1
        cb = self.printer.probe.load_cell.clients[0]
        for i in range(0, len(rows), 40):
            assert cb({'data': rows[i:i + 40], 'errors': 0, 'overflows': 0})
    def status(self):
        return self.guard.get_status(self.printer.reactor.monotonic())
    def move(self, x, y, z, e=0.):
        self.guard.move([x, y, z, e], 100.)
    def refused(self, x, y, z, e=0.):
        try:
            self.move(x, y, z, e)
        except CommandError as err:
            self.report(err)
            return str(err)
        return None
    def report(self, err, levels=1):
        """gcode.py sends gcode:command_error while handling the error,
        once per macro level it passes through"""
        for _ in range(levels):
            try:
                raise err
            except CommandError:
                self.printer.send_event("gcode:command_error")
    def homing_move(self, axis='z', travel=5.):
        hmove = FakeHomingMove(axis, travel)
        self.printer.send_event("homing:homing_move_begin", hmove)
        self.printer.send_event("homing:homing_move_end", hmove)
    def set_position(self, at=None):
        # toolhead.set_position marks the trapq at the current print_time;
        # the samples that follow are newer.  print_time is then put back
        # so later syncs do not wait for motion that never ran.
        th = self.printer.toolhead
        print_time, th.print_time = th.print_time, (self.t if at is None
                                                    else at)
        self.printer.send_event("toolhead:set_position")
        th.print_time = print_time
    def home_z(self, axes=(2,)):
        """G28 Z through home_rails: resets the position, then homes."""
        hs = FakeHomingState(list(axes))
        self.printer.send_event("homing:home_rails_begin", hs, [])
        self.set_position()
        self.printer.send_event("homing:home_rails_end", hs, [])
    def z_motor(self, enabled):
        """SET_STEPPER_ENABLE STEPPER=stepper_z ENABLE=..."""
        line = self.printer.objects['stepper_enable'].lookup_enable(
            'stepper_z')
        for cb in line.callbacks:
            cb(self.t, enabled)
    def in_contact(self):
        self.feed(0, 0.5)
        self.feed(200000, 0.05)
        assert self.status() == {'contact': True, 'load': 200000,
                                 'active': True, 'enabled': True}, \
            self.status()


def check(cond, what):
    if not cond:
        raise AssertionError(what)
    print("ok  " + what)


def test_contact_hysteresis():
    r = Rig()
    r.feed(0, 0.5)
    check(r.status()['active'] and not r.status()['contact'],
          "active and clear once samples arrive")
    # N samples at 2000/s span (N - 1) * 0.5 ms
    r.feed(200000, 0.02)         # samples at 0 .. 19.5 ms
    check(not r.status()['contact'], "19.5 ms above threshold: no contact")
    r.feed(200000, 0.0005)       # the sample at 20 ms
    check(r.status()['contact'], "20 ms above threshold: contact")
    r.feed(THRESHOLD * 0.5 + 1000, 0.5)
    check(r.status()['contact'], "just above half the threshold: held")
    r.feed(THRESHOLD * 0.5 - 1000, 0.05)
    check(r.status()['contact'], "49.5 ms below half: still contact")
    r.feed(THRESHOLD * 0.5 - 1000, 0.0005)
    check(not r.status()['contact'], "50 ms below half: released")
    r.feed(THRESHOLD - 1000, 0.1)
    check(not r.status()['contact'], "just under the threshold: no contact")


def test_moves_in_contact():
    r = Rig()
    r.in_contact()
    check(r.refused(60, 50, 5) == REFUSAL, "X move refused, exact message")
    check(r.refused(50, 60, 5) == REFUSAL, "Y move refused")
    check(r.refused(60, 60, 4) == REFUSAL, "XY move lowering Z refused")
    check(r.printer.toolhead.moves == [],
          "nothing reached the toolhead while refusing")
    check(r.refused(50, 50, 6) is None, "Z lift allowed")
    check(r.refused(50, 50, 5.5) is None, "Z lowering alone allowed")
    check(r.refused(50, 50, 5.5, 1.) is None, "E move allowed")
    check(r.refused(60, 60, 6) is None, "XY move raising Z allowed")
    check(r.printer.toolhead.moves[-1] == [60, 60, 6, 0.],
          "allowed moves reach the next transform unchanged")
    r.feed(0, 0.1)
    check(r.refused(70, 70, 6) is None, "XY allowed once released")


def test_inactive_states():
    r = Rig()
    r.in_contact()
    for state in ('printing', 'paused'):
        r.printer.print_stats.state = state
        check(r.status() == {'contact': False, 'load': 200000,
                             'active': False, 'enabled': True},
              "inactive while print_stats is %s" % state)
        check(r.refused(60, 50, 5) is None, "XY allowed while %s" % state)
    r.printer.print_stats.state = 'complete'
    check(r.status()['contact'], "active again once the print completes")
    r.printer.send_event("homing:home_rails_begin", None, [])
    check(not r.status()['active'], "inactive while homing rails")
    r.printer.send_event("gcode:command_error")
    check(r.status()['active'], "a failed G28 does not leave it inactive")
    r.printer.homing_override.in_script = True
    check(not r.status()['active'], "inactive inside homing_override")
    r.printer.homing_override.in_script = False
    r.printer.nozzle_wipe.active = True
    check(not r.status()['active'], "inactive during NOZZLE_WIPE_SMART")
    r.printer.nozzle_wipe.active = False
    r.printer.reactor.now += 1.01
    check(not r.status()['active'], "inactive with no samples for 1 s")
    r.feed(200000, 0.05)
    check(r.status()['contact'], "active again when samples resume")


def test_homing_move():
    r = Rig()
    r.feed(0, 0.5)
    hmove = FakeHomingMove('z', 5.)
    r.printer.send_event("homing:homing_move_begin", hmove)
    check(not r.status()['active'], "inactive during a homing move")
    r.feed(200000, 0.3)          # a tap: ignored, not contact or drift
    r.printer.send_event("homing:homing_move_end", hmove)
    # 10 ms is shorter than the 50 ms a real contact takes to release
    r.feed(0, 0.01)
    check(r.status() == {'contact': False, 'load': 0, 'active': True,
                         'enabled': True},
          "samples taken during the move were not contact")


def test_trigger_holdoff():
    r = Rig()
    r.feed(0, 0.5)
    r.printer.probe.trigger_time = r.t
    r.feed(200000, 0.5)          # ring: 0 .. 499.5 ms after the trigger
    check(r.status() == {'contact': False, 'load': 0, 'active': False,
                         'enabled': True},
          "inactive and ignoring load for 0.5 s after a trigger")
    # An XY move in the hold-off waits it out and decides on what follows
    r.printer.reactor.on_pause.append(lambda: r.feed(0, 0.005))
    check(r.refused(60, 50, 5) is None and r.printer.reactor.pauses > 0,
          "XY in the hold-off waits for it to end; clear after: allowed")
    r.printer.reactor.on_pause[:] = []
    r = Rig()
    r.feed(0, 0.5)
    r.printer.probe.trigger_time = r.t
    r.feed(200000, 0.5)
    r.feed(200000, 0.02)         # 500 .. 519.5 ms
    check(r.status()['active'] and not r.status()['contact'],
          "active at 0.5 s; contact needs 20 ms from there")
    r.feed(200000, 0.0005)
    check(r.status()['contact'], "load past the hold-off is contact")


class Cmd:
    def __init__(self, **params):
        self.params = params
        self.info = []
    def get_int(self, name, default, minval=None, maxval=None):
        return self.params.get(name, default)
    def respond_info(self, msg):
        self.info.append(msg)


def test_enable():
    r = Rig()
    r.in_contact()
    r.guard.cmd_SET_CONTACT_GUARD(Cmd(ENABLE=0))
    check(r.status() == {'contact': False, 'load': 200000, 'active': False,
                         'enabled': False}, "ENABLE=0 makes it inactive")
    check(r.refused(60, 50, 5) is None, "XY allowed with ENABLE=0")
    r.guard.cmd_SET_CONTACT_GUARD(Cmd(ENABLE=1))
    check(r.refused(70, 50, 5) == REFUSAL, "ENABLE=1 refuses again")


def test_drift():
    # Raw zero drift, measured at ~9000 counts/s.  Toward unloading it is
    # followed without limit; toward pressing up to max_absorbed_counts
    # since the last real zero, and past that it reads as load
    r = Rig()
    r.feed(0, 1.)
    worst = 0
    for _ in range(60):
        r.feed(0, 1., drift=9000.)          # raw up: unloading on the M1
        worst = max(worst, abs(r.status()['load']))
        if r.status()['contact']:
            raise AssertionError("drift read as contact")
    check(worst < THRESHOLD / 4,
          "9000 counts/s unloading drift for 60 s is followed inside the"
          " band (worst %d)" % worst)
    for _ in range(4):
        r.feed(0, 1., drift=-9000.)         # 36000 back: within the cap
        worst = max(worst, abs(r.status()['load']))
    check(worst < THRESHOLD / 4 and not r.status()['contact'],
          "36000 counts of pressing drift is still followed (worst %d)"
          % worst)


def test_relock():
    r = Rig()
    r.feed(0, 0.5)
    r.feed(40000, 0.4995)        # outside the band, under the threshold
    check(r.status()['load'] == 40000, "a lingering offset is not absorbed"
          " before 0.5 s")
    r.feed(40000, 0.001)         # the sample at 0.5 s re-zeroes
    r.feed(40000, 0.0005)
    check(abs(r.status()['load']) < 1000, "and is re-zeroed at 0.5 s")
    r.feed(40000 + 200000, 0.05)
    check(r.status()['contact'], "contact is measured from the new zero")
    # A zero taken while pressing reads strongly negative once released
    r = Rig()
    r.feed(200000, 0.5)          # first samples are under load
    r.feed(0, 0.6)
    check(not r.status()['contact'], "release of a loaded zero is not"
          " contact")
    r.feed(200000, 0.05)
    check(r.status()['contact'], "after re-zeroing, a press is contact")


def test_homing_reanchors():
    r = Rig()
    r.feed(0, 0.5)
    r.feed(200000, 0.05)
    check(r.status()['contact'], "contact")
    # A long contact let the zero drift 150000 counts in the pressing
    # direction.  Lifting leaves a reading that looks like contact
    # forever, until a homing or probing move re-zeroes it.
    r.drift = 150000. * PRESS
    r.feed(0, 0.2)
    check(r.status()['contact'], "stale zero still reads contact")
    r.homing_move('z', 5.)
    r.feed(0, 0.1)
    check(not r.status()['contact'] and abs(r.status()['load']) < 1000,
          "a Z homing move that travelled 5 mm re-zeroes on what it read"
          " before moving")


def test_sync_before_deciding_near_the_plate():
    # Codex P1: a queued descent then an XY move.  The XY move is checked
    # as it enters the lookahead, before the descent has run, so the
    # guard must wait for the motion and the samples that follow it.
    r = Rig(z=0.5)
    r.feed(0, 0.5)
    th = r.printer.toolhead
    th.print_time = r.t + 0.1        # the descent ends 100 ms from now
    # The stream, during the wait, shows the nozzle reaching the plate
    r.printer.reactor.on_pause.append(lambda: r.feed(200000, 0.005))
    check(r.refused(60, 50, 0.5) == REFUSAL,
          "XY after a queued descent waits and is refused")
    check(th.waits == 1, "it waited for the queued moves to execute")
    check(r.guard.data_time >= th.print_time + 0.02,
          "and for samples 20 ms past the end of the motion")
    # Same, but the descent stops clear of the plate
    r = Rig(z=0.5)
    r.feed(0, 0.5)
    r.printer.toolhead.print_time = r.t + 0.1
    r.printer.reactor.on_pause.append(lambda: r.feed(0, 0.005))
    check(r.refused(60, 50, 0.5) is None,
          "XY after a queued descent that stays clear is allowed")


def test_no_stall_away_from_the_plate():
    r = Rig(z=5.)
    r.feed(0, 0.5)
    check(r.refused(60, 50, 5) is None and r.printer.toolhead.waits == 0
          and r.printer.reactor.pauses == 0,
          "XY at Z 5 does not wait (sync_below_z 3)")
    r = Rig(z=2.9)
    r.feed(0, 0.5)
    r.printer.reactor.on_pause.append(lambda: r.feed(0, 0.005))
    check(r.refused(60, 50, 2.9) is None and r.printer.toolhead.waits == 1,
          "XY at Z 2.9 waits")
    r = Rig(z=5.)
    r.printer.toolhead.homed_axes = 'xy'
    r.feed(0, 0.5)
    r.printer.reactor.on_pause.append(lambda: r.feed(0, 0.005))
    check(r.refused(60, 50, 5) is None and r.printer.toolhead.waits == 1,
          "XY with Z unhomed waits")
    r = Rig(z=0.5)
    r.feed(0, 0.5)
    check(r.refused(50, 50, 1) is None and r.refused(50, 50, 0.5) is None
          and r.printer.toolhead.waits == 0, "Z-only moves never wait")
    r = Rig(z=0.5)
    r.feed(0, 0.5)
    r.printer.print_stats.state = 'printing'
    check(r.refused(60, 50, 0.5) is None and r.printer.toolhead.waits == 0,
          "a print is never stalled")


def test_sync_gives_up_without_samples():
    # Review #86 R4-1b: the wait used to give up and decide on samples
    # from before the motion.  It now fails closed.
    r = Rig(z=0.5)
    r.feed(0, 0.5)
    r.printer.reactor.now = 90. + r.t   # the MCU clock is at the data
    start = r.printer.reactor.now
    r.printer.toolhead.print_time = r.t + 0.1
    check(r.refused(60, 50, 0.5) == contact_guard.NO_DATA,
          "no samples after the motion: refused (no current data)")
    waited = r.printer.reactor.now - start
    # 0.1 s of motion + 0.025 s for on_time and margin + the 0.5 s timeout
    check(0.6 <= waited < 0.65,
          "after waiting 0.5 s past when the samples were due (waited"
          " %.3f s)" % waited)
    # A stream that lags (1 ms of data per 5 ms) never covers the move
    r = Rig(z=0.5)
    r.feed(0, 0.5)
    r.printer.reactor.now = 90. + r.t
    r.printer.toolhead.print_time = r.t + 0.3
    r.printer.reactor.on_pause.append(lambda: r.feed(0, 0.001))
    check(r.refused(60, 50, 0.5) == contact_guard.NO_DATA,
          "a lagging stream that does not reach the move: refused")



def test_sync_waits_out_the_ring():
    r = Rig(z=0.5)
    r.feed(0, 0.5)
    r.printer.probe.trigger_time = r.t
    r.printer.toolhead.print_time = r.t
    r.printer.reactor.on_pause.append(lambda: r.feed(200000, 0.005))
    check(r.refused(60, 50, 0.5) == REFUSAL,
          "XY just after a tap waits out the 0.5 s ring, then decides")
    check(r.guard.data_time >= r.printer.probe.trigger_time + 0.52,
          "on samples from after the hold-off")


def test_streamed_print():
    # Codex P2: a print streamed over the G-code pty never sets print_stats
    r = Rig()
    r.in_contact()
    io = r.printer.gcode_io
    io.is_processing_data = True
    check(not r.status()['active'], "inactive while pty input is running")
    check(r.refused(60, 50, 5) is None, "XY allowed in a streamed print")
    io.is_processing_data = False
    r.printer.reactor.now += 4.9
    r.feed(200000, 0.05)
    check(not r.status()['active'],
          "still a job 4.9 s after the last pty batch")
    r.printer.reactor.now += 0.2
    check(r.status()['contact'], "active again 5 s after the stream stops")
    r = Rig()
    r.in_contact()
    r.printer.gcode_io.is_fileinput = True
    r.printer.gcode_io.is_processing_data = True
    check(r.refused(60, 50, 5) == REFUSAL,
          "batch-mode file input is not a streamed job")


def test_requires_load_cell_probe():
    p = FakePrinter()
    del p.objects['load_cell_probe']
    contact_guard.load_config(FakeConfig(p))
    try:
        p.send_event("klippy:connect")
    except ConfigError as e:
        check("[load_cell_probe]" in str(e),
              "config error without [load_cell_probe]")
        return
    raise AssertionError("no config error without [load_cell_probe]")


class TrapQMove:
    pass


class ReplayTrapQ:
    """The executed toolhead trapq, in load-cell print time.

    Constant-velocity moves only.  `z_moves` (with `axis`) are short 1 mm
    moves along one axis; `add_move` records a real displacement, so the
    position follows it.  FORCE_MOVE and a hand on the bed never appear
    here, exactly as in klippy.
    """
    def __init__(self, pos=(50., 50., 21.6)):
        self.start = tuple(pos)
        self.moves = []
        self.z_moves = []
        self.axis = 'z'
    def add_move(self, begin, duration, dx=0., dy=0., dz=0.):
        pos = self.position(float('inf'))
        self.moves.append((begin, duration, pos, (dx, dy, dz)))
    def add_reset(self, begin, z):
        """SET_KINEMATIC_POSITION: a zero-length marker at a new Z."""
        x, y, _ = self.position(float('inf'))
        self.moves.append((begin, 0., (x, y, z), (0., 0., 0.)))
    def _all(self):
        moves = list(self.moves)
        for begin, finish in self.z_moves:
            d = {'x': (1., 0., 0.), 'y': (0., 1., 0.),
                 'z': (0., 0., 1.)}[self.axis]
            moves.append((begin, finish - begin, self.start,
                          tuple(-c for c in d)))
        return sorted(moves)
    def position(self, t):
        pos = self.start
        for begin, duration, start, d in self._all():
            if begin >= t:  # as trapq_extract_old
                break
            frac = (1. if not duration
                    else max(0., min(1., (t - begin) / duration)))
            pos = tuple(s + c * frac for s, c in zip(start, d))
        return pos
    def get_trapq_position(self, t):
        return self.position(t), 0.
    def extract_trapq(self, start, end):
        moves = []
        for begin, duration, s, d in self._all():
            if begin < end and begin + duration > start:
                dist = sum(c * c for c in d) ** .5
                m = TrapQMove()
                m.print_time, m.move_t = begin, duration
                m.start_v = dist / duration if duration else 0.
                m.accel = 0.
                m.start_x, m.start_y, m.start_z = s
                m.x_r, m.y_r, m.z_r = ((c / dist for c in d) if dist
                                       else (0., 0., 0.))
                moves.append(m)
        return moves, []


def replay_motion(r, pos=None):
    trapq = ReplayTrapQ(pos or (50., 50., r.printer.toolhead.position[2]))
    class Report:
        dtrapqs = {'toolhead': trapq}
    r.printer.objects['motion_report'] = Report()
    return trapq


def test_stationary_contact_stays_latched():
    # KAN-828: the toolhead trapq is still, the load is a sustained real
    # touch (FORCE_MOVE of a Z stepper, or the bed moved by hand)
    r = Rig(z=0.5)
    replay_motion(r)
    r.feed(0, 0.5)
    r.feed(200000, 0.05)
    check(r.status()['contact'], 'stationary touch is contact at 50 ms')
    r.feed(200000, 5.)
    check(r.status()['contact'] and r.status()['load'] == 200000,
          'a still toolhead never ages a held touch into the zero (5 s)')
    check(r.refused(60, 50, 0.5) == REFUSAL,
          'XY still refused while the stationary touch remains')
    r.feed(0, 0.1)
    check(not r.status()['contact'] and r.refused(60, 50, 0.5) is None,
          'and allowed once the load falls away')


def test_direct_stepper_contact_stays_latched():
    # FORCE_MOVE runs on its own trapq: the toolhead trapq shows only the
    # XY move that ended long before, and reports the toolhead high above
    # the plate, while the stepper drove the nozzle into it in steps.
    for z in (0.5, 21.6):
        r = Rig(z=z)
        trapq = replay_motion(r)
        r.feed(0, 0.5)
        trapq.add_move(r.t - 2., 0.2, dx=10.)
        for load in (240000, 480000, 717295):
            r.feed(load, 0.1)
        r.feed(717295, 3.)
        check(r.status()['contact'] and r.status()['load'] == 717295,
              'FORCE_MOVE contact at toolhead Z %.1f stays latched' % z)
        check(r.refused(60, 50, z) == REFUSAL,
              'and XY is refused at toolhead Z %.1f' % z)
        # A FORCE_MOVE back off the plate is not seen in the trapq, but
        # the load falling is a normal release
        r.feed(0, 0.06)
        check(not r.status()['contact'] and r.refused(60, 50, z) is None,
              'FORCE_MOVE back off releases at toolhead Z %.1f' % z)


def test_unlock_offset_needs_explicit_rezero():
    # KAN-742 incident envelope: three 16 mm FORCE_MOVE filament retracts
    # left a 717295-count offset at Z=21.6 with no Z motion.  The guard
    # cannot tell that from a touch, so it stays latched until told.
    r = Rig(z=21.6)
    replay_motion(r)
    r.feed(0, 0.5)
    for load, duration in ((240000, 0.10), (480000, 0.10),
                           (717295, 2.)):
        r.feed(load, duration)
    check(r.status()['contact'], 'steady unlock offset stays latched')
    check(r.refused(60, 50, 21.6) == REFUSAL, 'XY refused until re-zeroed')
    r.home_z()
    cmd = Cmd(REZERO=1, FORCE=1)
    r.guard.cmd_SET_CONTACT_GUARD(cmd)
    check(r.status() == {'contact': False, 'load': 0, 'active': True,
                         'enabled': True}, 'REZERO=1 at Z 21.6 re-zeroes')
    check('load was 717295' in cmd.info[0], 'and reports what it absorbed')
    r.feed(717295, 0.1)
    check(abs(r.status()['load']) < 1000 and r.refused(60, 50, 21.6) is None,
          'XY resumes after the explicit re-zero')
    r.feed(917295, 0.025)
    check(r.status()['contact'] and r.status()['load'] == 200000,
          'new contact detects 200000 counts above the re-zeroed load')
    check(r.refused(70, 50, 21.6) == REFUSAL,
          'new real contact still refuses XY after the re-zero')


def rezero_error(r, force=1):
    """REZERO=1 (FORCE=1 by default: these offsets are at contact level)"""
    try:
        r.guard.cmd_SET_CONTACT_GUARD(Cmd(REZERO=1, FORCE=force))
    except CommandError as err:
        r.report(err)
        return str(err)
    return None


def test_rezero_is_bounded():
    def held(z=21.6):
        r = Rig(z=z)
        r.home_z()
        r.feed(0, 0.5)
        r.feed(717295, 1.)
        return r
    r = held(z=0.5)
    err = rezero_error(r)
    check(err is not None and 'Z 10.0 or higher' in err,
          'REZERO refused near the plate: %s' % err)
    check(r.status()['contact'] and r.refused(60, 50, 0.5) == REFUSAL,
          'and the contact stays latched')
    r = held(z=9.9)
    check(rezero_error(r) is not None, 'REZERO refused at Z 9.9')
    r = held()
    r.printer.toolhead.homed_axes = 'xy'
    check('Z homed' in (rezero_error(r) or ''), 'REZERO refused, Z unhomed')
    check(r.status()['contact'], 'and the contact stays latched')
    r = held()
    r.printer.homing_override.in_script = True
    check('homing' in (rezero_error(r) or ''), 'REZERO refused while homing')
    r = held()
    for _ in range(5):
        r.feed(717295, 0.004)
        r.feed(717295 + 3 * r.guard.band, 0.004)
    check('changing' in (rezero_error(r) or ''),
          'REZERO refused on a changing load')
    check(r.status()['contact'], 'and the contact stays latched')
    r = held()
    r.printer.reactor.now += 1.01
    check('current' in (rezero_error(r) or ''),
          'REZERO refused with a stale stream')
    r = held()
    r.printer.probe.trigger_time = r.t + 1.
    check('current' in (rezero_error(r) or ''),
          'REZERO refused inside a tap ring')
    r = held(z=10.)
    check(rezero_error(r) is None and not r.status()['contact'],
          'REZERO allowed at exactly rezero_min_z')


def test_positive_clearance_rezeroes_the_residual():
    # A touch latched at the plate, and while it was held the zero drifted
    # 150000 counts in the pressing direction.  Lifting Z drops the press,
    # and the residual then stays put while Z keeps rising: cleared.
    r = Rig(z=0.5)
    trapq = replay_motion(r)
    r.feed(0, 0.5)
    r.feed(350000, 0.3)
    check(r.status()['contact'], 'contact')
    trapq.add_move(r.t, 0.3, dz=1.5)   # 5 mm/s from Z 0.5 to 2.0
    r.feed(350000, 0.01)               # the first 0.05 mm still pressing
    r.feed(150000, 0.14)               # 0.75 mm lifted: not yet
    check(r.status()['contact'], 'still latched 0.75 mm into the lift')
    r.feed(150000, 0.08)               # past 1.0 mm of steady lift
    check(not r.status()['contact'] and abs(r.status()['load']) < 1000,
          'load fell 200000 then held over 1 mm of Z rise: re-zeroed')
    r.move(50, 50, 2.)
    check(r.refused(60, 50, 2.) is None, 'XY allowed after clearance')
    r.feed(150000 + 200000, 0.05)
    check(r.status()['contact'], 'a new press above the residual is contact')


def test_clearance_needs_steady_load_over_z():
    def latched():
        r = Rig(z=0.5)
        trapq = replay_motion(r)
        r.feed(0, 0.5)
        r.feed(350000, 0.3)
        return r, trapq
    # Lift too short
    r, trapq = latched()
    trapq.add_move(r.t, 0.16, dz=0.8)
    r.feed(350000, 0.01)
    r.feed(150000, 2.)
    check(r.status()['contact'], 'a 0.8 mm lift is not clearance')
    # A compliant contact: the load keeps falling the whole lift
    r, trapq = latched()
    trapq.add_move(r.t, 1., dz=2.)
    for i in range(100):
        r.feed(350000 - 1500 * i, 0.01)
    r.feed(200000, 2.)
    check(r.status()['contact'],
          'a load still falling with Z is not clearance')
    # No executed-motion history at all
    r = Rig(z=0.5)
    r.feed(0, 0.5)
    r.feed(350000, 0.3)
    r.feed(150000, 2.)
    check(r.status()['contact'], 'no trapq: no clearance')
    # A latched load that does not follow 1 mm or more of real Z lift is
    # not a touch: the plate would change it by millions
    for what, before in (('a lift the load ignores', None),
                         ('a fall before the lift, then held', 0.5)):
        r, trapq = latched()
        if before:
            r.feed(150000, before)
        trapq.add_move(r.t, 1., dz=5.)
        r.feed(150000, 2.)
        check(not r.status()['contact'] and abs(r.status()['load']) < 1000,
              '%s: re-zeroed' % what)
    # Review #86 R3-a: latched, a descent the reading ignores is a cell
    # that stopped measuring, not clearance
    r, trapq = latched()
    trapq.add_move(r.t, 1., dz=-1.5)
    r.feed(150000, 2.)
    check(r.status()['contact'] and r.refused(60, 50, 0.5) == REFUSAL,
          'latched, a descent the load ignores stays latched')


def test_samples_older_than_a_reset_are_no_evidence():
    # Samples trail the host: both resets of review #86 A can be handled
    # before the samples that span them arrive.  Those samples straddle
    # two frames and must not be compared across them.
    r = Rig(z=5.)
    trapq = replay_motion(r)
    r.feed(0, 0.5)
    r.feed(300000, 0.5)
    t1, t2 = r.t + 0.1, r.t + 0.4
    trapq.add_reset(t1, 7.)
    r.set_position(at=t1)
    trapq.add_reset(t2, 8.5)
    r.set_position(at=t2)
    r.feed(300000, 0.2)          # spans the jump to 7
    r.feed(200000, 0.4)          # partial relax, then spans the jump to 8.5
    check(r.status()['contact'] and r.status()['load'] == 200000,
          'samples from before a reset are not compared with Z after it')


def test_position_reset_is_not_clearance():
    # Review #86 A: two SET_KINEMATIC_POSITION jumps with a partial relax
    # between them looked like "the load fell as Z rose, then held while
    # Z rose 1.5 mm" while the nozzle still pressed 200000
    r = Rig(z=5.)
    trapq = replay_motion(r)
    r.feed(0, 0.5)
    r.feed(300000, 0.5)
    check(r.status()['contact'], 'held touch')
    trapq.add_reset(r.t, 7.)
    r.set_position()
    r.feed(300000, 0.1)
    r.feed(200000, 0.2)
    trapq.add_reset(r.t, 8.5)
    r.set_position()
    r.feed(200000, 0.2)
    check(r.status()['contact'] and r.status()['load'] == 200000,
          'position resets are not Z rise: still latched at 200000')
    check(r.refused(100, 50, 5.) == REFUSAL, 'and XY is refused')
    # A real lift after the reset is still evidence, in the new frame
    trapq.add_move(r.t, 0.3, dz=1.5)
    r.feed(300000, 0.01)
    r.feed(100000, 0.29)
    check(not r.status()['contact'],
          'a real lift after the reset still clears')


def test_rezero_needs_real_z_home():
    # Review #86 B: toolhead Z >= 10 is not proof of height after
    # SET_KINEMATIC_POSITION or a FORCE_MOVE of a Z stepper
    def held():
        r = Rig(z=20.)
        r.feed(0, 0.5)
        r.feed(200000, 1.)
        return r
    r = held()
    check('G28' in (rezero_error(r) or ''),
          'REZERO refused before any real Z home')
    r = held()
    r.home_z()
    r.set_position()                       # SET_KINEMATIC_POSITION Z=20
    check('G28' in (rezero_error(r) or ''),
          'REZERO refused after a position reset')
    check(r.status()['contact'], 'and the touch stays latched')
    r = held()
    r.home_z()
    r.printer.send_event("force_move:manual_move", FakeStepper('z'))
    check('G28' in (rezero_error(r) or ''),
          'REZERO refused after FORCE_MOVE of a Z stepper')
    r = held()
    r.home_z(axes=(0, 1))
    check('G28' in (rezero_error(r) or ''),
          'homing only X and Y does not make Z trusted')
    r = held()
    r.home_z()
    r.printer.send_event("force_move:manual_move", FakeStepper('x'))
    check(rezero_error(r) is None and not r.status()['contact'],
          'FORCE_MOVE of a non-Z stepper (the unlock retract) keeps trust')


def test_only_clear_z_homing_rezeroes():
    # Review #86 C: an X or Y homing move, or a probe started on the
    # plate, used to re-zero on the pressed load
    for axis, travel, what in (('x', 50., 'an X homing move'),
                               ('y', 50., 'a Y homing move'),
                               ('z', 0.005, 'a probe started on the plate')):
        r = Rig(z=5.)
        r.feed(0, 0.5)
        r.feed(200000, 0.5)
        r.homing_move(axis, travel)
        r.feed(200000, 0.5)
        check(r.status()['contact'] and r.status()['load'] == 200000,
              '%s does not absorb a held touch' % what)
        check(r.refused(100, 50, 5.) == REFUSAL,
              'XY still refused after %s' % what)
    # A Z move that travelled 0.1 mm or more began clear: its re-zero
    # also clears an offset latched as contact (the KAN-742 unlock offset)
    r = Rig(z=21.6)
    r.feed(0, 0.5)
    r.feed(717295, 1.)
    r.homing_move('z', 0.1)
    r.feed(717295, 0.1)
    check(not r.status()['contact'] and abs(r.status()['load']) < 1000,
          'G28 Z / a probe that travelled re-zeroes a latched offset')


def test_staircase_is_bounded():
    # Review #86 D: four steady 80000 steps each relocked; 320000 of real
    # press read as zero
    r = Rig(z=5.)
    r.feed(0, 0.5)
    r.feed(80000, 0.6)
    check(not r.status()['contact'], 'one 80000 step is not contact')
    check(r.status()['load'] == 80000 - THRESHOLD // 2,
          'and only max_absorbed_counts (45000) of it is absorbed')
    r.feed(160000, 0.6)
    check(r.status()['contact'] and r.status()['load'] == 160000 - 45000,
          'the second step is contact')
    for load in (240000, 320000):
        r.feed(load, 0.6)
    check(r.status()['contact'], 'and the staircase stays latched')
    check(r.refused(100, 50, 5.) == REFUSAL, 'XY refused')
    # Unloading gives the capacity back: a released step can be absorbed
    # again
    r = Rig(z=5.)
    r.feed(0, 0.5)
    r.feed(80000, 0.6)
    r.feed(0, 0.6)
    check(abs(r.status()['load']) < 1000, 'release relocks back to zero')
    r.feed(80000, 0.6)
    check(r.status()['load'] == 80000 - 45000,
          'and the next step is absorbed up to the cap again')
    # A zero first taken under load gives no extra capacity: once
    # released, later steps are bounded from the real zero
    r = Rig(z=5.)
    r.feed(200000, 0.5)
    r.feed(0, 0.6)
    r.feed(80000, 0.6)
    r.feed(160000, 0.6)
    check(r.status()['contact'],
          'steps after a released loaded zero are bounded from the real'
          ' zero')


def lift_clears(r, trapq, z):
    """A 1 mm G1 Z lift, watched by the stream, then XY at the new Z."""
    trapq.add_move(r.t, 0.2, dz=1.)
    r.feed(0, 0.25, drift=r.last_drift)
    r.move(50, 50, z + 1.)
    return (not r.status()['contact']
            and r.refused(60, 50, z + 1.) is None)


def test_drift_latch_clears_with_a_lift():
    # Review #86 R2-1 / R3-b: drift toward pressing past max_absorbed_counts
    # latches contact with nothing touching.  That is a nuisance, not a
    # lockout: the 1 mm Z lift the refusal asks for clears it.
    for rate in (9000., 4500.):
        r = Rig(z=20.)
        trapq = replay_motion(r)
        r.feed(0, 0.5)
        r.last_drift = -rate
        r.feed(0, 60., drift=-rate)        # raw down: pressing on the M1
        check(r.status()['contact'],
              '60 s of %d counts/s pressing drift latches' % rate)
        check(lift_clears(r, trapq, 20.),
              'and a 1 mm Z lift clears it; XY allowed')
    # Out and back: the unloading leg gives nothing back on the return
    r = Rig(z=20.)
    trapq = replay_motion(r)
    r.feed(0, 0.5)
    r.feed(0, 60., drift=9000.)
    r.feed(0, 60., drift=-9000.)
    r.last_drift = 0.
    check(lift_clears(r, trapq, 20.),
          'out-and-back drift: whatever it reads, a 1 mm lift clears it')


def test_slow_creep_is_bounded():
    # Review #86 R3-b: creep up to ~22000 counts/s (1.1 um/s at the M1's
    # stiffness) was followed by the drift tracker without limit: 132 um
    # of press absorbed in 120 s, then XY allowed
    for rate in (5000., 22000.):
        r = Rig(z=0.5)
        r.feed(0, 0.5)
        r.feed(0, 120., drift=-rate)
        check(r.status()['contact'] and r.refused(60, 50, 0.5) == REFUSAL,
              'creep of %d counts/s for 120 s latches and refuses XY' % rate)
    # Inside the budget it is still followed
    r = Rig(z=0.5)
    r.feed(0, 0.5)
    r.feed(0, 4., drift=-9000.)
    check(not r.status()['contact'] and abs(r.status()['load']) < 22500,
          '36000 counts of creep is followed')


def test_end_of_range_is_no_clearance():
    # Review #86 R3-a: the M1's preloaded cell reads flat near
    # reference_max_load_counts however hard it is pressed, and the ADC
    # clips at its rails.  A flat reading there is no evidence.
    def latched(ref_max):
        r = Rig(z=0.5)
        r.printer.probe.reference_max = ref_max
        trapq = replay_motion(r)
        r.feed(0, 0.5)
        r.feed(350000, 0.3)
        return r, trapq
    ref_max = ZERO + PRESS * 1920000       # tare to reference_max: 1.92 M
    for plateau in (1800000, 1830000):
        for dz, what in ((-1.5, 'descent'), (1.5, 'lift')):
            r, trapq = latched(ref_max)
            trapq.add_move(r.t, 1., dz=dz)
            r.feed(plateau, 2.)
            check(r.status()['contact'] and r.refused(60, 50, 0.5) == REFUSAL,
                  'flat at %d (reference_max plateau) through a 1.5 mm %s:'
                  ' stays latched' % (plateau, what))
    # Clipped at the ADC rail, with no reference_max configured
    rail = 0x800000 + ZERO * PRESS         # load that puts raw at -0x800000
    r, trapq = latched(None)
    trapq.add_move(r.t, 1., dz=1.5)
    r.feed(rail, 2.)
    check(r.status()['contact'], 'clipped at the ADC rail through a lift:'
          ' stays latched')
    # Well inside the range the same lift still clears
    r, trapq = latched(ref_max)
    trapq.add_move(r.t, 1., dz=1.5)
    r.feed(150000, 2.)
    check(not r.status()['contact'], 'inside the range a lift still clears')


def test_own_refusal_keeps_z_trust():
    # Review #86 R3-c: the guard's own refusal is a command error, and it
    # used to drop the Z trust, so REZERO needed a fresh G28.  R4-7: inside
    # a macro gcode:command_error fires once per level.
    for levels in (1, 2, 3):
        r = Rig(z=20.)
        r.home_z()
        r.feed(0, 0.5)
        r.feed(200000, 1.)
        try:
            r.move(60, 50, 20.)
        except CommandError as err:
            r.report(err, levels)
        check(rezero_error(r) is None and not r.status()['contact'],
              'REZERO still works after a refusal %d macro level(s) deep'
              % levels)
    # REZERO's own refusals keep it too
    r = Rig(z=5.)
    r.home_z()
    r.feed(0, 0.5)
    r.feed(200000, 1.)
    check(rezero_error(r) is not None, 'REZERO refused at Z 5')
    r.printer.toolhead.position[2] = 20.
    check(rezero_error(r) is None, 'and works once the toolhead is high')
    # Any homing activity after the refusal takes the exemption away: a
    # failed probe after it still drops the trust
    r = Rig(z=20.)
    r.home_z()
    r.feed(0, 0.5)
    r.feed(200000, 1.)
    r.refused(60, 50, 20.)
    r.homing_move('z', 0.005)
    r.printer.send_event("gcode:command_error")
    check('G28' in (rezero_error(r) or ''),
          'a failed homing move after a refusal still drops the trust')
    # Other command errors still drop it
    r = Rig(z=20.)
    r.home_z()
    r.report(CommandError('Move out of range'))
    r.feed(0, 0.5)
    r.feed(200000, 1.)
    check('G28' in (rezero_error(r) or ''),
          'any other command error drops the trust')



def test_steady_offset_over_z_rezeroes():
    # The KAN-742 unlock offset, latched as contact at Z 21.6, clears once
    # the toolhead really moves 1 mm and the load ignores it: no REZERO, no
    # G28 needed
    r = Rig(z=21.6)
    trapq = replay_motion(r)
    r.feed(0, 0.5)
    for load, duration in ((240000, 0.1), (480000, 0.1), (717295, 1.)):
        r.feed(load, duration)
    check(r.status()['contact'], 'unlock offset latched')
    trapq.add_move(r.t, 0.5, dz=1.)
    r.feed(717295, 0.49)
    check(r.status()['contact'], 'not before 1 mm of travel')
    r.feed(717295, 0.02)
    check(not r.status()['contact'] and abs(r.status()['load']) < 1000,
          'steady over 1 mm of Z: re-zeroed')
    r.feed(717295 + 200000, 0.05)
    check(r.status()['contact'] and r.status()['load'] == 200000,
          'a new press is measured from the new zero')
    # A capped sub-threshold offset (not contact) also re-zeroes fully
    r = Rig(z=20.)
    trapq = replay_motion(r)
    r.feed(0, 0.5)
    r.feed(80000, 0.6)
    check(r.status()['load'] == 80000 - THRESHOLD // 2, 'capped relock')
    trapq.add_move(r.t, 0.5, dz=-1.)
    r.feed(80000, 0.6)
    check(abs(r.status()['load']) < 1000, 'then re-zeroed by 1 mm of Z')
    # A position reset in the middle of the travel breaks the evidence
    r = Rig(z=21.6)
    trapq = replay_motion(r)
    r.feed(0, 0.5)
    r.feed(300000, 0.5)
    trapq.add_move(r.t, 0.3, dz=0.6)
    r.feed(300000, 0.4)
    trapq.add_reset(r.t, 30.)
    r.set_position()
    r.feed(300000, 0.1)
    trapq.add_move(r.t, 0.3, dz=0.6)
    r.feed(300000, 0.4)
    check(r.status()['contact'],
          'two 0.6 mm moves split by a reset are not 1 mm of travel')


def test_clearance_needs_samples_through_the_travel():
    # A stream gap across the Z move: the load before and after match, but
    # nothing watched the travel, so it is no evidence
    r = Rig(z=5.)
    trapq = replay_motion(r)
    r.feed(0, 0.5)
    r.feed(200000, 0.5)
    trapq.add_move(r.t, 0.3, dz=15.)
    r.n += int(0.4 * SPS)            # 0.4 s with no samples
    r.feed(200000, 0.5)
    check(r.status()['contact'] and r.status()['load'] == 200000,
          'travel during a sample gap is not clearance')


def test_m1_homing_override_trusts_z():
    # Review #86 R2-2: the M1 homes Z in homing_override with
    # SET_KINEMATIC_POSITION Z=max, PROBE, SET_KINEMATIC_POSITION from the
    # probe.  That counts as a real Z home when the probe triggered after
    # clear downward travel.
    def held(probe_travel=5., fail=False):
        r = Rig(z=20.)
        ho = r.printer.homing_override
        ho.in_script = True
        r.set_position()                    # Z = axis maximum
        r.homing_move('z', probe_travel)
        if fail:
            r.printer.send_event("gcode:command_error")
        r.set_position()                    # Z from the probe
        ho.in_script = False
        r.feed(0, 0.5)
        r.feed(200000, 1.)
        return r
    r = held()
    check(rezero_error(r) is None and not r.status()['contact'],
          'REZERO works after the M1 homing_override Z home')
    r = held(probe_travel=0.005)
    check('G28' in (rezero_error(r) or ''),
          'a probe that started on the plate is not a Z home')
    r = held(probe_travel=-10.)
    check('G28' in (rezero_error(r) or ''),
          'upward travel is not a Z home')
    r = held(fail=True)
    check('G28' in (rezero_error(r) or ''), 'a failed probe is not a Z home')
    # The trust is for the one reset that follows the probe
    r = held()
    r.printer.homing_override.in_script = True
    r.set_position()                        # the next G28's Z = max
    r.printer.homing_override.in_script = False
    check('G28' in (rezero_error(r) or ''),
          'the next homing script reset is not covered')
    # A bare PROBE outside homing_override does not re-reference Z
    r = Rig(z=20.)
    r.set_position()
    r.homing_move('z', 5.)
    r.feed(0, 0.5)
    r.feed(200000, 1.)
    check('G28' in (rezero_error(r) or ''), 'a bare PROBE is not a Z home')


def test_flags_only_reset_keeps_trust():
    # SET_KINEMATIC_POSITION SET_HOMED=Z with Z unchanged (the M1's X/Y
    # homing macros) is no new frame
    r = Rig(z=20.)
    replay_motion(r, pos=(50., 50., 20.))
    r.home_z()
    r.set_position()
    r.feed(0, 0.5)
    r.feed(200000, 1.)
    check(rezero_error(r) is None, 'a reset to the same Z keeps the trust')


def test_z_motor_off_clears_trust():
    # Review #86 R2-3: with the Z motor off the axis can be turned by hand
    r = Rig(z=20.)
    r.home_z()
    r.feed(0, 0.5)
    r.z_motor(False)
    r.feed(200000, 1.)
    check('motor off' in (rezero_error(r) or ''),
          'REZERO refused after the Z motor was disabled')
    check(r.status()['contact'] and r.refused(100, 50, 20.) == REFUSAL,
          'and the touch stays latched, XY refused')
    r.z_motor(True)
    check('G28' in (rezero_error(r) or ''),
          're-enabling the motor does not restore the trust')


def test_upward_or_failed_homing_does_not_rezero():
    # Review #86 R2-4: an upward Z homing move (AXIS_PROTECTED_MOVE Z+10)
    # started while pressing re-zeroed on the pressed load from before it.
    # Now the pre-move reading is never used for a lift; after it the touch
    # is gone and the load reads its true zero.
    r = Rig(z=5.)
    r.feed(0, 0.5)
    r.feed(200000, 0.5)
    r.homing_move('z', -10.)
    r.feed(0, 0.5)
    check(not r.status()['contact'] and abs(r.status()['load']) < 1000,
          'after an upward Z homing move the released load reads zero')
    r.feed(200000, 0.05)
    check(r.status()['contact'] and r.status()['load'] == 200000,
          'and a new press reads in full (the pressed load was not the'
          ' zero)')
    # A downward move that ends in an error does not re-zero either
    r = Rig(z=5.)
    r.feed(0, 0.5)
    r.feed(200000, 0.5)
    r.homing_move('z', 5.)
    r.printer.send_event("gcode:command_error")
    r.feed(200000, 0.5)
    check(r.status()['contact'] and r.status()['load'] == 200000,
          'a homing move that errored does not re-zero')



def test_stale_stream_fails_closed():
    # Review #86 R4-1a: with no samples for 1 s a latched contact was
    # forgotten and XY allowed
    for z in (0.5, 20.):
        r = Rig(z=z)
        r.feed(0, 0.5)
        r.feed(200000, 0.5)
        r.printer.reactor.now += 1.01
        check(r.refused(60, 50, z) == contact_guard.NO_DATA,
              'latched, then no samples for 1 s: XY refused at Z %.1f' % z)
        check(r.refused(50, 50, z + 1.) is None,
              'and Z alone still moves (the way out)')
    # Clear, near the plate, stream stopped: refused (the wait finds none)
    r = Rig(z=0.5)
    r.feed(0, 0.5)
    r.printer.reactor.now += 1.01
    check(r.refused(60, 50, 0.5) == contact_guard.NO_DATA,
          'clear near the plate with no samples: refused')
    # Clear, high above the plate, stream stopped: allowed.  The last state
    # was clear and nothing queued can reach the plate; this is the
    # toolhead unplugged (its MCU is non-critical) parking or docking.
    r = Rig(z=20.)
    r.feed(0, 0.5)
    r.printer.reactor.now += 1.01
    check(r.refused(60, 50, 20.) is None,
          'clear at Z 20 with no samples: allowed (documented)')
    check(r.refused(70, 50, 20.) is None and
          r.guard.cmd_SET_CONTACT_GUARD(Cmd(ENABLE=0)) is None,
          'ENABLE=0 is always available')


def test_post_trigger_waits_at_any_z():
    # Review #86 R4-2: PROBE leaves the nozzle on what it hit.  At Z >= 3
    # nothing waited, so XY in the 0.52 s after a trigger went through.
    for z in (5., 20.):
        r = Rig(z=z)
        r.feed(0, 0.5)
        r.printer.probe.trigger_time = r.t
        r.feed(300000, 0.2)          # in the ring
        r.printer.reactor.on_pause.append(lambda: r.feed(300000, 0.005))
        check(r.refused(60, 50, z) == REFUSAL,
              'XY 0.2 s after a trigger at Z %.0f waits, then is refused'
              % z)
        r = Rig(z=z)
        r.feed(0, 0.5)
        r.printer.probe.trigger_time = r.t
        r.feed(300000, 0.51)         # 10 ms past the hold-off
        r.printer.reactor.on_pause.append(lambda: r.feed(300000, 0.005))
        check(r.refused(60, 50, z) == REFUSAL,
              '10 ms after the hold-off at Z %.0f: waits for on_time, then'
              ' refused' % z)
    # Any gap in the stream gets the same wait
    r = Rig(z=20.)
    r.feed(0, 0.5)
    r.printer.send_event("homing:homing_move_begin", FakeHomingMove('x'))
    r.feed(300000, 0.1)              # ignored while homing
    r.printer.send_event("homing:homing_move_end", FakeHomingMove('x'))
    r.feed(300000, 0.005)            # 5 ms of data since the gap
    r.printer.reactor.on_pause.append(lambda: r.feed(300000, 0.005))
    check(r.refused(60, 50, 20.) == REFUSAL,
          'XY 5 ms after a gap waits for on_time of data, then refused')


def test_rezero_refuses_contact_level_and_end_of_range():
    # Review #86 R4-3: REZERO's Z >= 10 says nothing about a part under the
    # nozzle, and it took a reading flat at the plateau
    def held(load, ref_max=None):
        r = Rig(z=30.)
        r.printer.probe.reference_max = ref_max
        r.home_z()
        r.feed(0, 0.5)
        r.feed(load, 1.)
        return r
    r = held(400000)                 # jogged down onto a 30 mm part
    err = rezero_error(r, force=0)
    check(err is not None and 'lift Z' in err,
          'REZERO refused at contact level without FORCE: %s' % err)
    check(r.refused(60, 50, 30.) == REFUSAL, 'and XY stays refused')
    r = held(60000)                  # under the threshold, not latched
    check(rezero_error(r, force=0) is None, 'a sub-threshold offset is'
          ' re-zeroed without FORCE')
    r = held(400000)
    check(rezero_error(r, force=1) is None and not r.status()['contact'],
          'FORCE=1 is the operator saying nothing touches (logged)')
    ref_max = ZERO + PRESS * 1920000
    r = held(1830000, ref_max)
    err = rezero_error(r, force=1)
    check(err is not None and 'end of its range' in err,
          'flat at the reference_max plateau: refused even with FORCE')
    r = held(0x800000 + ZERO * PRESS)
    check('end of its range' in (rezero_error(r, force=1) or ''),
          'clipped at the ADC rail: refused even with FORCE')


def test_m1_home_rezeroes_with_samples_flowing():
    # Review #86 R4-6: the M1's homing_override tares (a reactor pause) with
    # samples arriving; they were ignored and emptied the anchor, so its
    # G28 never re-zeroed
    r = Rig(z=20.)
    r.feed(0, 0.5)
    r.feed(300000, 1.)
    check(r.status()['contact'], 'false latch (a 300000 offset)')
    ho = r.printer.homing_override
    ho.in_script = True
    r.set_position()                 # SET_KINEMATIC_POSITION Z=max
    r.feed(300000, 0.3)              # LOAD_CELL_TARE, PROBE's tare pause
    check(r.status()['active'] is False, 'inactive inside the script')
    r.homing_move('z', 5.)           # the probe: 5 mm down, then a trigger
    r.set_position()                 # Z from the probe
    ho.in_script = False
    r.feed(300000, 0.1)
    check(not r.status()['contact'] and abs(r.status()['load']) < 1000,
          'the M1 G28 re-zeroes on the reading before its probe')
    check(rezero_error(r) is None, 'and Z is trusted for REZERO')


def test_g28_lift_rezero_survives_home_y():
    # Review #86 R5-1: in a plain G28 the macro's _HOME_Y homing move began
    # before any post-lift samples arrived, which dropped the lift re-zero;
    # _HOME_X's G1 Y10 was then refused after its SET_HOMED placeholders,
    # leaving the machine falsely homed (KAN-573).  The re-zero is now
    # settled inside the lift's own homing_move_end, while samples flow.
    r = Rig(z=20.)
    trapq = replay_motion(r)
    r.feed(0, 0.5)
    for load, d in ((240000, .1), (480000, .1), (717295, 1.)):
        r.feed(load, d)                      # unlock tension, latched
    check(r.status()['contact'], 'tension latched')
    r.printer.toolhead.homed_axes = ''       # idle_timeout M84
    r.z_motor(False)
    r.set_position()                         # SKP Z=100 SET_HOMED=Z
    hm = FakeHomingMove('z', -10.)           # AXIS_PROTECTED_MOVE Z+10
    r.printer.send_event("homing:homing_move_begin", hm)
    trapq.add_move(r.t, 1., dz=10.)
    r.feed(717295, 1.)                       # ignored: inside the move
    r.printer.toolhead.print_time = r.t
    r.printer.reactor.on_pause.append(lambda: r.feed(717295, 0.005))
    r.printer.send_event("homing:homing_move_end", hm)
    r.printer.reactor.on_pause[:] = []
    check(r.guard.lift_zero is None and not r.status()['contact'],
          'the lift re-zero is decided before homing_move_end returns')
    # _HOME_Y: SKPs, AXIS_PROTECTED_MOVE AXIS=Y, G28.1 Y
    r.set_position()
    hy = FakeHomingMove('y', 5.)
    r.printer.send_event("homing:homing_move_begin", hy)
    r.feed(717295, 0.2)
    r.printer.send_event("homing:homing_move_end", hy)
    hs = FakeHomingState([1])
    r.printer.send_event("homing:home_rails_begin", hs, [])
    r.printer.send_event("homing:homing_move_begin", hy)
    r.feed(717295, 0.5)
    r.printer.send_event("homing:homing_move_end", hy)
    r.printer.send_event("homing:home_rails_end", hs, [])
    r.feed(717295, 0.2)
    r.printer.toolhead.homed_axes = 'xyz'    # _HOME_X: SET_HOMED=YXZ
    r.guard.last_position = [50., 50., 110., 0.]
    r.printer.toolhead.print_time = 0.
    check(r.refused(50, 60, 110.) is None,
          'with _HOME_Y in between, _HOME_X G1 Y10 is allowed')
    r.feed(717295 + 200000, 0.05)
    check(r.status()['contact'], 'a real touch afterwards still latches')


def test_nothing_is_judged_inside_a_homing_script():
    # Inside homing_override the readings only anchor the next probe's
    # re-zero: no relock or drift following happens there, so a probe that
    # then fails leaves the zero as it was
    r = Rig(z=20.)
    r.feed(0, 0.5)
    ho = r.printer.homing_override
    ho.in_script = True
    r.feed(60000, 0.6)               # a steady sub-threshold step
    r.homing_move('z', 0.005)        # the probe triggers at once: fails
    r.printer.send_event("gcode:command_error")
    ho.in_script = False
    r.feed(60000, 0.1)
    check(r.status()['load'] == 60000,
          'a step seen only inside the script was not absorbed')


def test_upward_homing_lift_clears_false_latch():
    # Review #86 R4-4: a false latch with Z unhomed (motors off): G1 Z is
    # not allowed, and the G28 macro's own XY moves were refused.  Its
    # AXIS_PROTECTED_MOVE Z+10 (a homing move) now proves the steady load
    # after it is no touch.
    r = Rig(z=20.)
    r.feed(0, 0.5)
    r.feed(300000, 1.)
    r.printer.toolhead.homed_axes = ''
    check(r.status()['contact'], 'false latch, Z unhomed')
    r.homing_move('z', -10.)         # AXIS_PROTECTED_MOVE AXIS=Z DIST=10
    # _HOME_X's first XY move comes before any sample: it waits for them
    r.printer.reactor.on_pause.append(lambda: r.feed(300000, 0.005))
    check(r.refused(50, 10, 20.) is None and not r.status()['contact'],
          'the G28 macro XY move after the lift is allowed')
    # The same lift with the cell at the end of its range is no evidence
    r = Rig(z=20.)
    r.printer.probe.reference_max = ZERO + PRESS * 1920000
    r.feed(0, 0.5)
    r.feed(1830000, 1.)
    r.homing_move('z', -10.)
    r.printer.reactor.on_pause.append(lambda: r.feed(1830000, 0.005))
    check(r.refused(50, 10, 20.) == REFUSAL,
          'a lift with the cell flat at the plateau: still refused')
    # A load still changing after the lift is no evidence either
    r = Rig(z=20.)
    r.feed(0, 0.5)
    r.feed(300000, 1.)
    r.homing_move('z', -10.)
    for i in range(60):
        r.feed(300000 + 30000 * (i % 2), 0.01)
    check(r.status()['contact'], 'a changing load after the lift: latched')


def test_unlock_retract_then_lift():
    # KAN-742 / D-048: the retract leaves a 717295-count tension offset at
    # Z 21.6 with nothing touching (latched); a 0.3 s settle and a 1.2 mm
    # lift and back at 5 mm/s clear it, no REZERO, no FORCE.  This is the
    # lift variant of _UNLOCK_CLEAR_GUARD_OFFSET (47971b0c) and any user
    # jog of the same size.
    r = Rig(z=21.6)
    trapq = replay_motion(r)
    r.feed(0, 0.5)
    for load, duration in ((240000, 0.10), (480000, 0.10), (717295, 0.10)):
        r.feed(load, duration)       # FORCE_MOVE extruder -16
    check(r.status()['contact'], 'the retract offset latches')
    r.feed(717295, 0.3)              # G4 P300
    trapq.add_move(r.t, 0.24, dz=1.2)
    r.feed(717295, 0.24)             # G1 Z1.2 F300
    trapq.add_move(r.t, 0.24, dz=-1.2)
    r.feed(717295, 0.24)             # G1 Z-1.2 F300
    check(not r.status()['contact'] and abs(r.status()['load']) < 1000,
          'the 1.2 mm lift and return clears the offset')
    check(r.refused(60, 50, 21.6) is None, 'and CANCEL_PRINT can park')
    r.feed(717295 + 200000, 0.05)
    check(r.status()['contact'] and r.refused(70, 50, 21.6) == REFUSAL,
          'a real touch afterwards is still caught')
    # A real touch under the nozzle is not cleared by the same lift: it
    # falls away as Z rises, so nothing is re-zeroed on it
    r = Rig(z=0.5)
    trapq = replay_motion(r)
    r.feed(0, 0.5)
    r.feed(300000, 0.4)
    trapq.add_move(r.t, 0.24, dz=1.2)
    r.feed(0, 0.24)
    trapq.add_move(r.t, 0.24, dz=-1.2)
    r.feed(300000, 0.24)
    check(r.status()['contact'] and r.status()['load'] == 300000,
          'a touch the lift leaves and the return finds is still contact')


def test_print_end_lift_clears_unlock_offset():
    # Review #86 R5-2 (r5_noop): with the unlock press making no motion,
    # KAN-742's own path still recovers: PRINT_END / CANCEL_PRINT raise Z
    # 6 mm, M400, then park XY.  That lift is the proof.
    for z in (21.6, 0.3):
        r = Rig(z=z)
        trapq = replay_motion(r)
        r.feed(0, 0.5)
        for load, d in ((240000, .1), (480000, .1), (717295, 1.)):
            r.feed(load, d)
        check(r.status()['contact'], 'tension latched at Z %.1f' % z)
        trapq.add_move(r.t, 0.6, dz=6.)      # G0 Z+6 F600
        r.feed(717295, 0.65)                 # M400
        r.move(50, 50, z + 6.)
        check(r.refused(195, 10, z + 6.) is None,
              'PRINT_END park after the 6 mm lift is allowed (Z %.1f)' % z)


def test_rezero_try_reports_instead_of_raising():
    # The macro fallback: REZERO=1 TRY=1 on a contact-level load (no FORCE)
    r = Rig(z=20.)
    r.home_z()
    r.feed(0, 0.5)
    r.feed(717295, 1.)
    cmd = Cmd(REZERO=1, TRY=1)
    r.guard.cmd_SET_CONTACT_GUARD(cmd)
    check(any('not re-zeroed' in m for m in cmd.info)
          and r.status()['contact'],
          'TRY=1: refused, reported, contact left latched')
    r.feed(40000, 0.6)               # a sub-threshold offset instead
    r = Rig(z=20.)
    r.home_z()
    r.feed(0, 0.5)
    r.feed(40000, 0.1)
    cmd = Cmd(REZERO=1, TRY=1)
    r.guard.cmd_SET_CONTACT_GUARD(cmd)
    check(any('re-zeroed, load was' in m for m in cmd.info),
          'TRY=1 re-zeroes what REZERO would')


def test_motion_contact_never_relocks_while_held():
    for axis in ('x', 'y', 'z'):
        r = Rig(z=0.5)
        trapq = replay_motion(r)
        trapq.axis = axis
        r.feed(0, 0.5)
        trapq.z_moves.append((r.t, r.t + 0.05))
        r.feed(200000, 1.5)
        check(r.status()['contact'] and r.status()['load'] == 200000,
              'contact beginning during %s motion remains latched' % axis)
        r.printer.toolhead.print_time = r.t - 0.1
        check(r.refused(60, 50, 0.5) == REFUSAL,
              'held %s contact still refuses XY after relock interval' % axis)


def test_unknown_motion_and_changing_offsets_do_not_relock():
    r = Rig()
    r.feed(0, 0.5)
    r.feed(717295, 1.5)
    check(r.status()['contact'] and r.status()['load'] == 717295,
          'unknown executed motion never absorbs above-threshold load')
    r = Rig(z=21.6)
    replay_motion(r)
    r.feed(0, 0.5)
    for _ in range(6):
        r.feed(200000, 0.1)
        r.feed(250000, 0.1)
    check(r.status()['contact'] and r.status()['load'] == 250000,
          'changing load outside the band does not count as steady offset')


def test_stationary_sub_threshold_offset_still_relocks():
    # Genuine offsets that are not contact still re-zero with the toolhead
    # still, as on master
    r = Rig(z=21.6)
    replay_motion(r)
    r.feed(0, 0.5)
    r.feed(40000, 0.6)
    check(abs(r.status()['load']) < 1000 and not r.status()['contact'],
          'a steady 40000-count offset under the threshold re-zeroes')
    r.feed(40000 + 200000, 0.05)
    check(r.status()['contact'], 'contact measured from the new zero')


def main():
    tests = [v for k, v in sorted(globals().items())
             if k.startswith('test_') and callable(v)]
    for t in tests:
        print("== " + t.__name__)
        t()
    print("All %d contact_guard tests passed" % len(tests))


if __name__ == '__main__':
    main()
