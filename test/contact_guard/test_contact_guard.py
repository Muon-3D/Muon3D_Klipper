"""contact_guard: the decisions test/klippy can only see as an exit code.

test/klippy/contact_guard*.test drive the guard inside klippy.  test_klippy
reports only whether klippy exited with an error, so this drives the real
ContactGuard with fakes and asserts what that cannot: the refusal's exact
text, the timing boundaries, and the states that make the guard inactive.

    python3 test/contact_guard/test_contact_guard.py
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
    def monotonic(self):
        return self.now


class FakeGCode:
    error = CommandError
    def __init__(self):
        self.commands = {}
    def register_command(self, name, func, desc=None):
        self.commands[name] = func


class FakeLoadCell:
    def __init__(self):
        self.clients = []
    def add_client(self, cb):
        self.clients.append(cb)


class FakeProbe:
    def __init__(self):
        self.load_cell = FakeLoadCell()
        self.trigger_time = 0.
    def get_load_cell(self):
        return self.load_cell
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
    def get_position(self):
        return list(self.position)
    def move(self, newpos, speed):
        self.moves.append(list(newpos))
        self.position = list(newpos)


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
        self.objects = {
            'gcode': self.gcode, 'load_cell_probe': self.probe,
            'print_stats': self.print_stats, 'toolhead': self.toolhead,
            'gcode_move': FakeGCodeMove(self.toolhead),
            'homing_override': self.homing_override,
            'nozzle_wipe_smart': self.nozzle_wipe,
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


class Rig:
    """A guard wired to fakes, with a sample clock."""
    def __init__(self, values=None):
        self.printer = FakePrinter()
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
            return str(err)
        return None
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
    r.printer.send_event("homing:homing_move_begin", None)
    check(not r.status()['active'], "inactive during a homing move")
    r.feed(200000, 0.3)          # a tap: ignored, not contact or drift
    r.printer.send_event("homing:homing_move_end", None)
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
    check(r.refused(60, 50, 5) is None, "XY allowed in the hold-off")
    r.feed(200000, 0.02)         # 500 .. 519.5 ms
    check(r.status()['active'] and not r.status()['contact'],
          "active at 0.5 s; contact needs 20 ms from there")
    r.feed(200000, 0.0005)
    check(r.status()['contact'], "load past the hold-off is contact")


def test_enable():
    r = Rig()
    r.in_contact()
    class Cmd:
        def __init__(self, enable):
            self.enable = enable
            self.info = []
        def get_int(self, name, default, minval=None, maxval=None):
            return self.enable
        def respond_info(self, msg):
            self.info.append(msg)
    r.guard.cmd_SET_CONTACT_GUARD(Cmd(0))
    check(r.status() == {'contact': False, 'load': 200000, 'active': False,
                         'enabled': False}, "ENABLE=0 makes it inactive")
    check(r.refused(60, 50, 5) is None, "XY allowed with ENABLE=0")
    r.guard.cmd_SET_CONTACT_GUARD(Cmd(1))
    check(r.refused(70, 50, 5) == REFUSAL, "ENABLE=1 refuses again")


def test_drift():
    r = Rig()
    r.feed(0, 1.)
    worst = 0
    for sign in (1., -1.):
        for _ in range(60):
            r.feed(0, 1., drift=sign * 9000.)
            worst = max(worst, abs(r.status()['load']))
            check_quiet = not r.status()['contact']
            if not check_quiet:
                raise AssertionError("drift read as contact")
    check(worst < THRESHOLD / 4,
          "9000 counts/s drift for 60 s each way is followed inside the"
          " band (worst %d)" % worst)


def test_relock():
    r = Rig()
    r.feed(0, 0.5)
    r.feed(50000, 0.4995)        # outside the band, under the threshold
    check(r.status()['load'] == 50000, "a lingering offset is not absorbed"
          " before 0.5 s")
    r.feed(50000, 0.001)         # the sample at 0.5 s re-zeroes
    r.feed(50000, 0.0005)
    check(abs(r.status()['load']) < 1000, "and is re-zeroed at 0.5 s")
    r.feed(50000 + 200000, 0.05)
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
    r.printer.send_event("homing:homing_move_begin", None)
    r.printer.send_event("homing:homing_move_end", None)
    r.feed(0, 0.1)
    check(not r.status()['contact'] and abs(r.status()['load']) < 1000,
          "a homing move re-zeroes on what it read before moving")


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


def main():
    tests = [v for k, v in sorted(globals().items())
             if k.startswith('test_') and callable(v)]
    for t in tests:
        print("== " + t.__name__)
        t()
    print("All %d contact_guard tests passed" % len(tests))


if __name__ == '__main__':
    main()
