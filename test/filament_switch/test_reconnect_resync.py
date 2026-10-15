"""KAN-558: the toolhead filament switch across an umbilical unplug.

On boxwood (2026-10-14 13:36 BST) TOOLHEAD_CONNECT started LOAD_FILAMENT on
reattach with no filament in the toolhead. No insert edge fired. The host
still held filament_detected = True from before the unplug: MCU_buttons
restarted every pin as "released", and the MCU only reports a pin that
differs from that, so a switch now reading empty sent nothing and the old
value stood.

Checks, with the real MCU_buttons and RunoutHelper and no reactor, MCU
dictionary or hardware:

* a resync button's MCU starts from the state the host last saw, so a
  change made while unplugged arrives as an edge on reconnect
* buttons without resync keep the upstream "released" start
* RunoutHelper takes no insert action while its MCU is offline or for
  RECONNECT_SETTLE_TIME after it returns, keeps tracking the state, and
  still runs insert_gcode for an insert after that (the auto-load stays
  armed)

    python3 test/filament_switch/test_reconnect_resync.py
"""
import os
import sys

KLIPPY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

from extras import buttons, filament_switch_sensor  # noqa: E402

DISCONNECT = "danger:non_critical_mcu_toolhead:disconnected"
RECONNECT = "danger:non_critical_mcu_toolhead:reconnected"
NEVER = 9999999999999999.


class FakeReactor:
    NEVER = NEVER
    def __init__(self):
        self.now = 0.
        self.callbacks = []
    def monotonic(self):
        return self.now
    def register_async_callback(self, cb):
        cb(self.now)
    def register_callback(self, cb, waketime=0.):
        self.callbacks.append(cb)
    def run_callbacks(self):
        callbacks, self.callbacks = self.callbacks, []
        for cb in callbacks:
            cb(self.now)


class FakePrinter:
    def __init__(self):
        self.reactor = FakeReactor()
        self.handlers = {}
        self.objects = {}
    def get_reactor(self):
        return self.reactor
    def register_event_handler(self, name, cb):
        self.handlers.setdefault(name, []).append(cb)
    def send_event(self, name):
        for cb in self.handlers.get(name, []):
            cb()
    def lookup_object(self, name):
        return self.objects[name]
    def load_object(self, config, name):
        return self.objects[name]


class FakeCommand:
    def __init__(self):
        self.sent = []
    def send(self, data):
        self.sent.append(list(data))


class FakeMcu:
    is_non_critical = True
    def __init__(self):
        self.config_cmds = []
        self.ack = FakeCommand()
    def register_config_callback(self, cb):
        pass
    def get_non_critical_disconnect_event_name(self):
        return DISCONNECT
    def get_non_critical_reconnect_event_name(self):
        return RECONNECT
    def create_oid(self):
        return 7
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        self.config_cmds.append(cmd)
    def alloc_command_queue(self):
        return None
    def lookup_command(self, msgformat, cq=None):
        return self.ack
    def get_query_slot(self, oid):
        return 0
    def seconds_to_clock(self, t):
        return int(t * 12000000)
    def register_response(self, cb, name, oid):
        pass
    def query_start_state(self):
        cmds = [c for c in self.config_cmds if c.startswith("buttons_query")]
        return int(cmds[-1].rsplit("invert=", 1)[1])


# Pin positions on the toolhead MCU_buttons: the unlock button first, as
# core.cfg declares it, then the filament switch. Both are inverted ("!").
UNLOCK, FILAMENT = 0, 1
INVERT = 0b11


def raw_state(unlock_pressed, filament_detected):
    logical = (unlock_pressed << UNLOCK) | (filament_detected << FILAMENT)
    return INVERT ^ logical


class Recorder:
    def __init__(self):
        self.states = []
    def __call__(self, eventtime, state):
        self.states.append(state)


def make_buttons():
    printer = FakePrinter()
    mcu = FakeMcu()
    mb = buttons.MCU_buttons(printer, mcu)
    unlock, filament = Recorder(), Recorder()
    mb.setup_buttons([{'pin': 'gpio5', 'pullup': 0, 'invert': 1}], unlock)
    mb.setup_buttons([{'pin': 'gpio3', 'pullup': 0, 'invert': 1}], filament,
                     resync_on_reconnect=True)
    return printer, mcu, mb, unlock, filament


class McuSide:
    """The MCU's buttons_state reports, with its own ack counter."""
    def __init__(self, mb):
        self.mb = mb
        self.ack_count = 0
    def report(self, state):
        self.mb.handle_buttons_state({'ack_count': self.ack_count,
                                      'state': bytes([state]),
                                      '#receive_time': 0.})
        self.ack_count += 1


def test_first_config_starts_released():
    printer, mcu, mb, unlock, filament = make_buttons()
    mb.build_config()
    assert mcu.query_start_state() == INVERT, mcu.query_start_state()


def test_filament_removed_while_unplugged_arrives_on_reconnect():
    printer, mcu, mb, unlock, filament = make_buttons()
    mb.build_config()
    mcu_side = McuSide(mb)
    mcu_side.report(raw_state(0, 1))          # filament loaded
    assert filament.states == [1], filament.states
    printer.send_event(DISCONNECT)            # umbilical out
    mb.build_config()                         # umbilical in: config resent
    mcu_side = McuSide(mb)                    # the MCU restarted its counter
    # The MCU starts from what the host last saw: filament present.
    assert mcu.query_start_state() == raw_state(0, 1), mcu.query_start_state()
    # The switch now reads empty, which differs from that start, so the
    # MCU reports it, and the host sees the change.
    mcu_side.report(raw_state(0, 0))
    printer.send_event(RECONNECT)
    assert filament.states == [1, 0], filament.states
    assert unlock.states == [], unlock.states


def test_unchanged_switch_sends_nothing_on_reconnect():
    printer, mcu, mb, unlock, filament = make_buttons()
    mb.build_config()
    McuSide(mb).report(raw_state(0, 1))
    printer.send_event(DISCONNECT)
    mb.build_config()
    printer.send_event(RECONNECT)
    # Filament still in: the MCU matches its start state and reports
    # nothing, and the host's belief is still right.
    assert filament.states == [1], filament.states
    assert mb.last_button == (1 << FILAMENT), mb.last_button


def test_unlock_button_keeps_upstream_start():
    # Only the filament switch opted in; the unlock button restarts
    # "released" exactly as before this change.
    printer, mcu, mb, unlock, filament = make_buttons()
    mb.build_config()
    McuSide(mb).report(raw_state(1, 0))       # unlock held
    assert unlock.states == [1], unlock.states
    printer.send_event(DISCONNECT)
    mb.build_config()
    assert mcu.query_start_state() == raw_state(0, 0), \
        mcu.query_start_state()


class FakeConfig:
    def __init__(self, printer, values):
        self.printer = printer
        self.values = values
    def get_name(self):
        return "filament_switch_sensor filament_sensor_toolhead"
    def get_printer(self):
        return self.printer
    def get(self, name, default=None):
        return self.values.get(name, default)
    def getboolean(self, name, default=None):
        return self.values.get(name, default)
    def getfloat(self, name, default=None, **kw):
        return self.values.get(name, default)


class FakeTemplate:
    def __init__(self, text):
        self.text = text
    def render(self):
        return self.text


class FakeGcodeMacro:
    def load_template(self, config, name, default=None):
        return FakeTemplate(config.get(name, default))


class FakeGcode:
    def __init__(self):
        self.scripts = []
    def register_mux_command(self, *args, **kw):
        pass
    def run_script(self, script):
        self.scripts.append(script)


class FakeIdleTimeout:
    def get_status(self, eventtime):
        return {"state": "Idle"}


def make_runout_helper():
    printer = FakePrinter()
    gcode = FakeGcode()
    printer.objects.update({'gcode': gcode,
                            'gcode_macro': FakeGcodeMacro(),
                            'idle_timeout': FakeIdleTimeout()})
    config = FakeConfig(printer, {'pause_on_runout': False,
                                  'insert_gcode': 'INSERT_FILAMENT_SENSOR'})
    helper = filament_switch_sensor.RunoutHelper(config)
    helper.note_sensor_mcu(FakeMcu())
    return printer, gcode, helper


def edge(printer, helper, t, present):
    printer.reactor.now = t
    helper.note_filament_present(t, present)
    printer.reactor.run_callbacks()


def inserts(gcode):
    return [s for s in gcode.scripts if 'INSERT_FILAMENT_SENSOR' in s]


def test_no_insert_action_until_toolhead_has_settled():
    printer, gcode, helper = make_runout_helper()
    printer.reactor.now = 0.
    helper._handle_ready()                    # startup grace ends at 2.0
    # The toolhead arrives late, through the reconnect path. Its first
    # reading lands while it is still connecting.
    edge(printer, helper, 5.0, True)
    assert inserts(gcode) == [], gcode.scripts
    assert helper.get_status(5.0)['filament_detected'] is True
    printer.reactor.now = 5.1
    printer.send_event(RECONNECT)             # settled at 7.1
    edge(printer, helper, 6.0, False)
    edge(printer, helper, 7.0, True)
    assert inserts(gcode) == [], gcode.scripts
    # A real insert after the toolhead has settled still loads: the
    # automatic load stays armed.
    edge(printer, helper, 8.0, False)
    edge(printer, helper, 9.0, True)
    assert inserts(gcode) == ['INSERT_FILAMENT_SENSOR\nM400'], gcode.scripts


def test_no_insert_action_while_unplugged_or_just_back():
    printer, gcode, helper = make_runout_helper()
    printer.reactor.now = 0.
    helper._handle_ready()
    printer.reactor.now = 3.
    printer.send_event(RECONNECT)             # settled at 5.0
    edge(printer, helper, 6.0, True)
    assert len(inserts(gcode)) == 1, gcode.scripts
    edge(printer, helper, 20.0, False)        # filament pulled out
    printer.reactor.now = 30.
    printer.send_event(DISCONNECT)            # umbilical out
    edge(printer, helper, 31.0, True)         # reading on the way out
    printer.reactor.now = 40.
    printer.send_event(RECONNECT)             # back in, settled at 42.0
    edge(printer, helper, 40.5, False)
    edge(printer, helper, 41.0, True)
    assert len(inserts(gcode)) == 1, gcode.scripts
    assert helper.get_status(41.0)['filament_detected'] is True
    edge(printer, helper, 43.0, False)
    edge(printer, helper, 50.0, True)
    assert len(inserts(gcode)) == 2, gcode.scripts


def test_sensor_on_a_critical_mcu_is_unchanged():
    printer, gcode, helper = make_runout_helper()
    other = filament_switch_sensor.RunoutHelper(FakeConfig(
        printer, {'pause_on_runout': False,
                  'insert_gcode': 'INSERT_FILAMENT_SENSOR'}))
    class MainMcu:
        is_non_critical = False
    other.note_sensor_mcu(MainMcu())
    assert other.settle_until == 0.
    printer.reactor.now = 0.
    other._handle_ready()
    edge(printer, other, 3.0, True)
    assert len(inserts(gcode)) == 1, gcode.scripts


def main():
    tests = [test_first_config_starts_released,
             test_filament_removed_while_unplugged_arrives_on_reconnect,
             test_unchanged_switch_sends_nothing_on_reconnect,
             test_unlock_button_keeps_upstream_start,
             test_no_insert_action_until_toolhead_has_settled,
             test_no_insert_action_while_unplugged_or_just_back,
             test_sensor_on_a_critical_mcu_is_unchanged]
    for test in tests:
        test()
        print("ok  %s" % test.__name__)
    return 0


if __name__ == '__main__':
    sys.exit(main())
