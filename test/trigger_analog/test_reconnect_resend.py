# Offline test for klippy/extras/trigger_analog.py: after a non-critical MCU
# (the M1 toolhead) reconnects, its trigger_analog and sos_filter are new
# objects on the MCU, so the host must not skip resending a raw range,
# trigger or filter section it sent before the reconnect.  The reconnect
# runs every config callback again, which is where the caches are reset.
import os, sys, types, unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..',
                                'klippy'))


class Trsync:
    REASON_ENDSTOP_HIT = 1
    REASON_HOST_REQUEST = 2
    REASON_PAST_END_TIME = 3
    REASON_COMMS_TIMEOUT = 4


class Dispatch:
    def __init__(self, mcu):
        pass
    def get_command_queue(self):
        return None
    def get_oid(self):
        return 9


fake_mcu = types.ModuleType('mcu')
fake_mcu.MCU_trsync = Trsync
fake_mcu.TriggerDispatch = Dispatch
sys.modules['mcu'] = fake_mcu
from extras import trigger_analog  # noqa: E402


class Cmd:
    def __init__(self, name, sent):
        self.name, self.sent = name, sent
    def send(self, args, **kw):
        self.sent.append((self.name, list(args)))


class FakeMcu:
    def __init__(self):
        self.sent = []
        self.callbacks = []
        self.oids = 0
    def get_printer(self):
        return None
    def create_oid(self):
        self.oids += 1
        return self.oids
    def add_config_cmd(self, cmd, **kw):
        pass
    def register_config_callback(self, cb):
        self.callbacks.append(cb)
    def lookup_command(self, msgformat, **kw):
        return Cmd(msgformat.split()[0], self.sent)
    def lookup_query_command(self, msgformat, respformat, **kw):
        return Cmd(msgformat.split()[0], self.sent)
    def get_enumerations(self):
        return {}
    def connect(self):
        # what klippy does at connect and again at every reconnect
        for cb in self.callbacks:
            cb()


class FakeSensor:
    def __init__(self, mcu):
        self.mcu = mcu
    def get_mcu(self):
        return self.mcu
    def setup_trigger_analog(self, oid):
        pass


class Design:
    def get_filter_sections(self):
        return [[1., 0., 0., 1., 0., 0.]]
    def get_initial_state(self):
        return [[0., 0.]]


def make():
    mcu = FakeMcu()
    ta = trigger_analog.MCU_trigger_analog(FakeSensor(mcu))
    sos = trigger_analog.MCU_SosFilter(mcu, None, 4)
    sos.set_filter_design(Design())
    ta.setup_sos_filter(sos)
    mcu.connect()
    return mcu, ta, sos


def arm(ta):
    # what a tare does before every probe on the M1 (fixed trigger_counts)
    ta.set_raw_range(-100, 1700000)
    ta.set_trigger('gt', 120000)
    ta._reset_filter()


def names(mcu):
    return [n for n, a in mcu.sent]


class Test(unittest.TestCase):
    def test_unchanged_settings_are_not_resent_without_a_reconnect(self):
        mcu, ta, sos = make()
        arm(ta)
        first = names(mcu)
        self.assertIn('trigger_analog_set_trigger', first)
        self.assertIn('trigger_analog_set_raw_range', first)
        self.assertIn('sos_filter_set_section', first)
        del mcu.sent[:]
        arm(ta)
        self.assertNotIn('trigger_analog_set_trigger', names(mcu))
        self.assertNotIn('trigger_analog_set_raw_range', names(mcu))
        self.assertNotIn('sos_filter_set_section', names(mcu))

    def test_everything_is_resent_after_a_reconnect(self):
        mcu, ta, sos = make()
        arm(ta)
        mcu.connect()       # the toolhead dropped and came back
        del mcu.sent[:]
        arm(ta)
        sent = dict(mcu.sent)
        # before the fix the MCU kept its reset trigger (abs_ge 0, which
        # fires on the first sample: "Probe triggered prior to movement")
        self.assertEqual(sent.get('trigger_analog_set_trigger'),
                         [ta.get_oid(), 'gt',
                          trigger_analog.to_fixed_32(120000)])
        self.assertEqual(sent.get('trigger_analog_set_raw_range'),
                         [ta.get_oid(), -100, 1700000])
        self.assertIn('sos_filter_set_section', names(mcu))
        self.assertIn('sos_filter_set_offset_scale', names(mcu))


if __name__ == '__main__':
    unittest.main()
