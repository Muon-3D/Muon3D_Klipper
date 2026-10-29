"""KAN-558: object commands must not reach an identifying toolhead."""
import pathlib
import sys
import types
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / 'klippy'))
import mcu
import msgproto
import serialhdl


FORMATS = {
    'query_ads1220_status oid=%c': 20,
    'spi_send oid=%c data=%*s': 21,
    'spi_transfer oid=%c data=%*s': 22,
    'queue_digital_out oid=%c clock=%u on_ticks=%u': 23,
    'get_config': 24,
    'get_clock': 25,
    'get_uptime': 26,
    'reset': 27,
    'config_buttons oid=%c button_count=%c': 28,
    'sensor_bulk_status oid=%c clock=%u query_ticks=%u'
    ' next_sequence=%hu buffered=%u possible_overflows=%hu': 29,
}


def make_serial():
    reader = object.__new__(serialhdl.SerialReader)
    reader.mcu = types.SimpleNamespace(
        is_non_critical=True, non_critical_disconnected=True,
        _connecting=True, _configured=False, get_name=lambda: 'toolhead',
        get_printer=lambda: types.SimpleNamespace(command_error=mcu.error),
        is_fileoutput=lambda: False)
    reader.serialqueue = object()
    reader.msgparser = msgproto.MessageParser()
    reader.msgparser._init_messages(FORMATS)
    reader.default_cmd_queue = object()
    reader.last_notify_id = 0
    reader.pending_notifications = {}
    reader.handlers = {}
    reader.lock = __import__('threading').Lock()
    sent = []
    reader.ffi_lib = types.SimpleNamespace(
        serialqueue_send=lambda sq, cq, cmd, *args: sent.append(bytes(cmd)))
    # Any blocked query must fail before allocating a completion or waiting.
    def unexpected_wait():
        raise AssertionError('unconfigured object query reached wait')
    reader.reactor = types.SimpleNamespace(completion=unexpected_wait)
    reader.get_msgparser = lambda: reader.msgparser
    helper = types.SimpleNamespace(get_serial=lambda: reader,
                                   get_mcu=lambda: reader.mcu)
    return reader, helper, sent


class UnconfiguredTest(unittest.TestCase):
    def test_step4_background_raw_commands_are_suppressed(self):
        reader, _, sent = make_serial()
        for command in ('query_ads1220_status oid=9',
                        'spi_send oid=13 data=0000000000',
                        'queue_digital_out oid=4 clock=0 on_ticks=0'):
            reader.send(command)
        self.assertEqual(sent, [], 'object traffic during identify')

    def test_step4_bulk_and_tmc_queries_fail_before_waiting(self):
        reader, helper, sent = make_serial()
        for fmt, response, args in (
                ('query_ads1220_status oid=%c',
                 'sensor_bulk_status oid=%c clock=%u query_ticks=%u'
                 ' next_sequence=%hu buffered=%u possible_overflows=%hu', [9]),
                ('spi_transfer oid=%c data=%*s',
                 'sensor_bulk_status oid=%c clock=%u query_ticks=%u'
                 ' next_sequence=%hu buffered=%u possible_overflows=%hu',
                 [13, bytes(5)])):
            query = mcu.CommandQueryWrapper(helper, fmt, response)
            with self.assertRaises(mcu.error):
                query.send(args)
        self.assertEqual(sent, [])

    def test_handshake_and_config_still_reach_the_wire(self):
        reader, _, sent = make_serial()
        for command in ('identify offset=0 count=40', 'get_config',
                        'get_clock', 'get_uptime', 'reset'):
            reader.send(command)
        self.assertEqual(len(sent), 5)
        reader.send('config_buttons oid=7 button_count=2', is_config=True)
        self.assertEqual(len(sent), 6)

    def test_configured_reconnect_restores_object_traffic(self):
        reader, helper, sent = make_serial()
        reader.mcu._configured = True
        reader.mcu.non_critical_disconnected = False
        cmd = mcu.CommandWrapper(helper, 'spi_send oid=%c data=%*s')
        cmd.send([13, b'abc'])
        self.assertEqual(len(sent), 1)
        # Losing the link must close the gate even if readiness was stale.
        reader.mcu.non_critical_disconnected = True
        cmd.send([13, b'abc'])
        self.assertEqual(len(sent), 1)

    def test_critical_mcu_path_is_unchanged(self):
        reader, _, sent = make_serial()
        reader.mcu.is_non_critical = False
        reader.send('query_ads1220_status oid=9')
        self.assertEqual(len(sent), 1)

    def test_config_helper_opens_gate_only_after_movequeue_setup(self):
        reader, conn, sent = make_serial()
        device = reader.mcu
        device._name = 'toolhead'
        reader.get_serialqueue = lambda: reader.serialqueue
        conn.log_info = lambda: 'identified'
        config = object.__new__(mcu.MCUConfigHelper)
        config._mcu, config._serial = device, reader
        config._conn_helper = conn
        config._name = 'toolhead'
        config._reserved_move_slots = 0
        stages = []

        def probe(stage):
            stages.append(stage)
            reader.send('query_ads1220_status oid=9')
            self.assertEqual(sent, [], stage)
            self.assertFalse(device._configured)

        def get_config():
            probe('get_config')
            return {'is_config': 1, 'crc': 123, 'move_count': 500}

        config._send_get_config = get_config
        config._send_config = lambda crc: probe('init')
        motion = types.SimpleNamespace(
            setup_mcu_movequeue=lambda *a: probe('movequeue'))
        config._printer = types.SimpleNamespace(
            get_start_args=lambda: {}, lookup_object=lambda name: motion,
            set_rollover_info=lambda *a, **kw: probe('rollover'))
        config._connect(allow_noncritical=True)
        self.assertEqual(stages, ['get_config', 'init', 'movequeue',
                                  'rollover'])
        self.assertTrue(device._configured)
        device.non_critical_disconnected = False
        reader.send('query_ads1220_status oid=9')
        self.assertEqual(len(sent), 1)


if __name__ == '__main__':
    unittest.main()
