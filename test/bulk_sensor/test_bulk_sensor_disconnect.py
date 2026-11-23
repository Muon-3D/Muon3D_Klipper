# KAN-5663: a bulk sensor whose non-critical MCU (the toolhead) disconnects
# stops with one INFO line, not a traceback; any other error still logs one.
import os
import sys
import unittest

KLIPPY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

from extras import bulk_sensor  # noqa: E402


class CommandError(Exception):
    pass


class Reactor:
    NEVER = 9e99
    def get_reactor(self):
        return self
    def unregister_timer(self, timer):
        pass


class Printer(Reactor):
    command_error = CommandError
    def __init__(self, disconnected):
        self.mcu = type('MCU', (), {'non_critical_disconnected':
                                    disconnected})()
    def lookup_objects(self, module=None):
        return [('mcu', object()), ('mcu toolhead', self.mcu)]


def helper(disconnected):
    def batch(eventtime):
        raise CommandError("Unable to obtain 'sensor_bulk_status' response")
    h = bulk_sensor.BatchBulkHelper(Printer(disconnected), batch)
    h.is_started = True
    return h


class TestBatchErrorOnDisconnect(unittest.TestCase):
    def test_disconnect_is_one_info_line_without_a_traceback(self):
        h = helper(True)
        with self.assertLogs(level='INFO') as logs:
            self.assertEqual(h._proc_batch(1.), Reactor.NEVER)
        self.assertEqual(len(logs.records), 1)
        self.assertEqual(logs.records[0].levelname, 'INFO')
        self.assertIsNone(logs.records[0].exc_info)
        self.assertIn('MCU disconnected', logs.output[0])
        self.assertFalse(h.is_started)

    def test_any_other_error_still_logs_the_traceback(self):
        h = helper(False)
        with self.assertLogs(level='INFO') as logs:
            h._proc_batch(1.)
        self.assertEqual(logs.records[0].levelname, 'ERROR')
        self.assertIsNotNone(logs.records[0].exc_info)


if __name__ == '__main__':
    unittest.main()
