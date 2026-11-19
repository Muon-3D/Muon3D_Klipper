# Tests for klippy/extras/muon_lights_out.py (KAN-4767): the finished-frame
# output for Moonraker's muon_lights component. Offline, stub hardware.
import base64
import os
import sys
import unittest

KLIPPY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', '..', 'klippy')
sys.path.insert(0, KLIPPY)

from extras import muon_lights_out  # noqa: E402


class WebError(Exception):
    pass


class Req:
    error = WebError

    def __init__(self, **params):
        self.params = params
        self.response = None

    def get(self, item, default=None, types=None):
        return self.params.get(item, default)

    def send(self, data):
        self.response = data


class Mutex:
    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False


class Pixel:
    """A neopixel stand-in with the attributes the extra relies on."""
    def __init__(self, count=40, ok=True):
        self.color_map = []
        for lidx in range(count):                  # GRB, like the M1
            for n, c in enumerate((1, 0, 2)):
                self.color_map.append((lidx * 3 + n, (lidx, c)))
        self.color_data = bytearray(3 * count)
        self.old_color_data = bytearray(3 * count)
        self.led_helper = type('H', (), {'mutex': Mutex()})()
        self.ok = ok
        self.calls = []

    def send_data(self, print_time=None, retries=8):
        self.calls.append(retries)
        if self.ok:
            self.old_color_data[:] = self.color_data
        return self.ok


class Effect:
    def __init__(self, enabled=False):
        self.enabled = enabled
        self.fadeValue = 0.
        self.fade = None

    def set_fade_time(self, t):
        self.fade = t

    def set_enabled(self, state):
        self.enabled = state


class Gcode:
    def __init__(self):
        self.ready_gcode_handlers = {}
        self.base_gcode_handlers = {}
        self.registered = []

    def register_command(self, *args, **kwargs):
        self.registered.append(args)

    register_mux_command = register_command


class Webhooks:
    def __init__(self):
        self.endpoints = {}

    def register_endpoint(self, path, cb):
        self.endpoints[path] = cb


class Printer:
    def __init__(self):
        self.objects = {'webhooks': Webhooks(), 'gcode': Gcode()}
        self.handlers = {}

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)

    def lookup_objects(self, prefix):
        return [(n, o) for n, o in self.objects.items()
                if n.split()[0] == prefix]

    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)

    def send_event(self, event, *args):
        for cb in self.handlers.get(event, []):
            cb(*args)


class Config:
    def __init__(self, printer, **opts):
        self.printer = printer
        self.opts = opts

    def get_printer(self):
        return self.printer

    def getboolean(self, key, default):
        return self.opts.get(key, default)

    def get(self, key, default):
        return self.opts.get(key, default)

    def getlist(self, key, default):
        return self.opts.get(key, default)


def load_stock_neopixel():
    """The real extras/neopixel.py, with a stub `led` so it imports
    without klippy's mcu chain (which needs termios, so Linux only)."""
    import importlib.util
    import types
    pkg = types.ModuleType('mlo_extras')
    pkg.__path__ = [os.path.join(KLIPPY, 'extras')]
    sys.modules['mlo_extras'] = pkg
    sys.modules['mlo_extras.led'] = pkg.led = types.ModuleType('led')
    spec = importlib.util.spec_from_file_location(
        'mlo_extras.neopixel',
        os.path.join(KLIPPY, 'extras', 'neopixel.py'))
    module = importlib.util.module_from_spec(spec)
    sys.modules['mlo_extras.neopixel'] = module
    spec.loader.exec_module(module)
    return module


def rgb(n, value):
    return base64.b64encode(bytes([value]) * (3 * n)).decode()


def make(enabled=True, ok=True, **opts):
    printer = Printer()
    printer.objects['neopixel Left'] = Pixel(ok=ok)
    printer.objects['neopixel Right'] = Pixel(ok=ok)
    for name in ('ready', 'muon_mode', 'error', 'progress_left'):
        printer.objects['led_effect ' + name] = Effect()
    extra = muon_lights_out.load_config(
        Config(printer, enabled=enabled, **opts))
    printer.send_event('klippy:connect')
    return printer, extra


def call(extra, **params):
    req = Req(**params)
    extra._handle_frame(req)
    return req.response


class TestNoDeadTests(unittest.TestCase):
    """CI runs this file as a script, so unittest.main() only sees the
    classes defined above it (KAN-3421). Keep this class first."""
    def test_main_guard_is_the_last_statement(self):
        import ast
        with open(os.path.abspath(__file__)) as f:
            body = ast.parse(f.read()).body
        guards = [i for i, node in enumerate(body) if isinstance(node, ast.If)
                  and ast.unparse(node.test) == "__name__ == '__main__'"]
        self.assertEqual(len(guards), 1)
        below = [getattr(node, 'name', type(node).__name__)
                 for node in body[guards[0] + 1:]]
        self.assertEqual(below, [])


class TestNoGcodeEntry(unittest.TestCase):
    def test_only_a_webhook_is_registered(self):
        printer, extra = make()
        self.assertEqual(printer.objects['gcode'].registered, [])
        self.assertEqual(list(printer.objects['webhooks'].endpoints),
                         ['muon_lights_out/frame'])

    def test_gcode_style_input_is_rejected(self):
        printer, extra = make()
        for bad in ('SET_LED LED=Left RED=1', 'LED_MODE MODE=off', 5, [1]):
            with self.assertRaises(WebError):
                call(extra, left=bad)
        self.assertEqual(printer.objects['neopixel Left'].calls, [])
        self.assertEqual(extra.counts['frames'], 0)

    def test_negative_control_valid_frame_is_accepted(self):
        printer, extra = make()
        self.assertTrue(call(extra, left=rgb(40, 9))['ok'])

    def test_validation_errors(self):
        printer, extra = make()
        with self.assertRaises(WebError):
            call(extra)                              # nothing at all
        with self.assertRaises(WebError):
            call(extra, left=rgb(39, 1))             # short
        with self.assertRaises(WebError):
            call(extra, left=rgb(41, 1))             # long
        with self.assertRaises(WebError):
            call(extra, right='***not base64***')


class TestDisabledIsInert(unittest.TestCase):
    def test_disabled_refuses_and_hooks_nothing(self):
        printer, extra = make(enabled=False)
        self.assertEqual(printer.handlers, {})
        self.assertEqual(call(extra, left=rgb(40, 1)),
                         {'ok': False, 'reason': 'disabled'})
        self.assertEqual(printer.objects['neopixel Left'].calls, [])


class TestSend(unittest.TestCase):
    def test_both_strips_in_one_call_left_first_one_try_each(self):
        printer, extra = make()
        resp = call(extra, left=rgb(40, 10), right=rgb(40, 20))
        self.assertEqual(resp, {'ok': True, 'sent': ['left', 'right']})
        left = printer.objects['neopixel Left']
        right = printer.objects['neopixel Right']
        self.assertEqual(left.calls, [1])
        self.assertEqual(right.calls, [1])      # retries=1: never the 8x
        self.assertEqual(set(left.color_data), {10})
        self.assertEqual(set(right.color_data), {20})

    def test_channel_order_follows_the_neopixel_color_map(self):
        printer, extra = make()
        data = bytes([1, 2, 3]) * 40            # R=1 G=2 B=3
        call(extra, left=base64.b64encode(data).decode())
        left = printer.objects['neopixel Left']
        self.assertEqual(list(left.color_data[:3]), [2, 1, 3])   # GRB

    def test_failure_is_dropped_counted_and_not_retried(self):
        printer, extra = make(ok=False)
        resp = call(extra, left=rgb(40, 5), right=rgb(40, 5))
        self.assertEqual(resp, {'ok': False, 'reason': 'failed'})
        self.assertEqual(printer.objects['neopixel Left'].calls, [1])
        self.assertEqual(printer.objects['neopixel Right'].calls, [])
        status = extra.get_status()
        self.assertEqual(status['failed'], 1)
        self.assertEqual(status['dropped'], 1)
        self.assertEqual(status['drops'], {'failed': 1})
        # the next frame must be a full resend, not "unchanged"
        left = printer.objects['neopixel Left']
        self.assertNotEqual(left.old_color_data, left.color_data)

    def test_stock_neopixel_retries_unless_asked_not_to(self):
        # The control for the above: stock send_data tries 8 times by
        # default, and exactly once with retries=1.
        neopixel = load_stock_neopixel()
        for retries, expect in ((8, 8), (1, 1)):
            pix = object.__new__(neopixel.PrinterNeoPixel)
            queries = []

            class Printer_:
                def get_start_args(self):
                    return {}
            pix.printer = Printer_()
            pix.color_data = bytearray([1, 2, 3])
            pix.old_color_data = bytearray(3)
            pix.oid = 1
            pix.mcu = type('M', (), {})()
            pix.neopixel_update_cmd = type(
                'U', (), {'send': lambda self, *a, **k: None})()
            pix.neopixel_send_cmd = type('S', (), {
                'send': lambda self, *a, **k: queries.append(1) or
                {'success': 0}})()
            self.assertFalse(pix.send_data(None, retries=retries))
            self.assertEqual(len(queries), expect)


class TestGates(unittest.TestCase):
    def test_homing_drops_frames_until_it_ends(self):
        printer, extra = make()
        printer.send_event('homing:home_rails_begin', None, None)
        self.assertEqual(call(extra, left=rgb(40, 1))['reason'], 'busy')
        printer.send_event('homing:home_rails_end', None, None)
        self.assertTrue(call(extra, left=rgb(40, 1))['ok'])
        self.assertEqual(printer.objects['neopixel Left'].calls, [1])

    def test_wrapped_command_gates_and_always_releases(self):
        printer, extra = make()
        gcode = printer.objects['gcode']
        seen = []

        def probe(params):
            seen.append(call(extra, left=rgb(40, 2)))
            raise RuntimeError('probe failed')
        gcode.ready_gcode_handlers['BED_MESH_CALIBRATE'] = probe
        printer.send_event('klippy:connect')        # wraps after load
        with self.assertRaises(RuntimeError):
            gcode.ready_gcode_handlers['BED_MESH_CALIBRATE']({})
        self.assertEqual(seen[0]['reason'], 'busy')
        self.assertTrue(call(extra, left=rgb(40, 2))['ok'])  # released

    def test_fault_and_print_effects_win(self):
        printer, extra = make()
        printer.objects['led_effect progress_left'].enabled = True
        self.assertEqual(call(extra, left=rgb(40, 3))['reason'], 'print')
        printer.objects['led_effect error'].enabled = True
        self.assertEqual(call(extra, left=rgb(40, 3))['reason'], 'fault')
        self.assertEqual(extra.get_status()['blocked'], 'fault')
        printer.objects['led_effect error'].enabled = False
        printer.objects['led_effect progress_left'].enabled = False
        self.assertTrue(call(extra, left=rgb(40, 3))['ok'])
        self.assertEqual(printer.objects['neopixel Left'].calls, [1])

    def test_idle_effects_stand_down_for_the_one_writer(self):
        printer, extra = make()
        ready = printer.objects['led_effect ready']
        mode = printer.objects['led_effect muon_mode']
        ready.enabled = mode.enabled = True
        self.assertTrue(extra.get_status()['idle_effect_on'])
        self.assertIsNone(extra.get_status()['blocked'])
        self.assertTrue(call(extra, left=rgb(40, 4))['ok'])
        self.assertFalse(ready.enabled or mode.enabled)
        self.assertEqual(ready.fade, 0.)
        self.assertFalse(extra.get_status()['idle_effect_on'])

    def test_shutdown_drops_everything(self):
        printer, extra = make()
        printer.send_event('klippy:shutdown')
        self.assertEqual(call(extra, left=rgb(40, 1))['reason'], 'busy')

    def test_stop_led_effects_is_counted_and_still_runs(self):
        printer, extra = make()
        gcode = printer.objects['gcode']
        ran = []
        gcode.ready_gcode_handlers['STOP_LED_EFFECTS'] = ran.append
        printer.send_event('klippy:connect')
        gcode.ready_gcode_handlers['STOP_LED_EFFECTS']({})
        self.assertEqual(ran, [{}])
        self.assertEqual(extra.get_status()['stop_count'], 1)


if __name__ == '__main__':
    unittest.main()
