# Finished-frame output for the Moonraker "muon_lights" component (KAN-4767).
#
# Hard rule: the lights never cost the printer anything. The LED strips share
# the main RP2040 with motion, so this extra does NO rendering. Moonraker
# renders every effect and stream on the host and hands klippy finished
# bytes. Here they are copied into the existing neopixel objects and sent
# once, without neopixel's 8x retry, and dropped when anything else matters
# more.
#
# The only way in is the webhooks endpoint "muon_lights_out/frame". There is
# no G-code command here (nothing is registered with the gcode object), so no
# macro, console or print file can drive it. Moonraker is the one caller.
#
# A frame is dropped, and counted, while:
#   - homing, probing, mesh, Z_POWER work or the shutdown state is active
#     ("busy"); the strips simply hold their last frame;
#   - a fault or print led_effect is enabled ("fault" / "print"): faults and
#     the print state win, so there is only ever one writer;
#   - the MCU does not confirm the send ("failed"); one try, no retry.
#
# Copyright (C) 2026 Muon3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import base64
import binascii
import logging

DEFAULT_IDLE_EFFECTS = ('ready', 'muon_mode')
DEFAULT_FAULT_EFFECTS = ('error', 'warning')
# Commands that keep the toolhead or the Z power path busy. Each is wrapped
# only if it exists, so a printer without a probe loses nothing.
DEFAULT_GATE_COMMANDS = (
    'PROBE', 'PROBE_ACCURACY', 'PROBE_CALIBRATE', 'BED_MESH_CALIBRATE',
    'Z_ENDSTOP_CALIBRATE', 'Z_POWER_RESET', 'Z_POWER_WAIT', 'Z_POWER_REQUIRE',
    'LOAD_CELL_TEST_TAP', 'LOAD_CELL_CALIBRATE_MAX')
HOMING_EVENTS = (('homing:home_rails_begin', 1), ('homing:home_rails_end', -1),
                 ('homing_scope:begin', 1), ('homing_scope:end', -1),
                 ('homing:homing_move_begin', 1),
                 ('homing:homing_move_end', -1))
STRIPS = ('left', 'right')


class MuonLightsOut:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.enabled = config.getboolean('enabled', False)
        self.names = {'left': config.get('left', 'Left'),
                      'right': config.get('right', 'Right')}
        self.idle_effects = tuple(config.getlist(
            'idle_effects', DEFAULT_IDLE_EFFECTS))
        self.fault_effects = tuple(config.getlist(
            'fault_effects', DEFAULT_FAULT_EFFECTS))
        self.gate_commands = tuple(config.getlist(
            'gate_commands', DEFAULT_GATE_COMMANDS))
        self.pixels = {}          # strip -> neopixel object
        self.sizes = {}           # strip -> LED count
        self.homing = 0
        self.busy_commands = 0
        self.down = False
        self.stop_count = 0
        self.counts = {'sent': 0, 'frames': 0, 'dropped': 0, 'failed': 0}
        self.drops = {}
        self.last_reason = None
        webhooks = self.printer.lookup_object('webhooks')
        webhooks.register_endpoint('muon_lights_out/frame', self._handle_frame)
        if not self.enabled:
            return
        for event, step in HOMING_EVENTS:
            self.printer.register_event_handler(
                event, lambda *args, s=step: self._homing(s))
        self.printer.register_event_handler('klippy:connect',
                                            self._handle_connect)
        self.printer.register_event_handler('klippy:shutdown',
                                            self._handle_shutdown)
        self.printer.register_event_handler('klippy:disconnect',
                                            self._handle_shutdown)

    # -- wiring -------------------------------------------------------------
    def _handle_connect(self):
        for strip in STRIPS:
            name = 'neopixel ' + self.names[strip]
            pixel = self.printer.lookup_object(name, None)
            if pixel is None:
                logging.warning("muon_lights_out: no [%s]; %s strip off",
                                name, strip)
                continue
            self.pixels[strip] = pixel
            self.sizes[strip] = 1 + max(
                lidx for _, (lidx, _) in pixel.color_map)
        gcode = self.printer.lookup_object('gcode')
        for cmd in self.gate_commands + ('STOP_LED_EFFECTS',):
            self._wrap(gcode, cmd)

    def _wrap(self, gcode, cmd):
        # Wrap the registered handler in place. register_command() would
        # run the parameters through the extended-params wrapper twice.
        for table in (gcode.ready_gcode_handlers, gcode.base_gcode_handlers):
            old = table.get(cmd)
            if old is None:
                continue
            if cmd == 'STOP_LED_EFFECTS':
                table[cmd] = self._stop_wrapper(old)
            else:
                table[cmd] = self._busy_wrapper(old)

    def _busy_wrapper(self, old):
        def wrapped(params):
            self.busy_commands += 1
            try:
                return old(params)
            finally:
                self.busy_commands = max(0, self.busy_commands - 1)
        return wrapped

    def _stop_wrapper(self, old):
        def wrapped(params):
            self.stop_count += 1
            return old(params)
        return wrapped

    def _homing(self, step):
        self.homing = max(0, self.homing + step)

    def _handle_shutdown(self, *args):
        self.down = True

    # -- gates --------------------------------------------------------------
    def _effects(self):
        try:
            return self.printer.lookup_objects('led_effect')
        except Exception:
            return []

    def _blocking(self):
        """Why a frame must not be sent right now, or None."""
        if self.down:
            return 'busy'
        if self.homing or self.busy_commands:
            return 'busy'
        reason = None
        for name, effect in self._effects():
            tail = name.split()[-1]
            if tail in self.idle_effects:
                continue
            if getattr(effect, 'enabled', False) or getattr(
                    effect, 'fadeValue', 0.) > 0.:
                if tail in self.fault_effects:
                    return 'fault'
                reason = 'print'
        return reason

    def _idle_effect_on(self):
        return any(name.split()[-1] in self.idle_effects
                   and getattr(effect, 'enabled', False)
                   for name, effect in self._effects())

    def _take_over(self):
        # One writer: the idle looks (ready, muon_mode) stand down while a
        # user preset or a stream is on the strips.
        for name, effect in self._effects():
            if (name.split()[-1] in self.idle_effects
                    and getattr(effect, 'enabled', False)):
                effect.set_fade_time(0.)
                effect.set_enabled(False)

    # -- the endpoint -------------------------------------------------------
    def _drop(self, reason):
        self.counts['dropped'] += 1
        self.drops[reason] = self.drops.get(reason, 0) + 1
        self.last_reason = reason
        return {'ok': False, 'reason': reason}

    def _decode(self, web_request):
        """Both strips arrive in one call, as base64 RGB. Either may be
        left out when it did not change. Validate everything first."""
        frames = {}
        for strip in STRIPS:
            raw = web_request.get(strip, None)
            if raw is None:
                continue
            if not isinstance(raw, str):
                raise web_request.error("Invalid Argument Type [%s]"
                                        % (strip,))
            try:
                data = base64.b64decode(raw, validate=True)
            except (binascii.Error, ValueError):
                raise web_request.error("Invalid base64 in [%s]" % (strip,))
            want = 3 * self.sizes.get(strip, 0)
            if len(data) != want:
                raise web_request.error(
                    "[%s] needs %d bytes, got %d" % (strip, want, len(data)))
            frames[strip] = data
        if not frames:
            raise web_request.error("Missing Argument [left] or [right]")
        return frames

    def _handle_frame(self, web_request):
        if not self.enabled:
            web_request.send({'ok': False, 'reason': 'disabled'})
            return
        if not self.pixels:
            web_request.send({'ok': False, 'reason': 'busy'})
            return
        frames = self._decode(web_request)
        self.counts['frames'] += 1
        reason = self._blocking()
        if reason is not None:
            web_request.send(self._drop(reason))
            return
        self._take_over()
        sent = []
        for strip in STRIPS:        # one after the other, left first
            if strip not in frames:
                continue
            if not self._send_strip(strip, frames[strip]):
                self.counts['failed'] += 1
                web_request.send(self._drop('failed'))
                return
            sent.append(strip)
        self.counts['sent'] += 1
        self.last_reason = None
        web_request.send({'ok': True, 'sent': sent})

    def _send_strip(self, strip, rgb):
        pixel = self.pixels[strip]
        data = pixel.color_data
        for cdidx, (lidx, cidx) in pixel.color_map:
            data[cdidx] = rgb[3 * lidx + cidx] if cidx < 3 else 0
        helper = getattr(pixel, 'led_helper', None)
        mutex = getattr(helper, 'mutex', None)
        try:
            if mutex is not None:
                with mutex:
                    ok = pixel.send_data(None, retries=1)
            else:
                ok = pixel.send_data(None, retries=1)
        except Exception:
            logging.exception("muon_lights_out: send failed")
            ok = False
        if not ok:
            # The MCU buffer may be half updated; make the next send full.
            pixel.old_color_data[:] = bytearray(d ^ 1 for d in data)
        return ok

    def get_status(self, eventtime=None):
        return {'enabled': self.enabled,
                'blocked': self._blocking() if self.enabled else None,
                'idle_effect_on': (self._idle_effect_on()
                                   if self.enabled else False),
                'stop_count': self.stop_count,
                'sent': self.counts['sent'],
                'frames': self.counts['frames'],
                'dropped': self.counts['dropped'],
                'failed': self.counts['failed'],
                'drops': dict(self.drops),
                'last_reason': self.last_reason,
                'leds': dict(self.sizes)}


def load_config(config):
    return MuonLightsOut(config)
