# Simple Probe support
#
# This file provides support for a simple bed probe that uses a control pin to deploy/retract
# and a sense pin to detect when the probe is triggered.
#
# Place this file in klippy/extras/simple_probe.py

import logging
from . import probe

class Muon3D_Probe:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        ppins = self.printer.lookup_object('pins')

        # Read configuration parameters
        self.position_endstop = config.getfloat('z_offset', minval=0.)
        self.stow_on_each_sample = config.getboolean('stow_on_each_sample', True)
        self.pin_move_time = config.getfloat('pin_move_time', 0.5, above=0.)
        self.control_pin = ppins.setup_pin('digital_out', config.get('control_pin'))
        self.sensor_pin = ppins.setup_pin('endstop', config.get('sensor_pin'))

        # Set initial state
        self.control_pin.set_digital(0)  # Ensure probe is retracted
        self.probing = False
        self.gcode = self.printer.lookup_object('gcode')

        # Probe offsets and session helpers
        self.probe_offsets = probe.ProbeOffsetsHelper(config)
        self.probe_helper = probe.ProbeHelper(self.printer)
        self.raise_probe()

        # Register event handlers
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

        # Register G-Code commands
        self.gcode.register_command("PROBE_DEPLOY", self.cmd_PROBE_DEPLOY, desc="Deploy the probe")
        self.gcode.register_command("PROBE_RETRACT", self.cmd_PROBE_RETRACT, desc="Retract the probe")
        self.gcode.register_command("PROBE_TOGGLE", self.cmd_PROBE_TOGGLE, desc="Toggle the probe deployment")
        self.gcode.register_command("PROBE", self.cmd_PROBE, desc="Perform a probe at the current position")
        self.gcode.register_command("QUERY_PROBE", self.cmd_QUERY_PROBE, desc="Query the probe's status")

    def handle_connect(self):
        # Ensure the probe is retracted on startup
        self.raise_probe()

    def get_position_endstop(self):
        return self.position_endstop

    def get_probe_position(self):
        # Return the XY offsets of the probe relative to the nozzle
        return self.probe_offsets.get_offsets()

    def deploy_probe(self):
        self.control_pin.set_digital(1)
        self.reactor.pause(self.pin_move_time)

    def raise_probe(self):
        self.control_pin.set_digital(0)
        self.reactor.pause(self.pin_move_time)

    def toggle_probe(self):
        # Toggle the probe deployment state
        current_state = self.control_pin.get_digital()
        if current_state:
            self.raise_probe()
            self.gcode.respond_info("Probe retracted")
        else:
            self.deploy_probe()
            self.gcode.respond_info("Probe deployed")

    def run_probe(self, gcmd):
        # Start probing sequence
        self.deploy_probe()
        try:
            # Perform probing move
            hmove = self.printer.lookup_object('homing')
            pos = self.probe_helper.get_probe_position(gcmd)
            speed = self.probe_helper.get_probe_speed(gcmd)
            hmove.probing_move(self.sensor_pin, pos, speed)
        finally:
            if self.stow_on_each_sample:
                self.raise_probe()

    def cmd_PROBE(self, gcmd):
        # Handle the PROBE command
        self.run_probe(gcmd)

    def cmd_QUERY_PROBE(self, gcmd):
        # Handle the QUERY_PROBE command
        triggered = self.sensor_pin.query_endstop()
        msg = "probe: %s" % ("TRIGGERED" if triggered else "open")
        self.gcode.respond_info(msg)

    def cmd_PROBE_DEPLOY(self, gcmd):
        # Handle the PROBE_DEPLOY command
        self.deploy_probe()
        self.gcode.respond_info("Probe deployed")

    def cmd_PROBE_RETRACT(self, gcmd):
        # Handle the PROBE_RETRACT command
        self.raise_probe()
        self.gcode.respond_info("Probe retracted")

    def cmd_PROBE_TOGGLE(self, gcmd):
        # Handle the PROBE_TOGGLE command
        self.toggle_probe()

    def get_status(self, eventtime):
        # Return the status of the probe
        return {
            'position_endstop': self.position_endstop,
            'is_triggered': self.sensor_pin.query_endstop(),
            'probe_deployed': bool(self.control_pin.get_digital()),
        }

def load_config(config):
    return Muon3D_Probe(config)
