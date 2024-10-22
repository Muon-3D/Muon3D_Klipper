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
        self.control_pin.setup_max_duration(0.)  # Ensure no max duration
        self.probing = False
        self.gcode = self.printer.lookup_object('gcode')

        # Probe offsets and session helpers
        self.probe_offsets = probe.ProbeOffsetsHelper(config)
        self.probe_helper = probe.ProbeHelper(self.printer)

        # Register event handlers
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

        # Register G-Code commands
        self.gcode.register_command("PROBE_DEPLOY", self.cmd_PROBE_DEPLOY, desc="Deploy the probe")
        self.gcode.register_command("PROBE_RETRACT", self.cmd_PROBE_RETRACT, desc="Retract the probe")
        self.gcode.register_command("PROBE_TOGGLE", self.cmd_PROBE_TOGGLE, desc="Toggle the probe deployment")
        self.gcode.register_command("PROBE", self.cmd_PROBE, desc="Perform a probe at the current position")
        self.gcode.register_command("QUERY_PROBE", self.cmd_QUERY_PROBE, desc="Query the probe's status")

    def handle_connect(self):
            # Ensure the control pin is configured properly
        self.control_pin.setup_max_duration(0.)  # Ensure no max duration
        # Ensure the probe is retracted on startup
        self.retract_probe()


    def get_position_endstop(self):
        return self.position_endstop

    def get_probe_position(self):
        # Return the XY offsets of the probe relative to the nozzle
        return self.probe_offsets.get_offsets()

    def set_control_pin(self, value):
        print_time = self.reactor.monotonic()
        self.control_pin.set_digital(print_time, value)

    def deploy_probe(self):
        self.set_control_pin(1)
        self.reactor.pause(self.pin_move_time)

    def retract_probe(self):
        
self.set/*************  ✨ Codeium Command ⭐  *************/
"""
Retract the probe by setting the control pin to 0 and pausing for the specified pin move time.
"""
/******  202950d3-5e5c-465f-91a9-3b02a961694f  *******/_control_pin(0)
        self.reactor.pause(self.pin_move_time)

    def toggle_probe(self):
        # Toggle the probe deployment state
        current_state = self.control_pin.get_commanded_value()
        new_state = 0 if current_state else 1
        self.set_control_pin(new_state)
        msg = "Probe deployed" if new_state else "Probe retracted"
        self.gcode.respond_info(msg)

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
                self.retract_probe()

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
        self.retract_probe()
        self.gcode.respond_info("Probe retracted")

    def cmd_PROBE_TOGGLE(self, gcmd):
        # Handle the PROBE_TOGGLE command
        self.toggle_probe()

    def get_status(self, eventtime):
        # Return the status of the probe
        return {
            'position_endstop': self.position_endstop,
            'is_triggered': self.sensor_pin.query_endstop(),
            'probe_deployed': bool(self.control_pin.get_commanded_value()),
        }

def load_config(config):
    return Muon3D_Probe(config)
