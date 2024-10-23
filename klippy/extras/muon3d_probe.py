import logging
from . import probe

class Muon3D_Probe:
    def __init__(self, config):
        self.next_cmd_time = 0.0

        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        ppins = self.printer.lookup_object('pins')

        # Read configuration parameters
        self.position_endstop = config.getfloat('z_offset', minval=0.)
        self.stow_on_each_sample = config.getboolean('stow_on_each_sample', True)
        self.pin_move_time = config.getfloat('pin_move_time', 0.5, above=0.)
        self.control_pin = ppins.setup_pin('digital_out', config.get('control_pin'))
        self.sensor_pin = ppins.setup_pin('endstop', config.get('sensor_pin'))

        # multi probes state
        self.multi = 'OFF'

        # Assign sensor_pin to mcu_endstop for clarity
        self.mcu_endstop = self.sensor_pin

        # Set initial state
        self.probing = False
        self.gcode = self.printer.lookup_object('gcode')

        # Initialize ProbeCommandHelper and ProbeSessionHelper
        self.cmd_helper = probe.ProbeCommandHelper(
            config, self, self.sensor_pin.query_endstop)
        self.probe_session = probe.ProbeSessionHelper(config, self)
        self.probe_offsets = probe.ProbeOffsetsHelper(config)

        # Wrappers to expose required methods
        self.get_mcu = self.mcu_endstop.get_mcu
        self.add_stepper = self.mcu_endstop.add_stepper
        self.get_steppers = self.mcu_endstop.get_steppers
        self.home_wait = self.mcu_endstop.home_wait
        self.query_endstop = self.mcu_endstop.query_endstop

        # Register event handlers
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

        # Register Debug G-Code commands
        self.gcode.register_command("DEPLOY_PROBE", self.cmd_PROBE_DEPLOY, desc="Deploy the probe")
        self.gcode.register_command("RETRACT_PROBE", self.cmd_PROBE_RETRACT, desc="Retract the probe")
        # self.gcode.register_command("PROBE_TOGGLE", self.cmd_PROBE_TOGGLE, desc="Toggle the probe deployment")

        self.control_pin.setup_max_duration(0.)  # Ensure no max duration



    def handle_connect(self):
        # Ensure the control pin is configured properly
        self.sync_mcu_print_time()
        self.next_cmd_time += 0.200
    
        self.retract_probe()



    def sync_mcu_print_time(self):
        curtime = self.reactor.monotonic()
        mcu = self.control_pin.get_mcu()
        est_time = mcu.estimated_print_time(curtime)
        self.next_cmd_time = max(self.next_cmd_time, est_time + 0.100)  # Schedule at least 100ms in the future

    def sync_print_time(self):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        if self.next_cmd_time > print_time:
            toolhead.dwell(self.next_cmd_time - print_time)
        else:
            self.next_cmd_time = print_time



    def get_probe_params(self, gcmd=None):
        return self.probe_session.get_probe_params(gcmd)
    def get_offsets(self):
        return self.probe_offsets.get_offsets()
    def get_status(self, eventtime):
        return self.cmd_helper.get_status(eventtime)
    def start_probe_session(self, gcmd):
        return self.probe_session.start_probe_session(gcmd)




    def set_control_pin(self, value):
        self.sync_mcu_print_time()
        self.control_pin.set_digital(self.next_cmd_time, value)
        self.action_end_time = self.next_cmd_time + self.pin_move_time
        self.next_cmd_time = self.action_end_time + 0.100  # Add some buffer



    def deploy_probe(self):
        self.set_control_pin(1)
        self.sync_print_time()
        # toolhead = self.printer.lookup_object('toolhead')
        # toolhead.dwell(self.pin_move_time)

    def retract_probe(self):
        # self.sync_print_time()
        self.set_control_pin(0)
        self.sync_print_time()
        # toolhead = self.printer.lookup_object('toolhead')
        # toolhead.dwell(self.pin_move_time)


    # def toggle_probe(self):
    #     # Toggle the probe deployment state
    #     current_state = self.control_pin.get_commanded_value()
    #     new_state = 0 if current_state else 1
    #     self.set_control_pin(new_state)
    #     msg = "Probe deployed" if new_state else "Probe retracted"
    #     self.gcode.respond_info(msg)


    def multi_probe_begin(self):
        if self.stow_on_each_sample:
            return
        self.multi = 'FIRST'
    def multi_probe_end(self):
        if self.stow_on_each_sample:
            return
        self.sync_print_time()
        self.retract_probe()

        self.sync_print_time()
        self.multi = 'OFF'
    def probing_move(self, pos, speed):
        phoming = self.printer.lookup_object('homing')
        return phoming.probing_move(self, pos, speed)
    def probe_prepare(self, hmove):
        if self.multi == 'OFF' or self.multi == 'FIRST':
            self.deploy_probe()
            if self.multi == 'FIRST':
                self.multi = 'ON'
        self.sync_print_time()
    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        self.finish_home_complete = self.mcu_endstop.home_start(
            print_time, sample_time, sample_count, rest_time, triggered)
        # Schedule wait_for_trigger callback
        r = self.printer.get_reactor()
        self.wait_trigger_complete = r.register_callback(self.wait_for_trigger)
        return self.finish_home_complete
    def wait_for_trigger(self, eventtime):
        self.finish_home_complete.wait()
        if self.multi == 'OFF':
            self.retract_probe()
    def probe_finish(self, hmove):
        self.wait_trigger_complete.wait()
        # if self.multi == 'OFF':
        #     self.verify_raise_probe()
        self.sync_print_time()
        if hmove.check_no_movement() is not None:
            raise self.printer.command_error("Muon3D Probe failed to deploy")
    def get_position_endstop(self):
        return self.position_endstop



    # Debug G-Code commands
    def cmd_PROBE_DEPLOY(self, gcmd):
        self.deploy_probe()
        self.gcode.respond_info("Probe deployed")

    def cmd_PROBE_RETRACT(self, gcmd):
        self.retract_probe()
        self.gcode.respond_info("Probe retracted")

    def cmd_PROBE_TOGGLE(self, gcmd):
        self.toggle_probe()

def load_config(config):
    m3dp = Muon3D_Probe(config)
    config.get_printer().add_object('probe', m3dp)
    return m3dp
