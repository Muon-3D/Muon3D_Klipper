# Printer cooling fan
#
# Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import json
from . import pulse_counter, output_pin
import socket
import importlib

class Fan:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.last_fan_value = self.last_req_value = 0.
        # Read config

        #TODO, Hardcode all values so user cant chanegthem and damage system

        # # ============== BASIC MOTOR/DRIVER PARAMETERS ==============
        # self.voltage_power_supply = config.getfloat('voltage_power_supply', 45.0, above=1.0, maxval=48.0)
        # self.pwm_frequency = config.getint('voltage_power_supply', 25000, above=500, maxval=120000)#120k pulled out my arse
        # self.phase_resistance = config.getfloat('phase_resistance', 2.0, above=0.2, maxval=10.0)
        # self.fos = config.getfloat('factor_of_saftey', 1, above=0.5, maxval=10.0)
        # self.acceleration_max_current = config.getfloat('acceleration_max_current', 4.0, above=0.1, maxval=5)
        # self.acceleration_rate = config.getfloat('acceleration_rate', 3333.0)
        # self.motor_kv = config.getfloat('motor_kv', 14000.0)

        # # ============== SPEED/STATE PARAMETERS ==============
        # self.motor_kv = config.getfloat('motor_kv', 14000.0)
        # self.min_rpm = config.getfloat('min_rpm', 200.0) # minimal speed to consider "running"
        # self.max_rpm = config.getfloat('max_rpm', 20000.0, above=self.min_rpm, maxval=20000.0)
        # self.rpm_tolerance = config.getfloat('rpm_tolerance', 200.0) # used for ACCEL->STEADY transitions

        # #============== "STEADY STATE" OVERHEAD ==============
        # self.steady_state_current = config.getfloat('steady_state_current', 0.4, below=2)

        # # ============== STANDBY + KICK PARAMETERS ==============
        # self.standby_speed_rpm = config.getfloat('standby_speed_rpm', 20.0)
        # self.standby_current = config.getfloat('standby_current', 0.3)
        # self.standby_kick_current = config.getfloat('standby_kick_current', 1.0)

        # # ============== KICK TIMING ==============
        # self.kick_time_ms_if_standby = config.getfloat('kick_time_ms_if_standby', 100)
        # self.kick_time_ms_from_stop = config.getfloat('kick_time_ms_from_stop', 300)

        # # ============== DISABLE THRESHOLD ==============
        # # If no speed above min_rpm for more than this time, disable driver
        # self.disable_threshold_ms = config.getfloat('disable_threshold_ms', 5*60*1000) # 5 minutes



        self.socket_adress = config.get('socket_adress')
        self.fan_config_adress = config.get('fan_config_adress')#.py file

        self.FanConfigModule = importlib.import_module(self.fan_config_adress)

        self.FanConfig = self.FanConfigModule.from_config(config)

        self.client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.client.connect(self.socket_adress)
        #TODO Error handling socket connection

               
        self.send_command_to_socket(self.client, "set_fan_config", self.FanConfig)
        

        # Setup tachometer
        self.tachometer = FanTachometer(config)

        # Register callbacks
        self.printer.register_event_handler("gcode:request_restart",
                                            self._handle_request_restart)
        

    def send_command_to_socket(sock, function_name, *args):
        """
        Build and send a command to the server, then wait for a response.
        :param sock: The connected socket.
        :param function_name: The function to call on the server (as a string).
        :param args: A list of arguments to be passed to the function.
        """
        message = {
            "function": function_name,
            "args": list(args)
        }
        
        # Send the JSON-encoded command
        sock.sendall(json.dumps(message).encode())
        
        # Wait for a response (assuming response size fits in one recv call)
        response_data = sock.recv(1024)
        response = json.loads(response_data.decode())
        print("Response:", response)
        return response


    


    def _apply_speed(self, print_time, value):
        self.send_command_to_socket(self.client, "set_fan_speed", value)
    def set_speed(self, value, print_time=None):
        self.send_command_to_socket(self.client, "set_fan_speed", value)
    def set_speed_from_command(self, value):
        self.send_command_to_socket(self.client, "set_fan_speed", value)
    def _handle_request_restart(self, print_time):
        self.set_speed(0., print_time)

    def get_status(self, eventtime):
        tachometer_status = self.tachometer.get_status(eventtime)
        return {
            'speed': self.last_req_value,
            'rpm': tachometer_status['rpm'],
        }

class FanTachometer:
    def __init__(self, config):
        printer = config.get_printer()
        self._freq_counter = None

        pin = config.get('tachometer_pin', None)
        if pin is not None:
            self.ppr = config.getint('tachometer_ppr', 2, minval=1)
            poll_time = config.getfloat('tachometer_poll_interval',
                                        0.0015, above=0.)
            sample_time = 1.
            self._freq_counter = pulse_counter.FrequencyCounter(
                printer, pin, sample_time, poll_time)

    def get_status(self, eventtime):
        if self._freq_counter is not None:
            rpm = self._freq_counter.get_frequency() * 30. / self.ppr
        else:
            rpm = None
        return {'rpm': rpm}

class muon3d_fan:
    def __init__(self, config):
        self.fan = Fan(config)
        # Register commands
        gcode = config.get_printer().lookup_object('gcode')
        gcode.register_command("M106", self.cmd_M106)
        gcode.register_command("M107", self.cmd_M107)
    def get_status(self, eventtime):
        return self.fan.get_status(eventtime)
    def cmd_M106(self, gcmd):
        # Set fan speed
        value = gcmd.get_float('S', 255., minval=0.) / 255.
        self.fan.set_speed_from_command(value)
    def cmd_M107(self, gcmd):
        # Turn fan off
        self.fan.set_speed_from_command(0.)

def load_config(config):
    return muon3d_fan(config)
