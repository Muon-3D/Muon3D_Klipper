# muon3d_fan.py  (Klipper-side fan module)
#
# Printer cooling fan integration with external socket-based comms
#
# Copyright (C) 2016-2024  Kevin O'Connor
# Distributed under GNU GPLv3.

import json
import logging
import subprocess
import sys
import socket
import threading
import queue
import importlib.util
from typing import Any, Callable

class error(Exception):
    pass

class SocketWorker(threading.Thread):
    def __init__(self, socket_address, initial_config_json):
        super().__init__(daemon=True, name="FanSocketWorker")
        self.socket_address    = socket_address
        self.initial_config    = initial_config_json
        self.cmd_queue         = queue.Queue()
        self.callbacks       : dict[str, list[Callable[..., Any]]] = {}
        self.stop_event        = threading.Event()
        self.connected_event   = threading.Event()

    def register_callback(self, func_name: str, cb: Callable[..., Any]):
        """Register cb(*args) for frames where 'function' == func_name."""
        self.callbacks.setdefault(func_name, []).append(cb)
        logging.debug("Callback registered for '%s': %s", func_name, cb)

    def run(self):
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            sock.connect(self.socket_address)
            logging.info("FanSocketWorker: connected to %s", self.socket_address)
            self.connected_event.set()

            # Send initial config
            self._send(sock, "set_fan_config", [self.initial_config])

            # Start background receiver thread
            threading.Thread(target=self._recv_loop, args=(sock,), daemon=True).start()

            # Process outgoing queue
            while not self.stop_event.is_set():
                try:
                    func, args = self.cmd_queue.get(timeout=1.0)
                except queue.Empty:
                    continue
                try:
                    self._send(sock, func, args)
                except Exception as e:
                    logging.error("FanSocketWorker: send error: %s", e)
                    break

        except Exception as e:
            logging.error("FanSocketWorker: connection/setup failed: %s", e)

        finally:
            # Tear down on any failure
            self.connected_event.clear()
            self.stop_event.set()
            try:
                sock.close()
            except Exception:
                pass
            logging.info("FanSocketWorker: terminated")

    def _send(self, sock, func, args):
        """Send one JSON command over the socket."""
        msg = {"function": func, "args": args}
        payload = (json.dumps(msg) + "\n").encode("utf-8")
        sock.sendall(payload)

    def _recv_loop(self, sock: socket.socket):
        """Continuously read lines and dispatch to the right callbacks."""
        buf = b""
        while not self.stop_event.is_set():
            try:
                data = sock.recv(1024)
            except Exception as e:
                logging.error("recv error: %s", e)
                break
            if not data:
                logging.info("Server disconnected")
                break

            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line:
                    continue
                try:
                    msg = json.loads(line.decode("utf-8"))
                except json.JSONDecodeError:
                    logging.warning("Invalid JSON frame: %r", line)
                    continue

                func = msg.get("function")
                args = msg.get("args", [])
                logging.debug("Received frame: %s %s", func, args)

                cbs = self.callbacks.get(func)
                if cbs:
                    for cb in cbs:
                        try:
                            cb(*args)
                        except Exception:
                            logging.exception("Error in callback for '%s'", func)
                else:
                    logging.debug("No callbacks registered for '%s'", func)

        # tear down
        self.connected_event.clear()
        self.stop_event.set()

    def send(self, function_name, args):
        """Queue a fan command; raises if not connected."""
        if not self.connected_event.is_set():
            raise RuntimeError(f"Cannot send '{function_name}': socket not connected")
        self.cmd_queue.put((function_name, args))

    def stop(self):
        self.stop_event.set()


class Fan:
    def __init__(self, config):
        self.printer              = config.get_printer()
        self.last_fan_value       = 0.0
        self.last_req_value       = 0.0
        self.tach_fan_speed_rpm   = 0.0

        # Load external FanConfig module
        self.socket_adress        = config.get('socket_adress')
        self.fan_config_adress    = config.get('fan_config_adress')
        self.openocd_config       = config.get('openocd_config')
        spec = importlib.util.spec_from_file_location("FanConfig", self.fan_config_adress)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        self.FanConfigModule = module.FanConfig
        self.FanConfig       = self.FanConfigModule.from_config(config)
        self.cfg_json = self.FanConfig.json()

        # Start the socket worker and register RPM callback
        self.socket_worker = SocketWorker(self.socket_adress, self.cfg_json)

        self.socket_worker.register_callback("update_rpm", self._recieve_update_rpm)        
        self.socket_worker.register_callback("error", self._recieve_error)

        self.socket_worker.start()

        # Fail fast if listener isn’t up
        if not self.socket_worker.connected_event.wait(timeout=5.0):
            raise error(f"Cannot connect to fan comms socket at {self.socket_adress}")

        # Schedule periodic health checks once Klipper is ready
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

        # Initialize tachometer and gcode restart hook
        self.tachometer = FanTachometer(self)
        self.printer.register_event_handler(
            "gcode:request_restart", self._handle_request_restart
        )
        self.printer.register_event_handler(
            "klippy:firmware_restart", self._firmware_restart)

    def _handle_ready(self):
        reactor = self.printer.get_reactor()
        reactor.register_timer(self._socket_health_check, reactor.NOW)

    def _socket_health_check(self, eventtime):
        """Runs in Klipper's main thread; aborts if socket is disconnected."""
        if not self.socket_worker.connected_event.is_set():
            raise error("Fatal: fan comms socket disconnected")
        return eventtime + 1.0  # reschedule in 1 second
    
    def _firmware_restart(self, force=False):
        logging.info(f"Attempting Reset of Muon3D_Fan MCU via swdio/openocd from config file: {self.openocd_config}")
        cmd = [
            "openocd",
            "-f", self.openocd_config,
            "-c", "init; reset; exit"
        ] #todo handle a bad openocd config file

        # Run the command, capture stdout/stderr, and decode to text
        result = subprocess.run(
            cmd,
            capture_output=True,   # captures both stdout and stderr
            text=True,             # returns strings instead of bytes
            check=True             # raises CalledProcessError on non-zero exit
        )
        # Print the outputs
        logging.info(f"STDOUT: {result.stdout}")
        logging.info(f"STDERR: {result.stderr}")

        self._send_fan_config()

    def _recieve_update_rpm(self, rpm):
        """Callback from SocketWorker to update tachometer reading."""
        try:
            self.tach_fan_speed_rpm = float(rpm)
            logging.debug(f"RPM UPDATED: {rpm}")
        except (TypeError, ValueError):
            logging.warning(f"Invalid RPM value from daemon: {rpm}")

    def _recieve_error(self, error_msg):
        raise Exception(error_msg)

    def send_command_to_socket(self, function_name, *args):
        """Queue a function call on the socket thread."""
        self.socket_worker.send(function_name, list(args))
        logging.debug(f"muon3d_fan: {function_name} ({list(args)})")

    def _send_fan_config(self):
        self.send_command_to_socket("set_fan_config", self.cfg_json)

    def _apply_speed(self, print_time, value):
        self.send_command_to_socket("set_fan_speed", value)

    def set_speed(self, value, print_time=None):
        self.send_command_to_socket("set_fan_speed", value)

    def set_speed_from_command(self, value):
        self.send_command_to_socket("set_fan_speed", value)

    def _handle_request_restart(self, print_time):
        self.set_speed(0.0, print_time)

    def get_status(self, eventtime):
        return {
            'speed': self.last_req_value,
            'rpm':   self.tach_fan_speed_rpm,
        }


class FanTachometer:
    def __init__(self, fanobject):
        self.fanobject = fanobject

    def get_status(self, eventtime):
        return {'rpm': self.fanobject.tach_fan_speed_rpm}


class muon3d_fan:
    def __init__(self, config):
        self.fan = Fan(config)
        gcode = config.get_printer().lookup_object('gcode')
        gcode.register_command("M106", self.cmd_M106)
        gcode.register_command("M107", self.cmd_M107)

    def get_status(self, eventtime):
        return self.fan.get_status(eventtime)

    def cmd_M106(self, gcmd):
        value = gcmd.get_float('S', 255., minval=0.) / 255.0
        self.fan.set_speed_from_command(value)

    def cmd_M107(self, gcmd):
        self.fan.set_speed_from_command(0.0)




def load_config(config):
    return muon3d_fan(config)
