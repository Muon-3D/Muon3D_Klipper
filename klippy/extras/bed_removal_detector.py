import logging


class BedRemovalDetector:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.threshold_temp = config.getfloat('threshold_temp', -15.0)
        self.bed = self.printer.lookup_object('heater_bed')
        self.bed_heater = self.bed.heater
        self.bed_check_interval = config.getfloat('interval', 0.1)
        self.gcode = self.printer.lookup_object('gcode')
        self.idle_timeout = self.printer.lookup_object("idle_timeout")
        self.bed_removed = False
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        

    def handle_ready(self):
        reactor = self.printer.get_reactor()
        reactor.register_timer(self.check_bed_temperature, reactor.NOW)

    def check_bed_temperature(self, eventtime):
        temp, _ = self.bed_heater.get_temp(eventtime)
        if temp < self.threshold_temp:
            if not self.bed_removed:
                self.handle_bed_removal(eventtime)
        else:
            if self.bed_removed:
                self.handle_bed_reconnection()
        return eventtime + self.bed_check_interval

    def handle_bed_removal(self, eventtime):
        self.bed_removed = True
        self.bed_heater.set_temp(0.0)
        self.gcode.respond_info("Bed Removed")

        # eventtime = self.reactor.monotonic()
        is_printing = self.idle_timeout.get_status(eventtime)["state"] == "Printing"
        if is_printing:
            self.gcode.run_script("PAUSE")
        # Additional code to notify UI can be added here

    def handle_bed_reconnection(self):
        self.bed_removed = False
        self.gcode.respond_info("Bed Attatched")
        # Additional code to update UI can be added here

    def get_status(self, eventtime):
        return {
            'bedRemoved': self.bed_removed
        }

    # def cmd_START_PRINT(self, gcmd):
    #     if self.bed_removed:
    #         raise self.printer.command_error("Cannot start print: Bed is removed.")
    #     # Proceed with starting the print

def load_config(config):
    return BedRemovalDetector(config)
