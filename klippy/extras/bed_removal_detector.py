import logging


class BedRemovalDetector:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.threshold_temp = config.getfloat('threshold_temp', -20.0)
        self.bed_heater = self.printer.lookup_object('heater_bed')
        # self.toolhead = self.printer.lookup_object('toolhead')
        self.gcode = self.printer.lookup_object('gcode')
        self.bed_removed = False
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

    def handle_ready(self):
        reactor = self.printer.get_reactor()
        reactor.register_timer(self.check_bed_temperature, reactor.NOW)

    def check_bed_temperature(self, eventtime):
        temp, _ = self.bed_heater.get_temp(eventtime)
        if temp < self.threshold_temp:
            if not self.bed_removed:
                self.handle_bed_removal()
        else:
            if self.bed_removed:
                self.handle_bed_reconnection()
        return eventtime + 1.0  # Check every second

    def handle_bed_removal(self):
        self.bed_removed = True
        self.bed_heater.set_temp(0.0)
        logging.info("Bed removed")
        self.gcode.respond_info("Bed removed. Print paused.")
        # self.toolhead.pause()
        # Additional code to notify UI can be added here

    def handle_bed_reconnection(self):
        self.bed_removed = False
        logging.info("Bed removed")
        self.gcode.respond_info("Bed reconnected. You may resume printing.")
        # Additional code to update UI can be added here

    def cmd_START_PRINT(self, gcmd):
        if self.bed_removed:
            raise self.printer.command_error("Cannot start print: Bed is removed.")
        # Proceed with starting the print

def load_config(config):
    return BedRemovalDetector(config)
