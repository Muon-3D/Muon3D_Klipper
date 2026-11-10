# TMC2241 configuration
#
# Copyright (C) 2018-2023  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2023  Alex Voinea <voinea.dragos.alexandru@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# The TMC2241 is the 65V variant of the TMC2240. Its register map is the
# TMC2240 register map plus the DIAG_CONF register and the sg4_thrs_shl
# field. The supply voltage ADC is scaled differently, and the datasheet
# current equation includes the 248/256 amplitude of the default
# microstep table.
import logging, math
from . import tmc, tmc2130, tmc2240, tmc_uart

TMC_FREQUENCY=12500000.

Registers = dict(tmc2240.Registers)
Registers.update({
    "DIAG_CONF":        0x07,
})

ReadRegisters = list(tmc2240.ReadRegisters)
ReadRegisters.insert(ReadRegisters.index("DRV_CONF"), "DIAG_CONF")

Fields = {reg: dict(fields) for reg, fields in tmc2240.Fields.items()}
# Field names must be unique across registers, so the DIAG_CONF fields
# carry a "diagconf_" prefix to keep them apart from the GCONF diag bits.
Fields["DIAG_CONF"] = {
    "diagconf_diag0_error":         0x01 << 0,
    "diagconf_diag0_otpw":          0x01 << 1,
    "diagconf_diag0_stall":         0x01 << 2,
    "diagconf_diag0_index":         0x01 << 3,
    "diagconf_diag0_ev_n":          0x01 << 10,
    "diagconf_diag0_overvoltage":   0x01 << 11,
    "diagconf_diag1_error":         0x01 << 16,
    "diagconf_diag1_otpw":          0x01 << 17,
    "diagconf_diag1_stall":         0x01 << 18,
    "diagconf_diag1_index":         0x01 << 19,
    "diagconf_diag1_ev_n":          0x01 << 26,
    "diagconf_diag1_overvoltage":   0x01 << 27
}
Fields["MSCURACT"] = {
    "cur_b":                    0x1ff << 0,
    "cur_a":                    0x1ff << 16
}
Fields["SG4_THRS"]["sg4_thrs_shl"] = 0x01 << 10

SignedFields = list(tmc2240.SignedFields)

# VS = ADC_VSUPPLY * 17.6mV (TMC2240: 9.732mV)
VSUPPLY_LSB = 0.0176

FieldFormatters = dict(tmc2240.FieldFormatters)
FieldFormatters.update({
    "adc_vsupply":      (lambda v: "0x%04x(%.3fV)" % (v, v * VSUPPLY_LSB)),
    "overvoltage_vth":  (lambda v: "0x%04x(%.3fV)" % (v, v * VSUPPLY_LSB)),
})


######################################################################
# TMC stepper current config helper
######################################################################

# Amplitude of the default microstep table (MSLUT)
MSLUT_AMPLITUDE = 248. / 256.

class TMC2241CurrentHelper(tmc2240.TMC2240CurrentHelper):
    # Burst current (Muon3D, 2026-11-10). The current range (DRV_CONF) is
    # chosen at startup from run_current and caps SET_TMC_CURRENT: 1.370 A
    # for run_current 0.8 A at RREF 12k. With burst_current set, a
    # SET_TMC_CURRENT above that cap moves the driver up a range for as long
    # as it lasts, and a lower one moves it back. burst_time is a backstop:
    # a burst still running that long after it started is ended (back to
    # run_current), so a macro that stops before its own restore can't leave
    # the motors at burst current. Without burst_current, nothing changes.
    def __init__(self, config, mcu_tmc):
        tmc2240.TMC2240CurrentHelper.__init__(self, config, mcu_tmc)
        self.base_range = self.fields.get_field("current_range")
        self.base_run_current = config.getfloat('run_current')
        self.burst_current = config.getfloat(
            'burst_current', None, above=self._get_ifs_rms(self.base_range),
            maxval=self._get_ifs_rms(3))
        self.burst_time = config.getfloat('burst_time', 5., above=0.)
        self.reactor = self.printer.get_reactor()
        self.burst_timer = None
        self.burst_seq = 0
    def _get_ifs_rms(self, current_range=None):
        if current_range is None:
            current_range = self.fields.get_field("current_range")
        KIFS = [11750., 24000., 36000., 36000.]
        return ((KIFS[current_range] / self.Rref) * MSLUT_AMPLITUDE
                / math.sqrt(2.))
    def get_current(self):
        run, hold, req_hold, ifs = (
            tmc2240.TMC2240CurrentHelper.get_current(self))
        # The most SET_TMC_CURRENT may ask for.
        if self.burst_current is not None:
            ifs = max(ifs, self.burst_current)
        return run, hold, req_hold, ifs
    def _write_range(self, current_range, print_time):
        val = self.fields.set_field("current_range", current_range)
        self.mcu_tmc.set_register("DRV_CONF", val, print_time)
    def set_current(self, run_current, hold_current, print_time):
        if self.burst_current is None:
            tmc2240.TMC2240CurrentHelper.set_current(
                self, run_current, hold_current, print_time)
            return
        new_range = max(self.base_range,
                        self._calc_current_range(run_current))
        old_range = self.fields.get_field("current_range")
        # Change the range on the side that keeps the current between the
        # old and new values: currents then range going up, the reverse
        # going down.
        if new_range < old_range:
            self._write_range(new_range, print_time)
        else:
            self.fields.set_field("current_range", new_range)
        tmc2240.TMC2240CurrentHelper.set_current(
            self, run_current, hold_current, print_time)
        if new_range > old_range:
            self.fields.set_field("current_range", old_range)
            self._write_range(new_range, print_time)
        if new_range > self.base_range:
            self._arm_backstop(print_time)
        else:
            self._disarm_backstop()
    def _arm_backstop(self, print_time):
        self.burst_seq += 1
        mcu = self.mcu_tmc.get_mcu()
        now = self.reactor.monotonic()
        lead = max(0., print_time - mcu.estimated_print_time(now))
        waketime = now + lead + self.burst_time
        if self.burst_timer is None:
            self.burst_timer = self.reactor.register_timer(
                self._end_burst, waketime)
        else:
            self.reactor.update_timer(self.burst_timer, waketime)
    def _disarm_backstop(self):
        if self.burst_timer is not None:
            self.reactor.update_timer(self.burst_timer, self.reactor.NEVER)
    def _end_burst(self, eventtime):
        if self.fields.get_field("current_range") > self.base_range:
            msg = ("%s: burst current still on after %.1f s, back to"
                   " run_current %.2f A" % (self.name, self.burst_time,
                                            self.base_run_current))
            logging.warning(msg)
            mcu = self.mcu_tmc.get_mcu()
            self.set_current(self.base_run_current, self.req_hold_current,
                             mcu.estimated_print_time(eventtime) + 0.100)
            gcode = self.printer.lookup_object('gcode', None)
            if gcode is not None:
                gcode.respond_info(msg)
        return self.reactor.NEVER


######################################################################
# TMC2241 printer object
######################################################################

class TMC2241:
    def __init__(self, config):
        # Setup mcu communication
        self.fields = tmc.FieldHelper(Fields, SignedFields, FieldFormatters)
        if config.get("uart_pin", None) is not None:
            # use UART for communication
            self.mcu_tmc = tmc_uart.MCU_TMC_uart(config, Registers, self.fields,
                                                 7, TMC_FREQUENCY)
        else:
            # Use SPI bus for communication
            self.mcu_tmc = tmc2130.MCU_TMC_SPI(config, Registers, self.fields,
                                               TMC_FREQUENCY)
        # Allow virtual pins to be created
        tmc.TMCVirtualPinHelper(config, self.mcu_tmc)
        # Register commands
        current_helper = TMC2241CurrentHelper(config, self.mcu_tmc)
        cmdhelper = tmc.TMCCommandHelper(config, self.mcu_tmc, current_helper)
        cmdhelper.setup_register_dump(ReadRegisters)
        self.get_phase_offset = cmdhelper.get_phase_offset
        self.get_status = cmdhelper.get_status
        # Setup basic register values
        tmc.TMCWaveTableHelper(config, self.mcu_tmc)
        self.fields.set_config_field(config, "offset_sin90", 0)
        tmc.TMCStealthchopHelper(config, self.mcu_tmc)
        tmc.TMCVcoolthrsHelper(config, self.mcu_tmc)
        tmc.TMCVhighHelper(config, self.mcu_tmc)
        # Allow other registers to be set from the config
        set_config_field = self.fields.set_config_field
        #   GCONF
        set_config_field(config, "multistep_filt", True)
        #   CHOPCONF
        set_config_field(config, "toff", 3)
        set_config_field(config, "hstrt", 5)
        set_config_field(config, "hend", 2)
        set_config_field(config, "fd3", 0)
        set_config_field(config, "disfdcc", 0)
        set_config_field(config, "chm", 0)
        set_config_field(config, "tbl", 2)
        set_config_field(config, "vhighfs", 0)
        set_config_field(config, "vhighchm", 0)
        set_config_field(config, "tpfd", 4)
        set_config_field(config, "diss2g", 0)
        set_config_field(config, "diss2vs", 0)
        #   COOLCONF
        set_config_field(config, "semin", 0)
        set_config_field(config, "seup", 0)
        set_config_field(config, "semax", 0)
        set_config_field(config, "sedn", 0)
        set_config_field(config, "seimin", 0)
        set_config_field(config, "sgt", 0)
        set_config_field(config, "sfilt", 0)
        #   IHOLDIRUN
        set_config_field(config, "iholddelay", 6)
        set_config_field(config, "irundelay", 4)
        #   PWMCONF
        set_config_field(config, "pwm_ofs", 29)
        set_config_field(config, "pwm_grad", 0)
        set_config_field(config, "pwm_freq", 0)
        set_config_field(config, "pwm_autoscale", True)
        set_config_field(config, "pwm_autograd", True)
        set_config_field(config, "freewheel", 0)
        set_config_field(config, "pwm_reg", 4)
        set_config_field(config, "pwm_lim", 12)
        #   TPOWERDOWN
        set_config_field(config, "tpowerdown", 10)
        #   SG4_THRS
        set_config_field(config, "sg4_angle_offset", 1)
        set_config_field(config, "sg4_thrs_shl", 0)
        #   DRV_CONF
        set_config_field(config, "slope_control", 0)

def load_config_prefix(config):
    return TMC2241(config)
