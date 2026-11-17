# Deprecated name for accelerometer_probe
#
# Keeps [adxl345_probe] configs and adxl345_probe:<axis>_virtual_endstop
# pins working: the section name becomes the pin chip name, so the alias
# needs nothing more than loading the same class.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import accelerometer_probe


def load_config(config):
    return accelerometer_probe.load_config(config)


def load_config_prefix(config):
    return accelerometer_probe.load_config_prefix(config)
