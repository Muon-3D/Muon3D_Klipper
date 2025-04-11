#!/bin/bash

#Based on https://docs.vorondesign.com/community/howto/drachenkatze/automating_klipper_mcu_updates.html

# Determine the directory where the script is located.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MAIN_BIN="${SCRIPT_DIR}/rp2040.bin"

OPENOCD_MAIN_CONFIG="${SCRIPT_DIR}/openocd.main.config"
OPENOCD_TOOLHEAD_CONFIG="${SCRIPT_DIR}/openocd.toolhead.config"



# Stop klipper service
sudo service klipper stop || { echo "Error: Failed to stop klipper service."; exit 1; }


# Program device using the main OpenOCD config
openocd -f "${OPENOCD_MAIN_CONFIG}" -c "program ${MAIN_BIN}/klipper.bin 0x10000000 verify reset; exit" || { echo "Error: OCD Flasing the MAIN MCU Failed."; exit 1; }
openocd -f "${OPENOCD_TOOLHEAD_CONFIG}" -c "program ${MAIN_BIN}/klipper.bin 0x10000000 verify reset; exit" || { echo "Error: Flasing the TOOLHEAD MCU Failed"; exit 1; }


# Start klipper service
sudo service klipper start || { echo "Error: Failed to start klipper service."; exit 1; }


echo "Firmware Flashed Sucessfully"
