#!/bin/bash

#Based on https://docs.vorondesign.com/community/howto/drachenkatze/automating_klipper_mcu_updates.html

# Determine the directory where the script is located.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MAIN_BIN="${SCRIPT_DIR}/rp2040.bin"

OPENOCD_MAIN_CONFIG="${SCRIPT_DIR}/openocd.main.config"
OPENOCD_TOOLHEAD_CONFIG="${SCRIPT_DIR}/openocd.toolhead.config"



# Program device using the main OpenOCD config
openocd -f "${OPENOCD_MAIN_CONFIG}" -c "init; reset run; exit" || { echo "Error: OCD Restarting the MAIN MCU Failed."; exit 1; }
openocd -f "${OPENOCD_TOOLHEAD_CONFIG}" -c "init; reset run; exit" || { echo "Error: OCD Restarting the TOOLHEAD MCU Failed"; exit 1; }


echo "MCU's Rebooted sucessfully"
