#!/bin/bash

#Based on https://docs.vorondesign.com/community/howto/drachenkatze/automating_klipper_mcu_updates.html

# Determine the directory where the script is located.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

KLIPPER_DIR="${SCRIPT_DIR}/../../.."
KLIPPER_OUT_DIR="${KLIPPER_DIR}/out"

MAIN_CONFIG="${SCRIPT_DIR}/config.rp2040"
TOOLHEAD_CONFIG="${MAIN_CONFIG}"
RPI_CONFIG="${SCRIPT_DIR}/config.rpi"

OPENOCD_MAIN_CONFIG="${SCRIPT_DIR}/openocd.main.config"
OPENOCD_TOOLHEAD_CONFIG="${SCRIPT_DIR}/openocd.toolhead.config"

# Stop klipper service
sudo service klipper stop || { echo "Error: Failed to stop klipper service."; exit 1; }

# Change directory to ~/klipper
cd ${KLIPPER_DIR} || { echo "Error: Failed to change directory to ~/klipper."; exit 1; }

# Build steps for MAIN_CONFIG
make clean KCONFIG_CONFIG="${MAIN_CONFIG}" || { echo "Error: 'make clean' for ${MAIN_CONFIG} failed."; exit 1; }
#make menuconfig KCONFIG_CONFIG="${MAIN_CONFIG}" || { echo "Error: 'make menuconfig' for ${MAIN_CONFIG} failed."; exit 1; }
#Force creation of BIN File
if ! grep -q "^CONFIG_RPXXXX_HAVE_BOOTLOADER=y$" "${MAIN_CONFIG}"; then
    echo "CONFIG_RPXXXX_HAVE_BOOTLOADER=y" >> "${MAIN_CONFIG}"
fi
make KCONFIG_CONFIG="${MAIN_CONFIG}" || { echo "Error: 'make' for ${MAIN_CONFIG} failed."; exit 1; }

# Program device using the main OpenOCD config
openocd -f "${OPENOCD_MAIN_CONFIG}" -c "program ${KLIPPER_OUT_DIR}/klipper.bin 0x10000000 verify reset; exit" || { echo "Error: OCD Flasing the MAIN MCU Failed."; exit 1; }
openocd -f "${OPENOCD_TOOLHEAD_CONFIG}" -c "program ${KLIPPER_OUT_DIR}/klipper.bin 0x10000000 verify reset; exit" || { echo "Error: Flasing the TOOLHEAD MCU Failed"; exit 1; }

# Build and flash for RPI_CONFIG
make clean KCONFIG_CONFIG="${RPI_CONFIG}" || { echo "Error: 'make clean' for ${RPI_CONFIG} failed."; exit 1; }
#make menuconfig KCONFIG_CONFIG="${RPI_CONFIG}" || { echo "Error: 'make menuconfig' for ${RPI_CONFIG} failed."; exit 1; }
make flash KCONFIG_CONFIG="${RPI_CONFIG}" || { echo "Error: 'make flash' for ${RPI_CONFIG} failed."; exit 1; }

# Start klipper service
sudo service klipper start || { echo "Error: Failed to start klipper service."; exit 1; }


echo "Firmware Flashed Sucessfully"
