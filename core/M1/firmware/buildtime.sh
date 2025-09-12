#!/bin/bash

#Based on https://docs.vorondesign.com/community/howto/drachenkatze/automating_klipper_mcu_updates.html

# Determine the directory where the script is located.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

KLIPPER_DIR="${SCRIPT_DIR}/../../.."
KLIPPER_OUT_DIR="${KLIPPER_DIR}/out"

MAIN_CONFIG="${SCRIPT_DIR}/config.rp2040"
TOOLHEAD_CONFIG="${MAIN_CONFIG}"
RPI_CONFIG="${SCRIPT_DIR}/config.rpi"

MAIN_BIN="${SCRIPT_DIR}/rp2040.bin"


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
cp "${KLIPPER_OUT_DIR}/klipper.bin" "${MAIN_BIN}"
echo "Main Binary created and statshed at: ${MAIN_BIN}"



echo "Building and Flashing RPI Host Microcontroller, does not require seperate reflashing"

cd ~/klipper/
sudo cp ./scripts/klipper-mcu.service /etc/systemd/system/
systemctl enable klipper-mcu.service

# Build and flash for RPI_CONFIG
make clean KCONFIG_CONFIG="${RPI_CONFIG}" || { echo "Error: 'make clean' for ${RPI_CONFIG} failed."; exit 1; }
#make menuconfig KCONFIG_CONFIG="${RPI_CONFIG}" || { echo "Error: 'make menuconfig' for ${RPI_CONFIG} failed."; exit 1; }
make flash KCONFIG_CONFIG="${RPI_CONFIG}" || { echo "Error: 'make flash' for ${RPI_CONFIG} failed."; exit 1; }

make clean KCONFIG_CONFIG="${RPI_CONFIG}" || { echo "Error: 'make clean' for ${RPI_CONFIG} failed."; exit 1; }

#TODO add FORCE_VERSION="build version info" to force version string, allowing us to check if the version is correct, and update it if not. Get build version from an enviroment variobale form the cd/ci pipeline?
# Mabye use a combined hash of the c files, toolchain and config file? Rather than reflashing every time there is a system update?

echo "Flashed RPI Host MCU"