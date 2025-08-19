#!/bin/bash
# Simple & robust RP2040 runtime reboot via OpenOCD (CM4 SWD)
set -euo pipefail

# Determine the directory where the script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

OPENOCD_MAIN_CONFIG="${SCRIPT_DIR}/openocd.main.config"
OPENOCD_TOOLHEAD_CONFIG="${SCRIPT_DIR}/openocd.toolhead.config"  # optional

die() { echo "Error: $*" >&2; exit 1; }

reboot_one() {
  local cfg="$1"
  [[ -f "$cfg" ]] || die "OpenOCD config not found: $cfg"

  echo "==> Rebooting via ${cfg}"
  # Split into two -c blocks for robustness; 'reset run' always executes
  openocd -f "${cfg}" \
    -c "init; reset halt" \
    -c "reset run; shutdown"
  echo "    OK: target released (reset run)"
}

# MAIN MCU
reboot_one "${OPENOCD_MAIN_CONFIG}"

# TOOLHEAD MCU (optional)
if [[ -f "${OPENOCD_TOOLHEAD_CONFIG}" ]]; then
  reboot_one "${OPENOCD_TOOLHEAD_CONFIG}"
else
  echo "==> Skipping toolhead (no ${OPENOCD_TOOLHEAD_CONFIG})"
fi

echo "MCUs rebooted successfully."
