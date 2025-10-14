#!/bin/bash
# Simple, robust RP2040 flasher via OpenOCD (CM4 SWD) — ELF-only
set -euo pipefail

# Paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAIN_ELF="${SCRIPT_DIR}/rp2040.elf"   # <-- build should produce this

OPENOCD_MAIN_CONFIG="${SCRIPT_DIR}/openocd.main.config"
OPENOCD_TOOLHEAD_CONFIG="${SCRIPT_DIR}/openocd.toolhead.config"

# Helpers
die() { echo "Error: $*" >&2; exit 1; }

flash_one() {
  local cfg="$1"
  local elf="$2"

  [[ -f "$cfg" ]] || die "OpenOCD config not found: $cfg"
  [[ -f "$elf" ]] || die "Firmware file not found: $elf"

  echo "==> Flashing with $cfg"
  openocd -f "$cfg" \
    -c "init; reset halt; program ${elf} verify" \
    -c "reset run; shutdown"
  echo "    OK: programmed + verified + released"
}

# Stop Klipper
echo "==> Stopping Klipper..."
systemctl stop klipper || die "Failed to stop Klipper service."

# Flash MAIN
flash_one "$OPENOCD_MAIN_CONFIG" "$MAIN_ELF"

# Flash TOOLHEAD
flash_one "$OPENOCD_TOOLHEAD_CONFIG" "$MAIN_ELF"

# Start Klipper
echo "==> Starting Klipper..."
systemctl start klipper || die "Failed to start Klipper service."

echo "Firmware flashed successfully."
