#!/bin/bash
# Simple, robust RP2040 flasher via OpenOCD (CM4 SWD)
set -euo pipefail

# Paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAIN_BIN="${SCRIPT_DIR}/rp2040.bin"

OPENOCD_MAIN_CONFIG="${SCRIPT_DIR}/openocd.main.config"
OPENOCD_TOOLHEAD_CONFIG="${SCRIPT_DIR}/openocd.toolhead.config"

# Helpers
die() { echo "Error: $*" >&2; exit 1; }

flash_one() {
  local cfg="$1"
  local bin="$2"

  [[ -f "$cfg" ]] || die "OpenOCD config not found: $cfg"
  [[ -f "$bin" ]] || die "Firmware file not found: $bin"

  echo "==> Flashing with $cfg"
  # Split into two -c blocks so reset/run always executes cleanly
  openocd -f "$cfg" \
    -c "init; reset halt; program ${bin} 0x10000000 verify" \
    -c "reset run; shutdown"
  echo "    OK: programmed + verified + released"
}

# Stop Klipper
echo "==> Stopping Klipper..."
systemctl stop klipper || die "Failed to stop Klipper service."

# Flash MAIN
flash_one "$OPENOCD_MAIN_CONFIG" "$MAIN_BIN"

# Flash TOOLHEAD (optional: only if the config exists)
if [[ -f "$OPENOCD_TOOLHEAD_CONFIG" ]]; then
  flash_one "$OPENOCD_TOOLHEAD_CONFIG" "$MAIN_BIN"
else
  echo "==> Skipping toolhead (no ${OPENOCD_TOOLHEAD_CONFIG})"
fi

# Start Klipper
echo "==> Starting Klipper..."
systemctl start klipper || die "Failed to start Klipper service."

echo "Firmware flashed successfully."
