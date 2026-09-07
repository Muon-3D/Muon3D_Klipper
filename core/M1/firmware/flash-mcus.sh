#!/bin/bash
# RP2040 flasher via OpenOCD (CM4 SWD) - ELF-only.
#
# Both RP2040s run the same firmware image: 14-run.build-klipper-firmware.sh
# builds one config.rp2040 and produces one rp2040.elf, so main and toolhead
# are programmed from the same file. That is intentional, not a copy-paste
# slip -- the variable is named RP2040_ELF to stop it reading like one.
#
# KAN-118. Previously this stopped Klipper, then flashed under `set -e` with
# no trap: an unplugged toolhead or a marginal SWD line aborted the script and
# left the machine with Klipper stopped and no automatic way back. It now
# always restarts Klipper, skips a toolhead that is not attached instead of
# failing on it, and keeps the two MCUs independent so a toolhead problem
# cannot leave the main MCU half-programmed.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RP2040_ELF="${SCRIPT_DIR}/rp2040.elf"

OPENOCD_MAIN_CONFIG="${SCRIPT_DIR}/openocd.main.config"
OPENOCD_TOOLHEAD_CONFIG="${SCRIPT_DIR}/openocd.toolhead.config"

# Seconds before a wedged openocd is killed. A stuck SWD transaction otherwise
# hangs here forever with Klipper stopped.
OPENOCD_TIMEOUT="${OPENOCD_TIMEOUT:-120}"

die() { echo "Error: $*" >&2; exit 1; }

klipper_was_running=0
restore_klipper() {
  if [[ "${klipper_was_running}" == "1" ]]; then
    echo "==> Restarting Klipper..."
    systemctl start klipper || echo "Warning: failed to restart Klipper" >&2
  fi
}
# Armed before the service is stopped, so every exit path restores it.
trap restore_klipper EXIT

# Can we reach this target at all? Used to tell "toolhead not attached" apart
# from "toolhead attached and the flash failed", which need different answers.
probe_one() {
  local cfg="$1"
  timeout "${OPENOCD_TIMEOUT}" openocd -f "${cfg}" \
    -c "init; shutdown" >/dev/null 2>&1
}

flash_one() {
  local cfg="$1"
  local elf="$2"

  [[ -f "${cfg}" ]] || { echo "Error: OpenOCD config not found: ${cfg}" >&2; return 1; }
  [[ -f "${elf}" ]] || { echo "Error: firmware file not found: ${elf}" >&2; return 1; }

  echo "==> Flashing with ${cfg}"
  timeout "${OPENOCD_TIMEOUT}" openocd -f "${cfg}" \
    -c "init; reset halt; program ${elf} verify" \
    -c "reset run; shutdown" || return 1
  echo "    OK: programmed + verified + released"
}

[[ -f "${RP2040_ELF}" ]] || die "Firmware file not found: ${RP2040_ELF}"

if systemctl is-active --quiet klipper; then
  klipper_was_running=1
  echo "==> Stopping Klipper..."
  systemctl stop klipper || die "Failed to stop Klipper service."
else
  echo "==> Klipper not running; leaving it stopped."
fi

main_result="failed"
toolhead_result="failed"

# Main MCU. Always present -- a failure here is a real failure.
if flash_one "${OPENOCD_MAIN_CONFIG}" "${RP2040_ELF}"; then
  main_result="flashed"
fi

# Toolhead MCU. Hot-swappable, so absence is a normal state and not an error.
if ! probe_one "${OPENOCD_TOOLHEAD_CONFIG}"; then
  echo "==> Toolhead not responding on SWD; skipping (is it plugged in?)"
  toolhead_result="skipped"
elif flash_one "${OPENOCD_TOOLHEAD_CONFIG}" "${RP2040_ELF}"; then
  toolhead_result="flashed"
fi

# Machine-readable, for the OTA commit-verification path to consume.
echo "flash-mcus-result: main=${main_result} toolhead=${toolhead_result}"

if [[ "${main_result}" != "flashed" || "${toolhead_result}" == "failed" ]]; then
  echo "Firmware flash incomplete." >&2
  exit 1
fi

echo "Firmware flashed successfully."
