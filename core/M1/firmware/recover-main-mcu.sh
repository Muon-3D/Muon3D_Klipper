#!/bin/bash
# KAN-229: recover a wedged main RP2040 before klipper tries to talk to it.
#
# The main RP2040 sometimes comes up with core0's debug access port missing:
#
#     Error: [rp2040.core0] Could not find MEM-AP to control the core
#     Error: [rp2040.core0] Examination failed
#     Info : [rp2040.core1] Examination succeed
#
# The SWD debug port is fine - DPIDR reads, and core1 (which Klipper does not
# use) examines normally. Only core0, the one running the firmware, is gone.
# Klippy then reports "mcu 'mcu': Unable to connect" and the machine is dead.
#
# FIRMWARE_RESTART cannot fix it. Klipper's restart_method: swdio shells out to
# openocd, and openocd's stock target/rp2040.cfg says:
#
#     # srst does not exist; use SYSRESETREQ to perform a soft reset
#     $_TARGETNAME_0 cortex_m reset_config sysresetreq
#
# SYSRESETREQ is a soft reset issued THROUGH the debug AP - precisely the thing
# that is unavailable in this failure. So openocd gives up with:
#
#     Error: [rp2040.core0] Debug AP not available, reset NOT asserted!
#
# That assumption is wrong for this board: SRST is wired to the RUN pin on
# GPIO 26 and works. Asserting it makes the whole DAP disappear, and pulsing it
# brings core0 back and lets klipper connect first try.
#
# Fixing it inside openocd.main.config was tried and rejected: overriding
# init_reset to pulse SRST on every `reset run` made klippy hang in startup
# indefinitely on a HEALTHY board, because Klipper's own reset path is not
# expecting the debug port to vanish underneath it. So the pulse lives here,
# out of band, and only runs when the board is actually wedged.
#
# Exits 0 unconditionally. This must never be the reason klipper fails to
# start - if recovery does not work, klipper should still get its own attempt
# and report the real fault.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG="${OPENOCD_MAIN_CONFIG:-${SCRIPT_DIR}/openocd.main.config}"
TAG="recover-main-mcu"

log() { echo "$*"; command -v logger >/dev/null 2>&1 && logger -t "$TAG" "$*"; }

if [[ ! -f "$CFG" ]]; then
    log "no openocd config at $CFG, nothing to do"
    exit 0
fi
if ! command -v openocd >/dev/null 2>&1; then
    log "openocd not installed, nothing to do"
    exit 0
fi

# Probe. `init` tolerates a failed core0 examination and still brings up core1,
# so this reports the fault rather than dying on it.
probe="$(timeout 30 openocd -f "$CFG" -c 'init; exit' 2>&1)"

if ! grep -q 'core0.*Examination failed' <<<"$probe"; then
    log "main MCU core0 examined OK, no recovery needed"
    exit 0
fi

log "main MCU core0 examination FAILED - pulsing SRST to recover"

# srst_nogate: openocd must not assume it can keep debugging while reset is
# held. It cannot - the RUN pin takes the whole chip down, DAP included - but
# the flag stops openocd aborting when the port disappears mid-pulse.
recover="$(timeout 45 openocd -f "$CFG" \
    -c 'reset_config srst_only srst_nogate' \
    -c 'init' \
    -c 'adapter assert srst' \
    -c 'sleep 300' \
    -c 'adapter deassert srst' \
    -c 'sleep 300' \
    -c 'exit' 2>&1)"

if grep -q 'core0.*Examination succeed' <<<"$recover"; then
    log "core0 recovered"
else
    log "core0 did NOT recover; klipper will report the fault"
    printf '%s\n' "$recover" | tail -20
fi

exit 0
