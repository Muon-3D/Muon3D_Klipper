#!/usr/bin/env python3
"""Verify first-layer nudge wiring in the shipped M1 configuration."""
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MACROS = ROOT / "core/M1/macros/macros.cfg"
PRINT = ROOT / "core/M1/macros/print.cfg"
CORE = ROOT / "core/M1/core.cfg"
NUDGE = ROOT / "core/M1/macros/first_layer_nudge.cfg"


def section(text, header):
    start = text.index("\n" + header) + 1 if not text.startswith(header) else 0
    end = text.find("\n[", start + len(header))
    return text[start:] if end < 0 else text[start:end]


def require(haystack, needle, where):
    if needle not in haystack:
        raise AssertionError(f"{where}: missing {needle!r}")


def main():
    macros = MACROS.read_text()
    printing = PRINT.read_text()
    core = CORE.read_text()
    nudge = NUDGE.read_text()

    require(macros, "[include first_layer_nudge.cfg]", MACROS)
    start = section(printing, "[gcode_macro PRINT_START]")
    # Command order matters: stale authorization goes before reset can arm.
    disarm = start.index("_FIRST_LAYER_NUDGE_DISARM")
    allow = start.index("_FIRST_LAYER_NUDGE_ALLOW_RESET")
    reset = start.index("_FIRST_LAYER_NUDGE_RESET")
    assert disarm < allow < reset
    for name in ("PRINT_END", "CANCEL_PRINT", "CANCEL_PRINT_BASE",
                 "SDCARD_RESET_FILE", "_ON_PRINT_ERROR"):
        body = section(printing, f"[gcode_macro {name}]")
        require(body, "_FIRST_LAYER_NUDGE_DISARM", f"print.cfg [{name}]")
    require(section(core, "[idle_timeout]"), "_FIRST_LAYER_NUDGE_DISARM",
            "core.cfg [idle_timeout]")
    require(nudge, "MOVE=1 MOVE_SPEED=5", NUDGE)
    require(nudge, "candidate_um|abs > 100", NUDGE)
    print("first-layer nudge production include chain and hooks are sound")


if __name__ == "__main__":
    main()
