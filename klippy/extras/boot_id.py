# Publish the Linux boot id, so a macro can tell "same power-on" from "new boot"
# across a klippy restart (KAN-747).
#
# Copyright (C) 2026  Muon 3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.

BOOT_ID_PATH = "/proc/sys/kernel/random/boot_id"

class BootId:
    def __init__(self, config):
        path = config.get("path", BOOT_ID_PATH)
        try:
            with open(path, encoding="ascii") as f:
                self.boot_id = f.read().strip()
        except (OSError, ValueError):
            # Unreadable or non-ASCII (UnicodeDecodeError is a ValueError): unknown.
            # Never equal to a saved id: callers must treat "" as
            # "cannot prove the same boot" and fall back to the safe path.
            self.boot_id = ""

    def get_status(self, eventtime=None):
        return {"id": self.boot_id}

def load_config(config):
    return BootId(config)
