#!/usr/bin/env python3
"""List the G-code sends in MuonUI and Control checkouts (KAN-1730).

    python3 test/muon_guard/scan_senders.py ~/MuonUI ~/Muon3D_Fluidd

Prints file:line for every call that sends a script to Klipper
(printer.runGcode / sendGcode / printerGcodeScript / gcode/script). Compare
with client_senders.txt and add what is new there; the test then runs it
through the gate. This is a manual check: neither repo runs it in CI.
"""
import os
import re
import sys

PATTERN = re.compile(r'(runGcode\(|sendGcode\(|printerGcodeScript\(|'
                     r'gcode/script|printer\.gcode\.script)')
SKIP = ('node_modules', '.git', 'dist', 'vendor')


def main(roots):
    for root in roots:
        for base, dirs, files in os.walk(root):
            dirs[:] = [d for d in dirs if d not in SKIP]
            for fn in files:
                if not fn.endswith(('.ts', '.vue', '.js')):
                    continue
                if '.spec.' in fn or '.test.' in fn:
                    continue
                path = os.path.join(base, fn)
                with open(path, encoding='utf-8', errors='replace') as f:
                    for n, line in enumerate(f, 1):
                        if PATTERN.search(line):
                            print('%s:%d: %s' % (os.path.relpath(path, root),
                                                 n, line.strip()[:150]))


if __name__ == '__main__':
    main(sys.argv[1:])
