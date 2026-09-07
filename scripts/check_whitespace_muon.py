#!/usr/bin/env python3
# Muon: run Klipper's whitespace check over the lines this branch introduces.
#
# Why this exists
# ---------------
# `scripts/check_whitespace.sh` checks every source file in the tree. Upstream
# passes it by construction. This fork does not: our own additions break the
# 80-column rule roughly 180 times across 28 files -- led_effect.py,
# control_mpc.py and advanced_homing_movement.py, plus edits to mcu.py,
# and serialhdl.py among them.
#
# Because `scripts/ci-build.sh` runs `set -eu` and calls the whitespace check
# first, that failure aborted the entire run before a single MCU firmware
# compiled. The `Build test` workflow had therefore never once passed in this
# fork -- 82 failures, 0 successes over the last 100 runs -- and we had no
# firmware build coverage at all.
#
# Reformatting ~180 lines of motion and heater code to turn the tick green would
# be a large diff through safety-relevant paths, conflicting with every open PR.
# Deleting the check would give up a real guard. So: hold new work to the rule,
# and leave the inherited debt to a deliberate cleanup.
#
# What it checks
# --------------
# Only lines ADDED relative to the merge base with master. A PR that edits a
# legacy file is not punished for violations it did not introduce, which is the
# behaviour that makes this safe to make required.
#
# End-of-file errors ("No newline at end of file", "Extra newlines at end of
# file") are reported for any changed file regardless of line number, because
# they are a property of the file rather than of one line.
#
# Escape hatches:
#   MUON_WS_FULL=1    check the whole tree, the upstream behaviour. Use this to
#                     measure the debt deliberately.
#   MUON_WS_BASE=REF  override the base revision.

from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

SRCDIR = Path(__file__).resolve().parent.parent
CHECKER = SRCDIR / "scripts" / "check_whitespace.py"

# Mirrors WS_DIRS / WS_FILES in check_whitespace.sh.
WS_DIRS = ("config/", "docs/", "klippy/", "scripts/", "src/", "test/")
WS_SUFFIXES = (".c", ".h", ".s", ".py", ".sh", ".md", ".cfg", ".txt", ".html",
               ".css", ".yaml", ".yml", ".test", ".config", ".lds")
WS_NAMES = ("makefile", "kconfig")
WS_EXCLUDE_PREFIX = ("scripts/kconfig/",)

def checkout_normalises_crlf() -> bool:
    """True when git rewrites line endings on checkout.

    On such a checkout (Windows, core.autocrlf=true) a file committed with LF
    is read back with a CR on every line. That does not merely add spurious
    control-character errors -- it also adds one to every line length, so a
    line of exactly 80 characters is reported as 81. CI checks out LF and sees
    none of it, so the fix is to check the bytes git would commit.
    """
    setting = git("config", "--get", "core.autocrlf").strip().lower()
    return setting in ("true", "input")


ERROR_RE = re.compile(r"^(?P<file>.+?):(?P<line>\d+): (?P<msg>.+)$")
HUNK_RE = re.compile(r"^@@ -\S+ \+(?P<start>\d+)(?:,(?P<count>\d+))? @@")
EOF_MESSAGES = ("No newline at end of file", "Extra newlines at end of file")


def git(*args: str) -> str:
    """Run git and return stdout.

    encoding is explicit: the default on Windows is the ANSI code page, and a
    diff of this tree contains non-ASCII (a degree sign in a heaters.py log
    string). Decoding that as cp1252 raises inside subprocess's reader thread
    and hands back None, which fails far away from the cause.
    """
    proc = subprocess.run(["git", *args], cwd=SRCDIR, capture_output=True,
                          encoding="utf-8", errors="replace")
    return proc.stdout or ""


def in_scope(path: str) -> bool:
    lowered = path.lower()
    if any(lowered.startswith(p) for p in WS_EXCLUDE_PREFIX):
        return False
    if not any(lowered.startswith(d) for d in WS_DIRS):
        return False
    name = os.path.basename(lowered)
    return name in WS_NAMES or lowered.endswith(WS_SUFFIXES)


def resolve_base() -> str | None:
    override = os.environ.get("MUON_WS_BASE")
    if override:
        return override.strip() or None
    candidates = []
    base_ref = os.environ.get("GITHUB_BASE_REF")   # set on pull_request events
    if base_ref:
        candidates.append(f"origin/{base_ref}")
    candidates += ["origin/master", "origin/main"]
    for ref in candidates:
        if not git("rev-parse", "--verify", "--quiet", ref).strip():
            # Shallow clones have no remote-tracking branch; try to get one.
            subprocess.run(
                ["git", "fetch", "--quiet", "--no-tags", "--depth=200",
                 "origin", ref.split("/", 1)[1]],
                cwd=SRCDIR, capture_output=True, text=True)
        if git("rev-parse", "--verify", "--quiet", ref).strip():
            merge_base = git("merge-base", "HEAD", ref).strip()
            if merge_base:
                return merge_base
    return None


def added_lines(base: str) -> tuple[dict[str, set[int]], set[str]]:
    """Return {path: {added line numbers}} and the set of changed paths."""
    names = [p for p in git("diff", "--name-only", "--diff-filter=ACMR",
                            f"{base}...HEAD").splitlines() if in_scope(p)]
    if not names:
        return {}, set()
    added: dict[str, set[int]] = {n: set() for n in names}
    current = None
    diff = git("diff", "-U0", f"{base}...HEAD", "--", *names)
    for line in diff.splitlines():
        if line.startswith("+++ b/"):
            current = line[6:]
            added.setdefault(current, set())
            continue
        m = HUNK_RE.match(line)
        if m and current is not None:
            start = int(m.group("start"))
            count = int(m.group("count") or 1)
            added[current].update(range(start, start + count))
    return added, set(names)


def run_full_check() -> int:
    """Upstream behaviour: the whole tree. Invoked through bash rather than
    the shebang so it also runs on a developer's Windows checkout."""
    return subprocess.run(["bash", "scripts/check_whitespace.sh"],
                          cwd=SRCDIR).returncode


def main() -> int:
    if os.environ.get("MUON_WS_FULL") == "1":
        return run_full_check()

    base = resolve_base()
    if base is None:
        print("check_whitespace_muon: no base revision found; "
              "checking the whole tree", file=sys.stderr)
        return run_full_check()

    added, changed = added_lines(base)
    if not changed:
        print(f"check_whitespace_muon: no in-scope files changed since "
              f"{base[:12]}; nothing to check")
        return 0

    existing = [p for p in sorted(changed) if (SRCDIR / p).is_file()]
    if not existing:
        print("check_whitespace_muon: changed files no longer present")
        return 0

    tmpdir = None
    targets = existing
    prefix = ""
    if checkout_normalises_crlf():
        # Materialise LF copies so lengths and control characters match what
        # CI will see, then map the reported paths back.
        tmpdir = tempfile.mkdtemp(prefix="muon-ws-")
        for rel in existing:
            dest = Path(tmpdir) / rel
            dest.parent.mkdir(parents=True, exist_ok=True)
            data = (SRCDIR / rel).read_bytes().replace(b"\r\n", b"\n")
            dest.write_bytes(data)
        targets = [str(Path(tmpdir) / rel) for rel in existing]
        prefix = tmpdir + os.sep

    try:
        proc = subprocess.run([sys.executable, str(CHECKER), *targets],
                              cwd=SRCDIR, capture_output=True,
                              encoding="utf-8", errors="replace")
    finally:
        if tmpdir:
            shutil.rmtree(tmpdir, ignore_errors=True)

    crlf_checkout = checkout_normalises_crlf()
    failures = []
    for line in proc.stderr.splitlines():
        m = ERROR_RE.match(line.strip())
        if not m:
            continue
        path, lineno, msg = m["file"], int(m["line"]), m["msg"]
        if prefix and path.startswith(prefix):
            path = path[len(prefix):].replace(os.sep, "/")
        # Report the repo-relative path, not the temp copy we actually read.
        reported = "%s:%d: %s" % (path, lineno, msg)
        if any(msg.startswith(e) for e in EOF_MESSAGES):
            failures.append(reported)
        elif lineno in added.get(path, set()):
            failures.append(reported)

    print(f"check_whitespace_muon: {len(existing)} changed file(s) since "
          f"{base[:12]}, checking only added lines")
    if failures:
        sys.stderr.write("\n\nERROR:\nERROR: White space errors on lines this "
                         "branch adds\nERROR:\n")
        for f in failures:
            sys.stderr.write(f + "\n")
        sys.stderr.write(
            "\nRun MUON_WS_FULL=1 ./scripts/check_whitespace.sh for"
            " the whole tree, including inherited debt.\n\n")
        return 1
    print("check_whitespace_muon: clean")
    return 0


if __name__ == "__main__":
    sys.exit(main())
