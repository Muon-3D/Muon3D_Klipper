# Locate the klippy sources under test, and the pre-change ("stock") version
# of a file for the tests that assert the default behaviour is unchanged.
import os
import subprocess
import sys
import tempfile

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


def patched(relpath):
    return os.path.join(ROOT, relpath)


def stock(relpath):
    """The file as it is on the merge base with master (falls back to
    origin/master, then master).  Written to a temp file; path returned."""
    for base in ('merge-base', 'origin/master', 'master'):
        try:
            if base == 'merge-base':
                ref = subprocess.check_output(
                    ['git', '-C', ROOT, 'merge-base', 'HEAD', 'origin/master'],
                    stderr=subprocess.DEVNULL).decode().strip()
            else:
                ref = base
            blob = subprocess.check_output(
                ['git', '-C', ROOT, 'show', '%s:%s' % (ref, relpath)],
                stderr=subprocess.DEVNULL)
        except (subprocess.CalledProcessError, OSError):
            continue
        fd, path = tempfile.mkstemp(suffix='_' + os.path.basename(relpath))
        with os.fdopen(fd, 'wb') as f:
            f.write(blob)
        return path
    sys.exit('cannot find a stock %s: need origin/master or master' % relpath)
