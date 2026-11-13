#!/usr/bin/env python3
"""Batch-mode and mutation check of the KAN-1361 tap confirmation.

    python3 test/tap_path/tap_confirm_mutation.py -d <dict dir>

Runs test/klippy/adxl345_probe_stall*.test in a copy of the tree and
checks from the klippy log that each one ends for the right reason (a
SHOULD_FAIL test passes on any error). Then breaks the fix one way at a
time and checks that a named test notices. <dict dir> holds
linuxprocess.dict, as for scripts/test_klippy.py.
"""
import argparse
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
PROBE = os.path.join('klippy', 'extras', 'adxl345_probe.py')
CONFIRM = os.path.join('klippy', 'extras', 'tap_confirm.py')

# test -> (passes, text the log must hold)
EXPECT = {
    'adxl345_probe_stall': (
        True, "y home confirmed with 2 agreeing taps after 4 retries"
              " (4 driven into the stop)"),
    'adxl345_probe_stall_ratchet': (
        False, "y home taps disagree by 0.800mm (tolerance 0.200mm) after"
               " 8 retries; 9 taps were driven into the stop"),
    'adxl345_probe_stall_other': (
        False, "y home taps disagree by 0.500mm (tolerance 0.200mm) after"
               " 2 retries; taps:"),
    'adxl345_probe_stall_far': (
        False, "y home taps disagree by 1.200mm (tolerance 0.200mm) after"
               " 2 retries; taps:"),
}

# name, file, old, new, the test that must notice
MUTATIONS = [
    ("capped no-trigger resumes the seek", PROBE,
     "                if cfg['overtravel'] is not None:\n",
     "                if False:\n",
     'unit_overtravel'),
    ("DRV_STATUS read exception propagates",
     os.path.join('klippy', 'extras', 'y_tap_log.py'),
     "        return 'unavailable(%s)' % str(e)\n",
     "        raise\n",
     'unit_y_log'),
    ("stream timeout warns instead of aborting", PROBE,
     "                raise self.printer.command_error("
     "self._stream_stop_error)\n",
     "                logging.warning(self._stream_stop_error)\n"
     "                break\n",
     'unit_failsafe'),
    ("acceptance compares to predecessor instead of anchor", PROBE,
     "            delta, kind = compare(accepted[0], trig, info)\n",
     "            delta, kind = compare(accepted[-1], trig, info)\n",
     'unit_failsafe'),
    ("no stall retries (the code before the fix)", CONFIRM,
     "        if kind == STALL and self.stall_retries > 0:\n",
     "        if False:\n",
     'adxl345_probe_stall'),
    ("a stall is accepted (agree modulo one cycle)", CONFIRM,
     "    if stall_dist and tolerance < delta <= stall_dist + tolerance:\n"
     "        return STALL\n",
     "    if stall_dist and tolerance < delta <= stall_dist + tolerance:\n"
     "        return AGREE\n",
     'adxl345_probe_stall_ratchet'),
    ("the direction of the difference is ignored", CONFIRM,
     "    if stall_dist and tolerance < delta <= stall_dist + tolerance:\n",
     "    if stall_dist and tolerance < abs(delta) <= stall_dist"
     " + tolerance:\n",
     'adxl345_probe_stall_other'),
    ("any distance further on counts as a stall", CONFIRM,
     "    if stall_dist and tolerance < delta <= stall_dist + tolerance:\n",
     "    if stall_dist and tolerance < delta:\n",
     'adxl345_probe_stall_far'),
    ("the loop keeps the tap before a stall", PROBE,
     "                         \" retrying confirmation\", axis, abs(delta),"
     " kind)\n            accepted = [trig]\n",
     "                         \" retrying confirmation\", axis, abs(delta),"
     " kind)\n",
     'adxl345_probe_stall'),
]


def make_tree(tmp):
    shutil.copytree(os.path.join(ROOT, 'klippy'), os.path.join(tmp, 'klippy'),
                    ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    os.makedirs(os.path.join(tmp, 'scripts'))
    shutil.copy(os.path.join(ROOT, 'scripts', 'test_klippy.py'),
                os.path.join(tmp, 'scripts'))
    dst = os.path.join(tmp, 'test', 'klippy')
    os.makedirs(dst)
    src = os.path.join(ROOT, 'test', 'klippy')
    for fname in os.listdir(src):
        if fname.startswith('adxl345_probe_stall'):
            shutil.copy(os.path.join(src, fname), dst)
    shutil.copytree(os.path.join(ROOT, 'test', 'tap_path'),
                    os.path.join(tmp, 'test', 'tap_path'),
                    ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    shutil.copy(os.path.join(ROOT, 'scripts', 'y_tap_log_summary.py'),
                os.path.join(tmp, 'scripts'))


def run_test(tmp, dictdir, name):
    """(passed, log text) for one .test, judged like scripts/test_klippy.py
    but with the log kept."""
    res = subprocess.run(
        [sys.executable, 'scripts/test_klippy.py', '-v', '-d', dictdir,
         os.path.join('test', 'klippy', name + '.test')],
        cwd=tmp, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return res.returncode == 0, res.stdout.decode('utf-8', 'replace')


def check(tmp, dictdir, name):
    """None if the test ends as EXPECT says, else why not."""
    if name in ('unit_failsafe', 'unit_y_log', 'unit_overtravel'):
        test = ('test_tap_failsafe.py' if name == 'unit_failsafe'
                else 'test_y_tap_log.py' if name == 'unit_y_log'
                else 'test_tap_overtravel.py')
        result = subprocess.run(
            [sys.executable, os.path.join('test', 'tap_path', test)], cwd=tmp,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        if result.returncode:
            lines = result.stdout.decode('utf-8', 'replace').strip().split('\n')
            return lines[-1]
        return None
    passes, text = EXPECT[name]
    ok, log = run_test(tmp, dictdir, name)
    log = log.replace('\r\n', '\n')
    if not ok:
        return "test_klippy.py failed"
    if text not in log:
        return "log lacks %r" % (text,)
    if passes and "taps disagree by" in log.split(text)[-1]:
        return "a later home failed"
    return None


def mutate(tmp, fname, old, new):
    path = os.path.join(tmp, fname)
    with open(path) as fh:
        src = fh.read()
    if src.count(old) != 1:
        raise SystemExit("mutation target not found once in %s:\n%s"
                         % (fname, old))
    with open(path, 'w') as fh:
        fh.write(src.replace(old, new))
    return src


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('-d', '--dictdir', required=True)
    args = ap.parse_args(argv)
    dictdir = os.path.abspath(args.dictdir)
    failed = 0
    tmp = tempfile.mkdtemp(prefix='tap_confirm_mutation_')
    try:
        make_tree(tmp)
        for name in sorted(EXPECT):
            why = check(tmp, dictdir, name)
            print('%s %s%s' % ('FAIL' if why else 'PASS', name,
                               ': ' + why if why else ''))
            failed += bool(why)
        if failed:
            print("the unmutated tree fails; mutations not run")
            return 1
        why = check(tmp, dictdir, 'unit_failsafe')
        if why:
            print('FAIL unmutated failsafe: ' + why)
            return 1
        why = check(tmp, dictdir, 'unit_y_log')
        if why:
            print('FAIL unmutated Y log: ' + why)
            return 1
        why = check(tmp, dictdir, 'unit_overtravel')
        if why:
            print('FAIL unmutated overtravel: ' + why)
            return 1
        for title, fname, old, new, name in MUTATIONS:
            src = mutate(tmp, fname, old, new)
            try:
                why = check(tmp, dictdir, name)
            finally:
                with open(os.path.join(tmp, fname), 'w') as fh:
                    fh.write(src)
            if why is None:
                failed += 1
                print('FAIL mutation survived: %s (%s still passes)'
                      % (title, name))
            else:
                print('PASS mutation killed: %s (%s: %s)'
                      % (title, name, why[:60]))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
