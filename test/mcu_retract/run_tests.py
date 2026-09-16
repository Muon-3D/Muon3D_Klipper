"""Run with Python 3 and GCC; no printer or Linux Klippy runtime required."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent


def native_tests():
    cc = os.environ.get('CC', 'gcc')
    variants = {
        'scheduled': [],
        'inline': ['-DCONFIG_INLINE_STEPPER_HACK=1'],
        'edge': ['-DCONFIG_INLINE_STEPPER_HACK=1',
                 '-DCONFIG_WANT_STEPPER_OPTIMIZED_BOTH_EDGE=1'],
        'avr': ['-DCONFIG_INLINE_STEPPER_HACK=1', '-DCONFIG_MACH_AVR=1'],
    }
    with tempfile.TemporaryDirectory(prefix='klipper-retract-tests-') as out:
        for enabled in (0, 1):
            for name, flags in variants.items():
                binary = str(Path(out) / ('stepper_%s_%d.exe' % (name, enabled)))
                subprocess.run([cc, '-std=gnu11', '-O2', '-I', str(HERE/'stubs'),
                                '-DCONFIG_EXPERIMENTAL_LOAD_CELL_RETRACT=%d' % enabled,
                                *flags, str(HERE/'stepper_test.c'), '-o', binary],
                               check=True, timeout=30)
                print('Firmware variant:', name, 'enabled:', enabled, flush=True)
                subprocess.run([binary], check=True, timeout=10)
        binary = str(Path(out) / 'dispatch.exe')
        subprocess.run([cc, '-std=gnu11', '-O2', '-pthread',
                        str(HERE/'trdispatch_test.c'), '-o', binary],
                       check=True, timeout=30)
        subprocess.run([binary], check=True, timeout=10)


if __name__ == '__main__':
    native_tests()
    suite = unittest.defaultTestLoader.discover(str(HERE), pattern='test_*.py')
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    raise SystemExit(not result.wasSuccessful())
