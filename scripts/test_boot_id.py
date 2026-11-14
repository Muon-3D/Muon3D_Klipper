#!/usr/bin/env python3
# Unit tests for klippy/extras/boot_id.py (KAN-747).
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import importlib.util, os, tempfile, unittest

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    "boot_id", os.path.join(HERE, "..", "klippy", "extras", "boot_id.py"))
boot_id = importlib.util.module_from_spec(spec)
spec.loader.exec_module(boot_id)

class FakeConfig:
    def __init__(self, path):
        self.path = path
    def get(self, name, default=None):
        return self.path

class BootIdTest(unittest.TestCase):
    def _id(self, data):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "boot_id")
            if data is not None:
                with open(p, "wb") as f:
                    f.write(data)
            return boot_id.load_config(FakeConfig(p)).get_status()["id"]

    def test_reads_and_strips(self):
        self.assertEqual(self._id(b"abc-123\n"), "abc-123")

    def test_missing_file_is_unknown(self):
        self.assertEqual(self._id(None), "")

    def test_undecodable_bytes_are_unknown_not_a_crash(self):
        self.assertEqual(self._id(b"\xff\xfe\x80"), "")

if __name__ == "__main__":
    unittest.main()
