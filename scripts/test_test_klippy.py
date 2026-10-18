#!/usr/bin/env python3
"""Self-tests for the Klippy batch-test harness."""

import test_klippy


def main():
    expected = "FIRST_LAYER_NUDGE refused: expected reason"
    config_dump = "gcode = {action_raise_error(\"%s\")}" % (expected,)
    wrong_runtime = (
        "Exiting SD card print (position 42): unrelated runtime failure")
    assert not test_klippy.runtime_error_matches(
        config_dump + "\n" + wrong_runtime, expected)
    assert test_klippy.runtime_error_matches(
        config_dump + "\nExiting SD card print (position 42): " + expected,
        expected)
    print("test_klippy: runtime EXPECT_ERROR matching is sound")


if __name__ == '__main__':
    main()
