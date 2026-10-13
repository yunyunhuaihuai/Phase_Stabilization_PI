"""Offline executable checks; CTest supplies binary and unchanged example config."""
import csv
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools import pi_experiment


class LoggingModeTests(unittest.TestCase):
    def run_cli(self, directory, arguments):
        result = subprocess.run(
            [str(BINARY), *map(str, arguments)], cwd=directory,
            env=dict(os.environ, PI_SIM_DISTURBANCE_MV="5"),
            capture_output=True, text=True, encoding="utf-8", errors="replace",
            timeout=45,
        )
        return result, result.stdout + result.stderr

    def assert_complete_run(self, result, output):
        self.assertEqual(result.returncode, 0, output)
        for marker in ("[State 0]", "[State 1/1.5]", "[Calibration]",
                       "[State 2]", "raw V=", "status=ready",
                       "reason=duration_complete", "stop_state_unknown=0"):
            self.assertIn(marker, output)
        self.assertNotIn("FATAL", output)

    def test_off_completes_startup_loop_and_exit_without_files(self):
        # Explicit config plus both options; then options alone using the
        # default ./config.txt, with the option order reversed.
        combinations = (
            [CONFIG, "--simulation-seconds", "2", "--log-mode", "off"],
            ["--log-mode", "off", "--simulation-seconds", "2"],
        )
        for arguments in combinations:
            with self.subTest(arguments=arguments), tempfile.TemporaryDirectory() as d:
                shutil.copyfile(CONFIG, Path(d) / "config.txt")
                result, output = self.run_cli(d, arguments)
                self.assert_complete_run(result, output)
                self.assertIn("[Logging] mode=off", output)
                self.assertNotIn("[DataLogger]", output)
                # Check every generated file, including diagnostics, safety and
                # metadata, rather than relying on a filename-prefix filter.
                self.assertEqual(sorted(p.name for p in Path(d).iterdir()), ["config.txt"])

    def test_explicit_full_accepts_config_between_options(self):
        with tempfile.TemporaryDirectory() as d:
            result, output = self.run_cli(d, ["--log-mode", "full", CONFIG,
                                             "--simulation-seconds", "2"])
            self.assert_complete_run(result, output)
            raw = [p for p in Path(d).glob("Data_*.csv")
                   if not p.name.endswith((".diagnostics.csv", ".safety.csv"))]
            self.assertEqual(len(raw), 1, output)
            self.assertEqual(len(list(Path(d).iterdir())), 4)
            for suffix in (".diagnostics.csv", ".safety.csv", ".metadata.txt"):
                self.assertTrue(Path(str(raw[0]) + suffix).is_file())
            with raw[0].open(encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertGreaterEqual(len(rows), 20)
            self.assertTrue(all(row["Send_accepted"] == "1" for row in rows))
            safety = Path(str(raw[0]) + ".safety.csv").read_text(encoding="utf-8")
            self.assertIn("homing_zero_start", safety)
            self.assertIn("duration_complete", safety)

    def test_existing_python_runner_keeps_default_full_and_in_loop_semantics(self):
        with tempfile.TemporaryDirectory() as d:
            manifest = pi_experiment.run(BINARY, Path(d) / "runner", seconds=2,
                                         config=CONFIG, disturbance_mv=5)
            self.assertEqual(manifest["status"], "completed")
            self.assertNotIn("--log-mode", manifest["command"])
            self.assertEqual(manifest["summary"]["data_origin"], "simulation")
            self.assertEqual(manifest["summary"]["measurement_scope"], "in_loop")
            self.assertGreaterEqual(manifest["summary"]["samples"], 20)
            self.assertEqual(manifest["summary"]["fault_rows"], 0)

    def test_invalid_arguments_fail_before_initialization_or_file_creation(self):
        cases = (
            ["--log-mode"], ["--log-mode", "invalid"],
            ["--log-mode", "off", "--log-mode", "full"],
            ["--simulation-seconds"], ["--simulation-seconds", "nan"],
            ["--simulation-seconds", "0"], ["--simulation-seconds", "86401"],
            ["--simulation-seconds", "1", "--simulation-seconds", "2"],
            [CONFIG, CONFIG], ["--unknown"],
        )
        for arguments in cases:
            with self.subTest(arguments=arguments), tempfile.TemporaryDirectory() as d:
                result, output = self.run_cli(d, arguments)
                self.assertNotEqual(result.returncode, 0)
                self.assertNotIn("[SimHAL]", output)
                self.assertEqual(list(Path(d).iterdir()), [])

    def test_help_describes_both_modes(self):
        with tempfile.TemporaryDirectory() as d:
            result, output = self.run_cli(d, ["--help"])
            self.assertEqual(result.returncode, 0)
            self.assertIn("--log-mode full|off", output)
            self.assertIn("default", output)
            self.assertEqual(list(Path(d).iterdir()), [])


if __name__ == "__main__":
    BINARY, CONFIG = (Path(value).resolve() for value in sys.argv[1:3])
    unittest.main(argv=[sys.argv[0]])
