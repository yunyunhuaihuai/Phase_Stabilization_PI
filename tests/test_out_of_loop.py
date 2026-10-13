"""Offline orchestration tests: no serial, SSH, VISA backend, or hardware access."""
import csv
import hashlib
import io
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest import mock

from tools import out_of_loop as runner
from tools import site_settings
from tools.board_session import download_artifacts, recv_exit_status
from tools.experiment_plan import full_plan, repeated_runs, verify_config
from tools.scope_86100c import Scope86100C, ScopeCollector
from tools.import_scope_native import archive as archive_scope_native


def parse_test_args(argv):
    extra=[]
    if "--execute" in argv:
        extra=["--source-address","SIM_SOURCE","--serial-port","TEST_CONSOLE","--hz-per-mv","1"]
    return runner.parse_args(extra+argv)


class FakeChannel:
    def __init__(self, stopped, exit_code=0):
        self.stopped, self.exit_code = stopped, exit_code

    def exit_status_ready(self):
        return self.stopped.is_set()

    def recv_exit_status(self):
        return self.exit_code


class FakeOutput:
    def __init__(self, board):
        self.board = board
        self.channel = FakeChannel(board.stopped, board.exit_code)

    def __iter__(self):
        yield "[DataLogger] 日志文件: Data_4_PI_this_run.csv\n"
        yield ">>> [State 2] control\n"
        self.board.stopped.wait(2)


class FakeBoard:
    def __init__(self, run, exit_code=0, bad_metadata=False):
        self.run_data, self.exit_code, self.bad_metadata = run, exit_code, bad_metadata
        self.stopped, self.closed = threading.Event(), False
        self.config = b""
        self.commands = []

    def run(self, command, **kwargs):
        self.commands.append(command)
        if "kill -INT" in command:
            self.stopped.set()
            return ""
        if command.startswith("cat "):
            return self.config.decode()
        if command.startswith("readlink "):
            return self.run_data["remote"]["binary"]
        if command.startswith("sha256sum "):
            return (hashlib.sha256(self.config).hexdigest() if "config.txt" in command else "a" * 64) + " path"
        if "rev-parse HEAD" in command:
            return "b" * 40
        return ""

    def put_file(self, local, remote):
        self.config = Path(local).read_bytes()

    def exec_long(self, command):
        self.launch = command
        return FakeOutput(self)

    def get_file(self, remote, local):
        if remote.endswith(".metadata.txt"):
            config = "previous_run" if self.bad_metadata else self.run_data["remote"]["config"]
            text = f"CONFIG_PATH={config}\nCALIBRATED_B0=.0025\nEFFECTIVE_LOGICAL_LIMIT=5500\nMAX_PHYSICAL_DELTA=5500\n"
        elif remote.endswith(".safety.csv"):
            text = "Event,Final_stop_state_UNKNOWN,Final_hardware_intervention_required\nshutdown,0,0\n"
        elif remote.endswith(".diagnostics.csv"):
            text = "Cycle,Sample_dt_s,Previous_cycle_total_us,Deadline_late,Send_result,Controller_status,Physical_delta_steps\n"
            text += "".join(f"{i},.05,50000,0,0,ready,5\n" for i in range(1, 11))
        else:
            text = "Elapsed_s,V_PHS_mV,State\n" + "".join(f"{i*.05},1000000,Ready\n" for i in range(1, 11))
        Path(local).write_text(text, encoding="utf-8")

    def close(self):
        self.closed = True


class FakeSource:
    def __init__(self, fail=False, wrong_model=False):
        self.commands, self.closed, self.fail = [], False, fail
        self.wrong_model = wrong_model

    def query(self, command):
        self.commands.append(command)
        return "Keysight,OTHER,SN,FW" if self.wrong_model else "Agilent Technologies,E8257D,SN,FW" if command == "*IDN?" else "+12.700"

    def write(self, command):
        self.commands.append(command)
        if self.fail and command != ":FREQ 2000000000Hz":
            if self.fail == "interrupt":
                raise KeyboardInterrupt()
            raise RuntimeError("simulated source disconnect")

    def close(self):
        self.closed = True


class FakeVisaScope:
    def __init__(self, latency=.01):
        self.latency, self.commands, self.closed = latency, [], False
        self.sample_count = 0

    def query(self, command):
        self.commands.append(command)
        if command == "*IDN?":
            return "Agilent Technologies,86100C,SN,FW"
        if command == ":MEASure:SENDvalid?":
            return "0"
        time.sleep(self.latency)
        self.sample_count += 1
        return ("\xb4Time(m1)-(2),1.2E-12,0,1E-12,2E-12,1.2E-12,0,"
                f"{self.sample_count}")

    def write(self, command):
        self.commands.append(command)

    def close(self):
        self.closed = True


class Manager:
    def __init__(self, resource):
        self.resource = resource

    def open_resource(self, address):
        return self.resource


def short_plan():
    return [dict(planned_s=0., event="MODULE_1_STEP_BASELINE_10s", module="step",
                 freq_hz=2e9, equiv_mv=0, apply_frequency=False),
            dict(planned_s=.1, event="STEP_+50.0mV", module="step", freq_hz=2e9+1350.5,
                 equiv_mv=50, apply_frequency=True),
            dict(planned_s=.15, event="STEP_RECOVERY", module="step", freq_hz=2e9,
                 equiv_mv=0, apply_frequency=True),
            dict(planned_s=.2, event="POST_WAIT_DONE", module=None, freq_hz=2e9,
                 equiv_mv=0, apply_frequency=False)]


def read_events(run):
    with open(run["events_csv"], encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


class OutOfLoopTests(unittest.TestCase):
    def test_frozen_scope_result_table_is_not_eligible_for_analysis(self):
        args = parse_test_args(["--dry-run", "--scope-capture", "lan"])
        run = {"status": "completed", "controller_exit_status": 0,
               "downloaded_artifacts": {"complete": True}, "cleanup_errors": [],
               "baseline": {"baseline_ps": 20.0},
               "scope_summary": {"query_count": 100,
                                 "observed_result_value_change_count": 0,
                                 "measurement_results_n_samples_observation":
                                     {"total_positive_change": 0}}}
        with mock.patch.object(runner, "evidence_health", return_value=([], {})):
            health = runner._health(run, [], args)
        self.assertFalse(health["eligible_for_analysis"])
        self.assertIn("scope_result_table_not_updating", health["reasons"])


    def make_run(self, root):
        args = parse_test_args(["--execute", "--scope-capture", "lan", "--scope-address", "SIM::SCOPE",
                                  "--output-dir", root, "--batch-id", "offline", "--repeats", "1"])
        batch = runner.create_batch(args)
        return args, batch, batch["runs"][0]

    def execute(self, args, run, *, fail=False, latency=.01, exit_code=0, wrong_model=False, bad_metadata=False):
        board = FakeBoard(run, exit_code, bad_metadata)
        source = FakeSource(fail, wrong_model)
        resource = FakeVisaScope(latency)
        code = runner.execute_run(run, args, plan=short_plan(),
            board_factory=lambda args: board,
            source_factory=lambda *args: source,
            scope_factory=lambda address, timeout_ms: Scope86100C.connect(address, timeout_ms, Manager(resource)),
            collector_factory=ScopeCollector)
        return code, board, source, resource

    def test_pi_schedule_and_repetitions(self):
        step = next(e for e in full_plan() if e["event"]=="STEP_+50.0mV")
        self.assertEqual(step["planned_s"],10)
        self.assertEqual(step["freq_hz"],2000000050)
        self.assertEqual(full_plan(hz_per_mv=2)[1]["freq_hz"],2000000100)
        self.assertEqual(len(full_plan()), 122)
        self.assertEqual(full_plan()[-1]["planned_s"], 165.5)
        runs = repeated_runs(3)
        self.assertEqual(runs, repeated_runs(3))
        for block in range(1, 4):
            self.assertEqual({r["condition"] for r in runs if r["block"] == block}, {"PI"})

    def test_dry_run_never_imports_live_dependencies(self):
        with tempfile.TemporaryDirectory() as root:
            forbidden = {"paramiko": None, "pyvisa": None, "serial": None}
            with mock.patch.dict("sys.modules", forbidden), mock.patch("sys.stdout", new_callable=io.StringIO):
                self.assertEqual(runner.main(["--dry-run", "--output-dir", root, "--batch-id", "dry"]), 0)
            manifest = json.loads((Path(root) / "dry" / "batch_manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(manifest["conn"], "serial")
            self.assertEqual(manifest["scope_capture"], "local")
            self.assertIsNone(manifest["scope_address"])
            self.assertEqual(manifest["maintenance_serial_port"], site_settings.SERIAL_PORT)
            self.assertEqual(len(manifest["runs"]), 3)
            for run in manifest["runs"]:
                values = verify_config(Path(run["config_path"]).read_text(), run["algo"])
                self.assertFalse(Path(run["events_csv"]).exists())

    def test_local_capture_never_opens_scope_and_keeps_run_pending(self):
        with tempfile.TemporaryDirectory() as root:
            args = parse_test_args(["--execute", "--scope-local-ready", "--output-dir", root,
                                      "--batch-id", "local", "--repeats", "1"])
            run = runner.create_batch(args)["runs"][0]
            board, source = FakeBoard(run), FakeSource()
            code = runner.execute_run(run, args, plan=short_plan(),
                board_factory=lambda _args: board,
                source_factory=lambda *_args: source,
                scope_factory=lambda *_args, **_kwargs: self.fail("scope connection opened"),
                collector_factory=lambda *_args, **_kwargs: self.fail("scope collector started"))
            self.assertEqual(code, 0)
            self.assertEqual(run["status"], "completed_pending_scope")
            self.assertFalse(run["health"]["eligible_for_analysis"])
            self.assertFalse(Path(run["scope_csv"]).exists())
            self.assertTrue(Path(run["scope_local_export_dir"]).is_dir())
            self.assertTrue(board.closed and source.closed)
            self.assertEqual(source.commands[-1], ":FREQ 2000000000Hz")
            self.assertIn("SCOPE_LOCAL_RECORDING_OPERATOR_CONFIRMED", [row["event"] for row in read_events(run)])

    def test_local_execute_requires_explicit_native_recording_confirmation(self):
        with self.assertRaisesRegex(ValueError, "scope-local-ready"):
            parse_test_args(["--execute"])

    def test_native_file_import_is_hash_checked_and_never_claims_timing(self):
        from tools.analyze_out_of_loop import analyze_batch
        with tempfile.TemporaryDirectory() as root:
            args = parse_test_args(["--dry-run", "--output-dir", root,
                                      "--batch-id", "native", "--repeats", "1"])
            batch = runner.create_batch(args)
            source = Path(root) / "sample.sum"
            source.write_bytes(b"original local summary, unknown format\r\n")
            record, = archive_scope_native(batch["batch_dir"], batch["runs"][0]["run_id"], [source])
            self.assertEqual(Path(record["file"]).read_bytes(), source.read_bytes())
            with self.assertRaises(FileExistsError):
                archive_scope_native(batch["batch_dir"], batch["runs"][0]["run_id"], [source])
            self.assertEqual(analyze_batch(batch["batch_dir"]), [])
            report = json.loads((Path(batch["runs"][0]["run_dir"]) / "analysis_diagnostics.json").read_text())
            self.assertIn("scope_native_timing_and_format_unverified", report["reasons"])
            self.assertEqual(report["native_file_count"], 1)

    def test_parallel_live_lifecycle_records_and_preserves_evidence(self):
        with tempfile.TemporaryDirectory() as root:
            args, batch, run = self.make_run(root)
            code, board, source, scope = self.execute(args, run)
            self.assertEqual(code, 0, run.get("health"))
            self.assertTrue(board.stopped.is_set())
            self.assertTrue(board.closed and source.closed and scope.closed)
            self.assertEqual(source.commands[-1], ":FREQ 2000000000Hz")
            self.assertFalse(any(cmd.startswith(":POWer ") for cmd in source.commands))
            self.assertTrue(run["downloaded_artifacts"]["complete"])
            rows = read_events(run)
            commands = [row for row in rows if row["event"] == "STEP_+50.0mV"]
            self.assertEqual([row["command_stage"] for row in commands], ["before", "after"])
            self.assertLessEqual(float(commands[0]["monotonic_s"]), float(commands[1]["command_end_monotonic_s"]))
            self.assertTrue(run["baseline"]["sample_count"] > 0)
            self.assertFalse(run["scope_summary"]["new_data_20hz_verified"])
            self.assertTrue(run["scope_summary"]["scope"]["closed"])

    def test_slow_scope_does_not_delay_source(self):
        with tempfile.TemporaryDirectory() as root:
            args, _, run = self.make_run(root)
            code, _, _, _ = self.execute(args, run, latency=.3)
            rows = read_events(run)
            step = next(row for row in rows if row["event"] == "STEP_+50.0mV" and row["command_stage"] == "before")
            self.assertLess(float(step["state2_elapsed_s"]), .22)
            self.assertIn("no_valid_pre_disturbance_scope_baseline", run["health"]["reasons"])
            self.assertTrue(Path(run["scope_csv"]).exists())

    def test_scope_stop_timeout_defers_close_and_saves_manifest(self):
        with tempfile.TemporaryDirectory() as root:
            args, _, run = self.make_run(root)
            args.scope_timeout_ms = 100
            board, source = FakeBoard(run), FakeSource()

            class BusyScope(FakeVisaScope):
                active = False

                def query(self, command):
                    self.active = command.startswith(":MEASure:DELT")
                    try:
                        return super().query(command)
                    finally:
                        self.active = False

                def write(self, command):
                    if self.active:
                        raise AssertionError("concurrent scope write while query active")
                    return super().write(command)

                def close(self):
                    if self.active:
                        raise AssertionError("concurrent scope close while query active")
                    return super().close()

            resource = BusyScope(1.8)
            code = runner.execute_run(run, args, plan=short_plan(), board_factory=lambda args: board,
                source_factory=lambda *args: source,
                scope_factory=lambda address, timeout_ms: Scope86100C.connect(address, timeout_ms, Manager(resource)),
                collector_factory=ScopeCollector)
            self.assertEqual(code, 1)
            self.assertTrue(run["scope_summary"]["scope_close_pending"])
            self.assertTrue(Path(run["run_manifest_path"]).exists())
            deadline = time.monotonic() + 2
            while not resource.closed and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertTrue(resource.closed)

    def test_failure_keeps_partial_and_restores_source(self):
        with tempfile.TemporaryDirectory() as root:
            args, _, run = self.make_run(root)
            code, board, source, scope = self.execute(args, run, fail=True)
            self.assertEqual(code, 1)
            self.assertTrue(board.stopped.is_set())
            self.assertEqual(source.commands[-1], ":FREQ 2000000000Hz")
            self.assertIn("simulated source disconnect", run["error"])
            self.assertTrue(run["downloaded_artifacts"]["files"])
            rows = read_events(run)
            self.assertTrue(any(row["command_stage"] == "error" for row in rows))
            self.assertTrue(Path(run["scope_csv"]).stat().st_size > 0)

    def test_keyboard_interrupt_preserves_evidence_and_controlled_stop(self):
        with tempfile.TemporaryDirectory() as root:
            args, _, run = self.make_run(root)
            code, board, source, _ = self.execute(args, run, fail="interrupt")
            self.assertEqual(code, 130)
            self.assertEqual(run["status"], "interrupted")
            self.assertTrue(board.stopped.is_set())
            self.assertEqual(source.commands[-1], ":FREQ 2000000000Hz")
            self.assertTrue(run["downloaded_artifacts"]["files"])

    def test_launch_response_failure_still_stops_owned_pid(self):
        with tempfile.TemporaryDirectory() as root:
            args, _, run = self.make_run(root)
            board, source, resource = FakeBoard(run), FakeSource(), FakeVisaScope()
            with mock.patch.object(board, "exec_long", side_effect=RuntimeError("launch response lost")):
                code = runner.execute_run(run, args, plan=short_plan(), board_factory=lambda args: board,
                    source_factory=lambda *args: source,
                    scope_factory=lambda address, timeout_ms: Scope86100C.connect(address, timeout_ms, Manager(resource)),
                    collector_factory=ScopeCollector)
            self.assertEqual(code, 1)
            self.assertTrue(board.stopped.is_set())
            self.assertTrue(run["launch_requested"])
            self.assertIsNone(run["controller_exit_status"])
            self.assertIn("launch response lost", run["error"])

    def test_nonzero_status_and_wrong_run_metadata_are_not_success(self):
        for option in ({"exit_code": 3}, {"bad_metadata": True}):
            with self.subTest(option=option), tempfile.TemporaryDirectory() as root:
                args, _, run = self.make_run(root)
                code, *_ = self.execute(args, run, **option)
                self.assertEqual(code, 1)
                self.assertFalse(run["health"]["eligible_for_analysis"])
                self.assertTrue(run["downloaded_artifacts"]["files"])

    def test_wrong_source_identity_never_mutates_it(self):
        with tempfile.TemporaryDirectory() as root:
            args, _, run = self.make_run(root)
            code, board, source, _ = self.execute(args, run, wrong_model=True)
            self.assertEqual(code, 1)
            self.assertEqual(source.commands, ["*IDN?"])
            self.assertFalse(hasattr(board, "launch"))

    def test_unknown_exit_and_missing_filename_never_infer_success(self):
        stop = threading.Event()
        stop.set()
        with self.assertRaises(RuntimeError):
            recv_exit_status(FakeChannel(stop, -1), .01)
        with self.assertRaises(RuntimeError):
            download_artifacts(None, {}, None, ".")

    def test_threshold_units_and_nonfinite_rejected(self):
        self.assertEqual(parse_test_args(["--dry-run", "--recovery-threshold-deg", "0.72"]).recovery_threshold_ps, 1)
        with self.assertRaises(ValueError):
            parse_test_args(["--dry-run", "--recovery-threshold-ps", "nan"])

    def test_batch_stops_after_one_failure(self):
        with tempfile.TemporaryDirectory() as root, mock.patch.object(runner, "execute_run", return_value=1) as execute, \
                mock.patch("tools.analyze_out_of_loop.analyze_batch"), mock.patch("sys.stdout", new_callable=io.StringIO):
            self.assertEqual(runner.main(["--source-address", "SIM_SOURCE", "--serial-port", "TEST_CONSOLE", "--hz-per-mv", "1", "--execute", "--scope-capture", "lan", "--scope-address", "SIM", "--repeats", "3",
                                          "--output-dir", root, "--batch-id", "fail"]), 1)
            self.assertEqual(execute.call_count, 1)

    def test_rate_check_does_not_create_board_or_source(self):
        api = mock.Mock()
        api.run_rate_check.return_value = {"new_data_20hz_verified": False}
        with mock.patch.object(runner, "_scope_api", return_value=api), \
                mock.patch.object(runner, "BoardSSH", side_effect=AssertionError("no SSH")), \
                mock.patch.object(runner, "VisaSource", side_effect=AssertionError("no source")), \
                mock.patch("sys.stdout", new_callable=io.StringIO):
            self.assertEqual(runner.main(["--scope-rate-check", "--scope-address", "SIM"]), 0)
            api.run_rate_check.assert_called_once()
            self.assertEqual(api.run_rate_check.call_args.kwargs["visa_library"], site_settings.SCOPE_VISA_LIBRARY)


if __name__ == "__main__":
    unittest.main()
