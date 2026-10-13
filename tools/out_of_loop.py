#!/usr/bin/env python3
"""Repeated PI out-of-loop experiment. Import and --dry-run never connect."""
import argparse
import csv
from datetime import datetime
import json
import math
import os
from pathlib import Path
import re
import secrets
import shlex
import shutil
import subprocess
import sys
import tempfile
import threading
import time

try:
    from site_settings import SERIAL_PORT, SERIAL_BAUD, SCOPE_VISA_ADDRESS, SCOPE_VISA_LIBRARY, SCOPE_VISA_DLL_DIR
    from experiment_plan import BASE_FREQ, HZ_PER_MV, CONDITIONS, full_plan, repeated_runs, verify_config
    from board_session import BoardSSH, capture_board, download_artifacts, evidence_health, logger_filename, recv_exit_status, remote_paths, sha256_file
except ImportError:
    from tools.site_settings import SERIAL_PORT, SERIAL_BAUD, SCOPE_VISA_ADDRESS, SCOPE_VISA_LIBRARY, SCOPE_VISA_DLL_DIR
    from tools.experiment_plan import BASE_FREQ, HZ_PER_MV, CONDITIONS, full_plan, repeated_runs, verify_config
    from tools.board_session import BoardSSH, capture_board, download_artifacts, evidence_health, logger_filename, recv_exit_status, remote_paths, sha256_file

PROJECT = Path(__file__).resolve().parents[1]
EVENT_FIELDS = ("event", "module", "planned_s", "monotonic_s", "local_datetime",
                "state2_elapsed_s", "command", "command_stage",
                "command_start_monotonic_s", "command_end_monotonic_s",
                "command_start_local", "command_end_local", "freq_hz",
                "commanded_freq_hz", "equiv_mv", "status", "error", "response")


def local_now():
    return datetime.now().astimezone().isoformat(timespec="microseconds")


def write_json(path, content):
    path = Path(path)
    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                     prefix=".manifest-", suffix=".tmp", delete=False) as stream:
        temporary = stream.name
        json.dump(content, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def local_provenance():
    files = ("out_of_loop.py", "experiment_plan.py", "board_session.py", "serial_board.py",
             "site_settings.py",
             "scope_86100c.py", "analyze_out_of_loop.py")
    result = dict(project_path=str(PROJECT), module_sha256={
        "tools/" + name: sha256_file(PROJECT / "tools" / name)
        for name in files if (PROJECT / "tools" / name).is_file()},
        git_commit="unavailable", git_status="unavailable", git_branch="unavailable")
    executable = shutil.which("git")
    if executable is None:
        bundled = Path.home() / ".cache" / "codex-runtimes" / "codex-primary-runtime" / "dependencies" / "native" / "git" / "cmd" / "git.exe"
        if bundled.is_file():
            executable = str(bundled)
    if executable:
        for key, command in (("git_commit", ["rev-parse", "HEAD"]),
                             ("git_branch", ["branch", "--show-current"]),
                             ("git_status", ["status", "--porcelain=v1", "--untracked-files=normal"])):
            try:
                output = subprocess.run([executable, "-C", str(PROJECT), *command], capture_output=True,
                                        text=True, encoding="utf-8", errors="replace", timeout=10, check=True)
                result[key] = output.stdout.strip()
            except (OSError, subprocess.SubprocessError) as exc:
                result[key + "_error"] = str(exc)
    return result


class EventLedger:
    """Flush command intent before sending, and preserve a returned/error row."""
    def __init__(self, path):
        self.stream = open(path, "x", encoding="utf-8", newline="")
        self.writer = csv.DictWriter(self.stream, fieldnames=EVENT_FIELDS)
        self.writer.writeheader()
        self.stream.flush()
        self.lock = threading.Lock()
        self.origin = None
        self.rows = []

    def record(self, event, **fields):
        now = fields.pop("monotonic_s", time.monotonic())
        row = dict(event=event, monotonic_s=now, local_datetime=local_now(),
                   state2_elapsed_s="" if self.origin is None else now - self.origin,
                   command_stage="marker", status="ok")
        row.update(fields)
        with self.lock:
            if not self.stream.closed:
                self.writer.writerow(row)
                self.stream.flush()
                self.rows.append(row)
        return row

    def command(self, source, command, event, query=False, **fields):
        start, start_local = time.monotonic(), local_now()
        common = dict(command=command, command_start_monotonic_s=start,
                      command_start_local=start_local, **fields)
        self.record(event, monotonic_s=start, command_stage="before", **common)
        try:
            response = source.query(command) if query else source.write(command)
        except BaseException as exc:
            self.record(event, command_stage="error", status="error", error=str(exc),
                        command_end_monotonic_s=time.monotonic(), command_end_local=local_now(), **common)
            raise
        self.record(event, command_stage="after", command_end_monotonic_s=time.monotonic(),
                    command_end_local=local_now(), response=str(response) if query else "", **common)
        return response

    def frequency(self, source, event, **fields):
        frequency = fields.get("freq_hz", BASE_FREQ)
        # Instrument frequency commands use integer Hz.
        command = f":FREQ {frequency:.0f}Hz"
        fields["commanded_freq_hz"] = float(f"{frequency:.0f}")
        return self.command(source, command, event, **fields)

    def close(self):
        with self.lock:
            self.stream.close()


class VisaSource:
    def __init__(self, address, timeout_ms):
        import pyvisa
        self.manager = pyvisa.ResourceManager()
        try:
            self.instrument = self.manager.open_resource(address)
            self.instrument.timeout = timeout_ms
            self.instrument.read_termination = "\n"
            self.instrument.write_termination = "\n"
        except BaseException:
            self.manager.close()
            raise

    def query(self, command):
        return self.instrument.query(command)

    def write(self, command):
        return self.instrument.write(command)

    def close(self):
        try:
            self.instrument.close()
        finally:
            self.manager.close()


def create_batch(args):
    batch_id = args.batch_id or datetime.now().strftime("%Y%m%d_%H%M%S_") + secrets.token_hex(3)
    if not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", batch_id):
        raise ValueError("batch-id must be 1-64 ASCII letters, digits, _ or -")
    templates = {}
    for algo, filename in (("PI", "config.txt"),):
        path = PROJECT / filename
        verify_config(path.read_text(encoding="utf-8"), algo)
        templates[algo] = path
    batch_dir = (Path(args.output_dir) / batch_id).resolve()
    batch_dir.mkdir(parents=True, exist_ok=False)
    runs = []
    for condition in repeated_runs(args.repeats):
        run_id = f"{batch_id}_B{condition['block']:02d}_P{condition['position']:02d}_{condition['algo']}"
        run_dir = batch_dir / f"block_{condition['block']:02d}" / f"{condition['position']:02d}_{condition['condition']}"
        run_dir.mkdir(parents=True)
        config_path = run_dir / "config.txt"
        config_path.write_bytes(templates[condition["algo"]].read_bytes())
        runs.append(dict(**condition, run_id=run_id, run_dir=str(run_dir),
                         config_path=str(config_path), config_sha256=sha256_file(config_path),
                         remote=remote_paths(args.board_project, args.binary, run_id),
                         run_manifest_path=str(run_dir / "run_manifest.json"),
                         scope_csv=str(run_dir / "scope_delta_time.csv"),
                         scope_local_export_dir=str(run_dir / "scope_native_export"),
                         events_csv=str(run_dir / "events.csv"),
                         controller_log=str(run_dir / "controller_output.log"), status="planned"))
    manifest = dict(schema="phase-stabilization-pi/out-of-loop/v1", batch_id=batch_id,
                    local_code=local_provenance(),
                    batch_dir=str(batch_dir), created_local=local_now(),
                    repeats=args.repeats, independent_unit="one full controller run",
                    repetition_order="one PI run per block",
                    conditions=list(CONDITIONS), planned_timeline=full_plan(args.base_freq_hz,args.hz_per_mv), planned_duration_s=165.5,
                    base_freq_hz=args.base_freq_hz, hz_per_equivalent_mv=args.hz_per_mv,
                    disturbance_units="equivalent in-loop mV, not measured out-of-loop phase",
                    scope_capture=args.scope_capture,
                    scope_address=args.scope_address if args.scope_capture == "lan" else None,
                    scope_visa_library=(args.scope_visa_library or "auto") if args.scope_capture == "lan" else None,
                    source_address=args.source_address,
                    scope_measurement="86100C Delta Time(M1, Ch2)",
                    scope_time_note=("Scope native files require independent verification of per-measurement values, timestamps and alignment with laptop events; no scope connection is made."
                                     if args.scope_capture == "local" else
                                     "Notebook monotonic query start/end approximate query time, not acquisition time. New acquisitions and 20 Hz coverage are unverified."),
                    scope_local_procedure_status=("requires_operator_confirmation_and_file_validation" if args.scope_capture == "local" else None),
                    scope_local_export_note=("After each batch, copy the instrument's original local files into each run's scope_native_export directory. "
                                             "Do not rename source files or infer timestamps from file order. Analysis remains pending until format, timing and alignment are verified."
                                             if args.scope_capture == "local" else None),
                    stimulus_conversion_note="operator-supplied; offline default is an illustrative assumption",
                    source_power_policy="query and record only; leave front-panel power unchanged",
                    recovery_threshold_ps=args.recovery_threshold_ps,
                    recovery_threshold_input_deg=args.recovery_threshold_deg,
                    recovery_hold_s=args.recovery_hold_s, recovery_max_gap_s=args.recovery_max_gap_s,
                    max_event_lateness_s=args.max_event_lateness_s,
                    host=args.host, conn=args.conn, board_project=args.board_project,
                    binary=args.binary, maintenance_serial_port=args.serial_port,
                    serial_baud=args.serial_baud,
                    serial_role="serial shell console" if args.conn == "serial" else "maintenance port; batch uses SSH",
                    status="dry_run" if args.dry_run else "planned", runs=runs)
    for run in runs:
        if args.dry_run:
            run["status"] = "dry_run"
        write_json(run["run_manifest_path"], run)
    write_json(batch_dir / "batch_manifest.json", manifest)
    return manifest


def _scope_api():
    try:
        import scope_86100c
    except ImportError:
        from tools import scope_86100c
    return scope_86100c


def _health(run, rows, args):
    reasons = []
    if run["status"] != "completed":
        reasons.append("run_not_completed")
    if run.get("controller_exit_status") != 0:
        reasons.append("controller_exit_not_zero_or_unknown")
    artifacts = run.get("downloaded_artifacts", {})
    if not artifacts.get("complete"):
        reasons.append("missing_or_unverified_board_artifacts")
    if run.get("cleanup_errors"):
        reasons.append("cleanup_error")
    commands, lateness_values = [], []
    for row in rows:
        if row.get("planned_s") is not None and row.get("command_stage") == "after":
            lateness = row["command_start_monotonic_s"] - run["state2_monotonic_s"] - row["planned_s"]
            lateness_values.append(lateness)
            commands.append(row)
            if abs(lateness) > args.max_event_lateness_s:
                reasons.append("disturbance_timing_deviation")
    interval_errors = []
    for before, after in zip(commands, commands[1:]):
        planned = after["planned_s"] - before["planned_s"]
        actual = after["command_start_monotonic_s"] - before["command_start_monotonic_s"]
        error = actual - planned
        interval_errors.append(error)
        if abs(error) > (.05 if planned <= .2 else .25):
            reasons.append("disturbance_interval_deviation")
    safety = artifacts.get("files", {}).get("safety_csv")
    if safety:
        with open(safety, encoding="utf-8", newline="") as stream:
            for row in csv.DictReader(stream):
                if any(row.get(key, "0") not in ("", "0") for key in (
                        "Final_stop_state_UNKNOWN", "Final_hardware_intervention_required")):
                    reasons.append("safety_stop_unknown_or_intervention_required")
                    break
    more_reasons, diagnostics = evidence_health(run)
    reasons.extend(more_reasons)
    if args.scope_capture == "local":
        reasons.append("scope_local_data_pending_validation")
        diagnostics.update(scope_capture="local", scope_connection_made=False,
                           scope_timestamp_alignment_verified=False,
                           event_start_lateness_s=lateness_values,
                           event_start_interval_error_s=interval_errors,
                           thresholds=dict(max_sample_dt_s=.1, max_previous_cycle_us=100000,
                                           max_event_lateness_s=args.max_event_lateness_s,
                                           max_short_interval_error_s=.05, max_long_interval_error_s=.25))
        return dict(eligible_for_analysis=False, reasons=sorted(set(reasons)), diagnostics=diagnostics)
    scope_summary = run.get("scope_summary", {})
    count_observation = scope_summary.get("measurement_results_n_samples_observation", {})
    if (scope_summary.get("query_count", 0) >= 2
            and count_observation.get("total_positive_change") == 0
            and scope_summary.get("observed_result_value_change_count") == 0):
        reasons.append("scope_result_table_not_updating")
    if run.get("baseline", {}).get("baseline_ps") is None:
        reasons.append("no_valid_pre_disturbance_scope_baseline")
    diagnostics.update(event_start_lateness_s=lateness_values,
                       event_start_interval_error_s=interval_errors,
                       thresholds=dict(max_sample_dt_s=.1, max_previous_cycle_us=100000,
                                       max_event_lateness_s=args.max_event_lateness_s,
                                       max_short_interval_error_s=.05, max_long_interval_error_s=.25))
    return dict(eligible_for_analysis=not reasons, reasons=sorted(set(reasons)), diagnostics=diagnostics)


def execute_run(run, args, *, plan=None, board_factory=None, source_factory=None,
                scope_factory=None, collector_factory=None):
    """Dependency injection supports complete offline SSH/VISA lifecycle tests."""
    plan = full_plan(args.base_freq_hz,args.hz_per_mv) if plan is None else plan
    if board_factory is None:
        if args.conn == "serial":
            try:
                from serial_board import SerialBoard
            except ImportError:
                from tools.serial_board import SerialBoard
            board_factory = SerialBoard
        else:
            board_factory = BoardSSH
    source_factory = source_factory or VisaSource
    if args.scope_capture == "lan" and (scope_factory is None or collector_factory is None):
        api = _scope_api()
        scope_factory = scope_factory or (lambda address, timeout_ms: api.Scope86100C.connect(
            address, timeout_ms=timeout_ms, visa_library=args.scope_visa_library,
            visa_dll_dir=args.scope_visa_dll_dir))
        collector_factory = collector_factory or api.ScopeCollector
    ledger = EventLedger(run["events_csv"])
    log = open(run["controller_log"], "x", encoding="utf-8")
    log_lock = threading.Lock()
    state2, eof, status_ready = threading.Event(), threading.Event(), threading.Event()
    board = source = scope = collector = output_thread = None
    launched = source_verified = False
    scope_close_deferred = False
    run.update(status="starting", started_local=local_now(), controller_exit_status=None,
               cleanup_errors=[], reader_error=None, controller_status_error=None)
    write_json(run["run_manifest_path"], run)

    def save():
        write_json(run["run_manifest_path"], run)

    def controller_reader(stdout):
        try:
            for line in stdout:
                received = time.monotonic()
                if isinstance(line, bytes):
                    line = line.decode("utf-8", errors="replace")
                with log_lock:
                    if not log.closed:
                        log.write(f"{local_now()} monotonic={received:.9f} {line.rstrip()}\n")
                        log.flush()
                filename = logger_filename(line)
                if filename:
                    if run.get("remote_csv_name") not in (None, filename):
                        raise RuntimeError("controller reported multiple DataLogger filenames")
                    run["remote_csv_name"] = filename
                if "[State 2]" in line and not state2.is_set():
                    run["state2_monotonic_s"] = received
                    ledger.origin = received
                    if collector is not None:
                        collector.mark_state2(received)
                    ledger.record("STATE2_RECEIVED", monotonic_s=received)
                    state2.set()
        except Exception as exc:
            run["reader_error"] = f"{type(exc).__name__}: {exc}"
        finally:
            eof.set()
            try:
                run["controller_exit_status"] = recv_exit_status(stdout.channel, args.exit_timeout_s)
            except Exception as exc:
                run["controller_status_error"] = str(exc)
            status_ready.set()

    def check_health():
        if eof.is_set():
            raise RuntimeError("controller output ended before the event plan completed")
        if collector is not None and collector.error:
            raise RuntimeError("scope acquisition transport failed: " + collector.error)

    try:
        board = board_factory(args)
        run["board"] = capture_board(board, run["remote"], args.board_project,
                                      run["config_path"], run["algo"])
        run["board"]["transport"] = getattr(board, "transport_metadata", {"kind": "ssh", "host": args.host})
        save()
        # Native scope recording is independent: never open a scope session in local mode.
        if args.scope_capture == "lan":
            scope = scope_factory(args.scope_address, timeout_ms=args.scope_timeout_ms)
            run["scope_idn"] = getattr(scope, "idn", "unavailable")
        else:
            Path(run["scope_local_export_dir"]).mkdir(exist_ok=True)
            run["scope_connection_made"] = False
            run["scope_native_file_status"] = "awaiting_operator_export_and_format_validation"
            ledger.record("SCOPE_LOCAL_RECORDING_OPERATOR_CONFIRMED",
                          response="Instrument-local recording confirmed by --scope-local-ready; not remotely verified")
        source = source_factory(args.source_address, args.source_timeout_ms)
        run["source_idn"] = str(ledger.command(source, "*IDN?", "SOURCE_IDN", query=True)).strip()
        id_fields = run["source_idn"].split(",")
        if len(id_fields) < 2 or id_fields[1].strip().upper() != "E8257D":
            raise ValueError(f"Expected E8257D source; received {run['source_idn']!r}")
        source_verified = True
        run["source_power_dbm_raw"] = str(ledger.command(source, ":POWer?", "SOURCE_POWER_QUERY", query=True)).strip()
        ledger.frequency(source, "INITIAL_BASE_FREQUENCY", freq_hz=args.base_freq_hz, equiv_mv=0)
        if args.scope_capture == "lan":
            collector = collector_factory(scope, run["scope_csv"], interval_s=args.scope_interval_s)
            collector.start()
        # A lost transport response can occur after a remote launch succeeded.
        # Treat launch intent as ownership and attempt PID-checked stop on error.
        launched = True
        run["launch_requested"] = True
        save()
        stdout = board.exec_long(run["remote"]["launch"])
        output_thread = threading.Thread(target=controller_reader, args=(stdout,), daemon=True)
        output_thread.start()
        deadline = time.monotonic() + args.state2_timeout_s
        while not state2.wait(.02):
            check_health()
            if time.monotonic() >= deadline:
                raise TimeoutError("controller did not reach State 2 before timeout")
        check_health()
        run["status"] = "running"
        save()
        baseline_frozen = False
        for event in plan:
            deadline = run["state2_monotonic_s"] + event["planned_s"]
            while True:
                check_health()
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                eof.wait(min(remaining, .02))
            fields = dict(planned_s=event["planned_s"], module=event["module"] or "",
                          freq_hz=event["freq_hz"], equiv_mv=event["equiv_mv"])
            if event["apply_frequency"]:
                if collector is not None and not baseline_frozen:
                    run["baseline"] = collector.freeze_baseline(time.monotonic())
                    baseline_frozen = True
                ledger.frequency(source, event["event"], **fields)
            else:
                ledger.record(event["event"], **fields)
        run["completed_plan_elapsed_s"] = time.monotonic() - run["state2_monotonic_s"]
        run["status"] = "plan_completed"
    except KeyboardInterrupt:
        run.update(status="interrupted", error="KeyboardInterrupt")
    except Exception as exc:
        run.update(status="failed", error=f"{type(exc).__name__}: {exc}")
    finally:
        # Restore RF first on every path.
        if source is not None and source_verified:
            try:
                ledger.frequency(source, "FINAL_BASE_FREQUENCY", freq_hz=args.base_freq_hz, equiv_mv=0)
                run["source_restored_base_frequency"] = True
            except Exception as exc:
                run["source_restored_base_frequency"] = False
                run["cleanup_errors"].append("source restore: " + str(exc))
        if board is not None and launched:
            try:
                ledger.record("CONTROLLER_STOP_REQUEST")
                board.run(run["remote"]["stop"])
                if output_thread is None:
                    raise RuntimeError("launch response failed; controlled stop attempted but exit status is unknown")
                if not eof.wait(args.exit_timeout_s):
                    raise TimeoutError("controller output did not end after controlled stop")
                if not status_ready.wait(args.exit_timeout_s + 1):
                    raise TimeoutError("controller exit status unavailable after controlled stop")
                ledger.record("CONTROLLER_EXIT", response=str(run["controller_exit_status"]))
            except Exception as exc:
                run["cleanup_errors"].append("controller stop: " + str(exc))
        if collector is not None:
            try:
                collector.stop()
                run["scope_summary"] = collector.summary()
                if collector.error:
                    run["cleanup_errors"].append("scope: " + collector.error)
            except Exception as exc:
                run["cleanup_errors"].append("scope stop: " + str(exc))
                scope_close_deferred = True
                collector.close_scope_when_stopped()
        if board is not None and launched:
            try:
                run["downloaded_artifacts"] = download_artifacts(
                    board, run["remote"], run.get("remote_csv_name"), run["run_dir"],
                    allow_partial=run["status"] != "plan_completed")
            except Exception as exc:
                run["cleanup_errors"].append("artifact download: " + str(exc))
        for label, device in (("source", source), ("scope", scope), ("SSH", board)):
            if device is not None:
                if label == "scope" and scope_close_deferred:
                    continue
                try:
                    device.close()
                except Exception as exc:
                    run["cleanup_errors"].append(label + " close: " + str(exc))
        if output_thread is not None:
            output_thread.join(timeout=1)
        with log_lock:
            log.close()
        if run["status"] == "plan_completed":
            if (run["controller_exit_status"] == 0 and not run["reader_error"]
                    and not run["controller_status_error"] and not run["cleanup_errors"]
                    and run.get("downloaded_artifacts", {}).get("complete")):
                run["status"] = "completed"
            else:
                run["status"] = "failed"
        if collector is not None:
            run["scope_summary"] = collector.summary()
        if scope is not None:
            run["scope_session"] = scope.metadata()
        try:
            run["health"] = _health(run, ledger.rows, args)
        except Exception as exc:
            run["health"] = dict(eligible_for_analysis=False, reasons=["health_check_failed"], error=str(exc))
        if run["status"] == "completed" and not run["health"]["eligible_for_analysis"]:
            run["status"] = ("completed_pending_scope" if args.scope_capture == "local" else "invalid")
        run["finished_local"] = local_now()
        ledger.close()
        # Hash only closed local evidence. A timed-out VISA reader may still
        # append its final response; leave that checksum explicitly pending.
        scope_summary = run.get("scope_summary", {})
        run["scope_hash_pending"] = bool(scope_close_deferred and (
            scope_summary.get("collector_thread_alive", True)
            or scope_summary.get("scope_close_pending", True)))
        run["evidence_sha256"] = {}
        for key in ("events_csv", "controller_log", "scope_csv"):
            path = Path(run[key])
            if key == "scope_csv" and run["scope_hash_pending"]:
                continue
            if path.is_file():
                try:
                    run["evidence_sha256"][path.name] = sha256_file(path)
                except OSError as exc:
                    run.setdefault("evidence_hash_errors", []).append(f"{path.name}: {exc}")
        if run.get("evidence_hash_errors"):
            run["health"]["eligible_for_analysis"] = False
            run["health"]["reasons"].append("local_evidence_hash_failure")
            if run["status"] in ("completed", "completed_pending_scope"):
                run["status"] = "invalid"
        save()
    return 0 if run["status"] in ("completed", "completed_pending_scope") else (130 if run["status"] == "interrupted" else 1)


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    mode = result.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-run", action="store_true", help="save complete preview; no SSH/VISA imports or connections")
    result.add_argument("--base-freq-hz",type=float,default=BASE_FREQ)
    result.add_argument("--hz-per-mv",type=float,help="operator-provided stimulus conversion; required for execute")
    mode.add_argument("--execute", action="store_true", help="explicitly run the physical experiment")
    mode.add_argument("--scope-rate-check", action="store_true", help="connect only to scope; no source or board")
    result.add_argument("--repeats", type=int, default=3)
    result.add_argument("--batch-id")
    result.add_argument("--output-dir", default=str(PROJECT / "experiments" / "out_of_loop"))
    result.add_argument("--conn", choices=("serial", "wired", "wifi"), default="serial")
    result.add_argument("--host")
    result.add_argument("--ssh-user")
    result.add_argument("--ssh-port", type=int)
    result.add_argument("--ssh-key")
    result.add_argument("--known-hosts")
    result.add_argument("--serial-port", default=SERIAL_PORT,
                        help=f"board serial console port (default {SERIAL_PORT}; edit tools/site_settings.py or override here)")
    result.add_argument("--serial-baud", type=int, default=SERIAL_BAUD)
    result.add_argument("--serial-password-env", help="optional environment variable holding serial login password")
    result.add_argument("--board-project", default="/opt/phase-stabilization-pi")
    result.add_argument("--binary", help="absolute board binary path; default BOARD_PROJECT/build-rp/PhaseStabilization")
    result.add_argument("--source-address", default=os.environ.get("PI_SOURCE_ADDRESS", ""))
    result.add_argument("--source-timeout-ms", type=int, default=5000)
    result.add_argument("--scope-capture", choices=("local", "lan"), default="local",
                        help="local: no computer-scope connection; native files exported later (default). lan: opt-in SCPI collector")
    result.add_argument("--scope-local-ready", action="store_true",
                        help="confirm instrument-local continuous M1->Ch2 recording is already armed; required for local --execute")
    result.add_argument("--scope-address", default=SCOPE_VISA_ADDRESS,
                        help=f"86100C VISA resource (default {SCOPE_VISA_ADDRESS})")
    result.add_argument("--scope-visa-library", default=SCOPE_VISA_LIBRARY,
                        help="VISA backend DLL or @py; empty string selects PyVISA default")
    result.add_argument("--scope-visa-dll-dir", default=SCOPE_VISA_DLL_DIR,
                        help="Windows directory containing dependencies of the chosen VISA DLL")
    result.add_argument("--scope-timeout-ms", type=int, default=2000)
    result.add_argument("--scope-interval-s", type=float, default=0.075,
                        help="minimum time between scope query starts (default 0.075 s; does not verify acquisition rate)")
    result.add_argument("--rate-check-duration-s", type=float, default=10)
    result.add_argument("--state2-timeout-s", type=float, default=300)
    result.add_argument("--exit-timeout-s", type=float, default=10)
    threshold = result.add_mutually_exclusive_group()
    threshold.add_argument("--recovery-threshold-ps", type=float)
    threshold.add_argument("--recovery-threshold-deg", type=float)
    result.add_argument("--recovery-hold-s", type=float, default=.5)
    result.add_argument("--recovery-max-gap-s", type=float, default=.1)
    result.add_argument("--max-event-lateness-s", type=float, default=.05)
    return result


def parse_args(argv=None):
    args = parser().parse_args(argv)
    args.host = args.host or os.environ.get("PI_BOARD_HOST")
    args.binary = args.binary or args.board_project.rstrip("/") + "/build-rp/PhaseStabilization"
    if args.execute:
        if not args.source_address: raise ValueError("--source-address is required")
        if args.conn == "serial" and not args.serial_port: raise ValueError("--serial-port is required")
        if args.conn != "serial" and not args.host: raise ValueError("--host is required")
        if args.hz_per_mv is None: raise ValueError("--hz-per-mv is required")
    args.hz_per_mv = HZ_PER_MV if args.hz_per_mv is None else args.hz_per_mv
    full_plan(args.base_freq_hz,args.hz_per_mv)  # validates stimulus parameters
    if args.base_freq_hz != BASE_FREQ:
        raise ValueError("this example requires a 2 GHz carrier for phase conversion")
    if args.repeats < 1:
        raise ValueError("--repeats must be positive")
    for field in ("serial_baud", "source_timeout_ms", "scope_timeout_ms", "rate_check_duration_s", "state2_timeout_s",
                  "exit_timeout_s", "recovery_hold_s", "recovery_max_gap_s", "max_event_lateness_s"):
        value = getattr(args, field)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"--{field.replace('_', '-')} must be positive and finite")
    if not math.isfinite(args.scope_interval_s) or args.scope_interval_s < 0:
        raise ValueError("--scope-interval-s must be nonnegative and finite")
    for value in (args.recovery_threshold_ps, args.recovery_threshold_deg):
        if value is not None and (not math.isfinite(value) or value <= 0):
            raise ValueError("recovery threshold must be positive and finite")
    if args.recovery_threshold_deg is not None:
        args.recovery_threshold_ps = args.recovery_threshold_deg / .72
    if (args.scope_rate_check or args.scope_capture == "lan") and not args.dry_run and not args.scope_address:
        raise ValueError("--scope-address is required for LAN capture or --scope-rate-check")
    if args.execute and args.scope_capture == "local" and not args.scope_local_ready:
        raise ValueError("--scope-local-ready is required: arm and verify native scope recording before --execute")
    return args


def main(argv=None):
    try:
        args = parse_args(argv)
        if args.scope_rate_check:
            try:
                result = _scope_api().run_rate_check(args.scope_address, args.output_dir,
                                                    duration_s=args.rate_check_duration_s,
                                                    interval_s=args.scope_interval_s,
                                                    timeout_ms=args.scope_timeout_ms,
                                                    visa_library=args.scope_visa_library,
                                                    visa_dll_dir=args.scope_visa_dll_dir)
            except Exception as exc:
                print(f"Scope rate check failed: {type(exc).__name__}: {exc}", file=sys.stderr)
                print(f"Report: {Path(args.output_dir) / 'scope_rate_check.json'}", file=sys.stderr)
                return 1
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 1 if result.get("error") else 0
        batch = create_batch(args)
        print("Batch manifest: " + str(Path(batch["batch_dir"]) / "batch_manifest.json"))
        for run in batch["runs"]:
            print(f"block {run['block']} order {run['position']}: {run['condition']} -> {run['run_dir']}")
        if args.dry_run:
            print("DRY-RUN: no SSH, E8257D or 86100C connections; complete event plan:")
            for event in batch["planned_timeline"]:
                print(f"{event['planned_s']:8.3f}s {event['event']:30} {event['freq_hz']:.3f} Hz; equivalent {event['equiv_mv']:+.3f} mV")
            print("Per-run output: config.txt, run_manifest.json, events.csv, controller_output.log, exact Data_* artifacts")
            if args.scope_capture == "local":
                print("Scope: native local recording only; export raw files after the batch to each scope_native_export directory. No scope connection or immediate out-of-loop metrics.")
            else:
                print("Scope: opt-in LAN capture creates scope_delta_time.csv")
            return 0
        batch["status"] = "running"
        result = 0
        for run in batch["runs"]:
            write_json(Path(batch["batch_dir"]) / "batch_manifest.json", batch)
            result = execute_run(run, args)
            write_json(Path(batch["batch_dir"]) / "batch_manifest.json", batch)
            if result:
                break
        batch["status"] = ("completed_pending_scope" if result == 0 and args.scope_capture == "local"
                           else "completed" if result == 0 else "stopped_after_failure")
        write_json(Path(batch["batch_dir"]) / "batch_manifest.json", batch)
        try:
            try:
                from analyze_out_of_loop import analyze_batch
            except ImportError:
                from tools.analyze_out_of_loop import analyze_batch
            analyze_batch(batch["batch_dir"])
        except Exception as exc:
            batch["analysis_error"] = str(exc)
            write_json(Path(batch["batch_dir"]) / "batch_manifest.json", batch)
            print("Offline analysis failed; raw evidence preserved: " + str(exc), file=sys.stderr)
            return result or 1
        return result
    except (ValueError, RuntimeError, OSError, ImportError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
