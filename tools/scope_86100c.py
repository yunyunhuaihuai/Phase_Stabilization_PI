"""86100C legacy-GUI Delta Time reader; no acquisition/trigger/M1 reconfiguration.

All timestamps use the laptop's time.monotonic() domain. A query envelope bounds
the software transaction, NOT the age of the waveform returned by the instrument.
Only ``connect`` imports PyVISA. Importing this module is safe for offline preview.
"""

from __future__ import annotations

import csv
import json
import math
import os
import re
import statistics
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

# Read the displayed measurement table without changing its definition.
DELTA_QUERY = ":MEASure:RESults?"
DIRECT_DELTA_QUERY = ":MEASure:DELTatime? WMEMory1,CHANnel2"
PHASE_DEG_PER_PS = 0.72  # This example assumes a 2 GHz carrier: 360*f*1e-12.
DEFAULT_SCOPE_INTERVAL_S = 0.075  # Software polling interval; acquisition rate unverified.
TIMESTAMP_SEMANTICS = "host_query_envelope_only; acquisition_timestamp_and_age_unknown"
CSV_FIELDS = [
    "query_index", "query", "query_start_monotonic_s", "query_end_monotonic_s",
    "query_start_local", "query_end_local", "query_duration_s",
    "query_start_interval_s", "state2_elapsed_s", "raw_response", "valid",
    "adjacent_valid_value_changed",
    "reason", "status_code", "result_name", "result_n_samples", "delta_time_ps", "baseline_ps",
    "relative_delay_ps", "relative_phase_deg", "timestamp_semantics",
    "new_acquisition_verified",
]

# Legacy guide 86100-90131, table 43. Nonzero states are always excluded.
_STATUS_REASONS = {
    0: "ok", 1: "questionable", 2: "upper_bound_only", 3: "lower_bound_only",
    4: "invalid", 5: "edge_missing", 6: "maximum_missing", 7: "minimum_missing",
    8: "time_missing", 9: "voltage_missing", 10: "equal_top_base",
    11: "small_measurement_zone", 12: "lower_threshold_missing",
    13: "upper_threshold_missing", 14: "thresholds_too_close", 15: "top_missing",
    16: "base_missing", 17: "incomplete", 18: "unsupported_signal",
    19: "signal_hidden", 20: "high_clipping", 21: "low_clipping",
    22: "both_clipping", 23: "holes_only", 24: "empty_display",
    25: "cursor_offscreen", 26: "measurement_aborted", 27: "measurement_timeout",
    28: "no_tracked_measurement", 30: "eye_missing", 32: "invalid_dark_level",
    33: "multiple_database_sources", 34: "rz_eye_missing",
    35: "excessive_er_correction", 37: "reference_plane_missing",
}


def local_time() -> str:
    return datetime.now().astimezone().isoformat(timespec="microseconds")


@dataclass(frozen=True)
class DeltaTimeResult:
    raw_response: str
    valid: bool
    reason: str
    status_code: int | None = None
    delta_time_ps: float | None = None
    result_name: str | None = None
    result_n_samples: float | None = None


def _without_header(raw: str, command_pattern: str) -> str:
    return re.sub(r"^\s*:?(?:" + command_pattern + r")\s+", "", raw,
                  count=1, flags=re.IGNORECASE).strip()


def parse_delta_time(raw: str) -> DeltaTimeResult:
    """Parse seconds[,state], optionally prefixed by the legacy response header.

    A finite scalar is accepted but explicitly says status_unavailable. With
    SENDvalid ON the documented normal response is value,0; any nonzero or
    unrecognized state is not a performance sample. The complete raw text remains.
    """
    raw = str(raw)
    payload = _without_header(raw, r"MEAS(?:URE)?:DELT(?:ATIME)?")
    fields = [part.strip() for part in payload.split(",")]
    if len(fields) not in (1, 2) or not fields[0]:
        return DeltaTimeResult(raw, False, "malformed_response")
    status = None
    if len(fields) == 2:
        if not re.fullmatch(r"[+-]?\d+", fields[1]):
            return DeltaTimeResult(raw, False, "malformed_status")
        status = int(fields[1])
    try:
        seconds = float(fields[0])
    except ValueError:
        return DeltaTimeResult(raw, False, "malformed_value", status)
    if not math.isfinite(seconds):
        return DeltaTimeResult(raw, False, "non_finite_value", status)
    # Manuals show both 9.999E+37 and 9.99999E+37, including negative examples.
    # Reject the entire absurd sentinel range before the seconds-to-ps conversion.
    if abs(seconds) >= 9.0e37:
        return DeltaTimeResult(raw, False, "invalid_sentinel", status)
    value_ps = seconds * 1e12
    if not math.isfinite(value_ps):
        return DeltaTimeResult(raw, False, "non_finite_ps", status)
    if status is not None and status != 0:
        reason = _STATUS_REASONS.get(status, "unknown_status")
        return DeltaTimeResult(raw, False, reason, status, value_ps)
    return DeltaTimeResult(raw, True, "ok" if status == 0 else "status_unavailable",
                           status, value_ps)


def _is_m1_ch2_delta_name(name: str) -> bool:
    # The legacy GUI emits a non-ASCII delta glyph (0xB4 on this instrument).
    # Match the sources exactly; a different displayed measurement is unsafe.
    compact = re.sub(r"\s+", "", name).casefold()
    return bool(re.fullmatch(r"(?:[^a-z0-9]*|delta)time\(m1\)-\(2\)", compact))


def parse_display_delta_time(raw: str) -> DeltaTimeResult:
    """Extract the displayed M1-to-Ch2 Delta Time from legacy RESults?.

    SENDvalid ON makes each measurement group name,current,state,min,max,
    mean,stddev,n-samples (plus optional limit-test fields). n-samples is a
    statistical field, never treated as a unique acquisition identifier.
    """
    raw = str(raw)
    payload = _without_header(raw, r"MEAS(?:URE)?:RES(?:ULTS)?")
    fields = [part.strip() for part in payload.split(",")]
    matches = [i for i, field in enumerate(fields) if _is_m1_ch2_delta_name(field)]
    if not matches:
        return DeltaTimeResult(raw, False, "measurement_not_displayed")
    if len(matches) != 1:
        return DeltaTimeResult(raw, False, "ambiguous_measurement")
    index = matches[0]
    if len(fields) < index + 8:
        return DeltaTimeResult(raw, False, "malformed_result_table", result_name=fields[index])
    selected = parse_delta_time(f"{fields[index + 1]},{fields[index + 2]}")
    n_samples = None
    try:
        candidate = float(fields[index + 7])
        if math.isfinite(candidate) and 0 <= candidate < 9e37:
            n_samples = candidate
    except ValueError:
        pass
    return DeltaTimeResult(raw, selected.valid, selected.reason, selected.status_code,
                           selected.delta_time_ps, fields[index], n_samples)


class Scope86100C:
    """One VISA session owned by one collector thread after connection.

    The sole instrument-setting write is SENDvalid ON, restored on close. Closing
    after an I/O failure attempts that write without another query, then closes the
    session. An I/O failure poisons the session: delayed replies are never reused.
    """

    def __init__(self, resource: Any, *, address: str = "", timeout_ms: int = 2000,
                 resource_manager: Any = None, owns_manager: bool = False,
                 visa_dll_handle: Any = None,
                 monotonic: Callable[[], float] = time.monotonic,
                 wall_clock: Callable[[], str] = local_time):
        self.resource = resource
        self.address = address
        self.timeout_ms = timeout_ms
        self.resource_manager = resource_manager
        self.owns_manager = owns_manager
        self.visa_dll_handle = visa_dll_handle
        self.monotonic = monotonic
        self.wall_clock = wall_clock
        self.idn = ""
        self.original_sendvalid: int | None = None
        self.changed_sendvalid = False
        self.transport_desynchronized = False
        self.closed = False
        self.close_errors: list[str] = []
        self.session_log: list[dict[str, Any]] = []

    @classmethod
    def connect(cls, address: str, timeout_ms: int = 2000,
                resource_manager: Any = None, visa_library: str | None = None,
                visa_dll_dir: str | None = None, **kwargs: Any) -> "Scope86100C":
        if not address or not address.strip():
            raise ValueError("A verified 86100C VISA address is required")
        if timeout_ms <= 0:
            raise ValueError("scope timeout must be positive")
        owns_manager = resource_manager is None
        dll_handle = None
        if resource_manager is None:
            import pyvisa  # Only the explicit live path creates a connection.
            if os.name == "nt" and visa_dll_dir:
                dll_handle = os.add_dll_directory(visa_dll_dir)
            try:
                resource_manager = pyvisa.ResourceManager(visa_library) if visa_library else pyvisa.ResourceManager()
            except BaseException:
                if dll_handle is not None:
                    dll_handle.close()
                raise
        try:
            resource = resource_manager.open_resource(address)
        except BaseException:
            try:
                if owns_manager:
                    resource_manager.close()
            finally:
                if dll_handle is not None:
                    dll_handle.close()
            raise
        scope = cls(resource, address=address, timeout_ms=timeout_ms,
                    resource_manager=resource_manager, owns_manager=owns_manager,
                    visa_dll_handle=dll_handle, **kwargs)
        try:
            resource.timeout = int(timeout_ms)
            # Legacy RESults? includes a one-byte non-ASCII delta glyph. Latin-1
            # maps each response byte reversibly to a Unicode code point.
            resource.encoding = "latin-1"
            resource.read_termination = "\n"
            resource.write_termination = "\n"
            scope.idn = scope._query("*IDN?")["raw_response"].strip()
            id_fields = scope.idn.split(",")
            if len(id_fields) < 2 or id_fields[1].strip().upper() != "86100C":
                raise ValueError(f"Expected 86100C legacy instrument, received {scope.idn!r}")
            raw = scope._query(":MEASure:SENDvalid?")["raw_response"]
            value = _without_header(raw, r"MEAS(?:URE)?:SEND(?:VALID)?")
            if value not in ("0", "1"):
                raise ValueError(f"Unexpected legacy SENDvalid response: {raw!r}")
            scope.original_sendvalid = int(value)
            if scope.original_sendvalid == 0:
                # Mark before sending: a transport exception may occur after receipt.
                scope.changed_sendvalid = True
                scope._write(":MEASure:SENDvalid ON")
            return scope
        except BaseException:
            scope.close()
            raise

    def _write(self, command: str) -> None:
        item = {"command": command, "kind": "write",
                "query_start_monotonic_s": self.monotonic(),
                "query_start_local": self.wall_clock()}
        try:
            self.resource.write(command)
        except Exception as exc:
            item["error"] = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            item.update(query_end_monotonic_s=self.monotonic(), query_end_local=self.wall_clock())
            self.session_log.append(item)

    def _query(self, command: str) -> dict[str, Any]:
        if self.closed or self.transport_desynchronized:
            raise RuntimeError("Scope session is closed or transport_desynchronized")
        item: dict[str, Any] = {"query": command, "kind": "query",
                                "query_start_monotonic_s": self.monotonic(),
                                "query_start_local": self.wall_clock(), "raw_response": ""}
        try:
            item["raw_response"] = str(self.resource.query(command))
            if self.closed:
                raise RuntimeError("Response arrived after session close; rejected")
        except Exception as exc:
            self.transport_desynchronized = True
            item["error"] = f"{type(exc).__name__}: {exc}"
            # Preserve this transaction including failures in the caller's CSV.
            setattr(exc, "scope_transaction", item)
            raise
        finally:
            item.update(query_end_monotonic_s=self.monotonic(), query_end_local=self.wall_clock())
            item["query_duration_s"] = item["query_end_monotonic_s"] - item["query_start_monotonic_s"]
            self.session_log.append(item)
        return item

    def query_delta_time(self) -> dict[str, Any]:
        item = dict(self._query(DELTA_QUERY))
        item.update(asdict(parse_display_delta_time(item["raw_response"])))
        return item

    def query_direct_delta_time(self) -> dict[str, Any]:
        """Read the legacy M1-to-Ch2 scalar without changing measurement setup."""
        item = dict(self._query(DIRECT_DELTA_QUERY))
        item.update(asdict(parse_delta_time(item["raw_response"])))
        return item

    def close(self) -> None:
        if self.closed:
            return
        try:
            if self.changed_sendvalid:
                try:
                    self._write(f":MEASure:SENDvalid {self.original_sendvalid}")
                except Exception as exc:
                    self.close_errors.append(f"SENDvalid restoration failed: {exc}")
        finally:
            # Mark before touching the transport so a late return cannot be valid.
            self.closed = True
            try:
                self.resource.close()
            except Exception as exc:
                self.close_errors.append(f"VISA close failed: {exc}")
            if self.owns_manager:
                try:
                    self.resource_manager.close()
                except Exception as exc:
                    self.close_errors.append(f"ResourceManager close failed: {exc}")
            if self.visa_dll_handle is not None:
                self.visa_dll_handle.close()
            self.closed = True

    def __enter__(self) -> "Scope86100C":
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()

    def metadata(self) -> dict[str, Any]:
        return {"visa_address": self.address, "idn": self.idn,
                "query": DELTA_QUERY, "result_source": "displayed M1-to-Ch2 Delta Time row",
                "response_byte_mapping": "latin-1", "timeout_ms": self.timeout_ms,
                "original_sendvalid": self.original_sendvalid,
                "sendvalid_changed": self.changed_sendvalid,
                "transport_desynchronized": self.transport_desynchronized,
                "closed": self.closed, "close_errors": list(self.close_errors),
                "setup_and_cleanup_transactions": [dict(row) for row in self.session_log
                                                    if row.get("query") not in (DELTA_QUERY, DIRECT_DELTA_QUERY)]}


def distribution(values: list[float]) -> dict[str, Any]:
    if not values:
        return {"n": 0, "min": None, "p05": None, "median": None,
                "mean": None, "p95": None, "max": None}
    ordered = sorted(values)
    def percentile(p: float) -> float:
        position = (len(ordered) - 1) * p
        lo, hi = math.floor(position), math.ceil(position)
        return ordered[lo] + (ordered[hi] - ordered[lo]) * (position - lo)
    return {"n": len(values), "min": ordered[0], "p05": percentile(.05),
            "median": statistics.median(ordered), "mean": statistics.fmean(values),
            "p95": percentile(.95), "max": ordered[-1]}


class ScopeCollector:
    """Independent streaming reader. No scope query runs on the event scheduler.

    stop() must finish before scope.close(). CSV is exclusive-create and flushed
    after every response, including invalid responses and a final I/O failure.
    """

    def __init__(self, scope: Scope86100C, csv_path: str | Path,
                 interval_s: float = DEFAULT_SCOPE_INTERVAL_S,
                 query_mode: str = "results", stale_after_s: float | None = None):
        if not math.isfinite(interval_s) or interval_s < 0:
            raise ValueError("scope interval must be finite and nonnegative")
        if query_mode not in ("results", "direct"):
            raise ValueError("query_mode must be results or direct")
        if stale_after_s is not None and (not math.isfinite(stale_after_s) or stale_after_s <= 0):
            raise ValueError("stale_after_s must be positive and finite")
        self.scope = scope
        self.csv_path = Path(csv_path)
        self.interval_s = interval_s
        self.query_mode = query_mode
        self.query_command = DELTA_QUERY if query_mode == "results" else DIRECT_DELTA_QUERY
        self.stale_after_s = stale_after_s
        self.error: str | None = None
        self._stop = threading.Event()
        self._done = threading.Event()
        self._lock = threading.Lock()
        self._rows: list[dict[str, Any]] = []
        self._state2: float | None = None
        self._baseline: dict[str, Any] | None = None
        self._thread: threading.Thread | None = None
        self._file: Any = None
        self._started: float | None = None
        self._ended: float | None = None
        self._io_finished = False
        self._close_when_stopped = False
        self._scope_close_claimed = False

    def start(self) -> "ScopeCollector":
        if self._thread is not None:
            raise RuntimeError("Collector cannot be started twice")
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.csv_path.open("x", encoding="utf-8", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=CSV_FIELDS, extrasaction="ignore")
        self._writer.writeheader()
        self._file.flush()
        self._started = self.scope.monotonic()
        self._thread = threading.Thread(target=self._run, name="86100C-reader", daemon=True)
        self._thread.start()
        return self

    def mark_state2(self, monotonic_s: float) -> None:
        with self._lock:
            if self._state2 is None:
                self._state2 = monotonic_s

    def freeze_baseline(self, cutoff_monotonic_s: float) -> dict[str, Any]:
        """Mean of valid queries entirely after State 2 and before first injection.

        Does not wait for an in-flight query. A missing baseline stays missing;
        never use a post-disturbance reading as a replacement baseline.
        """
        with self._lock:
            if self._baseline is None:
                selected_rows = [row for row in self._rows
                                 if row["valid"] and self._state2 is not None
                                 and row["query_start_monotonic_s"] >= self._state2
                                 and row["query_end_monotonic_s"] <= cutoff_monotonic_s]
                values = [row["delta_time_ps"] for row in selected_rows]
                self._baseline = {
                    "baseline_ps": statistics.fmean(values) if values else None,
                    "sample_count": len(values), "method": "arithmetic_mean_valid_pre_disturbance_queries",
                    "query_indices": [row["query_index"] for row in selected_rows],
                    "state2_monotonic_s": self._state2, "cutoff_monotonic_s": cutoff_monotonic_s,
                    "status": "ok" if values else "no_valid_pre_disturbance_queries",
                    "acquisition_freshness": "unverified",
                }
            return dict(self._baseline)

    def _run(self) -> None:
        previous_start = None
        last_distinct_time = None
        previous_valid_value = None
        previous_valid = False
        try:
            while not self._stop.is_set():
                failure = None
                try:
                    row = (self.scope.query_delta_time() if self.query_mode == "results"
                           else self.scope.query_direct_delta_time())
                except Exception as exc:
                    failure = f"{type(exc).__name__}: {exc}"
                    row = dict(getattr(exc, "scope_transaction", {}))
                    now = self.scope.monotonic()
                    row.setdefault("query_start_monotonic_s", now)
                    row.setdefault("query_end_monotonic_s", now)
                    row.setdefault("query_start_local", self.scope.wall_clock())
                    row.setdefault("query_end_local", self.scope.wall_clock())
                    row.setdefault("raw_response", "")
                    row.setdefault("query_duration_s", 0.0)
                    row.update(valid=False, reason=f"transport_desynchronized: {failure}",
                               status_code=None, delta_time_ps=None)
                    self.error = row["reason"]
                start = row["query_start_monotonic_s"]
                with self._lock:
                    baseline = self._baseline.get("baseline_ps") if self._baseline else None
                    relative = (row["delta_time_ps"] - baseline
                                if row["valid"] and baseline is not None else None)
                    row.update(query_index=len(self._rows) + 1, query=self.query_command,
                               adjacent_valid_value_changed=(previous_valid and row["valid"]
                                                             and row["delta_time_ps"] != previous_valid_value),
                               query_start_interval_s=(start - previous_start if previous_start is not None else None),
                               state2_elapsed_s=(start - self._state2 if self._state2 is not None else None),
                               baseline_ps=baseline, relative_delay_ps=relative,
                               relative_phase_deg=(relative * PHASE_DEG_PER_PS if relative is not None else None),
                               timestamp_semantics=TIMESTAMP_SEMANTICS, new_acquisition_verified=False)
                    self._rows.append(row)
                # No disk I/O or VISA operations hold the scheduler-facing lock.
                self._writer.writerow(row)
                self._file.flush()
                previous_start = start
                if failure:
                    break  # Never send a next query into a desynchronized stream.
                if row["valid"]:
                    if previous_valid_value is None or row["delta_time_ps"] != previous_valid_value:
                        last_distinct_time = row["query_end_monotonic_s"]
                    previous_valid_value = row["delta_time_ps"]
                previous_valid = bool(row["valid"])
                if (self.stale_after_s is not None and last_distinct_time is not None
                        and row["query_end_monotonic_s"] - last_distinct_time >= self.stale_after_s):
                    self.error = f"remote_value_frozen_for_{self.stale_after_s:g}s"
                    break
                remaining = self.interval_s - (self.scope.monotonic() - start)
                if remaining > 0:
                    self._stop.wait(remaining)
        except Exception as exc:
            self.error = f"collector_failure: {type(exc).__name__}: {exc}"
        finally:
            self._ended = self.scope.monotonic()
            try:
                self._file.flush()
                self._file.close()
            except Exception as exc:
                self.error = f"{self.error or ''}; csv_close_failure: {exc}"
            with self._lock:
                self._io_finished = True
                close_scope = self._close_when_stopped and not self._scope_close_claimed
                if close_scope:
                    self._scope_close_claimed = True
            if close_scope:
                self.scope.close()
            self._done.set()

    def stop(self, timeout_s: float | None = None) -> None:
        self._stop.set()
        if self._thread is None:
            return
        self._thread.join(self.scope.timeout_ms / 1000 + 1.0 if timeout_s is None else timeout_s)
        if self._thread.is_alive():
            self.error = "scope reader did not stop within its VISA timeout"
            self.scope.transport_desynchronized = True
            raise TimeoutError(self.error)

    def close_scope_when_stopped(self) -> None:
        """Defer all VISA cleanup until the reader leaves its I/O section.

        Use after stop() timeout instead of calling scope.close() on the calling
        thread. If a broken VISA backend never returns, the manifest explicitly
        records pending cleanup; no competing write/close is sent to that session.
        """
        self._stop.set()
        with self._lock:
            self._close_when_stopped = True
            if self._thread is None:
                self._io_finished = True
            close_scope = self._io_finished and not self._scope_close_claimed
            if close_scope:
                self._scope_close_claimed = True
        if close_scope:
            self.scope.close()

    def wait(self, timeout_s: float) -> bool:
        """Wait for reader termination, or timeout without stopping it."""
        return self._done.wait(timeout_s)

    def summary(self) -> dict[str, Any]:
        with self._lock:
            rows = list(self._rows)
            baseline = dict(self._baseline) if self._baseline else None
        finish = self._ended if self._ended is not None else self.scope.monotonic()
        duration = max(0.0, finish - self._started) if self._started is not None else 0.0
        valid = sum(bool(row["valid"]) for row in rows)
        invalid_reasons: dict[str, int] = {}
        for row in rows:
            if not row["valid"]:
                invalid_reasons[row["reason"]] = invalid_reasons.get(row["reason"], 0) + 1
        sample_counts = [row["result_n_samples"] for row in rows
                         if row.get("result_n_samples") is not None]
        sample_deltas = [later - earlier for earlier, later in zip(sample_counts, sample_counts[1:])]
        value_changes = sum(earlier["valid"] and later["valid"]
                            and earlier["delta_time_ps"] != later["delta_time_ps"]
                            for earlier, later in zip(rows, rows[1:]))
        return {
            "query_count": len(rows), "valid_count": valid, "elapsed_s": duration,
            "queries_per_second": len(rows) / duration if duration else None,
            "valid_results_per_second": valid / duration if duration else None,
            "valid_result_fraction": valid / len(rows) if rows else None,
            "query_duration_s": distribution([row["query_duration_s"] for row in rows]),
            "query_start_interval_s": distribution([row["query_start_interval_s"] for row in rows
                                                     if row["query_start_interval_s"] is not None]),
            "requested_min_start_interval_s": self.interval_s,
            "query_mode": self.query_mode, "stale_after_s": self.stale_after_s,
            "observed_result_value_change_count": value_changes,
            "observed_result_value_changes_per_second": value_changes / duration if duration else None,
            "observed_result_change_is_new_acquisition": False,
            "invalid_reasons": invalid_reasons, "baseline": baseline,
            "csv_path": str(self.csv_path), "error": self.error,
            "collector_thread_alive": bool(self._thread and self._thread.is_alive()),
            "scope_close_pending": self._close_when_stopped and not self.scope.closed,
            "timestamp_semantics": TIMESTAMP_SEMANTICS,
            "new_data_update_rate_hz": None, "new_data_20hz_verified": False,
            "new_measurement_per_50ms_coverage": None,
            "freshness_status": "UNVERIFIED: no validated acquisition identifier; query rate is not new-data rate",
            "measurement_results_n_samples_is_acquisition_id": False,
            "measurement_results_n_samples_observation": {
                "first": sample_counts[0] if sample_counts else None,
                "last": sample_counts[-1] if sample_counts else None,
                "positive_step_count": sum(delta > 0 for delta in sample_deltas),
                "total_positive_change": sum(delta for delta in sample_deltas if delta > 0),
                "nonpositive_step_count": sum(delta <= 0 for delta in sample_deltas),
                "semantics": "displayed statistical n-samples only; not a validated acquisition identifier",
            },
            "phase_deg_per_ps": PHASE_DEG_PER_PS,
            "scope": self.scope.metadata(),
        }


def run_rate_check(address: str, output_dir: str | Path, duration_s: float = 10.0,
                   interval_s: float = DEFAULT_SCOPE_INTERVAL_S, timeout_ms: int = 2000,
                   resource_manager: Any = None, visa_library: str | None = None,
                   visa_dll_dir: str | None = None) -> dict[str, Any]:
    """Scope-only read test. Never imports SSH or connects the RF source/board."""
    if not math.isfinite(duration_s) or duration_s <= 0:
        raise ValueError("rate-check duration must be positive and finite")
    directory = Path(output_dir)
    directory.mkdir(parents=True, exist_ok=True)
    report_path = directory / "scope_rate_check.json"
    if report_path.exists() or (directory / "scope_delta_time.csv").exists():
        raise FileExistsError("Use a new rate-check output directory; previous evidence is preserved")
    scope = None
    collector = None
    report: dict[str, Any] = {"mode": "scope_rate_check", "requested_duration_s": duration_s,
                              "scope_address": address, "visa_library": visa_library or "auto"}
    pending: BaseException | None = None
    close_deferred = False
    try:
        scope = Scope86100C.connect(address, timeout_ms=timeout_ms, resource_manager=resource_manager,
                                    visa_library=visa_library, visa_dll_dir=visa_dll_dir)
        collector = ScopeCollector(scope, directory / "scope_delta_time.csv", interval_s).start()
        collector.wait(duration_s)
    except BaseException as exc:
        pending = exc
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if collector is not None:
            try:
                collector.stop()
            except Exception as exc:
                pending = pending or exc
                report["error"] = f"{type(exc).__name__}: {exc}"
                close_deferred = True
                collector.close_scope_when_stopped()
        if scope is not None and not close_deferred:
            scope.close()
        if collector is not None:
            report.update(collector.summary())
        if pending is not None:
            report["error"] = f"{type(pending).__name__}: {pending}"
        with report_path.open("x", encoding="utf-8") as handle:
            json.dump(report, handle, ensure_ascii=False, indent=2, allow_nan=False)
    if pending is not None:
        raise pending
    return report
