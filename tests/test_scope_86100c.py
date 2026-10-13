"""Offline only: simulated VISA, bounded threads, no pyvisa/SSH import needed."""

import csv
import itertools
import json
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from scope_86100c import (DELTA_QUERY, Scope86100C, ScopeCollector,
                         parse_delta_time, parse_display_delta_time, run_rate_check)


class Visa:
    def __init__(self, values=None, sendvalid="0", idn="Agilent Technologies,86100C,SYNTHETIC,A.10.80"):
        self.values = list(values or ["1e-12,0"])
        self.sendvalid = sendvalid
        self.idn = idn
        self.commands = []
        self.closed = False
        self.read_count = 0
        self.hold = None
        self.entered = threading.Event()
        self.fail_restore = False

    def query(self, command):
        self.commands.append(command)
        if command == "*IDN?":
            return self.idn
        if command == ":MEASure:SENDvalid?":
            return self.sendvalid
        if command != DELTA_QUERY:
            raise AssertionError(f"Unexpected command: {command}")
        self.entered.set()
        if self.hold is not None:
            if not self.hold.wait(1):
                raise TimeoutError("simulated VISA timeout")
        time.sleep(.001)
        index = min(self.read_count, len(self.values) - 1)
        self.read_count += 1
        value = self.values[index]
        if isinstance(value, BaseException):
            raise value
        if "Time(m1)-(2)" in value:
            return value
        parts = value.split(",")
        status = parts[1] if len(parts) > 1 else "0"
        return ("V p-p(2),0.1,0,0.1,0.1,0.1,0.0,10,"
                f"\xb4Time(m1)-(2),{parts[0]},{status},1e-12,3e-12,2e-12,0,{self.read_count}")

    def write(self, command):
        self.commands.append(command)
        if self.fail_restore and command == ":MEASure:SENDvalid 0":
            raise OSError("simulated restore failure")

    def close(self):
        self.closed = True


class Manager:
    def __init__(self, visa):
        self.visa = visa
        self.addresses = []
        self.closed = False

    def open_resource(self, address):
        self.addresses.append(address)
        return self.visa

    def close(self):
        self.closed = True


def connect(visa, **kwargs):
    return Scope86100C.connect("TCPIP0::mock::gpib0,7::INSTR", timeout_ms=100,
                               resource_manager=Manager(visa), **kwargs)


class ParseTests(unittest.TestCase):
    def test_displayed_legacy_result_table_selects_m1_ch2(self):
        raw = ("V p-p(2),0.11,0,0.1,0.12,0.11,0.01,99,"
               "\xb4Time(m1)-(2),2.57E-011,0,1e-11,3e-11,2e-11,1e-12,123")
        result = parse_display_delta_time(raw)
        self.assertTrue(result.valid)
        self.assertEqual(result.delta_time_ps, 25.7)
        self.assertEqual(result.result_n_samples, 123)
        self.assertEqual(result.result_name, "\xb4Time(m1)-(2)")
        self.assertEqual(result.raw_response, raw)

    def test_displayed_row_missing_ambiguous_and_invalid(self):
        row = "\xb4Time(m1)-(2),9.999E+37,0,1,2,3,4,5"
        self.assertEqual(parse_display_delta_time(row).reason, "invalid_sentinel")
        self.assertEqual(parse_display_delta_time("V p-p(2),0.1,0,0,0,0,0,1").reason,
                         "measurement_not_displayed")
        self.assertEqual(parse_display_delta_time(row + "," + row).reason,
                         "ambiguous_measurement")
        self.assertEqual(parse_display_delta_time("\xb4Time(m1)-(2),2e-11,0").reason,
                         "malformed_result_table")
        self.assertEqual(parse_display_delta_time(
            "\xb4Time(m1)-(2),2e-11,5,0,0,0,0,1").reason, "edge_missing")

    def test_seconds_ps_and_status(self):
        value = parse_delta_time("-1.25e-12,0\n")
        self.assertTrue(value.valid)
        self.assertEqual(value.delta_time_ps, -1.25)
        self.assertEqual(value.raw_response, "-1.25e-12,0\n")

    def test_header_forms_without_changing_scope_header(self):
        for header in (":MEASURE:DELTATIME", "MEAS:DELT", ":Meas:DeltaTime"):
            self.assertTrue(parse_delta_time(f"{header} 2E-12,0").valid)

    def test_scalar_retains_missing_status(self):
        value = parse_delta_time("4e-12")
        self.assertTrue(value.valid)
        self.assertIsNone(value.status_code)
        self.assertEqual(value.reason, "status_unavailable")

    def test_all_nonzero_codes_excluded(self):
        for code in range(1, 41):
            with self.subTest(code=code):
                value = parse_delta_time(f"5e-12,{code}")
                self.assertFalse(value.valid)
                self.assertEqual(value.status_code, code)
                self.assertEqual(value.delta_time_ps, 5.)
        self.assertEqual(parse_delta_time("5e-12,999").reason, "unknown_status")

    def test_sentinel_variants_rejected(self):
        for text in ("9.999E+37", "-9.99999E+37", "+9.999E37,0", "9.99999e37,1"):
            self.assertEqual(parse_delta_time(text).reason, "invalid_sentinel")
            self.assertIsNone(parse_delta_time(text).delta_time_ps)

    def test_nonfinite_rejected(self):
        for text in ("nan", "NaN,0", "inf,0", "-inf", "1e999,0"):
            self.assertEqual(parse_delta_time(text).reason, "non_finite_value")

    def test_malformed_not_heuristically_parsed(self):
        for text in ("", "garbage 1e-12", "1e-12,0,3", "1e-12,CORR", "1e-12,0.0",
                     "1e-12,", ":MEAS:VPP 1e-12", "1e-12 seconds"):
            self.assertFalse(parse_delta_time(text).valid, text)


class SessionTests(unittest.TestCase):
    def test_only_idn_sendvalid_and_delta_commands(self):
        visa = Visa()
        with connect(visa) as scope:
            self.assertEqual(visa.commands[0], "*IDN?")
            self.assertEqual(visa.timeout, 100)
            result = scope.query_delta_time()
            self.assertTrue(result["valid"])
            self.assertLessEqual(result["query_start_monotonic_s"], result["query_end_monotonic_s"])
            self.assertIn("T", result["query_start_local"])
        self.assertEqual(visa.commands, ["*IDN?", ":MEASure:SENDvalid?",
                                         ":MEASure:SENDvalid ON", DELTA_QUERY,
                                         ":MEASure:SENDvalid 0"])
        self.assertTrue(visa.closed)
        self.assertFalse(scope.resource_manager.closed)  # Injected manager stays caller-owned.

    def test_already_sendvalid_on_no_writes(self):
        visa = Visa(sendvalid=":MEAS:SEND 1")
        scope = connect(visa)
        scope.close()
        scope.close()
        self.assertEqual(visa.commands, ["*IDN?", ":MEASure:SENDvalid?"])

    def test_wrong_instrument_closes_without_mutation(self):
        visa = Visa(idn="Keysight,N1000A,123,A.01")
        with self.assertRaisesRegex(ValueError, "Expected 86100C"):
            connect(visa)
        self.assertEqual(visa.commands, ["*IDN?"])
        self.assertTrue(visa.closed)

    def test_unknown_sendvalid_closes_without_writes(self):
        visa = Visa(sendvalid="banana")
        with self.assertRaisesRegex(ValueError, "SENDvalid"):
            connect(visa)
        self.assertEqual(len(visa.commands), 2)
        self.assertTrue(visa.closed)

    def test_timeout_poison_prevents_late_reply_reuse(self):
        visa = Visa([TimeoutError("VI_ERROR_TMO"), "44e-12,0"])
        scope = connect(visa)
        with self.assertRaises(TimeoutError) as failed:
            scope.query_delta_time()
        self.assertIn("query_end_monotonic_s", failed.exception.scope_transaction)
        with self.assertRaisesRegex(RuntimeError, "transport_desynchronized"):
            scope.query_delta_time()
        self.assertEqual(visa.read_count, 1)
        scope.close()

    def test_restore_failure_recorded_and_resource_closed(self):
        visa = Visa()
        scope = connect(visa)
        visa.fail_restore = True
        scope.close()
        self.assertTrue(visa.closed)
        self.assertIn("restoration failed", scope.metadata()["close_errors"][0])


class CollectorTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name) / "scope_delta_time.csv"

    def tearDown(self):
        self.temp.cleanup()

    def wait_for_rows(self, collector, count):
        deadline = time.monotonic() + 1
        while collector.summary()["query_count"] < count:
            if collector.error:
                self.fail(collector.error)
            if time.monotonic() > deadline:
                self.fail("mock did not produce expected rows")
            time.sleep(.001)

    def read(self):
        with self.path.open(encoding="utf-8", newline="") as handle:
            return list(csv.DictReader(handle))

    def test_stream_baseline_relative_phase_and_query_time_semantics(self):
        visa = Visa(["1e-12,0", "3e-12,0", "5e-12,0"])
        with connect(visa) as scope:
            collector = ScopeCollector(scope, self.path, interval_s=.05)
            collector.mark_state2(time.monotonic())
            collector.start()
            self.wait_for_rows(collector, 2)
            baseline = collector.freeze_baseline(time.monotonic())
            self.assertEqual(baseline["sample_count"], 2)
            self.assertEqual(baseline["query_indices"], [1, 2])
            self.assertEqual(baseline["baseline_ps"], 2.)
            self.wait_for_rows(collector, 3)
            collector.stop()
        rows = self.read()
        self.assertEqual(rows[0]["relative_delay_ps"], "")
        self.assertEqual(float(rows[2]["delta_time_ps"]), 5.)
        self.assertEqual(float(rows[2]["relative_delay_ps"]), 3.)
        self.assertAlmostEqual(float(rows[2]["relative_phase_deg"]), 2.16)
        self.assertEqual(collector.freeze_baseline(time.monotonic()), baseline)

    def test_slow_query_does_not_block_baseline_or_scheduler(self):
        visa = Visa()
        visa.hold = threading.Event()
        with connect(visa) as scope:
            collector = ScopeCollector(scope, self.path).start()
            self.assertTrue(visa.entered.wait(.5))
            started = time.monotonic()
            collector.mark_state2(started)
            baseline = collector.freeze_baseline(started)
            self.assertLess(time.monotonic() - started, .05)
            self.assertIsNone(baseline["baseline_ps"])
            self.assertEqual(baseline["query_indices"], [])
            visa.hold.set()
            self.wait_for_rows(collector, 1)
            collector.stop()
        self.assertTrue(all(row["relative_delay_ps"] == "" for row in self.read()))

    def test_bad_values_retained_then_transport_timeout_preserves_partial(self):
        visa = Visa(["1e-12,0", "9.999E37", "2e-12,5", TimeoutError("timed out")])
        ticks = itertools.count()
        with connect(visa, monotonic=lambda: next(ticks) * .001) as scope:
            collector = ScopeCollector(scope, self.path).start()
            self.assertTrue(collector.wait(1))
            collector.stop()
        rows = self.read()
        self.assertEqual(len(rows), 4)
        self.assertIn("9.999E37", rows[1]["raw_response"])
        self.assertEqual(rows[2]["reason"], "edge_missing")
        self.assertIn("transport_desynchronized", rows[3]["reason"])
        self.assertEqual(visa.read_count, 4)
        summary = collector.summary()
        self.assertEqual(summary["query_count"], 4)
        self.assertEqual(summary["valid_count"], 1)
        self.assertEqual(summary["valid_result_fraction"], .25)
        self.assertGreater(summary["queries_per_second"], 0)
        self.assertEqual(summary["query_start_interval_s"]["n"], 3)
        self.assertEqual(summary["query_duration_s"]["n"], 4)

    def test_duplicate_values_never_claim_fresh_acquisition(self):
        with connect(Visa(["1e-12,0"])) as scope:
            collector = ScopeCollector(scope, self.path, interval_s=.005).start()
            self.wait_for_rows(collector, 4)
            collector.stop()
        summary = collector.summary()
        self.assertGreater(summary["queries_per_second"], 20)
        self.assertFalse(summary["new_data_20hz_verified"])
        self.assertIsNone(summary["new_data_update_rate_hz"])
        self.assertIsNone(summary["new_measurement_per_50ms_coverage"])

    def test_does_not_overwrite_evidence(self):
        self.path.write_text("retained", encoding="utf-8")
        with connect(Visa()) as scope:
            with self.assertRaises(FileExistsError):
                ScopeCollector(scope, self.path).start()
        self.assertEqual(self.path.read_text(), "retained")

    def test_scope_only_rate_check_preserves_report_and_no_second_run_overwrite(self):
        visa = Visa(["2e-12,0", "9.999E37", "2e-12,0"])
        result = run_rate_check("mock", self.temp.name, duration_s=.035,
                                interval_s=.005, resource_manager=Manager(visa))
        self.assertGreaterEqual(result["query_count"], 3)
        self.assertTrue(visa.closed)
        saved = json.loads((Path(self.temp.name) / "scope_rate_check.json").read_text())
        self.assertEqual(saved["query_count"], result["query_count"])
        self.assertTrue(saved["scope"]["closed"])
        with self.assertRaises(FileExistsError):
            run_rate_check("mock", self.temp.name, resource_manager=Manager(Visa()))

    def test_backend_ignoring_timeout_defers_all_close_until_reader_stops(self):
        visa = Visa()
        visa.hold = threading.Event()
        scope = connect(visa)
        collector = ScopeCollector(scope, self.path).start()
        self.assertTrue(visa.entered.wait(.5))
        with self.assertRaises(TimeoutError):
            collector.stop(timeout_s=.001)
        collector.close_scope_when_stopped()
        self.assertNotIn(":MEASure:SENDvalid 0", visa.commands)
        self.assertFalse(visa.closed)
        self.assertTrue(collector.summary()["scope_close_pending"])
        visa.hold.set()
        self.assertTrue(collector.wait(.5))
        self.assertTrue(visa.closed)
        self.assertFalse(collector.summary()["scope_close_pending"])
        self.assertEqual(visa.commands.count(":MEASure:SENDvalid 0"), 1)
        collector.close_scope_when_stopped()
        self.assertEqual(visa.commands.count(":MEASure:SENDvalid 0"), 1)

    def test_deferred_close_after_reader_already_finished_closes_once(self):
        visa = Visa([TimeoutError("timed out")])
        scope = connect(visa)
        collector = ScopeCollector(scope, self.path).start()
        self.assertTrue(collector.wait(.5))
        collector.close_scope_when_stopped()
        collector.close_scope_when_stopped()
        self.assertTrue(visa.closed)
        self.assertEqual(visa.commands.count(":MEASure:SENDvalid 0"), 1)

    def test_keyboard_interrupt_saves_rate_report_and_closes_scope(self):
        visa = Visa()
        with patch.object(ScopeCollector, "wait", side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                run_rate_check("mock", self.temp.name, resource_manager=Manager(visa))
        self.assertTrue(visa.closed)
        saved = json.loads((Path(self.temp.name) / "scope_rate_check.json").read_text())
        self.assertIn("KeyboardInterrupt", saved["error"])


if __name__ == "__main__":
    unittest.main()
