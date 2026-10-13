import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest
from tools.experiment_plan import full_plan

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import analyze_out_of_loop as analysis


class AnalysisTests(unittest.TestCase):
    def test_pi_batch_keeps_three_independent_repeats(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runs = []
            for index in range(3):
                run = dict(run_id=f"run{index}", condition="PI",
                           block=index + 1, status="completed", run_dir=f"run{index}")
                runs.append(run)
                path = root / run["run_dir"]
                path.mkdir()
                events = [dict(event="STATE2_RECEIVED", monotonic_s=100, command_stage="marker", status="ok")]
                for event in full_plan():
                    t = 100 + event["planned_s"]
                    events.append(dict(event=event["event"], planned_s=event["planned_s"],
                                       monotonic_s=t, command_start_monotonic_s=t,
                                       command_end_monotonic_s=t + .001,
                                       command_stage="after" if event["apply_frequency"] else "marker", status="ok"))
                analysis.write_csv(path / "events.csv", events,
                    ["event", "planned_s", "monotonic_s", "command_stage", "status",
                     "command_start_monotonic_s", "command_end_monotonic_s"])
                rows = [dict(query_start_monotonic_s=100 + i / 20,
                             query_end_monotonic_s=100.001 + i / 20,
                             delta_time_ps=10 + (i % 2) * .2, valid=True) for i in range(3311)]
                analysis.write_csv(path / "scope_delta_time.csv", rows, list(rows[0]))
                (path / "run_manifest.json").write_text(json.dumps({"health": {"eligible_for_analysis": True}}))
            (root / "batch_manifest.json").write_text(json.dumps({"runs": runs, "recovery_threshold_ps": None}))
            summary = analysis.analyze_batch(root)
            self.assertEqual(len(summary), 20 * len(analysis.METRICS))
            self.assertTrue(all(row["independent_runs"] == 3 for row in summary))
            self.assertTrue(all(row["runs_with_metric"] == 0 for row in summary
                                if row["metric"] == "recovery_s_query_estimate"))
            self.assertTrue((root / "run0" / "scope_curve.svg").exists())

    def test_no_threshold_never_invents_recovery(self):
        result = analysis.metric_row("step", "step", 0, 2, True, [(0, 10), (1, 0)], None, .5, .1)
        self.assertIsNone(result["recovery_s_query_estimate"])
        self.assertEqual(result["recovery_status"], "threshold_not_set")
        self.assertEqual(result["peak_abs_delay_ps"], 10)

    def test_invalid_query_breaks_recovery_and_long_gap_does_not_confirm(self):
        self.assertIsNone(analysis.recovery([(0, 10), (.1, 0), (1, 0)], 0, 1, .5, .1)[0])
        samples = [(0, 10), (.1, 0), (.2, None), (.3, 0), (.4, 0)]
        self.assertIsNone(analysis.recovery(samples, 0, 1, .2, .11)[0])
        result = analysis.recovery([(0, 10), (.1, 0), (.2, 0), (.3, 0)], 0, 1, .2, .11)
        self.assertAlmostEqual(result[0], .1)
        self.assertIn("unverified", result[1])

    def test_unresolved_excursion_is_not_zero_recovery(self):
        value, status = analysis.recovery([(i / 20, 0) for i in range(20)], 0, 1, .5, .1)
        self.assertIsNone(value)
        self.assertIn("no_excursion", status)

    def test_delayed_excursion_is_still_detected(self):
        samples = [(0, 0), (.1, 0), (.2, 0), (.3, 5), (.4, 0), (.5, 0), (.6, 0)]
        value, status = analysis.recovery(samples, 0, 1, .2, .11)
        self.assertAlmostEqual(value, .4)
        self.assertIn("unverified", status)

    def test_events_do_not_inflate_independent_n_and_invalid_runs_excluded(self):
        rows = []
        for run in range(3):
            for event in ("pulse1", "pulse2", "pulse3", "pulse4", "pulse5"):
                rows.append(dict(run_id=str(run), condition="PI", event_key=event,
                                 eligible_for_analysis=run < 2, rms_delay_ps=run + 1))
        summary = analysis.aggregate(rows)
        self.assertTrue(all(row["independent_runs"] == 2 for row in summary))
        self.assertEqual(summary[0]["runs_with_metric"], 0)  # first metric absent
        rms = next(row for row in summary if row["metric"] == "rms_delay_ps")
        self.assertEqual(rms["mean"], 1.5)
        with self.assertRaisesRegex(ValueError, "duplicate"):
            analysis.aggregate(rows + [rows[0]])

    def test_rms_is_delay_residual_not_equivalent_mv(self):
        row = analysis.metric_row("steady", "steady", 0, 10, False, [(1, 3), (2, 4)], None, .5, .1)
        self.assertAlmostEqual(row["rms_phase_deg"], row["rms_delay_ps"] * .72)
        self.assertEqual(row["mae_delay_ps"], 3.5)
        self.assertEqual(row["centered_rms_delay_ps"], .5)

    def test_baseline_ends_before_source_write_and_checks_saved_baseline(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            events = [
                dict(event="STATE2_RECEIVED", monotonic_s=100,
                     command_stage="marker", status="ok"),
                dict(event="STEP_+50.0mV", planned_s=10, monotonic_s=112,
                     command_start_monotonic_s=110, command_end_monotonic_s=112,
                     command_stage="after", status="ok"),
                dict(event="STEP_RECOVERY", planned_s=25, monotonic_s=125.01,
                     command_start_monotonic_s=125, command_end_monotonic_s=125.01,
                     command_stage="after", status="ok"),
                dict(event="POST_WAIT_DONE", planned_s=30, monotonic_s=130,
                     command_stage="marker", status="ok"),
            ]
            analysis.write_csv(root / "events.csv", events,
                               ["event", "planned_s", "monotonic_s", "command_start_monotonic_s",
                                "command_end_monotonic_s", "command_stage", "status"])
            rows = [dict(query_start_monotonic_s=start, query_end_monotonic_s=end,
                         delta_time_ps=value, valid=True)
                    for start, end, value in [(101, 101.1, 10), (102, 102.1, 12),
                                               (109, 109.1, 30), (110.1, 110.2, 999),
                                               (113, 113.1, 41)]]
            analysis.write_csv(root / "scope_delta_time.csv", rows, list(rows[0]))
            run = dict(run_id="baseline-test", condition="PI", block=1, status="completed")
            cases = [
                # The 110.1--110.2 query finishes before the write midpoint 111,
                # but after sending began, so it must never enter the baseline.
                ({}, 52 / 3, 3, False),
                # The runner can freeze the baseline before the first send.
                ({"cutoff_monotonic_s": 105, "baseline_ps": 11}, 11, 2, False),
                # A later saved cutoff cannot relax the command-start bound.
                ({"cutoff_monotonic_s": 113, "baseline_ps": 52 / 3}, 52 / 3, 3, False),
                ({"cutoff_monotonic_s": 105, "baseline_ps": 99}, 11, 2, True),
            ]
            for saved, expected, count, mismatch in cases:
                with self.subTest(saved_baseline=saved):
                    (root / "run_manifest.json").write_text(json.dumps({
                        "baseline": saved, "health": {"eligible_for_analysis": True}}))
                    results, diagnostic = analysis.analyze_run(root, run, {})
                    self.assertAlmostEqual(diagnostic["baseline_ps"], expected)
                    self.assertEqual(diagnostic["baseline_valid_queries"], count)
                    self.assertEqual("saved_baseline_mismatch" in diagnostic["reasons"], mismatch)
                    self.assertEqual(diagnostic["eligible_for_analysis"], not mismatch)
                    self.assertEqual(results[0]["eligible_for_analysis"], not mismatch)
                    self.assertAlmostEqual(results[0]["peak_abs_delay_ps"], 41 - expected)

    def test_failed_partial_run_still_writes_curve_without_analysis(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            events = [dict(event="STATE2_RECEIVED", monotonic_s=100, command_stage="marker"),
                      dict(event="STEP_+50.0mV", monotonic_s=110, planned_s=10, command_stage="after", status="ok"),
                      dict(event="STEP_RECOVERY", monotonic_s=125, planned_s=25, command_stage="after", status="ok")]
            analysis.write_csv(root / "events.csv", events,
                               ["event", "monotonic_s", "planned_s", "command_stage", "status"])
            rows = [dict(query_start_monotonic_s=t, query_end_monotonic_s=t + .01,
                         delta_time_ps=value, valid=valid) for t, value, valid in
                    [(101, 10, True), (102, 12, True), (111, 15, True), (112, 999, False)]]
            analysis.write_csv(root / "scope_delta_time.csv", rows, list(rows[0]))
            (root / "run_manifest.json").write_text(json.dumps({"health": {"eligible_for_analysis": False}}))
            result, diagnostic = analysis.analyze_run(root, dict(run_id="a", condition="PI", block=1, status="failed"), {})
            self.assertEqual(diagnostic["baseline_ps"], 11)
            self.assertEqual(result[0]["peak_abs_delay_ps"], 4)
            self.assertFalse(diagnostic["eligible_for_analysis"])
            self.assertEqual(analysis.aggregate(result), [])
            self.assertTrue((root / "scope_curve.svg").exists())


if __name__ == "__main__": unittest.main()
