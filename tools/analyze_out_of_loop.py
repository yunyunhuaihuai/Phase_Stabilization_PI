#!/usr/bin/env python3
"""Offline 86100C query-domain results; independent unit = one controller run.

No V_PHS values enter these metrics. Query midpoints are plotting/analysis
approximations, not hardware acquisition timestamps or evidence of freshness.
"""
import argparse
import csv
import hashlib
import html
import json
import math
from pathlib import Path
import statistics

DEG_PER_PS = 0.72
METRICS = ("peak_abs_delay_ps", "rms_delay_ps", "mae_delay_ps",
           "rms_phase_deg", "centered_rms_delay_ps", "recovery_s_query_estimate")


def finite(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (ValueError, TypeError):
        return None


def read_csv(path):
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_csv(path, rows, fields):
    with Path(path).open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def event_windows(events):
    """Use successful, actually transmitted command times, never scheduled times."""
    actual = []
    for row in events:
        if row.get("command_stage") not in ("after", "marker"):
            continue
        if row.get("status") not in ("", "ok", "success", "sent", "recorded", "completed"):
            continue
        planned = finite(row.get("planned_s"))
        start = finite(row.get("command_start_monotonic_s"))
        end = finite(row.get("command_end_monotonic_s"))
        t = (start + end) / 2 if start is not None and end is not None else finite(row.get("monotonic_s"))
        if planned is not None and t is not None:
            actual.append(dict(row, t=t, planned=planned))
    actual.sort(key=lambda row: row["planned"])
    windows = []
    last_step = None
    for i, row in enumerate(actual[:-1]):
        name, t, end = row["event"], row["t"], actual[i + 1]["t"]
        if name.startswith("STEP_"):
            if name == "STEP_RECOVERY":
                key = f"{last_step}_withdrawal"
            else:
                last_step = name
                key = f"{name}_application"
            windows.append((key, "step", t, end, True))
        elif name.startswith("PULSE_") and name.endswith("_ON"):
            # Five within-run responses remain five endpoints, never five repeats.
            next_on = next((r["t"] for r in actual[i + 1:] if r["event"].endswith("_ON")
                            or r["event"].startswith("MODULE_3")), end)
            windows.append((name.replace("_ON", "_response"), "pulse", t, next_on, False))
        elif name.startswith("PULSE_") and name.endswith("_OFF"):
            windows.append((name.replace("_OFF", "_withdrawal"), "pulse", t, end, True))
        elif name == "RAMP_RECOVERY":
            windows.append(("RAMP_withdrawal", "ramp", t, end, True))
        elif name == "INTERFERENCE_DONE":
            windows.append(("STEADY_final", "steady", t, end, False))
    ramp = [r for r in actual if r["event"].startswith("RAMP_") and r["event"] != "RAMP_RECOVERY"]
    release = next((r for r in actual if r["event"] == "RAMP_RECOVERY"), None)
    if ramp and release:
        # v5 increments to 250 mV at t=135.4, then waits 0.1 + 5 s.
        hold_start = ramp[-1]["t"] + 0.1
        windows += [("RAMP_tracking", "ramp", ramp[0]["t"], hold_start, False),
                    ("RAMP_hold", "ramp", hold_start, release["t"], False)]
    return windows, actual


def recovery(samples, start, threshold, hold, max_gap):
    """Observed-query estimate. Invalid rows/gaps break sustained evidence."""
    if threshold is None:
        return None, "threshold_not_set"
    first = previous = None
    saw_excursion = False
    for t, value in samples:
        if value is None or abs(value) > threshold:
            if value is not None:
                saw_excursion = True
            first = previous = None
            continue
        if previous is None or t - previous > max_gap:
            first = t
        previous = t
        if t - first >= hold - 1e-9:
            if saw_excursion:
                return max(0.0, first - start), "query_estimate_freshness_unverified"
    if not saw_excursion:
        return None, "no_excursion_resolved_by_queries"
    return None, "not_confirmed_in_observation_window"


def metric_row(key, module, start, end, recoverable, samples, threshold, hold, max_gap):
    window = [(t, v) for t, v in samples if start <= t < end]
    values = [v for _, v in window if v is not None]
    result = dict(event_key=key, module=module, window_start_monotonic_s=start,
                  window_end_monotonic_s=end, valid_queries=len(values),
                  recovery_threshold_ps=threshold, recovery_hold_s=hold,
                  recovery_max_gap_s=max_gap)
    result.update({name: None for name in METRICS})
    if values:
        mean = statistics.fmean(values)
        rms = math.sqrt(statistics.fmean(v * v for v in values))
        result.update(peak_abs_delay_ps=max(map(abs, values)), rms_delay_ps=rms,
                      mae_delay_ps=statistics.fmean(map(abs, values)),
                      rms_phase_deg=rms * DEG_PER_PS,
                      centered_rms_delay_ps=math.sqrt(statistics.fmean((v - mean) ** 2 for v in values)))
    value, status = recovery(window, start, threshold, hold, max_gap) if recoverable else (None, "not_applicable")
    if not window:
        status = "no_queries_in_window"
    result.update(recovery_s_query_estimate=value, recovery_status=status)
    return result


def plot_queries(path, rows, baseline, state2, events):
    """Standalone vector plot, preserving gaps and all valid raw query points."""
    points = []
    for row in rows:
        a, b, y = (finite(row.get(k)) for k in
                   ("query_start_monotonic_s", "query_end_monotonic_s", "delta_time_ps"))
        if a is None or b is None:
            continue
        valid = str(row.get("valid", "")).lower() in ("1", "true") and y is not None
        points.append(((a + b) / 2 - state2, y - baseline if valid and baseline is not None else y if valid else None))
    usable = [(t, y) for t, y in points if y is not None]
    x0, x1 = (min(t for t, _ in points), max(t for t, _ in points)) if points else (0, 1)
    y0, y1 = (min(y for _, y in usable), max(y for _, y in usable)) if usable else (-1, 1)
    if x0 == x1: x1 += 1
    pad = max((y1 - y0) * .08, .01)
    y0, y1 = y0 - pad, y1 + pad
    xx = lambda t: 75 + (t - x0) / (x1 - x0) * 1050
    yy = lambda y: 340 - (y - y0) / (y1 - y0) * 300
    fragments = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1200 430">',
                 '<rect width="1200" height="430" fill="white"/>',
                 '<path d="M75 40V340H1125" stroke="#333" fill="none"/>']
    for i in range(6):
        t = x0 + (x1 - x0) * i / 5
        y = y0 + (y1 - y0) * i / 5
        fragments += [f'<text x="{xx(t):.1f}" y="363" text-anchor="middle">{t:.2f}</text>',
                      f'<text x="68" y="{yy(y):.1f}" text-anchor="end">{y:.3g}</text>']
    for row in events:
        t = row["t"] - state2
        if x0 <= t <= x1 and not row["event"].startswith("RAMP_"):
            fragments.append(f'<path d="M{xx(t):.1f} 40V340" stroke="#ddd"><title>{html.escape(row["event"])}</title></path>')
    for t, y in usable:
        fragments.append(f'<circle cx="{xx(t):.2f}" cy="{yy(y):.2f}" r="1.2" fill="#2266aa"/>')
    label = "Relative delay (ps)" if baseline is not None else "Raw delta time (ps); baseline unavailable"
    fragments += [f'<text x="75" y="25">{label}</text>',
                  '<text x="600" y="396" text-anchor="middle">Seconds after laptop received State 2 (query midpoint approximation)</text>',
                  '<text x="75" y="423" font-size="13">Points are queries, not verified new acquisitions. Invalid responses omitted; query timestamps; no independent acquisition timestamp.</text>', '</svg>']
    Path(path).write_text("\n".join(fragments), encoding="utf-8")


def analyze_run(run_dir, run, policy):
    if run.get("condition") != "PI":
        raise ValueError("run condition must be PI")
    run_dir = Path(run_dir)
    scope_path, event_path = run_dir / "scope_delta_time.csv", run_dir / "events.csv"
    if policy.get("scope_capture") == "local":
        native = run.get("scope_native_files", [])
        mismatches = []
        for record in native:
            path = Path(record["file"])
            if (not path.is_file() or path.resolve().parent != (run_dir / "scope_native_export").resolve()
                    or file_sha256(path) != record["sha256"]):
                mismatches.append(str(path))
        diagnostic = dict(eligible_for_analysis=False,
                          reasons=["scope_native_timing_and_format_unverified"] +
                                  (["scope_native_file_hash_mismatch"] if mismatches else []) +
                                  (["no_scope_native_files_imported"] if not native else []),
                          native_file_count=len(native), mismatched_files=mismatches,
                          event_rows=len(read_csv(event_path)) if event_path.exists() else 0,
                          timestamp_semantics="native acquisition timestamps and laptop clock alignment not verified; no event-aligned metrics",
                          aggregation="one independent controller run; native scope records not yet interpreted")
        (run_dir / "analysis_diagnostics.json").write_text(json.dumps(diagnostic, ensure_ascii=False, indent=2), encoding="utf-8")
        return [], diagnostic
    if not scope_path.exists() or not event_path.exists():
        return [], {"eligible_for_analysis": False, "reasons": ["missing_scope_or_events"]}
    rows, events = read_csv(scope_path), read_csv(event_path)
    windows, actual = event_windows(events)
    state2 = next((finite(r.get("monotonic_s")) for r in events if r.get("event") == "STATE2_RECEIVED"), None)
    first_command = next((r for r in actual if r.get("command_stage") == "after"), None)
    first_injection = (finite(first_command.get("command_start_monotonic_s"))
                       if first_command else None)
    if first_injection is None and first_command:
        first_injection = first_command["t"]
    rm_path = run_dir / "run_manifest.json"
    rm = json.loads(rm_path.read_text(encoding="utf-8")) if rm_path.exists() else {}
    saved_baseline = rm.get("baseline", {})
    saved_cutoff = finite(saved_baseline.get("cutoff_monotonic_s"))
    baseline_indices = saved_baseline.get("query_indices")
    baseline_indices = set(baseline_indices) if baseline_indices is not None else None
    if first_injection is not None and saved_cutoff is not None:
        first_injection = min(first_injection, saved_cutoff)
    baseline_values = []
    parsed = []
    for row in rows:
        a, b, raw = (finite(row.get(k)) for k in
                     ("query_start_monotonic_s", "query_end_monotonic_s", "delta_time_ps"))
        valid = str(row.get("valid", "")).lower() in ("1", "true") and raw is not None
        if a is None or b is None or b < a:
            continue
        selected = baseline_indices is None or finite(row.get("query_index")) in baseline_indices
        if valid and selected and state2 is not None and first_injection is not None and state2 <= a <= b <= first_injection:
            baseline_values.append(raw)
        parsed.append(((a + b) / 2, raw if valid else None))
    baseline = statistics.fmean(baseline_values) if baseline_values else None
    samples = [(t, raw - baseline if raw is not None and baseline is not None else None) for t, raw in parsed]
    reasons = []
    if baseline is None: reasons.append("no_valid_pre_disturbance_baseline")
    if state2 is None: reasons.append("missing_state2_receive_time")
    if not any(r["event"] == "POST_WAIT_DONE" for r in actual): reasons.append("incomplete_event_plan")
    if run.get("status") != "completed": reasons.append("run_not_completed")
    stored_mean = finite(saved_baseline.get("baseline_ps"))
    if stored_mean is not None and (baseline is None or not math.isclose(stored_mean, baseline, abs_tol=1e-9)):
        reasons.append("saved_baseline_mismatch")
    if baseline_indices is not None and len(baseline_values) != len(baseline_indices):
        reasons.append("saved_baseline_query_set_incomplete")
    for name, digest in rm.get("evidence_sha256", {}).items():
        path = run_dir / name
        if path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            reasons.append("raw_evidence_hash_mismatch:" + name)
    # Fail closed when the orchestration has not established evidence health.
    health = rm.get("health", {})
    if not health.get("eligible_for_analysis", False): reasons.append("run_evidence_not_eligible")
    reasons.extend(health.get("reasons", []))
    threshold = policy.get("recovery_threshold_ps")
    hold, gap = policy.get("recovery_hold_s", .5), policy.get("recovery_max_gap_s", .1)
    results = []
    for key, module, start, end, recoverable in windows:
        result = metric_row(key, module, start, end, recoverable, samples, threshold, hold, gap)
        result.update(run_id=run["run_id"], condition=run["condition"], block=run["block"],
                      measurement_scope="out_of_loop", baseline_ps=baseline,
                      eligible_for_analysis=not reasons)
        results.append(result)
    fields = ["run_id", "condition", "block", "measurement_scope", "event_key", "module",
              "eligible_for_analysis", "baseline_ps", "window_start_monotonic_s", "window_end_monotonic_s",
              "valid_queries", *METRICS, "recovery_status", "recovery_threshold_ps", "recovery_hold_s", "recovery_max_gap_s"]
    write_csv(run_dir / "event_metrics.csv", results, fields)
    diagnostic = dict(eligible_for_analysis=not reasons, reasons=sorted(set(reasons)),
                      baseline_ps=baseline, baseline_valid_queries=len(baseline_values),
                      query_rows=len(rows), valid_query_rows=sum(v is not None for _, v in parsed),
                      timestamp_semantics="query midpoint approximation; acquisition time/freshness unverified",
                      aggregation="one independent controller run per endpoint; events are not repeats")
    (run_dir / "analysis_diagnostics.json").write_text(json.dumps(diagnostic, ensure_ascii=False, indent=2), encoding="utf-8")
    plot_queries(run_dir / "scope_curve.svg", rows, baseline, state2 or 0.0, actual)
    return results, diagnostic


def aggregate(rows):
    groups = {}
    for row in rows:
        if row.get("condition") != "PI":
            raise ValueError("summary condition must be PI")
        if not row.get("eligible_for_analysis"):
            continue
        key = row["condition"], row["event_key"]
        runs = groups.setdefault(key, {})
        if row["run_id"] in runs:
            raise ValueError("duplicate event endpoint for one independent run")
        runs[row["run_id"]] = row
    output = []
    for (condition, event), runs in sorted(groups.items()):
        for metric in METRICS:
            values = [finite(row.get(metric)) for row in runs.values()]
            values = [value for value in values if value is not None]
            output.append(dict(condition=condition, event_key=event, metric=metric,
                               independent_runs=len(runs), runs_with_metric=len(values),
                               mean=statistics.fmean(values) if values else None,
                               sample_sd=statistics.stdev(values) if len(values) > 1 else None))
    return output


def analyze_batch(batch_dir):
    batch_dir = Path(batch_dir)
    manifest = json.loads((batch_dir / "batch_manifest.json").read_text(encoding="utf-8"))
    policy = manifest.get("analysis", manifest.get("analysis_policy", manifest))
    all_rows, reports = [], {}
    for run in manifest["runs"]:
        directory = Path(run.get("run_dir", run["run_id"]))
        if not directory.is_absolute(): directory = batch_dir / directory
        results, report = analyze_run(directory, run, policy)
        all_rows.extend(results)
        reports[run["run_id"]] = report
    summary = aggregate(all_rows)
    write_csv(batch_dir / "pi_summary.csv", summary,
              ["condition", "event_key", "metric", "independent_runs", "runs_with_metric", "mean", "sample_sd"])
    write_csv(batch_dir / "pi_run_metrics.csv", all_rows,
              ["run_id", "condition", "block", "event_key", "eligible_for_analysis", *METRICS, "recovery_status"])
    (batch_dir / "analysis_diagnostics.json").write_text(json.dumps(reports, ensure_ascii=False, indent=2), encoding="utf-8")
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("batch_dir", type=Path)
    args = parser.parse_args(argv)
    analyze_batch(args.batch_dir)
    print(args.batch_dir / "pi_summary.csv")


if __name__ == "__main__":
    main()
