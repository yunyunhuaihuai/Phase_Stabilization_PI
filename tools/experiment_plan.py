"""PI stimulus schedule; frequency-to-voltage conversion is an example assumption."""
import math

BASE_FREQ = 2.0e9
HZ_PER_MV = 1.0
CONDITIONS = ("PI",)


def full_plan(base_freq=BASE_FREQ, hz_per_mv=HZ_PER_MV):
    if not all(math.isfinite(x) and x>0 for x in (base_freq,hz_per_mv)):
        raise ValueError("stimulus conversion must be finite and positive")
    events, elapsed = [], 0.0

    def add(name, frequency=None, apply=False, module=None):
        frequency = base_freq if frequency is None else frequency
        events.append(dict(planned_s=round(elapsed, 9), event=name,
                           freq_hz=float(frequency),
                           equiv_mv=(frequency - base_freq) / hz_per_mv,
                           apply_frequency=apply, module=module))

    add("MODULE_1_STEP_BASELINE_10s", module="step")
    elapsed += 10
    for mv in (50.0, -150.0, 250.0):
        add(f"STEP_{mv:+.1f}mV", base_freq + mv * hz_per_mv, True, "step")
        elapsed += 15
        add("STEP_RECOVERY", base_freq, True, "step")
        elapsed += 15
    add("MODULE_2_PULSE_BASELINE_10s", module="pulse")
    elapsed += 10
    for pulse in range(1, 6):
        add(f"PULSE_{pulse}_ON", base_freq + 7.4 * hz_per_mv, True, "pulse")
        elapsed += .1
        add(f"PULSE_{pulse}_OFF", base_freq, True, "pulse")
        elapsed += 1
    add("MODULE_3_RAMP_BASELINE_10s", module="ramp")
    elapsed += 10
    for index in range(1, 101):
        ramp_elapsed = index * .1
        add(f"RAMP_{ramp_elapsed:.1f}s", base_freq + 25 * ramp_elapsed * hz_per_mv,
            True, "ramp")
        elapsed += .1
    elapsed += 5
    add("RAMP_RECOVERY", base_freq, True, "ramp")
    elapsed += 15
    add("INTERFERENCE_DONE")
    elapsed += 10
    add("POST_WAIT_DONE")
    return events


def repeated_runs(repeats):
    # One algorithm: each block is one independent PI run.
    if repeats < 1:
        raise ValueError("repeats must be positive")
    return [dict(block=b, position=1, condition="PI", algo="PI") for b in range(1,repeats+1)]


def expected_config(algo="PI", limit=5500):
    if algo != "PI":
        raise ValueError("controller must be PI")
    result = {'CONTROL_FREQ_HZ': '20', 'TOTAL_DURATION_MIN': '4', 'MICROSTEP_RATIO': '1', 'BAUD_RATE': '115200', 'MAX_STEP_PER_CYCLE': '5500', 'MAX_PHYSICAL_DELTA': '5500', 'BACKLASH_STEPS': '0', 'BASE_B0': '0.0025', 'POS_MIN_LIMIT': '0', 'POS_MAX_LIMIT': '2112000', 'POS_CENTER': '1056000', 'TARGET_VPHS_MV': '0', 'DEADBAND_MV': '0.15', 'SWEEP_TOLERANCE_MV': '2', 'KP': '40', 'KI': '3200', 'FLUSH_INTERVAL': '200'}
    result.update(MAX_STEP_PER_CYCLE=str(limit), MAX_PHYSICAL_DELTA=str(limit))
    return result


def verify_config(text, algo="PI", limit=5500):
    values = {}
    for line in text.splitlines():
        line = line.split("#", 1)[0].split("//", 1)[0].strip()
        if "=" in line:
            key, value = [part.strip() for part in line.split("=", 1)]
            if key in values:
                raise ValueError(f"duplicate config key: {key}")
            values[key] = value
    if set(values) != set(expected_config(algo, limit)):
        raise ValueError("config keys differ from PI schema")
    for key, wanted in expected_config(algo, limit).items():
        actual = values.get(key, "")
        try:
            equal = math.isclose(float(actual), float(wanted), rel_tol=0, abs_tol=1e-12)
        except ValueError:
            equal = actual.upper() == wanted.upper()
        if not equal:
            raise ValueError(f"config mismatch {key}: {actual!r}, expected {wanted!r}")
    return values
