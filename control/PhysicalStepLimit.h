#pragma once

#include <cstdint>
#include <limits>

struct PhysicalStepCorrection {
    int32_t target = 0;
    bool corrected_single_step_overrun = false;
};

// Integer conversion of a floating absolute target can land one pulse beyond
// the configured per-cycle delta, even when the PI delta was already clamped.
// Correct only that single-pulse quantization case; larger violations remain
// visible to CommandGate and are rejected before serial transmission.
inline PhysicalStepCorrection CorrectSingleStepQuantizationOverrun(
        int32_t proposed_target,
        int32_t last_accepted_target,
        int32_t max_physical_delta) {
    PhysicalStepCorrection result;
    result.target = proposed_target;
    if (max_physical_delta < 0) return result;

    const int64_t delta = static_cast<int64_t>(proposed_target) -
                          static_cast<int64_t>(last_accepted_target);
    const int64_t magnitude = delta < 0 ? -delta : delta;
    if (magnitude != static_cast<int64_t>(max_physical_delta) + 1 ||
        delta == 0) {
        return result;
    }

    const int64_t corrected = static_cast<int64_t>(last_accepted_target) +
        (delta > 0 ? static_cast<int64_t>(max_physical_delta)
                   : -static_cast<int64_t>(max_physical_delta));
    if (corrected < std::numeric_limits<int32_t>::min() ||
        corrected > std::numeric_limits<int32_t>::max()) {
        return result;
    }

    result.target = static_cast<int32_t>(corrected);
    result.corrected_single_step_overrun = true;
    return result;
}
