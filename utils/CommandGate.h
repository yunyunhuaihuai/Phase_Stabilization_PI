#ifndef COMMAND_GATE_H
#define COMMAND_GATE_H

#include <cmath>
#include <cstdint>
#include <limits>
#include "RunawayGuard.h"

struct CommandGateDecision {
    int32_t proposed_delta = 0;
    bool bounds_ok = false;
    bool guard_trip = false;
    bool allowed = false;
};

// Pure pre-send gate shared by main and offline tests.  RunawayGuard observes
// the proposal before allowed can become true; callers must only commit the
// proposal after a hold is accepted or a complete serial write succeeds.
inline CommandGateDecision EvaluateCommandProposal(
        RunawayGuard& guard,
        bool input_finite,
        bool controller_fault,
        float v_phs,
        float target_mv,
        int32_t proposed_target,
        int32_t last_accepted_target,
        int32_t position_min,
        int32_t position_max,
        int32_t max_physical_delta) {
    CommandGateDecision decision;
    const int64_t delta64 = static_cast<int64_t>(proposed_target) -
                            static_cast<int64_t>(last_accepted_target);
    const int64_t abs_delta64 = delta64 < 0 ? -delta64 : delta64;
    const bool delta_representable =
        delta64 >= std::numeric_limits<int32_t>::min() &&
        delta64 <= std::numeric_limits<int32_t>::max();
    if (delta_representable)
        decision.proposed_delta = static_cast<int32_t>(delta64);
    decision.bounds_ok = delta_representable &&
        proposed_target >= position_min && proposed_target <= position_max &&
        max_physical_delta >= 0 && abs_delta64 <= max_physical_delta;
    decision.guard_trip = input_finite && !controller_fault &&
        guard.Feed(v_phs,target_mv,decision.proposed_delta);
    decision.allowed = input_finite && !controller_fault &&
        decision.bounds_ok && !decision.guard_trip;
    return decision;
}

#endif // COMMAND_GATE_H
