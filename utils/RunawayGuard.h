#ifndef RUNAWAY_GUARD_H
#define RUNAWAY_GUARD_H

#include <cstdint>
#include <cmath>

//
// Pre-send protection: five rising-error saturated beats, directional travel
// above 200000 pulses, or 50 beats with absolute error above 150 mV.
// A qualified recovery requires 21 consecutive samples with |error|<=0.5 mV,
// |proposal delta|<=200 pulses and no saturation. Trips latch until Reset().
// These are software observations; physical safety has not been certified.

class RunawayGuard {
public:
    enum TripBits : uint32_t {
        NONE = 0,
        RISING_SATURATION = 1,
        DIRECTIONAL_TRAVEL = 2,
        LARGE_ERROR = 4,
        INVALID_INPUT = 8
    };

    void Init(int32_t max_step) {
        max_step_ = max_step;
        Reset();
    }

    void Reset() {
        prev_abs_err_ = -1.0f;
        ramp_beats_ = 0;
        last_dir_ = 0;
        disp_steps_ = 0;
        accepted_disp_steps_ = 0;
        pending_disp_steps_ = 0;
        pending_proposal_ = false;
        v_abs_beats_ = 0;
        recovery_beats_ = 0;
        recovery_resets_ = 0;
        trip_mask_ = NONE;
    }

    // The caller must reject the proposed command when this returns true.
    // A trip is latched until Reset()/Init(); main.cpp exits on the first trip.
    bool Feed(float v_phs, float target_mv, int32_t cmd_delta) {
        pending_proposal_ = false;
        if (trip_mask_ != NONE) return true;
        const float err = std::fabs(v_phs - target_mv);
        if (max_step_ <= 0 || !std::isfinite(v_phs) ||
            !std::isfinite(target_mv) || !std::isfinite(err)) {
            trip_mask_ = INVALID_INPUT;
            recovery_beats_ = 0;
            return true;
        }
        // Promote before negation so INT32_MIN cannot overflow.
        const int64_t abs_delta = cmd_delta < 0
            ? -static_cast<int64_t>(cmd_delta)
            : static_cast<int64_t>(cmd_delta);
        const bool sat = abs_delta >= static_cast<int64_t>(max_step_);
        const int dir = cmd_delta > 0 ? 1 : (cmd_delta < 0 ? -1 : 0);

        // Rule 1: preserve the original counting/first-saturated-beat convention.
        if (sat && dir != 0 && dir == last_dir_ &&
            prev_abs_err_ >= 0.0f && err > prev_abs_err_) {
            if (ramp_beats_ < TRIP_BEATS) ++ramp_beats_;
        } else if (sat && dir != 0) {
            ramp_beats_ = 1;
        } else {
            ramp_beats_ = 0;
        }

        // Rule 2: reversal starts a new directional segment, as before.
        // A zero command preserves the count until qualified recovery below.
        if (dir != 0) {
            if (dir == last_dir_) disp_steps_ += abs_delta;
            else disp_steps_ = abs_delta;
        }

        // Rule 3: unchanged threshold and persistence.
        if (err > V_ABS_MV) {
            if (v_abs_beats_ < V_ABS_BEATS) ++v_abs_beats_;
        } else {
            v_abs_beats_ = 0;
        }

        const bool recovery_sample =
            err <= RECOVERY_ERROR_MV &&
            abs_delta <= RECOVERY_COMMAND_STEPS && !sat;
        if (recovery_sample) {
            if (recovery_beats_ < RECOVERY_SAMPLES) ++recovery_beats_;
        } else {
            recovery_beats_ = 0;
        }

        if (dir != 0) last_dir_ = dir;
        prev_abs_err_ = err;

        // Check all trip conditions BEFORE any recovery-based reset.
        if (ramp_beats_ >= TRIP_BEATS) trip_mask_ |= RISING_SATURATION;
        if (disp_steps_ > DISP_LIMIT_STEPS) trip_mask_ |= DIRECTIONAL_TRAVEL;
        if (v_abs_beats_ >= V_ABS_BEATS) trip_mask_ |= LARGE_ERROR;
        if (trip_mask_ != NONE) {
            pending_disp_steps_ = disp_steps_;
            pending_proposal_ = true;
            return true;
        }

        if (recovery_beats_ >= RECOVERY_SAMPLES) {
            if (disp_steps_ != 0) {
                disp_steps_ = 0;
                ++recovery_resets_;
            }
            // Require another full window before another recovery reset.
            recovery_beats_ = 0;
        }
        pending_disp_steps_ = disp_steps_;
        pending_proposal_ = true;
        return false;
    }

    // Main calls this only after a hold is accepted or the complete pos command
    // is written.  It does not prove driver acceptance or mechanical arrival.
    // The prospective total used for the pre-send trip remains in disp_steps_.
    void CommitAcceptedProposal() {
        if (!pending_proposal_ || trip_mask_ != NONE) return;
        accepted_disp_steps_ = pending_disp_steps_;
        pending_proposal_ = false;
    }

    uint32_t TripMask() const { return trip_mask_; }
    int64_t DirectionalSteps() const { return disp_steps_; }
    int64_t AcceptedDirectionalSteps() const { return accepted_disp_steps_; }
    int RisingBeats() const { return ramp_beats_; }
    int LargeErrorBeats() const { return v_abs_beats_; }
    int RecoveryBeats() const { return recovery_beats_; }
    uint64_t RecoveryResets() const { return recovery_resets_; }
    int32_t MaxStep() const { return max_step_; }

    static const char* Version() { return "pi-guard/1.0"; }
    static int64_t DisplacementLimit() { return DISP_LIMIT_STEPS; }
    static float RecoveryErrorMv() { return RECOVERY_ERROR_MV; }
    static int32_t RecoveryCommandSteps() { return RECOVERY_COMMAND_STEPS; }
    static int RecoverySamples() { return RECOVERY_SAMPLES; }

private:
    static const int TRIP_BEATS = 5;
    // Software threshold; not hardware-safety certified.
    static const int64_t DISP_LIMIT_STEPS = 200000;
    static constexpr float V_ABS_MV = 150.0f;
    static const int V_ABS_BEATS = 50;

    static constexpr float RECOVERY_ERROR_MV = 0.5f;
    static const int32_t RECOVERY_COMMAND_STEPS = 200;
    static const int RECOVERY_SAMPLES = 21;

    int32_t max_step_ = 0;
    float prev_abs_err_ = -1.0f;
    int ramp_beats_ = 0;
    int last_dir_ = 0;
    int64_t disp_steps_ = 0;
    int64_t accepted_disp_steps_ = 0;
    int64_t pending_disp_steps_ = 0;
    bool pending_proposal_ = false;
    int v_abs_beats_ = 0;
    int recovery_beats_ = 0;
    uint64_t recovery_resets_ = 0;
    uint32_t trip_mask_ = NONE;
};

#endif // RUNAWAY_GUARD_H
