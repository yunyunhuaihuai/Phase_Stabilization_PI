#include "DiscretePI.h"
#include "PhysicalStepLimit.h"

DiscretePI::DiscretePI()
    : cfg_(nullptr)
    , integrator_(0.0f)
    , last_logical_pos_(0.0f)
    , last_physical_cmd_(0)
    , current_dir_(1)
    , system_polarity_(1.0f)
{
}

void DiscretePI::BumplessTransfer(int32_t current_physical_pos, float measured_polarity) {
    system_polarity_ = measured_polarity;
    integrator_      = static_cast<float>(current_physical_pos);
    last_logical_pos_ = static_cast<float>(current_physical_pos);
    last_physical_cmd_ = current_physical_pos;
    current_dir_ = 1;
}

void DiscretePI::Compute(float current_vphs, float dt,
                         int32_t& out_physical_cmd, float& out_logical_pos) {
    // 安全防护
    if (!cfg_) {
        out_physical_cmd = last_physical_cmd_;
        out_logical_pos  = last_logical_pos_;
        return;
    }

    // 提取配置 (局部变量加速)
    const float target_mv     = cfg_->target_vphs_mv;
    const float deadband_mv   = cfg_->deadband_mv;
    const float kp            = cfg_->kp;
    const float ki            = cfg_->ki;
    const int32_t max_step    = cfg_->max_step_per_cycle;
    const int32_t backlash    = cfg_->backlash_steps;
    const int32_t pos_min     = cfg_->pos_min_limit;
    const int32_t pos_max     = cfg_->pos_max_limit;

    // 1. 误差计算与极性修正
    float error = target_mv - current_vphs;
    error *= system_polarity_;

    // 2. 死区保护
    if (fabs(error) < deadband_mv) {
        out_physical_cmd = last_physical_cmd_;
        out_logical_pos  = last_logical_pos_;
        return;
    }

    // 3. 比例项
    float p_term = kp * error;

    // 4. 积分项候选
    float i_candidate = integrator_ + ki * error * dt;

    // 5. 理论总需求
    float logical_pos_unlimited = p_term + i_candidate;
    float delta_unlimited = logical_pos_unlimited - last_logical_pos_;

    // 6. 执行器物理限幅
    float delta_saturated = delta_unlimited;
    if (delta_unlimited > static_cast<float>(max_step)) {
        delta_saturated = static_cast<float>(max_step);
    } else if (delta_unlimited < -static_cast<float>(max_step)) {
        delta_saturated = -static_cast<float>(max_step);
    }

    // 7. Anti-Windup: 如果发生截断, 反算积分器
    // 用 float 比较, 允许微小误差
    if (fabs(delta_unlimited - delta_saturated) > 1e-6f) {
        // 钳位积分器: 使其严格等于真实物理位置减去比例项
        integrator_ = (last_logical_pos_ + delta_saturated) - p_term;
    } else {
        integrator_ = i_candidate;
    }

    // 8. 计算逻辑位置
    float logical_pos = last_logical_pos_ + delta_saturated;

    // 9. 边界保护 + 积分反算
    if (logical_pos > static_cast<float>(pos_max)) {
        logical_pos = static_cast<float>(pos_max);
        integrator_ = logical_pos - p_term;
    }
    if (logical_pos < static_cast<float>(pos_min)) {
        logical_pos = static_cast<float>(pos_min);
        integrator_ = logical_pos - p_term;
    }

    // 10. 换向判定
    if (logical_pos > last_logical_pos_) {
        current_dir_ = 1;
    } else if (logical_pos < last_logical_pos_) {
        current_dir_ = -1;
    }

    last_logical_pos_ = logical_pos;

    // 11. 齿隙补偿
    int32_t physical_cmd;
    if (current_dir_ == 1) {
        physical_cmd = static_cast<int32_t>(logical_pos);
    } else {
        physical_cmd = static_cast<int32_t>(logical_pos) - backlash;
    }

    if (physical_cmd < pos_min) physical_cmd = pos_min;
    if (physical_cmd > pos_max) physical_cmd = pos_max;

    // Keep the integer absolute target within the physical per-cycle bound.
    // The gate remains authoritative: only a one-step float-to-int overrun is
    // corrected here; any larger violation is still rejected before sending.
    const PhysicalStepCorrection correction =
        CorrectSingleStepQuantizationOverrun(
            physical_cmd,last_physical_cmd_,cfg_->max_physical_delta);
    if (correction.corrected_single_step_overrun) {
        physical_cmd = correction.target;
        // Rebase the controller to the corrected physical command so the
        // fractional overrun is not proposed again on the next cycle.
        logical_pos = static_cast<float>(physical_cmd) +
            (current_dir_ < 0 ? static_cast<float>(backlash) : 0.0f);
        integrator_ = logical_pos - p_term;
        last_logical_pos_ = logical_pos;
    }

    last_physical_cmd_ = physical_cmd;

    // 12. 输出
    out_physical_cmd = physical_cmd;
    out_logical_pos  = logical_pos;
}
