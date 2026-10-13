#ifndef DISCRETE_PI_H
#define DISCRETE_PI_H

#include <cstdint>
#include <cmath>
#include "../config/SystemConfig.h"

// ============================================================================
// DiscretePI: 离散 PI 控制器 (带 Anti-Windup 动态反算)
//
// 核心特点:
//   1. Anti-Windup: 当执行器饱和时, 从积分器中"吐出"被截断部分
//   2. 死区保护: 误差小于 DEADBAND 时冻结输出
//   3. 反方向目标偏移: 负向运动时减去 BACKLASH_STEPS
//   4. 边界保护: 位置上限/下限钳位
//   5. 绝对零动态内存/STL
//
// 算法:
//   error = Target - V_PHS
//   error *= polarity
//   P = KP * error
//   I_candidate = integrator + KI * error * dt
//   delta = P + I_candidate - last_logical_pos
//   delta_sat = clamp(delta, -MAX_STEP, +MAX_STEP)
//   if delta != delta_sat: integrator = (last_logical_pos + delta_sat) - P
//   else: integrator = I_candidate
//   logical_pos = last_logical_pos + delta_sat
//   clamp(logical_pos, POS_MIN, POS_MAX)
//   negative direction: physical = trunc(logical) - backlash
// ============================================================================

class DiscretePI {
public:
    DiscretePI();

    // --- 初始化 (注入配置指针; 控制器只有读取权限, 拒绝修改) ---
    void Init(const SystemConfig* cfg) { cfg_ = cfg; }

    // --- 无扰切换: 注入当前物理位置和极性 ---
    void BumplessTransfer(int32_t current_physical_pos, float measured_polarity);

    // --- 核心计算: 输入当前电压, 输出 (物理指令, 逻辑位置) ---
    // dt 单位为秒 (float)
    void Compute(float current_vphs, float dt,
                 int32_t& out_physical_cmd, float& out_logical_pos);

    // --- 获取当前状态 (用于诊断日志) ---
    float GetIntegrator() const { return integrator_; }
    float GetPolarity()   const { return system_polarity_; }

private:
    // 配置指针 (只读, 由 Init 在构造函数后注入)
    const SystemConfig* cfg_;

    // 控制器状态
    float   integrator_;          // 积分器累计值
    float   last_logical_pos_;    // 上一拍逻辑位置 (连续值)
    int32_t last_physical_cmd_;   // 上一拍控制器提出的物理目标
    int32_t current_dir_;         // 当前运动方向: 1 正, -1 负
    float   system_polarity_;     // 系统极性: +1 或 -1
};

#endif // DISCRETE_PI_H