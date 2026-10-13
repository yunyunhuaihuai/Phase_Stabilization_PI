#pragma once
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <csignal>
#include "../config/SystemConfig.h"
#include "../hal/IHardwareHAL.h"

// Home, center, search, probe both slopes, then four-point gain/polarity calibration.
// Driver coordinates support startup checks; they do not establish mechanical arrival.
class WorkingPoint {
public:
    struct Diagnostics {
        bool homing=false, sweep=false, calibrated=false;
        bool failure_in_homing=false;
        HomingExecutionReport home_report;
        int32_t anchor=0;
        float polarity=1, gain=0;
        int sweep_budget=60;
        double left_slope=NAN, right_slope=NAN;
        double calibration_slope=NAN, calibration_r2=NAN;
    } diagnostics;
    WorkingPoint(volatile std::sig_atomic_t& running, void (*sleep_ms)(int))
        : g_running(running), SleepMilliseconds(sleep_ms) {}
bool SendPosition(IHardwareHAL& hal, int32_t pos) {
    if (!g_running || !hal.SendPosCmd(pos)) {
        g_running = 0;
        fprintf(stderr, "FATAL: motion command not accepted; startup/loop aborted\n");
        return false;
    }
    return true;
}
float ReadPhase(IHardwareHAL& hal) {
    const float value=hal.ReadVPHS();
    if (!std::isfinite(value)) {
        g_running=0;
        fprintf(stderr,"FATAL: nonfinite ADC value; aborting control/startup\n");
    }
    return value;
}
int32_t ClampPos(int32_t pos, const SystemConfig& cfg) {
    if (pos > cfg.pos_max_limit) return cfg.pos_max_limit;
    if (pos < cfg.pos_min_limit) return cfg.pos_min_limit;
    return pos;
}

bool ValidateAnchor(IHardwareHAL& hal, const SystemConfig& cfg,
                           int32_t anchor, int32_t& escape_dir) {
    const int32_t PROBE = 10000;
    const float   GATE  = 0.5f * cfg.b0;
    escape_dir = 1;

    const float v_anchor = ReadPhase(hal);

    const int32_t left  = ClampPos(anchor - PROBE, cfg);
    const int32_t d_left = anchor - left;
    if (!SendPosition(hal, left)) return false;
    if (!hal.WaitForMoveComplete(5000)) return false;
    const float v_left = ReadPhase(hal);

    if (!SendPosition(hal, anchor)) return false;
    if (!hal.WaitForMoveComplete(5000)) return false;

    const int32_t right = ClampPos(anchor + PROBE, cfg);
    const int32_t d_right = right - anchor;
    if (!SendPosition(hal, right)) return false;
    if (!hal.WaitForMoveComplete(5000)) return false;
    const float v_right = ReadPhase(hal);

    if (!SendPosition(hal, anchor)) return false;
    if (!hal.WaitForMoveComplete(5000)) return false;

    if (d_left < PROBE / 2 || d_right < PROBE / 2) {
        escape_dir = (anchor > (cfg.pos_min_limit + cfg.pos_max_limit) / 2) ? -1 : 1;
        return false;
    }

    const float s_left  = (v_anchor - v_left)  / static_cast<float>(d_left);
    const float s_right = (v_right - v_anchor) / static_cast<float>(d_right);

    const bool monotonic = (s_left * s_right > 0.0f);
    const bool strong    = (fabs(s_left) >= GATE) && (fabs(s_right) >= GATE);
    const bool ok = monotonic && strong;
    diagnostics.left_slope = s_left;
    diagnostics.right_slope = s_right;

    if (!ok) {

        escape_dir = (fabs(s_right) >= fabs(s_left)) ? 1 : -1;
        printf("    [Anchor] 锚点验证拒绝: sL=%.6f sR=%.6f (门槛 %.6f)\n",
               static_cast<double>(s_left), static_cast<double>(s_right),
               static_cast<double>(GATE));
    }
    return ok;
}

bool CalibrateGain(IHardwareHAL& hal, const SystemConfig& cfg,
                          int32_t anchor, float& out_polarity, float& out_b0) {
    const int N = 4;
    const int32_t STEP = 1500;
    const int32_t start = ClampPos(anchor - STEP * (N - 1), cfg);

    const int32_t preload = ClampPos(start - 1000, cfg);
    if (!SendPosition(hal, preload)) return false;
    if (!hal.WaitForMoveComplete(4000)) return false;

    float   v[N];
    int32_t p[N];

    for (int i = 0; i < N; ++i) {
        const int32_t target = ClampPos(start + STEP * i, cfg);
        if (!SendPosition(hal, target)) return false;
        if (!hal.WaitForMoveComplete(4000)) return false;
        const int32_t rep = hal.GetReportedPos();
        if (abs(rep - target) > 1000) return false;
        p[i] = rep;
        v[i] = ReadPhase(hal);
        if (!std::isfinite(v[i])) return false;
    }

    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (int i = 0; i < N; ++i) {
        const double x = static_cast<double>(p[i] - p[0]);
        const double y = static_cast<double>(v[i] - v[0]);
        sx += x; sy += y; sxx += x * x; sxy += x * y;
    }
    const double denom = static_cast<double>(N) * sxx - sx * sx;
    if (fabs(denom) < 1.0) return false;
    const double slope     = (static_cast<double>(N) * sxy - sx * sy) / denom;
    const double intercept = (sy - slope * sx) / static_cast<double>(N);

    const double ybar = sy / static_cast<double>(N);
    double ss_tot = 0, ss_res = 0;
    for (int i = 0; i < N; ++i) {
        const double x = static_cast<double>(p[i] - p[0]);
        const double y = static_cast<double>(v[i] - v[0]);
        const double e = y - (slope * x + intercept);
        ss_tot += (y - ybar) * (y - ybar);
        ss_res += e * e;
    }
    if (ss_tot < 1e-12) return false;
    const double r2 = 1.0 - ss_res / ss_tot;
    diagnostics.calibration_slope = slope;
    diagnostics.calibration_r2 = r2;

    if (!(r2 >= 0.93) ||
        !(fabs(slope) >= 0.5 * static_cast<double>(cfg.b0))) {
        fprintf(stderr, "    [Calibration] 标定拒绝: k=%.6f mV/步 R2=%.4f (需 R2>=0.93 且 |k|>=%.6f)\n",
                slope, r2, static_cast<double>(0.5f * cfg.b0));
        return false;
    }

    out_polarity = (slope > 0.0) ? 1.0f : -1.0f;
    out_b0 = static_cast<float>(fabs(slope));
    if (out_b0 < 0.0005f) out_b0 = 0.0005f;
    if (out_b0 > 0.0040f) out_b0 = 0.0040f;

    printf("    [Calibration] 标定: k=%.6f mV/步 R2=%.4f 极性=%+.0f b0=%.6f\n",
           slope, r2, static_cast<double>(out_polarity),
           static_cast<double>(out_b0));

    if (!SendPosition(hal, anchor)) return false;
    if (!hal.WaitForMoveComplete(4000)) return false;
    return true;
}

bool BlindSweepToZero(IHardwareHAL& hal, const SystemConfig& cfg,
                             int32_t& current_phys, int& budget) {
    const float v_start = ReadPhase(hal);
    if (!std::isfinite(v_start)) return false;
    if (fabs(v_start) <= cfg.sweep_tolerance_mv) return true;

    int32_t probe = ClampPos(current_phys + 1000, cfg);
    if (!SendPosition(hal, probe)) return false;
    if (!hal.WaitForMoveComplete(3000)) return false;
    const float v_probe = ReadPhase(hal);
    if (!std::isfinite(v_probe)) return false;
    const int dir = (fabs(v_probe) < fabs(v_start)) ? 1 : -1;
    float v_prev = v_probe;
    current_phys = probe;

    while (budget > 0) {
        --budget;
        int32_t step = 500;
        const float scale = fabs(v_prev) * 250.0f;
        if (scale > 500.0f) {
            if (!std::isfinite(scale) || scale > 20000) step = 20000;
            else
            step = static_cast<int32_t>(scale);
            if (step < 1) step = 500;
        }
        if (step > 20000) step = 20000;

        current_phys = ClampPos(current_phys + dir * step, cfg);
        if (!SendPosition(hal, current_phys)) return false;
        if (!hal.WaitForMoveComplete(5000)) return false;

        const float v_curr = ReadPhase(hal);
        if (!std::isfinite(v_curr)) return false;
        printf("    Sweep: pos=%d V_PHS=%.3f\n",
               static_cast<int>(current_phys), static_cast<double>(v_curr));

        if ((v_prev * v_curr <= 0.0f) ||
            (fabs(v_curr) < cfg.sweep_tolerance_mv)) {
            return true;
        }
        v_prev = v_curr;
    }
    return false;
}

    bool Establish(IHardwareHAL& hal, const SystemConfig& cfg) {
        diagnostics = Diagnostics{};
        int32_t& current_phys = diagnostics.anchor;
        current_phys = cfg.pos_center;
    printf("\n>>> [State 0] 执行硬件归零 (zero start)...\n");
    diagnostics.home_report=hal.HomeWithSafety(60000,&g_running);
    if (diagnostics.home_report.result != HomingExecutionResult::Complete) {
        diagnostics.failure_in_homing=true;
        fprintf(stderr,
                "\nFATAL: 硬件归零失败 result=%s cleanup_owned=%d "
                "stop_unknown=%d hardware_intervention_required=%d, 终止\n",
                HomingExecutionResultName(diagnostics.home_report.result),
                diagnostics.home_report.shutdown_sequence_handled,
                diagnostics.home_report.stop_state_unknown,
                diagnostics.home_report.hardware_intervention_required);
        return false;
    }
    diagnostics.homing = true;
    printf("    归零完成: 坐标原点已标定为 0\n");

    printf("    执行绝对坐标强制归中...\n");
    if (SendPosition(hal, cfg.pos_center) && hal.WaitForMoveComplete(15000)) {
        current_phys = cfg.pos_center;
        const int32_t home_reported = hal.GetReportedPos();
        printf("    归中完成: 命令坐标 %d | 驱动内部回报位置: %d "
               "(非独立机械反馈)\n",
               static_cast<int>(current_phys),
               static_cast<int>(home_reported));

        if (abs(home_reported - cfg.pos_center) > 1000) {
            fprintf(stderr, "\nFATAL: 归中驱动内部回报位置 %d ≠ 目标 %d; "
                            "机械位置未独立核验, 终止\n",
                    static_cast<int>(home_reported),
                    static_cast<int>(cfg.pos_center));
                return false;
        }
    } else {
        fprintf(stderr, "\nFATAL: 归中移动未获一致的驱动内部停止/目标位置指示 "
                        "(15s 超时); 机械位置未知, 终止\n");
        return false;
    }

    SleepMilliseconds(1000);

    printf("\n>>> [State 1/1.5] 盲扫寻零 + 工作点验证 + 多点标定...\n");

    float& sys_polarity = diagnostics.polarity;
    float& measured_b0 = diagnostics.gain;
    measured_b0 = cfg.b0;
    bool  anchored      = false;
    int& sweep_budget = diagnostics.sweep_budget;

    while (!anchored && g_running) {
        if (fabs(ReadPhase(hal)) > cfg.sweep_tolerance_mv) {
            if (!BlindSweepToZero(hal, cfg, current_phys, sweep_budget)) {
                fprintf(stderr, "\nFATAL: 盲扫未找到过零点/预算耗尽/运动超时, 拒绝闭环\n");
                break;
            }
        }

        int32_t escape_dir = 1;
        if (ValidateAnchor(hal, cfg, current_phys, escape_dir)) {
            anchored = true;
        } else {
            if (!g_running) break;
            if (sweep_budget <= 0) {
                fprintf(stderr, "\nFATAL: 锚定点始终未通过单调性验证 (预算耗尽), 拒绝闭环\n");
                break;
            }

            --sweep_budget;
            current_phys = ClampPos(current_phys + escape_dir * 10000, cfg);
            if (!SendPosition(hal, current_phys) || !hal.WaitForMoveComplete(5000)) {
                fprintf(stderr, "\nFATAL: 逃离非单调区移动超时, 拒绝闭环\n");
                break;
            }
        }
    }
    diagnostics.sweep = anchored;
    if (!anchored) {
        return false;
    }

    if (!CalibrateGain(hal, cfg, current_phys, sys_polarity, measured_b0)) {
        fprintf(stderr, "\nFATAL: 标定失败 (超时/假完成/R2 或信噪比不足), 拒绝进入闭环\n");
        return false;
    }
    printf("    工作点: pos=%d | 极性 %+.0f | b0=%.6f\n",
           static_cast<int>(current_phys),
           static_cast<double>(sys_polarity),
           static_cast<double>(measured_b0));

        diagnostics.calibrated = true;
        return true;
    }
private:
    volatile std::sig_atomic_t& g_running;
    void (*SleepMilliseconds)(int);
};
