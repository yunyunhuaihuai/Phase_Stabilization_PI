// PI phase stabilization: safe startup, nominal 20 Hz loop, bounded logging.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <thread>
#include <csignal>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif
static void SleepMilliseconds(int ms) {
#ifdef _WIN32
    Sleep(static_cast<DWORD>(ms));
#else
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
#endif
}
template<class TimePoint> static void SleepUntil(TimePoint deadline) {
#ifdef _WIN32
    const double ms=std::chrono::duration<double,std::milli>(deadline-std::chrono::steady_clock::now()).count();
    if(ms>0) SleepMilliseconds(static_cast<int>(std::ceil(ms)));
#else
    std::this_thread::sleep_until(deadline);
#endif
}

// 项目内部头文件
#include "config/ConfigParser.h"
#include "config/SystemConfig.h"
#include "hal/IHardwareHAL.h"
#include "hal/LinuxHAL.h"
#include "hal/SimHAL.h"
#include "control/DiscretePI.h"
#include "comm/VSMDParser.h"
#include "utils/DataLogger.h"
#include "utils/LoggerShutdown.h"
#include "utils/RunawayGuard.h"   // [T3] 防飞车熔断器
#include "utils/CommandGate.h"
#include "utils/RealTimeHelper.h"
#include "tools/Timing.h"
#include "startup/WorkingPoint.h"

// ============================================================================
// 全局静态实例 (零动态分配)
// ============================================================================

// SIGINT 安全退出标志: 置 0 后主循环干净滑落执行 日志 fsync + 硬件释放
static volatile std::sig_atomic_t g_running = 1;
static void sigint_handler(int) {
    g_running = 0;
}

// 配置
static SystemConfig g_config;
static SystemConfig g_pi_config;

// 硬件抽象层 — 条件编译选择 (全局静态实例, 无 new)
#if USE_REAL_HARDWARE
static LinuxHAL g_hal;
#else
static SimHAL g_hal;
#endif

// 控制器
static DiscretePI  g_pi_controller;

// 日志
static DataLogger g_logger;
// Keep optional file logging at the application boundary. Control and terminal
// reports remain active even when no logger streams or writer are started.
static bool g_logging_enabled = true;

static void PrintStopReport(const char* reason,const MotorStopReport& report) {
    fprintf(stderr,
        "[MotorStop] reason=%s mode=%s write_outcome=%s "
        "host_accepted_bytes=%d/%d write_attempts=%d command_written=%d "
        "status_query_attempted=%d query_write_outcome=%s "
        "query_host_accepted_bytes=%d/%d status_observed_after_active_query=%d "
        "driver_internal_stop_indicated=%d stop_state_unknown=%d "
        "hardware_intervention_required=%d status_result=%s "
        "driver_reported_speed=%.9g driver_reported_position=%d "
        "driver_status_flags=%u mechanical_stop_independently_verified=0\n",
        reason,MotorStopModeName(report.mode),
        SerialWriteOutcomeName(report.command_write.outcome),
        report.command_write.accepted_bytes,report.command_write.requested_bytes,
        report.command_write.write_attempts,report.command_write.IsComplete(),
        report.status_query_attempted,
        SerialWriteOutcomeName(static_cast<SerialWriteOutcome>(
            report.status.active_query_write_outcome)),
        report.status.active_query_accepted_bytes,
        report.status.active_query_requested_bytes,
        report.status.observed_after_active_query,
        report.driver_stop_confirmed,report.stop_state_unknown,
        report.hardware_intervention_required,
        MotorStatusResultName(report.status.result),
        static_cast<double>(report.status.speed),
        static_cast<int>(report.status.driver_reported_position),
        static_cast<unsigned>(report.status.status_flags));
}

static void PrintHomingReport(const HomingExecutionReport& report) {
    fprintf(stderr,
        "[HomingSafety] result=%s prepare_zero_stop=%s(%d/%d) "
        "prepare_status=%s prepare_observed_after_query=%d zero_start=%s(%d/%d) "
        "completion_status=%s completion_observed_after_query=%d cleanup_attempted=%d "
        "abort_zero_stop=%s(%d/%d) ordinary_stop_attempted=%d "
        "driver_home_indicated=%d driver_internal_stop_indicated=%d "
        "stop_state_unknown=%d hardware_intervention_required=%d "
        "mechanical_stop_independently_verified=0\n",
        HomingExecutionResultName(report.result),
        SerialWriteOutcomeName(report.preparation_zero_stop_write.outcome),
        report.preparation_zero_stop_write.accepted_bytes,
        report.preparation_zero_stop_write.requested_bytes,
        MotorStatusResultName(report.preparation_status.result),
        report.preparation_status.observed_after_active_query,
        SerialWriteOutcomeName(report.zero_start_write.outcome),
        report.zero_start_write.accepted_bytes,
        report.zero_start_write.requested_bytes,
        MotorStatusResultName(report.completion_status.result),
        report.completion_status.observed_after_active_query,
        report.cleanup_attempted,
        SerialWriteOutcomeName(report.failure_stop.zero_stop_write.outcome),
        report.failure_stop.zero_stop_write.accepted_bytes,
        report.failure_stop.zero_stop_write.requested_bytes,
        report.failure_stop.ordinary_stop_attempted,
        report.driver_home_confirmed,
        HomingExecutionReportsDriverStopped(report),
        report.stop_state_unknown,
        report.hardware_intervention_required);
}

static MotorStopReport StopForExit(IHardwareHAL& hal,MotorStopMode mode,
                                   const char* reason) {
    const MotorStopReport report=hal.StopMotion(mode,250);
    PrintStopReport(reason,report);
    if (g_logging_enabled) g_logger.LogMotorStopReport(reason,report);
    return report;
}

// ============================================================================
static WorkingPoint g_startup(g_running, SleepMilliseconds);

// ============================================================================
// 状态机执行
// ============================================================================
int main(int argc, char* argv[]) {
    double simulation_seconds = 0;
    const char* config_path = "config.txt";
    bool config_seen = false, simulation_seen = false, log_mode_seen = false;
    if (argc == 2 && !std::strcmp(argv[1], "--help")) {
        printf("Usage: PhaseStabilization [config.txt] [--simulation-seconds N] [--log-mode full|off]\n"
               "  config.txt: defaults to ./config.txt; options may appear in any order\n"
               "  --simulation-seconds N: offline only, finite N in (0, 86400]\n"
               "  --log-mode full: record board data (default; required by Python runners)\n"
               "  --log-mode off: no board data files or cycle writer; terminal reports remain\n");
        return EXIT_SUCCESS;
    }
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--simulation-seconds")) {
            if (simulation_seen || i + 1 == argc) {
                fprintf(stderr,"Missing or duplicate simulation duration; use --help\n");
                return EXIT_FAILURE;
            }
            simulation_seen = true;
#if USE_REAL_HARDWARE
            fprintf(stderr,"Simulation duration override requires an offline build\n");
            return EXIT_FAILURE;
#else
            const char* value = argv[++i];
            char* end=nullptr;
            simulation_seconds=std::strtod(value,&end);
            if (end==value || *end || !std::isfinite(simulation_seconds) ||
                simulation_seconds<=0 || simulation_seconds>86400) {
                fprintf(stderr,"Invalid simulation duration\n"); return EXIT_FAILURE;
            }
#endif
        } else if (!std::strcmp(argv[i], "--log-mode")) {
            if (log_mode_seen || i + 1 == argc) {
                fprintf(stderr,"Missing or duplicate log mode; use --help\n");
                return EXIT_FAILURE;
            }
            log_mode_seen = true;
            const char* value = argv[++i];
            if (!std::strcmp(value, "full")) g_logging_enabled = true;
            else if (!std::strcmp(value, "off")) g_logging_enabled = false;
            else {
                fprintf(stderr,"Invalid log mode; expected full or off\n");
                return EXIT_FAILURE;
            }
        } else if (argv[i][0] == '-' || config_seen) {
            fprintf(stderr,"Invalid arguments; use --help\n"); return EXIT_FAILURE;
        } else {
            config_path = argv[i];
            config_seen = true;
        }
    }

    // ---- 0. 实时提权 ----
#if USE_REAL_HARDWARE
    RealTime_Enable();
#endif

    // Ctrl+C: 置位退出标志, 由主循环干净清理 (日志 fsync + 硬件释放)
    std::signal(SIGINT, sigint_handler);
    std::signal(SIGTERM, sigint_handler);
#ifdef __linux__
    // [T1] SIGPIPE 忽略: 无人值守长测经 SSH/tmux 启动, SSH 掉线或合盖后
    // stdout 变为断管, 下一次 printf 默认直接杀死进程 (exit 141),
    // 表象与"偶发段错误"一致。忽略后 write 返回 EPIPE, 程序自行续跑。
    std::signal(SIGPIPE, SIG_IGN);
#endif

    // ---- 1. 加载配置 ----
    printf("==========================================\n");
    printf("[Logging] mode=%s\n", g_logging_enabled ? "full" : "off (terminal only)");
    printf("  %s: PI phase stabilization\n", SOFTWARE_VERSION);
    printf("==========================================\n");

    if (ConfigParser_Load(config_path, g_config) != 0) {
        fprintf(stderr, "FATAL: rejected configuration '%s'\n", config_path);
        return EXIT_FAILURE;
    }
    ConfigParser_Print(g_config);
    printf("[RunawayGuard] %s: displacement_limit=%lld "
           "qualification=not_hardware_safety_validated "
           "recovery_error_mv=%.3f recovery_command_steps=%d recovery_samples=%d\n",
           RunawayGuard::Version(),
           static_cast<long long>(RunawayGuard::DisplacementLimit()),
           static_cast<double>(RunawayGuard::RecoveryErrorMv()),
           static_cast<int>(RunawayGuard::RecoveryCommandSteps()),
           RunawayGuard::RecoverySamples());
    g_pi_config = g_config;
    g_pi_config.max_step_per_cycle = g_config.EffectiveLogicalLimit();

    // ---- 2. 硬件初始化 ----
#if USE_REAL_HARDWARE
    g_hal.SetBaudRate(g_config.baud_rate);
    const char* serial_device=std::getenv("PI_SERIAL_DEVICE");
    if (!serial_device || !g_hal.SetSerialDevice(serial_device)) {
        fprintf(stderr,"FATAL: set PI_SERIAL_DEVICE to the Linux serial device path\n");
        return EXIT_FAILURE;
    }
#endif

    if (g_hal.Init() != 0) {
        fprintf(stderr, "FATAL: 硬件初始化失败\n");
        return EXIT_FAILURE;
    }

    // ODL 配置 (初始化序列; 禁止下发 cfg mcs, 纪律红线)
    if (g_hal.ConfigODL(g_config.microstep_ratio) != 0) {
        fprintf(stderr,"FATAL: ODL 驱动配置命令写入失败\n");
        StopForExit(g_hal,MotorStopMode::Immediate,"configuration_failure");
        g_hal.Release();
        return EXIT_FAILURE;
    }

    // ---- 3. 配置控制器 (只读配置指针注入) ----
    g_pi_controller.Init(&g_pi_config);

    // ---- 4. 主循环时序参数 ----
    const double loop_dt_s = 1.0 / static_cast<double>(g_config.control_freq_hz);
    using Clock      = std::chrono::steady_clock;
    using Duration   = std::chrono::duration<double>;
    using TimePoint  = std::chrono::time_point<Clock, Duration>;

    const Duration loop_dt(loop_dt_s);
    const double   total_duration_s =
        simulation_seconds>0 ? simulation_seconds : static_cast<double>(g_config.total_duration_min) * 60.0;
    const float    dt_float = static_cast<float>(loop_dt_s);

    // ---- 5. 状态机变量 ----
    int32_t current_phys = g_config.pos_center;
    int32_t last_phys_cmd = g_config.pos_center;

    // In full mode, preserve fatal log-open handling. Off mode never opens
    // streams, creates metadata, or starts the cycle writer.
    const char* algo_str = "PI";
    if (g_logging_enabled) {
        g_logger.SetFlushInterval(g_config.flush_interval);
        if (g_logger.Open(algo_str, g_config.total_duration_min) != 0) {
            fprintf(stderr, "FATAL: 日志文件创建失败, 长测无数据, 终止\n");
            StopForExit(g_hal,MotorStopMode::Immediate,"log_open_failure");
            g_hal.Release();
            return EXIT_FAILURE;
        }
    }

    // ---- 6. 主循环时序锚点 ----
    TimePoint t_start = Clock::now();
    TimePoint t_next  = t_start + loop_dt;

    // ============================================================================
    // State 0: 硬件归零 (zero start) — 绝对定位的必需前置!
    // 开环步进电机无断电记忆: 驱动器重启后把当前物理位置强制标定为坐标 0。
    // 归零失败必须终止, 严禁带错位坐标系继续定位。
    // ============================================================================
    const bool startup_ok = g_startup.Establish(g_hal, g_config);
    if (g_logging_enabled && g_startup.diagnostics.home_report.result !=
        HomingExecutionResult::NotAttempted) {
        g_logger.LogHomingReport(g_startup.diagnostics.home_report);
    }
    if (!startup_ok) {
        if (g_startup.diagnostics.failure_in_homing) {
            const HomingExecutionReport& home =
                g_startup.diagnostics.home_report;
            PrintHomingReport(home);
            // HomeWithSafety owns every shutdown decision made after homing
            // has begun.  Do not append an unconditional stp after zero-stop,
            // or after a partial ASCII line quarantined the serial stream.
            if (!home.shutdown_sequence_handled) {
                StopForExit(g_hal,MotorStopMode::Immediate,
                            "homing_not_started_failure");
            }
        } else {
            StopForExit(g_hal,MotorStopMode::Immediate,"startup_failure");
        }
        g_hal.Release();
        return EXIT_FAILURE;
    }
    current_phys = g_startup.diagnostics.anchor;
    const float sys_polarity = g_startup.diagnostics.polarity;
    const float measured_b0 = g_startup.diagnostics.gain;

    // 回锚后验证最终初态电压为有限值。
    const float v_initial = g_startup.ReadPhase(g_hal);
    if (!g_running || !std::isfinite(v_initial)) {
        StopForExit(g_hal,MotorStopMode::Immediate,"invalid_initial_sample");
        g_hal.Release(); return EXIT_FAILURE;
    }

    g_pi_controller.BumplessTransfer(current_phys, sys_polarity);

    if (g_logging_enabled &&
        !g_logger.OpenDiagnostics(g_config, measured_b0, sys_polarity, +1, config_path)) {
        fprintf(stderr, "FATAL: diagnostic log creation failed\n");
        StopForExit(g_hal,MotorStopMode::Immediate,"diagnostic_log_failure");
        g_hal.Release(); return EXIT_FAILURE;
    }
    last_phys_cmd = current_phys;
#if !USE_REAL_HARDWARE
    if (const char* amplitude=std::getenv("PI_SIM_DISTURBANCE_MV")) {
        char* end=nullptr;
        const double value=std::strtod(amplitude,&end);
        if (end==amplitude || *end || !std::isfinite(value) || value<0 || value>10) {
            StopForExit(g_hal,MotorStopMode::Immediate,"invalid_simulation_disturbance");
            g_hal.Release(); return EXIT_FAILURE;
        }
        g_hal.SetDisturbance(value);
    }
#endif

    // ---- State 2: 闭环控制 ----
    printf("\n>>> [State 2] 切入 %.1fHz 连续伺服引擎\n",
           static_cast<double>(g_config.control_freq_hz));
    printf("    算法: %s\n", algo_str);
    printf("------------------------------------------\n");

    // [T3] 防飞车熔断器 (SimHAL 兼容: 只用主循环已有量)
    RunawayGuard runaway;
    runaway.Init(g_config.EffectiveLogicalLimit());
    runaway.Reset();

    // 重置计时起点
    t_start = Clock::now();
    t_next  = t_start + loop_dt;

    uint64_t cycle = 0;
    bool failed = false;
    double previous_cycle_us = 0;
    double previous_sample_seconds = timing::Seconds();
    double accepted_logical = current_phys;
    float previous_pi_logical = static_cast<float>(current_phys);
    int pi_direction = 1;
    MotorStatusSnapshot last_driver_status;
    uint64_t driver_status_age_cycles = 0;
    bool stop_attempted = false;
    while (g_running) {
        SleepUntil(t_next);
        const auto cycle_start = Clock::now();
        const double cycle_clock = timing::Seconds();
        if (!g_running) break;
        const bool deadline_late = cycle_start > t_next + std::chrono::milliseconds(5);
        if (deadline_late) t_next = cycle_start;
        const double elapsed_s = Duration(cycle_start - t_start).count();
        if (elapsed_s > total_duration_s) break;

        const float v_phs = g_startup.ReadPhase(g_hal);
        const double sampled_seconds = timing::Seconds();
        const float temp_c = g_hal.ReadTemp();
        // Consume only feedback bytes that have already arrived (normally the
        // previous pos reply).  No sts command, select wait or sleep is allowed
        // on this 50 ms path.
        const MotorStatusSnapshot polled_driver_status = g_hal.PollMotorStatus();
        if (polled_driver_status.IsValid()) {
            last_driver_status = polled_driver_status;
            driver_status_age_cycles = 0;
        } else if (last_driver_status.IsValid()) {
            ++driver_status_age_cycles;
        }
        CycleDiagnostics record;
        record.cycle = ++cycle;
        record.monotonic_s = sampled_seconds;
        record.sample_dt_s = sampled_seconds - previous_sample_seconds;
        previous_sample_seconds = sampled_seconds;
        record.previous_cycle_us = previous_cycle_us;
        record.deadline_late = deadline_late || previous_cycle_us > 50000;
        record.driver_poll_result = polled_driver_status.result;
        record.driver_status_available = last_driver_status.IsValid();
        record.driver_status_age_cycles = driver_status_age_cycles;
        record.driver_status = last_driver_status;
        int32_t phys_cmd = last_phys_cmd;
        float log_cmd = static_cast<float>(accepted_logical);
        const bool finite = std::isfinite(v_phs);
        const double compute_start = timing::Seconds();
        if (finite)
            g_pi_controller.Compute(v_phs, dt_float, phys_cmd, log_cmd);
        record.compute_us = 1e6*(timing::Seconds()-compute_start);
        const double pi_requested = log_cmd-previous_pi_logical;
        if (pi_requested != 0) pi_direction = pi_requested > 0 ? 1 : -1;
        previous_pi_logical = log_cmd;
        const double logical_before = accepted_logical;
        const CommandGateDecision decision = EvaluateCommandProposal(
            runaway,finite,false,v_phs,g_config.target_vphs_mv,
            phys_cmd,last_phys_cmd,g_config.pos_min_limit,g_config.pos_max_limit,
            g_config.max_physical_delta);
        const int32_t delta = decision.proposed_delta;
        record.proposed_logical = log_cmd;
        record.proposed_physical = phys_cmd;
        record.proposed_physical_delta = delta;
        const bool bounds_ok = decision.bounds_ok;
        const bool trip = decision.guard_trip;
        const bool allowed = decision.allowed;
        bool send_ok = false;
        if (allowed) {
            send_ok = delta == 0 || g_hal.SendPosCmd(phys_cmd);
            if (delta != 0) {
                record.position_write = g_hal.LastPositionWriteReport();
            }
            record.send_result = delta == 0 ? 0 : (send_ok ? 1 : -1);
            if (send_ok) {
                runaway.CommitAcceptedProposal();
                last_phys_cmd = phys_cmd;
                accepted_logical =
                    static_cast<double>(phys_cmd)+(pi_direction<0 ? g_config.backlash_steps : 0);
            }
        } else {
            record.send_result = -2;
        }
        failed = !allowed || !send_ok;
        record.send_accepted = send_ok;
        record.accepted_physical = last_phys_cmd;
        record.accepted_logical = accepted_logical;
        record.guard_proposed_directional_steps = runaway.DirectionalSteps();
        record.guard_accepted_directional_steps = runaway.AcceptedDirectionalSteps();
        record.guard_directional_limit_steps = RunawayGuard::DisplacementLimit();
        record.control.raw_error = sys_polarity*(g_config.target_vphs_mv-v_phs);
        record.control.integrator = g_pi_controller.GetIntegrator();
        record.control.physical_delta = send_ok ? delta : 0;
        record.control.requested_u = pi_requested;
        record.control.submitted_u = send_ok ? accepted_logical-logical_before : 0;
        record.control.lower = -g_config.EffectiveLogicalLimit();
        record.control.upper = g_config.EffectiveLogicalLimit();
        record.status = failed ? (!finite ? PIStatus::InvalidInput :
            (trip ? PIStatus::GuardFault : (!bounds_ok ? PIStatus::Boundary : PIStatus::SendFault))) : PIStatus::Ready;
        if (failed) {
            // Fault policy: on a clean stream, send addressed VSMD
            // "1 stp 1" (immediate stop), then use a bounded sts query.  A
            // partial ASCII line permanently quarantines this connection: no
            // newline, query, retry, or stop bytes are appended because they
            // could complete a different valid command.  The report separates
            // local writes from driver state; neither proves mechanical stop.
            record.fault_stop = StopForExit(
                g_hal,MotorStopMode::Immediate,"closed_loop_fault");
            stop_attempted = true;
        }
        const char* state = failed ? "FAULT" : (std::fabs(g_config.target_vphs_mv-v_phs) < g_config.deadband_mv ? "LOCKED" : "TRACK");
        // Raw position columns are controller proposals,
        // including an inhibited FAULT row.  The diagnostic row explicitly
        // carries proposed, last software-accepted, and driver-reported values.
        if (g_logging_enabled)
            g_logger.LogCycle(elapsed_s,v_phs,temp_c,log_cmd,phys_cmd,state,record);
        if (cycle % 20 == 0)
            printf("[%7.2f s] raw V=%+.4f mV command=%d status=%s\n",elapsed_s,double(v_phs),int(last_phys_cmd),PIStatusName(record.status));
        if (g_logging_enabled && !g_logger.Healthy()) {
            failed = true;
            fprintf(stderr,"FATAL: logging failure\n");
            if (!stop_attempted) {
                StopForExit(g_hal,MotorStopMode::Immediate,"logging_failure");
                stop_attempted = true;
            }
        }
        previous_cycle_us = 1e6*(timing::Seconds()-cycle_clock);
        if (failed) {
            if (trip) {
                const uint32_t mask = runaway.TripMask();
                fprintf(stderr,
                    "[RunawayGuard] TRIP version=%s cycle=%llu t=%.6f "
                    "mask=%u rising=%d directional=%d large_error=%d invalid=%d "
                    "abs_error_mv=%.6f proposed_delta=%d proposed_target_not_sent=%d "
                    "last_software_accepted_target=%d proposed_directional_steps=%lld "
                    "accepted_directional_steps=%lld limit_steps=%lld "
                    "rising_beats=%d large_error_beats=%d recovery_beats=%d "
                    "recovery_resets=%llu\n",
                    RunawayGuard::Version(), static_cast<unsigned long long>(cycle),
                    elapsed_s, static_cast<unsigned>(mask),
                    (mask & RunawayGuard::RISING_SATURATION) != 0,
                    (mask & RunawayGuard::DIRECTIONAL_TRAVEL) != 0,
                    (mask & RunawayGuard::LARGE_ERROR) != 0,
                    (mask & RunawayGuard::INVALID_INPUT) != 0,
                    static_cast<double>(std::fabs(v_phs-g_config.target_vphs_mv)),
                    static_cast<int>(delta), static_cast<int>(phys_cmd),
                    static_cast<int>(last_phys_cmd),
                    static_cast<long long>(runaway.DirectionalSteps()),
                    static_cast<long long>(runaway.AcceptedDirectionalSteps()),
                    static_cast<long long>(RunawayGuard::DisplacementLimit()),
                    runaway.RisingBeats(), runaway.LargeErrorBeats(),
                    runaway.RecoveryBeats(),
                    static_cast<unsigned long long>(runaway.RecoveryResets()));
            }
            fprintf(stderr,"FATAL: cycle %llu %s send=%d\n",static_cast<unsigned long long>(cycle),PIStatusName(record.status),record.send_result);
            break;
        }
        t_next += loop_dt;
    }

    // ---- 清理 ----
    if (!stop_attempted) {
        StopForExit(g_hal,
            failed ? MotorStopMode::Immediate : MotorStopMode::Decelerated,
            failed ? "fault_exit" : (g_running ? "duration_complete" : "signal_exit"));
        stop_attempted = true;
    }
    if (g_logging_enabled) {
        printf("\n[System] 关闭日志...\n");
        g_logger.Close();
        if (!g_logger.Healthy()) {
            ReportLoggerShutdownFailure(failed, stderr);
        }
    }

    printf("[System] 释放硬件...\n");
    g_hal.Release();

    if (failed) {
        fprintf(stderr, "[System] 故障退出 (exit=1)，请检查此前 FATAL 日志\n");
    } else {
        printf("[System] 正常退出\n");
    }
    return failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
