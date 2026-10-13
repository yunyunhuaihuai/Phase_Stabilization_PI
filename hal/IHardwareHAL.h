#ifndef I_HARDWARE_HAL_H
#define I_HARDWARE_HAL_H

#include <csignal>
#include <cstdint>
#include "MotorSafety.h"

// ============================================================================
// IHardwareHAL: 纯虚基类 — 硬件接口抽象层
//
// 用途: 将 Red Pitaya (ADC/温度) 和 VSMD 电机驱动(串口)统一抽象为接口
// 实现:
//   - LinuxHAL: 封装 <rp.h> + <termios.h> 真实硬件访问
//   - SimHAL  : 加噪高斯随机数 + printf 桩函数 (SIL 仿真)
//
// 注入方式: 全局静态实例化，按引用传入 Controller
// 禁止: new/delete 动态分配
// ============================================================================

// 温度无效标记
static const float TEMP_INVALID = -9999.0f;

class IHardwareHAL {
public:
    virtual ~IHardwareHAL() {}

    // --- 初始化 (返回值 0 成功, -1 失败) ---
    virtual int Init() = 0;

    // --- 读取稳相电压 (mV) ---
    virtual float ReadVPHS() = 0;

    // --- 读取温度 (°C) ---
    virtual float ReadTemp() = 0;

    // --- 向 VSMD 发送绝对位置指令 ---
    // True means the software send layer accepted the complete command only.
    virtual bool SendPosCmd(int32_t pos) = 0;

    // Detailed host-side result for the most recent position-line write.
    // Complete means only that the local serial stack accepted the whole line;
    // it is not a driver acknowledgement or position-arrival indication.
    virtual SerialWriteReport LastPositionWriteReport() const {
        return SerialWriteReport();
    }

    // --- 获取当前电机速度 (rps) 轮询等待移动完成 ---
    // 返回值: 0.0f 表示已停止 (有效帧); NAN 表示超时/读取失败 (状态不可知)
    // 调用方必须用 isnan() 区分二者, 严禁把超时当停稳 (假完成会导致状态机穿透)
    virtual float GetMotorSpeed() = 0;

    // --- 驱动器/仿真回报的当前绝对位置 (诊断用) ---
    // LinuxHAL 从 sts 回帧解析并缓存; 默认 0 表示实现未提供
    virtual int32_t GetReportedPos() { return 0; }

    // --- 无等待地消费已经到达的驱动器状态反馈 ---
    // 闭环使用此接口读取上一条 pos 命令的回帧；实现必须有界且不得等待。
    // 返回位置是驱动器内部坐标，不是独立机械反馈。
    virtual MotorStatusSnapshot PollMotorStatus() {
        MotorStatusSnapshot status;
        status.result = MotorStatusResult::Unsupported;
        return status;
    }

    // --- 主动查询驱动器状态（非实时路径） ---
    // timeout_ms 是明确的等待上限。禁止把 150 ms 查询塞进 50 ms 闭环。
    virtual MotorStatusSnapshot QueryMotorStatus(int32_t timeout_ms) {
        (void)timeout_ms;
        MotorStatusSnapshot status;
        status.result = MotorStatusResult::Unsupported;
        return status;
    }

    // --- 有界停止并读取驱动器停止状态（非实时/退出路径） ---
    // command_written 仅代表软件写入完整；只有 driver_stop_confirmed 才表示
    // 驱动器随后报告了近零速度和 stp 位，仍不等于独立机械停止证明。
    virtual MotorStopReport StopMotion(MotorStopMode mode,
                                       int32_t verify_timeout_ms) {
        (void)mode;
        (void)verify_timeout_ms;
        MotorStopReport report;
        report.mode = mode;
        report.status.result = MotorStatusResult::Unsupported;
        return report;
    }

    // --- 配置 ODL 倍率 (ODL 初始化时调用) ---
    virtual int ConfigODL(int ratio) = 0;

    // --- 等待电机宏动完成 (轮询速度趋近 0) ---
    // timeout_ms: 最大等待毫秒数
    // 返回值: true 移动完成, false 超时
    virtual bool WaitForMoveComplete(int32_t timeout_ms) = 0;

    // --- 硬件归零 (zero start): 退回机械原点并标定坐标 0 ---
    // 开环步进无断电记忆, 驱动器重启后坐标系漂移 (当前位置被标为 0),
    // 归零是任何绝对定位的前提, 否则大行程指令会撞硬件限位。
    // timeout_ms: 最大等待毫秒数 (典型 20-60s)
    // 返回值: 0 成功, -1 超时/失败
    virtual int Home(int32_t timeout_ms) { (void)timeout_ms; return 0; }   // 默认实现: 无归零机制

    // Safety-aware homing entry used by production startup.  Hardware HALs
    // override this to implement the VSMD zero-stop-first failure sequence and
    // bounded interruption.  The legacy adapter keeps host-only fakes working
    // but does not claim that a dedicated shutdown sequence ran.
    virtual HomingExecutionReport HomeWithSafety(
            int32_t timeout_ms,
            const volatile std::sig_atomic_t* running) {
        HomingExecutionReport report;
        if (running && !*running) {
            report.result = HomingExecutionResult::Interrupted;
            return report;
        }
        const int rc = Home(timeout_ms);
        report.result = rc == 0 ? HomingExecutionResult::Complete
                                : HomingExecutionResult::StatusFailure;
        report.driver_home_confirmed = rc == 0;
        report.stop_state_unknown = rc != 0;
        report.hardware_intervention_required = rc != 0;
        return report;
    }

    // --- 释放 / 关闭资源 ---
    virtual void Release() = 0;

    // --- 写入串口数据 (低级别, 被 SendPosCmd 等使用) ---
    virtual int WriteSerial(const uint8_t* data, int len) = 0;

    // --- 读取串口数据到缓冲区 (低级别, 被 GetMotorSpeed 等使用) ---
    virtual int ReadSerial(uint8_t* buf, int max_len) = 0;
};

#endif // I_HARDWARE_HAL_H
