#ifndef LINUX_HAL_H
#define LINUX_HAL_H

#include "IHardwareHAL.h"
#include "../comm/RingBuffer.h"
#include "../comm/VSMDParser.h"

// ============================================================================
// LinuxHAL: 真实 Red Pitaya 硬件接口
//
// 封装:
//   - <rp.h>   : Red Pitaya C API (ADC 采样、温度)
//   - <termios.h> : Linux 原生串口 (Raw 模式, 非阻塞)
//
// 串口参数:
//   设备: PI_SERIAL_DEVICE 环境变量
//   波特率: 115200 (由 config.txt BAUD_RATE 注入, main 中调用 SetBaudRate)
//   Raw 模式, 8N1, 非阻塞
//
// ADC:
//   通道: RP_CH_1
//   降采样: RP_DEC_1024
//   采样窗口: 20ms = 0.02 * (125e6 / 1024) 点
//
// 温度:
//   rp_AIpinGetValue(0)
//
// 内存安全:
//   - 全局静态预分配缓冲区
//   - 无动态内存分配
// ============================================================================

#if USE_REAL_HARDWARE

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <errno.h>
#include <mutex>

// 串口缓冲区大小
static const int SERIAL_RX_BUF_SIZE = 1024;

// ADC 采样点数
// 20ms 采样 @ 125MHz / 1024 ≈ 122kHz -> 约 2440 点
static const int ADC_SAMPLE_POINTS = 2500;

class LinuxHAL : public IHardwareHAL {
public:
    LinuxHAL();
    virtual ~LinuxHAL();

    // --- IHardwareHAL ---
    int  Init() override;
    float ReadVPHS() override;
    float ReadTemp() override;
    bool SendPosCmd(int32_t pos) override;
    SerialWriteReport LastPositionWriteReport() const override {
        std::lock_guard<std::mutex> lock(position_report_mutex_);
        return last_position_write_;
    }
    float GetMotorSpeed() override;
    MotorStatusSnapshot PollMotorStatus() override;
    MotorStatusSnapshot QueryMotorStatus(int32_t timeout_ms) override;
    MotorStopReport StopMotion(MotorStopMode mode,
                               int32_t verify_timeout_ms) override;
    int  ConfigODL(int ratio) override;
    bool WaitForMoveComplete(int32_t timeout_ms) override;
    int  Home(int32_t timeout_ms) override;
    HomingExecutionReport HomeWithSafety(
        int32_t timeout_ms,
        const volatile std::sig_atomic_t* running) override;
    void Release() override;
    int  WriteSerial(const uint8_t* data, int len) override;
    int  ReadSerial(uint8_t* buf, int max_len) override;

    // --- 运行时设置波特率 (从 SystemConfig 读取后调用) ---
    void SetBaudRate(int baud) { configured_baud_ = baud; }
    bool SetSerialDevice(const char* device) {
        if (!device || device[0]!='/' || std::strlen(device)>=sizeof(serial_device_)) return false;
        std::snprintf(serial_device_,sizeof(serial_device_),"%s",device);
        return true;
    }

    // --- 驱动器回报的当前绝对位置 (GetMotorSpeed 解析 sts 帧时刷新) ---
    int32_t GetReportedPos() override {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        return reported_pos_;
    }

private:
    // --- 串口操作 ---
    int  openSerial(const char* device, int baud_rate);
    int  baudToBps(int baud);
    void closeSerial();
    SerialWriteReport writeSerialUnlocked(const uint8_t* data, int len,
                                          int32_t deadline_ms);
    int  readSerialWaitUnlocked(uint8_t* buf, int max_len,
                                int32_t timeout_ms);
    int  readSerialNowUnlocked(uint8_t* buf, int max_len);
    MotorStatusSnapshot queryMotorStatusUnlocked(int32_t timeout_ms);
    MotorStopReport stopMotionUnlocked(MotorStopMode mode,
                                       int32_t verify_timeout_ms);
    bool decodeStatusFrame(const VSMD_Frame& frame,
                           MotorStatusSnapshot& status);

    // --- ADC 操作 ---
    int  initADC();
    float readADC();

    // --- 温度操作 ---
    float readTempSensor();

    bool    initialized_;
    bool    adc_initialized_;
    bool    serial_open_;

    // 串口 fd
    int     serial_fd_;

    // 串口名称 (运行前从环境变量注入)
    char    serial_device_[32];
    int     configured_baud_;

    // 状态变量
    int32_t last_sent_pos_;
    int32_t reported_pos_;   // 驱动器 sts 帧回报的位置 (诊断用)
    float   reported_speed_;
    uint32_t reported_status_flags_;
    AsciiStreamState ascii_stream_;
    SerialWriteReport last_position_write_;
    bool    motion_may_be_active_;
    bool    complete_stop_written_since_motion_;
    bool    driver_stop_confirmed_since_motion_;
    bool    homing_shutdown_handled_;

    // 所有串口 fd 和解析器访问均串行化；缓存快照由当前单线程控制流程读取。
    // 闭环只使用无等待 Poll；主动 Query/Stop 仅允许出现在启动、宏动或退出路径。
    std::mutex serial_mutex_;
    mutable std::mutex position_report_mutex_;

    // 串口环形缓冲区 (异步接收)
    RingBuffer<uint8_t, SERIAL_RX_BUF_SIZE> rx_ring_;

    // VSMD 帧解析器
    VSMDParser  parser_;

    // ADC 缓冲 (r.p. API 需要)
    // 注意: rp.h 的 rp_AcqGetLatestDataV 需要外部缓冲区
    // 我们使用静态数组
    float   adc_buffer_[ADC_SAMPLE_POINTS];
    int     adc_pts_needed_;
};

#else
// 非 USE_REAL_HARDWARE 模式: LinuxHAL 不编译, 定义空桩
// 防止 CMake 编译时报未定义
class LinuxHAL {
public:
    LinuxHAL() {}
    int Init() { return -1; }
};
#endif // USE_REAL_HARDWARE

#endif // LINUX_HAL_H
