#include "LinuxHAL.h"

#if USE_REAL_HARDWARE

#include <rp.h>     // Red Pitaya 官方 C API
#include <chrono>   // std::chrono::steady_clock / milliseconds (GetMotorSpeed 使用)

// ============================================================================
// LinuxHAL 实现 (极致排雷版)
// ============================================================================

LinuxHAL::LinuxHAL()
    : initialized_(false)
    , adc_initialized_(false)
    , serial_open_(false)
    , serial_fd_(-1)
    , configured_baud_(115200)
    , last_sent_pos_(1056000)
    , reported_pos_(0)
    , reported_speed_(NAN)
    , reported_status_flags_(0)
    , motion_may_be_active_(false)
    , complete_stop_written_since_motion_(false)
    , driver_stop_confirmed_since_motion_(false)
    , homing_shutdown_handled_(false)
    , adc_pts_needed_(0)
{
    memset(serial_device_, 0, sizeof(serial_device_));
    serial_device_[0]=0; // supplied by PI_SERIAL_DEVICE before Init
    memset(adc_buffer_, 0, sizeof(adc_buffer_));
}

LinuxHAL::~LinuxHAL() {
    Release();
}

// ============================================================================
// 初始化
// ============================================================================
int LinuxHAL::Init() {
    printf("[LinuxHAL] 正在初始化 Red Pitaya 硬件...\n");

    if (initADC() != 0) {
        fprintf(stderr, "[LinuxHAL] FATAL: ADC 初始化失败!\n");
        return -1;
    }

    // 波特率由 main 中调用 SetBaudRate() 提前注入 configured_baud_
    if (openSerial(serial_device_, configured_baud_) != 0) {
        fprintf(stderr, "[LinuxHAL] FATAL: 串口 %s 打开失败!\n", serial_device_);
        rp_Release();
        return -1;
    }

    initialized_ = true;
    printf("[LinuxHAL] hardware interface initialized\n");
    return 0;
}

// ============================================================================
// ADC 初始化 (修正了官方 API 签名)
// ============================================================================
int LinuxHAL::initADC() {
    if (rp_Init() != RP_OK) {
        fprintf(stderr, "[LinuxHAL] rp_Init 调用失败\n");
        return -1;
    }

    rp_AcqReset();
    rp_AcqSetDecimation(RP_DEC_1024);
    rp_AcqStart(); // 彻底开启无阻塞自由轮询

    // 125e6 / 1024 ≈ 122070.3 Hz 采样率
    // 0.02s * 122070.3 = 2441 点
    adc_pts_needed_ = static_cast<int>(0.02 * (125e6 / 1024.0));
    if (adc_pts_needed_ > ADC_SAMPLE_POINTS) {
        adc_pts_needed_ = ADC_SAMPLE_POINTS;
    }

    adc_initialized_ = true;
    return 0;
}

// ============================================================================
// 读取 VPHS (mV)
// ============================================================================
float LinuxHAL::readADC() {
    // 读取失败必须返回 NAN 哨兵, 严禁返回 0.0f:
    // 0.0f 恰为目标电压, 会触发死区冻结造成"假锁相" (传感器失效却显示 LOCKED)
    if (!adc_initialized_) return NAN;

    // 【致命修正】：rp_AcqGetLatestDataV 需要 uint32_t 的指针作为输入输出大小
    uint32_t pts = static_cast<uint32_t>(adc_pts_needed_);
    if (rp_AcqGetLatestDataV(RP_CH_1, &pts, adc_buffer_) != RP_OK || pts == 0) {
        return NAN;
    }

    // Compute the mean of the returned latest ADC buffer.
    double sum = 0.0;
    for (uint32_t i = 0; i < pts; ++i) {
        sum += static_cast<double>(adc_buffer_[i]);
    }

    // 转换为 mV
    return static_cast<float>((sum / static_cast<double>(pts)) * 1000.0);
}

float LinuxHAL::ReadVPHS() {
    return readADC();
}

// ============================================================================
// 温度读取 (修正了官方 API 签名)
// ============================================================================
float LinuxHAL::readTempSensor() {
    float raw_voltage = 0.0f;
    uint32_t raw_int = 0; // 新版 rp.h API 第三参数: 底层原始计数值
    if (rp_AIpinGetValue(0, &raw_voltage, &raw_int) != RP_OK) {
        return 0.0f;
    }

    if (raw_voltage >= 0.0f && raw_voltage <= 5.5f) {
        return raw_voltage * 10.0f;  // fixed 10 C/V conversion; calibration required
    }
    return 0.0f;
}

float LinuxHAL::ReadTemp() {
    return readTempSensor();
}

// ============================================================================
// 串口操作
// ============================================================================
int LinuxHAL::baudToBps(int baud) {
    switch (baud) {
        case 9600:    return B9600;
        case 115200:  return B115200;
        case 230400:  return B230400;
        case 460800:  return B460800;
        case 921600:  return B921600;
        default:      return B115200;
    }
}

int LinuxHAL::openSerial(const char* device, int baud_rate) {
    serial_fd_ = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_fd_, &tty) != 0) {
        close(serial_fd_);
        serial_fd_ = -1;
        return -1;
    }

    cfsetospeed(&tty, static_cast<speed_t>(baudToBps(baud_rate)));
    cfsetispeed(&tty, static_cast<speed_t>(baudToBps(baud_rate)));

    // 【致命修正】：纯粹的 Raw 模式，彻底关闭 Linux 一切干预，保护 0xFF 数据帧
    cfmakeraw(&tty);

    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;     // 无硬件流控
    tty.c_cflag |= CREAD | CLOCAL;

    // 清除一切可能破坏二进制帧头的位操作
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;

    // 0 阻塞，依赖外层 select 轮询
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        close(serial_fd_);
        serial_fd_ = -1;
        return -1;
    }

    serial_open_ = true;
    return 0;
}

void LinuxHAL::closeSerial() {
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
        serial_open_ = false;
    }
}

SerialWriteReport LinuxHAL::writeSerialUnlocked(const uint8_t* data, int len,
                                                int32_t deadline_ms) {
    if (ascii_stream_.IsTainted()) return BlockedAsciiWrite(len);
    if (serial_fd_ < 0 || !data || len <= 0 || deadline_ms < 0) {
        SerialWriteReport report;
        report.outcome = SerialWriteOutcome::ErrorNoBytes;
        report.requested_bytes = len > 0 ? len : 0;
        return report;
    }
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(deadline_ms);
    const SerialWriteReport report = WriteBoundedAsciiLine(
        ascii_stream_, data, len,
        [this](const uint8_t* remaining, int remaining_len) {
            const ssize_t n = write(serial_fd_, remaining,
                                    static_cast<size_t>(remaining_len));
            if (n > 0)
                return SerialWriteAttempt::Progress(static_cast<int>(n));
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK ||
                          errno == EINTR))
                return SerialWriteAttempt::Retry();
            return SerialWriteAttempt::Error();
        },
        [&deadline]() {
            return std::chrono::steady_clock::now() >= deadline;
        },
        []() { usleep(100); });
    if (report.IsPartial()) {
        motion_may_be_active_ = true;
        complete_stop_written_since_motion_ = false;
        driver_stop_confirmed_since_motion_ = false;
    }
    return report;
}

int LinuxHAL::readSerialWaitUnlocked(uint8_t* buf, int max_len,
                                     int32_t timeout_ms) {
    if (serial_fd_ < 0 || !buf || max_len <= 0 || timeout_ms < 0) return -1;
    fd_set read_fds;
    struct timeval timeout;
    FD_ZERO(&read_fds);
    FD_SET(serial_fd_, &read_fds);
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    const int sel = select(serial_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
    if (sel < 0) return errno == EINTR ? 0 : -1;
    if (sel == 0) return 0;
    const ssize_t n = read(serial_fd_, buf, static_cast<size_t>(max_len));
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
        return 0;
    return static_cast<int>(n);
}

int LinuxHAL::readSerialNowUnlocked(uint8_t* buf, int max_len) {
    if (serial_fd_ < 0 || !buf || max_len <= 0) return -1;
    const ssize_t n = read(serial_fd_, buf, static_cast<size_t>(max_len));
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
        return 0;
    return static_cast<int>(n);
}

int LinuxHAL::WriteSerial(const uint8_t* data, int len) {
    // A 64-byte frame takes about 5.6 ms at 115200 baud.  Ten milliseconds is
    // an explicit upper bound that remains below the 50 ms control period.
    std::lock_guard<std::mutex> lock(serial_mutex_);
    const SerialWriteReport report = writeSerialUnlocked(data, len, 10);
    return report.accepted_bytes > 0 ? report.accepted_bytes :
           (report.IsComplete() ? report.requested_bytes : -1);
}

int LinuxHAL::ReadSerial(uint8_t* buf, int max_len) {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    return readSerialWaitUnlocked(buf, max_len, 2);
}

// ============================================================================
// 硬件指令层
// ============================================================================
bool LinuxHAL::SendPosCmd(int32_t pos) {
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "1 pos %d\n", static_cast<int>(pos));
    const int cmd_len = static_cast<int>(strlen(cmd));
    // Never wait behind an active serial query on the 50 ms control path.
    // The current program is single-threaded; try_lock also keeps a future
    // accidental concurrent query from consuming the real-time budget.
    std::unique_lock<std::mutex> lock(serial_mutex_,std::try_to_lock);
    if (!lock.owns_lock()) {
        fprintf(stderr,"[LinuxHAL] WARN: serial busy; pos command not written\n");
        SerialWriteReport busy;
        busy.outcome = SerialWriteOutcome::BusyNoBytes;
        busy.requested_bytes = cmd_len;
        {
            std::lock_guard<std::mutex> report_lock(position_report_mutex_);
            last_position_write_ = busy;
        }
        return false;
    }
    // Any attempted position command invalidates an earlier stop indication.
    // A partial line may leave the driver target unknowable even though the
    // software never commits this proposal.
    motion_may_be_active_ = true;
    complete_stop_written_since_motion_ = false;
    driver_stop_confirmed_since_motion_ = false;
    homing_shutdown_handled_ = false;
    const SerialWriteReport position_write = writeSerialUnlocked(
        reinterpret_cast<const uint8_t*>(cmd), cmd_len, 10);
    {
        std::lock_guard<std::mutex> report_lock(position_report_mutex_);
        last_position_write_ = position_write;
    }
    if (!position_write.IsComplete()) {
        fprintf(stderr,
                "[LinuxHAL] ERROR: pos local write outcome=%s accepted=%d/%d "
                "stream=%s; driver target and stop state are unknown%s\n",
                SerialWriteOutcomeName(position_write.outcome),
                position_write.accepted_bytes,
                position_write.requested_bytes,
                ascii_stream_.IsTainted() ? "quarantined" : "clean",
                ascii_stream_.IsTainted()
                    ? "; do not append bytes; use approved hardware stop/disable procedure"
                    : "");
        return false;
    }
    last_sent_pos_ = pos;
    return true;
}

bool LinuxHAL::decodeStatusFrame(const VSMD_Frame& frame,
                                 MotorStatusSnapshot& status) {
    // VSMD v3.2 addressed status feedback: FF, cid=1, feedback=2,
    // speed[5], position[5], flags[5], BCC[2], FE.  The manual's generic
    // example contradicts its sts section about speed/position order; the
    // project parser follows the sts section and the established hardware
    // trace: speed first, position second.
    if (!frame.valid || frame.raw_len != VSMD_MIN_FRAME ||
        frame.raw[0] != 0xFF || frame.raw[1] != 1 ||
        frame.raw[2] != 2 || frame.raw[frame.raw_len - 1] != 0xFE ||
        !std::isfinite(frame.speed_rps)) {
        return false;
    }
    status.result = MotorStatusResult::Valid;
    status.speed = frame.speed_rps;
    status.driver_reported_position = frame.position_steps;
    status.status_flags = frame.status_flags;
    reported_speed_ = status.speed;
    reported_pos_ = status.driver_reported_position;
    reported_status_flags_ = status.status_flags;
    return true;
}

MotorStatusSnapshot LinuxHAL::PollMotorStatus() {
    // No select(), sleep, command or flush: consume at most 1024 already
    // available bytes (normally the previous pos reply), then return.
    std::unique_lock<std::mutex> lock(serial_mutex_,std::try_to_lock);
    MotorStatusSnapshot latest;
    if (!lock.owns_lock()) {
        latest.result = MotorStatusResult::Busy;
        return latest;
    }
    latest.result = MotorStatusResult::NoData;
    bool saw_protocol_error = false;
    uint8_t buf[128];
    for (int read_count = 0; read_count < 8; ++read_count) {
        const int n = readSerialNowUnlocked(buf, sizeof(buf));
        if (n < 0) {
            latest.result = MotorStatusResult::ReadFailure;
            return latest;
        }
        if (n == 0) break;
        for (int i = 0; i < n; ++i) {
            if (!parser_.Feed(buf[i])) continue;
            MotorStatusSnapshot decoded;
            if (decodeStatusFrame(parser_.GetFrame(), decoded)) latest = decoded;
            else saw_protocol_error = true;
        }
    }
    if (!latest.IsValid() && saw_protocol_error)
        latest.result = MotorStatusResult::ProtocolError;
    return latest;
}

MotorStatusSnapshot LinuxHAL::queryMotorStatusUnlocked(int32_t timeout_ms) {
    MotorStatusSnapshot status;
    if (ascii_stream_.IsTainted()) {
        status.result = MotorStatusResult::StreamTainted;
        status.active_query_write_outcome =
            static_cast<int>(SerialWriteOutcome::BlockedTainted);
        status.active_query_requested_bytes = 6;
        return status;
    }
    if (serial_fd_ < 0) {
        status.result = MotorStatusResult::ReadFailure;
        return status;
    }
    if (timeout_ms <= 0) {
        status.result = MotorStatusResult::Timeout;
        return status;
    }

    // Active queries are startup/exit-only.  Flush stale one-command replies,
    // reset partial parser state, then wait only until the explicit deadline.
    if (tcflush(serial_fd_, TCIFLUSH) != 0) {
        status.result = MotorStatusResult::ReadFailure;
        return status;
    }
    parser_.Reset();
    const char cmd[] = "1 sts\n";
    const SerialWriteReport query_write = writeSerialUnlocked(
        reinterpret_cast<const uint8_t*>(cmd),
        static_cast<int>(strlen(cmd)), 10);
    const auto attach_query_write = [&query_write](MotorStatusSnapshot& target) {
        target.active_query_requested_bytes = query_write.requested_bytes;
        target.active_query_accepted_bytes = query_write.accepted_bytes;
        target.active_query_write_attempts = query_write.write_attempts;
        target.active_query_write_outcome =
            static_cast<int>(query_write.outcome);
    };
    attach_query_write(status);
    if (!query_write.IsComplete()) {
        status.result = ascii_stream_.IsTainted()
            ? MotorStatusResult::StreamTainted
            : MotorStatusResult::WriteFailure;
        return status;
    }

    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(timeout_ms);
    bool saw_protocol_error = false;
    uint8_t buf[128];
    while (std::chrono::steady_clock::now() < deadline) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count();
        const int wait_ms = remaining > 2 ? 2 : static_cast<int>(remaining);
        const int n = readSerialWaitUnlocked(buf, sizeof(buf), wait_ms < 0 ? 0 : wait_ms);
        if (n < 0) {
            parser_.Reset();
            status.result = MotorStatusResult::ReadFailure;
            attach_query_write(status);
            return status;
        }
        for (int i = 0; i < n; ++i) {
            if (!parser_.Feed(buf[i])) continue;
            if (decodeStatusFrame(parser_.GetFrame(), status)) {
                parser_.Reset();
                status.observed_after_active_query = true;
                attach_query_write(status);
                return status;
            }
            saw_protocol_error = true;
        }
    }
    parser_.Reset();
    status.result = saw_protocol_error
        ? MotorStatusResult::ProtocolError
        : MotorStatusResult::Timeout;
    attach_query_write(status);
    return status;
}

MotorStatusSnapshot LinuxHAL::QueryMotorStatus(int32_t timeout_ms) {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    return queryMotorStatusUnlocked(timeout_ms);
}

MotorStopReport LinuxHAL::StopMotion(MotorStopMode mode,
                                     int32_t verify_timeout_ms) {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    return stopMotionUnlocked(mode,verify_timeout_ms);
}

MotorStopReport LinuxHAL::stopMotionUnlocked(MotorStopMode mode,
                                             int32_t verify_timeout_ms) {
    MotorStopReport report = ExecuteBoundedMotorStop(
        ascii_stream_, mode, verify_timeout_ms,
        [this](const uint8_t* data, int len) {
            return writeSerialUnlocked(data, len, 10);
        },
        [this](int32_t timeout_ms) {
            return queryMotorStatusUnlocked(timeout_ms);
        });
    complete_stop_written_since_motion_ = report.command_write.IsComplete();
    driver_stop_confirmed_since_motion_ = report.driver_stop_confirmed;
    if (report.driver_stop_confirmed) motion_may_be_active_ = false;
    return report;
}

float LinuxHAL::GetMotorSpeed() {
    // This legacy macro-move helper may wait up to 150 ms.  The 50 ms control
    // loop uses PollMotorStatus(), never this active query.
    const MotorStatusSnapshot status = QueryMotorStatus(150);
    return status.IsValid() ? status.speed : NAN;
}

int LinuxHAL::ConfigODL(int ratio) {
    char cmd[64];
    (void)ratio; // 软件倍率参数仅供兼容; 驱动配置完全按 Python 实测版执行
    std::lock_guard<std::mutex> lock(serial_mutex_);

    // ============================================================================
    // 初始化序列 (1:1 复刻 Python ODL_HAL.__init__, 实物闭环 10 小时稳定版):
    //   1 dev -> 1 ena -> 1 cfg crh=0 -> 1 cfg spd=-150000
    //   -> 1 cfg acc=1500000 -> 1 cfg dec=1500000
    //
    // 【严禁下发 cfg mcs】: 驱动器上电默认即为 1/32 微步模式, 与全部物理标定
    // (齿隙 40 步 / B0=0.0025 mV/步) 坐标系一致。下发 mcs 会改写细分模式,
    // 破坏物理当量。Python 实测版从不下发 mcs —— 本版修正对齐。
    //
    // 【缺失 spd/acc/dec 的历史教训】旧 C++ 版只发 dev/ena/crh/mcs,
    // 从未设置速度/加速度/减速度 —— 驱动器默认速度档不同会导致运动异常。
    // ============================================================================
    static const char* const INIT_SEQ[] = {
        "1 dev\n",
        "1 ena\n",
        "1 cfg crh=0\n",
        "1 cfg spd=-150000\n",
        "1 cfg acc=1500000\n",
        "1 cfg dec=1500000\n"
    };
    const int seq_len = static_cast<int>(sizeof(INIT_SEQ) / sizeof(INIT_SEQ[0]));
    for (int i = 0; i < seq_len; ++i) {
        snprintf(cmd, sizeof(cmd), "%s", INIT_SEQ[i]);
        const int cmd_len = static_cast<int>(strlen(cmd));
        const SerialWriteReport written = writeSerialUnlocked(
            reinterpret_cast<const uint8_t*>(cmd), cmd_len, 10);
        if (!written.IsComplete()) {
            fprintf(stderr,
                    "[LinuxHAL] FATAL: 初始化命令本地写入 outcome=%s "
                    "accepted=%d/%d stream=%s: %s",
                    SerialWriteOutcomeName(written.outcome),
                    written.accepted_bytes,written.requested_bytes,
                    ascii_stream_.IsTainted() ? "quarantined" : "clean",cmd);
            return -1;
        }
        usleep(50000);   // Python: time.sleep(0.05)
        // 指令与反馈一一对应: 丢弃本条命令的回波, 防止堆积污染后续 sts 解析
        if (tcflush(serial_fd_, TCIFLUSH) != 0) {
            fprintf(stderr, "[LinuxHAL] FATAL: 初始化反馈清理失败: %s\n",
                    strerror(errno));
            return -1;
        }
        parser_.Reset();
    }
    printf("[LinuxHAL] VSMD requested config: spd=-150000 pulse/s "
           "acc=1500000 pulse/s^2 dec=1500000 pulse/s^2; "
           "driver readback not yet available\n");
    return 0;
}

bool LinuxHAL::WaitForMoveComplete(int32_t timeout_ms) {
    // Macro moves are outside the 50 ms control loop.  Require two consecutive
    // command-after active status observations that both carry the VSMD stp
    // bit, near-zero speed, and the same driver-internal target coordinate.
    // This rejects the former single cached zero-speed false completion.  It
    // is still not independent mechanical arrival feedback.
    if (timeout_ms <= 0) return false;
    int32_t expected_position=0;
    {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        expected_position=last_sent_pos_;
    }
    const auto deadline=std::chrono::steady_clock::now()+
        std::chrono::milliseconds(timeout_ms);
    const int initial_wait_ms=timeout_ms<500?timeout_ms:500;
    usleep(static_cast<useconds_t>(initial_wait_ms)*1000);
    int consecutive=0;
    while (std::chrono::steady_clock::now()<deadline) {
        const auto remaining=std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline-std::chrono::steady_clock::now()).count();
        if (remaining<=0) break;
        const int query_ms=remaining<150?static_cast<int>(remaining):150;
        const MotorStatusSnapshot status=QueryMotorStatus(query_ms);
        if (DriverReportsStopped(status) &&
            status.driver_reported_position==expected_position) {
            if (++consecutive>=2) return true;
        } else {
            consecutive=0;
        }
        const auto after_query=std::chrono::steady_clock::now();
        if (after_query>=deadline) break;
        const auto wait_left=std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline-after_query).count();
        const int wait_ms=wait_left<200?static_cast<int>(wait_left):200;
        if (wait_ms>0) usleep(static_cast<useconds_t>(wait_ms)*1000);
    }
    return false;
}

HomingExecutionReport LinuxHAL::HomeWithSafety(
        int32_t timeout_ms,
        const volatile std::sig_atomic_t* running) {
    // VSMD v3.2 requires zero-stop before zero-start to clear bit 14.  After a
    // completed start, every timeout, failed active query, or interruption owns
    // a zero-stop-first abort.  All ASCII writes share ascii_stream_; a partial
    // line therefore prevents even the abort commands from being appended.
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(timeout_ms > 0 ? timeout_ms : 0);
    // Hold one transaction lock across zero-stop, its status query, zero-start
    // and any cleanup.  SendPosCmd uses try_lock and therefore fails without
    // emitting bytes instead of interleaving an absolute target in this flow.
    std::lock_guard<std::mutex> transaction_lock(serial_mutex_);
    const auto write_line = [this](const uint8_t* data, int len) {
        return writeSerialUnlocked(data,len,10);
    };
    const auto query_status = [this](int32_t query_timeout_ms) {
        return queryMotorStatusUnlocked(query_timeout_ms);
    };
    const auto deadline_expired = [&deadline]() {
        return std::chrono::steady_clock::now() >= deadline;
    };
    const auto interrupted = [running]() {
        return running && !*running;
    };
    const auto wait_poll = [&deadline,running]() {
        // Split the legacy 500 ms pacing interval so SIGINT is noticed within
        // at most one 50 ms slice instead of after the full 60 s home budget.
        for (int elapsed = 0; elapsed < 500; elapsed += 50) {
            if ((running && !*running) ||
                std::chrono::steady_clock::now() >= deadline) return;
            usleep(50000);
        }
    };

    motion_may_be_active_ = true;
    complete_stop_written_since_motion_ = false;
    driver_stop_confirmed_since_motion_ = false;
    homing_shutdown_handled_ = false;
    HomingExecutionReport report = ExecuteBoundedHoming(
        ascii_stream_,150,write_line,query_status,deadline_expired,
        interrupted,wait_poll);

    homing_shutdown_handled_ = report.shutdown_sequence_handled;
    if (report.driver_home_confirmed) {
        motion_may_be_active_ = false;
        driver_stop_confirmed_since_motion_ = true;
    } else if (HomingExecutionReportsDriverStopped(report)) {
        motion_may_be_active_ = false;
        driver_stop_confirmed_since_motion_ = true;
    } else {
        motion_may_be_active_ = report.zero_start_written ||
                                report.stop_state_unknown ||
                                ascii_stream_.IsTainted();
    }
    complete_stop_written_since_motion_ =
        report.failure_stop.zero_stop_write.IsComplete() ||
        report.failure_stop.ordinary_stop.command_write.IsComplete() ||
        report.preparation_recovery_stop.command_write.IsComplete();

    fprintf(stderr,
        "[Homing] result=%d prepare_zero_stop=%s(%d/%d) "
        "prepare_status=%s observed_after_query=%d speed=%.6g pos=%d flags=%u "
        "prepare_recovery_stop=%d confirmed=%d recovery_status=%s recovery_flags=%u "
        "zero_start=%s(%d/%d) queries=%d "
        "cleanup=%d abort_zero_stop=%s(%d/%d) ordinary_stop=%d "
        "driver_home_indicated=%d driver_internal_stop_indicated=%d "
        "stream=%s stop_unknown=%d hardware_intervention_required=%d\n",
        static_cast<int>(report.result),
        SerialWriteOutcomeName(report.preparation_zero_stop_write.outcome),
        report.preparation_zero_stop_write.accepted_bytes,
        report.preparation_zero_stop_write.requested_bytes,
        MotorStatusResultName(report.preparation_status.result),
        report.preparation_status.observed_after_active_query,
        static_cast<double>(report.preparation_status.speed),
        report.preparation_status.driver_reported_position,
        static_cast<unsigned>(report.preparation_status.status_flags),
        report.preparation_recovery_stop_attempted,
        report.preparation_recovery_stop.driver_stop_confirmed,
        MotorStatusResultName(report.preparation_recovery_stop.status.result),
        static_cast<unsigned>(report.preparation_recovery_stop.status.status_flags),
        SerialWriteOutcomeName(report.zero_start_write.outcome),
        report.zero_start_write.accepted_bytes,
        report.zero_start_write.requested_bytes,
        report.homing_status_queries,report.cleanup_attempted,
        SerialWriteOutcomeName(report.failure_stop.zero_stop_write.outcome),
        report.failure_stop.zero_stop_write.accepted_bytes,
        report.failure_stop.zero_stop_write.requested_bytes,
        report.failure_stop.ordinary_stop_attempted,
        report.driver_home_confirmed,
        HomingExecutionReportsDriverStopped(report),
        ascii_stream_.IsTainted() ? "quarantined" : "clean",
        report.stop_state_unknown,report.hardware_intervention_required);
    return report;
}

int LinuxHAL::Home(int32_t timeout_ms) {
    const HomingExecutionReport report=HomeWithSafety(timeout_ms,nullptr);
    return report.result==HomingExecutionResult::Complete ? 0 : -1;
}

void LinuxHAL::Release() {
    if (!initialized_ && !adc_initialized_ && !serial_open_) return;

    ReleaseFallbackDecision fallback;
    MotorStopReport fallback_stop;
    bool fallback_attempted=false;
    bool stream_tainted=false;
    {
        // Keep the fallback decision, optional stop transaction and fd close
        // atomic with respect to every other serial operation.
        std::lock_guard<std::mutex> lock(serial_mutex_);
        fallback = DecideReleaseFallback(
            ascii_stream_,serial_open_,motion_may_be_active_,
            complete_stop_written_since_motion_ || homing_shutdown_handled_,
            driver_stop_confirmed_since_motion_);
        if (fallback.attempt_stop) {
            fallback_stop=stopMotionUnlocked(MotorStopMode::Decelerated,250);
            fallback_attempted=true;
        }
        stream_tainted=ascii_stream_.IsTainted();
        closeSerial();
    }
    if (fallback_attempted) {
        fprintf(stderr,
                "[LinuxHAL] Release fallback mode=%s write=%s accepted=%d/%d "
                "driver_internal_stop_indicated=%d status=%s "
                "observed_after_active_query=%d "
                "stop_unknown=%d hardware_intervention_required=%d\n",
                MotorStopModeName(fallback_stop.mode),
                SerialWriteOutcomeName(fallback_stop.command_write.outcome),
                fallback_stop.command_write.accepted_bytes,
                fallback_stop.command_write.requested_bytes,
                fallback_stop.driver_stop_confirmed,
                MotorStatusResultName(fallback_stop.status.result),
                fallback_stop.status.observed_after_active_query,
                fallback_stop.stop_state_unknown,
                fallback_stop.hardware_intervention_required);
    } else if (fallback.stop_state_unknown || stream_tainted) {
        fprintf(stderr,
                "[LinuxHAL] Release sends no fallback bytes: action=%d "
                "stream=%s stop_state=UNKNOWN hardware_intervention_required=1; "
                "use the approved independent hardware stop/disable procedure, "
                "verify mechanical standstill, reset VSMD receive state, and "
                "re-home before reuse\n",
                static_cast<int>(fallback.action),
                stream_tainted ? "quarantined" : "clean");
    }
    if (adc_initialized_) {
        rp_AcqStop();
        rp_Release();
        adc_initialized_ = false;
    }
    initialized_ = false;
    printf("[LinuxHAL] 底层资源已释放；资源关闭本身不证明机械停止\n");
}

#endif // USE_REAL_HARDWARE
