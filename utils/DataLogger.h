#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <atomic>
#ifdef _WIN32
#include <process.h>
#else
#include <thread>
#endif
#include "../config/SystemConfig.h"
#include "PIDiagnostics.h"
#include "../hal/MotorSafety.h"

struct CycleDiagnostics {
    uint64_t cycle=0;
    double monotonic_s=0, sample_dt_s=0, compute_us=0, previous_cycle_us=0;
    bool deadline_late=false;
    int send_result=0; // 0 hold/not sent, 1 full software send, -1 failure, -2 inhibited
    bool send_accepted=false;
    int32_t accepted_physical=0;
    double accepted_logical=0;
    int32_t proposed_physical=0;
    double proposed_logical=0;
    int32_t proposed_physical_delta=0;
    int64_t guard_proposed_directional_steps=0;
    int64_t guard_accepted_directional_steps=0;
    int64_t guard_directional_limit_steps=0;
    MotorStatusResult driver_poll_result=MotorStatusResult::NotAttempted;
    bool driver_status_available=false;
    uint64_t driver_status_age_cycles=0;
    MotorStatusSnapshot driver_status;
    // Host-side write evidence for this cycle's absolute position command.
    // Complete only means the local serial driver accepted the whole ASCII
    // line; it is not VSMD parse, execution, arrival, or mechanical proof.
    SerialWriteReport position_write;
    MotorStopReport fault_stop;
    PIStatus status=PIStatus::Ready;
    PIDiagnostics control;
};

// ============================================================================
// DataLogger: CSV 日志记录器
//
// 特点:
//   - 实时调用者仅向固定容量 SPSC 队列复制记录，不做文件 I/O 或等待
//   - 独立线程写原始/诊断 CSV，并按 FLUSH_INTERVAL 刷盘
//   - 队列满或后台 I/O 出错时 Healthy() 置假，由控制程序按故障路径停机
//   - 记录控制器提案、软件接受值和驱动报告值
// ============================================================================

// 状态字符串最大长度
static const int STATE_STR_LEN = 16;

// 最大文件名长度
static const int LOG_FILENAME_LEN = 128;

class DataLogger {
public:
    DataLogger();
    ~DataLogger() { Close(); }
    bool OpenDiagnostics(const SystemConfig& cfg,double gain,double polarity,int direction,const char* config_path);
    void LogDiagnostics(const CycleDiagnostics& row);
    // Safety events are available immediately after Open(), before homing and
    // before the per-cycle diagnostics file can be opened.  Each call is
    // synchronously flushed so an early startup failure remains inspectable.
    bool LogMotorStopReport(const char* reason,const MotorStopReport& report);
    bool LogHomingReport(const HomingExecutionReport& report);
    bool Healthy() const;
    const char* Filename() const { return filename_; }

    // --- 打开日志文件 (构造文件名, 写 CSV 头部) ---
    // algo: "PI"
    // duration_min: 实验时长 (分钟, 来自 TOTAL_DURATION_MIN), 用于文件名
    int Open(const char* algo, int duration_min);

    // --- 写入一行日志 ---
    void Log(double elapsed_s, float vphs_mv, float temp_c,
             float logical_pos, int32_t physical_pos,
             const char* state_str,
             const CycleDiagnostics* diagnostics=nullptr);
    // Submit both raw and diagnostic rows as one bounded, nonblocking queue
    // operation. A false result means the writer queue is full or unavailable;
    // callers must treat that as a logging fault rather than block the cycle.
    bool LogCycle(double elapsed_s,float vphs_mv,float temp_c,
                  float logical_pos,int32_t physical_pos,const char* state_str,
                  const CycleDiagnostics& diagnostics);

    // --- 强制刷盘 ---
    void Flush();

    // --- 关闭日志 ---
    void Close();

    // --- 设置刷盘间隔 ---
    void SetFlushInterval(int interval) { flush_interval_ = interval; }

    // --- 获取当前拍数 ---
    int GetCounter() const { return counter_; }

private:
    friend struct DataLoggerTestAccess;
    static_assert(ATOMIC_INT_LOCK_FREE==2,
                  "real-time log queue requires lock-free 32-bit atomics");
    struct QueuedCycle {
        double elapsed_s=0;
        float vphs_mv=0, temp_c=0, logical_pos=0;
        int32_t physical_pos=0;
        char state[STATE_STR_LEN]={};
        CycleDiagnostics diagnostics;
        bool write_raw=false;
        bool write_diagnostics=false;
    };
    static constexpr uint32_t QUEUE_CAPACITY=512;
    bool Enqueue(const QueuedCycle& row);
    void WriterLoop();
    void WriteRawRow(const QueuedCycle& row);
    void WriteDiagnosticRow(const CycleDiagnostics& row);
    void FlushCycleStreams();
    void FlushSafetyStream();
    bool WriterJoinable() const;
    void JoinWriter();
#ifdef _WIN32
    static unsigned __stdcall WriterThreadEntry(void* context);
#endif

    FILE*   fp_;
    char    filename_[LOG_FILENAME_LEN];
    int     counter_;
    int     flush_interval_;
    bool    is_open_;
    FILE* diagnostics_=nullptr;
    FILE* safety_=nullptr;
    std::atomic<uint32_t> healthy_{1};
    QueuedCycle queue_[QUEUE_CAPACITY];
    std::atomic<uint32_t> queue_head_{0},queue_tail_{0};
    std::atomic<uint32_t> flush_requested_{0},flush_completed_{0};
    std::atomic<uint32_t> writer_stop_{0};
#ifdef _WIN32
    uintptr_t writer_handle_=0;
#else
    std::thread writer_;
#endif
    bool writer_started_=false;
    uint64_t last_embedded_cycle_=0;
    bool last_embedded_cycle_valid_=false;
};

#endif // DATA_LOGGER_H
