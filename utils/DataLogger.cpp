#include "DataLogger.h"
#include "RunawayGuard.h"
#include <ctime>
#include <chrono>
#include <stdexcept>
#ifndef _WIN32
#include <thread>
#endif

#ifdef __linux__
#include <unistd.h>
#endif
#ifdef _WIN32
#include <io.h>
#endif
#ifdef _WIN32
#include <windows.h>
#endif

namespace {

const char* StatusQueryWriteOutcomeName(const MotorStatusSnapshot& status) {
    return SerialWriteOutcomeName(static_cast<SerialWriteOutcome>(
        status.active_query_write_outcome));
}

bool WriteSafetyRow(FILE* file,
                    const char* event,
                    const char* reason,
                    const char* operation_result,
                    const char* command,
                    const SerialWriteReport& write,
                    bool status_query_attempted,
                    const MotorStatusSnapshot& status,
                    bool driver_confirmation,
                    bool stop_state_unknown,
                    bool hardware_intervention_required) {
    if (!file) return false;
    return fprintf(file,
        "%s,%s,%s,%s,%s,%d,%d,%d,%d,%d,%s,%d,%s,%d,%d,%d,"
        "%.9g,%d,%u,%d,%d,%d,0\n",
        event ? event : "unknown",
        reason ? reason : "unspecified",
        operation_result ? operation_result : "unknown",
        command ? command : "none",
        SerialWriteOutcomeName(write.outcome),
        write.IsComplete(),
        write.requested_bytes,
        write.accepted_bytes,
        write.write_attempts,
        status_query_attempted,
        MotorStatusResultName(status.result),
        status.observed_after_active_query,
        StatusQueryWriteOutcomeName(status),
        status.active_query_requested_bytes,
        status.active_query_accepted_bytes,
        status.active_query_write_attempts,
        static_cast<double>(status.speed),
        status.driver_reported_position,
        static_cast<unsigned>(status.status_flags),
        driver_confirmation,
        stop_state_unknown,
        hardware_intervention_required) >= 0;
}

void SleepWriterPoll() {
#ifdef _WIN32
    Sleep(1);
#else
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
#endif
}

} // namespace

DataLogger::DataLogger()
    : fp_(nullptr)
    , counter_(0)
    , flush_interval_(200)
    , is_open_(false)
{
    memset(filename_, 0, sizeof(filename_));
}

int DataLogger::Open(const char* algo, int duration_min) {
    if (is_open_) {
        Close();
    }

    // 构造文件名: Data_<实验时长>_<算法>_<启动时间>.csv
    // 例: 60 分钟 + PI + 14日15:30:12 -> Data_60_PI_20260914_153012.csv
    char time_str[24];
    time_t now = time(nullptr);
    strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", localtime(&now));
    static unsigned sequence=0;
    const auto tick=std::chrono::steady_clock::now().time_since_epoch().count();
    snprintf(filename_, sizeof(filename_),
             "Data_%d_%s_%s_%lld_%u.csv",duration_min,algo,time_str,
             static_cast<long long>(tick),++sequence);

    // 打开文件 (覆盖写入)
    fp_ = fopen(filename_, "w");
    if (!fp_) {
        perror("DataLogger: 无法创建日志文件");
        return -1;
    }

    // Open the safety ledger at the same time as the raw log.  Homing happens
    // before OpenDiagnostics(), so creating this file later would lose the
    // most important startup-failure evidence.
    char safety_name[256];
    snprintf(safety_name,sizeof(safety_name),"%s.safety.csv",filename_);
    safety_=fopen(safety_name,"w");
    if(!safety_) {
        perror("DataLogger: 无法创建安全事件日志");
        fclose(fp_);
        fp_=nullptr;
        healthy_=false;
        return -1;
    }

    // 写入 CSV 头部
    // The first six columns describe the sample and controller proposal; tools
    // that select them by name.  Appended fields remove the ambiguity of a
    // fault row whose legacy position columns contain an inhibited proposal.
    counter_ = 0;
    healthy_=true;
    queue_head_.store(0,std::memory_order_relaxed);
    queue_tail_.store(0,std::memory_order_relaxed);
    flush_requested_.store(0,std::memory_order_relaxed);
    flush_completed_.store(0,std::memory_order_relaxed);
    writer_stop_.store(false,std::memory_order_relaxed);
    writer_started_=false;
    last_embedded_cycle_valid_=false;
    is_open_ = true;
    const bool headers_written=
        fprintf(fp_, "Elapsed_s,V_PHS_mV,Temp_C,Logical_Pos,Physical_Pos,State,Proposed_logical_target,Proposed_physical_command,Last_accepted_logical_target,Last_accepted_physical_command,Send_result,Send_accepted,Driver_status_available,Driver_status_age_cycles,Driver_reported_speed,Driver_reported_position,Driver_status_flags,Position_write_outcome,Position_write_complete,Position_write_requested_bytes,Position_write_host_accepted_bytes,Position_write_attempts\n")>=0 &&
        fprintf(safety_,"Event,Reason,Operation_result,Command,Write_outcome,Host_write_complete,Requested_bytes,Host_accepted_bytes,Write_attempts,Status_query_attempted,Status_result,Status_observed_after_active_query,Status_query_write_outcome,Status_query_requested_bytes,Status_query_host_accepted_bytes,Status_query_write_attempts,Driver_reported_speed,Driver_reported_position,Driver_status_flags,Driver_internal_stop_indicated,Final_stop_state_UNKNOWN,Final_hardware_intervention_required,Mechanical_stop_independently_verified\n")>=0;
    if(!headers_written || fflush(fp_)!=0 || fflush(safety_)!=0) {
        healthy_=false;
        Close();
        return -1;
    }
    printf("[DataLogger] 日志文件: %s\n", filename_);
    return 0;
}

void DataLogger::Log(double elapsed_s, float vphs_mv, float temp_c,
                     float logical_pos, int32_t physical_pos,
                     const char* state_str,
                     const CycleDiagnostics* diagnostics) {
    if (!fp_ || !is_open_) return;
    QueuedCycle row;
    row.elapsed_s=elapsed_s;
    row.vphs_mv=vphs_mv;
    row.temp_c=temp_c;
    row.logical_pos=logical_pos;
    row.physical_pos=physical_pos;
    if(state_str) {
        std::strncpy(row.state,state_str,sizeof(row.state)-1);
        row.state[sizeof(row.state)-1]='\0';
    }
    row.write_raw=true;
    if(diagnostics) {
        row.diagnostics=*diagnostics;
        row.write_diagnostics=true;
        last_embedded_cycle_=diagnostics->cycle;
        last_embedded_cycle_valid_=true;
    } else {
        last_embedded_cycle_valid_=false;
    }
    if(Enqueue(row)) ++counter_;
    else healthy_=false;
}

bool DataLogger::LogCycle(double elapsed_s,float vphs_mv,float temp_c,
                          float logical_pos,int32_t physical_pos,
                          const char* state_str,
                          const CycleDiagnostics& diagnostics) {
    if(!fp_ || !is_open_) return false;
    QueuedCycle row;
    row.elapsed_s=elapsed_s;
    row.vphs_mv=vphs_mv;
    row.temp_c=temp_c;
    row.logical_pos=logical_pos;
    row.physical_pos=physical_pos;
    if(state_str) {
        std::strncpy(row.state,state_str,sizeof(row.state)-1);
        row.state[sizeof(row.state)-1]='\0';
    }
    row.diagnostics=diagnostics;
    row.write_raw=true;
    row.write_diagnostics=true;
    last_embedded_cycle_valid_=false;
    const bool accepted=Enqueue(row);
    if(accepted) ++counter_;
    else healthy_=false;
    return accepted;
}

bool DataLogger::Enqueue(const QueuedCycle& row) {
    if(!writer_started_ || !healthy_.load(std::memory_order_acquire)) return false;
    const uint32_t head=queue_head_.load(std::memory_order_relaxed);
    const uint32_t tail=queue_tail_.load(std::memory_order_acquire);
    if(head-tail>=QUEUE_CAPACITY) {
        healthy_=false;
        return false;
    }
    queue_[head%QUEUE_CAPACITY]=row;
    queue_head_.store(head+1,std::memory_order_release);
    return true;
}

void DataLogger::WriteRawRow(const QueuedCycle& row) {
    const CycleDiagnostics* diagnostics=row.write_diagnostics
        ? &row.diagnostics : nullptr;
    const double proposed_logical=diagnostics
        ? diagnostics->proposed_logical : static_cast<double>(row.logical_pos);
    const int32_t proposed_physical=diagnostics
        ? diagnostics->proposed_physical : row.physical_pos;
    const double accepted_logical=diagnostics
        ? diagnostics->accepted_logical : static_cast<double>(row.logical_pos);
    const int32_t accepted_physical=diagnostics
        ? diagnostics->accepted_physical : row.physical_pos;
    const int send_result=diagnostics ? diagnostics->send_result : 0;
    const int send_accepted=diagnostics ? diagnostics->send_accepted : 0;
    const int driver_available=diagnostics
        ? diagnostics->driver_status_available : 0;
    const uint64_t driver_age=diagnostics
        ? diagnostics->driver_status_age_cycles : 0;
    const MotorStatusSnapshot driver=diagnostics
        ? diagnostics->driver_status : MotorStatusSnapshot();
    const SerialWriteReport position_write=diagnostics
        ? diagnostics->position_write : SerialWriteReport();
    if(fprintf(fp_, "%.3f,%.4f,%.2f,%.1f,%d,%s,%.9g,%d,%.9g,%d,%d,%d,%d,%llu,%.9g,%d,%u,%s,%d,%d,%d,%d\n",
            row.elapsed_s,static_cast<double>(row.vphs_mv),
            static_cast<double>(row.temp_c),static_cast<double>(row.logical_pos),
            static_cast<int>(row.physical_pos),row.state,proposed_logical,
            static_cast<int>(proposed_physical),accepted_logical,
            static_cast<int>(accepted_physical),send_result,send_accepted,
            driver_available,static_cast<unsigned long long>(driver_age),
            static_cast<double>(driver.speed),
            static_cast<int>(driver.driver_reported_position),
            static_cast<unsigned>(driver.status_flags),
            SerialWriteOutcomeName(position_write.outcome),
            position_write.IsComplete(),position_write.requested_bytes,
            position_write.accepted_bytes,position_write.write_attempts)<0)
        healthy_=false;
}

void DataLogger::WriteDiagnosticRow(const CycleDiagnostics& r) {
    if(!diagnostics_) { healthy_=false; return; }
    const auto& d=r.control;
    if(fprintf(diagnostics_,"%llu,%.9f,%.9f,%.3f,%.3f,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%d,%.9g,%d,%.9g,%.9g,%s,%.9g,%d,%d,%lld,%lld,%lld,%d,%d,%llu,%.9g,%d,%u,%d,%d,%d,%d,%.9g,%d,%u,%s,%d,%d,%d,%d,%s,%d,%d,%d,%d,%d,%s,%d,%d,%d,%d,%d,0\n",
        static_cast<unsigned long long>(r.cycle),r.monotonic_s,r.sample_dt_s,r.compute_us,r.previous_cycle_us,r.deadline_late,
        d.raw_error,d.integrator,d.requested_u,d.submitted_u,d.physical_delta,r.send_result,r.send_accepted,r.accepted_logical,r.accepted_physical,
        d.lower,d.upper,PIStatusName(r.status),
        r.proposed_logical,r.proposed_physical,r.proposed_physical_delta,
        static_cast<long long>(r.guard_proposed_directional_steps),
        static_cast<long long>(r.guard_accepted_directional_steps),
        static_cast<long long>(r.guard_directional_limit_steps),
        static_cast<int>(r.driver_poll_result),r.driver_status_available,
        static_cast<unsigned long long>(r.driver_status_age_cycles),
        static_cast<double>(r.driver_status.speed),r.driver_status.driver_reported_position,
        r.driver_status.status_flags,static_cast<int>(r.fault_stop.mode),
        r.fault_stop.command_written,static_cast<int>(r.fault_stop.status.result),
        r.fault_stop.driver_stop_confirmed,static_cast<double>(r.fault_stop.status.speed),
        r.fault_stop.status.driver_reported_position,r.fault_stop.status.status_flags,
        SerialWriteOutcomeName(r.position_write.outcome),r.position_write.IsComplete(),
        r.position_write.requested_bytes,r.position_write.accepted_bytes,r.position_write.write_attempts,
        SerialWriteOutcomeName(r.fault_stop.command_write.outcome),
        r.fault_stop.command_write.requested_bytes,r.fault_stop.command_write.accepted_bytes,
        r.fault_stop.command_write.write_attempts,r.fault_stop.status_query_attempted,
        r.fault_stop.status.observed_after_active_query,StatusQueryWriteOutcomeName(r.fault_stop.status),
        r.fault_stop.status.active_query_requested_bytes,
        r.fault_stop.status.active_query_accepted_bytes,
        r.fault_stop.status.active_query_write_attempts,r.fault_stop.stop_state_unknown,
        r.fault_stop.hardware_intervention_required)<0) healthy_=false;
}

void DataLogger::FlushCycleStreams() {
    if(!fp_ || !is_open_) return;
    if(fflush(fp_)!=0) healthy_=false;
    if(diagnostics_ && fflush(diagnostics_)!=0) healthy_=false;
#ifdef __linux__
    if(fsync(fileno(fp_))!=0) healthy_=false;
    if(diagnostics_ && fsync(fileno(diagnostics_))!=0) healthy_=false;
#else
    #ifdef _WIN32
    if(_commit(_fileno(fp_))!=0) healthy_=false;
    if(diagnostics_ && _commit(_fileno(diagnostics_))!=0) healthy_=false;
    #endif
#endif
}

void DataLogger::FlushSafetyStream() {
    if(!safety_) return;
    if(fflush(safety_)!=0) healthy_=false;
#ifdef __linux__
    if(fsync(fileno(safety_))!=0) healthy_=false;
#else
    #ifdef _WIN32
    if(_commit(_fileno(safety_))!=0) healthy_=false;
    #endif
#endif
}

void DataLogger::WriterLoop() {
    uint64_t raw_rows=0;
    for(;;) {
        const uint32_t tail=queue_tail_.load(std::memory_order_relaxed);
        const uint32_t head=queue_head_.load(std::memory_order_acquire);
        if(tail!=head) {
            const QueuedCycle& row=queue_[tail%QUEUE_CAPACITY];
            if(row.write_raw) {
                WriteRawRow(row);
                ++raw_rows;
            }
            if(row.write_diagnostics) WriteDiagnosticRow(row.diagnostics);
            queue_tail_.store(tail+1,std::memory_order_release);
            if(flush_interval_>0 && raw_rows>0 && raw_rows%flush_interval_==0)
                FlushCycleStreams();
            continue;
        }
        const uint32_t requested=flush_requested_.load(std::memory_order_acquire);
        const uint32_t completed=flush_completed_.load(std::memory_order_relaxed);
        if(requested!=completed) {
            FlushCycleStreams();
            flush_completed_.store(requested,std::memory_order_release);
            continue;
        }
        if(writer_stop_.load(std::memory_order_acquire)) break;
        SleepWriterPoll();
    }
    FlushCycleStreams();
    flush_completed_.store(flush_requested_.load(std::memory_order_acquire),
                            std::memory_order_release);
}

void DataLogger::Flush() {
    if (!fp_ || !is_open_) return;
    if(writer_started_ && WriterJoinable()) {
        const uint32_t requested=flush_requested_.fetch_add(1,std::memory_order_acq_rel)+1;
        while(flush_completed_.load(std::memory_order_acquire)!=requested &&
              !writer_stop_.load(std::memory_order_acquire))
            SleepWriterPoll();
    } else {
        FlushCycleStreams();
    }
    FlushSafetyStream();
}

void DataLogger::Close() {
    if (fp_ && is_open_) {
        if(writer_started_ && WriterJoinable()) {
            // Shutdown is outside the control cycle. Let the writer drain the
            // bounded queue and sync its owned streams before closing them.
            writer_stop_.store(true,std::memory_order_release);
            JoinWriter();
            writer_started_=false;
        } else {
            FlushCycleStreams();
        }
        FlushSafetyStream();
        if(diagnostics_) {
            if(fclose(diagnostics_)!=0) healthy_=false;
            diagnostics_=nullptr;
        }
        if(safety_) {
            if(fclose(safety_)!=0) healthy_=false;
            safety_=nullptr;
        }
        if(fclose(fp_)!=0) healthy_=false;
        fp_ = nullptr;
        is_open_ = false;
        printf("[DataLogger] 日志文件已关闭 (%d 条记录)\n", counter_);
    } else if(WriterJoinable()) {
        writer_stop_.store(true,std::memory_order_release);
        JoinWriter();
        writer_started_=false;
    }
}

bool DataLogger::Healthy() const {
    return healthy_.load(std::memory_order_acquire);
}
bool DataLogger::OpenDiagnostics(const SystemConfig& cfg,double gain,double polarity,int direction,const char* path) {
    if(!is_open_ || diagnostics_) return false;
    if(!path) { healthy_=false; return false; }
#ifdef _WIN32
    // Windows narrow argv uses the active ANSI code page. Metadata is UTF-8,
    // while fopen still receives the native path used by ConfigParser.
    wchar_t wide_path[1024];
    char utf8_path[3072];
    if(!MultiByteToWideChar(CP_ACP,0,path,-1,wide_path,1024) ||
       !WideCharToMultiByte(CP_UTF8,0,wide_path,-1,utf8_path,3072,nullptr,nullptr)) {
        healthy_=false; return false;
    }
    const char* metadata_path=utf8_path;
#else
    const char* metadata_path=path;
#endif
    char name[256];
    snprintf(name,sizeof(name),"%s.diagnostics.csv",filename_);
    diagnostics_=fopen(name,"w"); if(!diagnostics_) return false;
    if(fprintf(diagnostics_,"Cycle,Monotonic_s,Sample_dt_s,Compute_us,Previous_cycle_total_us,Deadline_late,Raw_error_mV,Integrator_steps,Requested_u_steps,Submitted_u_steps,Physical_delta_steps,Send_result,Send_accepted,Accepted_logical_target,Accepted_physical_command,Lower_u,Upper_u,Controller_status,Proposed_logical_target,Proposed_physical_command,Proposed_physical_delta_steps,Guard_proposed_directional_steps,Guard_accepted_directional_steps,Guard_directional_limit_steps,Driver_poll_result,Driver_status_available,Driver_status_age_cycles,Driver_reported_speed,Driver_reported_position,Driver_status_flags,Fault_stop_mode,Fault_stop_command_written,Fault_stop_status_result,Fault_stop_driver_confirmed,Fault_stop_driver_speed,Fault_stop_driver_position,Fault_stop_status_flags,Position_write_outcome,Position_write_complete,Position_write_requested_bytes,Position_write_host_accepted_bytes,Position_write_attempts,Fault_stop_command_write_outcome,Fault_stop_command_requested_bytes,Fault_stop_command_host_accepted_bytes,Fault_stop_command_write_attempts,Fault_stop_status_query_attempted,Fault_stop_status_observed_after_active_query,Fault_stop_status_query_write_outcome,Fault_stop_status_query_requested_bytes,Fault_stop_status_query_host_accepted_bytes,Fault_stop_status_query_write_attempts,Fault_stop_final_state_UNKNOWN,Fault_stop_final_hardware_intervention_required,Mechanical_stop_independently_verified\n")<0) healthy_=false;
    snprintf(name,sizeof(name),"%s.metadata.txt",filename_);
    FILE* meta=fopen(name,"w"); if(!meta) return false;
    fprintf(meta,"VERSION=%s\nCONTROLLER=PI\nCONFIG_PATH=%s\nCALIBRATED_B0=%.12g\nPOLARITY=%.0f\nINITIAL_DIRECTION=%d\nEFFECTIVE_LOGICAL_LIMIT=%d\nMAX_PHYSICAL_DELTA=%d\nBACKLASH=%d\nTS=%.12g\nKP=%.12g\nKI=%.12g\n",SOFTWARE_VERSION,metadata_path,gain,polarity,direction,cfg.EffectiveLogicalLimit(),cfg.max_physical_delta,cfg.backlash_steps,1.0/cfg.control_freq_hz,double(cfg.kp),double(cfg.ki));
#if USE_REAL_HARDWARE
    fprintf(meta,"DATA_ORIGIN=instrument_interface\n");
#else
    fprintf(meta,"DATA_ORIGIN=simulation\n");
#endif
    fprintf(meta,"POSITIONS=command targets, not independent mechanical feedback\n");
    fprintf(meta,"RAW_POSITIONS=proposed command targets, including inhibited fault rows; never driver or independent mechanical feedback\n");
    fprintf(meta,"ACCEPTED_POSITIONS=last software-accepted targets; complete host-side serial write or hold only, not VSMD parse, execution, arrival, or mechanical proof\n");
    fprintf(meta,"DRIVER_POSITIONS=VSMD internal status coordinates; not independent mechanical feedback\n");
    fprintf(meta,"DRIVER_STATUS_CODES=1:valid,2:no_data,3:busy,0:not_attempted,-1:timeout,-2:write_failure,-3:read_failure,-4:protocol_error,-5:unsupported,-6:stream_tainted\n");
    fprintf(meta,"SERIAL_WRITE_OUTCOMES=complete means only that the host accepted the full ASCII line within the write budget; complete_late means the full line was host-accepted after that budget and is not cycle-compliant; neither proves VSMD parsing or execution; partial/timeout/error/blocked outcomes include requested and host-accepted byte counts\n");
    fprintf(meta,"SERIAL_PARTIAL_POLICY=any non-empty partial ASCII line permanently quarantines this process stream; no newline, status query, retry, zero-stop, or stp is appended; stop state remains UNKNOWN and out-of-band hardware intervention is required\n");
    fprintf(meta,"POSITION_WRITE_EVIDENCE=Position_write fields describe host-side bytes only and never prove VSMD parsing, motion, target arrival, or independent mechanical position\n");
    fprintf(meta,"GUARD_DIRECTIONAL_LIMIT_STEPS=%lld\nGUARD_LIMIT_QUALIFICATION=200000-step software threshold; not hardware-safety certified\n",
            static_cast<long long>(RunawayGuard::DisplacementLimit()));
    fprintf(meta,"DRIVER_CONFIG_INTENT=spd_abs:150000 pulse/s,acc:1500000 pulse/s^2,dec:1500000 pulse/s^2\nDRIVER_CONFIG_READBACK=not available; intent values are not proof of active driver settings\n");
    fprintf(meta,"FAULT_STOP=on a clean ASCII stream, addressed '1 stp 1' requests immediate stop; host write, a VSMD frame observed after an active status query, internal stop indication, UNKNOWN state, and hardware-intervention requirement are logged separately; frame attribution is unverified\n");
    fprintf(meta,"HOMING_FAILURE_STOP=after zero-start failure, addressed '1 zero stop' is attempted first on a clean stream, followed by an active status query; ordinary addressed '1 stp 1' is conditional and has its own write/query evidence; returned-frame attribution is unverified\n");
    fprintf(meta,"SAFETY_LOG=.safety.csv opens before homing; complete writes are not driver confirmation, and Mechanical_stop_independently_verified remains 0 because VSMD status is not independent feedback\n");
    fprintf(meta,"DRIVER_STATUS_LIMITATIONS=VSMD internal snapshot observed after an active query but unassociated because the ASCII protocol has no request identifier; structural frame checks only, BCC is not validated; never independent mechanical feedback\n");
    fprintf(meta,"LOGGER_QUEUE_CAPACITY=%llu cycle records; control-thread enqueue is bounded and non-waiting; queue saturation or writer I/O failure raises a logging fault\n",
            static_cast<unsigned long long>(QUEUE_CAPACITY));
    fprintf(meta,"LOGGER_DURABILITY=cycle CSV writes and periodic fflush/fsync run on a dedicated writer thread; shutdown drains and syncs; abrupt power loss may lose records since the last writer sync\n");
    fprintf(meta,"TIMING=sample timestamp is ADC read completion; previous-cycle total includes queue submission and console calls but excludes background file writes/fsync; final cycle total is not available in the next row\n--- resolved PI configuration ---\n");
    fprintf(meta,"TARGET_VPHS_MV=%.12g\nDEADBAND_MV=%.12g\nPOS_MIN_LIMIT=%d\nPOS_MAX_LIMIT=%d\nPOS_CENTER=%d\n",double(cfg.target_vphs_mv),double(cfg.deadband_mv),cfg.pos_min_limit,cfg.pos_max_limit,cfg.pos_center);
    if(ferror(meta)) healthy_=false;
    if(ferror(diagnostics_)) healthy_=false;
    if(fclose(meta)!=0) healthy_=false;
    if(!Healthy()) return false;
    writer_stop_.store(false,std::memory_order_relaxed);
    try {
#ifdef _WIN32
        writer_handle_=_beginthreadex(nullptr,0,&DataLogger::WriterThreadEntry,this,0,nullptr);
        if(writer_handle_==0) throw std::runtime_error("logger thread start failed");
#else
        writer_=std::thread(&DataLogger::WriterLoop,this);
#endif
        writer_started_=true;
    } catch(...) {
        healthy_=false;
        return false;
    }
    return Healthy();
}

bool DataLogger::WriterJoinable() const {
#ifdef _WIN32
    return writer_handle_!=0;
#else
    return writer_.joinable();
#endif
}

void DataLogger::JoinWriter() {
#ifdef _WIN32
    if(writer_handle_!=0) {
        HANDLE handle=reinterpret_cast<HANDLE>(writer_handle_);
        WaitForSingleObject(handle,INFINITE);
        CloseHandle(handle);
        writer_handle_=0;
    }
#else
    if(writer_.joinable()) writer_.join();
#endif
}

#ifdef _WIN32
unsigned __stdcall DataLogger::WriterThreadEntry(void* context) {
    static_cast<DataLogger*>(context)->WriterLoop();
    return 0;
}
#endif
void DataLogger::LogDiagnostics(const CycleDiagnostics& r) {
    if(last_embedded_cycle_valid_ && last_embedded_cycle_==r.cycle) {
        last_embedded_cycle_valid_=false;
        return;
    }
    QueuedCycle row;
    row.diagnostics=r;
    row.write_diagnostics=true;
    if(!Enqueue(row)) healthy_=false;
}

bool DataLogger::LogMotorStopReport(const char* reason,
                                    const MotorStopReport& report) {
    if(!is_open_ || !safety_) {
        healthy_=false;
        return false;
    }
    const char* command=report.mode==MotorStopMode::Immediate
        ? "1 stp 1\\n" : "1 stp 0\\n";
    if(!WriteSafetyRow(safety_,"motor_stop",reason,"stop_attempt",command,
                       report.command_write,report.status_query_attempted,
                       report.status,report.driver_stop_confirmed,
                       report.stop_state_unknown,
                       report.hardware_intervention_required)) {
        healthy_=false;
    }
    // Keep fault evidence durable without waiting for a possibly slow cycle
    // CSV fsync worker after the stop command has already been issued.
    FlushSafetyStream();
    return Healthy();
}

bool DataLogger::LogHomingReport(const HomingExecutionReport& report) {
    if(!is_open_ || !safety_) {
        healthy_=false;
        return false;
    }
    const char* result=HomingExecutionResultName(report.result);
    bool ok=true;
    ok=WriteSafetyRow(
        safety_,"homing_prepare_zero_stop",result,result,"1 zero stop\\n",
        report.preparation_zero_stop_write,
        report.preparation_status_query_attempted,
        report.preparation_status,
        DriverReportsReadyAfterZeroStop(report.preparation_status),
        report.stop_state_unknown,
        report.hardware_intervention_required) && ok;
    if(report.preparation_recovery_stop_attempted) {
        const bool ready_after_stop =
            report.preparation_recovery_stop.driver_stop_confirmed &&
            DriverReportsReadyAfterZeroStop(
                report.preparation_recovery_stop.status);
        ok=WriteSafetyRow(
            safety_,"homing_prepare_recovery_stop","stale_zero_flag",
            ready_after_stop ? "ready_after_stop" : "not_ready_after_stop",
            "1 stp 1\\n",report.preparation_recovery_stop.command_write,
            report.preparation_recovery_stop.status_query_attempted,
            report.preparation_recovery_stop.status,
            report.preparation_recovery_stop.driver_stop_confirmed,
            report.preparation_recovery_stop.stop_state_unknown,
            report.preparation_recovery_stop.hardware_intervention_required) && ok;
    }
    ok=WriteSafetyRow(
        safety_,"homing_zero_start",result,result,"1 zero start\\n",
        report.zero_start_write,
        report.homing_status_queries>0,
        report.completion_status,
        report.driver_home_confirmed,
        report.stop_state_unknown,
        report.hardware_intervention_required) && ok;
    if(!report.failure_stop.zero_stop_reused_from_preparation) {
        ok=WriteSafetyRow(
            safety_,"homing_failure_zero_stop",result,result,"1 zero stop\\n",
            report.failure_stop.zero_stop_write,
            report.failure_stop.post_zero_status_query_attempted,
            report.failure_stop.post_zero_status,
            DriverReportsReadyAfterZeroStop(report.failure_stop.post_zero_status),
            report.stop_state_unknown,
            report.hardware_intervention_required) && ok;
    }
    ok=WriteSafetyRow(
        safety_,"homing_failure_ordinary_stop",result,result,"1 stp 1\\n",
        report.failure_stop.ordinary_stop.command_write,
        report.failure_stop.ordinary_stop.status_query_attempted,
        report.failure_stop.ordinary_stop.status,
        report.failure_stop.ordinary_stop.driver_stop_confirmed,
        report.stop_state_unknown,
        report.hardware_intervention_required) && ok;
    if(!ok) healthy_=false;
    FlushSafetyStream();
    return Healthy();
}
