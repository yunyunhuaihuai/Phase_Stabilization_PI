#include "SimHAL.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <thread>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif
int SimHAL::Init() {
    initialized_=true; position_=target_=1056000; last_=Clock::now();
    rng_.seed(41024); normal_.reset();
    std::printf("[SimHAL] time-driven SIL; gain=0.0025 (simulation assumption)\n");
    return 0;
}
void SimHAL::Advance(double dt) {
    if(!initialized_ || !std::isfinite(dt) || dt<=0) return;
    disturbance_seconds_+=dt;
    const double diff=target_-position_;
    const double amount=std::min(std::fabs(diff)*(1-std::exp(-dt/0.04)),150000*dt);
    position_+=(diff<0?-amount:amount);
    if(std::fabs(target_-position_)<0.01) position_=target_;
}
void SimHAL::Update() {
    if(manual_) return;
    auto now=Clock::now();
    Advance(std::chrono::duration<double>(now-last_).count()); last_=now;
}
float SimHAL::ReadVPHS() {
    Update();
    if(!initialized_) return NAN;
    return static_cast<float>((position_-1056000)*0.0025+noise_sd_*normal_(rng_)+
        disturbance_mv_*std::sin(2*3.141592653589793*.4*disturbance_seconds_));
}
bool SimHAL::SendPosCmd(int32_t pos) {
    Update();
    char cmd[64];
    const int len=std::snprintf(cmd,sizeof(cmd),"1 pos %d\n",pos);
    last_position_write_=SerialWriteReport();
    last_position_write_.requested_bytes=len>0?len:0;
    last_position_write_.write_attempts=1;
    if(!initialized_ || pos<0 || pos>2112000 || len<=0) {
        last_position_write_.outcome=SerialWriteOutcome::ErrorNoBytes;
        return false;
    }
    last_position_write_.accepted_bytes=len;
    last_position_write_.outcome=SerialWriteOutcome::Complete;
    target_=pos; return true;
}
float SimHAL::GetMotorSpeed() {
    Update(); return position_==target_?0.0f:static_cast<float>((target_-position_)/0.04/3200);
}
int32_t SimHAL::GetReportedPos() { Update(); return static_cast<int32_t>(std::round(position_)); }
MotorStatusSnapshot SimHAL::PollMotorStatus() {
    MotorStatusSnapshot status=QueryMotorStatus(0);
    status.observed_after_active_query=false;
    status.active_query_requested_bytes=0;
    status.active_query_accepted_bytes=0;
    status.active_query_write_attempts=0;
    status.active_query_write_outcome=0;
    return status;
}
MotorStatusSnapshot SimHAL::QueryMotorStatus(int32_t) {
    Update();
    MotorStatusSnapshot status;
    if(!initialized_) {
        status.result=MotorStatusResult::ReadFailure;
        return status;
    }
    status.result=MotorStatusResult::Valid;
    status.speed=GetMotorSpeed();
    status.driver_reported_position=GetReportedPos();
    status.status_flags=(1u<<13); // pwr
    if(std::fabs(status.speed)<0.001f) status.status_flags|=(1u<<8); // stp
    status.observed_after_active_query=true;
    status.active_query_requested_bytes=6;
    status.active_query_accepted_bytes=6;
    status.active_query_write_attempts=1;
    status.active_query_write_outcome=static_cast<int>(SerialWriteOutcome::Complete);
    return status;
}
MotorStopReport SimHAL::StopMotion(MotorStopMode mode,int32_t verify_timeout_ms) {
    const auto write_line=[this](const uint8_t* data,int len) {
        return WriteAsciiLineOnce(ascii_stream_,data,len,
            [this](const uint8_t*,int requested) {
                if(!initialized_) return -1;
                Update();
                target_=static_cast<int32_t>(std::round(position_));
                position_=target_;
                return requested;
            });
    };
    return ExecuteBoundedMotorStop(
        ascii_stream_,mode,verify_timeout_ms,write_line,
        [this](int32_t timeout_ms) { return QueryMotorStatus(timeout_ms); });
}
bool SimHAL::WaitForMoveComplete(int32_t timeout_ms) {
    const auto start=Clock::now();
    for(int elapsed=0;elapsed<timeout_ms;elapsed+=5) {
        if(manual_) Advance(0.005);
        if(std::fabs(GetMotorSpeed())<0.001f) return true;
        if(!manual_) {
            if(std::chrono::duration<double,std::milli>(Clock::now()-start).count()>=timeout_ms) break;
#ifdef _WIN32
            Sleep(5);
#else
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
#endif
        }
    }
    return false;
}
int SimHAL::Home(int32_t) {
    if(!initialized_) return -1;
    position_=target_=0; last_=Clock::now(); return 0;
}
HomingExecutionReport SimHAL::HomeWithSafety(
        int32_t,const volatile std::sig_atomic_t* running) {
    HomingExecutionReport report;
    if(running && !*running) {
        report.result=HomingExecutionResult::Interrupted;
        return report;
    }
    if(Home(0)!=0) {
        report.result=HomingExecutionResult::StatusFailure;
        report.hardware_intervention_required=true;
        return report;
    }
    report.result=HomingExecutionResult::Complete;
    report.driver_home_confirmed=true;
    report.stop_state_unknown=false;
    report.completion_status=QueryMotorStatus(0);
    report.completion_status.status_flags|=(1u<<14); // zero complete
    return report;
}
