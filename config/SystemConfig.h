#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H
#include <cstdint>
#include <cmath>
#include <algorithm>
constexpr int32_t BASE_BACKLASH=0, BASE_MAX_STEP=5500, BASE_POS_MAX=2112000, BASE_POS_CENTER=1056000;
// Software exploration ceiling only; it is not a certified hardware-safe limit.
constexpr int32_t MAX_CONFIGURED_STEP_LIMIT=8000;
static_assert(BASE_MAX_STEP<=MAX_CONFIGURED_STEP_LIMIT,"default step limit exceeds software ceiling");
constexpr float BASE_B0=0.0025f;
constexpr const char* SOFTWARE_VERSION="phase-stabilization-pi/1.0.0";
struct SystemConfig {
    int32_t microstep_ratio=1;
    int32_t backlash_steps=BASE_BACKLASH, max_step_per_cycle=BASE_MAX_STEP;
    float b0=BASE_B0;
    int32_t pos_max_limit=BASE_POS_MAX, pos_center=BASE_POS_CENTER;
    float target_vphs_mv=0, deadband_mv=0.15f, sweep_tolerance_mv=2;
    int32_t pos_min_limit=0;
    float kp=40, ki=3200;
    float control_freq_hz=20;
    int32_t total_duration_min=4, flush_interval=200, baud_rate=115200;
    int32_t max_physical_delta=BASE_MAX_STEP;
    int32_t EffectiveLogicalLimit() const { return std::min(max_step_per_cycle,max_physical_delta-backlash_steps); }
    bool Validate(const char** why=nullptr) const {
#define CHECK(c,msg) do { if (!(c)) { if(why) *why=msg; return false; } } while(false)
        CHECK(microstep_ratio==1,"requires MICROSTEP_RATIO=1");
        CHECK(std::isfinite(control_freq_hz) && std::fabs(control_freq_hz-20)<1e-6,"requires 20Hz");
        CHECK(max_step_per_cycle>0 && max_step_per_cycle<=MAX_CONFIGURED_STEP_LIMIT,"logical limit");
        CHECK(backlash_steps>=0 && backlash_steps<max_physical_delta && max_physical_delta<=MAX_CONFIGURED_STEP_LIMIT,"physical limit/backlash");
        CHECK(pos_min_limit>=0 && pos_min_limit<=BASE_POS_MAX && pos_max_limit>=0 && pos_max_limit<=BASE_POS_MAX && pos_max_limit>pos_min_limit+backlash_steps && pos_center>=pos_min_limit+backlash_steps && pos_center<=pos_max_limit,"travel bounds");
        CHECK(std::isfinite(b0) && b0>=0.0005f && b0<=0.0040f,"gain");
        CHECK(std::isfinite(target_vphs_mv) && std::fabs(target_vphs_mv)<=900,"target");
        CHECK(std::isfinite(deadband_mv) && deadband_mv>=0 && deadband_mv<=2,"deadband");
        CHECK(std::isfinite(sweep_tolerance_mv) && sweep_tolerance_mv>0 && sweep_tolerance_mv<=2,"sweep tolerance");
        CHECK(std::isfinite(kp) && kp>=0 && kp<=1e6 && std::isfinite(ki) && ki>=0 && ki<=1e7,"PI gains");
        CHECK(total_duration_min>=1 && total_duration_min<=1440 && flush_interval>=1 && flush_interval<=100000,"duration/flush");
        CHECK(baud_rate==115200,"validated baud is 115200");
        if(why) *why="OK";
        return true;
#undef CHECK
    }
};
#endif
