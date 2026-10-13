#ifndef SIM_HAL_H
#define SIM_HAL_H
#include "IHardwareHAL.h"
#include <chrono>
#include <random>
// Offline startup and closed-loop simulator; parameters are illustrative assumptions.
class SimHAL : public IHardwareHAL {
public:
    int Init() override;
    float ReadVPHS() override;
    float ReadTemp() override { return initialized_?25.0f:TEMP_INVALID; }
    bool SendPosCmd(int32_t pos) override;
    SerialWriteReport LastPositionWriteReport() const override {
        return last_position_write_;
    }
    float GetMotorSpeed() override;
    int32_t GetReportedPos() override;
    MotorStatusSnapshot PollMotorStatus() override;
    MotorStatusSnapshot QueryMotorStatus(int32_t timeout_ms) override;
    MotorStopReport StopMotion(MotorStopMode mode,
                               int32_t verify_timeout_ms) override;
    int ConfigODL(int) override { return initialized_?0:-1; }
    bool WaitForMoveComplete(int32_t timeout_ms) override;
    int Home(int32_t) override;
    HomingExecutionReport HomeWithSafety(
        int32_t timeout_ms,
        const volatile std::sig_atomic_t* running) override;
    void Release() override { initialized_=false; }
    int WriteSerial(const uint8_t*,int len) override { return initialized_?len:-1; }
    int ReadSerial(uint8_t*,int) override { return 0; }
    int32_t GetSimulatedPos() const { return static_cast<int32_t>(position_); }
    void UseManualTime(bool enable) { manual_=enable; last_=Clock::now(); }
    void Advance(double dt);
    void SetNoise(double sd) { noise_sd_=sd; }
    void SetDisturbance(double amplitude_mv) { disturbance_mv_=amplitude_mv; disturbance_seconds_=0; }
private:
    using Clock=std::chrono::steady_clock;
    void Update();
    bool initialized_=false, manual_=false;
    double position_=1056000, noise_sd_=0.05;
    double disturbance_mv_=0, disturbance_seconds_=0;
    int32_t target_=1056000;
    Clock::time_point last_;
    std::mt19937 rng_{41024};
    std::normal_distribution<double> normal_{0,1};
    AsciiStreamState ascii_stream_;
    SerialWriteReport last_position_write_;
};
#endif
