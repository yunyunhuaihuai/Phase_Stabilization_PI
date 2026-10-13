#include "startup/WorkingPoint.h"
#include "hal/SimHAL.h"
#include "utils/DataLogger.h"
#include "utils/LoggerShutdown.h"
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <thread>
#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#include <unistd.h>
#endif

static int checks=0,failures=0;
#define CHECK(c) do { ++checks; if(!(c)) { ++failures; std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#c); } } while(false)
static void NoSleep(int) {}
struct InstantHAL : IHardwareHAL {
    int32_t position=1056000; int direction=1,sends=0,fail_at=0;
    bool invalid_adc=false, timeout=false, fail_home=false; float gain=.0025f;
    int Init() override { return 0; }
    float ReadVPHS() override { return invalid_adc?NAN:(position-1056000)*gain; }
    float ReadTemp() override { return 25; }
    bool SendPosCmd(int32_t p) override {
        if(++sends==fail_at) return false;
        if(p!=position) direction=p>position?1:-1;
        position=p; return true;
    }
    float GetMotorSpeed() override { return 0; }
    int32_t GetReportedPos() override { return position; }
    int ConfigODL(int) override { return 0; }
    bool WaitForMoveComplete(int32_t) override { return !timeout; }
    HomingExecutionReport HomeWithSafety(int32_t,const volatile std::sig_atomic_t*) override {
        HomingExecutionReport r;
        r.result=fail_home?HomingExecutionResult::Timeout:HomingExecutionResult::Complete;
        r.driver_home_confirmed=!fail_home; return r;
    }
    void Release() override {}
    int WriteSerial(const uint8_t*,int len) override { return len; }
    int ReadSerial(uint8_t*,int) override { return 0; }
};
static std::vector<std::string> Split(const std::string& line) {
    std::vector<std::string> v; size_t b=0;
    for(size_t i=0;i<=line.size();++i) if(i==line.size() || line[i]==',') { v.push_back(line.substr(b,i-b)); b=i+1; }
    return v;
}
static void MakeDirectory(const std::string& path) {
#ifdef _WIN32
    CHECK(_mkdir(path.c_str())==0);
#else
    CHECK(mkdir(path.c_str(),0700)==0);
#endif
}
static void RemoveDirectory(const std::string& path) {
#ifdef _WIN32
    CHECK(_rmdir(path.c_str())==0);
#else
    CHECK(rmdir(path.c_str())==0);
#endif
}
// Fault injection occurs before any cycle is enqueued; the release/acquire
// queue synchronizes the deliberate missing diagnostic stream with the writer.
struct DataLoggerTestAccess {
    static void LoseDiagnostics(DataLogger& logger) {
        std::fclose(logger.diagnostics_); logger.diagnostics_=nullptr;
    }
    static void FullQueueWithoutWorker(DataLogger& logger) {
        logger.writer_started_=true;
        logger.queue_head_.store(logger.QUEUE_CAPACITY);
        logger.queue_tail_.store(0);
    }
};
int main(int argc,char** argv) {
    if(argc!=2) return 2;
    SystemConfig cfg; CHECK(cfg.Validate());
    volatile std::sig_atomic_t running=1;
    WorkingPoint startup(running,NoSleep);
    InstantHAL hal;
    CHECK(startup.Establish(hal,cfg));
    CHECK(startup.diagnostics.homing && startup.diagnostics.sweep && startup.diagnostics.calibrated);
    CHECK(hal.direction==1 && hal.position==cfg.pos_center);
    CHECK(startup.diagnostics.polarity==1 && std::fabs(startup.diagnostics.gain-.0025)<1e-7);
    hal.gain=-.0025f; running=1;
    CHECK(startup.Establish(hal,cfg) && startup.diagnostics.polarity==-1);
    hal.gain=.0025f; hal.fail_at=hal.sends+1; running=1;
    float polarity=0,gain=0;
    CHECK(!startup.CalibrateGain(hal,cfg,cfg.pos_center,polarity,gain) && running==0);
    hal.fail_at=0;hal.invalid_adc=true;running=1;
    int32_t escape=0;int previous_sends=hal.sends;
    CHECK(!startup.ValidateAnchor(hal,cfg,cfg.pos_center,escape) && !running && hal.sends==previous_sends);
    hal.invalid_adc=false;hal.timeout=true;running=1;
    CHECK(!startup.Establish(hal,cfg));
    hal.timeout=false;hal.fail_home=true;running=1;
    CHECK(!startup.Establish(hal,cfg) && startup.diagnostics.failure_in_homing);
    hal.fail_home=false;hal.gain=.0001f;running=1;
    CHECK(!startup.CalibrateGain(hal,cfg,cfg.pos_center,polarity,gain));
    SimHAL sim;sim.UseManualTime(true);
    CHECK(!sim.SendPosCmd(1056001) && sim.Init()==0);
    sim.SetNoise(0);CHECK(sim.SendPosCmd(1057000));
    float before=sim.ReadVPHS();sim.Advance(.05);CHECK(sim.ReadVPHS()>before+1);
    CHECK(!sim.SendPosCmd(-1));CHECK(!sim.WaitForMoveComplete(0));
    CHECK(sim.StopMotion(MotorStopMode::Immediate,250).driver_stop_confirmed);
    sim.Release();

    static DataLogger logger;logger.SetFlushInterval(20);
    CHECK(logger.Open("PI",1)==0 && logger.OpenDiagnostics(cfg,.0025,1,1,argv[1]));
    // Entirely synthetic fault evidence: rejected target is separate from the
    // last software-accepted target and a deliberately different driver value.
    CycleDiagnostics row;row.cycle=1;row.status=PIStatus::GuardFault;
    row.proposed_logical=row.proposed_physical=20000;
    row.accepted_logical=row.accepted_physical=25000;
    row.send_result=-2;row.send_accepted=false;
    row.driver_status_available=true;row.driver_status.result=MotorStatusResult::Valid;
    row.driver_status.speed=0;row.driver_status.driver_reported_position=24900;
    CHECK(logger.LogCycle(.05f,1,25,20000,20000,"FAULT",row));
    MotorStopReport stop;stop.mode=MotorStopMode::Immediate;
    stop.stop_state_unknown=true;stop.hardware_intervention_required=true;
    CHECK(logger.LogMotorStopReport("synthetic_stop",stop));
    const std::string name=logger.Filename();logger.Close();CHECK(logger.Healthy());
    std::ifstream raw(name),diag(name+".diagnostics.csv"),safety(name+".safety.csv"),meta(name+".metadata.txt");
    std::string header,line;CHECK(bool(std::getline(raw,header)) && bool(std::getline(raw,line)));
    auto fields=Split(line);CHECK(Split(header).size()==22 && fields.size()==22);
    if(fields.size()==22) CHECK(fields[7]=="20000" && fields[9]=="25000" && fields[11]=="0" && fields[15]=="24900");
    CHECK(bool(std::getline(diag,header)) && bool(std::getline(diag,line)));
    CHECK(Split(header).size()==55 && Split(line).size()==55);
    auto names=Split(header);fields=Split(line);
    auto field=[&](const char* key) { auto i=std::find(names.begin(),names.end(),key);return i==names.end()?std::string():fields[static_cast<size_t>(i-names.begin())]; };
    CHECK(field("Controller_status")=="guard_fault" && field("Send_result")=="-2");
    CHECK(field("Mechanical_stop_independently_verified")=="0");
    CHECK(bool(std::getline(safety,header)) && bool(std::getline(safety,line)));
    CHECK(Split(header).size()==23 && Split(line).size()==23);
    const std::string metadata((std::istreambuf_iterator<char>(meta)),{});
    CHECK(metadata.find("CONTROLLER=PI")!=std::string::npos && metadata.find("DATA_ORIGIN=simulation")!=std::string::npos);

    static DataLogger writer_failure;
    CHECK(writer_failure.Open("PI",1)==0 && writer_failure.OpenDiagnostics(cfg,.0025,1,1,argv[1]));
    DataLoggerTestAccess::LoseDiagnostics(writer_failure);
    CHECK(writer_failure.LogCycle(.05,0,25,0,0,"TEST",row));writer_failure.Flush();
    CHECK(!writer_failure.Healthy());writer_failure.Close();
    bool failed=false;if(!writer_failure.Healthy()) ReportLoggerShutdownFailure(failed,stderr);CHECK(failed);
    static DataLogger full;
    CHECK(full.Open("PI",1)==0);DataLoggerTestAccess::FullQueueWithoutWorker(full);
    CHECK(!full.LogCycle(.05,0,25,0,0,"TEST",row) && !full.Healthy());full.Close();
    static DataLogger unavailable;
    CHECK(unavailable.Open("missing-directory/PI",1)!=0);
    CHECK(unavailable.Open("PI",1)==0);
    const std::string blocked=std::string(unavailable.Filename())+".diagnostics.csv";
    MakeDirectory(blocked);
    CHECK(!unavailable.OpenDiagnostics(cfg,.0025,1,1,argv[1]));unavailable.Close();RemoveDirectory(blocked);
    std::printf("Integration checks=%d failures=%d\n",checks,failures);
    return failures?1:0;
}
