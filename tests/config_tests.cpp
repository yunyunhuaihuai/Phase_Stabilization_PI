#include "config/ConfigParser.h"
#include <cstdio>
#include <string>
#include <limits>

static int checks=0, failures=0;
#define CHECK(c) do { ++checks; if(!(c)) { ++failures; std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#c); } } while(false)
int main(int argc,char** argv) {
    if(argc!=2) return 2;
    SystemConfig cfg;
    CHECK(ConfigParser_Load(argv[1],cfg)==0 && cfg.Validate());
    CHECK(cfg.kp==40 && cfg.ki==3200 && cfg.control_freq_hz==20);
    CHECK(cfg.EffectiveLogicalLimit()==5500);
    const char* bad[]={"CONTROL_FREQ_HZ=nan\n","CONTROL_FREQ_HZ=inf\n","CONTROL_FREQ_HZ=10\n",
        "CONTROL_FREQ_HZ=20junk\n","MAX_STEP_PER_CYCLE=5500.5\n","MAX_STEP_PER_CYCLE=8001\n",
        "MAX_PHYSICAL_DELTA=0\n","MAX_PHYSICAL_DELTA=8001\n","BACKLASH_STEPS=40\nMAX_PHYSICAL_DELTA=40\n",
        "MICROSTEP_RATIO=2\n","KP=-1\n","KI=1e9\n","POS_MIN_LIMIT=-1\n","POS_CENTER=2112001\n",
        "POS_MIN_LIMIT=20\nPOS_MAX_LIMIT=10\n","BASE_B0=.0001\n","TARGET_VPHS_MV=901\n",
        "DEADBAND_MV=-1\n","SWEEP_TOLERANCE_MV=0\n","TOTAL_DURATION_MIN=0\n",
        "FLUSH_INTERVAL=0\n","BAUD_RATE=9600\n","UNSUPPORTED_KEY=2\n","missing_equals\n"};
    for(const char* input:bad) {
        FILE* f=std::fopen("invalid-config.txt","w"); CHECK(f!=nullptr);
        if(!f) return 2;
        std::fputs(input,f); std::fclose(f);
        SystemConfig candidate=cfg; candidate.kp=123;
        CHECK(ConfigParser_Load("invalid-config.txt",candidate)!=0 && candidate.kp==123);
    }
    const std::string longline="KP="+std::string(200,'1')+"\n";
    FILE* f=std::fopen("invalid-config.txt","w");CHECK(f!=nullptr);
    if(!f) return 2;
    std::fputs(longline.c_str(),f);std::fclose(f);
    CHECK(ConfigParser_Load("invalid-config.txt",cfg)!=0);
    CHECK(ConfigParser_Load("not-present-config.txt",cfg)!=0);
    for(int limit:{5000,5500,7250,8000}) {
        SystemConfig valid; valid.max_step_per_cycle=valid.max_physical_delta=limit;
        CHECK(valid.Validate() && valid.EffectiveLogicalLimit()==limit);
    }
    SystemConfig nonfinite; nonfinite.ki=std::numeric_limits<float>::infinity();CHECK(!nonfinite.Validate());
    std::printf("Config checks=%d failures=%d\n",checks,failures);
    return failures?1:0;
}
