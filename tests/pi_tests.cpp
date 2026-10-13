#include "control/DiscretePI.h"
#include "utils/CommandGate.h"
#include <cstdio>
#include <limits>

static int checks=0, failures=0;
#define CHECK(c) do { ++checks; if(!(c)) { ++failures; std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#c); } } while(false)

int main() {
    SystemConfig cfg;
    int32_t p=0; float q=0;
    DiscretePI uninitialized;
    uninitialized.BumplessTransfer(12345,1);
    uninitialized.Compute(2,.05f,p,q);
    CHECK(p==12345 && q==12345);
    for(int polarity : {-1,1}) {
        SystemConfig effective=cfg;
        effective.max_step_per_cycle=cfg.EffectiveLogicalLimit();
        DiscretePI pi; pi.Init(&effective);
        pi.BumplessTransfer(cfg.pos_center,static_cast<float>(polarity));
        CHECK(pi.GetIntegrator()==cfg.pos_center && pi.GetPolarity()==polarity);
        const float errors[]={6.375f,1.375f,.2f,-1,-1,0,100,-100,100,.333f,-.333f};
        int32_t previous=cfg.pos_center;
        for(unsigned i=0;i<sizeof(errors)/sizeof(errors[0]);++i) {
            const float old_i=pi.GetIntegrator();
            pi.Compute(-polarity*errors[i],.05f,p,q);
            CHECK(p==static_cast<int32_t>(q));
            CHECK(std::abs(p-previous)<=cfg.EffectiveLogicalLimit());
            CHECK(p>=cfg.pos_min_limit && p<=cfg.pos_max_limit);
            if(i==0) CHECK(p-previous==1275);
            if(i==1) CHECK(p-previous==20);
            if(i==2) CHECK(p-previous==-15);
            if(i==5) CHECK(p==previous && pi.GetIntegrator()==old_i);
            if(i==6 || i==8) CHECK(p-previous==5500);
            if(i==7) CHECK(p-previous==-5500);
            if(i==6 || i==7 || i==8) CHECK(std::fabs(pi.GetIntegrator()-(q-cfg.kp*errors[i]))<.13f);
            previous=p;
        }
    }
    // Deadband is strict (<); the threshold itself computes an output.
    DiscretePI deadband; deadband.Init(&cfg); deadband.BumplessTransfer(10000,1);
    deadband.Compute(-.149f,.05f,p,q); CHECK(p==10000 && deadband.GetIntegrator()==10000);
    deadband.Compute(-cfg.deadband_mv,.05f,p,q); CHECK(q>10000);
    for(int direction:{-1,1}) {
        DiscretePI pi; pi.Init(&cfg);
        pi.BumplessTransfer(direction>0?cfg.pos_max_limit-1:cfg.pos_min_limit+1,1);
        pi.Compute(-direction*100.0f,.05f,p,q);
        CHECK(p==(direction>0?cfg.pos_max_limit:cfg.pos_min_limit));
        CHECK(pi.GetIntegrator()==q-cfg.kp*direction*100.0f);
    }
    // Effective logical limit reserves the negative-direction position offset.
    SystemConfig offset=cfg;
    offset.backlash_steps=50; offset.max_step_per_cycle=6000; offset.max_physical_delta=4000;
    CHECK(offset.Validate() && offset.EffectiveLogicalLimit()==3950);
    offset.max_step_per_cycle=offset.EffectiveLogicalLimit();
    DiscretePI reverse; reverse.Init(&offset); reverse.BumplessTransfer(10000,1);
    reverse.Compute(100,.05f,p,q);
    CHECK(q==6050 && p==6000);
    RunawayGuard guard; guard.Init(offset.EffectiveLogicalLimit());
    CHECK(EvaluateCommandProposal(guard,true,false,100,0,p,10000,0,2112000,4000).allowed);
    // Synthetic float binade crossing: the first proposal retains a fraction;
    // adding 5000 rounds one pulse beyond the host limit. Correct and rebase.
    SystemConfig quant=cfg;
    quant.kp=0; quant.ki=1; quant.deadband_mv=0;
    quant.max_step_per_cycle=quant.max_physical_delta=5000;
    DiscretePI pi; pi.Init(&quant); pi.BumplessTransfer(1045000,1);
    pi.Compute(-.9375f,1,p,q); CHECK(p==1045000 && q==1045000.9375f);
    pi.Compute(-50000,1,p,q); CHECK(p==1050000 && q==1050000 && pi.GetIntegrator()==1050000);
    pi.Compute(-50000,1,p,q); CHECK(p==1055000);
    for(int limit:{1,200,5500,7250,8000}) {
        SystemConfig bounded=cfg; bounded.max_step_per_cycle=bounded.max_physical_delta=limit;
        CHECK(bounded.Validate());
        DiscretePI c; c.Init(&bounded); c.BumplessTransfer(cfg.pos_center,-1);
        int32_t prev=cfg.pos_center;
        for(int n=0;n<1000;++n) {
            const float v=30*std::sin(n*.13f);
            c.Compute(v,.05f,p,q);
            CHECK(std::abs(p-prev)<=limit && p>=0 && p<=cfg.pos_max_limit);
            CHECK(std::isfinite(q) && std::isfinite(c.GetIntegrator()));
            prev=p;
        }
    }
    guard.Reset();
    const auto bad=EvaluateCommandProposal(guard,false,false,std::numeric_limits<float>::quiet_NaN(),0,p,p,0,2112000,4000);
    CHECK(!bad.allowed); // Main never passes a nonfinite ADC value into Compute.
    std::printf("PI checks=%d failures=%d\n",checks,failures);
    return failures?1:0;
}
