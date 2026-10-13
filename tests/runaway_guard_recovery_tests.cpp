#include "utils/RunawayGuard.h"
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cmath>
#include <limits>
#include <fstream>
#include <sstream>
#include <string>
#include <stdexcept>

static int cases = 0, checks = 0, failures = 0;
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; \
    std::fprintf(stderr, "FAIL %s:%d: %s\n", __func__, __LINE__, #c); } } while (false)
#define CASE(f) do { ++cases; f(); } while (false)

// Synthetic protection sequences; no recorded experiment data.
static RunawayGuard Loaded(int64_t steps=60000) {
    RunawayGuard g; g.Init(6500);
    while (steps > 0) {
        const int32_t n = static_cast<int32_t>(steps > 6000 ? 6000 : steps);
        CHECK(!g.Feed(20, 0, -n)); steps -= n;
    }
    return g;
}

static void constants_and_init() {
    RunawayGuard g; g.Init(5750);
    CHECK(std::string(RunawayGuard::Version())=="pi-guard/1.0");
    CHECK(g.MaxStep()==5750);
    CHECK(g.DisplacementLimit()==200000);
    CHECK(g.RecoveryErrorMv()==0.5f);
    CHECK(g.RecoveryCommandSteps()==200);
    CHECK(g.RecoverySamples()==21);
    CHECK(g.TripMask()==0 && g.DirectionalSteps()==0);
}
static void rising_rule_both_directions_all_limits() {
    for (int limit : {5500,5750,6000,6250,6500,6750,7000,7250,7500,7750,8000}) for (int dir : {-1,1}) {
        RunawayGuard g; g.Init(limit);
        for (int i=0;i<4;++i) CHECK(!g.Feed(float(10+i),0,dir*limit));
        CHECK(g.Feed(14,0,dir*limit));
        CHECK(g.TripMask()==RunawayGuard::RISING_SATURATION);
    }
}
static void constant_error_runaway_all_limits() {
    for (int limit : {5500,5750,6000,6250,6500,6750,7000,7250,7500,7750,8000}) for (int dir : {-1,1}) {
        RunawayGuard g; g.Init(limit);
        const int first = 200000/limit+1;
        for(int i=1;i<first;++i) CHECK(!g.Feed(20,0,dir*limit));
        CHECK(g.Feed(20,0,dir*limit));
        CHECK(g.TripMask()==RunawayGuard::DIRECTIONAL_TRAVEL);
    }
}
static void large_error_without_motion() {
    RunawayGuard g; g.Init(5500);
    for(int i=0;i<49;++i) CHECK(!g.Feed(-200,0,0));
    CHECK(g.Feed(-200,0,0));
    CHECK(g.TripMask()==RunawayGuard::LARGE_ERROR);
}
static void recovery_needs_full_sample_window() {
    auto g=Loaded();
    for(int i=0;i<20;++i) CHECK(!g.Feed(0.1f,0,0));
    CHECK(g.DirectionalSteps()==60000 && g.RecoveryBeats()==20);
    CHECK(!g.Feed(0.1f,0,0));
    CHECK(g.DirectionalSteps()==0 && g.RecoveryResets()==1);
}
static void idle_without_recovery_keeps_count() {
    auto g=Loaded();
    for(int i=0;i<1000;++i) CHECK(!g.Feed(10,0,0));
    CHECK(g.DirectionalSteps()==60000 && g.RecoveryResets()==0);
}
static void error_excursion_restarts_dwell() {
    auto g=Loaded();
    for(int i=0;i<20;++i) CHECK(!g.Feed(0.1f,0,0));
    CHECK(!g.Feed(0.5001f,0,0));
    CHECK(g.RecoveryBeats()==0);
    for(int i=0;i<20;++i) CHECK(!g.Feed(0.1f,0,0));
    CHECK(g.DirectionalSteps()==60000);
    CHECK(!g.Feed(0.1f,0,0)); CHECK(g.DirectionalSteps()==0);
}
static void command_excursion_restarts_dwell() {
    auto g=Loaded();
    for(int i=0;i<20;++i) CHECK(!g.Feed(0.1f,0,0));
    CHECK(!g.Feed(0.1f,0,-201)); CHECK(g.RecoveryBeats()==0);
    for(int i=0;i<20;++i) CHECK(!g.Feed(0.1f,0,0));
    CHECK(g.DirectionalSteps()==60201);
    CHECK(!g.Feed(0.1f,0,0)); CHECK(g.DirectionalSteps()==0);
}
static void large_commands_never_rearm_at_small_error() {
    RunawayGuard g; g.Init(6500);
    for(int i=0;i<30;++i) CHECK(!g.Feed(0.1f,0,-6500));
    CHECK(g.Feed(0.1f,0,-6500));
    CHECK(g.RecoveryResets()==0 && g.TripMask()==RunawayGuard::DIRECTIONAL_TRAVEL);
}
static void ramp_sized_commands_do_not_rearm() {
    RunawayGuard g; g.Init(5500);
    for(int i=0;i<200000/523;++i) CHECK(!g.Feed(0.1f,0,523));
    CHECK(g.Feed(0.1f,0,523));
    CHECK(g.TripMask()==RunawayGuard::DIRECTIONAL_TRAVEL && g.RecoveryResets()==0);
}
static void separate_recovered_disturbances_no_longer_accumulate() {
    RunawayGuard g; g.Init(5750);
    for(int i=0;i<11;++i) CHECK(!g.Feed(30,0,-5750));
    for(int i=0;i<100;++i) CHECK(!g.Feed(0.1f,0,0));
    CHECK(g.DirectionalSteps()==0);
    for(int i=0;i<24;++i) CHECK(!g.Feed(30,0,-5750));
    CHECK(g.DirectionalSteps()==24*5750 && g.RecoveryResets()>=1);
}
static void nonrecovered_disturbances_still_accumulate() {
    auto g=Loaded(65000);
    for(int i=0;i<100;++i) CHECK(!g.Feed(2,0,0));
    for(int i=0;i<20;++i) CHECK(!g.Feed(100,0,-6500));
    CHECK(g.DirectionalSteps()==195000);
    CHECK(g.Feed(90,0,-6500));
    CHECK(g.TripMask()==RunawayGuard::DIRECTIONAL_TRAVEL);
}
static void trip_wins_over_same_sample_recovery() {
    auto g=Loaded(199900);
    for(int i=0;i<20;++i) CHECK(!g.Feed(0.1f,0,0));
    CHECK(g.Feed(0.1f,0,-200));
    CHECK(g.TripMask()==RunawayGuard::DIRECTIONAL_TRAVEL);
    CHECK(g.DirectionalSteps()==200100 && g.RecoveryResets()==0);
}
static void exact_displacement_threshold_preserved() {
    RunawayGuard g; g.Init(6000);
    for(int i=0;i<40;++i) CHECK(!g.Feed(20,0,5000));
    CHECK(g.DirectionalSteps()==200000);
    CHECK(g.Feed(20,0,1)); CHECK(g.DirectionalSteps()==200001);
}

static void reversal_behavior_preserved() {
    auto g=Loaded();
    CHECK(!g.Feed(20,0,100)); CHECK(g.DirectionalSteps()==100);
    CHECK(!g.Feed(20,0,0)); CHECK(g.DirectionalSteps()==100);
    CHECK(!g.Feed(20,0,100)); CHECK(g.DirectionalSteps()==200);
}
static void recovery_boundaries_and_nonzero_target() {
    auto g=Loaded();
    for(int i=0;i<21;++i) CHECK(!g.Feed(3.5f,3.0f,-200));
    CHECK(g.DirectionalSteps()==0 && g.RecoveryResets()==1);
}
static void slow_recovered_tracking_is_not_permanent_travel() {
    RunawayGuard g; g.Init(5500);
    for(int i=0;i<2000;++i) CHECK(!g.Feed(0.1f,0,100));
    CHECK(g.RecoveryResets()>50 && g.DirectionalSteps()<2100);
}
static void latch_and_explicit_reset() {
    RunawayGuard g; g.Init(5500);
    for(int i=0;i<5;++i) g.Feed(float(10+i),0,5500);
    const auto steps=g.DirectionalSteps();
    for(int i=0;i<50;++i) CHECK(g.Feed(0,0,0));
    CHECK(g.DirectionalSteps()==steps);
    g.Reset(); CHECK(g.TripMask()==0 && g.DirectionalSteps()==0);
    CHECK(!g.Feed(0,0,0));
}
static void integer_min_has_no_overflow() {
    RunawayGuard g; g.Init(6500);
    CHECK(g.Feed(20,0,std::numeric_limits<int32_t>::min()));
    CHECK(g.DirectionalSteps()==2147483648LL);
    CHECK(g.TripMask()==RunawayGuard::DIRECTIONAL_TRAVEL);
}
static void invalid_inputs_are_not_recovery() {
    RunawayGuard g; CHECK(g.Feed(0,0,0));
    CHECK(g.TripMask()==RunawayGuard::INVALID_INPUT);
    g.Init(5500); CHECK(g.Feed(std::numeric_limits<float>::quiet_NaN(),0,0));
    CHECK(g.TripMask()==RunawayGuard::INVALID_INPUT);
    g.Init(5500); CHECK(g.Feed(0,std::numeric_limits<float>::infinity(),0));
    CHECK(g.TripMask()==RunawayGuard::INVALID_INPUT);
}




int main() {
    CASE(constants_and_init);
    CASE(rising_rule_both_directions_all_limits);
    CASE(constant_error_runaway_all_limits);
    CASE(large_error_without_motion);
    CASE(recovery_needs_full_sample_window);
    CASE(idle_without_recovery_keeps_count);
    CASE(error_excursion_restarts_dwell);
    CASE(command_excursion_restarts_dwell);
    CASE(large_commands_never_rearm_at_small_error);
    CASE(ramp_sized_commands_do_not_rearm);
    CASE(separate_recovered_disturbances_no_longer_accumulate);
    CASE(nonrecovered_disturbances_still_accumulate);
    CASE(trip_wins_over_same_sample_recovery);
    CASE(exact_displacement_threshold_preserved);
    CASE(reversal_behavior_preserved);
    CASE(recovery_boundaries_and_nonzero_target);
    CASE(slow_recovered_tracking_is_not_permanent_travel);
    CASE(latch_and_explicit_reset);
    CASE(integer_min_has_no_overflow);
    CASE(invalid_inputs_are_not_recovery);
    std::printf("RunawayGuard recovery tests: cases=%d checks=%d failures=%d; hardware_actions=0\n",cases,checks,failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
