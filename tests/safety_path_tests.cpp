#include "hal/MotorSafety.h"
#include "utils/CommandGate.h"
#include "control/PhysicalStepLimit.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

static int checks=0;
static int failures=0;
#define CHECK(c) do { ++checks; if(!(c)) { ++failures; \
    std::fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#c); } } while(false)

static MotorStatusSnapshot StoppedStatus(bool after_active_query=false) {
    MotorStatusSnapshot status;
    status.result=MotorStatusResult::Valid;
    status.speed=0.0f;
    status.driver_reported_position=608821;
    status.status_flags=(1u<<8)|(1u<<13); // stp + pwr
    status.observed_after_active_query=after_active_query;
    if(after_active_query) {
        status.active_query_requested_bytes=6;
        status.active_query_accepted_bytes=6;
        status.active_query_write_attempts=1;
        status.active_query_write_outcome=
            static_cast<int>(SerialWriteOutcome::Complete);
    }
    return status;
}

static MotorStatusSnapshot TimeoutStatus() {
    MotorStatusSnapshot status;
    status.result=MotorStatusResult::Timeout;
    return status;
}

struct ScriptedLineWriter {
    explicit ScriptedLineWriter(const std::vector<int>& accepted_counts)
        : accepted(accepted_counts) {}

    SerialWriteReport Write(AsciiStreamState& stream,
                            const uint8_t* data,int len) {
        const int planned=next<accepted.size()?accepted[next++]:len;
        int accepted_this_line=0;
        return WriteBoundedAsciiLine(stream,data,len,
            [this,planned,&accepted_this_line](const uint8_t* bytes,
                                               int requested) {
                ++calls;
                const int available=planned-accepted_this_line;
                if(available<=0) return SerialWriteAttempt::Error();
                const int count=available<requested?available:requested;
                wire.append(reinterpret_cast<const char*>(bytes),
                            static_cast<size_t>(count));
                accepted_this_line+=count;
                return SerialWriteAttempt::Progress(count);
            },[](){return false;},[](){});
    }

    std::vector<int> accepted;
    size_t next=0;
    int calls=0;
    std::string wire;
};

static MotorStatusSnapshot QueryThroughWire(ScriptedLineWriter& tx,
                                            AsciiStreamState& stream,
                                            MotorStatusSnapshot response) {
    const char command[]="1 sts\n";
    const SerialWriteReport write=tx.Write(
        stream,reinterpret_cast<const uint8_t*>(command),
        static_cast<int>(sizeof(command)-1));
    response.active_query_requested_bytes=write.requested_bytes;
    response.active_query_accepted_bytes=write.accepted_bytes;
    response.active_query_write_attempts=write.write_attempts;
    response.active_query_write_outcome=static_cast<int>(write.outcome);
    response.observed_after_active_query=
        write.IsComplete() && response.result==MotorStatusResult::Valid;
    if(!write.IsComplete()) {
        response.result=stream.IsTainted()
            ? MotorStatusResult::StreamTainted
            : MotorStatusResult::WriteFailure;
    }
    return response;
}

static std::vector<long> ParsedPositionTargets(const std::string& wire) {
    std::vector<long> targets;
    size_t begin=0;
    for(;;) {
        const size_t end=wire.find('\n',begin);
        if(end==std::string::npos) break; // unterminated tail is not a line
        const std::string line=wire.substr(begin,end-begin);
        static const std::string prefix="1 pos ";
        if(line.compare(0,prefix.size(),prefix)==0 && line.size()>prefix.size()) {
            char* parsed_end=nullptr;
            const long value=std::strtol(line.c_str()+prefix.size(),&parsed_end,10);
            if(parsed_end && *parsed_end=='\0') targets.push_back(value);
        }
        begin=end+1;
    }
    return targets;
}

static void bounded_ascii_writer_tracks_every_byte_and_deadline() {
    const std::string command="1 pos 652321\n";
    {
        AsciiStreamState stream;
        std::string wire;
        int calls=0;
        const SerialWriteReport report=WriteBoundedAsciiLine(
            stream,reinterpret_cast<const uint8_t*>(command.data()),
            static_cast<int>(command.size()),
            [&wire,&calls](const uint8_t* data,int remaining) {
                ++calls;
                wire.append(reinterpret_cast<const char*>(data),
                            static_cast<size_t>(remaining));
                return SerialWriteAttempt::Progress(remaining);
            },[](){return true;},[](){});
        CHECK(report.outcome==SerialWriteOutcome::TimeoutNoBytes);
        CHECK(report.accepted_bytes==0 && report.write_attempts==0);
        CHECK(calls==0 && wire.empty() && stream.IsClean());
    }
    {
        // A low-level write may accept the complete line just as its host-side
        // budget expires.  The line is intact, but it is explicitly late.
        AsciiStreamState stream;
        std::string wire;
        bool expired=false;
        const SerialWriteReport report=WriteBoundedAsciiLine(
            stream,reinterpret_cast<const uint8_t*>(command.data()),
            static_cast<int>(command.size()),
            [&wire,&expired](const uint8_t* data,int remaining) {
                wire.append(reinterpret_cast<const char*>(data),
                            static_cast<size_t>(remaining));
                expired=true;
                return SerialWriteAttempt::Progress(remaining);
            },[&expired](){return expired;},[](){});
        CHECK(report.outcome==SerialWriteOutcome::CompleteLate);
        CHECK(report.IsComplete());
        CHECK(report.accepted_bytes==static_cast<int>(command.size()));
        CHECK(wire==command && stream.IsClean());
    }
    {
        AsciiStreamState stream;
        std::string wire;
        const std::vector<int> chunks{1,2,3,1,2,4};
        size_t next=0;
        int ticks=0;
        const SerialWriteReport report=WriteBoundedAsciiLine(
            stream,reinterpret_cast<const uint8_t*>(command.data()),
            static_cast<int>(command.size()),
            [&wire,&chunks,&next,&ticks](const uint8_t* data,int remaining) {
                ++ticks;
                if(next>=chunks.size()) return SerialWriteAttempt::Error();
                const int count=chunks[next++]<remaining?chunks[next-1]:remaining;
                wire.append(reinterpret_cast<const char*>(data),
                            static_cast<size_t>(count));
                return SerialWriteAttempt::Progress(count);
            },
            [&ticks]() { return ticks>=100; },
            [&ticks]() { ++ticks; });
        CHECK(report.IsComplete());
        CHECK(report.write_attempts==6);
        CHECK(report.accepted_bytes==static_cast<int>(command.size()));
        CHECK(stream.IsClean());
        CHECK(wire==command);
    }
    {
        // Positive progress must not bypass the deadline indefinitely.
        AsciiStreamState stream;
        std::string wire;
        int ticks=0;
        const SerialWriteReport report=WriteBoundedAsciiLine(
            stream,reinterpret_cast<const uint8_t*>(command.data()),
            static_cast<int>(command.size()),
            [&wire,&ticks](const uint8_t* data,int) {
                ++ticks; wire.push_back(static_cast<char>(*data));
                return SerialWriteAttempt::Progress(1);
            },
            [&ticks]() { return ticks>=3; },
            []() {});
        CHECK(report.outcome==SerialWriteOutcome::TimeoutPartial);
        CHECK(report.accepted_bytes==3 && report.write_attempts==3);
        CHECK(wire==command.substr(0,3));
        CHECK(stream.IsTainted());
    }
    {
        AsciiStreamState stream;
        int ticks=0;
        const SerialWriteReport report=WriteBoundedAsciiLine(
            stream,reinterpret_cast<const uint8_t*>(command.data()),
            static_cast<int>(command.size()),
            [&ticks](const uint8_t*,int) {
                ++ticks; return SerialWriteAttempt::Retry();
            },
            [&ticks]() { return ticks>=1; },
            []() {});
        CHECK(report.outcome==SerialWriteOutcome::TimeoutNoBytes);
        CHECK(report.accepted_bytes==0 && stream.IsClean());
    }
    {
        AsciiStreamState stream;
        std::string wire;
        int call=0;
        const SerialWriteReport report=WriteBoundedAsciiLine(
            stream,reinterpret_cast<const uint8_t*>(command.data()),
            static_cast<int>(command.size()),
            [&wire,&call](const uint8_t* data,int) {
                if(call++==0) {
                    wire.append(reinterpret_cast<const char*>(data),4);
                    return SerialWriteAttempt::Progress(4);
                }
                return SerialWriteAttempt::Error();
            },[](){return false;},[](){});
        CHECK(report.outcome==SerialWriteOutcome::ErrorPartial);
        CHECK(report.accepted_bytes==4 && stream.IsTainted());
        CHECK(wire==command.substr(0,4));

        int calls=0;
        const SerialWriteReport blocked=WriteBoundedAsciiLine(
            stream,reinterpret_cast<const uint8_t*>("1 stp 1\n"),8,
            [&calls](const uint8_t*,int){++calls;return SerialWriteAttempt::Error();},
            [](){return false;},[](){});
        CHECK(blocked.outcome==SerialWriteOutcome::BlockedTainted);
        CHECK(calls==0);
    }
    {
        AsciiStreamState stream;
        const SerialWriteReport report=WriteBoundedAsciiLine(
            stream,reinterpret_cast<const uint8_t*>(command.data()),
            static_cast<int>(command.size()),
            [](const uint8_t*,int){return SerialWriteAttempt::Error();},
            [](){return false;},[](){});
        CHECK(report.outcome==SerialWriteOutcome::ErrorNoBytes);
        CHECK(report.accepted_bytes==0 && report.write_attempts==1);
        CHECK(stream.IsClean());
    }
    {
        AsciiStreamState stream;
        int calls=0;
        const char invalid[]="1 pos 1";
        const SerialWriteReport report=WriteBoundedAsciiLine(
            stream,reinterpret_cast<const uint8_t*>(invalid),
            static_cast<int>(sizeof(invalid)-1),
            [&calls](const uint8_t*,int){++calls;return SerialWriteAttempt::Error();},
            [](){return false;},[](){});
        CHECK(report.outcome==SerialWriteOutcome::InvalidLine);
        CHECK(calls==0 && stream.IsClean());
    }
}

static void every_position_truncation_uses_bounded_writer_outcomes() {
    const std::string command="1 pos 652321\n";
    for(int cut=0;cut<static_cast<int>(command.size());++cut) {
        {
            AsciiStreamState stream;
            ScriptedLineWriter tx({cut});
            const SerialWriteReport report=tx.Write(
                stream,reinterpret_cast<const uint8_t*>(command.data()),
                static_cast<int>(command.size()));
            CHECK(report.outcome==(cut==0
                ?SerialWriteOutcome::ErrorNoBytes
                :SerialWriteOutcome::ErrorPartial));
            CHECK(report.accepted_bytes==cut);
            CHECK(report.write_attempts==(cut==0?1:2));
            CHECK(tx.wire==command.substr(0,static_cast<size_t>(cut)));
            CHECK(cut==0?stream.IsClean():stream.IsTainted());
            CHECK(ParsedPositionTargets(tx.wire).empty());
        }
        {
            AsciiStreamState stream;
            std::string wire;
            int accepted=0;
            const SerialWriteReport report=WriteBoundedAsciiLine(
                stream,reinterpret_cast<const uint8_t*>(command.data()),
                static_cast<int>(command.size()),
                [&wire,&accepted,cut](const uint8_t* data,int requested) {
                    const int count=cut-accepted<requested
                        ?cut-accepted:requested;
                    if(count<=0) return SerialWriteAttempt::Retry();
                    wire.append(reinterpret_cast<const char*>(data),
                                static_cast<size_t>(count));
                    accepted+=count;
                    return SerialWriteAttempt::Progress(count);
                },[&accepted,cut](){return accepted>=cut;},[](){});
            CHECK(report.outcome==(cut==0
                ?SerialWriteOutcome::TimeoutNoBytes
                :SerialWriteOutcome::TimeoutPartial));
            CHECK(report.accepted_bytes==cut);
            CHECK(wire==command.substr(0,static_cast<size_t>(cut)));
            CHECK(cut==0?stream.IsClean():stream.IsTainted());
            CHECK(ParsedPositionTargets(wire).empty());
        }
    }
}

static void every_position_truncation_blocks_fault_bytes() {
    const std::string command="1 pos 652321\n";
    const std::string stop="1 stp 1\n";
    for(int cut=0;cut<static_cast<int>(command.size());++cut) {
        AsciiStreamState stream;
        ScriptedLineWriter tx({cut,1000});
        const SerialWriteReport pos=tx.Write(
            stream,reinterpret_cast<const uint8_t*>(command.data()),
            static_cast<int>(command.size()));
        int queries=0;
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        const MotorStopReport fault_stop=ExecuteBoundedMotorStop(
            stream,MotorStopMode::Immediate,250,write_line,
            [&queries,&tx,&stream](int32_t){
                ++queries;
                return QueryThroughWire(tx,stream,StoppedStatus());
            });

        if(cut==0) {
            CHECK(pos.outcome==SerialWriteOutcome::ErrorNoBytes);
            CHECK(stream.IsClean());
            CHECK(fault_stop.command_written && fault_stop.driver_stop_confirmed);
            CHECK(queries==1 && tx.wire==stop+"1 sts\n");
        } else {
            CHECK(pos.outcome==SerialWriteOutcome::ErrorPartial);
            CHECK(pos.accepted_bytes==cut && stream.IsTainted());
            CHECK(fault_stop.command_write.outcome==SerialWriteOutcome::BlockedTainted);
            CHECK(!fault_stop.command_written && !fault_stop.driver_stop_confirmed);
            CHECK(fault_stop.stop_state_unknown);
            CHECK(fault_stop.hardware_intervention_required);
            CHECK(fault_stop.status.result==MotorStatusResult::StreamTainted);
            CHECK(queries==0);
            CHECK(tx.wire==command.substr(0,static_cast<size_t>(cut)));
        }

        const ReleaseFallbackDecision release=DecideReleaseFallback(
            stream,true,true,fault_stop.command_written,
            fault_stop.driver_stop_confirmed);
        if(cut==0) {
            CHECK(release.action==ReleaseFallbackAction::NoneNeeded);
            CHECK(!release.attempt_stop);
        } else {
            CHECK(release.action==ReleaseFallbackAction::BlockedTainted);
            CHECK(!release.attempt_stop && release.stop_state_unknown);
            CHECK(release.hardware_intervention_required);
        }
        CHECK(ParsedPositionTargets(tx.wire).empty());

        // Regression oracle: the former leading-newline recovery really would
        // turn each numeric prefix into another valid absolute-position line.
        if(cut>=7) {
            const std::string legacy=command.substr(0,static_cast<size_t>(cut))+
                                     "\n1 stp 1\n";
            const std::vector<long> targets=ParsedPositionTargets(legacy);
            CHECK(targets.size()==1);
            CHECK(targets.front()==std::strtol(
                command.substr(6,static_cast<size_t>(cut-6)).c_str(),nullptr,10));
        }
    }
}

static void every_stop_truncation_is_recorded_and_release_is_safe() {
    const std::string immediate="1 stp 1\n";
    const std::string decelerated="1 stp 0\n";
    for(int cut=0;cut<static_cast<int>(immediate.size());++cut) {
        AsciiStreamState stream;
        ScriptedLineWriter tx({cut,1000});
        int queries=0;
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        const MotorStopReport stop=ExecuteBoundedMotorStop(
            stream,MotorStopMode::Immediate,250,write_line,
            [&queries,&tx,&stream](int32_t){
                ++queries;
                return QueryThroughWire(tx,stream,StoppedStatus());
            });
        CHECK(!stop.command_written && !stop.driver_stop_confirmed);
        CHECK(stop.stop_state_unknown && stop.hardware_intervention_required);
        CHECK(queries==0);

        const ReleaseFallbackDecision release=DecideReleaseFallback(
            stream,true,true,stop.command_written,stop.driver_stop_confirmed);
        if(cut==0) {
            CHECK(stream.IsClean());
            CHECK(stop.command_write.outcome==SerialWriteOutcome::ErrorNoBytes);
            CHECK(release.action==ReleaseFallbackAction::AttemptDeceleratedStop);
            CHECK(release.attempt_stop);
            const MotorStopReport fallback=ExecuteBoundedMotorStop(
                stream,MotorStopMode::Decelerated,250,write_line,
                [&queries,&tx,&stream](int32_t){
                    ++queries;
                    return QueryThroughWire(tx,stream,StoppedStatus());
                });
            CHECK(fallback.command_written && fallback.driver_stop_confirmed);
            CHECK(tx.wire==decelerated+"1 sts\n");
        } else {
            CHECK(stream.IsTainted());
            CHECK(stop.command_write.outcome==SerialWriteOutcome::ErrorPartial);
            CHECK(release.action==ReleaseFallbackAction::BlockedTainted);
            CHECK(!release.attempt_stop);
            CHECK(tx.wire==immediate.substr(0,static_cast<size_t>(cut)));
        }
    }

    // The Release fallback itself may short-write.  A second fallback must not
    // append a newline or retry on the contaminated stream.
    for(int cut=1;cut<static_cast<int>(decelerated.size());++cut) {
        AsciiStreamState stream;
        ScriptedLineWriter tx({cut});
        const ReleaseFallbackDecision first=DecideReleaseFallback(
            stream,true,true,false,false);
        CHECK(first.attempt_stop);
        int queries=0;
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        const MotorStopReport fallback=ExecuteBoundedMotorStop(
            stream,MotorStopMode::Decelerated,250,write_line,
            [&queries,&tx,&stream](int32_t){
                ++queries;
                return QueryThroughWire(tx,stream,StoppedStatus());
            });
        CHECK(!fallback.command_written && stream.IsTainted());
        CHECK(queries==0);
        const ReleaseFallbackDecision second=DecideReleaseFallback(
            stream,true,true,false,false);
        CHECK(second.action==ReleaseFallbackAction::BlockedTainted);
        CHECK(!second.attempt_stop);
        CHECK(tx.wire==decelerated.substr(0,static_cast<size_t>(cut)));
    }
}

static void stop_confirmation_requires_a_post_command_driver_report() {
    const auto run=[](MotorStatusSnapshot status) {
        AsciiStreamState stream;
        ScriptedLineWriter tx({1000});
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        return ExecuteBoundedMotorStop(stream,MotorStopMode::Immediate,250,
                                       write_line,
                                       [&tx,&stream,status](int32_t){
                                           return QueryThroughWire(
                                               tx,stream,status);
                                       });
    };

    const MotorStatusSnapshot unassociated=StoppedStatus();
    CHECK(unassociated.IsValid());
    CHECK(!DriverReportsStopped(unassociated));
    MotorStatusSnapshot forged=StoppedStatus(true);
    forged.active_query_accepted_bytes=0;
    CHECK(!DriverReportsStopped(forged));

    MotorStatusSnapshot moving=StoppedStatus(); moving.speed=1.0f;
    CHECK(!run(moving).driver_stop_confirmed);
    MotorStatusSnapshot no_stp=StoppedStatus(); no_stp.status_flags=(1u<<13);
    CHECK(!run(no_stp).driver_stop_confirmed);
    CHECK(!run(TimeoutStatus()).driver_stop_confirmed);

    const MotorStopReport stopped=run(StoppedStatus());
    CHECK(stopped.driver_stop_confirmed && !stopped.stop_state_unknown);
    CHECK(!stopped.hardware_intervention_required);
}

static void homing_failure_cleanup_obeys_zero_stop_order() {
    const std::string zero_stop="1 zero stop\n";
    const std::string immediate="1 stp 1\n";
    {
        AsciiStreamState stream;
        ScriptedLineWriter tx({1000});
        int queries=0;
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        const HomingFailureStopReport report=ExecuteBoundedHomingFailureStop(
            stream,250,write_line,
            [&queries,&tx,&stream](int32_t){
                ++queries;
                return QueryThroughWire(tx,stream,StoppedStatus());
            });
        CHECK(report.zero_stop_written);
        CHECK(report.post_zero_status_query_attempted && queries==1);
        CHECK(report.driver_stop_confirmed && !report.stop_state_unknown);
        CHECK(!report.ordinary_stop_needed && !report.ordinary_stop_attempted);
        CHECK(tx.wire==zero_stop+"1 sts\n");
    }
    {
        // A valid post-query frame without the VSMD stp bit still requires the
        // ordinary stop and a second active query.
        AsciiStreamState stream;
        ScriptedLineWriter tx({1000,1000});
        int queries=0;
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        const HomingFailureStopReport report=ExecuteBoundedHomingFailureStop(
            stream,250,write_line,[&queries,&tx,&stream](int32_t) {
                ++queries;
                MotorStatusSnapshot response=StoppedStatus();
                if(queries==1) response.status_flags=(1u<<13);
                return QueryThroughWire(tx,stream,response);
            });
        CHECK(report.zero_stop_written && report.ordinary_stop_needed);
        CHECK(report.ordinary_stop_attempted && report.ordinary_stop.command_written);
        CHECK(report.driver_stop_confirmed && queries==2);
        CHECK(tx.wire==zero_stop+"1 sts\n"+immediate+"1 sts\n");
    }
    {
        // Even a post-query zero-speed/stp frame is not a completed homing abort
        // while VSMD status bit 14 still says zeroing is active.
        AsciiStreamState stream;
        ScriptedLineWriter tx({1000,1000});
        int queries=0;
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        const HomingFailureStopReport report=ExecuteBoundedHomingFailureStop(
            stream,250,write_line,[&queries,&tx,&stream](int32_t) {
                ++queries;
                MotorStatusSnapshot status=StoppedStatus();
                if(queries==1) status.status_flags|=(1u<<14);
                return QueryThroughWire(tx,stream,status);
            });
        CHECK(report.zero_stop_written && report.ordinary_stop_needed);
        CHECK(report.ordinary_stop_attempted && report.driver_stop_confirmed);
        CHECK(queries==2 &&
              tx.wire==zero_stop+"1 sts\n"+immediate+"1 sts\n");
    }
    {
        // Missing replies do not turn complete writes into stop proof.
        AsciiStreamState stream;
        ScriptedLineWriter tx({1000,1000});
        int queries=0;
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        const HomingFailureStopReport report=ExecuteBoundedHomingFailureStop(
            stream,250,write_line,
            [&queries,&tx,&stream](int32_t){
                ++queries;
                return QueryThroughWire(tx,stream,TimeoutStatus());
            });
        CHECK(report.zero_stop_written && report.ordinary_stop_attempted);
        CHECK(report.ordinary_stop.command_written && queries==2);
        CHECK(!report.driver_stop_confirmed && report.stop_state_unknown);
        CHECK(report.hardware_intervention_required);
        CHECK(tx.wire==zero_stop+"1 sts\n"+immediate+"1 sts\n");
    }
    {
        MotorStatusSnapshot moving=StoppedStatus(); moving.speed=3.0f;
        AsciiStreamState stream;
        ScriptedLineWriter tx({1000,1000});
        int queries=0;
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        const HomingFailureStopReport report=ExecuteBoundedHomingFailureStop(
            stream,250,write_line,[&queries,&tx,&stream,moving](int32_t) {
                ++queries;
                return QueryThroughWire(
                    tx,stream,queries==1?moving:StoppedStatus());
            });
        CHECK(report.ordinary_stop_needed && report.driver_stop_confirmed);
        CHECK(queries==2 &&
              tx.wire==zero_stop+"1 sts\n"+immediate+"1 sts\n");
    }
}

static void homing_short_writes_never_append_a_second_command() {
    const std::string zero_stop="1 zero stop\n";
    const std::string immediate="1 stp 1\n";
    for(int cut=0;cut<static_cast<int>(zero_stop.size());++cut) {
        AsciiStreamState stream;
        ScriptedLineWriter tx({cut,1000});
        int queries=0;
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        const HomingFailureStopReport report=ExecuteBoundedHomingFailureStop(
            stream,250,write_line,
            [&queries,&tx,&stream](int32_t){
                ++queries;
                return QueryThroughWire(tx,stream,StoppedStatus());
            });
        CHECK(!report.zero_stop_written && !report.driver_stop_confirmed);
        CHECK(!report.ordinary_stop_attempted && queries==0);
        CHECK(report.stop_state_unknown && report.hardware_intervention_required);
        CHECK(tx.wire==zero_stop.substr(0,static_cast<size_t>(cut)));
        if(cut==0) CHECK(stream.IsClean());
        else CHECK(stream.IsTainted());
    }

    // A short `sts` query after a complete zero-stop taints the same stream;
    // the ordinary stop is then blocked rather than terminating that prefix.
    {
        AsciiStreamState stream;
        ScriptedLineWriter tx({1000,3});
        int queries=0;
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        const HomingFailureStopReport report=ExecuteBoundedHomingFailureStop(
            stream,250,write_line,[&](int32_t) {
                ++queries;
                return QueryThroughWire(tx,stream,TimeoutStatus());
            });
        CHECK(queries==1 && stream.IsTainted());
        CHECK(report.ordinary_stop_needed && !report.ordinary_stop_attempted);
        CHECK(!report.driver_stop_confirmed && report.hardware_intervention_required);
        CHECK(tx.wire==zero_stop+std::string("1 s"));
    }

    // The ordinary stop itself may be short after a complete zero-stop and a
    // missing status reply.  No second status query or Release retry is safe.
    for(int cut=1;cut<static_cast<int>(immediate.size());++cut) {
        AsciiStreamState stream;
        ScriptedLineWriter tx({1000,1000,cut});
        int queries=0;
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        const HomingFailureStopReport report=ExecuteBoundedHomingFailureStop(
            stream,250,write_line,
            [&queries,&tx,&stream](int32_t){
                ++queries;
                return QueryThroughWire(tx,stream,TimeoutStatus());
            });
        CHECK(report.zero_stop_written && report.ordinary_stop_attempted);
        CHECK(!report.ordinary_stop.command_written && stream.IsTainted());
        CHECK(queries==1); // no query after the partial stp line
        CHECK(!report.driver_stop_confirmed && report.hardware_intervention_required);
        CHECK(tx.wire==zero_stop+"1 sts\n"+
              immediate.substr(0,static_cast<size_t>(cut)));
        const ReleaseFallbackDecision release=DecideReleaseFallback(
            stream,true,true,false,false);
        CHECK(release.action==ReleaseFallbackAction::BlockedTainted);
    }

    // A pre-existing partial zero-start/status line makes zero-stop itself
    // unsafe, so cleanup emits no bytes at all.
    {
        AsciiStreamState stream; stream.MarkPartialLine();
        ScriptedLineWriter tx({1000});
        int queries=0;
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        const HomingFailureStopReport report=ExecuteBoundedHomingFailureStop(
            stream,250,write_line,
            [&queries,&tx,&stream](int32_t){
                ++queries;
                return QueryThroughWire(tx,stream,StoppedStatus());
            });
        CHECK(report.zero_stop_write.outcome==SerialWriteOutcome::BlockedTainted);
        CHECK(tx.wire.empty() && tx.calls==0 && queries==0);
        CHECK(report.stop_state_unknown && report.hardware_intervention_required);
    }
}

static void complete_homing_orchestration_is_host_injectable() {
    const std::string zero_stop="1 zero stop\n";
    const std::string zero_start="1 zero start\n";
    const std::string sts="1 sts\n";
    {
        // Normal flow: clear a possibly stale bit-14 zero state, observe that
        // reset in a post-query frame, start homing, then observe origin completion.
        AsciiStreamState stream;
        ScriptedLineWriter tx({});
        std::vector<MotorStatusSnapshot> statuses;
        statuses.push_back(StoppedStatus());
        MotorStatusSnapshot at_origin=StoppedStatus();
        at_origin.driver_reported_position=0;
        CHECK(!DriverReportsHomingComplete(at_origin));
        at_origin.status_flags|=(1u<<14);
        statuses.push_back(at_origin);
        size_t next_status=0;
        int waits=0;
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        const auto query=[&](int32_t timeout_ms) {
            CHECK(timeout_ms==150);
            CHECK(next_status<statuses.size());
            const MotorStatusSnapshot response=next_status<statuses.size()
                ?statuses[next_status++]:TimeoutStatus();
            return QueryThroughWire(tx,stream,response);
        };
        const HomingExecutionReport report=ExecuteBoundedHoming(
            stream,150,write_line,query,[](){return false;},
            [](){return false;},[&waits](){++waits;});
        CHECK(report.result==HomingExecutionResult::Complete);
        CHECK(report.preparation_zero_stop_write.IsComplete());
        CHECK(report.preparation_status_query_attempted);
        CHECK(report.zero_start_written && report.homing_status_queries==1);
        CHECK(report.driver_home_confirmed && !report.cleanup_attempted);
        CHECK(!report.stop_state_unknown && !report.hardware_intervention_required);
        CHECK(waits==0 && next_status==2);
        CHECK(tx.wire==zero_stop+sts+zero_start+sts);
    }
    {
        // A completed home may leave bit 14 set at a nonzero position. Permit
        // zero-start only after an addressed immediate stop clears that stale
        // flag and its fresh status confirms the same ready predicate.
        AsciiStreamState stream;
        ScriptedLineWriter tx({});
        int query_count=0;
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        const auto query=[&](int32_t) {
            ++query_count;
            MotorStatusSnapshot status=StoppedStatus();
            if(query_count==1) status.status_flags|=(1u<<14);
            if(query_count==3) {
                status.driver_reported_position=0;
                status.status_flags|=(1u<<14);
            }
            return QueryThroughWire(tx,stream,status);
        };
        const HomingExecutionReport report=ExecuteBoundedHoming(
            stream,150,write_line,query,[](){return false;},
            [](){return false;},[](){});
        CHECK(report.result==HomingExecutionResult::Complete);
        CHECK(report.preparation_recovery_stop_attempted);
        CHECK(report.preparation_recovery_stop.driver_stop_confirmed);
        CHECK(DriverReportsReadyAfterZeroStop(
            report.preparation_recovery_stop.status));
        CHECK(report.zero_start_written && report.driver_home_confirmed);
        CHECK(!report.cleanup_attempted && !report.stop_state_unknown &&
              !report.hardware_intervention_required);
        CHECK(query_count==3 &&
              tx.wire==zero_stop+sts+"1 stp 1\n"+sts+zero_start+sts);
    }
    {
        // A stop command that does not clear bit 14 is not enough to start
        // homing. Preserve the recovery evidence and do not send zero-start.
        AsciiStreamState stream;
        ScriptedLineWriter tx({});
        int query_count=0;
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        const auto query=[&](int32_t) {
            ++query_count;
            MotorStatusSnapshot status=StoppedStatus();
            status.status_flags|=(1u<<14);
            return QueryThroughWire(tx,stream,status);
        };
        const HomingExecutionReport report=ExecuteBoundedHoming(
            stream,150,write_line,query,[](){return false;},
            [](){return false;},[](){});
        CHECK(report.result==HomingExecutionResult::PreparationStatusFailure);
        CHECK(!report.zero_start_written && report.cleanup_attempted);
        CHECK(report.preparation_recovery_stop_attempted);
        CHECK(report.failure_stop.zero_stop_reused_from_preparation);
        CHECK(report.failure_stop.ordinary_stop_attempted);
        CHECK(report.failure_stop.driver_stop_confirmed);
        CHECK(query_count==2 &&
              tx.wire==zero_stop+sts+"1 stp 1\n"+sts);
        CHECK(!report.stop_state_unknown &&
              !report.hardware_intervention_required);
    }
    {
        // A missing preparation status still owns the zero-stop-first shutdown
        // decision.  On a clean stream it uses one bounded ordinary stop and
        // requires a separate post-stop query before indicating standstill.
        AsciiStreamState stream;
        ScriptedLineWriter tx({});
        int query_count=0;
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        const auto query=[&](int32_t) {
            ++query_count;
            return QueryThroughWire(
                tx,stream,query_count==1?TimeoutStatus():StoppedStatus());
        };
        const HomingExecutionReport report=ExecuteBoundedHoming(
            stream,150,write_line,query,[](){return false;},
            [](){return false;},[](){});
        CHECK(report.result==HomingExecutionResult::PreparationStatusFailure);
        CHECK(report.cleanup_attempted &&
              report.failure_stop.zero_stop_reused_from_preparation);
        CHECK(report.failure_stop.ordinary_stop_attempted);
        CHECK(report.failure_stop.driver_stop_confirmed);
        CHECK(query_count==2 &&
              tx.wire==zero_stop+sts+"1 stp 1\n"+sts);
        CHECK(HomingExecutionReportsDriverStopped(report));
    }
    {
        // Interruption already present at entry still performs the documented
        // zero-stop and active query before deciding not to issue zero-start.
        AsciiStreamState stream;
        ScriptedLineWriter tx({});
        int query_count=0;
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        const auto query=[&](int32_t) {
            ++query_count;
            return QueryThroughWire(tx,stream,StoppedStatus());
        };
        const HomingExecutionReport report=ExecuteBoundedHoming(
            stream,150,write_line,query,[](){return false;},
            [](){return true;},[](){});
        CHECK(report.result==HomingExecutionResult::Interrupted);
        CHECK(!report.zero_start_written && !report.cleanup_attempted);
        CHECK(report.shutdown_sequence_handled);
        CHECK(report.failure_stop.zero_stop_reused_from_preparation);
        CHECK(query_count==1 && tx.wire==zero_stop+sts);
        CHECK(HomingExecutionReportsDriverStopped(report));
    }
    {
        // The overall deadline may expire while the preparation query is in
        // flight.  Its stopped result is retained, but zero-start is omitted.
        AsciiStreamState stream;
        ScriptedLineWriter tx({});
        bool expired=false;
        int query_count=0;
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        const auto query=[&](int32_t) {
            ++query_count;
            const MotorStatusSnapshot status=QueryThroughWire(
                tx,stream,StoppedStatus());
            expired=true;
            return status;
        };
        const HomingExecutionReport report=ExecuteBoundedHoming(
            stream,150,write_line,query,[&expired](){return expired;},
            [](){return false;},[](){});
        CHECK(report.result==HomingExecutionResult::Timeout);
        CHECK(!report.zero_start_written && !report.cleanup_attempted);
        CHECK(report.shutdown_sequence_handled);
        CHECK(report.failure_stop.zero_stop_reused_from_preparation);
        CHECK(query_count==1 && tx.wire==zero_stop+sts);
        CHECK(HomingExecutionReportsDriverStopped(report));
    }
    {
        // SIGINT can arrive during the completion query itself.  Even if that
        // query returns a stopped/zero/position-0 frame, it must not be accepted
        // as a normal completion after the interrupt.
        AsciiStreamState stream;
        ScriptedLineWriter tx({});
        bool interrupted=false;
        int query_count=0;
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        const auto query=[&](int32_t) {
            ++query_count;
            MotorStatusSnapshot response=StoppedStatus();
            if(query_count==2) {
                response.driver_reported_position=0;
                response.status_flags|=(1u<<14);
            }
            const MotorStatusSnapshot status=QueryThroughWire(
                tx,stream,response);
            if(query_count==2) interrupted=true;
            return status;
        };
        const HomingExecutionReport report=ExecuteBoundedHoming(
            stream,150,write_line,query,[](){return false;},
            [&interrupted](){return interrupted;},[](){});
        CHECK(report.result==HomingExecutionResult::Interrupted);
        CHECK(!report.driver_home_confirmed && report.cleanup_attempted);
        CHECK(report.failure_stop.zero_stop_written);
        CHECK(report.failure_stop.driver_stop_confirmed);
        CHECK(query_count==3);
        CHECK(tx.wire==zero_stop+sts+zero_start+sts+zero_stop+sts);
    }
    {
        // The same ordering applies when the overall deadline expires during
        // a query that otherwise looks like successful homing completion.
        AsciiStreamState stream;
        ScriptedLineWriter tx({});
        bool expired=false;
        int query_count=0;
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        const auto query=[&](int32_t) {
            ++query_count;
            MotorStatusSnapshot response=StoppedStatus();
            if(query_count==2) {
                response.driver_reported_position=0;
                response.status_flags|=(1u<<14);
            }
            const MotorStatusSnapshot status=QueryThroughWire(
                tx,stream,response);
            if(query_count==2) expired=true;
            return status;
        };
        const HomingExecutionReport report=ExecuteBoundedHoming(
            stream,150,write_line,query,[&expired](){return expired;},
            [](){return false;},[](){});
        CHECK(report.result==HomingExecutionResult::Timeout);
        CHECK(!report.driver_home_confirmed && report.cleanup_attempted);
        CHECK(report.failure_stop.zero_stop_written);
        CHECK(report.failure_stop.driver_stop_confirmed);
        CHECK(query_count==3);
        CHECK(tx.wire==zero_stop+sts+zero_start+sts+zero_stop+sts);
    }
    {
        // Overall timeout after a moving homing frame invokes zero-stop first;
        // a post-query stopped/zero-cleared cleanup frame avoids redundant stp.
        AsciiStreamState stream;
        ScriptedLineWriter tx({});
        MotorStatusSnapshot moving=StoppedStatus(); moving.speed=2.0f;
        std::vector<MotorStatusSnapshot> statuses{
            StoppedStatus(),moving,StoppedStatus()
        };
        size_t next_status=0;
        bool expired=false;
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        const auto query=[&](int32_t) {
            CHECK(next_status<statuses.size());
            const MotorStatusSnapshot response=next_status<statuses.size()
                ?statuses[next_status++]:TimeoutStatus();
            return QueryThroughWire(tx,stream,response);
        };
        const HomingExecutionReport report=ExecuteBoundedHoming(
            stream,150,write_line,query,[&expired](){return expired;},
            [](){return false;},[&expired](){expired=true;});
        CHECK(report.result==HomingExecutionResult::Timeout);
        CHECK(report.cleanup_attempted && report.failure_stop.zero_stop_written);
        CHECK(report.failure_stop.driver_stop_confirmed);
        CHECK(!report.hardware_intervention_required && !report.stop_state_unknown);
        CHECK(next_status==3);
        CHECK(tx.wire==zero_stop+sts+zero_start+sts+zero_stop+sts);
    }
    {
        // A missing active status frame fails immediately and follows the same
        // homing-specific cleanup ordering.
        AsciiStreamState stream;
        ScriptedLineWriter tx({});
        std::vector<MotorStatusSnapshot> statuses{
            StoppedStatus(),TimeoutStatus(),StoppedStatus()
        };
        size_t next_status=0;
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        const auto query=[&](int32_t) {
            CHECK(next_status<statuses.size());
            const MotorStatusSnapshot response=next_status<statuses.size()
                ?statuses[next_status++]:TimeoutStatus();
            return QueryThroughWire(tx,stream,response);
        };
        const HomingExecutionReport report=ExecuteBoundedHoming(
            stream,150,write_line,query,[](){return false;},
            [](){return false;},[](){});
        CHECK(report.result==HomingExecutionResult::StatusFailure);
        CHECK(report.cleanup_attempted && report.failure_stop.zero_stop_written);
        CHECK(report.failure_stop.driver_stop_confirmed && next_status==3);
        CHECK(tx.wire==zero_stop+sts+zero_start+sts+zero_stop+sts);
    }
    {
        // Interruption is sampled between bounded polls and executes zero-stop
        // before any ordinary stop command.
        AsciiStreamState stream;
        ScriptedLineWriter tx({});
        MotorStatusSnapshot moving=StoppedStatus(); moving.speed=-2.0f;
        std::vector<MotorStatusSnapshot> statuses{
            StoppedStatus(),moving,StoppedStatus()
        };
        size_t next_status=0;
        bool interrupted=false;
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        const auto query=[&](int32_t) {
            CHECK(next_status<statuses.size());
            const MotorStatusSnapshot response=next_status<statuses.size()
                ?statuses[next_status++]:TimeoutStatus();
            return QueryThroughWire(tx,stream,response);
        };
        const HomingExecutionReport report=ExecuteBoundedHoming(
            stream,150,write_line,query,[](){return false;},
            [&interrupted](){return interrupted;},
            [&interrupted](){interrupted=true;});
        CHECK(report.result==HomingExecutionResult::Interrupted);
        CHECK(report.cleanup_attempted && report.failure_stop.zero_stop_written);
        CHECK(report.failure_stop.driver_stop_confirmed && next_status==3);
        CHECK(tx.wire==zero_stop+sts+zero_start+sts+zero_stop+sts);
    }

    // Every incomplete zero-start write ends the flow immediately.  In
    // particular, the numeric/text prefix is never terminated by cleanup.
    for(int cut=0;cut<static_cast<int>(zero_start.size());++cut) {
        AsciiStreamState stream;
        ScriptedLineWriter tx({1000,1000,cut,1000});
        int query_count=0;
        const auto write_line=[&tx,&stream](const uint8_t* data,int len) {
            return tx.Write(stream,data,len);
        };
        const auto query=[&](int32_t) {
            ++query_count;
            return QueryThroughWire(tx,stream,StoppedStatus());
        };
        const HomingExecutionReport report=ExecuteBoundedHoming(
            stream,150,write_line,query,[](){return false;},
            [](){return false;},[](){});
        CHECK(report.result==HomingExecutionResult::StartWriteFailure);
        CHECK(!report.zero_start_written && !report.cleanup_attempted);
        CHECK(query_count==1);
        CHECK(tx.wire==zero_stop+sts+
              zero_start.substr(0,static_cast<size_t>(cut)));
        if(cut==0) {
            CHECK(stream.IsClean());
            CHECK(!report.stop_state_unknown);
        } else {
            CHECK(stream.IsTainted());
            CHECK(report.stop_state_unknown && report.hardware_intervention_required);
        }
    }
}

static void release_decision_never_treats_close_as_stop_proof() {
    AsciiStreamState clean;
    ReleaseFallbackDecision d=DecideReleaseFallback(
        clean,true,true,false,false);
    CHECK(d.action==ReleaseFallbackAction::AttemptDeceleratedStop);
    CHECK(d.attempt_stop && d.stop_state_unknown);

    d=DecideReleaseFallback(clean,true,true,true,false);
    CHECK(d.action==ReleaseFallbackAction::PriorStopUnconfirmed);
    CHECK(!d.attempt_stop && d.hardware_intervention_required);

    d=DecideReleaseFallback(clean,false,true,false,false);
    CHECK(d.action==ReleaseFallbackAction::SerialUnavailable);
    CHECK(!d.attempt_stop && d.stop_state_unknown);

    d=DecideReleaseFallback(clean,true,true,true,true);
    CHECK(d.action==ReleaseFallbackAction::NoneNeeded);
    CHECK(!d.stop_state_unknown && !d.hardware_intervention_required);

    d=DecideReleaseFallback(clean,true,false,false,false);
    CHECK(d.action==ReleaseFallbackAction::NoneNeeded);
    CHECK(!d.attempt_stop);

    AsciiStreamState tainted; tainted.MarkPartialLine();
    d=DecideReleaseFallback(tainted,true,true,false,false);
    CHECK(d.action==ReleaseFallbackAction::BlockedTainted);
    CHECK(!d.attempt_stop && d.hardware_intervention_required);
}

static void command_gate_preserves_all_other_limits() {
    RunawayGuard guard;
    guard.Init(7250);
    CommandGateDecision decision=EvaluateCommandProposal(
        guard,true,false,-1.0f,0.0f,107251,100000,0,2112000,7250);
    CHECK(!decision.bounds_ok && !decision.allowed && !decision.guard_trip);
    CHECK(decision.proposed_delta==7251); // per-beat physical limit unchanged

    guard.Reset();
    decision=EvaluateCommandProposal(
        guard,true,false,-1.0f,0.0f,107250,100000,0,2112000,7250);
    CHECK(decision.bounds_ok && decision.allowed && !decision.guard_trip);
    CHECK(decision.proposed_delta==7250); // exact per-beat boundary is allowed

    guard.Reset();
    decision=EvaluateCommandProposal(
        guard,true,false,-1.0f,0.0f,-1,1000,0,2112000,7250);
    CHECK(!decision.bounds_ok && !decision.allowed && !decision.guard_trip);
    CHECK(decision.proposed_delta==-1001); // absolute ODL bound unchanged

    guard.Reset();
    decision=EvaluateCommandProposal(
        guard,true,false,-1.0f,0.0f,0,1000,0,2112000,7250);
    CHECK(decision.bounds_ok && decision.allowed); // exact absolute bound allowed

    guard.Reset();
    decision=EvaluateCommandProposal(
        guard,true,false,-1.0f,0.0f,
        std::numeric_limits<int32_t>::max(),
        std::numeric_limits<int32_t>::min(),0,2112000,7250);
    CHECK(!decision.bounds_ok && !decision.allowed);
}

static void pi_integer_quantization_stays_within_physical_limit() {
    // Synthetic pulse boundary, unrelated to recorded experiments.
    const PhysicalStepCorrection quantized =
        CorrectSingleStepQuantizationOverrun(15001,10000,5000);
    CHECK(quantized.corrected_single_step_overrun);
    CHECK(quantized.target==15000);
    CHECK(quantized.target-10000==5000);
    RunawayGuard guard;
    guard.Init(5000);
    const CommandGateDecision quantized_gate=EvaluateCommandProposal(
        guard,true,false,-1.0f,0.0f,quantized.target,10000,0,2112000,5000);
    CHECK(quantized_gate.allowed && quantized_gate.bounds_ok);
    CHECK(quantized_gate.proposed_delta==5000);

    const PhysicalStepCorrection reverse =
        CorrectSingleStepQuantizationOverrun(9999,15000,5000);
    CHECK(reverse.corrected_single_step_overrun);
    CHECK(reverse.target==10000);
    CHECK(15000-reverse.target==5000);

    const PhysicalStepCorrection exact =
        CorrectSingleStepQuantizationOverrun(15000,10000,5000);
    CHECK(!exact.corrected_single_step_overrun && exact.target==15000);

    const PhysicalStepCorrection larger =
        CorrectSingleStepQuantizationOverrun(15002,10000,5000);
    CHECK(!larger.corrected_single_step_overrun && larger.target==15002);
    guard.Reset();
    const CommandGateDecision rejected=EvaluateCommandProposal(
        guard,true,false,-1.0f,0.0f,larger.target,10000,0,2112000,5000);
    CHECK(!rejected.allowed && !rejected.bounds_ok);
    CHECK(rejected.proposed_delta==5002);

    const PhysicalStepCorrection at_odl_edge =
        CorrectSingleStepQuantizationOverrun(5001,0,5000);
    CHECK(at_odl_edge.corrected_single_step_overrun && at_odl_edge.target==5000);
    guard.Reset();
    const CommandGateDecision lower_bound=EvaluateCommandProposal(
        guard,true,false,-1.0f,0.0f,-1,0,0,2112000,5000);
    CHECK(!lower_bound.allowed && !lower_bound.bounds_ok);
}

static void serial_failure_does_not_commit_software_acceptance() {
    RunawayGuard guard;
    guard.Init(5000);
    const CommandGateDecision decision=EvaluateCommandProposal(
        guard,true,false,-10.0f,0.0f,1051000,1056000,0,2112000,5000);
    CHECK(decision.allowed && !decision.guard_trip);
    CHECK(guard.DirectionalSteps()==5000);
    const bool complete_serial_write=false;
    if(complete_serial_write) guard.CommitAcceptedProposal();
    CHECK(guard.AcceptedDirectionalSteps()==0);
}

static void guard_rejects_before_dangerous_send() {
    RunawayGuard guard;
    guard.Init(7250);
    int32_t accepted=804445;
    int sends=0;
    for(int i=0;i<27;++i) {
        const int32_t proposed=accepted-7250;
        const CommandGateDecision decision=EvaluateCommandProposal(
            guard,true,false,-40.0f,0.0f,proposed,accepted,0,2112000,7250);
        CHECK(decision.allowed);
        if(decision.allowed) {
            ++sends;
            guard.CommitAcceptedProposal();
            accepted=proposed;
        }
    }
    CHECK(guard.AcceptedDirectionalSteps()==195750);
    const int32_t rejected=accepted-7250;
    const CommandGateDecision trip=EvaluateCommandProposal(
        guard,true,false,-40.0f,0.0f,rejected,accepted,0,2112000,7250);
    if(trip.allowed) ++sends;
    CHECK(trip.guard_trip && !trip.allowed);
    CHECK(guard.TripMask()==RunawayGuard::DIRECTIONAL_TRAVEL);
    CHECK(guard.DirectionalSteps()==203000);
    CHECK(guard.AcceptedDirectionalSteps()==195750);
    CHECK(sends==27); // the trip proposal never reached the send callback
}

int main() {
    bounded_ascii_writer_tracks_every_byte_and_deadline();
    every_position_truncation_uses_bounded_writer_outcomes();
    every_position_truncation_blocks_fault_bytes();
    every_stop_truncation_is_recorded_and_release_is_safe();
    stop_confirmation_requires_a_post_command_driver_report();
    homing_failure_cleanup_obeys_zero_stop_order();
    homing_short_writes_never_append_a_second_command();
    complete_homing_orchestration_is_host_injectable();
    release_decision_never_treats_close_as_stop_proof();
    command_gate_preserves_all_other_limits();
    pi_integer_quantization_stays_within_physical_limit();
    serial_failure_does_not_commit_software_acceptance();
    guard_rejects_before_dangerous_send();
    std::printf("safety path checks=%d failures=%d; hardware_actions=0\n",
                checks,failures);
    return failures?EXIT_FAILURE:EXIT_SUCCESS;
}
