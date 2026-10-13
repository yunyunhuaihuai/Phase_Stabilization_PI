#ifndef MOTOR_SAFETY_H
#define MOTOR_SAFETY_H

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

// Driver status is diagnostic feedback from the VSMD controller.  It is not
// an independent measurement of the mechanical ODL position.
enum class MotorStatusResult : int8_t {
    NotAttempted = 0,
    Valid = 1,
    NoData = 2,
    Busy = 3,
    Timeout = -1,
    WriteFailure = -2,
    ReadFailure = -3,
    ProtocolError = -4,
    Unsupported = -5,
    StreamTainted = -6
};

struct MotorStatusSnapshot {
    MotorStatusResult result = MotorStatusResult::NotAttempted;
    float speed = std::numeric_limits<float>::quiet_NaN();
    int32_t driver_reported_position = 0;
    uint32_t status_flags = 0;

    // True only when this frame was received after a complete active `sts`
    // query was sent for the operation being verified.  The protocol has no
    // request identifier, so temporal ordering does not prove response
    // attribution.  This is only a VSMD internal report, not independent
    // mechanical feedback.
    bool observed_after_active_query = false;
    // Host-side write details for the active `sts` line that preceded this
    // result.  These primitive fields avoid conflating a complete local write
    // with a returned status frame, while keeping the snapshot self-contained.
    int active_query_requested_bytes = 0;
    int active_query_accepted_bytes = 0;
    int active_query_write_attempts = 0;
    int active_query_write_outcome = 0;

    bool IsValid() const {
        return result == MotorStatusResult::Valid && std::isfinite(speed);
    }
};

// A write accepted by the host serial driver is not proof that VSMD parsed or
// executed it.  More importantly, once any non-empty prefix of an ASCII line
// has been accepted, appending a newline (including as part of a stop command)
// could turn that prefix into a different valid command.  The stream therefore
// stays tainted until an explicit out-of-band hardware recovery; closing and
// reopening the file descriptor is not such a recovery.
enum class AsciiStreamIntegrity : int8_t {
    Clean = 0,
    TaintedPartialLine = 1
};

struct AsciiStreamState {
    AsciiStreamIntegrity integrity = AsciiStreamIntegrity::Clean;

    bool IsClean() const {
        return integrity == AsciiStreamIntegrity::Clean;
    }
    bool IsTainted() const { return !IsClean(); }
    void MarkPartialLine() {
        integrity = AsciiStreamIntegrity::TaintedPartialLine;
    }
};

enum class SerialWriteOutcome : int8_t {
    NotAttempted = 0,
    Complete = 1,
    CompleteLate = 2,
    TimeoutNoBytes = -1,
    TimeoutPartial = -2,
    ErrorNoBytes = -3,
    ErrorPartial = -4,
    BlockedTainted = -5,
    InvalidLine = -6,
    BusyNoBytes = -7
};

struct SerialWriteReport {
    SerialWriteOutcome outcome = SerialWriteOutcome::NotAttempted;
    int requested_bytes = 0;
    int accepted_bytes = 0;
    int write_attempts = 0;

    bool IsComplete() const {
        return (outcome == SerialWriteOutcome::Complete ||
                outcome == SerialWriteOutcome::CompleteLate) &&
               requested_bytes > 0 && accepted_bytes == requested_bytes;
    }
    bool IsPartial() const {
        return accepted_bytes > 0 && accepted_bytes < requested_bytes;
    }
    bool AcceptedNoBytes() const { return accepted_bytes == 0; }
};

inline const char* SerialWriteOutcomeName(SerialWriteOutcome outcome) {
    switch (outcome) {
        case SerialWriteOutcome::NotAttempted: return "not_attempted";
        case SerialWriteOutcome::Complete: return "complete";
        case SerialWriteOutcome::CompleteLate: return "complete_late";
        case SerialWriteOutcome::TimeoutNoBytes: return "timeout_no_bytes";
        case SerialWriteOutcome::TimeoutPartial: return "timeout_partial";
        case SerialWriteOutcome::ErrorNoBytes: return "error_no_bytes";
        case SerialWriteOutcome::ErrorPartial: return "error_partial";
        case SerialWriteOutcome::BlockedTainted: return "blocked_tainted";
        case SerialWriteOutcome::InvalidLine: return "invalid_line";
        case SerialWriteOutcome::BusyNoBytes: return "busy_no_bytes";
    }
    return "unknown";
}

enum class SerialWriteAttemptKind : int8_t {
    Progress = 0,
    Retry = 1,
    Error = 2
};

struct SerialWriteAttempt {
    SerialWriteAttemptKind kind = SerialWriteAttemptKind::Error;
    int bytes = 0;

    static SerialWriteAttempt Progress(int count) {
        SerialWriteAttempt result;
        result.kind = SerialWriteAttemptKind::Progress;
        result.bytes = count;
        return result;
    }
    static SerialWriteAttempt Retry() {
        SerialWriteAttempt result;
        result.kind = SerialWriteAttemptKind::Retry;
        return result;
    }
    static SerialWriteAttempt Error() { return SerialWriteAttempt(); }
};

inline bool IsSingleAsciiLine(const uint8_t* data, int len) {
    if (!data || len <= 0 || data[len - 1] != '\n') return false;
    for (int i = 0; i + 1 < len; ++i) {
        if (data[i] == '\n') return false;
    }
    return true;
}

inline SerialWriteReport BlockedAsciiWrite(int len) {
    SerialWriteReport report;
    report.outcome = SerialWriteOutcome::BlockedTainted;
    report.requested_bytes = len > 0 ? len : 0;
    return report;
}

inline void FinishIncompleteAsciiWrite(AsciiStreamState& stream,
                                       SerialWriteReport& report,
                                       bool timeout) {
    if (report.accepted_bytes > 0) {
        report.outcome = timeout ? SerialWriteOutcome::TimeoutPartial
                                 : SerialWriteOutcome::ErrorPartial;
        stream.MarkPartialLine();
    } else {
        report.outcome = timeout ? SerialWriteOutcome::TimeoutNoBytes
                                 : SerialWriteOutcome::ErrorNoBytes;
    }
}

// Stateful, transport-injectable bounded whole-line writer.  write_some must
// perform one non-blocking low-level write and return Progress/Retry/Error.
// deadline_expired is checked after every partial progress as well as after a
// retry, so a stream of positive short writes cannot silently evade the bound.
template <typename WriteSomeFn, typename DeadlineFn, typename BackoffFn>
SerialWriteReport WriteBoundedAsciiLine(AsciiStreamState& stream,
                                        const uint8_t* data,
                                        int len,
                                        WriteSomeFn write_some,
                                        DeadlineFn deadline_expired,
                                        BackoffFn backoff) {
    if (stream.IsTainted()) return BlockedAsciiWrite(len);

    SerialWriteReport report;
    report.requested_bytes = len > 0 ? len : 0;
    if (!IsSingleAsciiLine(data, len)) {
        report.outcome = SerialWriteOutcome::InvalidLine;
        return report;
    }

    while (report.accepted_bytes < len) {
        if (deadline_expired()) {
            FinishIncompleteAsciiWrite(stream, report, true);
            return report;
        }
        const int remaining = len - report.accepted_bytes;
        const SerialWriteAttempt attempt = write_some(
            data + report.accepted_bytes, remaining);
        ++report.write_attempts;

        if (attempt.kind == SerialWriteAttemptKind::Progress) {
            if (attempt.bytes <= 0 || attempt.bytes > remaining) {
                FinishIncompleteAsciiWrite(stream, report, false);
                return report;
            }
            report.accepted_bytes += attempt.bytes;
            if (report.accepted_bytes == len) {
                report.outcome = deadline_expired()
                    ? SerialWriteOutcome::CompleteLate
                    : SerialWriteOutcome::Complete;
                return report;
            }
            if (deadline_expired()) {
                FinishIncompleteAsciiWrite(stream, report, true);
                return report;
            }
            continue;
        }

        if (attempt.kind == SerialWriteAttemptKind::Error) {
            FinishIncompleteAsciiWrite(stream, report, false);
            return report;
        }

        if (deadline_expired()) {
            FinishIncompleteAsciiWrite(stream, report, true);
            return report;
        }
        backoff();
        if (deadline_expired()) {
            FinishIncompleteAsciiWrite(stream, report, true);
            return report;
        }
    }

    // The loop can only terminate through a complete write or a failure.
    FinishIncompleteAsciiWrite(stream, report, false);
    return report;
}

// Adapter for an already-bounded whole-write function.  This is useful while
// migrating callers and is also the production classification point for a
// write function that returns exactly the number of accepted bytes.
template <typename WholeWriteFn>
SerialWriteReport WriteAsciiLineOnce(AsciiStreamState& stream,
                                     const uint8_t* data,
                                     int len,
                                     WholeWriteFn write_fn) {
    if (stream.IsTainted()) return BlockedAsciiWrite(len);

    SerialWriteReport report;
    report.requested_bytes = len > 0 ? len : 0;
    if (!IsSingleAsciiLine(data, len)) {
        report.outcome = SerialWriteOutcome::InvalidLine;
        return report;
    }

    const int written = write_fn(data, len);
    report.write_attempts = 1;
    if (written == len) {
        report.accepted_bytes = written;
        report.outcome = SerialWriteOutcome::Complete;
    } else if (written > 0 && written < len) {
        report.accepted_bytes = written;
        report.outcome = SerialWriteOutcome::ErrorPartial;
        stream.MarkPartialLine();
    } else {
        report.outcome = SerialWriteOutcome::ErrorNoBytes;
    }
    return report;
}

enum class MotorStopMode : int8_t {
    Decelerated = 0,
    Immediate = 1
};

struct MotorStopReport {
    MotorStopMode mode = MotorStopMode::Decelerated;
    SerialWriteReport command_write;
    bool command_written = false;  // compatibility alias for IsComplete()
    bool status_query_attempted = false;
    bool driver_stop_confirmed = false;
    bool stop_state_unknown = true;
    bool hardware_intervention_required = false;
    MotorStatusSnapshot status;
};

inline const char* MotorStatusResultName(MotorStatusResult result) {
    switch (result) {
        case MotorStatusResult::NotAttempted: return "not_attempted";
        case MotorStatusResult::Valid: return "valid";
        case MotorStatusResult::NoData: return "no_data";
        case MotorStatusResult::Busy: return "busy";
        case MotorStatusResult::Timeout: return "timeout";
        case MotorStatusResult::WriteFailure: return "write_failure";
        case MotorStatusResult::ReadFailure: return "read_failure";
        case MotorStatusResult::ProtocolError: return "protocol_error";
        case MotorStatusResult::Unsupported: return "unsupported";
        case MotorStatusResult::StreamTainted: return "stream_tainted";
    }
    return "unknown";
}

inline const char* MotorStopModeName(MotorStopMode mode) {
    return mode == MotorStopMode::Immediate ? "immediate" : "decelerated";
}

inline bool ActiveStatusQueryWriteComplete(
        const MotorStatusSnapshot& status) {
    const SerialWriteOutcome outcome = static_cast<SerialWriteOutcome>(
        status.active_query_write_outcome);
    return status.observed_after_active_query &&
           (outcome == SerialWriteOutcome::Complete ||
            outcome == SerialWriteOutcome::CompleteLate) &&
           status.active_query_requested_bytes > 0 &&
           status.active_query_accepted_bytes ==
               status.active_query_requested_bytes &&
           status.active_query_write_attempts > 0;
}

inline bool DriverReportsStopped(const MotorStatusSnapshot& status) {
    // VSMD v3.2: status bit 8 is stp (1 = stopped).  The indication also
    // requires a frame observed after the active query was sent.  Because the
    // protocol has no request id, this is not proof that the frame belongs to
    // that query; it is never independent mechanical proof.
    static const uint32_t VSMD_STATUS_STOPPED = (1u << 8);
    return ActiveStatusQueryWriteComplete(status) && status.IsValid() &&
           std::fabs(status.speed) < 0.001f &&
           (status.status_flags & VSMD_STATUS_STOPPED) != 0;
}

inline bool DriverReportsReadyAfterZeroStop(const MotorStatusSnapshot& status) {
    // VSMD status bit 14 is zero.  After `zero stop`, a stopped-looking frame
    // is not sufficient until the driver also reports that zeroing is no
    // longer active.  The later ordinary-stp confirmation intentionally uses
    // DriverReportsStopped() and does not impose this homing-only condition.
    static const uint32_t VSMD_STATUS_ZERO_ACTIVE = (1u << 14);
    return DriverReportsStopped(status) &&
           (status.status_flags & VSMD_STATUS_ZERO_ACTIVE) == 0;
}

// Shared stop sequence for a stateful ASCII stream.  write_line must use the
// same stream state.  There is deliberately no leading newline: if a previous
// line was partial, this function sends no bytes at all.
template <typename WriteLineFn, typename QueryFn>
MotorStopReport ExecuteBoundedMotorStop(AsciiStreamState& stream,
                                        MotorStopMode mode,
                                        int32_t verify_timeout_ms,
                                        WriteLineFn write_line,
                                        QueryFn query_fn) {
    MotorStopReport report;
    report.mode = mode;
    const char* command = mode == MotorStopMode::Immediate
        ? "1 stp 1\n"
        : "1 stp 0\n";
    const int command_length = static_cast<int>(std::strlen(command));

    if (stream.IsTainted()) {
        report.command_write = BlockedAsciiWrite(command_length);
        report.status.result = MotorStatusResult::StreamTainted;
        report.hardware_intervention_required = true;
        return report;
    }

    report.command_write = write_line(
        reinterpret_cast<const uint8_t*>(command), command_length);
    if (report.command_write.IsPartial()) stream.MarkPartialLine();
    report.command_written = report.command_write.IsComplete();
    if (!report.command_written) {
        report.status.result = stream.IsTainted()
            ? MotorStatusResult::StreamTainted
            : MotorStatusResult::WriteFailure;
        report.hardware_intervention_required = true;
        return report;
    }

    if (verify_timeout_ms <= 0) {
        report.status.result = MotorStatusResult::NotAttempted;
        report.hardware_intervention_required = true;
        return report;
    }

    report.status_query_attempted = true;
    report.status = query_fn(verify_timeout_ms);
    report.driver_stop_confirmed = stream.IsClean() &&
                                   DriverReportsStopped(report.status);
    report.stop_state_unknown = !report.driver_stop_confirmed;
    report.hardware_intervention_required = report.stop_state_unknown ||
                                            stream.IsTainted();
    if (stream.IsTainted()) {
        report.driver_stop_confirmed = false;
        report.stop_state_unknown = true;
        report.hardware_intervention_required = true;
    }
    return report;
}

struct HomingFailureStopReport {
    SerialWriteReport zero_stop_write;
    bool zero_stop_written = false;
    // True when this evidence is the preparatory zero-stop already logged,
    // rather than a second command emitted after zero-start.
    bool zero_stop_reused_from_preparation = false;
    bool post_zero_status_query_attempted = false;
    MotorStatusSnapshot post_zero_status;
    bool ordinary_stop_needed = false;
    bool ordinary_stop_attempted = false;
    MotorStopReport ordinary_stop;
    bool driver_stop_confirmed = false;
    bool stop_state_unknown = true;
    bool hardware_intervention_required = false;
};

// VSMD v3.2 homing-failure order: first complete `zero stop`, then obtain a
// post-command active status.  Only if that status does not report stopped may
// an addressed immediate stop be attempted.  A partial zero/status/stop line
// taints the stream and suppresses every subsequent write.
template <typename WriteLineFn, typename QueryFn>
HomingFailureStopReport ExecuteBoundedHomingFailureStop(
        AsciiStreamState& stream,
        int32_t verify_timeout_ms,
        WriteLineFn write_line,
        QueryFn query_fn) {
    HomingFailureStopReport report;
    const char command[] = "1 zero stop\n";
    const int command_length = static_cast<int>(sizeof(command) - 1);

    if (stream.IsTainted()) {
        report.zero_stop_write = BlockedAsciiWrite(command_length);
        report.post_zero_status.result = MotorStatusResult::StreamTainted;
        report.hardware_intervention_required = true;
        return report;
    }

    report.zero_stop_write = write_line(
        reinterpret_cast<const uint8_t*>(command), command_length);
    if (report.zero_stop_write.IsPartial()) stream.MarkPartialLine();
    report.zero_stop_written = report.zero_stop_write.IsComplete();
    if (!report.zero_stop_written) {
        report.post_zero_status.result = stream.IsTainted()
            ? MotorStatusResult::StreamTainted
            : MotorStatusResult::WriteFailure;
        report.hardware_intervention_required = true;
        return report;
    }

    if (verify_timeout_ms > 0) {
        report.post_zero_status_query_attempted = true;
        report.post_zero_status = query_fn(verify_timeout_ms);
    }
    if (stream.IsClean() &&
        DriverReportsReadyAfterZeroStop(report.post_zero_status)) {
        report.driver_stop_confirmed = true;
        report.stop_state_unknown = false;
        return report;
    }

    report.ordinary_stop_needed = true;
    if (stream.IsTainted()) {
        report.ordinary_stop.status.result = MotorStatusResult::StreamTainted;
        report.ordinary_stop.hardware_intervention_required = true;
        report.hardware_intervention_required = true;
        return report;
    }

    report.ordinary_stop_attempted = true;
    report.ordinary_stop = ExecuteBoundedMotorStop(
        stream, MotorStopMode::Immediate, verify_timeout_ms,
        write_line, query_fn);
    report.driver_stop_confirmed = report.ordinary_stop.driver_stop_confirmed;
    report.stop_state_unknown = !report.driver_stop_confirmed;
    report.hardware_intervention_required = report.stop_state_unknown ||
                                            stream.IsTainted();
    return report;
}

enum class HomingExecutionResult : int8_t {
    NotAttempted = 0,
    Complete = 1,
    Interrupted = -1,
    PreparationWriteFailure = -2,
    PreparationStatusFailure = -3,
    StartWriteFailure = -4,
    StatusFailure = -5,
    Timeout = -6
};

inline const char* HomingExecutionResultName(HomingExecutionResult result) {
    switch (result) {
        case HomingExecutionResult::NotAttempted: return "not_attempted";
        case HomingExecutionResult::Complete: return "complete";
        case HomingExecutionResult::Interrupted: return "interrupted";
        case HomingExecutionResult::PreparationWriteFailure:
            return "preparation_write_failure";
        case HomingExecutionResult::PreparationStatusFailure:
            return "preparation_status_failure";
        case HomingExecutionResult::StartWriteFailure:
            return "start_write_failure";
        case HomingExecutionResult::StatusFailure: return "status_failure";
        case HomingExecutionResult::Timeout: return "timeout";
    }
    return "unknown";
}

struct HomingExecutionReport {
    HomingExecutionResult result = HomingExecutionResult::NotAttempted;
    SerialWriteReport preparation_zero_stop_write;
    bool preparation_status_query_attempted = false;
    MotorStatusSnapshot preparation_status;
    // A completed homing can leave bit 14 set at a nonzero position. If the
    // post-zero-stop frame is otherwise stopped, clear that stale indication
    // with an addressed immediate stop and require a fresh ready status before
    // issuing zero-start.
    bool preparation_recovery_stop_attempted = false;
    MotorStopReport preparation_recovery_stop;
    SerialWriteReport zero_start_write;
    bool zero_start_written = false;
    int homing_status_queries = 0;
    MotorStatusSnapshot completion_status;
    bool cleanup_attempted = false;
    // True when this state machine has taken ownership of a failed homing
    // shutdown decision.  main must not append an unconditional ordinary stop
    // after such a failure; the dedicated zero-stop-first sequence either ran
    // or was deliberately suppressed because the ASCII stream was unsafe.
    bool shutdown_sequence_handled = false;
    HomingFailureStopReport failure_stop;
    bool driver_home_confirmed = false;
    bool stop_state_unknown = true;
    bool hardware_intervention_required = false;
};

inline bool DriverReportsHomingComplete(const MotorStatusSnapshot& status) {
    // VSMD uses bit 14 differently across the two phases: zero-stop clears it,
    // while a successfully completed zero-start reports zero=1 at position 0.
    static const uint32_t VSMD_STATUS_ZERO_COMPLETE = (1u << 14);
    return DriverReportsStopped(status) &&
           (status.status_flags & VSMD_STATUS_ZERO_COMPLETE) != 0 &&
           status.driver_reported_position == 0;
}

// Consolidated driver-internal stop indication for a completed homing state
// machine.  This never means independent mechanical standstill.  A zero-byte
// zero-start failure may reuse the post-query preparation status because no byte of
// the motion command entered the host serial stack; any partial start remains
// UNKNOWN and can never satisfy this predicate.
inline bool HomingExecutionReportsDriverStopped(
        const HomingExecutionReport& report) {
    if (report.stop_state_unknown) return false;
    if (report.result == HomingExecutionResult::Complete) {
        return report.driver_home_confirmed &&
               DriverReportsHomingComplete(report.completion_status);
    }
    const MotorStatusSnapshot& preparation_ready_status =
        report.preparation_recovery_stop_attempted
            ? report.preparation_recovery_stop.status
            : report.preparation_status;
    if (report.result == HomingExecutionResult::StartWriteFailure &&
        report.zero_start_write.requested_bytes > 0 &&
        report.zero_start_write.AcceptedNoBytes() &&
        DriverReportsReadyAfterZeroStop(preparation_ready_status)) {
        return true;
    }
    const HomingFailureStopReport& stop = report.failure_stop;
    if (!stop.driver_stop_confirmed || stop.stop_state_unknown) return false;
    if (stop.ordinary_stop_attempted) {
        return stop.ordinary_stop.driver_stop_confirmed &&
               !stop.ordinary_stop.stop_state_unknown &&
               DriverReportsStopped(stop.ordinary_stop.status);
    }
    return stop.zero_stop_written &&
           DriverReportsReadyAfterZeroStop(stop.post_zero_status);
}

// Production-reusable homing orchestration. Before every zero start, a
// complete zero stop and a fresh ready status are required. If the driver is
// already stopped but still reports bit 14 from a previous completed home,
// an immediate stop may clear that stale indication; its own fresh status
// must satisfy the same readiness predicate before zero-start is allowed.
// Once zero start is complete, timeout, missing/invalid status or interruption
// always enters the homing-specific cleanup above. A partial start line is
// not followed by any bytes.
//
// deadline_expired and interrupted are side-effect-free predicates. wait_poll
// is the only pacing hook, making the same state machine deterministic in host
// tests and bounded by the caller's monotonic deadline in production.
template <typename WriteLineFn, typename QueryFn, typename DeadlineFn,
          typename InterruptedFn, typename WaitFn>
HomingExecutionReport ExecuteBoundedHoming(
        AsciiStreamState& stream,
        int32_t status_timeout_ms,
        WriteLineFn write_line,
        QueryFn query_fn,
        DeadlineFn deadline_expired,
        InterruptedFn interrupted,
        WaitFn wait_poll) {
    HomingExecutionReport report;
    const char prepare_command[] = "1 zero stop\n";
    report.preparation_zero_stop_write = write_line(
        reinterpret_cast<const uint8_t*>(prepare_command),
        static_cast<int>(sizeof(prepare_command) - 1));
    if (report.preparation_zero_stop_write.IsPartial()) {
        stream.MarkPartialLine();
    }
    if (!report.preparation_zero_stop_write.IsComplete()) {
        report.result = HomingExecutionResult::PreparationWriteFailure;
        report.shutdown_sequence_handled = true;
        report.hardware_intervention_required = true;
        return report;
    }

    // Reuse the already-issued preparatory zero-stop as the first command of
    // the failure sequence. If the recovery stop was already attempted, keep
    // and report that exact transaction rather than sending a duplicate.
    // Otherwise, attempt one addressed ordinary stop only while the shared
    // ASCII stream is still clean. In particular, main and Release must not
    // append another command after this decision is owned.
    const auto fail_preparation_status = [&]() {
        report.result = HomingExecutionResult::PreparationStatusFailure;
        report.cleanup_attempted = true;
        report.shutdown_sequence_handled = true;
        HomingFailureStopReport& stop = report.failure_stop;
        stop.zero_stop_write = report.preparation_zero_stop_write;
        stop.zero_stop_written = stop.zero_stop_write.IsComplete();
        stop.zero_stop_reused_from_preparation = true;
        stop.post_zero_status_query_attempted =
            report.preparation_status_query_attempted;
        stop.post_zero_status = report.preparation_status;
        stop.ordinary_stop_needed = true;
        if (report.preparation_recovery_stop_attempted) {
            stop.ordinary_stop_attempted = true;
            stop.ordinary_stop = report.preparation_recovery_stop;
            stop.driver_stop_confirmed =
                stop.ordinary_stop.driver_stop_confirmed;
            stop.stop_state_unknown = !stop.driver_stop_confirmed;
            stop.hardware_intervention_required =
                stop.stop_state_unknown || stream.IsTainted();
        } else if (stream.IsTainted()) {
            stop.ordinary_stop.status.result = MotorStatusResult::StreamTainted;
            stop.ordinary_stop.hardware_intervention_required = true;
            stop.stop_state_unknown = true;
            stop.hardware_intervention_required = true;
        } else {
            stop.ordinary_stop_attempted = true;
            stop.ordinary_stop = ExecuteBoundedMotorStop(
                stream, MotorStopMode::Immediate, status_timeout_ms,
                write_line, query_fn);
            stop.driver_stop_confirmed =
                stop.ordinary_stop.driver_stop_confirmed;
            stop.stop_state_unknown = !stop.driver_stop_confirmed;
            stop.hardware_intervention_required =
                stop.stop_state_unknown || stream.IsTainted();
        }
        report.stop_state_unknown = stop.stop_state_unknown;
        report.hardware_intervention_required =
            stop.hardware_intervention_required;
    };

    if (status_timeout_ms <= 0) {
        fail_preparation_status();
        return report;
    }
    report.preparation_status_query_attempted = true;
    report.preparation_status = query_fn(status_timeout_ms);

    if (!stream.IsTainted() &&
        DriverReportsStopped(report.preparation_status) &&
        (report.preparation_status.status_flags & (1u << 14)) != 0) {
        report.preparation_recovery_stop_attempted = true;
        report.preparation_recovery_stop = ExecuteBoundedMotorStop(
            stream, MotorStopMode::Immediate, status_timeout_ms,
            write_line, query_fn);
    }

    const MotorStatusSnapshot& ready_status =
        report.preparation_recovery_stop_attempted
            ? report.preparation_recovery_stop.status
            : report.preparation_status;
    const bool recovered_ready =
        report.preparation_recovery_stop_attempted &&
        report.preparation_recovery_stop.driver_stop_confirmed &&
        DriverReportsReadyAfterZeroStop(ready_status);
    if (stream.IsTainted() ||
        (!DriverReportsReadyAfterZeroStop(report.preparation_status) &&
         !recovered_ready)) {
        fail_preparation_status();
        return report;
    }

    // A signal or elapsed overall budget may arrive during the bounded status
    // query.  The driver has just reported stopped after zero-stop, so retain
    // that observed evidence and never start homing merely to stop it again.
    const bool interrupted_before_start = interrupted();
    const bool deadline_before_start = deadline_expired();
    if (interrupted_before_start || deadline_before_start) {
        report.result = interrupted_before_start
            ? HomingExecutionResult::Interrupted
            : HomingExecutionResult::Timeout;
        report.shutdown_sequence_handled = true;
        HomingFailureStopReport& stop = report.failure_stop;
        stop.zero_stop_write = report.preparation_zero_stop_write;
        stop.zero_stop_written = true;
        stop.zero_stop_reused_from_preparation = true;
        stop.post_zero_status_query_attempted = true;
        stop.post_zero_status = ready_status;
        stop.driver_stop_confirmed = true;
        stop.stop_state_unknown = false;
        report.stop_state_unknown = false;
        report.hardware_intervention_required = false;
        return report;
    }

    const char start_command[] = "1 zero start\n";
    report.zero_start_write = write_line(
        reinterpret_cast<const uint8_t*>(start_command),
        static_cast<int>(sizeof(start_command) - 1));
    if (report.zero_start_write.IsPartial()) stream.MarkPartialLine();
    report.zero_start_written = report.zero_start_write.IsComplete();
    if (!report.zero_start_written) {
        // Do not append even zero-stop here.  A non-empty prefix might become
        // another valid line; a no-byte failure is also returned without a
        // speculative retry so the caller has an exact, bounded record.
        report.result = HomingExecutionResult::StartWriteFailure;
        report.shutdown_sequence_handled = true;
        report.stop_state_unknown = stream.IsTainted();
        report.hardware_intervention_required = stream.IsTainted();
        return report;
    }

    const auto fail_after_start = [&](HomingExecutionResult result) {
        report.result = result;
        report.cleanup_attempted = true;
        report.shutdown_sequence_handled = true;
        report.failure_stop = ExecuteBoundedHomingFailureStop(
            stream, status_timeout_ms, write_line, query_fn);
        report.stop_state_unknown = report.failure_stop.stop_state_unknown;
        report.hardware_intervention_required =
            report.failure_stop.hardware_intervention_required;
    };

    for (;;) {
        if (interrupted()) {
            fail_after_start(HomingExecutionResult::Interrupted);
            return report;
        }
        if (deadline_expired()) {
            fail_after_start(HomingExecutionResult::Timeout);
            return report;
        }

        report.completion_status = query_fn(status_timeout_ms);
        ++report.homing_status_queries;
        if (stream.IsTainted() ||
            !report.completion_status.observed_after_active_query ||
            !report.completion_status.IsValid()) {
            fail_after_start(HomingExecutionResult::StatusFailure);
            return report;
        }
        // The bounded query itself may consume the remaining overall budget or
        // overlap a signal.  Never accept even a stopped/zero/position-0 frame
        // as a normal completion after that event; zero-start was already sent,
        // so the documented zero-stop-first shutdown sequence owns the exit.
        const bool interrupted_after_query = interrupted();
        const bool deadline_after_query = deadline_expired();
        if (interrupted_after_query || deadline_after_query) {
            fail_after_start(interrupted_after_query
                ? HomingExecutionResult::Interrupted
                : HomingExecutionResult::Timeout);
            return report;
        }
        if (DriverReportsHomingComplete(report.completion_status)) {
            report.result = HomingExecutionResult::Complete;
            report.driver_home_confirmed = true;
            report.stop_state_unknown = false;
            return report;
        }

        wait_poll();
    }
}

enum class ReleaseFallbackAction : int8_t {
    NoneNeeded = 0,
    AttemptDeceleratedStop = 1,
    BlockedTainted = -1,
    PriorStopUnconfirmed = -2,
    SerialUnavailable = -3
};

struct ReleaseFallbackDecision {
    ReleaseFallbackAction action = ReleaseFallbackAction::NoneNeeded;
    bool attempt_stop = false;
    bool stop_state_unknown = false;
    bool hardware_intervention_required = false;
};

// Pure decision used by Release().  Closing the fd is never stop proof.  A
// tainted stream must not receive a fallback newline or retry.
inline ReleaseFallbackDecision DecideReleaseFallback(
        const AsciiStreamState& stream,
        bool serial_open,
        bool motion_may_be_active,
        bool complete_stop_written_since_motion,
        bool driver_stop_confirmed_since_motion) {
    ReleaseFallbackDecision decision;
    if (!motion_may_be_active || driver_stop_confirmed_since_motion) {
        return decision;
    }
    decision.stop_state_unknown = true;
    decision.hardware_intervention_required = true;
    if (!serial_open) {
        decision.action = ReleaseFallbackAction::SerialUnavailable;
        return decision;
    }
    if (stream.IsTainted()) {
        decision.action = ReleaseFallbackAction::BlockedTainted;
        return decision;
    }
    if (complete_stop_written_since_motion) {
        decision.action = ReleaseFallbackAction::PriorStopUnconfirmed;
        return decision;
    }
    decision.action = ReleaseFallbackAction::AttemptDeceleratedStop;
    decision.attempt_stop = true;
    return decision;
}

#endif // MOTOR_SAFETY_H
