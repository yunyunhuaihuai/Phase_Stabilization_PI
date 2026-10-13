#pragma once
enum class PIStatus { Ready, InvalidInput, GuardFault, Boundary, SendFault };
inline const char* PIStatusName(PIStatus s) {
    switch(s) {
    case PIStatus::Ready: return "ready";
    case PIStatus::InvalidInput: return "invalid_input";
    case PIStatus::GuardFault: return "guard_fault";
    case PIStatus::Boundary: return "boundary";
    case PIStatus::SendFault: return "send_fault";
    }
    return "unknown";
}
struct PIDiagnostics {
    double raw_error=0, integrator=0, requested_u=0, submitted_u=0;
    double physical_delta=0, lower=0, upper=0;
};
