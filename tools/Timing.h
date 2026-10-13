#ifndef PHASE_V4_TIMING_H
#define PHASE_V4_TIMING_H
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <chrono>
#endif
namespace timing {
// Some older MinGW steady_clock implementations quantize timestamps to 1ms.
// QPC supplies measured sub-microsecond ticks for host computation benchmarks.
inline double Seconds() {
#ifdef _WIN32
    LARGE_INTEGER count,frequency;
    QueryPerformanceCounter(&count); QueryPerformanceFrequency(&frequency);
    return static_cast<double>(count.QuadPart)/static_cast<double>(frequency.QuadPart);
#else
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
#endif
}
}
#endif
