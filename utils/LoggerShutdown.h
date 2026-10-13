#ifndef LOGGER_SHUTDOWN_H
#define LOGGER_SHUTDOWN_H

#include <cstdio>

// Called only after Close has drained the writer and final stream checks fail.
inline void ReportLoggerShutdownFailure(bool& failed, FILE* errors) {
    failed = true;
    std::fprintf(errors, "FATAL: logging shutdown failed during final write, flush, or close\n");
}

#endif
