#include "comm/VSMDParser.h"

#include <cstdio>
#include <cmath>

int main() {
    // The captured startup stream had an unexpected FF 01 prefix directly
    // before a valid 21-byte status reply: FF 01 FF 01 02 ... FE.
    const uint8_t delayed_prefix_and_status[] = {
        0xFF, 0x01,
        0xFF, 0x01, 0x02,
        0x00, 0x00, 0x00, 0x00, 0x00, // speed
        0x00, 0x00, 0x00, 0x00, 0x00, // position
        0x00, 0x00, 0x0C, 0x63, 0x1B, // status flags
        0x00, 0x77, 0xFE
    };
    static_assert(sizeof(delayed_prefix_and_status) == 23,
                  "test vector must match the captured 23-byte stream");

    VSMDParser parser;
    int completed_frames = 0;
    for (uint8_t byte : delayed_prefix_and_status) {
        if (parser.Feed(byte)) ++completed_frames;
    }

    const VSMD_Frame& frame = parser.GetFrame();
    if (completed_frames != 1 || !frame.valid || frame.raw_len != VSMD_MIN_FRAME ||
        frame.raw[0] != 0xFF || frame.raw[1] != 1 || frame.raw[2] != 2 ||
        frame.raw[frame.raw_len - 1] != 0xFE ||
        !std::isfinite(frame.speed_rps) || frame.speed_rps != 0.0f ||
        frame.position_steps != 0) {
        std::fprintf(stderr,
                     "parser failed to resync: frames=%d valid=%d raw_len=%d\n",
                     completed_frames, frame.valid ? 1 : 0, frame.raw_len);
        return 1;
    }

    std::puts("PASS: parser resynchronizes after delayed FF 01 prefix");
    return 0;
}
