#include "VSMDParser.h"

// VSMD status parser: header, three five-byte encoded fields, then tail.
// Fields at 3..7, 8..12 and 13..17 decode speed, driver coordinate and flags.
// Structural validation does not validate BCC or establish mechanical arrival.


VSMDParser::VSMDParser() {
    Reset();
}

void VSMDParser::Reset() {
    state_     = VSMD_IDLE;
    frame_idx_ = 0;
    memset(frame_buf_, 0, sizeof(frame_buf_));
    memset(&frame_, 0, sizeof(frame_));
}

bool VSMDParser::Feed(uint8_t byte) {
    switch (state_) {

    case VSMD_IDLE:
        if (byte == 0xFF) {
            state_ = VSMD_HEADER;
            frame_idx_ = 0;
            frame_buf_[frame_idx_++] = byte;
        }
        break;

    case VSMD_HEADER:
        // A short prefix can arrive just before a complete status frame. FF
        // is reserved as the frame marker, so resync here instead of folding
        // it into the three-byte header (which would turn a valid 21-byte
        // reply into a malformed 23-byte frame).
        if (byte == 0xFF) {
            frame_idx_ = 0;
            frame_buf_[frame_idx_++] = byte;
            break;
        }

        // 收集帧头 3 字节 (含 0xFF)
        frame_buf_[frame_idx_++] = byte;
        if (frame_idx_ >= 3) {
            state_ = VSMD_DATA;
        }
        break;

    case VSMD_DATA:
        if (frame_idx_ >= VSMD_MAX_FRAME) {
            // 超长帧 — 保护性复位
            Reset();
            break;
        }
        frame_buf_[frame_idx_++] = byte;

        if (byte == 0xFE && frame_idx_ >= VSMD_MIN_FRAME) {
            // 一帧完成
            frame_.valid   = true;
            frame_.raw_len = frame_idx_;
            memcpy(frame_.raw, frame_buf_, frame_idx_);

            // 数据段 15 字节全部解析 (当前解码布局, 21 字节帧保证段完整):
            //   [3..7] 当前速度 (Float32), [8..12] 绝对位置 (Int32), [13..17] 状态位 (UInt32)
            frame_.speed_rps =
                Int32ToFloat(Decode5ByteTo32Bit(&frame_buf_[3]));
            frame_.position_steps =
                static_cast<int32_t>(Decode5ByteTo32Bit(&frame_buf_[8]));
            frame_.status_flags =
                Decode5ByteTo32Bit(&frame_buf_[13]);

            state_ = VSMD_FRAME_DONE;
            return true;  // 通知调用方一帧完成
        }

        // 如果在 DATA 阶段遇到第二个 0xFF, 可能断帧 -> 重新开始
        if (byte == 0xFF && frame_idx_ > 1) {
            // 将当前字节作为新帧的开始
            state_ = VSMD_HEADER;
            frame_idx_ = 0;
            frame_buf_[frame_idx_++] = 0xFF;  // 上一个 0xFF 可能不对, 直接用当前
            // 但是我们已经把当前字节写入了, 清零
            // 更好的做法: 重置
            Reset();
            frame_buf_[frame_idx_++] = 0xFF;
            state_ = VSMD_HEADER;
        }
        break;

    case VSMD_FRAME_DONE:
        // 调用方应在 Feed 返回 true 后立即读取, 否则自动覆盖
        Reset();
        // 重新处理当前字节
        return Feed(byte);

    default:
        Reset();
        break;
    }

    return false;
}

int VSMDParser::FeedBulk(const uint8_t* data, int len) {
    int frames = 0;
    for (int i = 0; i < len; ++i) {
        if (Feed(data[i])) {
            ++frames;
        }
    }
    return frames;
}

// ============================================================================
// 5 字节 -> 32-bit 整数解码
// ============================================================================
uint32_t VSMDParser::Decode5ByteTo32Bit(const uint8_t* chunk) {
    return ((static_cast<uint32_t>(chunk[0] & 0x0F) << 28) |
            (static_cast<uint32_t>(chunk[1] & 0x7F) << 21) |
            (static_cast<uint32_t>(chunk[2] & 0x7F) << 14) |
            (static_cast<uint32_t>(chunk[3] & 0x7F) << 7)  |
            (static_cast<uint32_t>(chunk[4] & 0x7F)));
}

// ============================================================================
// uint32_t -> float 安全转换 (防 Strict Aliasing UB)
// ============================================================================
float VSMDParser::Int32ToFloat(uint32_t raw) {
    float result;
    memcpy(&result, &raw, sizeof(result));
    return result;
}
