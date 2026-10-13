#ifndef VSMD_PARSER_H
#define VSMD_PARSER_H

#include <cstdint>
#include <cstring>
#include <cstdio>

// VSMD status parser: header, three five-byte encoded fields, then tail.
// Fields at 3..7, 8..12 and 13..17 decode speed, driver coordinate and flags.
// Structural validation does not validate BCC or establish mechanical arrival.


// 解析状态
enum VSMDParserState : uint8_t {
    VSMD_IDLE      = 0,   // 等待帧起始 0xFF
    VSMD_HEADER    = 1,   // 已收到 0xFF, 收集帧头
    VSMD_DATA      = 2,   // 帧头收集完毕, 收集数据体
    VSMD_FRAME_DONE = 3   // 帧尾 0xFE 确认, 一帧完成
};

// 帧信息
struct VSMD_Frame {
    bool   valid;          // 是否解析出一帧有效数据
    uint8_t raw[64];       // 原始帧数据
    int     raw_len;       // 实际帧长度

    // 解析出的速度值 (rps, 布局: 字节 3~7, Float32)
    float   speed_rps;

    // 解析出的位置值 (步, 布局: 字节 8~12, Int32)
    int32_t position_steps;

    // 解析出的状态位 (字节 13~17, UInt32; 诊断字段)
    uint32_t status_flags;
};

// ============================================================================
// VSMDParser 类
// ============================================================================

static const int VSMD_MAX_FRAME = 64;   // 最大帧长度
static const int VSMD_MIN_FRAME = 21;   // 最小帧长度 (含头尾)

class VSMDParser {
public:
    VSMDParser();

    // --- 喂入一个字节, 驱动状态机 ---
    // 返回值: true 表示一帧解析完成 (可通过 GetFrame 获取)
    bool Feed(uint8_t byte);

    // --- 批量喂入 ---
    // 返回成功解析的帧个数
    int FeedBulk(const uint8_t* data, int len);

    // --- 获取已解析帧 (仅当 Feed 返回 true 时有效) ---
    const VSMD_Frame& GetFrame() const { return frame_; }

    // --- 解码 5 字节 -> 32-bit 整数 (公开静态工具) ---
    static uint32_t Decode5ByteTo32Bit(const uint8_t* chunk);

    // --- 将 32-bit 整数通过 memcpy 安全转为 float (防 Strict Aliasing) ---
    static float Int32ToFloat(uint32_t raw);

    // --- 重置状态机 ---
    void Reset();

private:
    VSMDParserState state_;
    int             frame_idx_;
    uint8_t         frame_buf_[VSMD_MAX_FRAME];
    VSMD_Frame      frame_;        // 当前完成帧
};

#endif // VSMD_PARSER_H