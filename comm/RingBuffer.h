#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <cstdint>
#include <cstring>
#include <cstdio>

// ============================================================================
// RingBuffer<T, SIZE>: 定长环形缓冲区 (静态分配)
//
// 特点:
//   - 完全静态: 无动态内存, 无 STL
//   - 线程不安全 (主循环单线程使用)
//   - 覆盖策略: 满则覆盖最旧数据 (保持最新 N 字节)
//   - 用于串口异步收数 -> 状态机解析
// ============================================================================

template <typename T, int SIZE>
class RingBuffer {
public:
    RingBuffer() : head_(0), tail_(0), full_(false) {
        // 全部清零
        clear();
    }

    // --- 写入一个元素 ---
    void push(T val) {
        buf_[head_] = val;
        if (full_) {
            // 覆盖最旧 -> 推进 tail
            tail_ = (tail_ + 1) % SIZE;
        }
        head_ = (head_ + 1) % SIZE;
        full_ = (head_ == tail_);
    }

    // --- 批量写入 ---
    void push_bulk(const T* src, int count) {
        for (int i = 0; i < count; ++i) {
            push(src[i]);
        }
    }

    // --- 读取一个元素 (不弹出) ---
    // index: 从 oldest (0) 到 newest (size-1)
    T at(int index) const {
        if (index < 0 || index >= size()) return T(0);
        return buf_[(tail_ + index) % SIZE];
    }

    // --- 弹出最旧元素 ---
    T pop() {
        if (empty()) return T(0);
        T val = buf_[tail_];
        tail_ = (tail_ + 1) % SIZE;
        full_ = false;
        return val;
    }

    // --- 当前元素个数 ---
    int size() const {
        if (full_) return SIZE;
        return (head_ - tail_ + SIZE) % SIZE;
    }

    // --- 容量 ---
    int capacity() const { return SIZE; }

    // --- 是否为空 ---
    bool empty() const { return (!full_ && head_ == tail_); }

    // --- 是否已满 ---
    bool is_full() const { return full_; }

    // --- 清空 ---
    void clear() {
        head_ = tail_ = 0;
        full_ = false;
        // 全零填充确保安全
        // 使用 memset 而不是 for 循环
        memset(buf_, 0, sizeof(buf_));
    }

    // --- 搜索字节 (返回相对于 oldest 的偏移, -1 未找到) ---
    int find(T val, int start_offset = 0) const {
        int sz = size();
        for (int i = start_offset; i < sz; ++i) {
            if (at(i) == val) return i;
        }
        return -1;
    }

    // --- 丢弃前 N 个字节 ---
    void discard(int n) {
        if (n <= 0) return;
        int sz = size();
        if (n >= sz) {
            clear();
            return;
        }
        tail_ = (tail_ + n) % SIZE;
        full_ = false;
    }

private:
    T     buf_[SIZE];
    int   head_;   // 下次写入位置
    int   tail_;   // 最旧元素位置
    bool  full_;   // 满标志
};

#endif // RING_BUFFER_H