#ifndef REAL_TIME_HELPER_H
#define REAL_TIME_HELPER_H

// ============================================================================
// RealTimeHelper: 实时提权与安全初始化
//
// 封装:
//   1. mlockall(MCL_CURRENT | MCL_FUTURE): 锁住所有内存页, 防止被 swap
//   2. pthread_setschedparam: 设置 SCHED_FIFO 优先级 99
//
// 失败时 perror 并 exit(EXIT_FAILURE)
//
// 跨平台:
//   - Linux: 完整实现
//   - Windows / 仿真: 桩函数, 仅打印警告
// ============================================================================

#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifdef __linux__
#include <sys/mman.h>
#include <pthread.h>
#include <sched.h>
#endif

// --- 执行实时提权 ---
// 仅在 Linux 上有效; 其他平台打印警告
inline void RealTime_Enable() {
#ifdef __linux__
    // 1. 锁内存
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        perror("FATAL: mlockall 失败");
        exit(EXIT_FAILURE);
    }

    // 2. 设置 SCHED_FIFO 优先级 99
    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = 99;

    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
        perror("FATAL: pthread_setschedparam (SCHED_FIFO, prio=99) 失败");
        exit(EXIT_FAILURE);
    }

    printf("[RealTime] 实时提权成功: mlockall + SCHED_FIFO prio=99\n");
#else
    (void)0;
    printf("[RealTime] 警告: 非 Linux 平台, 跳过实时提权 (仅仿真模式)\n");
#endif
}

#endif // REAL_TIME_HELPER_H
