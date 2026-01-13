/*
 * 编码器模块 - 头文件
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化 */
rt_err_t encoder1_init(void);
rt_err_t encoder2_init(void);
rt_err_t encoders_init(void);

/* 获取脉冲计数 (正=正转, 负=反转) */
rt_int32_t encoder1_get_count(void);
rt_int32_t encoder2_get_count(void);

/* 获取周期内脉冲增量 (调用后更新last_count, 考虑溢出) */
rt_int32_t encoder1_get_delta(void);
rt_int32_t encoder2_get_delta(void);

/* 重置计数 */
void encoder1_reset(void);
void encoder2_reset(void);
void encoders_reset(void);

/* 启动编码器打印线程 (20Hz) */
rt_err_t encoder_print_thread_start(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
