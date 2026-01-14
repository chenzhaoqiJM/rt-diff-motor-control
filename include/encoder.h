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

/* 获取脉冲计数 (只累加，无方向) */
rt_uint32_t encoder1_get_count(void);
rt_uint32_t encoder2_get_count(void);

/* 获取周期内脉冲增量 (调用后更新last_count) */
rt_uint32_t encoder1_get_delta(void);
rt_uint32_t encoder2_get_delta(void);

/* 重置计数 */
void encoder1_reset(void);
void encoder2_reset(void);
void encoders_reset(void);

/* 启动编码器读取线程 (20Hz) */
rt_err_t encoder_print_thread_start(void);

/* 获取共享的速度值 (转/秒，供底盘控制线程读取) */
float encoder_get_shared_speed1(void);
float encoder_get_shared_speed2(void);

/* 获取共享的 delta 值 (用于调试) */
rt_uint32_t encoder_get_shared_delta1(void);
rt_uint32_t encoder_get_shared_delta2(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
