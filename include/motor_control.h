/*
 * 电机控制模块 - 头文件
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 方向定义 */
#define MOTOR_DIR_STOP      0
#define MOTOR_DIR_FORWARD   1
#define MOTOR_DIR_BACKWARD  2

/**
 * @brief 控制单个电机
 * @param motor_id 电机编号 (1 或 2)
 * @param direction 方向 (0=停止, 1=正转, 2=反转)
 * @param duty 占空比 (0.0 ~ 1.0)
 */
void motor_control(int motor_id, int direction, float duty);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */
