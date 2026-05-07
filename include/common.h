#ifndef MYCOMMON_H
#define MYCOMMON_H

/* 定义全局变量 */

// 线程频率控制
#define DEFAULT_FEEDBACK_INTERVAL_MS 50 /* 默认反馈间隔 50ms (20Hz) */
#define ENCODER_INTERVAL_MS 50 /* 编码器读取线程周期 50ms (20Hz) */
#define CHASSIS_CTRL_INTERVAL_MS 50 /* 底盘控制线程周期 50ms (20Hz) */

// PWM
#define PWM_CHANNEL     1           /* PWM通道号 */
#define PWM_PERIOD      100000      /* 周期: 100us = 100000ns (10KHz) */

#define MOTOR_ENCODER_PPR     13
#define MOTOR_REDUCTION_RATIO 56 // 减速比

// Motor1 -------------------------------------------------------------------------------------------------

/* ================= GPIO 输出引脚定义, 控制电机正反转的 ================= */
#define GPIO_OUTPUT_IO_MOTOR1_0    125
#define GPIO_OUTPUT_IO_MOTOR1_1    127

/* ================= PWM ================= */

#define PWM_DEV_NAME_MOTOR_1    "rpwm9"     /* PWM设备名称 对应引脚 112 由设备树决定 */


/* ================= 编码器引脚参数设置 ================= */
#define ENCODER_GPIO_MOTOR1_A   158

// Motor2 --------------------------------------------------------------------------------------------------

/* ================= GPIO 输出引脚定义, 控制电机正反转的 ================= */
#define GPIO_OUTPUT_IO_MOTOR2_0    71
#define GPIO_OUTPUT_IO_MOTOR2_1    61

/* ================= PWM ================= */

#define PWM_DEV_NAME_MOTOR_2    "rpwm8"     /* PWM设备名称 对应引脚 111 由设备树决定 */

/* ================= 编码器引脚参数设置 ================= */
#define ENCODER_GPIO_MOTOR2_A   163


#endif  // MYCOMMON_H