#ifndef MYCOMMON_H 
#define MYCOMMON_H

// GPIO8 和 GPIO9 是 boot 引脚，最好不要用来控制电机

// common

/* ================= UART ================= */

#define MY_RX_BUF_SIZE 1024 // 定义 UART 接收缓冲区大小

#define TXD_PIN (4)   // GPIO4
#define RXD_PIN (5)   // GPIO5

#define MY_UART_BAUD_RATE 115200

// PWM
#define PWM_CHANNEL     1           /* PWM通道号 */
#define PWM_PERIOD      100000      /* 周期: 100us = 100000ns (10KHz) */

// Motor1 -------------------------------------------------------------------------------------------------

/* ================= GPIO 输出引脚定义, 控制电机正反转的 ================= */
#define GPIO_OUTPUT_IO_MOTOR1_0    125
#define GPIO_OUTPUT_IO_MOTOR1_1    127

/* ================= PWM ================= */

#define PWM_DEV_NAME_MOTOR_1    "rpwm9"     /* PWM设备名称 对应引脚 112 由设备树决定 */


/* ================= 编码器引脚参数设置 ================= */
#define ENCODER_GPIO_MOTOR1_A   158

#define MOTOR1_ENCODER_PPR     11
#define MOTOR1_REDUCTION_RATIO 56 // 减速比

// Motor2 --------------------------------------------------------------------------------------------------

/* ================= GPIO 输出引脚定义, 控制电机正反转的 ================= */
#define GPIO_OUTPUT_IO_MOTOR2_0    71
#define GPIO_OUTPUT_IO_MOTOR2_1    61

/* ================= PWM ================= */

#define PWM_DEV_NAME_MOTOR_2    "rpwm8"     /* PWM设备名称 对应引脚 111 由设备树决定 */

/* ================= 编码器引脚参数设置 ================= */
#define ENCODER_GPIO_MOTOR2_A   163

#define MOTOR2_ENCODER_PPR     11

#define MOTOR2_REDUCTION_RATIO 56 // 减速比


#endif  // MYCOMMON_H