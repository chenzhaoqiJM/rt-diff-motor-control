/*
 * 电机 GPIO 控制模块 - 头文件
 *
 * 用于控制电机正转、反转、刹车和滑行
 */

#ifndef MOTOR_GPIO_H
#define MOTOR_GPIO_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 初始化函数 ==================== */

/**
 * @brief 初始化电机1的GPIO引脚
 */
void motor1_gpio_init(void);

/**
 * @brief 初始化电机2的GPIO引脚
 */
void motor2_gpio_init(void);

/**
 * @brief 初始化两个电机的GPIO引脚
 */
void motors_gpio_init(void);

/* ==================== 电机1 GPIO 控制 ==================== */

/**
 * @brief 设置电机1的GPIO_0引脚电平
 * @param level 0: LOW, 非0: HIGH
 */
void motor1_set_pin0(rt_uint8_t level);

/**
 * @brief 设置电机1的GPIO_1引脚电平
 * @param level 0: LOW, 非0: HIGH
 */
void motor1_set_pin1(rt_uint8_t level);

/**
 * @brief 设置电机1的两个引脚电平
 * @param pin0_level GPIO_0引脚电平 (0: LOW, 非0: HIGH)
 * @param pin1_level GPIO_1引脚电平 (0: LOW, 非0: HIGH)
 */
void motor1_set_pins(rt_uint8_t pin0_level, rt_uint8_t pin1_level);

/**
 * @brief 电机1正转 (pin0=HIGH, pin1=LOW)
 */
void motor1_forward(void);

/**
 * @brief 电机1反转 (pin0=LOW, pin1=HIGH)
 */
void motor1_backward(void);

/**
 * @brief 电机1刹车 (pin0=HIGH, pin1=HIGH)
 */
void motor1_brake(void);

/**
 * @brief 电机1滑行/停止 (pin0=LOW, pin1=LOW)
 */
void motor1_coast(void);

/* ==================== 电机2 GPIO 控制 ==================== */

/**
 * @brief 设置电机2的GPIO_0引脚电平
 * @param level 0: LOW, 非0: HIGH
 */
void motor2_set_pin0(rt_uint8_t level);

/**
 * @brief 设置电机2的GPIO_1引脚电平
 * @param level 0: LOW, 非0: HIGH
 */
void motor2_set_pin1(rt_uint8_t level);

/**
 * @brief 设置电机2的两个引脚电平
 * @param pin0_level GPIO_0引脚电平 (0: LOW, 非0: HIGH)
 * @param pin1_level GPIO_1引脚电平 (0: LOW, 非0: HIGH)
 */
void motor2_set_pins(rt_uint8_t pin0_level, rt_uint8_t pin1_level);

/**
 * @brief 电机2正转 (pin0=HIGH, pin1=LOW)
 */
void motor2_forward(void);

/**
 * @brief 电机2反转 (pin0=LOW, pin1=HIGH)
 */
void motor2_backward(void);

/**
 * @brief 电机2刹车 (pin0=HIGH, pin1=HIGH)
 */
void motor2_brake(void);

/**
 * @brief 电机2滑行/停止 (pin0=LOW, pin1=LOW)
 */
void motor2_coast(void);

/* ==================== 两个电机同时控制 ==================== */

/**
 * @brief 两个电机同时正转
 */
void motors_forward(void);

/**
 * @brief 两个电机同时反转
 */
void motors_backward(void);

/**
 * @brief 两个电机同时刹车
 */
void motors_brake(void);

/**
 * @brief 两个电机同时滑行/停止
 */
void motors_coast(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_GPIO_H */
