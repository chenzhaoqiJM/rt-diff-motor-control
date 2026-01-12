/*
 * 电机 PWM 控制模块 - 头文件
 */

#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化电机1的PWM
 * @return RT_EOK 成功, 其他值表示失败
 */
rt_err_t motor1_pwm_init(void);

/**
 * @brief 初始化电机2的PWM
 * @return RT_EOK 成功, 其他值表示失败
 */
rt_err_t motor2_pwm_init(void);

/**
 * @brief 初始化两个电机的PWM
 * @return RT_EOK 成功, 其他值表示失败
 */
rt_err_t motors_pwm_init(void);

/**
 * @brief 设置电机1的占空比
 * @param duty_percent 占空比百分比 (0-100)
 * @return RT_EOK 成功, 其他值表示失败
 */
rt_err_t motor1_set_duty(rt_uint8_t duty_percent);

/**
 * @brief 设置电机2的占空比
 * @param duty_percent 占空比百分比 (0-100)
 * @return RT_EOK 成功, 其他值表示失败
 */
rt_err_t motor2_set_duty(rt_uint8_t duty_percent);

/**
 * @brief 设置电机1的原始脉冲宽度
 * @param pulse_ns 脉冲宽度 (纳秒), 0 ~ PWM_PERIOD
 * @return RT_EOK 成功, 其他值表示失败
 */
rt_err_t motor1_set_pulse(rt_uint32_t pulse_ns);

/**
 * @brief 设置电机2的原始脉冲宽度
 * @param pulse_ns 脉冲宽度 (纳秒), 0 ~ PWM_PERIOD
 * @return RT_EOK 成功, 其他值表示失败
 */
rt_err_t motor2_set_pulse(rt_uint32_t pulse_ns);

/**
 * @brief 停止电机1 (占空比设为0)
 * @return RT_EOK 成功, 其他值表示失败
 */
rt_err_t motor1_stop(void);

/**
 * @brief 停止电机2 (占空比设为0)
 * @return RT_EOK 成功, 其他值表示失败
 */
rt_err_t motor2_stop(void);

/**
 * @brief 停止两个电机
 * @return RT_EOK 成功, 其他值表示失败
 */
rt_err_t motors_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PWM_H */
