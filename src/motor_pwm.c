/*
 * 电机 PWM 控制模块
 *
 * 提供两个电机的 PWM 初始化和占空比设置接口
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "common.h"
#include "motor_pwm.h"

/* PWM 设备句柄 */
static struct rt_device_pwm *pwm_dev_motor1 = RT_NULL;
static struct rt_device_pwm *pwm_dev_motor2 = RT_NULL;

/* 初始化标志 */
static rt_bool_t motor1_pwm_initialized = RT_FALSE;
static rt_bool_t motor2_pwm_initialized = RT_FALSE;

/**
 * @brief 初始化电机1的PWM
 * @return RT_EOK 成功, 其他值表示失败
 */
rt_err_t motor1_pwm_init(void)
{
    rt_err_t ret;

    if (motor1_pwm_initialized)
    {
        return RT_EOK;
    }

    /* 查找PWM设备 */
    pwm_dev_motor1 = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME_MOTOR_1);
    if (pwm_dev_motor1 == RT_NULL)
    {
        rt_kprintf("[Motor1] PWM device '%s' not found!\n", PWM_DEV_NAME_MOTOR_1);
        return -RT_ENOSYS;
    }
    rt_kprintf("[Motor1] PWM device '%s' found.\n", PWM_DEV_NAME_MOTOR_1);

    /* 设置初始PWM参数，占空比为0 */
    ret = rt_pwm_set(pwm_dev_motor1, PWM_CHANNEL, PWM_PERIOD, 0);
    if (ret != RT_EOK)
    {
        rt_kprintf("[Motor1] Failed to set PWM parameters! (err=%d)\n", ret);
        return ret;
    }

    /* 启用PWM */
    ret = rt_pwm_enable(pwm_dev_motor1, PWM_CHANNEL);
    if (ret != RT_EOK)
    {
        rt_kprintf("[Motor1] Failed to enable PWM! (err=%d)\n", ret);
        return ret;
    }

    motor1_pwm_initialized = RT_TRUE;
    rt_kprintf("[Motor1] PWM initialized successfully.\n");
    return RT_EOK;
}

/**
 * @brief 初始化电机2的PWM
 * @return RT_EOK 成功, 其他值表示失败
 */
rt_err_t motor2_pwm_init(void)
{
    rt_err_t ret;

    if (motor2_pwm_initialized)
    {
        return RT_EOK;
    }

    /* 查找PWM设备 */
    pwm_dev_motor2 = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME_MOTOR_2);
    if (pwm_dev_motor2 == RT_NULL)
    {
        rt_kprintf("[Motor2] PWM device '%s' not found!\n", PWM_DEV_NAME_MOTOR_2);
        return -RT_ENOSYS;
    }
    rt_kprintf("[Motor2] PWM device '%s' found.\n", PWM_DEV_NAME_MOTOR_2);

    /* 设置初始PWM参数，占空比为0 */
    ret = rt_pwm_set(pwm_dev_motor2, PWM_CHANNEL, PWM_PERIOD, 0);
    if (ret != RT_EOK)
    {
        rt_kprintf("[Motor2] Failed to set PWM parameters! (err=%d)\n", ret);
        return ret;
    }

    /* 启用PWM */
    ret = rt_pwm_enable(pwm_dev_motor2, PWM_CHANNEL);
    if (ret != RT_EOK)
    {
        rt_kprintf("[Motor2] Failed to enable PWM! (err=%d)\n", ret);
        return ret;
    }

    motor2_pwm_initialized = RT_TRUE;
    rt_kprintf("[Motor2] PWM initialized successfully.\n");
    return RT_EOK;
}

/**
 * @brief 初始化两个电机的PWM
 * @return RT_EOK 成功, 其他值表示失败
 */
rt_err_t motors_pwm_init(void)
{
    rt_err_t ret1, ret2;

    ret1 = motor1_pwm_init();
    ret2 = motor2_pwm_init();

    if (ret1 != RT_EOK || ret2 != RT_EOK)
    {
        return -RT_ERROR;
    }
    return RT_EOK;
}

/**
 * @brief 设置电机1的占空比
 * @param duty_percent 占空比百分比 (0-100)
 * @return RT_EOK 成功, 其他值表示失败
 */
rt_err_t motor1_set_duty(rt_uint8_t duty_percent)
{
    rt_uint32_t pulse;

    if (!motor1_pwm_initialized || pwm_dev_motor1 == RT_NULL)
    {
        rt_kprintf("[Motor1] PWM not initialized!\n");
        return -RT_ERROR;
    }

    /* 限制占空比范围 */
    if (duty_percent > 100)
    {
        duty_percent = 100;
    }

    /* 计算脉冲宽度 (ns) */
    pulse = (PWM_PERIOD * duty_percent) / 100;

    return rt_pwm_set(pwm_dev_motor1, PWM_CHANNEL, PWM_PERIOD, pulse);
}

/**
 * @brief 设置电机2的占空比
 * @param duty_percent 占空比百分比 (0-100)
 * @return RT_EOK 成功, 其他值表示失败
 */
rt_err_t motor2_set_duty(rt_uint8_t duty_percent)
{
    rt_uint32_t pulse;

    if (!motor2_pwm_initialized || pwm_dev_motor2 == RT_NULL)
    {
        rt_kprintf("[Motor2] PWM not initialized!\n");
        return -RT_ERROR;
    }

    /* 限制占空比范围 */
    if (duty_percent > 100)
    {
        duty_percent = 100;
    }

    /* 计算脉冲宽度 (ns) */
    pulse = (PWM_PERIOD * duty_percent) / 100;

    return rt_pwm_set(pwm_dev_motor2, PWM_CHANNEL, PWM_PERIOD, pulse);
}

/**
 * @brief 设置电机1的原始脉冲宽度
 * @param pulse_ns 脉冲宽度 (纳秒), 0 ~ PWM_PERIOD
 * @return RT_EOK 成功, 其他值表示失败
 */
rt_err_t motor1_set_pulse(rt_uint32_t pulse_ns)
{
    if (!motor1_pwm_initialized || pwm_dev_motor1 == RT_NULL)
    {
        return -RT_ERROR;
    }

    if (pulse_ns > PWM_PERIOD)
    {
        pulse_ns = PWM_PERIOD;
    }

    return rt_pwm_set(pwm_dev_motor1, PWM_CHANNEL, PWM_PERIOD, pulse_ns);
}

/**
 * @brief 设置电机2的原始脉冲宽度
 * @param pulse_ns 脉冲宽度 (纳秒), 0 ~ PWM_PERIOD
 * @return RT_EOK 成功, 其他值表示失败
 */
rt_err_t motor2_set_pulse(rt_uint32_t pulse_ns)
{
    if (!motor2_pwm_initialized || pwm_dev_motor2 == RT_NULL)
    {
        return -RT_ERROR;
    }

    if (pulse_ns > PWM_PERIOD)
    {
        pulse_ns = PWM_PERIOD;
    }

    return rt_pwm_set(pwm_dev_motor2, PWM_CHANNEL, PWM_PERIOD, pulse_ns);
}

/**
 * @brief 停止电机1 (占空比设为0)
 * @return RT_EOK 成功, 其他值表示失败
 */
rt_err_t motor1_stop(void)
{
    return motor1_set_duty(0);
}

/**
 * @brief 停止电机2 (占空比设为0)
 * @return RT_EOK 成功, 其他值表示失败
 */
rt_err_t motor2_stop(void)
{
    return motor2_set_duty(0);
}

/**
 * @brief 停止两个电机
 * @return RT_EOK 成功, 其他值表示失败
 */
rt_err_t motors_stop(void)
{
    motor1_stop();
    motor2_stop();
    return RT_EOK;
}
