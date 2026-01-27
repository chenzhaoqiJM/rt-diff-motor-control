/*
 * 双电机脉冲计数测试程序
 *
 * 使用两个 GPIO 引脚分别检测两个电机的脉冲，以 50Hz 频率打印计数结果
 * 参考 encoder.c 实现
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "motor_control.h"
#include "motor_model.h"
#include "motor_pwm.h"
#include "motor_gpio.h"
#include "common.h"

/* ================= 配置参数 ================= */

/* 打印线程参数 */
#define PULSE_PRINT_THREAD_STACK_SIZE   2048
#define PULSE_PRINT_THREAD_PRIORITY     10
#define PULSE_PRINT_THREAD_TIMESLICE    5

/* 采样周期 20ms = 50Hz */
#define PULSE_SAMPLE_PERIOD_MS          20

/* ================= 脉冲计数变量 ================= */

/* Motor1 脉冲计数器 */
static volatile rt_uint32_t pulse_count_motor1 = 0;
static rt_uint32_t pulse_last_count_motor1 = 0;
static volatile rt_bool_t pulse_has_rising_motor1 = RT_FALSE;

/* Motor2 脉冲计数器 */
static volatile rt_uint32_t pulse_count_motor2 = 0;
static rt_uint32_t pulse_last_count_motor2 = 0;
static volatile rt_bool_t pulse_has_rising_motor2 = RT_FALSE;

/* 初始化标志 */
static rt_bool_t pulse_initialized = RT_FALSE;

/* 打印线程句柄 */
static rt_thread_t pulse_print_thread = RT_NULL;

/* ================= 中断回调函数 ================= */

/**
 * @brief Motor1 GPIO 中断回调函数
 *        使用状态机消抖：上升沿 + 下降沿 = 一个完整脉冲
 */
static void pulse_gpio_irq_callback_motor1(void *args)
{
    (void)args;
    rt_uint8_t level = rt_pin_read(ENCODER_GPIO_MOTOR1_A);

    if (level) /* 高电平 = 上升沿 */
    {
        pulse_has_rising_motor1 = RT_TRUE;
    }
    else /* 低电平 = 下降沿 */
    {
        if (pulse_has_rising_motor1)
        {
            pulse_count_motor1++;
            pulse_has_rising_motor1 = RT_FALSE;
        }
    }
}

/**
 * @brief Motor2 GPIO 中断回调函数
 *        使用状态机消抖：上升沿 + 下降沿 = 一个完整脉冲
 */
static void pulse_gpio_irq_callback_motor2(void *args)
{
    (void)args;
    rt_uint8_t level = rt_pin_read(ENCODER_GPIO_MOTOR2_A);

    if (level) /* 高电平 = 上升沿 */
    {
        pulse_has_rising_motor2 = RT_TRUE;
    }
    else /* 低电平 = 下降沿 */
    {
        if (pulse_has_rising_motor2)
        {
            pulse_count_motor2++;
            pulse_has_rising_motor2 = RT_FALSE;
        }
    }
}

/* ================= 接口函数 ================= */

/**
 * @brief 初始化两个电机的脉冲计数 GPIO
 * @return RT_EOK 成功, 其他失败
 */
static rt_err_t pulse_gpio_init(void)
{
    if (pulse_initialized)
    {
        return RT_EOK;
    }

    /* ===== Motor1 GPIO 初始化 ===== */
    /* 配置 GPIO 为输入模式 (内部上拉) */
    rt_pin_mode(ENCODER_GPIO_MOTOR1_A, PIN_MODE_INPUT_PULLUP);

    /* 绑定中断，双边沿触发 */
    rt_err_t attach_ret1 = rt_pin_attach_irq(ENCODER_GPIO_MOTOR1_A, PIN_IRQ_MODE_RISING_FALLING,
                                             pulse_gpio_irq_callback_motor1, RT_NULL);
    rt_kprintf("[PulseTest] Motor1 rt_pin_attach_irq returned: %d\n", attach_ret1);

    rt_err_t enable_ret1 = rt_pin_irq_enable(ENCODER_GPIO_MOTOR1_A, PIN_IRQ_ENABLE);
    rt_kprintf("[PulseTest] Motor1 rt_pin_irq_enable returned: %d\n", enable_ret1);

    if (attach_ret1 != RT_EOK || enable_ret1 != RT_EOK)
    {
        rt_kprintf("[PulseTest] WARNING: Motor1 IRQ setup may have failed!\n");
    }

    /* ===== Motor2 GPIO 初始化 ===== */
    /* 配置 GPIO 为输入模式 (内部上拉) */
    rt_pin_mode(ENCODER_GPIO_MOTOR2_A, PIN_MODE_INPUT_PULLUP);

    /* 绑定中断，双边沿触发 */
    rt_err_t attach_ret2 = rt_pin_attach_irq(ENCODER_GPIO_MOTOR2_A, PIN_IRQ_MODE_RISING_FALLING,
                                             pulse_gpio_irq_callback_motor2, RT_NULL);
    rt_kprintf("[PulseTest] Motor2 rt_pin_attach_irq returned: %d\n", attach_ret2);

    rt_err_t enable_ret2 = rt_pin_irq_enable(ENCODER_GPIO_MOTOR2_A, PIN_IRQ_ENABLE);
    rt_kprintf("[PulseTest] Motor2 rt_pin_irq_enable returned: %d\n", enable_ret2);

    if (attach_ret2 != RT_EOK || enable_ret2 != RT_EOK)
    {
        rt_kprintf("[PulseTest] WARNING: Motor2 IRQ setup may have failed!\n");
    }

    /* 初始化计数器 */
    pulse_count_motor1 = 0;
    pulse_last_count_motor1 = 0;
    pulse_has_rising_motor1 = RT_FALSE;

    pulse_count_motor2 = 0;
    pulse_last_count_motor2 = 0;
    pulse_has_rising_motor2 = RT_FALSE;

    pulse_initialized = RT_TRUE;

    rt_kprintf("[PulseTest] Init OK (Motor1 GPIO=%d, Motor2 GPIO=%d)\n",
               ENCODER_GPIO_MOTOR1_A, ENCODER_GPIO_MOTOR2_A);

    return RT_EOK;
}

/**
 * @brief 获取 Motor1 脉冲增量
 * @return 自上次调用以来的脉冲增量
 */
static rt_uint32_t pulse_get_delta_motor1(void)
{
    rt_uint32_t current = pulse_count_motor1;
    rt_uint32_t delta = current - pulse_last_count_motor1;
    pulse_last_count_motor1 = current;

    return delta;
}

/**
 * @brief 获取 Motor2 脉冲增量
 * @return 自上次调用以来的脉冲增量
 */
static rt_uint32_t pulse_get_delta_motor2(void)
{
    rt_uint32_t current = pulse_count_motor2;
    rt_uint32_t delta = current - pulse_last_count_motor2;
    pulse_last_count_motor2 = current;

    return delta;
}

/**
 * @brief 获取 Motor1 总脉冲计数
 * @return 总脉冲计数
 */
static rt_uint32_t pulse_get_count_motor1(void)
{
    return pulse_count_motor1;
}

/**
 * @brief 获取 Motor2 总脉冲计数
 * @return 总脉冲计数
 */
static rt_uint32_t pulse_get_count_motor2(void)
{
    return pulse_count_motor2;
}

/**
 * @brief 重置所有脉冲计数
 */
static void pulse_reset(void)
{
    pulse_count_motor1 = 0;
    pulse_last_count_motor1 = 0;
    pulse_has_rising_motor1 = RT_FALSE;

    pulse_count_motor2 = 0;
    pulse_last_count_motor2 = 0;
    pulse_has_rising_motor2 = RT_FALSE;
}

/* ================= 打印线程 ================= */

/**
 * @brief 脉冲计数打印线程入口函数
 *        以 50Hz 频率读取并打印两个电机的脉冲计数
 */
static void pulse_print_thread_entry(void *parameter)
{
    (void)parameter;

    rt_tick_t last_tick = rt_tick_get();

    while (1)
    {
        /* 获取两个电机的脉冲增量和总计数 */
        rt_uint32_t delta1 = pulse_get_delta_motor1();
        rt_uint32_t total1 = pulse_get_count_motor1();
        rt_uint32_t delta2 = pulse_get_delta_motor2();
        rt_uint32_t total2 = pulse_get_count_motor2();

        /* 计算实际采样间隔 (毫秒) */
        rt_tick_t now = rt_tick_get();
        rt_uint32_t elapsed_ms = (now - last_tick) * 1000 / RT_TICK_PER_SECOND;
        last_tick = now;

        /* 计算线程采样频率 (Hz) = 1000 / elapsed_ms */
        float sample_freq = 0.0f;
        if (elapsed_ms > 0)
        {
            sample_freq = 1000.0f / elapsed_ms;
        }

        /* 打印结果 */
        rt_kprintf("[PulseTest] M1: delta=%u, total=%u | M2: delta=%u, total=%u | period=%ums, freq=%dHz\n",
                   delta1, total1, delta2, total2, elapsed_ms, (int)sample_freq);

        rt_thread_mdelay(PULSE_SAMPLE_PERIOD_MS);
    }
}

/**
 * @brief 启动脉冲计数打印线程
 * @return RT_EOK 成功, -RT_ERROR 失败
 */
static rt_err_t pulse_print_thread_start(void)
{
    pulse_print_thread = rt_thread_create("pulse_print",
                                          pulse_print_thread_entry,
                                          RT_NULL,
                                          PULSE_PRINT_THREAD_STACK_SIZE,
                                          PULSE_PRINT_THREAD_PRIORITY,
                                          PULSE_PRINT_THREAD_TIMESLICE);
    if (pulse_print_thread != RT_NULL)
    {
        rt_thread_startup(pulse_print_thread);
        rt_kprintf("[PulseTest] Print thread started (50Hz, period=%dms)\n",
                   PULSE_SAMPLE_PERIOD_MS);
        return RT_EOK;
    }
    else
    {
        rt_kprintf("[PulseTest] Failed to create print thread!\n");
        return -RT_ERROR;
    }
}

/**
 * @brief 停止脉冲计数打印线程
 * @return RT_EOK 成功
 */
static rt_err_t pulse_print_thread_stop(void)
{
    if (pulse_print_thread != RT_NULL)
    {
        rt_thread_delete(pulse_print_thread);
        pulse_print_thread = RT_NULL;
        rt_kprintf("[PulseTest] Print thread stopped\n");
    }
    return RT_EOK;
}

/* ================= MSH 命令 ================= */

/**
 * @brief MSH 命令: 启动脉冲计数测试
 *        用法: pulse_test_start
 */
static void pulse_test_start_cmd(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    if (pulse_gpio_init() != RT_EOK)
    {
        rt_kprintf("[PulseTest] GPIO init failed!\n");
        return;
    }

    pulse_print_thread_start();
}
MSH_CMD_EXPORT_ALIAS(pulse_test_start_cmd, pulse_test_start, Start single pin pulse counting test at 50Hz);

/**
 * @brief MSH 命令: 停止脉冲计数测试
 *        用法: pulse_test_stop
 */
static void pulse_test_stop_cmd(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    pulse_print_thread_stop();
}
MSH_CMD_EXPORT_ALIAS(pulse_test_stop_cmd, pulse_test_stop, Stop single pin pulse counting test);

/**
 * @brief MSH 命令: 读取当前脉冲计数信息
 *        用法: pulse_info
 */
static void pulse_info_cmd(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    rt_kprintf("[PulseTest] Motor1: GPIO=%d, total_count=%u\n",
               ENCODER_GPIO_MOTOR1_A, pulse_count_motor1);
    rt_kprintf("[PulseTest] Motor2: GPIO=%d, total_count=%u\n",
               ENCODER_GPIO_MOTOR2_A, pulse_count_motor2);
    rt_kprintf("[PulseTest] initialized=%d\n", pulse_initialized);
}
MSH_CMD_EXPORT_ALIAS(pulse_info_cmd, pulse_info, Read pulse counter info);

/**
 * @brief MSH 命令: 重置脉冲计数
 *        用法: pulse_reset
 */
static void pulse_reset_cmd(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    pulse_reset();
    rt_kprintf("[PulseTest] Counter reset\n");
}
MSH_CMD_EXPORT_ALIAS(pulse_reset_cmd, pulse_reset, Reset pulse counter);

/* ================= 主函数 ================= */

/**
 * @brief 主函数入口
 *        初始化脉冲计数并提示用户启动测试
 */
int main(void)
{
    rt_kprintf("==========================================\n");
    rt_kprintf("  Dual Motor Pulse Counter Test\n");
    rt_kprintf("==========================================\n\n");

    motors_gpio_init();
    motors_pwm_init();

    rt_kprintf("Motor1 Encoder GPIO: %d\n", ENCODER_GPIO_MOTOR1_A);
    rt_kprintf("Motor2 Encoder GPIO: %d\n", ENCODER_GPIO_MOTOR2_A);
    rt_kprintf("Sample frequency: 50Hz (period=%dms)\n\n", PULSE_SAMPLE_PERIOD_MS);

    rt_kprintf("Available MSH commands:\n");
    rt_kprintf("  pulse_test_start  -- Start pulse counting test\n");
    rt_kprintf("  pulse_test_stop   -- Stop pulse counting test\n");
    rt_kprintf("  pulse_info        -- Show pulse counter info\n");
    rt_kprintf("  pulse_reset       -- Reset pulse counter\n");
    rt_kprintf("  cmd_motor         -- Control motor (e.g. cmd_motor 1,0.5;1,0.5)\n\n");

    return 0;
}
