/*
 * 单引脚脉冲计数测试程序
 *
 * 使用单个 GPIO 引脚检测脉冲，以 50Hz 频率打印计数结果
 * 参考 encoder.c 实现
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "motor_control.h"
#include "motor_model.h"
#include "motor_pwm.h"
#include "motor_gpio.h"

/* ================= 配置参数 ================= */

/* 测试用 GPIO 引脚，使用 ENCODER_GPIO_MOTOR1_A */
#define TEST_PULSE_GPIO     158

/* 打印线程参数 */
#define PULSE_PRINT_THREAD_STACK_SIZE   2048
#define PULSE_PRINT_THREAD_PRIORITY     10
#define PULSE_PRINT_THREAD_TIMESLICE    5

/* 采样周期 20ms = 50Hz */
#define PULSE_SAMPLE_PERIOD_MS          20

/* ================= 脉冲计数变量 ================= */

/* 脉冲计数器 */
static volatile rt_uint32_t pulse_count = 0;

/* 上一次读取时的计数值 */
static rt_uint32_t pulse_last_count = 0;

/* 状态机：是否已检测到上升沿 (用于消抖) */
static volatile rt_bool_t pulse_has_rising = RT_FALSE;

/* 初始化标志 */
static rt_bool_t pulse_initialized = RT_FALSE;

/* 打印线程句柄 */
static rt_thread_t pulse_print_thread = RT_NULL;

/* ================= 中断回调函数 ================= */

/**
 * @brief GPIO 中断回调函数
 *        使用状态机消抖：上升沿 + 下降沿 = 一个完整脉冲
 */
static void pulse_gpio_irq_callback(void *args)
{
    (void)args;
    rt_uint8_t level = rt_pin_read(TEST_PULSE_GPIO);
    
    if (level) /* 高电平 = 上升沿 */
    {
        pulse_has_rising = RT_TRUE;
    }
    else /* 低电平 = 下降沿 */
    {
        if (pulse_has_rising)
        {
            pulse_count++;
            pulse_has_rising = RT_FALSE;
        }
    }
}

/* ================= 接口函数 ================= */

/**
 * @brief 初始化脉冲计数 GPIO
 * @return RT_EOK 成功, 其他失败
 */
static rt_err_t pulse_gpio_init(void)
{
    if (pulse_initialized)
    {
        return RT_EOK;
    }

    /* 配置 GPIO 为输入模式 (内部上拉) */
    rt_pin_mode(TEST_PULSE_GPIO, PIN_MODE_INPUT_PULLUP);

    /* 绑定中断，双边沿触发 */
    rt_err_t attach_ret = rt_pin_attach_irq(TEST_PULSE_GPIO, PIN_IRQ_MODE_RISING_FALLING,
                                            pulse_gpio_irq_callback, RT_NULL);
    rt_kprintf("[PulseTest] rt_pin_attach_irq returned: %d\n", attach_ret);
    
    rt_err_t enable_ret = rt_pin_irq_enable(TEST_PULSE_GPIO, PIN_IRQ_ENABLE);
    rt_kprintf("[PulseTest] rt_pin_irq_enable returned: %d\n", enable_ret);
    
    if (attach_ret != RT_EOK || enable_ret != RT_EOK)
    {
        rt_kprintf("[PulseTest] WARNING: IRQ setup may have failed!\n");
        return -RT_ERROR;
    }

    pulse_count = 0;
    pulse_last_count = 0;
    pulse_has_rising = RT_FALSE;
    pulse_initialized = RT_TRUE;

    rt_kprintf("[PulseTest] Init OK (GPIO=%d)\n", TEST_PULSE_GPIO);

    return RT_EOK;
}

/**
 * @brief 获取脉冲增量
 * @return 自上次调用以来的脉冲增量
 */
static rt_uint32_t pulse_get_delta(void)
{
    rt_uint32_t current = pulse_count;
    rt_uint32_t delta = current - pulse_last_count;
    pulse_last_count = current;
    
    return delta;
}

/**
 * @brief 获取总脉冲计数
 * @return 总脉冲计数
 */
static rt_uint32_t pulse_get_count(void)
{
    return pulse_count;
}

/**
 * @brief 重置脉冲计数
 */
static void pulse_reset(void)
{
    pulse_count = 0;
    pulse_last_count = 0;
    pulse_has_rising = RT_FALSE;
}

/* ================= 打印线程 ================= */

/**
 * @brief 脉冲计数打印线程入口函数
 *        以 50Hz 频率读取并打印脉冲计数
 */
static void pulse_print_thread_entry(void *parameter)
{
    (void)parameter;
    
    rt_tick_t last_tick = rt_tick_get();
    
    while (1)
    {
        /* 获取脉冲增量和总计数 */
        rt_uint32_t delta = pulse_get_delta();
        rt_uint32_t total = pulse_get_count();
        
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
        rt_kprintf("[PulseTest] delta=%u, total=%u, period=%ums, sample_freq=%d Hz\n",
                   delta, total, elapsed_ms, (int)sample_freq);
        
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
    
    rt_kprintf("[PulseTest] GPIO=%d, total_count=%u, initialized=%d\n",
               TEST_PULSE_GPIO, pulse_count, pulse_initialized);
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
    rt_kprintf("  Single Pin Pulse Counter Test\n");
    rt_kprintf("==========================================\n\n");

    motors_gpio_init();
    motors_pwm_init();

    rt_kprintf("GPIO pin: %d\n", TEST_PULSE_GPIO);
    rt_kprintf("Sample frequency: 50Hz (period=%dms)\n\n", PULSE_SAMPLE_PERIOD_MS);

    rt_kprintf("Available MSH commands:\n");
    rt_kprintf("  pulse_test_start  -- Start pulse counting test\n");
    rt_kprintf("  pulse_test_stop   -- Stop pulse counting test\n");
    rt_kprintf("  pulse_info        -- Show pulse counter info\n");
    rt_kprintf("  pulse_reset       -- Reset pulse counter\n");
    rt_kprintf("  cmd_motor         -- Control motor (e.g. cmd_motor 1,0.5;1,0.5)\n\n");

    return 0;
}
