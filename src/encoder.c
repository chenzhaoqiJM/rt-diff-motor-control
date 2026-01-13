/*
 * 编码器模块 (防抖版)
 *
 * 通过 GPIO 中断计数霍尔编码器脉冲
 * 只使用 A 相，使用状态机消除信号抖动
 * 只有完整的 上升沿 -> 下降沿 才计为一个脉冲
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "common.h"
#include "encoder.h"

/* 编码器计数器 (无符号，只累加) */
static volatile rt_uint32_t encoder1_count = 0;
static volatile rt_uint32_t encoder2_count = 0;

/* 上一次读取时的计数值 (用于计算增量) */
static rt_uint32_t encoder1_last_count = 0;
static rt_uint32_t encoder2_last_count = 0;

/* 状态机：是否已检测到上升沿 (用于消抖) */
static volatile rt_bool_t encoder1_has_rising = RT_FALSE;
static volatile rt_bool_t encoder2_has_rising = RT_FALSE;

/* 初始化标志 */
static rt_bool_t encoder1_initialized = RT_FALSE;
static rt_bool_t encoder2_initialized = RT_FALSE;

/**
 * @brief 编码器1 A相中断回调
 *        使用状态机消抖：上升沿 + 下降沿 = 一个完整脉冲
 */
static void encoder1_a_irq_callback(void *args)
{
    (void)args;
    rt_uint8_t level = rt_pin_read(ENCODER_GPIO_MOTOR1_A);
    
    if (level) /* 高电平 = 上升沿 */
    {
        encoder1_has_rising = RT_TRUE;
    }
    else /* 低电平 = 下降沿 */
    {
        if (encoder1_has_rising)
        {
            encoder1_count++;
            encoder1_has_rising = RT_FALSE;
        }
    }
}

/**
 * @brief 编码器2 A相中断回调
 *        使用状态机消抖：上升沿 + 下降沿 = 一个完整脉冲
 */
static void encoder2_a_irq_callback(void *args)
{
    (void)args;
    rt_uint8_t level = rt_pin_read(ENCODER_GPIO_MOTOR2_A);
    
    if (level) /* 高电平 = 上升沿 */
    {
        encoder2_has_rising = RT_TRUE;
    }
    else /* 低电平 = 下降沿 */
    {
        if (encoder2_has_rising)
        {
            encoder2_count++;
            encoder2_has_rising = RT_FALSE;
        }
    }
}

/**
 * @brief 初始化编码器1
 */
rt_err_t encoder1_init(void)
{
    if (encoder1_initialized)
    {
        return RT_EOK;
    }

    /* 配置 A 相为输入模式 */
    rt_pin_mode(ENCODER_GPIO_MOTOR1_A, PIN_MODE_INPUT);

    /* 绑定 A 相中断，双边沿触发 */
    rt_err_t attach_ret = rt_pin_attach_irq(ENCODER_GPIO_MOTOR1_A, PIN_IRQ_MODE_RISING_FALLING,
                                            encoder1_a_irq_callback, RT_NULL);
    rt_kprintf("[Encoder1] rt_pin_attach_irq returned: %d\n", attach_ret);
    
    rt_err_t enable_ret = rt_pin_irq_enable(ENCODER_GPIO_MOTOR1_A, PIN_IRQ_ENABLE);
    rt_kprintf("[Encoder1] rt_pin_irq_enable returned: %d\n", enable_ret);
    
    if (attach_ret != RT_EOK || enable_ret != RT_EOK)
    {
        rt_kprintf("[Encoder1] WARNING: IRQ setup may have failed!\n");
    }

    encoder1_count = 0;
    encoder1_initialized = RT_TRUE;

    rt_kprintf("[Encoder1] Init OK (A=GPIO%d)\n", ENCODER_GPIO_MOTOR1_A);

    return RT_EOK;
}

/**
 * @brief 初始化编码器2
 */
rt_err_t encoder2_init(void)
{
    if (encoder2_initialized)
    {
        return RT_EOK;
    }

    /* 配置 A 相为输入模式 */
    rt_pin_mode(ENCODER_GPIO_MOTOR2_A, PIN_MODE_INPUT);

    /* 绑定 A 相中断，双边沿触发 */
    rt_pin_attach_irq(ENCODER_GPIO_MOTOR2_A, PIN_IRQ_MODE_RISING_FALLING,
                      encoder2_a_irq_callback, RT_NULL);
    rt_pin_irq_enable(ENCODER_GPIO_MOTOR2_A, PIN_IRQ_ENABLE);

    encoder2_count = 0;
    encoder2_initialized = RT_TRUE;

    rt_kprintf("[Encoder2] Init OK (A=GPIO%d)\n", ENCODER_GPIO_MOTOR2_A);

    return RT_EOK;
}

/**
 * @brief 初始化两个编码器
 */
rt_err_t encoders_init(void)
{
    encoder1_init();
    encoder2_init();
    return RT_EOK;
}

/**
 * @brief 获取编码器1脉冲计数
 */
rt_uint32_t encoder1_get_count(void)
{
    return encoder1_count;
}

/**
 * @brief 获取编码器2脉冲计数
 */
rt_uint32_t encoder2_get_count(void)
{
    return encoder2_count;
}

/**
 * @brief 重置编码器1计数
 */
void encoder1_reset(void)
{
    encoder1_count = 0;
}

/**
 * @brief 重置编码器2计数
 */
void encoder2_reset(void)
{
    encoder2_count = 0;
}

/**
 * @brief 重置两个编码器
 */
void encoders_reset(void)
{
    encoder1_reset();
    encoder2_reset();
}

/**
 * @brief 获取编码器1在一个周期内的脉冲增量
 *        调用后会更新 last_count
 * @return 自上次调用以来的脉冲增量
 */
rt_uint32_t encoder1_get_delta(void)
{
    rt_uint32_t current = encoder1_count;
    rt_uint32_t delta = current - encoder1_last_count;
    encoder1_last_count = current;
    return delta;
}

/**
 * @brief 获取编码器2在一个周期内的脉冲增量
 *        调用后会更新 last_count
 * @return 自上次调用以来的脉冲增量
 */
rt_uint32_t encoder2_get_delta(void)
{
    rt_uint32_t current = encoder2_count;
    rt_uint32_t delta = current - encoder2_last_count;
    encoder2_last_count = current;
    return delta;
}

/* ================= 编码器打印线程 ================= */

#define ENCODER_PRINT_THREAD_STACK_SIZE  4096
#define ENCODER_PRINT_THREAD_PRIORITY    12
#define ENCODER_PRINT_THREAD_TIMESLICE   5

static rt_thread_t encoder_print_thread = RT_NULL;

/**
 * @brief 编码器脉冲打印线程入口函数
 *        以 20Hz 频率打印两个编码器的脉冲计数
 */
static void encoder_print_thread_entry(void *parameter)
{
    (void)parameter;
    while (1)
    {
        rt_uint32_t count1 = encoder1_get_count();
        rt_uint32_t count2 = encoder2_get_count();
        rt_uint32_t delta1 = encoder1_get_delta();
        rt_uint32_t delta2 = encoder2_get_delta();

        /*
         * 转速计算 (使用整数运算，避免浮点):
         * RPM = (脉冲数 / PPR / 减速比) * 60 * 采样频率
         *     = delta * 60 * 20 / (PPR * 减速比)
         *     = delta * 1200 / (PPR * 减速比)
         * 
         * 为避免整数溢出和精度丢失，先乘后除
         */
        rt_uint32_t rpm1 = delta1 * 1200 / (MOTOR1_ENCODER_PPR * MOTOR1_REDUCTION_RATIO);
        rt_uint32_t rpm2 = delta2 * 1200 / (MOTOR2_ENCODER_PPR * MOTOR2_REDUCTION_RATIO);

        rt_kprintf("[Encoder] C1=%u C2=%u D1=%u D2=%u RPM1=%u RPM2=%u\n", 
                   count1, count2, delta1, delta2, rpm1, rpm2);

        /* 休眠 50ms, 实现 20Hz 频率 */
        rt_thread_mdelay(50);
    }
}

/**
 * @brief 启动编码器打印线程
 * @return RT_EOK 成功, -RT_ERROR 失败
 */
rt_err_t encoder_print_thread_start(void)
{
    encoder_print_thread = rt_thread_create("enc_print",
                                            encoder_print_thread_entry,
                                            RT_NULL,
                                            ENCODER_PRINT_THREAD_STACK_SIZE,
                                            ENCODER_PRINT_THREAD_PRIORITY,
                                            ENCODER_PRINT_THREAD_TIMESLICE);
    if (encoder_print_thread != RT_NULL)
    {
        rt_thread_startup(encoder_print_thread);
        rt_kprintf("[Encoder] Print thread started (20Hz)\n");
        return RT_EOK;
    }
    else
    {
        rt_kprintf("[Encoder] Failed to create print thread!\n");
        return -RT_ERROR;
    }
}

/* ================= 调试用 MSH 命令 ================= */

/**
 * @brief MSH 命令: 读取编码器 A 相 GPIO 电平 (用于调试)
 *        用法: enc_gpio
 */
static void enc_gpio_cmd(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    rt_uint8_t a1 = rt_pin_read(ENCODER_GPIO_MOTOR1_A);
    rt_uint8_t a2 = rt_pin_read(ENCODER_GPIO_MOTOR2_A);
    
    rt_kprintf("Encoder1: A(GPIO%d)=%d\n", ENCODER_GPIO_MOTOR1_A, a1);
    rt_kprintf("Encoder2: A(GPIO%d)=%d\n", ENCODER_GPIO_MOTOR2_A, a2);
}
MSH_CMD_EXPORT_ALIAS(enc_gpio_cmd, enc_gpio, Read encoder A-phase GPIO levels for debug);
