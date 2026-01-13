/*
 * 编码器模块 (简化版)
 *
 * 通过 GPIO 中断计数霍尔编码器脉冲
 * 支持正交编码器，判断旋转方向
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "common.h"
#include "encoder.h"

/* 编码器计数器 (有符号，支持正反转) */
static volatile rt_int32_t encoder1_count = 0;
static volatile rt_int32_t encoder2_count = 0;

/* 上一次读取时的计数值 (用于计算增量) */
static rt_int32_t encoder1_last_count = 0;
static rt_int32_t encoder2_last_count = 0;

/* 上一次的A相电平状态 */
static volatile rt_uint8_t encoder1_last_a = 0;
static volatile rt_uint8_t encoder2_last_a = 0;

/* 初始化标志 */
static rt_bool_t encoder1_initialized = RT_FALSE;
static rt_bool_t encoder2_initialized = RT_FALSE;

/**
 * @brief 编码器1 A相中断回调
 *        根据B相电平判断方向
 */
static void encoder1_a_irq_callback(void *args)
{
    rt_uint8_t a_level = rt_pin_read(ENCODER_GPIO_MOTOR1_A);
    rt_uint8_t b_level = rt_pin_read(ENCODER_GPIO_MOTOR1_B);

    if (a_level != encoder1_last_a)
    {
        if (a_level)
        {
            /* A上升沿: B低=正转, B高=反转 */
            encoder1_count += (b_level == 0) ? 1 : -1;
        }
        else
        {
            /* A下降沿: B高=正转, B低=反转 */
            encoder1_count += (b_level == 1) ? 1 : -1;
        }
        encoder1_last_a = a_level;
    }
}

/**
 * @brief 编码器2 A相中断回调
 */
static void encoder2_a_irq_callback(void *args)
{
    rt_uint8_t a_level = rt_pin_read(ENCODER_GPIO_MOTOR2_A);
    rt_uint8_t b_level = rt_pin_read(ENCODER_GPIO_MOTOR2_B);

    if (a_level != encoder2_last_a)
    {
        if (a_level)
        {
            encoder2_count += (b_level == 0) ? 1 : -1;
        }
        else
        {
            encoder2_count += (b_level == 1) ? 1 : -1;
        }
        encoder2_last_a = a_level;
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

    /* 配置为输入模式 */
    rt_pin_mode(ENCODER_GPIO_MOTOR1_A, PIN_MODE_INPUT);
    rt_pin_mode(ENCODER_GPIO_MOTOR1_B, PIN_MODE_INPUT);

    /* 读取初始状态 */
    encoder1_last_a = rt_pin_read(ENCODER_GPIO_MOTOR1_A);

    /* 只绑定A相中断，双边沿触发 */
    rt_pin_attach_irq(ENCODER_GPIO_MOTOR1_A, PIN_IRQ_MODE_RISING_FALLING,
                      encoder1_a_irq_callback, RT_NULL);
    rt_pin_irq_enable(ENCODER_GPIO_MOTOR1_A, PIN_IRQ_ENABLE);

    encoder1_count = 0;
    encoder1_initialized = RT_TRUE;

    rt_kprintf("[Encoder1] Init OK (A=%d, B=%d)\n",
               ENCODER_GPIO_MOTOR1_A, ENCODER_GPIO_MOTOR1_B);

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

    /* 配置为输入模式 */
    rt_pin_mode(ENCODER_GPIO_MOTOR2_A, PIN_MODE_INPUT);
    rt_pin_mode(ENCODER_GPIO_MOTOR2_B, PIN_MODE_INPUT);

    /* 读取初始状态 */
    encoder2_last_a = rt_pin_read(ENCODER_GPIO_MOTOR2_A);

    /* 只绑定A相中断 */
    rt_pin_attach_irq(ENCODER_GPIO_MOTOR2_A, PIN_IRQ_MODE_RISING_FALLING,
                      encoder2_a_irq_callback, RT_NULL);
    rt_pin_irq_enable(ENCODER_GPIO_MOTOR2_A, PIN_IRQ_ENABLE);

    encoder2_count = 0;
    encoder2_initialized = RT_TRUE;

    rt_kprintf("[Encoder2] Init OK (A=%d, B=%d)\n",
               ENCODER_GPIO_MOTOR2_A, ENCODER_GPIO_MOTOR2_B);

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
rt_int32_t encoder1_get_count(void)
{
    return encoder1_count;
}

/**
 * @brief 获取编码器2脉冲计数
 */
rt_int32_t encoder2_get_count(void)
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
 * @return 自上次调用以来的脉冲增量 (正=正转, 负=反转)
 */
rt_int32_t encoder1_get_delta(void)
{
    rt_int32_t current = encoder1_count;
    rt_int32_t delta = current - encoder1_last_count;
    encoder1_last_count = current;
    return delta;
}

/**
 * @brief 获取编码器2在一个周期内的脉冲增量
 *        调用后会更新 last_count
 * @return 自上次调用以来的脉冲增量 (正=正转, 负=反转)
 */
rt_int32_t encoder2_get_delta(void)
{
    rt_int32_t current = encoder2_count;
    rt_int32_t delta = current - encoder2_last_count;
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
    while (1)
    {
        rt_int32_t count1 = encoder1_get_count();
        rt_int32_t count2 = encoder2_get_count();
        rt_int32_t delta1 = encoder1_get_delta();
        rt_int32_t delta2 = encoder2_get_delta();

        rt_kprintf("[Encoder] C1=%d C2=%d D1=%d D2=%d\n", count1, count2, delta1, delta2);

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
