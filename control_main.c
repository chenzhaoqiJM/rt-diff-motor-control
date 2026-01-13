/*
 * 双电机控制主程序 (精简版)
 *
 * 主要功能已拆分到各个模块:
 * - motor_control.c: 电机控制函数和 MSH 命令
 * - encoder.c: 编码器计数和打印线程
 * - motor_pwm.c: PWM 驱动
 * - motor_gpio.c: GPIO 方向控制
 */

#include <rtthread.h>
#include <rtdevice.h>

#include "motor_pwm.h"
#include "motor_gpio.h"
#include "encoder.h"
#include "common.h"

/* ================= 底盘控制线程 ================= */

#define CHASSIS_CTRL_THREAD_STACK_SIZE  4096
#define CHASSIS_CTRL_THREAD_PRIORITY    10
#define CHASSIS_CTRL_THREAD_TIMESLICE   5

static rt_thread_t chassis_ctrl_thread = RT_NULL;

/**
 * @brief 底盘控制线程入口函数
 *        以 20Hz 频率获取编码器 delta，计算转速并打印
 */
static void chassis_ctrl_thread_entry(void *parameter)
{
    (void)parameter;
    while (1)
    {
        /* 从编码器模块获取共享的 delta 值 */
        rt_uint32_t delta1 = encoder_get_shared_delta1();
        rt_uint32_t delta2 = encoder_get_shared_delta2();

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

        rt_kprintf("[Chassis] D1=%u D2=%u RPM1=%u RPM2=%u\n", 
                   delta1, delta2, rpm1, rpm2);

        /* 休眠 33ms, 实现 30Hz 频率 */
        rt_thread_mdelay(33);
    }
}

/**
 * @brief 启动底盘控制线程
 * @return RT_EOK 成功, -RT_ERROR 失败
 */
static rt_err_t chassis_ctrl_thread_start(void)
{
    chassis_ctrl_thread = rt_thread_create("chassis",
                                           chassis_ctrl_thread_entry,
                                           RT_NULL,
                                           CHASSIS_CTRL_THREAD_STACK_SIZE,
                                           CHASSIS_CTRL_THREAD_PRIORITY,
                                           CHASSIS_CTRL_THREAD_TIMESLICE);
    if (chassis_ctrl_thread != RT_NULL)
    {
        rt_thread_startup(chassis_ctrl_thread);
        rt_kprintf("[Chassis] Control thread started (20Hz)\n");
        return RT_EOK;
    }
    else
    {
        rt_kprintf("[Chassis] Failed to create control thread!\n");
        return -RT_ERROR;
    }
}

int main(void)
{
    rt_kprintf("==========================================\n");
    rt_kprintf("  Dual Motor Control System\n");
    rt_kprintf("==========================================\n\n");

    /* 初始化电机 GPIO 和 PWM */
    motors_gpio_init();
    motors_pwm_init();

    /* 初始化编码器并启动读取线程 */
    encoders_init();
    encoder_print_thread_start();

    /* 启动底盘控制线程 (计算速度并打印) */
    chassis_ctrl_thread_start();

    rt_kprintf("\nMotor control ready. Use 'cmd_motor' command:\n");
    rt_kprintf("  cmd_motor 1,0.5;1,0.5   -- Both motors forward at 50%%\n");
    rt_kprintf("  cmd_motor 0,0;0,0       -- Stop both motors\n");
    rt_kprintf("  cmd_motor_stop          -- Emergency stop\n\n");

    return 0;
}
