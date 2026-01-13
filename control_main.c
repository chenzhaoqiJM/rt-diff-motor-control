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

/**
 * @brief 主函数
 */
int main(void)
{
    rt_kprintf("==========================================\n");
    rt_kprintf("  Dual Motor Control System\n");
    rt_kprintf("==========================================\n\n");

    /* 初始化电机 GPIO 和 PWM */
    motors_gpio_init();
    motors_pwm_init();

    /* 初始化编码器并启动打印线程 */
    encoders_init();
    // encoder_print_thread_start();

    rt_kprintf("\nMotor control ready. Use 'cmd_motor' command:\n");
    rt_kprintf("  cmd_motor 1,0.5;1,0.5   -- Both motors forward at 50%%\n");
    rt_kprintf("  cmd_motor 0,0;0,0       -- Stop both motors\n");
    rt_kprintf("  cmd_motor_stop          -- Emergency stop\n\n");

    return 0;
}
