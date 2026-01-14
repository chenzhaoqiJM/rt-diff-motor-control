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
#include <stdlib.h>
#include <string.h>

#include "motor_pwm.h"
#include "motor_gpio.h"
#include "encoder.h"
#include "common.h"
#include "motor_control.h"
#include "motor_model.h"

/* ================= 目标速度控制 ================= */

/* 电机目标方向: 0=停止, 1=正转, 2=反转 */
static int motor1_target_dir = 0;
static int motor2_target_dir = 0;

/* 电机目标转速: 单位 转/秒 (r/s) */
static double motor1_target_speed = 0.0;
static double motor2_target_speed = 0.0;

/* 互斥锁保护目标值 */
static rt_mutex_t target_mutex = RT_NULL;

/* ================= 底盘控制线程 ================= */

#define CHASSIS_CTRL_THREAD_STACK_SIZE  4096
#define CHASSIS_CTRL_THREAD_PRIORITY    10
#define CHASSIS_CTRL_THREAD_TIMESLICE   5

static rt_thread_t chassis_ctrl_thread = RT_NULL;

/**
 * @brief 底盘控制线程入口函数
 *        以 30Hz 频率执行前馈控制 (后续可扩展为闭环控制)
 */
static void chassis_ctrl_thread_entry(void *parameter)
{
    (void)parameter;

    int dir1, dir2;
    double target_speed1, target_speed2;
    double duty1, duty2;

    while (1)
    {
        /* 从编码器模块获取共享的速度值 (转/秒) */
        float actual_speed1 = encoder_get_shared_speed1();
        float actual_speed2 = encoder_get_shared_speed2();
        
        /* 获取 delta 用于调试 */
        rt_uint32_t delta1 = encoder_get_shared_delta1();
        rt_uint32_t delta2 = encoder_get_shared_delta2();

        /* 获取目标值 (加锁保护) */
        rt_mutex_take(target_mutex, RT_WAITING_FOREVER);
        dir1 = motor1_target_dir;
        dir2 = motor2_target_dir;
        target_speed1 = motor1_target_speed;
        target_speed2 = motor2_target_speed;
        rt_mutex_release(target_mutex);

        /* 使用前馈模型计算 PWM 占空比 */
        duty1 = motor1_model(dir1, target_speed1);
        duty2 = motor2_model(dir2, target_speed2);

        /* TODO: 后续在这里添加 PID 闭环控制 */
        /* 
         * double error1 = target_speed1 - actual_speed1;
         * duty1 += pid_compute(&pid1, error1);
         */

        /* 限制占空比范围 */
        if (duty1 < 0.0) duty1 = 0.0;
        if (duty1 > 1.0) duty1 = 1.0;
        if (duty2 < 0.0) duty2 = 0.0;
        if (duty2 > 1.0) duty2 = 1.0;

        /* 执行电机控制 */
        motor_control(1, dir1, (float)duty1);
        motor_control(2, dir2, (float)duty2);

        /* 调试打印 (速度单位: 转/秒, mr/s = 毫转/秒) */
        rt_kprintf("[Chassis] D1=%u D2=%u S1=%d S2=%d mr/s | T:%d,%d mr/s D:%d%%,%d%%\n", 
                   delta1, delta2, 
                   (int)(actual_speed1*1000), (int)(actual_speed2*1000),
                   (int)(target_speed1*1000), (int)(target_speed2*1000), 
                   (int)(duty1*100), (int)(duty2*100));

        /* 休眠 33ms, 实现 30Hz 控制频率 */
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

    /* 创建目标值互斥锁 */
    target_mutex = rt_mutex_create("tgt_mtx", RT_IPC_FLAG_PRIO);
    if (target_mutex == RT_NULL)
    {
        rt_kprintf("[Error] Failed to create target mutex!\n");
        return -1;
    }

    /* 初始化电机 GPIO 和 PWM */
    motors_gpio_init();
    motors_pwm_init();

    /* 初始化编码器并启动读取线程 */
    encoders_init();
    encoder_print_thread_start();

    /* 启动底盘控制线程 (前馈控制) */
    chassis_ctrl_thread_start();

    rt_kprintf("\nMotor control ready. Use 'cmd_speed' command:\n");
    rt_kprintf("  cmd_speed 1,2.0;1,2.0   -- Both motors forward at 2.0 r/s\n");
    rt_kprintf("  cmd_speed 0,0;0,0       -- Stop both motors\n");
    rt_kprintf("  cmd_motor_stop          -- Emergency stop\n\n");

    return 0;
}

/* ================= MSH 速度控制命令 ================= */

/**
 * @brief 解析单个电机的速度指令
 * @param cmd 指令字符串，格式: "1,2.0" (方向,转速)
 * @param motor_id 电机编号 (1 或 2)
 * @param[out] dir 输出方向
 * @param[out] speed 输出速度
 * @return RT_EOK 成功, -RT_ERROR 失败
 */
static rt_err_t parse_speed_cmd(const char *cmd, int *dir, double *speed)
{
    char *comma;
    char buf[32];

    if (cmd == RT_NULL || *cmd == '\0')
    {
        return -RT_ERROR;
    }

    /* 复制命令到缓冲区 */
    strncpy(buf, cmd, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    /* 查找逗号 */
    comma = strchr(buf, ',');
    if (comma == RT_NULL)
    {
        rt_kprintf("[Error] Invalid format, expected: direction,speed\n");
        return -RT_ERROR;
    }

    /* 分割字符串 */
    *comma = '\0';

    /* 解析方向和速度 */
    *dir = atoi(buf);
    *speed = atof(comma + 1);

    return RT_EOK;
}

/**
 * @brief MSH命令: 设置电机目标速度
 * @param argc 参数个数
 * @param argv 参数列表
 *
 * 用法: cmd_speed 1,2.0;2,2.0
 * 格式: 方向,转速;方向,转速
 *   方向: 0=停止, 1=正转, 2=反转
 *   转速: 单位为 转/秒 (r/s)
 */
static int cmd_speed(int argc, char *argv[])
{
    char *cmd;
    char *semicolon;
    char buf[64];
    int dir1 = 0, dir2 = 0;
    double speed1 = 0.0, speed2 = 0.0;

    if (argc < 2)
    {
        rt_kprintf("Usage: cmd_speed <dir1,speed1>[;<dir2,speed2>]\n");
        rt_kprintf("  dir: 0=stop, 1=forward, 2=backward\n");
        rt_kprintf("  speed: rotation speed in r/s (revolutions per second)\n");
        rt_kprintf("Example:\n");
        rt_kprintf("  cmd_speed 1,2.0;1,2.0   -- Both motors forward at 2.0 r/s\n");
        rt_kprintf("  cmd_speed 2,1.5;0,0     -- Motor1 backward 1.5 r/s, Motor2 stop\n");
        rt_kprintf("  cmd_speed 0,0;0,0       -- Stop both motors\n");
        return -1;
    }

    cmd = argv[1];

    /* 复制命令到缓冲区 */
    strncpy(buf, cmd, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    /* 查找分号，分隔两个电机的指令 */
    semicolon = strchr(buf, ';');
    if (semicolon != RT_NULL)
    {
        /* 分割字符串 */
        *semicolon = '\0';

        /* 解析电机1和电机2指令 */
        if (parse_speed_cmd(buf, &dir1, &speed1) != RT_EOK)
            return -1;
        if (parse_speed_cmd(semicolon + 1, &dir2, &speed2) != RT_EOK)
            return -1;
    }
    else
    {
        /* 只有一个电机的指令，默认控制电机1 */
        if (parse_speed_cmd(buf, &dir1, &speed1) != RT_EOK)
            return -1;
    }

    /* 设置目标值 (加锁保护) */
    rt_mutex_take(target_mutex, RT_WAITING_FOREVER);
    motor1_target_dir = dir1;
    motor2_target_dir = dir2;
    motor1_target_speed = speed1;
    motor2_target_speed = speed2;
    rt_mutex_release(target_mutex);

    rt_kprintf("[cmd_speed] Motor1: dir=%d, speed=%.2f r/s\n", dir1, speed1);
    rt_kprintf("[cmd_speed] Motor2: dir=%d, speed=%.2f r/s\n", dir2, speed2);

    return 0;
}
MSH_CMD_EXPORT(cmd_speed, Set motor target speed in r/s);

/**
 * @brief MSH命令: 紧急停止所有电机
 */
static int cmd_chassis_stop(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    /* 设置目标值为0 */
    rt_mutex_take(target_mutex, RT_WAITING_FOREVER);
    motor1_target_dir = 0;
    motor2_target_dir = 0;
    motor1_target_speed = 0.0;
    motor2_target_speed = 0.0;
    rt_mutex_release(target_mutex);

    rt_kprintf("[cmd_chassis_stop] All motors stopped.\n");
    return 0;
}
MSH_CMD_EXPORT(cmd_chassis_stop, Emergency stop all motors);
