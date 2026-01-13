/*
 * 电机控制模块
 *
 * 提供电机控制函数和 MSH 命令
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <string.h>

#include "motor_pwm.h"
#include "motor_gpio.h"
#include "motor_control.h"

/**
 * @brief 控制单个电机
 * @param motor_id 电机编号 (1 或 2)
 * @param direction 方向 (0=停止, 1=正转, 2=反转)
 * @param duty 占空比 (0.0 ~ 1.0)
 */
void motor_control(int motor_id, int direction, float duty)
{
    if (motor_id == 1)
    {
        switch (direction)
        {
        case 0:  /* 停止 */
            motor1_coast();
            motor1_set_duty(0.0f);
            break;
        case 1:  /* 正转 */
            motor1_forward();
            motor1_set_duty(duty);
            break;
        case 2:  /* 反转 */
            motor1_backward();
            motor1_set_duty(duty);
            break;
        default:
            rt_kprintf("[Motor1] Invalid direction: %d\n", direction);
            break;
        }
    }
    else if (motor_id == 2)
    {
        switch (direction)
        {
        case 0:  /* 停止 */
            motor2_coast();
            motor2_set_duty(0.0f);
            break;
        case 1:  /* 正转 */
            motor2_forward();
            motor2_set_duty(duty);
            break;
        case 2:  /* 反转 */
            motor2_backward();
            motor2_set_duty(duty);
            break;
        default:
            rt_kprintf("[Motor2] Invalid direction: %d\n", direction);
            break;
        }
    }
}

/**
 * @brief 解析单个电机的控制指令
 * @param cmd 指令字符串，格式: "1,0.5"
 * @param motor_id 电机编号 (1 或 2)
 */
static void parse_motor_cmd(const char *cmd, int motor_id)
{
    int direction;
    float duty;
    char *comma;
    char buf[32];

    if (cmd == RT_NULL || *cmd == '\0')
    {
        return;
    }

    /* 复制命令到缓冲区 */
    strncpy(buf, cmd, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    /* 查找逗号 */
    comma = strchr(buf, ',');
    if (comma == RT_NULL)
    {
        rt_kprintf("[Error] Invalid format, expected: direction,duty\n");
        return;
    }

    /* 分割字符串 */
    *comma = '\0';

    /* 解析方向和占空比 */
    direction = atoi(buf);
    duty = (float)atof(comma + 1);

    /* 执行控制 */
    motor_control(motor_id, direction, duty);

    rt_kprintf("[Motor%d] dir=%d, duty=%d%%\n", motor_id, direction, (int)(duty * 100));
}

/**
 * @brief MSH命令: 控制电机
 * @param argc 参数个数
 * @param argv 参数列表
 *
 * 用法: motor 1,0.5;2,0.1
 */
static int cmd_motor(int argc, char *argv[])
{
    char *cmd;
    char *semicolon;
    char buf[64];

    if (argc < 2)
    {
        rt_kprintf("Usage: motor <dir1,duty1>[;<dir2,duty2>]\n");
        rt_kprintf("  dir: 0=stop, 1=forward, 2=backward\n");
        rt_kprintf("  duty: 0.0 ~ 1.0\n");
        rt_kprintf("Example:\n");
        rt_kprintf("  motor 1,0.5;1,0.5   -- Both motors forward at 50%%\n");
        rt_kprintf("  motor 2,0.3;0,0     -- Motor1 backward 30%%, Motor2 stop\n");
        rt_kprintf("  motor 0,0;0,0       -- Stop both motors\n");
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

        /* 解析电机1指令 */
        parse_motor_cmd(buf, 1);

        /* 解析电机2指令 */
        parse_motor_cmd(semicolon + 1, 2);
    }
    else
    {
        /* 只有一个电机的指令，默认控制电机1 */
        parse_motor_cmd(buf, 1);
    }

    return 0;
}
MSH_CMD_EXPORT(cmd_motor, Control motors);

/**
 * @brief MSH命令: 停止所有电机
 */
static int cmd_motor_stop(int argc, char *argv[])
{
    motors_coast();
    motors_stop();
    rt_kprintf("All motors stopped.\n");
    return 0;
}
MSH_CMD_EXPORT(cmd_motor_stop, Stop all motors);
