/*
 * LED 闪烁测试命令
 *
 * 使用方法: 在 MSH 终端输入 led_test <pin>
 * 功能: 指定引脚的 LED 闪烁 10 次
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>

#define BLINK_COUNT 10   /* 闪烁次数 */
#define BLINK_DELAY 200  /* 闪烁间隔(ms) */

/**
 * @brief LED 测试命令
 *        LED 闪烁 10 次
 * @param argc 参数个数
 * @param argv 参数列表 argv[1] = LED 引脚号
 * @return 0 成功, -1 参数错误
 */
static int led_test(int argc, char *argv[])
{
    int i;
    int led_pin;

    /* 检查参数 */
    if (argc < 2)
    {
        rt_kprintf("Usage: led_test <pin>\n");
        rt_kprintf("  pin: LED GPIO pin number\n");
        rt_kprintf("Example: led_test 114\n");
        return -1;
    }

    /* 解析引脚号 */
    led_pin = atoi(argv[1]);
    if (led_pin < 0)
    {
        rt_kprintf("Error: Invalid pin number '%s'\n", argv[1]);
        return -1;
    }

    rt_kprintf("LED Test: Blinking %d times on PIN %d\n", BLINK_COUNT, led_pin);

    /* 配置 LED 引脚为输出模式 */
    rt_pin_mode(led_pin, PIN_MODE_OUTPUT);

    /* 闪烁 10 次 */
    for (i = 0; i < BLINK_COUNT; i++)
    {
        rt_pin_write(led_pin, PIN_HIGH);
        rt_thread_mdelay(BLINK_DELAY);
        rt_pin_write(led_pin, PIN_LOW);
        rt_thread_mdelay(BLINK_DELAY);
        rt_kprintf("LED blink: %d/%d\n", i + 1, BLINK_COUNT);
    }

    rt_kprintf("LED Test: Done\n");

    return 0;
}

/* 导出为 MSH 命令 */
MSH_CMD_EXPORT(led_test, LED blink test - usage: led_test <pin>);
