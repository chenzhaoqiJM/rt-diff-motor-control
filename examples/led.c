/*
 * LED 闪烁示例
 *
 * 使用方法: cp led_blink.c ../main.c
 */

#include <rtthread.h>
#include <rtdevice.h>

#define LED_PIN     114  /* 根据实际硬件修改 */

static void led_blink_thread_entry(void *parameter)
{
    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);

    while (1)
    {
        rt_pin_write(LED_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
}

int main(void)
{
    rt_thread_t tid;

    rt_kprintf("LED Blink Example\n");

    tid = rt_thread_create("led", led_blink_thread_entry, RT_NULL,
                           4096, 20, 10);
    if (tid != RT_NULL)
        rt_thread_startup(tid);

    return 0;
}
