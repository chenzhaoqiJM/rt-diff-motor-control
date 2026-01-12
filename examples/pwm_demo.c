/*
 * PWM 呼吸灯示例
 *
 * 使用 rpwm9 设备实现 LED 呼吸灯效果
 * 参考: bsp/spacemit/drivers/pwm/pwm-test.c 中的 pwm_test_0002
 */

#include <rtthread.h>
#include <rtdevice.h>

#define PWM_DEV_NAME    "rpwm9"     /* PWM设备名称 */
#define PWM_CHANNEL     1           /* PWM通道号 */
#define PWM_PERIOD      100000      /* 周期: 100us = 100000ns (10KHz) */
#define BREATH_STEP     1000        /* 占空比步进: 1us = 1000ns (1%) */
#define BREATH_DELAY    20          /* 每步延时: 20ms */

static void breathing_led_thread_entry(void *parameter)
{
    struct rt_device_pwm *pwm_dev = RT_NULL;
    rt_uint32_t pulse = 0;          /* 当前占空比 (ns) */
    rt_int8_t direction = 1;        /* 1: 变亮, -1: 变暗 */
    rt_err_t ret;

    /* 查找PWM设备 */
    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    if (pwm_dev == RT_NULL)
    {
        rt_kprintf("[ERROR] PWM device '%s' not found!\n", PWM_DEV_NAME);
        return;
    }
    rt_kprintf("[INFO] PWM device '%s' found.\n", PWM_DEV_NAME);

    /* 设置初始PWM参数 */
    ret = rt_pwm_set(pwm_dev, PWM_CHANNEL, PWM_PERIOD, pulse);
    if (ret != RT_EOK)
    {
        rt_kprintf("[ERROR] Failed to set PWM parameters! (err=%d)\n", ret);
        return;
    }

    /* 启用PWM */
    ret = rt_pwm_enable(pwm_dev, PWM_CHANNEL);
    if (ret != RT_EOK)
    {
        rt_kprintf("[ERROR] Failed to enable PWM! (err=%d)\n", ret);
        return;
    }
    rt_kprintf("[INFO] PWM enabled, breathing LED started...\n");

    /* 呼吸灯主循环 */
    while (1)
    {
        /* 更新占空比 */
        ret = rt_pwm_set(pwm_dev, PWM_CHANNEL, PWM_PERIOD, pulse);
        if (ret != RT_EOK)
        {
            rt_kprintf("[WARNING] Failed to update PWM duty (pulse=%d)\n", pulse);
        }

        /* 计算下一个占空比 */
        if (direction > 0)
        {
            /* 变亮 */
            pulse += BREATH_STEP;
            if (pulse >= PWM_PERIOD)
            {
                pulse = PWM_PERIOD;
                direction = -1;     /* 切换为变暗 */
            }
        }
        else
        {
            /* 变暗 */
            if (pulse >= BREATH_STEP)
            {
                pulse -= BREATH_STEP;
            }
            else
            {
                pulse = 0;
                direction = 1;      /* 切换为变亮 */
            }
        }

        rt_thread_mdelay(BREATH_DELAY);
    }
}

int main(void)
{
    rt_thread_t tid;

    rt_kprintf("==========================================\n");
    rt_kprintf("  PWM Breathing LED Example (rpwm9)\n");
    rt_kprintf("==========================================\n\n");

    tid = rt_thread_create("breath",
                           breathing_led_thread_entry,
                           RT_NULL,
                           4096,    /* 栈大小 */
                           20,      /* 优先级 */
                           10);     /* 时间片 */
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
        rt_kprintf("[INFO] Breathing LED thread started.\n");
    }
    else
    {
        rt_kprintf("[ERROR] Failed to create breathing LED thread!\n");
    }

    return 0;
}
