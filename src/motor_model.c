#include "motor_model.h"

/**
 * @brief 计算电机1的前馈控制值 (PWM duty cycle)
 * 
 * 基于线性拟合模型: duty = k * speed + b
 * 正转和反转使用不同的参数
 */
double motor1_model(int dir, double speed)
{
    if (dir == 0) return 0.0;

    if (dir == 1)  /* forward */
    {
        static const double k = 0.2781;
        static const double b = 0.0233;
        return k * speed + b;
    }
    else           /* backward */
    {
        static const double k = 0.2549;
        static const double b = 0.0306;
        return k * speed + b;
    }
}

/**
 * @brief 计算电机2的前馈控制值 (PWM duty cycle)
 * 
 * 基于线性拟合模型: duty = k * speed + b
 * 正转和反转使用不同的参数
 */
double motor2_model(int dir, double speed)
{
    if (dir == 0) return 0.0;

    if (dir == 1)  /* forward */
    {
        static const double k = 0.2542;
        static const double b = 0.0612;
        return k * speed + b;
    }
    else           /* backward */
    {
        static const double k = 0.2829;
        static const double b = 0.0359;
        return k * speed + b;
    }
}
