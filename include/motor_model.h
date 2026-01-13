#ifndef MOTOR_MODEL_H
#define MOTOR_MODEL_H

/**
 * @brief 计算电机1的前馈控制值 (PWM duty cycle)
 * @param dir 方向: 0=停止, 1=正转, -1=反转
 * @param speed 目标速度 (单位根据实际情况定义)
 * @return PWM duty cycle (0.0 ~ 1.0)
 */
double motor1_model(int dir, double speed);

/**
 * @brief 计算电机2的前馈控制值 (PWM duty cycle)
 * @param dir 方向: 0=停止, 1=正转, -1=反转
 * @param speed 目标速度 (单位根据实际情况定义)
 * @return PWM duty cycle (0.0 ~ 1.0)
 */
double motor2_model(int dir, double speed);

#endif /* MOTOR_MODEL_H */
