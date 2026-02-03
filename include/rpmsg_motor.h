/*
 * RPMsg 电机控制模块 - 头文件
 *
 * 实现大核(Linux)与小核(RCPU)之间的异步双向通信
 *
 * 协议:
 * - 速度指令: "dir1,speed1;dir2,speed2"
 * - 状态反馈: "dir1,speed1_mrs;dir2,speed2_mrs"
 */

#ifndef RPMSG_MOTOR_H
#define RPMSG_MOTOR_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 RPMsg 电机控制服务
 *        创建端点并启动状态反馈线程
 * @return RT_EOK 成功, 其他表示失败
 */
rt_err_t rpmsg_motor_init(void);

/**
 * @brief 设置状态反馈间隔
 * @param ms 反馈间隔 (毫秒), 最小 10ms
 */
void rpmsg_motor_set_feedback_interval(int ms);

/* ================= 外部接口声明 (由 control_main.c 实现) ================= */

/**
 * @brief 设置电机目标速度
 * @param dir1 电机1方向 (0=停止, 1=正转, 2=反转)
 * @param speed1 电机1目标转速 (转/秒)
 * @param dir2 电机2方向
 * @param speed2 电机2目标转速
 */
extern void chassis_set_target(int dir1, double speed1, int dir2,
                               double speed2);

/**
 * @brief 获取电机实际状态
 * @param[out] dir1 电机1实际方向
 * @param[out] speed1_mrs 电机1实际转速 (毫转/秒)
 * @param[out] dir2 电机2实际方向
 * @param[out] speed2_mrs 电机2实际转速 (毫转/秒)
 */
extern void chassis_get_status(int *dir1, int *speed1_mrs, int *dir2,
                               int *speed2_mrs);

#ifdef __cplusplus
}
#endif

#endif /* RPMSG_MOTOR_H */
