/*
 * 里程计模块 - 头文件
 *
 * 在小核上计算差速驱动机器人的里程计
 * 采用 ROS2 标准坐标系:
 *   - X+ = 前进方向
 *   - Y+ = 左侧
 *   - θ+ = 逆时针
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 里程计配置参数结构体
 */
typedef struct {
  float wheel_radius; /* 轮径 (m) */
  float wheel_base;   /* 轮距 (m) */
  float gear_ratio;   /* 减速比 */
  float encoder_ppr;  /* 编码器每转脉冲数 */
} OdometryConfig;

/**
 * @brief 里程计状态结构体
 */
typedef struct {
  float x;                  /* 位置 X (m) */
  float y;                  /* 位置 Y (m) */
  float theta;              /* 航向角 (rad) */
  float v;                  /* 线速度 (m/s) */
  float w;                  /* 角速度 (rad/s) */
  rt_uint32_t timestamp_ms; /* 时间戳 (ms) */
} OdometryState;

/**
 * @brief 初始化里程计模块
 *        使用默认配置参数
 */
void odometry_init(void);

/**
 * @brief 配置里程计参数
 * @param config 配置参数结构体指针
 */
void odometry_configure(const OdometryConfig *config);

/**
 * @brief 配置里程计参数 (通过单独参数)
 * @param wheel_radius 轮径 (m)
 * @param wheel_base 轮距 (m)
 * @param gear_ratio 减速比
 * @param encoder_ppr 编码器每转脉冲数
 */
void odometry_set_params(float wheel_radius, float wheel_base, float gear_ratio,
                         float encoder_ppr);

/**
 * @brief 更新里程计
 *        根据左右轮线速度和时间增量更新位姿
 * @param v_left 左轮线速度 (m/s)
 * @param v_right 右轮线速度 (m/s)
 * @param dt 时间增量 (s)
 */
void odometry_update(float v_left, float v_right, float dt);

/**
 * @brief 获取当前里程计状态
 * @param[out] state 状态输出结构体指针
 */
void odometry_get_state(OdometryState *state);

/**
 * @brief 重置里程计
 *        将位姿归零
 */
void odometry_reset(void);

/**
 * @brief 检查里程计是否已配置
 * @return RT_TRUE 已配置, RT_FALSE 未配置
 */
rt_bool_t odometry_is_configured(void);

/**
 * @brief 获取配置的轮径
 * @return 轮径 (m)
 */
float odometry_get_wheel_radius(void);

/**
 * @brief 获取配置的轮距
 * @return 轮距 (m)
 */
float odometry_get_wheel_base(void);

#ifdef __cplusplus
}
#endif

#endif /* ODOMETRY_H */
