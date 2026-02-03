/*
 * 里程计模块 - 实现
 *
 * 在小核上计算差速驱动机器人的里程计
 * 采用 ROS2 标准坐标系:
 *   - X+ = 前进方向
 *   - Y+ = 左侧
 *   - θ+ = 逆时针
 *
 * 运动学模型:
 *   v = (v_left + v_right) / 2        (线速度)
 *   w = (v_right - v_left) / wheel_base  (角速度)
 *
 * 积分更新:
 *   x += v * cos(theta) * dt
 *   y += v * sin(theta) * dt
 *   theta += w * dt
 */

#include <math.h>
#include <rtthread.h>
#include <string.h>

#include "common.h"
#include "odometry.h"

/* ================= 模块状态 ================= */

/* 里程计配置 */
static OdometryConfig odom_config = {
    .wheel_radius = 0.0f,
    .wheel_base = 0.0f,
    .gear_ratio = 0.0f,
    .encoder_ppr = 0.0f,
};

/* 里程计状态 */
static OdometryState odom_state = {
    .x = 0.0f,
    .y = 0.0f,
    .theta = 0.0f,
    .v = 0.0f,
    .w = 0.0f,
    .timestamp_ms = 0,
};

/* 配置标志 */
static rt_bool_t odom_configured = RT_FALSE;

/* 互斥锁保护状态 */
static rt_mutex_t odom_mutex = RT_NULL;

/* ================= 内部函数 ================= */

/**
 * @brief 将角度归一化到 [-PI, PI] 范围
 */
static float normalize_angle(float angle) {
  while (angle > M_PI) {
    angle -= 2.0f * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0f * M_PI;
  }
  return angle;
}

/* ================= 公共接口 ================= */

/**
 * @brief 初始化里程计模块
 */
void odometry_init(void) {
  /* 创建互斥锁 */
  if (odom_mutex == RT_NULL) {
    odom_mutex = rt_mutex_create("odom_mtx", RT_IPC_FLAG_PRIO);
    if (odom_mutex == RT_NULL) {
      rt_kprintf("[Odometry] Failed to create mutex!\n");
      return;
    }
  }

  /* 使用默认配置 (从 common.h) */
  odom_config.wheel_radius = 0.0f; /* 需要从大核配置 */
  odom_config.wheel_base = 0.0f;
  odom_config.gear_ratio = (float)MOTOR1_REDUCTION_RATIO;
  odom_config.encoder_ppr = (float)MOTOR1_ENCODER_PPR;

  /* 重置状态 */
  odometry_reset();

  odom_configured = RT_FALSE;
  rt_kprintf("[Odometry] Initialized, waiting for configuration...\n");
}

/**
 * @brief 配置里程计参数
 */
void odometry_configure(const OdometryConfig *config) {
  if (config == RT_NULL) {
    return;
  }

  rt_mutex_take(odom_mutex, RT_WAITING_FOREVER);
  odom_config = *config;
  odom_configured = RT_TRUE;
  rt_mutex_release(odom_mutex);

  rt_kprintf("[Odometry] Configured: R=%.4f, L=%.4f, G=%.1f, PPR=%.0f\n",
             odom_config.wheel_radius, odom_config.wheel_base,
             odom_config.gear_ratio, odom_config.encoder_ppr);
}

/**
 * @brief 配置里程计参数 (通过单独参数)
 */
void odometry_set_params(float wheel_radius, float wheel_base, float gear_ratio,
                         float encoder_ppr) {
  OdometryConfig config = {
      .wheel_radius = wheel_radius,
      .wheel_base = wheel_base,
      .gear_ratio = gear_ratio,
      .encoder_ppr = encoder_ppr,
  };
  odometry_configure(&config);
}

/**
 * @brief 更新里程计
 *        根据左右轮线速度和时间增量更新位姿
 */
void odometry_update(float v_left, float v_right, float dt) {
  if (!odom_configured || dt <= 0.0f) {
    return;
  }

  /* 计算线速度和角速度 */
  float v = (v_left + v_right) / 2.0f;
  float w = (v_right - v_left) / odom_config.wheel_base;

  rt_mutex_take(odom_mutex, RT_WAITING_FOREVER);

  /* 更新位姿 (使用中点积分) */
  float theta_mid = odom_state.theta + w * dt / 2.0f;
  odom_state.x += v * cosf(theta_mid) * dt;
  odom_state.y += v * sinf(theta_mid) * dt;
  odom_state.theta = normalize_angle(odom_state.theta + w * dt);

  /* 更新速度 */
  odom_state.v = v;
  odom_state.w = w;

  /* 更新时间戳 */
  odom_state.timestamp_ms = rt_tick_get() * 1000 / RT_TICK_PER_SECOND;

  rt_mutex_release(odom_mutex);
}

/**
 * @brief 获取当前里程计状态
 */
void odometry_get_state(OdometryState *state) {
  if (state == RT_NULL) {
    return;
  }

  rt_mutex_take(odom_mutex, RT_WAITING_FOREVER);
  *state = odom_state;
  rt_mutex_release(odom_mutex);
}

/**
 * @brief 重置里程计
 */
void odometry_reset(void) {
  rt_mutex_take(odom_mutex, RT_WAITING_FOREVER);
  odom_state.x = 0.0f;
  odom_state.y = 0.0f;
  odom_state.theta = 0.0f;
  odom_state.v = 0.0f;
  odom_state.w = 0.0f;
  odom_state.timestamp_ms = 0;
  rt_mutex_release(odom_mutex);

  rt_kprintf("[Odometry] Reset\n");
}

/**
 * @brief 检查里程计是否已配置
 */
rt_bool_t odometry_is_configured(void) { return odom_configured; }

/**
 * @brief 获取配置的轮径
 */
float odometry_get_wheel_radius(void) { return odom_config.wheel_radius; }

/**
 * @brief 获取配置的轮距
 */
float odometry_get_wheel_base(void) { return odom_config.wheel_base; }

/* ================= MSH 调试命令 ================= */

/**
 * @brief MSH 命令: 显示里程计状态
 */
static int cmd_odom_info(int argc, char *argv[]) {
  (void)argc;
  (void)argv;

  OdometryState state;
  odometry_get_state(&state);

  rt_kprintf("[Odometry] Position: x=%.3f, y=%.3f, theta=%.3f rad (%.1f deg)\n",
             state.x, state.y, state.theta, state.theta * 180.0f / M_PI);
  rt_kprintf("[Odometry] Velocity: v=%.3f m/s, w=%.3f rad/s\n", state.v,
             state.w);
  rt_kprintf("[Odometry] Timestamp: %u ms\n", state.timestamp_ms);
  rt_kprintf("[Odometry] Configured: %s\n",
             odom_configured ? "YES" : "NO (waiting for CFG command)");

  if (odom_configured) {
    rt_kprintf("[Odometry] Config: R=%.4f m, L=%.4f m, G=%.1f, PPR=%.0f\n",
               odom_config.wheel_radius, odom_config.wheel_base,
               odom_config.gear_ratio, odom_config.encoder_ppr);
  }

  return 0;
}

/**
 * @brief MSH 命令: 重置里程计
 */
static int cmd_odom_reset(int argc, char *argv[]) {
  (void)argc;
  (void)argv;

  odometry_reset();
  return 0;
}

#include <finsh.h>
MSH_CMD_EXPORT(cmd_odom_info, Show odometry status);
MSH_CMD_EXPORT(cmd_odom_reset, Reset odometry);
