/*
 * RPMsg 电机控制模块 - 实现
 *
 * 实现大核(Linux)与小核(RCPU)之间的异步双向通信:
 *
 * 新协议:
 * - 接收配置命令: "CFG:wheel_radius=0.05;wheel_base=0.2;gear_ratio=56;ppr=11"
 * - 接收速度指令: "VEL:0.5,0.2" (线速度 m/s, 角速度 rad/s)
 * - 发送里程计反馈: "ODM:x,y,theta,v,w,timestamp_ms"
 *
 * 旧协议 (兼容):
 * - 接收速度指令: "1,0.5;1,0.5" (方向1,转速1;方向2,转速2)
 * - 发送状态反馈: "1,500;2,480" (方向1,转速1 mr/s;方向2,转速2 mr/s)
 */

#include <math.h>
#include <openamp/remoteproc.h>
#include <openamp/rpmsg.h>
#include <openamp/rpmsg_virtio.h>
#include <openamp/virtio.h>
#include <rtdef.h>
#include <rtthread.h>
#include <stdlib.h>
#include <string.h>

#include "odometry.h"
#include "rpmsg_motor.h"

/* ================= 配置参数 ================= */

#define RPMSG_MOTOR_SERVICE_NAME "rpmsg:motor_ctrl"
#define RPMSG_MOTOR_ADDR_SRC 1002
#define RPMSG_MOTOR_ADDR_DST 1003

#define FEEDBACK_THREAD_STACK_SIZE 4096
#define FEEDBACK_THREAD_PRIORITY 15
#define FEEDBACK_THREAD_TIMESLICE 5

#define DEFAULT_FEEDBACK_INTERVAL_MS 20 /* 默认反馈间隔 20ms (50Hz) */

/* 命令前缀 */
#define CMD_PREFIX_CFG "CFG:"
#define CMD_PREFIX_VEL "VEL:"
#define CMD_PREFIX_RST "RST:" /* 里程计重置 */

/* ================= 外部依赖 ================= */

extern struct rpmsg_device *rpdev;

/* ================= 模块状态 ================= */

struct rpmsg_motor_ctx {
  char *service_name;
  struct rpmsg_endpoint endp;
  rt_bool_t endpoint_ready;
};

static struct rpmsg_motor_ctx motor_ctx;
static rt_thread_t feedback_thread = RT_NULL;
static int feedback_interval_ms = DEFAULT_FEEDBACK_INTERVAL_MS;
static rt_bool_t feedback_enabled = RT_TRUE;

/* 新协议模式标志 */
static rt_bool_t new_protocol_enabled = RT_FALSE;

/* 目标速度 (m/s, rad/s) - 供控制线程使用 */
static volatile float target_linear_vel = 0.0f;
static volatile float target_angular_vel = 0.0f;
static rt_mutex_t vel_mutex = RT_NULL;

/* ================= CFG 命令解析 ================= */

/**
 * @brief 解析 CFG 命令
 *        格式: "wheel_radius=0.05;wheel_base=0.2;gear_ratio=56;ppr=11"
 */
static rt_err_t parse_cfg_command(const char *cmd) {
  float wheel_radius = 0.0f;
  float wheel_base = 0.0f;
  float gear_ratio = 0.0f;
  float encoder_ppr = 0.0f;

  char buf[128];
  char *token;
  char *saveptr;

  strncpy(buf, cmd, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  /* 按分号分割 */
  token = strtok_r(buf, ";", &saveptr);
  while (token != RT_NULL) {
    char *eq = strchr(token, '=');
    if (eq != RT_NULL) {
      *eq = '\0';
      char *key = token;
      float value = atof(eq + 1);

      if (strcmp(key, "wheel_radius") == 0) {
        wheel_radius = value;
      } else if (strcmp(key, "wheel_base") == 0) {
        wheel_base = value;
      } else if (strcmp(key, "gear_ratio") == 0) {
        gear_ratio = value;
      } else if (strcmp(key, "ppr") == 0) {
        encoder_ppr = value;
      }
    }
    token = strtok_r(RT_NULL, ";", &saveptr);
  }

  /* 检查必要参数 */
  if (wheel_radius <= 0.0f || wheel_base <= 0.0f) {
    rt_kprintf("[rpmsg_motor] CFG error: invalid wheel_radius or wheel_base\n");
    return -RT_ERROR;
  }

  /* 应用配置 */
  odometry_set_params(wheel_radius, wheel_base, gear_ratio, encoder_ppr);
  new_protocol_enabled = RT_TRUE;

  rt_kprintf("[rpmsg_motor] CFG applied: R=%.4f, L=%.4f, G=%.1f, PPR=%.0f\n",
             wheel_radius, wheel_base, gear_ratio, encoder_ppr);

  return RT_EOK;
}

/* ================= VEL 命令解析 ================= */

/**
 * @brief 解析 VEL 命令
 *        格式: "v,w" (线速度 m/s, 角速度 rad/s)
 */
static rt_err_t parse_vel_command(const char *cmd, float *v, float *w) {
  char buf[32];
  char *comma;

  strncpy(buf, cmd, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  *v = 0.0f;
  *w = 0.0f;

  comma = strchr(buf, ',');
  if (comma != RT_NULL) {
    *comma = '\0';
    *v = atof(buf);
    *w = atof(comma + 1);
  } else {
    /* 只有线速度，角速度为 0 */
    *v = atof(buf);
    *w = 0.0f;
  }

  return RT_EOK;
}

/* ================= 旧协议解析 (兼容) ================= */

/**
 * @brief 解析旧协议速度指令
 *        格式: "1,0.5;1,0.5" (方向1,转速1;方向2,转速2)
 */
static rt_err_t parse_legacy_speed_command(const char *cmd, int *dir1,
                                           double *speed1, int *dir2,
                                           double *speed2) {
  char buf[64];
  char *semicolon;
  char *comma;

  if (cmd == RT_NULL || *cmd == '\0') {
    return -RT_ERROR;
  }

  strncpy(buf, cmd, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  *dir1 = 0;
  *speed1 = 0.0;
  *dir2 = 0;
  *speed2 = 0.0;

  semicolon = strchr(buf, ';');
  if (semicolon != RT_NULL) {
    *semicolon = '\0';

    comma = strchr(buf, ',');
    if (comma != RT_NULL) {
      *comma = '\0';
      *dir1 = atoi(buf);
      *speed1 = atof(comma + 1);
    }

    char *motor2_str = semicolon + 1;
    comma = strchr(motor2_str, ',');
    if (comma != RT_NULL) {
      *comma = '\0';
      *dir2 = atoi(motor2_str);
      *speed2 = atof(comma + 1);
    }
  } else {
    comma = strchr(buf, ',');
    if (comma != RT_NULL) {
      *comma = '\0';
      *dir1 = atoi(buf);
      *speed1 = atof(comma + 1);
    }
  }

  return RT_EOK;
}

/* ================= 速度设置接口 ================= */

/**
 * @brief 设置目标速度 (m/s, rad/s)
 *        供外部调用
 */
void rpmsg_motor_set_velocity(float linear, float angular) {
  if (vel_mutex != RT_NULL) {
    rt_mutex_take(vel_mutex, RT_WAITING_FOREVER);
  }
  target_linear_vel = linear;
  target_angular_vel = angular;
  if (vel_mutex != RT_NULL) {
    rt_mutex_release(vel_mutex);
  }
}

/**
 * @brief 获取目标速度 (m/s, rad/s)
 */
void rpmsg_motor_get_velocity(float *linear, float *angular) {
  if (vel_mutex != RT_NULL) {
    rt_mutex_take(vel_mutex, RT_WAITING_FOREVER);
  }
  *linear = target_linear_vel;
  *angular = target_angular_vel;
  if (vel_mutex != RT_NULL) {
    rt_mutex_release(vel_mutex);
  }
}

/**
 * @brief 将 (v, w) 转换为左右轮速度 (m/s)
 */
void rpmsg_motor_vel_to_wheel_speeds(float v, float w, float *v_left,
                                     float *v_right) {
  float wheel_base = odometry_get_wheel_base();
  if (wheel_base <= 0.0f) {
    wheel_base = 0.2f; /* 默认值 */
  }

  /* 差速驱动逆运动学 */
  *v_left = v - (w * wheel_base / 2.0f);
  *v_right = v + (w * wheel_base / 2.0f);
}

/**
 * @brief 将轮子线速度 (m/s) 转换为电机转速 (转/秒)
 */
float rpmsg_motor_wheel_speed_to_motor_rps(float wheel_speed) {
  float wheel_radius = odometry_get_wheel_radius();
  if (wheel_radius <= 0.0f) {
    return 0.0f;
  }

  /* 轮子角速度 (rad/s) = 线速度 / 轮径 */
  /* 轮子转速 (r/s) = 角速度 / (2 * PI) */
  /* 电机转速 = 轮子转速 (减速比在控制环节处理) */
  return wheel_speed / (2.0f * M_PI * wheel_radius);
}

/* ================= RPMsg 回调函数 ================= */

/**
 * @brief RPMsg 端点回调函数 - 接收命令
 */
static int rpmsg_motor_endpoint_cb(struct rpmsg_endpoint *ept, void *data,
                                   size_t len, uint32_t src, void *priv) {
  char *recv_str = (char *)data;

  (void)ept;
  (void)priv;
  (void)len;

  rt_kprintf("[rpmsg_motor] Recv: \"%s\" (src=%d)\n", recv_str, src);

  /* 检测命令类型 */
  if (strncmp(recv_str, CMD_PREFIX_CFG, strlen(CMD_PREFIX_CFG)) == 0) {
    /* CFG 配置命令 */
    const char *cfg_data = recv_str + strlen(CMD_PREFIX_CFG);
    if (parse_cfg_command(cfg_data) == RT_EOK) {
      rt_kprintf("[rpmsg_motor] Configuration applied\n");
    }
  } else if (strncmp(recv_str, CMD_PREFIX_VEL, strlen(CMD_PREFIX_VEL)) == 0) {
    /* VEL 速度命令 */
    const char *vel_data = recv_str + strlen(CMD_PREFIX_VEL);
    float v = 0.0f, w = 0.0f;
    if (parse_vel_command(vel_data, &v, &w) == RT_EOK) {
      rt_kprintf("[rpmsg_motor] VEL: v=%.3f m/s, w=%.3f rad/s\n", v, w);

      /* 保存目标速度 */
      rpmsg_motor_set_velocity(v, w);

      /* 转换为左右轮速度 */
      float v_left, v_right;
      rpmsg_motor_vel_to_wheel_speeds(v, w, &v_left, &v_right);

      /* 转换为电机转速 (转/秒) */
      float rps_left = rpmsg_motor_wheel_speed_to_motor_rps(v_left);
      float rps_right = rpmsg_motor_wheel_speed_to_motor_rps(v_right);

      /* 确定方向 */
      int dir1 = (rps_left > 0.001f) ? 1 : ((rps_left < -0.001f) ? 2 : 0);
      int dir2 = (rps_right > 0.001f) ? 1 : ((rps_right < -0.001f) ? 2 : 0);

      rt_kprintf("[rpmsg_motor] Wheel: L=%.3f m/s, R=%.3f m/s -> Motor: "
                 "D1=%d,%.2f r/s, D2=%d,%.2f r/s\n",
                 v_left, v_right, dir1, fabsf(rps_left), dir2,
                 fabsf(rps_right));

      /* 设置电机目标 */
      chassis_set_target(dir1, fabsf(rps_left), dir2, fabsf(rps_right));
    }
  } else if (strncmp(recv_str, CMD_PREFIX_RST, strlen(CMD_PREFIX_RST)) == 0) {
    /* RST 里程计重置命令 */
    odometry_reset();
    rt_kprintf("[rpmsg_motor] Odometry reset\n");
  } else {
    /* 尝试旧协议格式 */
    int dir1 = 0, dir2 = 0;
    double speed1 = 0.0, speed2 = 0.0;
    if (parse_legacy_speed_command(recv_str, &dir1, &speed1, &dir2, &speed2) ==
        RT_EOK) {
      rt_kprintf("[rpmsg_motor] Legacy: M1(dir=%d, speed=%.2f), M2(dir=%d, "
                 "speed=%.2f)\n",
                 dir1, speed1, dir2, speed2);
      chassis_set_target(dir1, speed1, dir2, speed2);
    } else {
      rt_kprintf("[rpmsg_motor] Unknown command format!\n");
    }
  }

  return 0;
}

/**
 * @brief 服务解绑回调
 */
static void rpmsg_motor_service_unbind(struct rpmsg_endpoint *ept) {
  (void)ept;
  rt_kprintf("[rpmsg_motor] Service unbound\n");
  motor_ctx.endpoint_ready = RT_FALSE;
}

/* ================= 状态反馈线程 ================= */

/**
 * @brief 状态反馈线程入口
 *        定期主动发送状态给大核
 */
static void feedback_thread_entry(void *parameter) {
  char feedback_buf[128];
  int dir1, dir2;
  int speed1_mrs, speed2_mrs;
  OdometryState odom;

  (void)parameter;

  rt_kprintf("[rpmsg_motor] Feedback thread started (interval=%dms)\n",
             feedback_interval_ms);

  while (1) {
    /* 等待端点就绪 */
    if (!motor_ctx.endpoint_ready) {
      rt_thread_mdelay(100);
      continue;
    }

    if (feedback_enabled) {
      if (new_protocol_enabled && odometry_is_configured()) {
        /* 新协议: 发送里程计数据 */
        odometry_get_state(&odom);

        /* 格式: "ODM:x,y,theta,v,w,timestamp" */
        rt_snprintf(feedback_buf, sizeof(feedback_buf),
                    "ODM:%.4f,%.4f,%.4f,%.3f,%.3f,%u", odom.x, odom.y,
                    odom.theta, odom.v, odom.w, odom.timestamp_ms);
      } else {
        /* 旧协议: 发送电机状态 */
        chassis_get_status(&dir1, &speed1_mrs, &dir2, &speed2_mrs);
        rt_snprintf(feedback_buf, sizeof(feedback_buf), "%d,%d;%d,%d", dir1,
                    speed1_mrs, dir2, speed2_mrs);
      }

      /* 发送反馈 */
      int ret =
          rpmsg_send(&motor_ctx.endp, feedback_buf, strlen(feedback_buf) + 1);
      if (ret < 0) {
        rt_kprintf("[rpmsg_motor] Send feedback failed: %d\n", ret);
      }

      /* 调试: 每10次打印一次 */
      static int print_cnt = 0;
      if (++print_cnt >= 10) {
        rt_kprintf("[rpmsg_motor] Feedback: %s\n", feedback_buf);
        print_cnt = 0;
      }
    }

    rt_thread_mdelay(feedback_interval_ms);
  }
}

/* ================= RPMsg 初始化线程 ================= */

/**
 * @brief RPMsg 初始化线程入口
 */
static void rpmsg_motor_init_thread_entry(void *parameter) {
  int ret;

  (void)parameter;

  /* 等待 rpdev 初始化完成 */
  while (rpdev == RT_NULL) {
    rt_thread_delay(10);
  }

  rt_kprintf("[rpmsg_motor] rpdev ready, creating endpoint...\n");

  motor_ctx.service_name = RPMSG_MOTOR_SERVICE_NAME;

  /* 创建 RPMsg 端点 */
  ret = rpmsg_create_ept(&motor_ctx.endp, rpdev, motor_ctx.service_name,
                         RPMSG_MOTOR_ADDR_SRC, RPMSG_MOTOR_ADDR_DST,
                         rpmsg_motor_endpoint_cb, rpmsg_motor_service_unbind);
  if (ret) {
    rt_kprintf("[rpmsg_motor] Create endpoint failed, ret=%d\n", ret);
    return;
  }

  motor_ctx.endpoint_ready = RT_TRUE;

  rt_kprintf("[rpmsg_motor] Endpoint created: %s (src=%d, dst=%d)\n",
             motor_ctx.service_name, RPMSG_MOTOR_ADDR_SRC,
             RPMSG_MOTOR_ADDR_DST);
  rt_kprintf("[rpmsg_motor] Ready. Commands:\n");
  rt_kprintf("  CFG:wheel_radius=R;wheel_base=L;gear_ratio=G;ppr=P\n");
  rt_kprintf("  VEL:v,w  (linear m/s, angular rad/s)\n");
  rt_kprintf("  RST:     (reset odometry)\n");
  rt_kprintf("  Legacy: dir1,speed1;dir2,speed2\n");
}

/* ================= 公共接口 ================= */

/**
 * @brief 初始化 RPMsg 电机控制服务
 */
rt_err_t rpmsg_motor_init(void) {
  rt_thread_t tid;

  rt_memset(&motor_ctx, 0, sizeof(motor_ctx));
  motor_ctx.endpoint_ready = RT_FALSE;

  /* 创建速度互斥锁 */
  vel_mutex = rt_mutex_create("vel_mtx", RT_IPC_FLAG_PRIO);

  /* 初始化里程计模块 */
  odometry_init();

  /* 创建初始化线程 */
  tid = rt_thread_create("rpmsg_mi", rpmsg_motor_init_thread_entry, RT_NULL,
                         4096, RT_THREAD_PRIORITY_MAX / 3, 20);
  if (!tid) {
    rt_kprintf("[rpmsg_motor] Failed to create init thread\n");
    return -RT_EINVAL;
  }
  rt_thread_startup(tid);

  /* 创建状态反馈线程 */
  feedback_thread = rt_thread_create(
      "rpmsg_fb", feedback_thread_entry, RT_NULL, FEEDBACK_THREAD_STACK_SIZE,
      FEEDBACK_THREAD_PRIORITY, FEEDBACK_THREAD_TIMESLICE);
  if (!feedback_thread) {
    rt_kprintf("[rpmsg_motor] Failed to create feedback thread\n");
    return -RT_EINVAL;
  }
  rt_thread_startup(feedback_thread);

  rt_kprintf("[rpmsg_motor] Service starting...\n");

  return RT_EOK;
}

/**
 * @brief 设置状态反馈间隔
 */
void rpmsg_motor_set_feedback_interval(int ms) {
  if (ms < 10)
    ms = 10; /* 最小 10ms */
  feedback_interval_ms = ms;
  rt_kprintf("[rpmsg_motor] Feedback interval set to %dms\n", ms);
}

/* ================= MSH 命令 ================= */

/**
 * @brief MSH 命令: 启用/禁用反馈
 */
static int cmd_rpmsg_feedback(int argc, char *argv[]) {
  if (argc < 2) {
    rt_kprintf("Usage: rpmsg_feedback <on|off|interval_ms>\n");
    rt_kprintf("Current: enabled=%d, interval=%dms, new_protocol=%d\n",
               feedback_enabled, feedback_interval_ms, new_protocol_enabled);
    return 0;
  }

  if (strcmp(argv[1], "on") == 0) {
    feedback_enabled = RT_TRUE;
    rt_kprintf("[rpmsg_motor] Feedback enabled\n");
  } else if (strcmp(argv[1], "off") == 0) {
    feedback_enabled = RT_FALSE;
    rt_kprintf("[rpmsg_motor] Feedback disabled\n");
  } else {
    int interval = atoi(argv[1]);
    if (interval > 0) {
      rpmsg_motor_set_feedback_interval(interval);
    } else {
      rt_kprintf("Invalid argument: %s\n", argv[1]);
    }
  }

  return 0;
}

#include <finsh.h>
MSH_CMD_EXPORT(cmd_rpmsg_feedback, Enable / disable motor status feedback);
