/*
 * RPMsg 电机控制模块 - 实现
 *
 * 实现大核(Linux)与小核(RCPU)之间的异步双向通信:
 *
 * 协议:
 * - 接收速度指令: "1,0.5;1,0.5" (方向1,转速1;方向2,转速2)
 * - 发送状态反馈: "1,500;2,480" (方向1,转速1 mr/s;方向2,转速2 mr/s)
 */

#include <openamp/remoteproc.h>
#include <openamp/rpmsg.h>
#include <openamp/rpmsg_virtio.h>
#include <openamp/virtio.h>
#include <rtdef.h>
#include <rtthread.h>
#include <stdlib.h>
#include <string.h>

#include "rpmsg_motor.h"

/* ================= 配置参数 ================= */

#define RPMSG_MOTOR_SERVICE_NAME "rpmsg:motor_ctrl"
#define RPMSG_MOTOR_ADDR_SRC 1002
#define RPMSG_MOTOR_ADDR_DST 1003

#define FEEDBACK_THREAD_STACK_SIZE 4096
#define FEEDBACK_THREAD_PRIORITY 15
#define FEEDBACK_THREAD_TIMESLICE 5

#define DEFAULT_FEEDBACK_INTERVAL_MS 20 /* 默认反馈间隔 20ms (50Hz) */

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

/* ================= 速度指令解析 ================= */

/**
 * @brief 解析速度指令
 *        格式: "1,0.5;1,0.5" (方向1,转速1;方向2,转速2)
 */
static rt_err_t parse_speed_command(const char *cmd, int *dir1, double *speed1,
                                    int *dir2, double *speed2) {
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

/* ================= RPMsg 回调函数 ================= */

/**
 * @brief RPMsg 端点回调函数 - 接收命令
 */
static int rpmsg_motor_endpoint_cb(struct rpmsg_endpoint *ept, void *data,
                                   size_t len, uint32_t src, void *priv) {
  char *recv_str = (char *)data;
  int dir1 = 0, dir2 = 0;
  double speed1 = 0.0, speed2 = 0.0;

  (void)ept;
  (void)priv;
  (void)len;

  rt_kprintf("[rpmsg_motor] Recv: \"%s\" (src=%d)\n", recv_str, src);

  /* 解析速度指令 */
  if (parse_speed_command(recv_str, &dir1, &speed1, &dir2, &speed2) == RT_EOK) {
    rt_kprintf("[rpmsg_motor] M1(dir=%d, speed=%.2f), M2(dir=%d, speed=%.2f)\n",
               dir1, speed1, dir2, speed2);
    chassis_set_target(dir1, speed1, dir2, speed2);
  } else {
    rt_kprintf("[rpmsg_motor] Unknown command format!\n");
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
  char feedback_buf[64];
  int dir1, dir2;
  int speed1_mrs, speed2_mrs;

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
      /* 发送电机状态反馈 */
      chassis_get_status(&dir1, &speed1_mrs, &dir2, &speed2_mrs);
      rt_snprintf(feedback_buf, sizeof(feedback_buf), "%d,%d;%d,%d", dir1,
                  speed1_mrs, dir2, speed2_mrs);

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
  rt_kprintf("[rpmsg_motor] Ready. Command format: dir1,speed1;dir2,speed2\n");
}

/* ================= 公共接口 ================= */

/**
 * @brief 初始化 RPMsg 电机控制服务
 */
rt_err_t rpmsg_motor_init(void) {
  rt_thread_t tid;

  rt_memset(&motor_ctx, 0, sizeof(motor_ctx));
  motor_ctx.endpoint_ready = RT_FALSE;

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
    rt_kprintf("Current: enabled=%d, interval=%dms\n", feedback_enabled,
               feedback_interval_ms);
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
