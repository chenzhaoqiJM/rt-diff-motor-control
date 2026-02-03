/*
 * Linux 端 RPMsg 电机控制程序 (异步版)
 *
 * 实现与 RCPU 端 rpmsg_motor.c 的异步双向通信:
 *
 * 新协议:
 * - 发送配置: "CFG:wheel_radius=R;wheel_base=L;gear_ratio=G;ppr=P"
 * - 发送速度: "VEL:v,w" (线速度 m/s, 角速度 rad/s)
 * - 接收里程计: "ODM:x,y,theta,v,w,timestamp_ms"
 * - 里程计重置: "RST:"
 *
 * 旧协议 (兼容):
 * - 发送速度: "dir1,speed1;dir2,speed2"
 * - 接收状态: "dir1,speed1_mrs;dir2,speed2_mrs"
 *
 * 编译: gcc -o rpmsg_motor_async rpmsg_motor_async.c -lpthread -lm
 * 运行: ./rpmsg_motor_async
 */

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

/* ================= RPMsg 配置 ================= */

#define RPMSG_NAME_SIZE 32
#define RPMSG_ADDR_ANY 0xFFFFFFFF

/* RPMsg 服务配置 - 必须与 RCPU 端匹配 */
#define RPMSG_SERVICE_NAME "rpmsg:motor_ctrl"
#define RPMSG_LOCAL_ADDR 1003  /* Linux 端地址 (对应 RCPU 的 dst) */
#define RPMSG_REMOTE_ADDR 1002 /* RCPU 端地址 (对应 RCPU 的 src) */

/**
 * struct rpmsg_endpoint_info - 端点信息结构
 */
struct rpmsg_endpoint_info {
  char name[32];
  unsigned int src;
  unsigned int dst;
};

/* RPMsg ioctl 命令定义 */
#define RPMSG_CREATE_EPT_IOCTL _IOW(0xb5, 0x1, struct rpmsg_endpoint_info)
#define RPMSG_DESTROY_EPT_IOCTL _IO(0xb5, 0x2)

/* ================= 全局变量 ================= */

static int rpmsg_fd = -1;
static int rpmsg_ctrl_fd = -1;
static volatile int running = 1;

/* 新协议模式标志 */
static volatile int new_protocol_mode = 0;

/* 里程计状态 */
typedef struct {
  double x;
  double y;
  double theta;
  double v;
  double w;
  unsigned int timestamp_ms;
} OdometryState;

static OdometryState odom_state = {0};
static pthread_mutex_t odom_mutex = PTHREAD_MUTEX_INITIALIZER;

/* 旧协议状态 */
static int recv_dir1 = 0, recv_dir2 = 0;
static int recv_speed1 = 0, recv_speed2 = 0;
static pthread_mutex_t status_mutex = PTHREAD_MUTEX_INITIALIZER;

/* ================= 信号处理 ================= */

static void signal_handler(int sig) {
  (void)sig;
  printf("\n[Linux] Caught signal, exiting...\n");
  running = 0;
}

/* ================= 接收线程 ================= */

/* 打印频率控制: 每 PRINT_EVERY_N 条消息打印一次 */
#define PRINT_EVERY_N 50

/**
 * @brief 解析 ODM 格式反馈
 */
static int parse_odm_feedback(const char *data, OdometryState *odom) {
  return sscanf(data, "%lf,%lf,%lf,%lf,%lf,%u", &odom->x, &odom->y,
                &odom->theta, &odom->v, &odom->w, &odom->timestamp_ms) == 6;
}

/**
 * @brief 接收线程 - 异步接收 RCPU 的状态反馈
 */
static void *recv_thread_func(void *arg) {
  char recv_buf[256];
  int ret;
  struct pollfd pfd;

  /* 频率统计变量 */
  struct timeval start_time, current_time;
  int msg_count = 0;
  int print_count = 0;
  double elapsed_time;
  gettimeofday(&start_time, NULL);

  (void)arg;

  printf("[Linux] Receive thread started (print every %d msgs)\n",
         PRINT_EVERY_N);

  pfd.fd = rpmsg_fd;
  pfd.events = POLLIN;

  while (running) {
    /* 使用 poll 实现超时等待 */
    ret = poll(&pfd, 1, 100); /* 100ms 超时 */
    if (ret < 0) {
      if (errno == EINTR)
        continue;
      perror("[Linux] Poll error");
      break;
    }

    if (ret == 0) {
      continue;
    }

    if (pfd.revents & POLLIN) {
      memset(recv_buf, 0, sizeof(recv_buf));
      ret = read(rpmsg_fd, recv_buf, sizeof(recv_buf) - 1);
      if (ret < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
          continue;
        perror("[Linux] Read error");
        break;
      }

      if (ret > 0) {
        /* 更新频率统计 */
        msg_count++;
        print_count++;
        gettimeofday(&current_time, NULL);
        elapsed_time = (current_time.tv_sec - start_time.tv_sec) +
                       (current_time.tv_usec - start_time.tv_usec) / 1000000.0;

        /* 检测消息类型 */
        if (strncmp(recv_buf, "ODM:", 4) == 0) {
          /* 新协议: 里程计数据 */
          new_protocol_mode = 1;
          OdometryState odom;
          if (parse_odm_feedback(recv_buf + 4, &odom)) {
            pthread_mutex_lock(&odom_mutex);
            odom_state = odom;
            pthread_mutex_unlock(&odom_mutex);

            /* 每 PRINT_EVERY_N 条消息打印一次 */
            if (print_count >= PRINT_EVERY_N) {
              double current_freq =
                  (elapsed_time > 0) ? msg_count / elapsed_time : 0;
              printf("[Odometry] x=%.3f y=%.3f theta=%.2f rad (%.1f deg) | "
                     "v=%.2f m/s w=%.2f rad/s | t=%u | %.1f Hz\n",
                     odom.x, odom.y, odom.theta, odom.theta * 180.0 / M_PI,
                     odom.v, odom.w, odom.timestamp_ms, current_freq);
              print_count = 0;
            }
          }
        } else {
          /* 旧协议: 电机状态 */
          int d1 = 0, s1 = 0, d2 = 0, s2 = 0;
          if (sscanf(recv_buf, "%d,%d;%d,%d", &d1, &s1, &d2, &s2) == 4) {
            pthread_mutex_lock(&status_mutex);
            recv_dir1 = d1;
            recv_speed1 = s1;
            recv_dir2 = d2;
            recv_speed2 = s2;
            pthread_mutex_unlock(&status_mutex);

            if (print_count >= PRINT_EVERY_N) {
              double current_freq =
                  (elapsed_time > 0) ? msg_count / elapsed_time : 0;
              printf("[Feedback] M1: dir=%d, speed=%d mr/s (%.3f r/s) | "
                     "M2: dir=%d, speed=%d mr/s (%.3f r/s) | Freq: %.1f Hz\n",
                     d1, s1, s1 / 1000.0, d2, s2, s2 / 1000.0, current_freq);
              print_count = 0;
            }
          } else {
            printf("[Linux] Recv: %s\n", recv_buf);
          }
        }

        /* 每10秒重置统计 */
        if (elapsed_time >= 10.0) {
          gettimeofday(&start_time, NULL);
          msg_count = 0;
        }
      }
    }
  }

  printf("[Linux] Receive thread exiting\n");
  return NULL;
}

/* ================= 发送函数 ================= */

/**
 * @brief 发送 CFG 配置命令
 */
static int send_cfg_command(double wheel_radius, double wheel_base,
                            double gear_ratio, double ppr) {
  char cmd_buf[128];

  snprintf(cmd_buf, sizeof(cmd_buf),
           "CFG:wheel_radius=%.4f;wheel_base=%.4f;gear_ratio=%.1f;ppr=%.0f",
           wheel_radius, wheel_base, gear_ratio, ppr);

  printf("[Linux] Sending: %s\n", cmd_buf);

  int ret = write(rpmsg_fd, cmd_buf, strlen(cmd_buf) + 1);
  if (ret < 0) {
    perror("[Linux] Write failed");
    return -1;
  }

  return 0;
}

/**
 * @brief 发送 VEL 速度命令
 */
static int send_vel_command(double v, double w) {
  char cmd_buf[64];

  snprintf(cmd_buf, sizeof(cmd_buf), "VEL:%.3f,%.3f", v, w);

  printf("[Linux] Sending: %s\n", cmd_buf);

  int ret = write(rpmsg_fd, cmd_buf, strlen(cmd_buf) + 1);
  if (ret < 0) {
    perror("[Linux] Write failed");
    return -1;
  }

  return 0;
}

/**
 * @brief 发送 RST 里程计重置命令
 */
static int send_rst_command(void) {
  const char *cmd = "RST:";

  printf("[Linux] Sending: %s\n", cmd);

  int ret = write(rpmsg_fd, cmd, strlen(cmd) + 1);
  if (ret < 0) {
    perror("[Linux] Write failed");
    return -1;
  }

  return 0;
}

/**
 * @brief 发送旧协议速度指令
 */
static int send_legacy_speed_command(int dir1, double speed1, int dir2,
                                     double speed2) {
  char cmd_buf[64];

  snprintf(cmd_buf, sizeof(cmd_buf), "%d,%.3f;%d,%.3f", dir1, speed1, dir2,
           speed2);

  printf("[Linux] Sending: %s\n", cmd_buf);

  int ret = write(rpmsg_fd, cmd_buf, strlen(cmd_buf) + 1);
  if (ret < 0) {
    perror("[Linux] Write failed");
    return -1;
  }

  return 0;
}

/**
 * @brief 发送字符串指令
 */
static int send_raw_command(const char *cmd) {
  printf("[Linux] Sending: %s\n", cmd);

  int ret = write(rpmsg_fd, cmd, strlen(cmd) + 1);
  if (ret < 0) {
    perror("[Linux] Write failed");
    return -1;
  }

  return 0;
}

/* ================= 初始化/清理 ================= */

/**
 * @brief 初始化 RPMsg 连接
 */
static int rpmsg_init(void) {
  struct rpmsg_endpoint_info epinfo;
  int ret;

  printf("[Linux] RPMsg Motor Control (Async)\n");
  printf("[Linux] Service: %s, Local: %d, Remote: %d\n", RPMSG_SERVICE_NAME,
         RPMSG_LOCAL_ADDR, RPMSG_REMOTE_ADDR);

  /* 1. 打开 RPMsg 控制设备 */
  rpmsg_ctrl_fd = open("/dev/rpmsg_ctrl0", O_RDWR);
  if (rpmsg_ctrl_fd < 0) {
    perror("[Linux] Failed to open /dev/rpmsg_ctrl0");
    return -1;
  }

  /* 2. 配置端点信息 */
  memset(&epinfo, 0, sizeof(epinfo));
  strncpy(epinfo.name, RPMSG_SERVICE_NAME, RPMSG_NAME_SIZE - 1);
  epinfo.src = RPMSG_LOCAL_ADDR;
  epinfo.dst = RPMSG_REMOTE_ADDR;

  /* 3. 创建 RPMsg 端点 */
  ret = ioctl(rpmsg_ctrl_fd, RPMSG_CREATE_EPT_IOCTL, &epinfo);
  if (ret < 0) {
    perror("[Linux] Failed to create endpoint");
    close(rpmsg_ctrl_fd);
    return -1;
  }
  printf("[Linux] Endpoint created successfully\n");

  /* 4. 打开 RPMsg 数据通道 */
  rpmsg_fd = open("/dev/rpmsg0", O_RDWR);
  if (rpmsg_fd < 0) {
    perror("[Linux] Failed to open /dev/rpmsg0");
    close(rpmsg_ctrl_fd);
    return -1;
  }

  /* 设置为非阻塞模式 */
  int flags = fcntl(rpmsg_fd, F_GETFL, 0);
  fcntl(rpmsg_fd, F_SETFL, flags | O_NONBLOCK);

  printf("[Linux] RPMsg initialized\n");
  return 0;
}

/**
 * @brief 清理 RPMsg 连接
 */
static void rpmsg_cleanup(void) {
  if (rpmsg_fd >= 0) {
    close(rpmsg_fd);
    rpmsg_fd = -1;
  }
  if (rpmsg_ctrl_fd >= 0) {
    close(rpmsg_ctrl_fd);
    rpmsg_ctrl_fd = -1;
  }
  printf("[Linux] RPMsg cleaned up\n");
}

/* ================= 用户交互 ================= */

static void print_help(void) {
  printf("\n--- RPMsg Motor Control Commands ---\n");
  printf("New Protocol:\n");
  printf("  cfg <R> <L> <G> <P>   - Configure: wheel_radius, wheel_base, "
         "gear_ratio, ppr\n");
  printf("                         Example: cfg 0.05 0.2 56 11\n");
  printf("  vel <v> <w>           - Set velocity: linear (m/s), angular "
         "(rad/s)\n");
  printf("                         Example: vel 0.5 0.2\n");
  printf("  rst                   - Reset odometry\n");
  printf("\nLegacy Protocol:\n");
  printf("  <dir1>,<speed1>;<dir2>,<speed2>  - Set speed (e.g. 1,0.5;1,0.5)\n");
  printf("  stop                             - Stop both motors\n");
  printf("\nOther:\n");
  printf("  status / odom        - Show last received status/odometry\n");
  printf("  help                 - Show this help\n");
  printf("  quit / exit          - Exit program\n");
  printf("------------------------------------\n\n");
}

static void show_status(void) {
  if (new_protocol_mode) {
    pthread_mutex_lock(&odom_mutex);
    printf("[Odometry] x=%.4f m, y=%.4f m, theta=%.4f rad (%.2f deg)\n",
           odom_state.x, odom_state.y, odom_state.theta,
           odom_state.theta * 180.0 / M_PI);
    printf("[Odometry] v=%.3f m/s, w=%.3f rad/s\n", odom_state.v, odom_state.w);
    printf("[Odometry] timestamp=%u ms\n", odom_state.timestamp_ms);
    pthread_mutex_unlock(&odom_mutex);
  } else {
    pthread_mutex_lock(&status_mutex);
    printf("[Status] M1: dir=%d, speed=%d mr/s (%.3f r/s)\n", recv_dir1,
           recv_speed1, recv_speed1 / 1000.0);
    printf("[Status] M2: dir=%d, speed=%d mr/s (%.3f r/s)\n", recv_dir2,
           recv_speed2, recv_speed2 / 1000.0);
    pthread_mutex_unlock(&status_mutex);
  }
}

/* ================= 主函数 ================= */

int main(int argc, char *argv[]) {
  pthread_t recv_tid;
  char input[256];
  int dir1, dir2;
  double speed1, speed2;
  double v, w;
  double wheel_radius, wheel_base, gear_ratio, ppr;

  (void)argc;
  (void)argv;

  /* 设置信号处理 */
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  /* 初始化 RPMsg */
  if (rpmsg_init() < 0) {
    return -1;
  }

  /* 启动接收线程 */
  if (pthread_create(&recv_tid, NULL, recv_thread_func, NULL) != 0) {
    perror("[Linux] Failed to create receive thread");
    rpmsg_cleanup();
    return -1;
  }

  print_help();

  /* 设置 stdin 为非阻塞 */
  struct pollfd stdin_pfd;
  stdin_pfd.fd = STDIN_FILENO;
  stdin_pfd.events = POLLIN;

  /* 主循环 - 交互式命令输入 */
  while (running) {
    printf("> ");
    fflush(stdout);

    /* 使用 poll 等待输入 */
    int poll_ret = poll(&stdin_pfd, 1, 500);
    if (poll_ret < 0) {
      if (errno == EINTR)
        continue;
      break;
    }

    if (poll_ret == 0) {
      continue;
    }

    if (!(stdin_pfd.revents & POLLIN)) {
      continue;
    }

    if (fgets(input, sizeof(input), stdin) == NULL) {
      break;
    }

    /* 移除换行符 */
    input[strcspn(input, "\n")] = '\0';

    /* 空输入 */
    if (strlen(input) == 0) {
      continue;
    }

    /* 解析命令 */
    if (strcmp(input, "quit") == 0 || strcmp(input, "exit") == 0) {
      break;
    } else if (strcmp(input, "stop") == 0) {
      send_vel_command(0.0, 0.0);
    } else if (strcmp(input, "status") == 0 || strcmp(input, "odom") == 0) {
      show_status();
    } else if (strcmp(input, "help") == 0) {
      print_help();
    } else if (strcmp(input, "rst") == 0) {
      send_rst_command();
    } else if (sscanf(input, "cfg %lf %lf %lf %lf", &wheel_radius, &wheel_base,
                      &gear_ratio, &ppr) == 4) {
      /* CFG 配置命令 */
      send_cfg_command(wheel_radius, wheel_base, gear_ratio, ppr);
    } else if (sscanf(input, "vel %lf %lf", &v, &w) == 2) {
      /* VEL 速度命令 */
      send_vel_command(v, w);
    } else if (sscanf(input, "vel %lf", &v) == 1) {
      /* VEL 只有线速度 */
      send_vel_command(v, 0.0);
    } else if (sscanf(input, "%d,%lf;%d,%lf", &dir1, &speed1, &dir2, &speed2) ==
               4) {
      /* 旧协议双电机指令 */
      send_legacy_speed_command(dir1, speed1, dir2, speed2);
    } else if (sscanf(input, "%d,%lf", &dir1, &speed1) == 2) {
      /* 旧协议单电机指令 */
      send_legacy_speed_command(dir1, speed1, 0, 0.0);
    } else {
      /* 直接发送原始字符串 */
      send_raw_command(input);
    }
  }

  /* 停止接收线程 */
  running = 0;
  pthread_join(recv_tid, NULL);

  /* 清理 */
  rpmsg_cleanup();

  printf("[Linux] Exited\n");
  return 0;
}
