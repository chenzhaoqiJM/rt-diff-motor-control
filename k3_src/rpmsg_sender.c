/*
 * Linux 端 RPMsg 发送测试程序
 *
 * 用于测试大核→小核的发送速率
 * 编译: gcc -o rpmsg_sender rpmsg_sender.c
 * 运行: ./rpmsg_sender [消息数量] [间隔ms]
 *       ./rpmsg_sender 1000 10     # 发送1000条，间隔10ms
 *       ./rpmsg_sender 100 0       # 发送100条，无间隔(最大速率)
 */

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>

/* RPMsg 配置 */
#define RPMSG_SERVICE_NAME "rpmsg:motor_ctrl"
#define RPMSG_LOCAL_ADDR 1003
#define RPMSG_REMOTE_ADDR 1002

struct rpmsg_endpoint_info {
  char name[32];
  unsigned int src;
  unsigned int dst;
};

#define RPMSG_CREATE_EPT_IOCTL _IOW(0xb5, 0x1, struct rpmsg_endpoint_info)

static int rpmsg_fd = -1;
static int rpmsg_ctrl_fd = -1;
static volatile int running = 1;

static void signal_handler(int sig) {
  (void)sig;
  running = 0;
}

static int rpmsg_init(void) {
  struct rpmsg_endpoint_info epinfo;
  int ret;

  printf("[Sender] Initializing RPMsg...\n");

  rpmsg_ctrl_fd = open("/dev/rpmsg_ctrl0", O_RDWR);
  if (rpmsg_ctrl_fd < 0) {
    perror("Failed to open /dev/rpmsg_ctrl0");
    return -1;
  }

  memset(&epinfo, 0, sizeof(epinfo));
  strncpy(epinfo.name, RPMSG_SERVICE_NAME, sizeof(epinfo.name) - 1);
  epinfo.src = RPMSG_LOCAL_ADDR;
  epinfo.dst = RPMSG_REMOTE_ADDR;

  ret = ioctl(rpmsg_ctrl_fd, RPMSG_CREATE_EPT_IOCTL, &epinfo);
  if (ret < 0) {
    perror("Failed to create endpoint");
    close(rpmsg_ctrl_fd);
    return -1;
  }

  rpmsg_fd = open("/dev/rpmsg0", O_RDWR);
  if (rpmsg_fd < 0) {
    perror("Failed to open /dev/rpmsg0");
    close(rpmsg_ctrl_fd);
    return -1;
  }

  printf("[Sender] RPMsg initialized (service=%s, src=%d, dst=%d)\n",
         RPMSG_SERVICE_NAME, RPMSG_LOCAL_ADDR, RPMSG_REMOTE_ADDR);
  return 0;
}

static void rpmsg_cleanup(void) {
  if (rpmsg_fd >= 0)
    close(rpmsg_fd);
  if (rpmsg_ctrl_fd >= 0)
    close(rpmsg_ctrl_fd);
  printf("[Sender] Cleaned up\n");
}

static double get_time_ms(void) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec * 1000.0 + tv.tv_usec / 1000.0;
}

int main(int argc, char *argv[]) {
  int count = 100;      /* 默认发送100条 */
  int interval_ms = 20; /* 默认间隔20ms */
  int i, ret;
  int success = 0, failed = 0;
  char msg[64];
  double start_time, end_time, elapsed;

  /* 解析参数 */
  if (argc >= 2)
    count = atoi(argv[1]);
  if (argc >= 3)
    interval_ms = atoi(argv[2]);

  printf("========================================\n");
  printf("  RPMsg 发送速率测试\n");
  printf("  消息数量: %d\n", count);
  printf("  发送间隔: %d ms\n", interval_ms);
  printf("========================================\n\n");

  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  if (rpmsg_init() < 0) {
    return -1;
  }

  printf("[Sender] Starting to send %d messages...\n\n", count);

  start_time = get_time_ms();

  for (i = 0; i < count && running; i++) {
    /* 构造消息: "1,0.5;1,0.5" 格式 */
    snprintf(msg, sizeof(msg), "1,0.5;1,0.5");

    ret = write(rpmsg_fd, msg, strlen(msg) + 1);
    if (ret < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        printf("[Sender] Buffer full at msg #%d, waiting...\n", i);
        usleep(10000); /* 等待10ms后重试 */
        i--;           /* 重试这条 */
        continue;
      }
      perror("[Sender] Write failed");
      failed++;
    } else {
      success++;
    }

    /* 每100条打印进度 */
    if ((i + 1) % 100 == 0) {
      printf("[Sender] Sent %d/%d messages...\n", i + 1, count);
    }

    /* 间隔延时 */
    if (interval_ms > 0) {
      usleep(interval_ms * 1000);
    }
  }

  end_time = get_time_ms();
  elapsed = end_time - start_time;

  printf("\n========================================\n");
  printf("  发送完成统计\n");
  printf("========================================\n");
  printf("  成功: %d 条\n", success);
  printf("  失败: %d 条\n", failed);
  printf("  总耗时: %.2f ms\n", elapsed);
  printf("  平均速率: %.2f msg/s (%.2f Hz)\n", success / (elapsed / 1000.0),
         success / (elapsed / 1000.0));
  printf("  平均间隔: %.3f ms\n", elapsed / success);
  printf("========================================\n");

  rpmsg_cleanup();
  return 0;
}
