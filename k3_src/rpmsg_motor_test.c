/*
 * Copyright (c) 2022-2025, Spacemit
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Linux 端 RPMsg 测试程序
 * 用于与 RCPU 端 rpmsg_test.c 进行字符串通信测试
 *
 * 编译: gcc -o rpmsg_motor_test rpmsg_motor_test.c
 * 运行: ./rpmsg_motor_test [消息内容]
 */

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define RPMSG_NAME_SIZE 32
#define RPMSG_ADDR_ANY 0xFFFFFFFF

/* RPMsg 服务配置 - 必须与 RCPU 端匹配 */
#define RPMSG_SERVICE_NAME "rpmsg:motor_test"
#define RPMSG_LOCAL_ADDR 1001  /* Linux 端地址 */
#define RPMSG_REMOTE_ADDR 1000 /* RCPU 端地址 */

/**
 * struct rpmsg_endpoint_info - 端点信息结构
 * @name: 服务名称
 * @src: 本地地址
 * @dst: 目标地址
 */
struct rpmsg_endpoint_info {
  char name[32];
  unsigned int src;
  unsigned int dst;
};

/* RPMsg ioctl 命令定义 */
#define RPMSG_CREATE_EPT_IOCTL _IOW(0xb5, 0x1, struct rpmsg_endpoint_info)
#define RPMSG_DESTROY_EPT_IOCTL _IO(0xb5, 0x2)

/**
 * @brief 发送消息并等待响应
 */
static int send_and_receive(int fd, const char *msg) {
  char recv_buf[256];
  int ret;

  printf("[Linux] Sending: %s\n", msg);

  ret = write(fd, msg, strlen(msg) + 1);
  if (ret < 0) {
    perror("[Linux] Write failed");
    return -1;
  }

  memset(recv_buf, 0, sizeof(recv_buf));
  ret = read(fd, recv_buf, sizeof(recv_buf));
  if (ret < 0) {
    perror("[Linux] Read failed");
    return -1;
  }

  printf("[Linux] Received: %s\n", recv_buf);
  return 0;
}

int main(int argc, char *argv[]) {
  int ret;
  int rpmsg_ctrl_fd, rpmsg_fd;
  struct rpmsg_endpoint_info epinfo;
  const char *message = "Hello from Linux";

  /* 使用命令行参数作为消息内容 */
  if (argc > 1) {
    message = argv[1];
  }

  printf("[Linux] RPMsg Motor Control Test\n");
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

  /* 5. 发送消息并接收响应 */
  printf("\n--- Starting communication test ---\n");
  send_and_receive(rpmsg_fd, message);

  /* 交互模式: 持续发送接收 */
  printf("\n--- Interactive mode (type 'quit' to exit) ---\n");
  while (1) {
    char input[256];
    printf("> ");
    fflush(stdout);

    if (fgets(input, sizeof(input), stdin) == NULL) {
      break;
    }

    /* 移除换行符 */
    input[strcspn(input, "\n")] = '\0';

    if (strcmp(input, "quit") == 0 || strcmp(input, "exit") == 0) {
      break;
    }

    if (strlen(input) > 0) {
      send_and_receive(rpmsg_fd, input);
    }
  }

  printf("[Linux] Closing...\n");
  close(rpmsg_fd);
  close(rpmsg_ctrl_fd);

  return 0;
}
