/*
 * Copyright (c) 2022-2025, Spacemit
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <openamp/remoteproc.h>
#include <openamp/rpmsg.h>
#include <openamp/rpmsg_virtio.h>
#include <openamp/virtio.h>
#include <rtdef.h>
#include <rtthread.h>
#include <string.h>


#define RPMSG_TEST_SERVICE_NAME "rpmsg:motor_test"
#define RPMSG_TEST_ADDR_SRC 1000
#define RPMSG_TEST_ADDR_DST 1001

extern struct rpmsg_device *rpdev;

struct rpmsg_test_ctx {
  char *service_name;
  struct rpmsg_endpoint endp;
};

/**
 * @brief RPMsg 端点回调函数
 *
 * 当接收到来自 Linux 端的消息时调用此函数
 * 接收一个字符串，处理后返回一个响应字符串
 */
static int rpmsg_test_endpoint_cb(struct rpmsg_endpoint *ept, void *data,
                                  size_t len, uint32_t src, void *priv) {
  char response[256];
  int response_len;
  char *recv_str = (char *)data;

  /* 打印接收到的消息 */
  rt_kprintf("[rpmsg_test] Received: %s (len=%d, src=%d)\n", recv_str, len,
             src);

  /* 构造响应字符串 */
  response_len =
      rt_snprintf(response, sizeof(response),
                  "[RCPU] Received: \"%s\", ACK from motor control", recv_str);

  /* 发送响应 */
  rpmsg_send(ept, response, response_len + 1);

  rt_kprintf("[rpmsg_test] Sent: %s\n", response);

  return 0;
}

/**
 * @brief 服务解绑回调
 */
static void rpmsg_test_service_unbind(struct rpmsg_endpoint *ept) {
  rt_kprintf("[rpmsg_test] Service unbound\n");
}

/**
 * @brief RPMsg 测试线程入口
 */
static void rpmsg_test_thread_entry(void *parameter) {
  int ret;
  struct rpmsg_test_ctx *ctx = (struct rpmsg_test_ctx *)parameter;

  /* 等待 rpdev 初始化完成 */
  while (rpdev == RT_NULL) {
    rt_thread_delay(10);
  }

  rt_kprintf("[rpmsg_test] rpdev ready, creating endpoint...\n");

  ctx->service_name = RPMSG_TEST_SERVICE_NAME;

  /* 创建 RPMsg 端点 */
  ret = rpmsg_create_ept(&ctx->endp, rpdev, ctx->service_name,
                         RPMSG_TEST_ADDR_SRC, RPMSG_TEST_ADDR_DST,
                         rpmsg_test_endpoint_cb, rpmsg_test_service_unbind);
  if (ret) {
    rt_kprintf("[rpmsg_test] Create endpoint failed, ret=%d\n", ret);
    rt_free(ctx);
    return;
  }

  rt_kprintf("[rpmsg_test] Endpoint created: %s (src=%d, dst=%d)\n",
             ctx->service_name, RPMSG_TEST_ADDR_SRC, RPMSG_TEST_ADDR_DST);
  rt_kprintf("[rpmsg_test] Waiting for messages from Linux...\n");
}

/**
 * @brief 启动 RPMsg 测试服务
 *
 * 在 MSH 中输入 rpmsg_test 即可启动服务
 */
static int rpmsg_test(void) {
  rt_thread_t tid;
  struct rpmsg_test_ctx *ctx;

  ctx = rt_calloc(1, sizeof(struct rpmsg_test_ctx));
  if (!ctx) {
    rt_kprintf("[rpmsg_test] No memory for context\n");
    return -RT_ENOMEM;
  }

  tid = rt_thread_create("rpmsg_test", rpmsg_test_thread_entry, (void *)ctx,
                         4096, RT_THREAD_PRIORITY_MAX / 3, 20);
  if (!tid) {
    rt_kprintf("[rpmsg_test] Failed to create thread\n");
    rt_free(ctx);
    return -RT_EINVAL;
  }

  rt_thread_startup(tid);
  rt_kprintf("[rpmsg_test] Service starting...\n");

  return 0;
}

#include <finsh.h>
MSH_CMD_EXPORT(rpmsg_test, RPMsg string echo test for motor control);

