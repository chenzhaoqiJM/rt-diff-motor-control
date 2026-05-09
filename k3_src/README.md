# K3 无 ROS2 底盘控制程序

`k3_chassis_control.c` 是参考 `jdbot_k3_esos_control` 中 `rpmsg_node` 实现的 Linux 端独立程序，不依赖 ROS2，仅使用 POSIX API 与小核 ESOS/RT-Thread 电机控制服务通信。

## 功能

- 创建 RPMsg endpoint：`rpmsg:motor_ctrl`
- 向小核发送底盘速度换算后的双电机转速指令
- 启动时可发送 `CFG,ratio,ff,kp,ki,kd,feedback_enable` 参数
- 接收小核反馈：`dir1,speed1_mrs;dir2,speed2_mrs`
- 根据反馈计算左右轮线速度并积分简易里程计
- 支持命令行初始速度和交互模式

## 协议参数

默认参数与参考 ROS2 节点保持一致：

| 参数 | 默认值 |
| --- | --- |
| 控制设备 | `/dev/rpmsg_ctrl0` |
| 数据设备 | `/dev/rpmsg0` |
| 服务名 | `rpmsg:motor_ctrl` |
| Linux 地址 | `1003` |
| RCPU 地址 | `1002` |
| 发送频率 | `20Hz` |
| 命令超时 | `0.4s` |
| 轮半径 | `0.0335m` |
| 轮距 | `0.183m` |
| 减速比 | `56.0` |
| 前馈系数 | `0.3` |
| PID | `kp=0.05, ki=0.2, kd=0.01` |

## 编译命令

在目标板 Linux 环境或对应交叉编译环境中执行：

```bash
cd /media/chenzhaoqi/data/tmp/whls/esos/bsp/spacemit/applications/rt-diff-motor-control/k3_src
gcc -Wall -Wextra -O2 -o k3_chassis_control k3_chassis_control.c -lpthread -lm
```

如需交叉编译，将 `gcc` 替换为目标工具链，例如：

```bash
cd /media/chenzhaoqi/data/tmp/whls/esos/bsp/spacemit/applications/rt-diff-motor-control/k3_src
riscv64-unknown-linux-gnu-gcc -Wall -Wextra -O2 -o k3_chassis_control k3_chassis_control.c -lpthread -lm
```

## 运行示例

### 交互模式

```bash
sudo ./k3_chassis_control -i
```

交互命令：

```text
cmd <v_mps> <w_radps>    设置线速度和角速度
stop                     停止底盘
cfg <ratio> <ff> <kp> <ki> <kd> <fb0_or_1>
odom                     打印当前里程计
quit                     停止并退出
```

示例：

```text
cmd 0.10 0.00
cmd 0.00 0.50
stop
odom
quit
```

### 非交互模式

启动后发送一次初始速度；超过命令超时时间后程序会自动发送停止指令。

```bash
sudo ./k3_chassis_control -v 0.10 -w 0.00
```

### 常用参数

```bash
sudo ./k3_chassis_control -i -r 0.0335 -b 0.183 -s 20 -t 0.4
```

- `-r`：轮半径，单位 m
- `-b`：轮距，单位 m
- `-s`：RPMsg 速度指令发送频率，单位 Hz
- `-t`：命令超时时间，单位 s
- `--no-cfg`：启动时不发送 CFG
- `--no-feedback`：通过 CFG 关闭小核反馈

## 注意事项

1. 小核侧需已运行 `rt-diff-motor-control`，并创建 `rpmsg:motor_ctrl` 服务。
2. Linux 侧需要存在 `/dev/rpmsg_ctrl0` 和 `/dev/rpmsg0`。
3. 速度单位：
   - 程序输入底盘速度：线速度 `m/s`，角速度 `rad/s`
   - 发送给小核的轮速：`r/s`
   - 小核反馈轮速：`mr/s`
4. 如普通用户无设备访问权限，请使用 `sudo` 或调整设备节点权限。
