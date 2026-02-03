# RT-Thread 差速小车电机控制

基于 RT-Thread 和 Spacemit K3 平台的双电机差速控制系统，支持里程计计算和大小核通信。

## 功能特性

- ✅ 双电机独立 PWM 调速控制
- ✅ GPIO 控制电机正反转方向
- ✅ 霍尔编码器脉冲计数 (GPIO 中断)
- ✅ 实时转速计算 (RPM)
- ✅ 前馈模型 (速度→占空比映射)
- ✅ PID 闭环控制器
- ✅ **里程计计算 (ROS2 标准坐标系)**
- ✅ **RPMsg 大小核异步通信**
- ✅ MSH 命令行控制接口

## 项目结构

```
rt-diff-motor-control/
├── control_main.c          # 主程序入口，初始化和底盘控制线程
├── include/
│   ├── common.h            # 引脚定义和通用参数
│   ├── encoder.h           # 编码器接口
│   ├── motor_control.h     # 电机控制接口
│   ├── motor_gpio.h        # GPIO 方向控制接口
│   ├── motor_model.h       # 前馈模型接口
│   ├── motor_pwm.h         # PWM 控制接口
│   ├── odometry.h          # 里程计接口
│   ├── pid.h               # PID 控制器接口
│   └── rpmsg_motor.h       # RPMsg 电机控制接口
├── src/
│   ├── encoder.c           # 编码器脉冲计数 (GPIO 中断 + 消抖)
│   ├── led_test.c          # LED 测试命令
│   ├── motor_control.c     # 电机控制和 MSH 命令
│   ├── motor_gpio.c        # 电机方向 GPIO 控制
│   ├── motor_model.c       # 速度→占空比前馈模型
│   ├── motor_pwm.c         # PWM 驱动封装
│   ├── odometry.c          # 里程计计算 (差速驱动运动学)
│   ├── pid.c               # PID 控制器实现
│   ├── rpmsg_motor.c       # RPMsg 电机控制服务
│   └── rpmsg_test.c        # RPMsg 测试程序
├── k3_src/
│   └── rpmsg_motor_async.c # Linux 端 RPMsg 客户端
└── examples/               # 示例代码
```

## 通信协议 (RPMsg)

大核 (Linux) 与小核 (RCPU) 之间通过 RPMsg 异步通信。

### 命令格式

| 方向 | 命令 | 格式 | 示例 |
|------|------|------|------|
| 大核→小核 | CFG | `CFG:wheel_radius=R;wheel_base=L;gear_ratio=G;ppr=P` | `CFG:wheel_radius=0.05;wheel_base=0.2;gear_ratio=56;ppr=11` |
| 大核→小核 | VEL | `VEL:v,w` (线速度 m/s, 角速度 rad/s) | `VEL:0.5,0.2` |
| 大核→小核 | RST | `RST:` (重置里程计) | `RST:` |
| 小核→大核 | ODM | `ODM:x,y,theta,v,w,timestamp` | `ODM:1.234,0.567,0.785,0.50,0.20,12345678` |

### 坐标系 (ROS2 标准)

- **X+**: 前进方向
- **Y+**: 左侧
- **θ+**: 逆时针

### 旧协议 (兼容)

| 方向 | 格式 | 示例 |
|------|------|------|
| 大核→小核 | `dir1,speed1;dir2,speed2` | `1,2.0;1,2.0` |
| 小核→大核 | `dir1,speed1_mrs;dir2,speed2_mrs` | `1,2000;1,2000` |

## 硬件配置

### 电机 1
| 功能 | 引脚 | 说明 |
|------|------|------|
| 方向控制 0 | GPIO 125 | H 桥控制 |
| 方向控制 1 | GPIO 127 | H 桥控制 |
| PWM | rpwm9 (GPIO 112) | 10KHz PWM |
| 编码器 A 相 | GPIO 158 | 中断输入 |

### 电机 2
| 功能 | 引脚 | 说明 |
|------|------|------|
| 方向控制 0 | GPIO 71 | H 桥控制 |
| 方向控制 1 | GPIO 61 | H 桥控制 |
| PWM | rpwm8 (GPIO 111) | 10KHz PWM |
| 编码器 A 相 | GPIO 163 | 中断输入 |

### 编码器参数
- PPR (每转脉冲数): 11
- 减速比: 56

## Linux 端使用

### 编译

```bash
cd bsp/spacemit/applications/rt-diff-motor-control/k3_src
gcc -o rpmsg_motor_async rpmsg_motor_async.c -lpthread -lm
```

### 运行

```bash
./rpmsg_motor_async

# 配置机器人参数 (轮径, 轮距, 减速比, PPR)
> cfg 0.05 0.2 56 11

# 发送速度指令 (线速度 m/s, 角速度 rad/s)
> vel 0.5 0.2

# 查看里程计
> odom

# 停止
> vel 0 0

# 重置里程计
> rst
```

## MSH 命令 (小核)

### 速度控制
```bash
# 格式: cmd_speed <dir1,speed1>;<dir2,speed2>
# dir: 0=停止, 1=正转, 2=反转
# speed: 转/秒 (r/s)

cmd_speed 1,2.0;1,2.0     # 双电机正转 2.0 r/s
cmd_speed 0,0;0,0         # 停止双电机
cmd_chassis_stop          # 紧急停止
```

### 里程计
```bash
cmd_odom_info             # 查看里程计状态
cmd_odom_reset            # 重置里程计
```

### RPMsg 反馈控制
```bash
cmd_rpmsg_feedback on     # 启用反馈
cmd_rpmsg_feedback off    # 禁用反馈
cmd_rpmsg_feedback 50     # 设置反馈间隔 (ms)
```

### 调试命令
```bash
enc_info                  # 读取编码器 delta 和速度
```

## 系统线程

| 线程名 | 频率 | 功能 |
|--------|------|------|
| enc1/enc2 | 50Hz | 读取编码器 delta，计算速度 |
| chassis | 50Hz | PID 控制，里程计更新 |
| rpmsg_fb | 50Hz | 发送状态/里程计反馈 |

## 编译 (小核)

项目源文件在 `bsp/spacemit/applications/SConscript` 中注册：

```python
src = [
    'rt-diff-motor-control/control_main.c',
    'rt-diff-motor-control/src/motor_pwm.c',
    'rt-diff-motor-control/src/motor_gpio.c',
    'rt-diff-motor-control/src/encoder.c',
    'rt-diff-motor-control/src/motor_control.c',
    'rt-diff-motor-control/src/led_test.c',
    'rt-diff-motor-control/src/pid.c',
    'rt-diff-motor-control/src/motor_model.c',
    'rt-diff-motor-control/src/rpmsg_test.c',
    'rt-diff-motor-control/src/rpmsg_motor.c',
    'rt-diff-motor-control/src/odometry.c',
]
```

## 前馈模型

`motor_model.c` 中实现了基于线性拟合的前馈模型：

```
duty = k * speed + b
```

正转和反转使用不同的 k、b 参数，通过实际测量标定得到。

## License

MIT
