# RT-Thread 差速小车电机控制

基于 RT-Thread 和 Spacemit K3 平台的双电机差速控制系统。

## 功能特性

- ✅ 双电机独立 PWM 调速控制
- ✅ GPIO 控制电机正反转方向
- ✅ 霍尔编码器脉冲计数 (GPIO 中断)
- ✅ 实时转速计算 (RPM)
- ✅ 前馈模型 (速度→占空比映射)
- ✅ PID 控制器
- ✅ MSH 命令行控制接口

## 项目结构

```
rt-diff-motor-control/
├── control_main.c        # 主程序入口，初始化和底盘控制线程
├── include/
│   ├── common.h          # 引脚定义和通用参数
│   ├── encoder.h         # 编码器接口
│   ├── motor_control.h   # 电机控制接口
│   ├── motor_gpio.h      # GPIO 方向控制接口
│   ├── motor_model.h     # 前馈模型接口
│   ├── motor_pwm.h       # PWM 控制接口
│   └── pid.h             # PID 控制器接口
├── src/
│   ├── encoder.c         # 编码器脉冲计数 (GPIO 中断 + 消抖)
│   ├── led_test.c        # LED 测试命令
│   ├── motor_control.c   # 电机控制和 MSH 命令
│   ├── motor_gpio.c      # 电机方向 GPIO 控制
│   ├── motor_model.c     # 速度→占空比前馈模型
│   ├── motor_pwm.c       # PWM 驱动封装
│   └── pid.c             # PID 控制器实现
└── examples/             # 示例代码
```

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
| 编码器 A 相 | GPIO 159 | 中断输入 |

### 编码器参数
- PPR (每转脉冲数): 11
- 减速比: 56

## MSH 命令

### 电机控制
```bash
# 控制电机 (格式: cmd_motor <dir1,duty1>;<dir2,duty2>)
# dir: 0=停止, 1=正转, 2=反转
# duty: 0.0 ~ 1.0

cmd_motor 1,0.5;1,0.5     # 双电机正转 50%
cmd_motor 2,0.3;1,0.7     # 电机1反转30%, 电机2正转70%
cmd_motor 0,0;0,0         # 停止双电机

# 紧急停止
cmd_motor_stop
```

### 调试命令
```bash
# 读取编码器 GPIO 电平
enc_gpio
```

## 系统线程

| 线程名 | 频率 | 功能 |
|--------|------|------|
| encoder | 20Hz | 读取编码器 delta，更新消抖参数 |
| chassis | 30Hz | 计算 RPM，打印状态 |

## 编译

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
