# Arknights-Chassis

麦克纳姆轮智能底盘控制系统，基于 STM32F103ZET6 微控制器。

## 项目概述

这是一个面向 **明日方舟 (Arknights)** 主题的麦轮底盘项目，支持蓝牙遥控、视觉颜色识别和实时运动控制。

## 硬件配置

| 项目 | 参数 |
|------|------|
| 主控芯片 | STM32F103ZET6 |
| 轮子直径 | 127 mm |
| 编码器 PPR | 3692 (4倍频) |
| 轮距 | 365.5 mm |
| 最大线速度 | 4000 mm/s |
| 最大旋转速度 | 2000 mm/s (等效) |

## 功能特性

### 底盘控制
- **麦克纳姆轮布局**：4轮全向移动，支持原地旋转
- **运动学解算**：逆运动学 (IK) 和正运动学 (FK)
- **速度限幅**：单轮超速时自动等比缩放，保证运动方向

### 遥控系统
- **蓝牙串口协议**：通过手机/手柄发送速度指令
- **协议格式**：`joystick,<left_x>,<left_y>,<right_x>,<right_y>`
- **DBUS 接口预留**：兼容未来 RC 遥控器扩展

### 视觉识别
- **树莓派相机**：接收颜色识别结果
- **颜色 LED 反馈**：
  - 红色 LED → PB0
  - 蓝色 LED → PB2
  - 黄色 LED → PF12
- **双颜色支持**：可同时识别两种颜色

### 通信
- **蓝牙串口**：调试信息、状态上报
- **OLED 显示**：实时显示底盘状态
- **USB 虚拟串口**：固件调试

## 软件架构

```
┌─────────────────────────────────────────────────────────┐
│                    FreeRTOS 任务                         │
├─────────────┬─────────────┬─────────────┬───────────────┤
│  Motor_Task │ Camera_Task │ BT_Task     │ PID_Task      │
│  (电机控制)  │ (相机处理)   │ (蓝牙通信)   │ (PID计算)      │
└─────────────┴─────────────┴─────────────┴───────────────┘
                          │
┌─────────────────────────────────────────────────────────┐
│                    驱动层 / HAL                          │
├─────────────┬─────────────┬─────────────┬───────────────┤
│ TIM (PWM)   │ UART (BT)   │ DMA         │ GPIO (LED)    │
└─────────────┴─────────────┴─────────────┴───────────────┘
```

## 目录结构

```
Arknights-chassis/
├── Core/
│   ├── Inc/           # 头文件
│   │   ├── config.h       # 全局配置 (机械参数、PID、速度限制)
│   │   ├── kinematics.h   # 运动学解算
│   │   ├── motor.h        # 电机驱动
│   │   ├── remote.h       # 遥控器
│   │   └── camera.h       # 相机数据处理
│   └── Src/            # 源文件
│       ├── kinematics.c    # 麦轮 IK/FK 解算
│       ├── motor.c        # 电机 PWM 控制
│       ├── remote.c       # 蓝牙遥控解析
│       ├── camera.c       # 颜色解析 + LED 控制
│       └── bluetooth.c    # 蓝牙通信
├── Drivers/            # STM32 HAL 驱动
├── Middlewares/        # FreeRTOS
├── Arknights-chassis.ioc  # STM32CubeMX 工程文件
└── README.md
```

## 蓝牙协议

### 摇杆数据
```
joystick,<left_x>,<left_y>,<right_x>,<right_y>
```
- `left_x/left_y`: 左摇杆 (±100)
- `right_x/right_y`: 右摇杆 (±100)

### 颜色数据 (相机 → STM32)
```
<color1>,<color2>
```
- 支持颜色: `red`, `blue`, `yellow`
- 大小写不敏感

### 按钮数据
```
button,<value>,
```

### 数字数据
```
number,<value>,
```

## 配置说明

### 速度限制 (`config.h`)
```c
#define MAX_LINEAR_SPEED_MM_S   4000.0f    // 最大线速度
#define MAX_ANGULAR_SPEED_MM_S  2000.0f    // 最大旋转等效速度
#define MAX_WHEEL_SPEED_MM_S    6000.0f    // 单轮速度上限
```

### 编码器标定
```c
#define ENCODER_STANDARD_PPR   3692        // 编码器每转脉冲数
```

## 开发环境

- **IDE**: STM32CubeMX + Keil MDK / STM32CubeIDE
- **编译器**: ARM GCC / ARMCC
- **框架**: STM32 HAL + FreeRTOS
- **构建**: CMake

## 快速开始

1. 使用 STM32CubeMX 打开 `.ioc` 文件
2. 生成代码
3. 编译并烧录
4. 通过蓝牙连接并发送控制指令

## 联系方式

- GitHub: [zzx-sign/Arknights-chassis](https://github.com/zzx-sign/Arknights-chassis)

## 许可证

MIT License
