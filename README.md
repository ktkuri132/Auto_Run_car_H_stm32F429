# STM32F429 自平衡小车项目

## 项目简介

这是一个基于STM32F429微控制器的自平衡小车项目，使用FreeRTOS实时操作系统，集成了IMU传感器、电机控制、PID算法等多个模块，实现小车的自平衡功能。

## 技术栈

### 硬件平台
- **主控芯片**: STM32F429
- **传感器**: MPU6050 (六轴陀螺仪加速度计)
- **电机驱动**: TB6612 双路电机驱动模块
- **编码器**: 两路正交编码器用于速度反馈
- **通信**: USART1串口调试，UART4扩展通信

### 软件框架
- **RTOS**: FreeRTOS v10.x
- **标准库**: STM32 标准库
- **开发环境**: 支持C++17标准

### 核心中间件
- **Shell系统**: 自定义命令行界面，支持参数配置和调试
- **环境变量系统**: 动态参数管理
- **滤波算法**: 卡尔曼滤波器 + 低通滤波器
- **控制算法**: PID控制器组合

## 项目结构

```
STM32_Auto_Balance_Car/
├── main.cpp                 # 主程序入口
├── bsp/
│   └── config.h            # 板级支持包配置
├── rtos/                   # FreeRTOS相关
│   ├── FreeRTOS.h
│   └── task.h
├── hw/                     # 硬件驱动层
│   └── inv_mpu.h          # MPU6050驱动
├── Dirver/                 # 设备驱动封装
│   ├── UART.cpp           # 串口驱动
│   ├── PWM.cpp            # PWM驱动
│   ├── Encoding.cpp       # 编码器驱动
│   ├── GPIO.cpp           # GPIO驱动
│   └── TIM.cpp            # 定时器驱动
├── Application/            # 应用层
│   ├── Button.cpp         # 按键应用
│   └── LED.cpp            # LED控制
├── Control/                # 控制算法
│   ├── Upright_Control.cpp # 直立控制
│   ├── Speed_Control.cpp   # 速度控制
│   ├── Turn_Control.cpp    # 转向控制
│   └── Filter/             # 滤波算法
│       ├── KalmanFilter.cpp
│       └── LowPassFilter.cpp
└── System/                 # 系统功能
    ├── Shell.h            # 命令行系统
    ├── Serial.h           # 串口通信
    └── env.h              # 环境变量管理
```

## 核心设计思路

### 1. 三环PID控制策略

项目采用经典的三环控制结构：

- **直立环 (Upright_Control)**: 控制小车姿态角度，防止倾倒
- **速度环 (Speed_Control)**: 控制小车前进后退速度
- **转向环 (Turn_Control)**: 控制小车左右转向

### 2. 传感器数据处理

```cpp
// 姿态解算流程
mpu_dmp_get_data(&pitch, &roll, &yaw);  // 获取DMP解算数据
kalman.Update(lowpass.update(data));     // 卡尔曼+低通滤波
```

### 3. 实时任务调度

使用FreeRTOS进行多任务管理：

- **Main_Thread**: 主控制逻辑，数据采集和Shell处理
- **TIM3_IRQHandler_CallBack**: 高频率控制环（中断触发）
- **Debug_log**: 调试信息输出
- **USART1_IRQHandler_CallBack**: 串口数据处理

### 4. 硬件抽象层设计

采用面向对象的设计模式，将硬件操作封装成类：

```cpp
Dirver::UART usart1(USART1, GPIOA, GPIO_Pin_9 | GPIO_Pin_10, 115200);
Dirver::PWM Motor(GPIOA, TIM2, GPIO_Pin_0 | GPIO_Pin_1, 9000, 1);
Application::LED led(GPIOC, GPIO_Pin_4 | GPIO_Pin_5);
```

## 控制算法详解

### 直立控制 (Upright Control)
- **输入**: 当前pitch角度、角速度
- **输出**: 电机PWM值
- **原理**: 通过PD控制维持小车直立状态

### 速度控制 (Speed Control)  
- **输入**: 编码器反馈的实际速度
- **输出**: 期望的倾斜角度
- **原理**: PI控制器，将速度误差转换为姿态角度调整

### 转向控制 (Turn Control)
- **输入**: 期望转向速度（来自遥控或自主导航）
- **输出**: 左右轮差速值
- **原理**: PD控制实现差速转向

## 编译和使用

### 环境要求
- arm-none-eabi-gcc 13.10
- CMAKE 2.29
- ST-Link调试器或CMSIS-DAP

### 调试使用
1. 连接串口调试工具（115200波特率）
2. 使用Shell命令进行参数调试
3. 观察Debug输出的控制参数

## 参数调试

通过串口Shell系统可以实时调整PID参数：

```bash
# 查看当前参数
get pid_kp
get pid_ki
get pid_kd

# 设置参数
set pid_kp 10.5
set pid_ki 0.8
```

## 学习建议

### 初学者路径
1. **理解硬件连接**: 先熟悉电路原理图和引脚定义
2. **学习FreeRTOS**: 掌握任务创建、中断处理、任务同步
3. **掌握传感器使用**: 理解MPU6050的DMP输出和数据滤波
4. **学习PID控制**: 从单环PID开始，逐步理解三环控制策略

### 进阶学习
1. **优化控制算法**: 尝试实现更先进的控制算法如LQR、模糊控制
2. **增加功能模块**: 添加蓝牙遥控、视觉识别等功能
3. **系统级优化**: 优化任务调度、内存使用、功耗管理

## 常见问题

### Q: 小车无法平衡，一直倾倒
A: 检查IMU安装方向，调整直立环PID参数，确保传感器数据正确

### Q: 小车震荡严重
A: 降低PID的D参数，增加滤波器的滤波强度，检查机械结构是否稳固

### Q: 串口无法正常通信
A: 确认波特率设置，检查串口引脚连接，验证Shell协议格式

## 贡献指南

欢迎提交Issue和Pull Request来改进项目：

1. Fork项目
2. 创建特性分支
3. 提交代码
4. 发起Pull Request

## 许可证

本项目采用MIT许可证，详见LICENSE文件。

## 联系方式

- 作者: Kilo
- 创建日期: 2025-06-02
- 项目地址: [[GitHub链接](https://github.com/ktkuri132/Auto_Run_car_H_stm32F429.git)]

---

**注意**: 本项目仅供学习交流使用，请根据实际硬件情况调整相关参数。
