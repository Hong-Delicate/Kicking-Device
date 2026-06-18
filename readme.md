# Kicking Device — 自由泳打腿检测系统

## 1. 项目概述

本设备用于自由泳打腿动作的实时检测与反馈。系统通过 IMU 传感器采集运动员腿部运动数据，经嵌入式算法分析后，以声、光、振动三种方式实时反馈打腿质量。

| 指标 | 参数 |
|------|------|
| 检测对象 | 自由泳打腿（flutter kick） |
| 检测维度 | 幅度 / 姿态 / 翻转 / 水平位移 / 力度 |
| 反馈方式 | LED（色光） + 蜂鸣器（长短鸣） + 马达（振动） |
| 工作电压 | 3.7 V DC（锂电池） |

---

## 2. 硬件架构

### 2.1 核心器件

| 序号 | 器件 | 型号 / 规格 | 功能 |
|------|------|------------|------|
| 1 | MCU | STM32F103ZET6 (Cortex-M3, 72 MHz) | 主控 |
| 2 | IMU | HiPNUC CH010 (9轴) | 姿态 & 加速度采集 |
| 3 | LED | WS2812 × 1 | 视觉反馈（红 / 绿 / 黄） |
| 4 | 蜂鸣器 | 有源电磁式, 3.3 V 驱动 | 听觉反馈 |
| 5 | 马达 | 微型振动电机 | 触觉反馈 |
| 6 | 按键 | 轻触开关 | 复位 / 模式切换 |
| 7 | 电源 | 3.7 V 锂电池 | 系统供电 |

### 2.2 引脚分配

| 外设 | 引脚 | 功能模式 | 备注 |
|------|------|---------|------|
| IMU TX | PA2 | AF_PP (USART2_TX) | — |
| IMU RX | PA3 | Input (USART2_RX) | 115200-8-N-1 |
| WS2812 | PD14 | AF_OD (TIM4_CH3) | 800 kHz PWM + DMA |
| 蜂鸣器 | PD13 | GPIO Output PP | 有源蜂鸣器，高电平触发 |
| 马达 | PD12 | GPIO Output PP | 高电平触发 |
| 按键 | PB14 | GPIO Input (Pull-Up) | 低电平有效 |
| 预留 | PB7 | GPIO Output PP | 未使用 |
| USART1 | PA9 / PA10 | AF_PP / Input | 上位机通讯（预留） |

### 2.3 时钟 & 总线

| 总线 | 时钟源 | 频率 | 备注 |
|------|--------|------|------|
| SYSCLK | HSE (8 MHz) × PLL9 | 72 MHz | — |
| AHB | SYSCLK ÷ 1 | 72 MHz | — |
| APB1 | SYSCLK ÷ 2 | 36 MHz | TIM4 位于此总线（定时器时钟 = 72 MHz） |
| APB2 | SYSCLK ÷ 1 | 72 MHz | USART1/2 位于此总线 |

---

## 3. 软件架构

### 3.1 代码分层

```
┌──────────────────────────────────────────┐
│  Algorithm / kick_detect.c               │  ← 打腿检测状态机
├──────────────────────────────────────────┤
│  BSP /                                   │  ← 板级驱动层
│    bsp_beep.c    蜂鸣器 (GPIO)            │
│    bsp_key.c     按键 (GPIO + 消抖)       │
│    bsp_motor.c   马达 (GPIO)              │
│    bsp_ws2812.c  LED (TIM4 PWM + DMA)    │
│    bsp_imu.c     IMU (USART2 + HiPNUC)   │
├──────────────────────────────────────────┤
│  Components / hipnuc_dec.c               │  ← HiPNUC 协议解码器 (SDK)
├──────────────────────────────────────────┤
│  Core / (CubeMX HAL)                     │  ← STM32 HAL + 外设初始化
└──────────────────────────────────────────┘
```

### 3.2 中断向量表

| 优先级 | 中断源 | 处理函数 | 用途 |
|--------|--------|---------|------|
| 0 | DMA1_Channel5 | `DMA1_Channel5_IRQHandler` | WS2812 DMA 传输完成 |
| 0 | USART2 | `USART2_IRQHandler` | IMU 数据接收 |
| 15 | SysTick | `SysTick_Handler` | HAL 时基 (1 ms) |

### 3.3 TIM4 时分复用

TIM4 外设由 WS2812 与蜂鸣器共享，通过分时复用来解决两设备所需频率不同的问题：

| 模式 | 通道 | 频率 | 占空比 | 说明 |
|------|------|------|--------|------|
| LED (常规) | CH3 | 800 kHz | 动态 (DMA) | WS2812 位编码输出 |
| 蜂鸣 (触发时) | CH2 | 2 kHz | 50% | 有源蜂鸣器改为 GPIO，无需复用 |

> **当前实现**：蜂鸣器已改为 GPIO 直驱（PD13），TIM4 由 WS2812 独占。若后续使用无源蜂鸣器需要 PWM，可通过 `BSP_WS2812_RestoreTimConfig()` 恢复 LED 模式。

---

## 4. 算法设计

### 4.1 状态机

```
                    ┌──────────┐
                    │ STATE    │
          ┌────────→│ _INIT    │←──────────────┐
          │         └────┬─────┘               │
          │              │ 姿态水平?            │
          │         ┌────┴─────┐               │
          │         │ 否       │ 是            │
          │         ▼          ▼               │
          │    ┌────────┐ ┌──────────┐         │
          │    │ 红灯   │ │ 绿灯     │         │
          │    │ 长鸣   │ │ 短鸣     │         │
          │    │ 振动   │ │          │         │
          │    └────┬───┘ └────┬─────┘         │
          │         │          │               │
          │         └──────────┤               │
          │                    ▼               │
          │              ┌──────────┐          │
          │              │ STATE    │          │
          │              │ _DETECT  │          │
          │              └────┬─────┘          │
          │                   │ 打腿检测       │
          │         ┌─────────┼─────────┐      │
          │         ▼         ▼         ▼      │
          │    ┌────────┐┌────────┐┌────────┐  │
          │    │ 红灯   ││ 绿灯   ││ 黄灯   │  │
          │    │ <30cm  ││ ≥30cm  ││ ≥30cm  │  │
          │    │        ││ 标准   ││ 非标准 │  │
          │    └────────┘│ 短鸣   ││ 长鸣   │  │
          │              └────────┘│ 振动   │  │
          │                       └────────┘  │
          │ 超时 (5 s) / 按键                   │
          └───────────────────────────────────┘
```

### 4.2 检测参数（可调宏）

| 宏定义 | 默认值 | 含义 |
|--------|--------|------|
| `POSTURE_HORIZONTAL_THRESHOLD_DEG` | 15° | 水平姿态判定阈值 (pitch / roll) |
| `KICK_AMPLITUDE_THRESHOLD_CM` | 30 cm | 打腿幅度合格线 |
| `FLIP_ROLL_THRESHOLD_DEG` | 30° | 翻转判定阈值 |
| `HORIZONTAL_DISPLACEMENT_THRESHOLD_M` | 0.5 m | 水平漂移判定阈值 |
| `KICK_FORCE_WEAK_THRESHOLD` | 10 m/s² | 力度弱判定线 |
| `KICK_FORCE_STRONG_THRESHOLD` | 25 m/s² | 力度强判定线 |
| `SAMPLE_WINDOW_MS` | 2000 ms | 打腿检测采样窗口 |
| `KICK_DETECT_TIMEOUT_MS` | 5000 ms | 无动作超时返回 INIT |
| `KICK_EVAL_COOLDOWN_MS` | 500 ms | 相邻判定冷却时间 |

### 4.3 位移估算模型

打腿可近似为正弦运动，通过加速度峰峰值和半周期估算位移：

```
D = (a_pp × T²) / π² × 100

其中:  a_pp = 垂直方向加速度峰峰值 (m/s²)
       T    = 半周期 (s)
       D    = 峰峰位移 (cm)
```

---

## 5. 反馈信号定义

| 场景 | LED | 蜂鸣器 | 马达 |
|------|-----|--------|------|
| 初始姿态不水平 | 红灯 | 长鸣 500 ms | 振动 200 ms |
| 初始姿态水平 | 绿灯 | 短鸣 200 ms | — |
| 打腿幅度 < 30 cm | 红灯 | — | — |
| 打腿幅度 ≥ 30 cm, 标准 | 绿灯 | 短鸣 200 ms | — |
| 打腿幅度 ≥ 30 cm, 非标准 | 黄灯 | 长鸣 500 ms | 振动 200 ms |
| 无动作超时 | 灭 | — | — |
| 按键复位 | 灭 | — | — |

---

## 6. 构建 & 烧录

### 6.1 工具链

| 工具 | 版本 |
|------|------|
| IDE | Keil MDK-ARM (μVision) |
| HAL 库 | STM32F1xx HAL Driver |
| 编译器 | Arm Compiler 5 / 6 |
| 调试器 | ST-Link / J-Link |

### 6.2 源文件清单

**BSP 层** (`Bsp/`)
- `bsp_beep.c` — 蜂鸣器 GPIO 驱动
- `bsp_key.c` — 按键消抖驱动
- `bsp_motor.c` — 马达 GPIO 驱动
- `bsp_ws2812.c` — WS2812 PWM + DMA 驱动
- `bsp_imu.c` — IMU USART2 + HiPNUC 驱动

**算法层** (`Algorithm/`)
- `kick_detect.c` — 打腿检测状态机

**组件** (`Components/`)
- `hipnuc_dec.c` — HiPNUC 协议解码器

**CubeMX 生成** (`Core/Src/`)
- `main.c` — 主程序入口 & 系统时钟配置
- `gpio.c` / `tim.c` / `usart.c` / `dma.c` — 外设初始化
- `stm32f1xx_it.c` — 中断服务函数

### 6.3 头文件路径

```
.\Bsp
.\Algorithm
.\Components
.\Core\Inc
.\Drivers\CMSIS\Include
.\Drivers\CMSIS\Device\ST\STM32F1xx\Include
.\Drivers\STM32F1xx_HAL_Driver\Inc
```

---

## 7. 变更记录

| 日期 | 版本 | 变更内容 |
|------|------|---------|
| 2026-06-14 | v1.0 | 初始版本 — BSP 驱动 & 打腿检测算法实现 |
