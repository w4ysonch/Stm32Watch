# Stm32Watch / STM32智能手表

> 基于 STM32F103 的 FreeRTOS 多任务智能手表。支持计步、温湿度/气压环境监测、日历、时钟、手电筒及互动小游戏，配备 128x64 OLED 显示屏。

**[English →](README_EN.md)**

---

## 功能

- **时钟** — 实时时钟与日期显示
- **日历** — 月历视图
- **计步器** — 基于 MPU6050 DMP（数字运动处理器）的计步功能
- **温湿度计** — 通过 DHT11 测量温度与湿度
- **气压计/海拔计** — 通过 BMP280 测量大气压与海拔
- **手电筒** — 白屏手电
- **木鱼** — 敲击互动游戏
- **设置** — 系统设置菜单
- **菜单系统** — 图标式应用启动器，带蜂鸣器反馈

## 硬件

| 组件 | 型号 | 接口 |
|------|------|------|
| 主控 | STM32F103xB | — |
| 显示屏 | 128×64 OLED (SSD1306) | I2C |
| 惯性测量单元 | MPU6050 (6轴) | I2C |
| 气压传感器 | BMP280 | I2C |
| 温湿度传感器 | DHT11 | GPIO (单总线) |
| 蜂鸣器 | 无源蜂鸣器 | PWM (TIM) |

## 软件架构

- **RTOS**: FreeRTOS + CMSIS-RTOS v2 API
- **HAL**: STM32F1xx HAL Library
- **中间件**: CMSIS Core, STM32CubeMX 生成驱动
- **多任务设计**: 每个应用作为独立 FreeRTOS 任务运行，通过 `vTaskSuspend` / `vTaskResume` 实现菜单切换调度
- **任务间通信**: FreeRTOS 队列处理按键事件，任务通知用于任务间通信

```
任务列表:
  ShowMenuTask      — 主菜单
  ShowClockTask     — 时钟表盘
  ShowCalendarTask  — 月历
  ShowStepTask      — 计步显示
  StepCountTask     — 后台计步检测 (MPU6050 DMP)
  ShowDHT11Task     — 温湿度显示
  ShowBMP280Task    — 气压海拔显示
  ShowFlashLightTask— 手电筒
  ShowTimeTask      — 时间显示
  ShowSettingTask   — 系统设置
  ShowWoodenFishTask— 木鱼游戏
```

## 项目结构

```
Stm32Watch/
├── STM32WATCH1.3/
│   ├── Core/                  # MCU 配置 (CubeMX 生成)
│   │   ├── Inc/               #   头文件
│   │   └── Src/               #   源文件
│   ├── driver/                # 外设驱动
│   │   ├── oled.c/h           #   SSD1306 OLED 驱动
│   │   ├── mpu6050.c/h        #   MPU6050 IMU 驱动
│   │   ├── mpu6050iic.c/h     #   MPU6050 I2C 通信
│   │   ├── inv_mpu.c/h        #   InvenSense MPU DMP 驱动
│   │   ├── bmp280.c/h         #   BMP280 气压传感器驱动
│   │   ├── driver_dht11.c/h   #   DHT11 温湿度驱动
│   │   ├── driver_passive_buzzer.c/h  # 蜂鸣器驱动
│   │   ├── driver_timer.c/h   #   定时器工具
│   │   └── beep.c/h           #   蜂鸣器音效
│   ├── mytasks/               # 应用任务
│   │   ├── ShowMenu.c/h        #   主菜单界面
│   │   ├── ShowClock.c/h      #   时钟表盘
│   │   ├── ShowCalendar.c/h   #   日历
│   │   ├── ShowStepTask.c/h   #   计步界面
│   │   ├── StepCountTask.c/h  #   计步检测
│   │   ├── ShowDHT11.c/h      #   温湿度界面
│   │   ├── ShowBMP280.c/h     #   气压海拔界面
│   │   ├── ShowFlashLight.c/h #   手电筒
│   │   ├── ShowTimeTask.c/h   #   时间显示
│   │   ├── ShowSetting.c/h    #   设置界面
│   │   ├── Data.c/h           #   共享数据类型
│   │   └── RootTask.c/h       #   根任务初始化
│   ├── Drivers/               # STM32 HAL & CMSIS 库
│   │   ├── CMSIS/
│   │   └── STM32F1xx_HAL_Driver/
│   ├── Middlewares/            # 第三方中间件
│   │   └── Third_Party/FreeRTOS/
│   ├── MDK-ARM/               # Keil MDK 工程
│   │   ├── hal_freertos_u8g2.uvprojx  # Keil 工程文件
│   │   └── startup_stm32f103xb.s      # 启动文件
│   ├── EIDE/                  # EIDE (Embedded IDE) 配置
│   └── hal_freertos_u8g2.ioc # STM32CubeMX 工程
└── docs/
    └── Stm32Watch软件设计概览.pdf  # 设计概览文档
```

## 编译

### 前置条件

- **Keil MDK-ARM** (v5 或更高)
- **STM32F1xx Device Family Pack** (通过 Keil Pack Installer 安装)
- **ARM Compiler** (AC5 或 AC6)

### 编译步骤

1. 克隆仓库
   ```bash
   git clone https://github.com/w4ysonch/Stm32Watch.git
   ```

2. 用 Keil MDK 打开 `STM32WATCH1.3/MDK-ARM/hal_freertos_u8g2.uvprojx`

3. **Project → Build Target** (F7) 编译

4. 固件 `Stm32Watch-vx.x.x.hex` 将生成在 `STM32WATCH1.3/MDK-ARM/hal_freertos_u8g2/` 目录下

> 也支持 **EIDE** (Embedded IDE) 打开，配置见 `STM32WATCH1.3/EIDE/`

## 烧录固件

### 方式一：ST-Link（推荐）

ST-Link 是 STM32 专用调试/烧录器，可购买独立烧录器或使用 ST 官方开发板自带的 ST-Link。

**接线 (4线 SWD):**

| ST-Link 引脚 | STM32F103 引脚 |
|-------------|---------------|
| SWCLK | PA14 (SWCLK) |
| SWDIO | PA13 (SWDIO) |
| GND | GND |
| 3.3V | 3.3V |

**步骤:**

1. 下载安装 [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)
2. 按上表将 ST-Link 连接至 STM32 开发板
3. 打开 STM32CubeProgrammer，选择 **ST-LINK** 连接方式
4. 点击 **Connect** 连接
5. 加载 `.hex` 固件文件
6. 点击 **Download** 开始烧录

### 方式二：串口烧录 (USB-TTL)

STM32F103 内置串口 Bootloader，只需一个 USB-TTL 模块即可烧录，无需额外硬件。

**接线 (3线 UART):**

| USB-TTL 引脚 | STM32F103 引脚 |
|-------------|---------------|
| TX | PA10 (USART1 RX) |
| RX | PA9 (USART1 TX) |
| GND | GND |

**步骤:**

1. 下载 [FlyMcu](http://www.mcuisp.com/) (Windows ISP 工具)
2. 上电前将 BOOT0 引脚接 **3.3V**（高电平），进入 Bootloader 模式
3. 上电或按复位键
4. 打开 FlyMcu，选择对应串口，波特率 115200
5. 加载 `.hex` 固件文件
6. 点击 **开始编程**
7. 烧录完成后将 BOOT0 接回 **GND**，按复位键运行

> **提示:** 方式一速度更快且支持在线调试，方式二成本低无需额外硬件。

### 引脚配置

完整引脚分配请用 STM32CubeMX 打开 `STM32WATCH1.3/hal_freertos_u8g2.ioc` 查看。

## 许可证

MIT License — 详见 [LICENSE](LICENSE)。

## 致谢

- STM32CubeMX HAL & CMSIS-RTOS
- FreeRTOS
- InvenSense MPU6050 DMP 驱动
- SSD1306 OLED 显示驱动
