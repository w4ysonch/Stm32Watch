# Stm32Watch / STM32智能手表

A feature-rich smartwatch based on STM32F103, running FreeRTOS with a multi-task architecture. Features include step counting, environmental sensing, calendar, clock, flashlight, and interactive apps — all displayed on a 128x64 OLED screen.

> 基于 STM32F103 的 FreeRTOS 多任务智能手表。支持计步、温湿度/气压环境监测、日历、时钟、手电筒及互动小游戏，配备 128x64 OLED 显示屏。

## Features / 功能

- **Clock / 时钟** — Real-time clock display with date / 实时时钟与日期显示
- **Calendar / 日历** — Monthly calendar view / 月历视图
- **Step Counter / 计步器** — Pedometer using MPU6050 DMP (Digital Motion Processor)
- **Thermometer & Hygrometer / 温湿度计** — Temperature and humidity via DHT11
- **Barometer / Altimeter / 气压计** — Atmospheric pressure and altitude via BMP280
- **Flashlight / 手电筒** — White screen torch / 白屏手电
- **Wooden Fish / 木鱼** — Interactive tap game / 敲击互动游戏
- **Settings / 设置** — System configuration menu / 系统设置菜单
- **Menu System / 菜单系统** — Icon-based app launcher with buzzer feedback / 图标式应用启动器，带蜂鸣器反馈

## Hardware / 硬件

| Component / 组件 | Model / 型号 | Interface / 接口 |
|-----------|-------|-----------|
| MCU / 主控 | STM32F103xB | — |
| Display / 显示屏 | 128×64 OLED (SSD1306) | I2C |
| IMU / 惯性测量单元 | MPU6050 (6-axis) | I2C |
| Pressure Sensor / 气压传感器 | BMP280 | I2C |
| Temp/Humidity / 温湿度 | DHT11 | GPIO (单总线) |
| Buzzer / 蜂鸣器 | Passive buzzer / 无源蜂鸣器 | PWM (TIM) |

## Software Architecture / 软件架构

- **RTOS**: FreeRTOS with CMSIS-RTOS v2 API
- **HAL**: STM32F1xx HAL Library
- **Middleware / 中间件**: CMSIS Core, STM32CubeMX-generated drivers
- **Task-based design / 多任务设计**: Each app runs as a separate FreeRTOS task, with menu navigation via `vTaskSuspend` / `vTaskResume` scheduling
- **IPC / 任务间通信**: FreeRTOS queues for key events, task notifications for inter-task communication

```
Task List / 任务列表:
  ShowMenuTask      — Main menu / app launcher  / 主菜单
  ShowClockTask     — Clock face with analog-style UI  / 时钟表盘
  ShowCalendarTask  — Monthly calendar  / 月历
  ShowStepTask      — Step counter display  / 计步显示
  StepCountTask     — Background step detection (MPU6050 DMP) / 后台计步检测
  ShowDHT11Task     — Temperature & humidity display  / 温湿度显示
  ShowBMP280Task    — Pressure & altitude display  / 气压海拔显示
  ShowFlashLightTask— Flashlight (screen torch) / 手电筒
  ShowTimeTask      — Time display  / 时间显示
  ShowSettingTask   — System settings  / 系统设置
  ShowWoodenFishTask— Interactive wooden fish game / 木鱼游戏
```

## Project Structure / 项目结构

```
Stm32Watch/
├── STM32WATCH1.3/
│   ├── Core/                  # MCU config (CubeMX generated) / MCU配置
│   │   ├── Inc/               #   Headers / 头文件
│   │   └── Src/               #   Sources / 源文件
│   ├── driver/                # Peripheral drivers / 外设驱动
│   │   ├── oled.c/h           #   SSD1306 OLED driver
│   │   ├── mpu6050.c/h        #   MPU6050 IMU driver
│   │   ├── mpu6050iic.c/h     #   MPU6050 I2C communication
│   │   ├── inv_mpu.c/h        #   InvenSense MPU DMP driver
│   │   ├── bmp280.c/h         #   BMP280 pressure sensor driver
│   │   ├── driver_dht11.c/h   #   DHT11 temperature/humidity driver
│   │   ├── driver_passive_buzzer.c/h  # Buzzer driver / 蜂鸣器驱动
│   │   ├── driver_timer.c/h   #   Timer utility / 定时器工具
│   │   └── beep.c/h           #   Buzzer sound effects / 蜂鸣器音效
│   ├── mytasks/               # Application tasks / 应用任务
│   │   ├── ShowMenu.c/h        #   Main menu UI / 主菜单界面
│   │   ├── ShowClock.c/h      #   Clock face / 时钟表盘
│   │   ├── ShowCalendar.c/h   #   Calendar / 日历
│   │   ├── ShowStepTask.c/h   #   Step counter UI / 计步界面
│   │   ├── StepCountTask.c/h  #   Step detection (MPU6050 DMP) / 计步检测
│   │   ├── ShowDHT11.c/h      #   Temperature & humidity UI / 温湿度界面
│   │   ├── ShowBMP280.c/h     #   Pressure & altitude UI / 气压海拔界面
│   │   ├── ShowFlashLight.c/h #   Flashlight / 手电筒
│   │   ├── ShowTimeTask.c/h   #   Time display / 时间显示
│   │   ├── ShowSetting.c/h    #   Settings UI / 设置界面
│   │   ├── Data.c/h           #   Shared data types & assets / 共享数据类型
│   │   └── RootTask.c/h       #   Root task initializer / 根任务初始化
│   ├── Drivers/               # STM32 HAL & CMSIS libraries
│   │   ├── CMSIS/
│   │   └── STM32F1xx_HAL_Driver/
│   ├── Middlewares/            # Third-party middleware / 第三方中间件
│   │   └── Third_Party/FreeRTOS/
│   ├── MDK-ARM/               # Keil MDK project / Keil工程
│   │   ├── hal_freertos_u8g2.uvprojx  # Keil project file
│   │   └── startup_stm32f103xb.s      # Startup assembly / 启动文件
│   ├── EIDE/                  # EIDE (Embedded IDE) config
│   └── hal_freertos_u8g2.ioc # STM32CubeMX project / CubeMX工程
└── docs/
    └── Stm32Watch软件设计概览.pdf  # Design overview / 设计概览文档
```

## Build / 编译

### Prerequisites / 前置条件

- **Keil MDK-ARM** (v5 or later)
- **STM32F1xx Device Family Pack** (via Keil Pack Installer)
- **ARM Compiler** (AC5 or AC6)

### Build Steps / 编译步骤

1. Clone the repository / 克隆仓库
   ```bash
   git clone https://github.com/w4ysonch/Stm32Watch.git
   ```

2. Open `STM32WATCH1.3/MDK-ARM/hal_freertos_u8g2.uvprojx` in Keil MDK / 用Keil MDK打开工程文件

3. **Project → Build Target** (F7) / 点击编译

4. The compiled firmware (`hal_freertos_u8g2.hex`) will be generated under `STM32WATCH1.3/MDK-ARM/hal_freertos_u8g2/` / 编译生成的固件位于该目录下

> Also supports **EIDE** (Embedded IDE) — open via `STM32WATCH1.3/EIDE/` / 也支持EIDE嵌入式IDE打开

## Flash / 烧录固件

### Method 1: ST-Link (Recommended / 推荐)

ST-Link is a dedicated debugger/programmer for STM32. Available as a standalone dongle or built into ST Discovery/Nucleo boards.

> ST-Link 是 STM32 专用调试/烧录器，可购买独立烧录器或使用 ST 官方开发板自带的 ST-Link。

**Wiring / 接线 (4-pin SWD):**

| ST-Link Pin | STM32F103 Pin |
|-------------|---------------|
| SWCLK | PA14 (SWCLK) |
| SWDIO | PA13 (SWDIO) |
| GND | GND |
| 3.3V | 3.3V |

**Steps / 步骤:**

1. Download and install [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) / 下载安装 STM32CubeProgrammer
2. Connect ST-Link to your STM32 board as per the wiring table / 按上表接线
3. Open STM32CubeProgrammer, select **ST-LINK** as the connection method / 选择ST-LINK连接方式
4. Click **Connect** / 点击连接
5. Load the `.hex` file / 加载hex文件
6. Click **Download** to flash / 点击Download开始烧录

### Method 2: Serial / 串口烧录 (USB-TTL)

STM32F103 has a built-in serial bootloader, so you can flash using a cheap USB-TTL converter without extra hardware.

> STM32F103 内置串口 Bootloader，只需一个便宜 USB-TTL 模块即可烧录，无需额外硬件。

**Wiring / 接线 (3-pin UART):**

| USB-TTL Pin | STM32F103 Pin |
|-------------|---------------|
| TX | PA10 (USART1 RX) |
| RX | PA9 (USART1 TX) |
| GND | GND |

**Steps / 步骤:**

1. Download [FlyMcu](http://www.mcuisp.com/) / 下载 FlyMcu (Windows ISP 工具)
2. Set BOOT0 pin to **3.3V** (HIGH) to enter bootloader mode / 将 BOOT0 接高电平进入 Bootloader 模式
3. Power on or press the RESET button / 上电或按复位键
4. Open FlyMcu, select the correct COM port, baud rate 115200 / 打开 FlyMcu，选择对应串口及波特率 115200
5. Load the `.hex` file / 加载 hex 文件
6. Click **Start Programming** / 点击开始编程
7. After flashing, set BOOT0 back to **GND** and press RESET to run / 烧录完成后将 BOOT0 接回 GND，按复位运行

> **Tip / 提示:** Method 1 is faster and supports debugging. Method 2 is cheaper (no extra hardware needed). / 方法1速度快且支持在线调试，方法2成本低无需额外硬件。

### Pin Configuration / 引脚配置

Refer to `STM32WATCH1.3/hal_freertos_u8g2.ioc` (open with STM32CubeMX) for the complete pinout.

> 完整引脚分配请用 STM32CubeMX 打开 `hal_freertos_u8g2.ioc` 查看。

## License / 许可证

MIT License — see [LICENSE](LICENSE) for details.

## Acknowledgments / 致谢

- STM32CubeMX HAL & CMSIS-RTOS
- FreeRTOS
- InvenSense MPU6050 DMP driver
- SSD1306 OLED display driver
