# Stm32Watch

> A feature-rich smartwatch based on STM32F103, running FreeRTOS with a multi-task architecture. Features include step counting, environmental sensing, calendar, clock, flashlight, and interactive apps — all displayed on a 128x64 OLED screen.

**[中文 →](README.md)**

---

## Features

- **Clock** — Real-time clock display with date
- **Calendar** — Monthly calendar view
- **Step Counter** — Pedometer using MPU6050 DMP (Digital Motion Processor)
- **Thermometer & Hygrometer** — Temperature and humidity via DHT11
- **Barometer / Altimeter** — Atmospheric pressure and altitude via BMP280
- **Flashlight** — White screen torch
- **Wooden Fish** — Interactive tap game
- **Settings** — System configuration menu
- **Menu System** — Icon-based app launcher with buzzer feedback

## Hardware

| Component | Model | Interface |
|-----------|-------|-----------|
| MCU | STM32F103xB | — |
| Display | 128×64 OLED (SSD1306) | I2C |
| IMU | MPU6050 (6-axis) | I2C |
| Pressure Sensor | BMP280 | I2C |
| Temp/Humidity | DHT11 | GPIO (One-wire) |
| Buzzer | Passive buzzer | PWM (TIM) |

## Software Architecture

- **RTOS**: FreeRTOS with CMSIS-RTOS v2 API
- **HAL**: STM32F1xx HAL Library
- **Middleware**: CMSIS Core, STM32CubeMX-generated drivers
- **Task-based design**: Each app runs as a separate FreeRTOS task, with menu navigation via `vTaskSuspend` / `vTaskResume` scheduling
- **IPC**: FreeRTOS queues for key events, task notifications for inter-task communication

```
Task List:
  ShowMenuTask      — Main menu / app launcher
  ShowClockTask     — Clock face
  ShowCalendarTask  — Monthly calendar
  ShowStepTask      — Step counter display
  StepCountTask     — Background step detection (MPU6050 DMP)
  ShowDHT11Task     — Temperature & humidity display
  ShowBMP280Task    — Pressure & altitude display
  ShowFlashLightTask— Flashlight (screen torch)
  ShowTimeTask      — Time display
  ShowSettingTask   — System settings
  ShowWoodenFishTask— Interactive wooden fish game
```

## Project Structure

```
Stm32Watch/
├── STM32WATCH1.3/
│   ├── Core/                  # MCU config (CubeMX generated)
│   │   ├── Inc/               #   Headers
│   │   └── Src/               #   Sources
│   ├── driver/                # Peripheral drivers
│   │   ├── oled.c/h           #   SSD1306 OLED driver
│   │   ├── mpu6050.c/h        #   MPU6050 IMU driver
│   │   ├── mpu6050iic.c/h     #   MPU6050 I2C communication
│   │   ├── inv_mpu.c/h        #   InvenSense MPU DMP driver
│   │   ├── bmp280.c/h         #   BMP280 pressure sensor driver
│   │   ├── driver_dht11.c/h   #   DHT11 temperature/humidity driver
│   │   ├── driver_passive_buzzer.c/h  # Buzzer driver
│   │   ├── driver_timer.c/h   #   Timer utility
│   │   └── beep.c/h           #   Buzzer sound effects
│   ├── mytasks/               # Application tasks
│   │   ├── ShowMenu.c/h        #   Main menu UI
│   │   ├── ShowClock.c/h      #   Clock face
│   │   ├── ShowCalendar.c/h   #   Calendar
│   │   ├── ShowStepTask.c/h   #   Step counter UI
│   │   ├── StepCountTask.c/h  #   Step detection (MPU6050 DMP)
│   │   ├── ShowDHT11.c/h      #   Temperature & humidity UI
│   │   ├── ShowBMP280.c/h     #   Pressure & altitude UI
│   │   ├── ShowFlashLight.c/h #   Flashlight
│   │   ├── ShowTimeTask.c/h   #   Time display
│   │   ├── ShowSetting.c/h    #   Settings UI
│   │   ├── Data.c/h           #   Shared data types & assets
│   │   └── RootTask.c/h       #   Root task initializer
│   ├── Drivers/               # STM32 HAL & CMSIS libraries
│   │   ├── CMSIS/
│   │   └── STM32F1xx_HAL_Driver/
│   ├── Middlewares/            # Third-party middleware
│   │   └── Third_Party/FreeRTOS/
│   ├── MDK-ARM/               # Keil MDK project
│   │   ├── hal_freertos_u8g2.uvprojx  # Keil project file
│   │   └── startup_stm32f103xb.s      # Startup assembly
│   ├── EIDE/                  # EIDE (Embedded IDE) config
│   └── hal_freertos_u8g2.ioc # STM32CubeMX project
└── docs/
    └── Stm32Watch软件设计概览.pdf  # Design overview
```

## Build

### Prerequisites

- **Keil MDK-ARM** (v5 or later)
- **STM32F1xx Device Family Pack** (via Keil Pack Installer)
- **ARM Compiler** (AC5 or AC6)

### Build Steps

1. Clone the repository
   ```bash
   git clone https://github.com/w4ysonch/Stm32Watch.git
   ```

2. Open `STM32WATCH1.3/MDK-ARM/hal_freertos_u8g2.uvprojx` in Keil MDK

3. **Project → Build Target** (F7)

4. The firmware (`Stm32Watch-vx.x.x.hex`) will be generated under `STM32WATCH1.3/MDK-ARM/hal_freertos_u8g2/`

> Also supports **EIDE** (Embedded IDE) — open via `STM32WATCH1.3/EIDE/`

## Flash (Upload Firmware to STM32)

### Method 1: ST-Link (Recommended)

ST-Link is a dedicated debugger/programmer for STM32. Available as a standalone dongle or built into ST Discovery/Nucleo boards.

**Wiring (4-pin SWD):**

| ST-Link Pin | STM32F103 Pin |
|-------------|---------------|
| SWCLK | PA14 (SWCLK) |
| SWDIO | PA13 (SWDIO) |
| GND | GND |
| 3.3V | 3.3V |

**Steps:**

1. Download and install [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)
2. Connect ST-Link to your STM32 board as per the wiring table above
3. Open STM32CubeProgrammer, select **ST-LINK** as the connection method
4. Click **Connect**
5. Load the `.hex` firmware file
6. Click **Download** to flash

### Method 2: Serial (USB-TTL)

STM32F103 has a built-in serial bootloader, so you can flash using a cheap USB-TTL converter without extra hardware.

**Wiring (3-pin UART):**

| USB-TTL Pin | STM32F103 Pin |
|-------------|---------------|
| TX | PA10 (USART1 RX) |
| RX | PA9 (USART1 TX) |
| GND | GND |

**Steps:**

1. Download [FlyMcu](http://www.mcuisp.com/) (Windows ISP tool)
2. Set BOOT0 pin to **3.3V** (HIGH) to enter bootloader mode
3. Power on or press the RESET button
4. Open FlyMcu, select the correct COM port, baud rate 115200
5. Load the `.hex` firmware file
6. Click **Start Programming**
7. After flashing, set BOOT0 back to **GND** and press RESET to run

> **Tip:** Method 1 is faster and supports debugging. Method 2 is cheaper (no extra hardware needed).

### Pin Configuration

Refer to `STM32WATCH1.3/hal_freertos_u8g2.ioc` (open with STM32CubeMX) for the complete pinout.

## License

MIT License — see [LICENSE](LICENSE) for details.

## Acknowledgments

- STM32CubeMX HAL & CMSIS-RTOS
- FreeRTOS
- InvenSense MPU6050 DMP driver
- SSD1306 OLED display driver
