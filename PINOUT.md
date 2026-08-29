# Mainboard Pinout

STM32H743IIT6, LQFP176. Every pin the firmware actually touches, plus the two
reserved-but-inactive ones.

Source of truth is `../../../mainboard2/mainboard2.ioc` (STM32CubeMX). This file
is written by hand — if you re-run CubeMX and the pin map moves, update both this
file and the generated blocks in [`src/main.cpp`](src/main.cpp).

## All assigned pins

| Pin | Signal | Peripheral | Used for | Active? |
|---|---|---|---|---|
| PI9 | `FDCAN1_RX` | FDCAN1 | ESC bus receive | yes |
| PH13 | `FDCAN1_TX` | FDCAN1 | ESC bus transmit | yes |
| PB6 | `I2C1_SCL` | I2C1 (`Wire`) | both OLED eye screens | yes |
| PB7 | `I2C1_SDA` | I2C1 (`Wire`) | both OLED eye screens | yes |
| PB10 | `I2C2_SCL` | I2C2 (`Wire2`) | MPU6050 IMU | yes |
| PB11 | `I2C2_SDA` | I2C2 (`Wire2`) | MPU6050 IMU | yes |
| PA9 | `USART1_TX` | USART1 (`Serial_`) | debug console | yes |
| PA10 | `USART1_RX` | USART1 (`Serial_`) | debug console | yes |
| PA13 | `DEBUG_JTMS-SWDIO` | SWD | flashing + debug | yes |
| PA14 | `DEBUG_JTCK-SWCLK` | SWD | flashing + debug | yes |
| PA11 | `USB_OTG_FS_DM` | USB OTG FS | — | **no** |
| PA12 | `USB_OTG_FS_DP` | USB OTG FS | — | **no** |
| PH0 | `RCC_OSC_IN` | RCC | — | **no** |

The `.ioc` also assigns two virtual pins (`VP_SYS_VS_Systick`,
`VP_MEMORYMAP_VS_MEMORYMAP`); neither corresponds to a physical pin.

### About the three inactive ones

**PA11 / PA12** are reserved for USB in the CubeMX project, but the PlatformIO
build never enables USB (no `USBCON` in the build defines), so nothing configures
them at runtime — they come up as floating inputs. They were the FDCAN pins in an
earlier revision, which is why stale `PA11/PA12` CAN references may still turn up
in old comments or notes.

**PH0** is set up as a 25 MHz HSE external clock input in the `.ioc`, but neither
clock config actually selects HSE — the board runs entirely on the internal HSI.
See the clocks section below.

## FDCAN1 — ESC bus

Configured in `HAL_FDCAN_MspInit()` and `MX_FDCAN1_Init()` in
[`src/main.cpp`](src/main.cpp), both pasted from CubeMX output.

| | |
|---|---|
| Pins | PI9 = RX, PH13 = TX (both `GPIO_AF9_FDCAN1`) |
| Kernel clock | 50 MHz, from PLL2Q |
| Nominal bit rate | 1 Mbit/s, sample point 80% |
| Frame format | `FDCAN_FRAME_FD_NO_BRS` — FD frames, no bit-rate switching |
| Mode | `FDCAN_MODE_INTERNAL_LOOPBACK` (bench testing; CubeMX generates `FDCAN_MODE_NORMAL`) |
| Rx FIFO0 | 24 elements × 64 bytes |
| Tx FIFO queue | 8 elements × 64 bytes |

Bit timing: 50 MHz ÷ prescaler 5 = 10 MHz time-quantum clock (100 ns/tq);
1 sync + 7 seg1 + 2 seg2 = 10 tq = 1 µs per bit.

> The firmware currently runs the CAN controller in **internal loopback**, so it
> never drives PI9/PH13. Flip `hfdcan1.Init.Mode` to `FDCAN_MODE_NORMAL` to talk
> to real hardware.

### Message layout

| Direction | ID | Payload |
|---|---|---|
| mainboard → all ESCs | `0x100` | 32 bytes = 8 × `float` actions |
| ESC *n* → mainboard | `0x200 + n`, n = 0..7 | bytes 0–3 = `joint_pos` (float), bytes 4–7 = `joint_vel` (float) |

The RX filter is a mask filter accepting standard IDs where `id & 0x700 == 0x200`
(so `0x200`–`0x2FF`), routed to Rx FIFO0. The global filter rejects everything
non-matching, and rejects remote frames on both standard and extended IDs.

Joint index order is documented in the comments around `Model()` in
[`src/main.cpp`](src/main.cpp) — note those comments currently disagree with each
other about left/right ordering, so treat the model's expected order as
authoritative over the comment.

## I2C1 (`Wire`) — eye screens

Both SSD1306 128×64 OLEDs share this bus, at 400 kHz.

| Device | U8g2 address | 7-bit address |
|---|---|---|
| `l_screen` | `0x78` | `0x3C` |
| `r_screen` | `0x7A` | `0x3D` |

Pins are set explicitly with `Wire.setSCL(PB6)` / `Wire.setSDA(PB7)` before
`Wire.begin()`, so the variant's default `Wire` pins don't matter.

## I2C2 (`Wire2`) — IMU

MPU6050 at 7-bit address `0x68`, 400 kHz, running the on-chip DMP for
quaternion / gravity / yaw-pitch-roll.

Declared as `TwoWire Wire2(PB11, PB10)` — the constructor takes **(sda, scl)**,
so that is SDA = PB11, SCL = PB10.

## USART1 (`Serial_`) — debug console

`HardwareSerial Serial_(PA10, PA9)` — the constructor takes **(rx, tx)**, so
RX = PA10, TX = PA9. 115200 baud, matching `monitor_speed` in
[`platformio.ini`](platformio.ini).

Named `Serial_` rather than `Serial` deliberately: the bare `Serial` object would
be the core's default instance on different pins.

## SWD

PA13 (SWDIO) and PA14 (SWCLK). Upload protocol defaults to `stlink` per the board
definition in [`boards/genericSTM32H743IIT6.json`](boards/genericSTM32H743IIT6.json).

## Clocks — read this before trusting the `.ioc`

**The frequencies in the CubeMX project are not the frequencies this firmware
runs at.** CubeMX is only used here to generate FDCAN setup; the Arduino core
supplies the actual system clock config, and it never calls CubeMX's
`SystemClock_Config()`.

| | CubeMX `.ioc` says | What actually runs |
|---|---|---|
| Source | HSI | HSI |
| SYSCLK | 80 MHz | **480 MHz** |
| HCLK | 80 MHz | **240 MHz** |
| APB1 / APB2 | 40 MHz | **120 MHz** |
| I2C1/2 kernel | 40 MHz | **120 MHz** (D2PCLK1) |
| USART1 kernel | 40 MHz | **120 MHz** (D2PCLK2) |
| FDCAN kernel | 50 MHz | 50 MHz ✅ |

The real config is the weak `SystemClock_Config()` in the framework variant
`generic_clock.c` (HSI 64 MHz ÷ M4 × N60 ÷ P2 = 480 MHz). `src/main.cpp` does not
override it.

FDCAN is the one row that matches, and only because `HAL_FDCAN_MspInit()`
explicitly reconfigures PLL2 (M=4, N=9, FRACN=3072 → ×9.375 → 150 MHz VCO,
Q=3 → 50 MHz) and selects it as the FDCAN kernel clock. That is safe because both
clock configs use HSI with the same ÷4 pre-divider, and because nothing in the
Arduino core actually consumes PLL2 (USB uses HSI48, QSPI uses D1HCLK, SDMMC and
SPI123 use PLL1, and the UART/I2C peripherals run off the APB clocks).

I2C and UART baud rates are computed by the drivers from the live clock, so the
480 MHz tree does not break them.

## Enabling the FDCAN HAL module

The Arduino STM32 core lists `HAL_FDCAN_MODULE_ENABLED` under "Unused HAL
modules" — it is **off by default**. It is switched on in
[`include/hal_conf_extra.h`](include/hal_conf_extra.h), which the core includes
ahead of its own defaults.

That header has to be reachable from the framework's `SrcWrapper` library build,
not just from `src/`, which is why [`platformio.ini`](platformio.ini) carries
`-I include` in `build_flags`. Without it the driver compiles out and you get
`undefined reference to HAL_FDCAN_Init` at link time.
