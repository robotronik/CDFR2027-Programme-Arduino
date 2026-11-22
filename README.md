# Robotic System Controller (CDFR 2027)

Firmware for the CDFR2027-Carte-Arduino motor board. An Arduino Nano is the
I2C slave of the main computer and drives all the robot's actuators through
two PCA9685PW PWM drivers.

## Hardware

| Item | Detail |
|------|--------|
| MCU | Arduino Nano (ATmega328P, 5 V, 16 MHz) |
| PWM driver #1 | PCA9685PW, I2C address **0x41** |
| PWM driver #2 | PCA9685PW, I2C address **0x42** |
| PWM frequency | 50 Hz on both drivers (servo compatible) |
| Power stages | 6 x TPS5450 buck converters enabled by `POW_EN` |
| Output stages | 16 x AO3400A MOSFETs driven by the PCA9685 outputs |

> The PCA9685 base address is 0x40 and its address pins are compared against
> bits A5..A0. The board straps A0 on U1 and A1 on U2, hence 0x41 and 0x42.
> Addresses 0x01/0x02 referenced during design are *not* valid PCA9685
> addresses; the drivers are therefore referred to as "PCA #1" (0x41) and
> "PCA #2" (0x42).

### Two separate I2C buses

| Bus | Pins | Usage |
|-----|------|-------|
| Master bus | `A4` (SDA) / `A5` (SCL), hardware TWI | I2C slave at address `0x64` for the main computer (J1) |
| PCA bus | `D2` (SDA) / `D3` (SCL), bit-banged (SoftwareWire) | Both PCA9685, as I2C master |

### Arduino Nano pin map

| Pin | Net | Function |
|-----|-----|----------|
| D2 | `I2C_DIO` | Software I2C data to the PCA9685s |
| D3 | `I2C_CLK` | Software I2C clock to the PCA9685s |
| D4 | `SENS7` | Digital sensor 7 (input, pull-up) |
| D5 | `SENS6` | Digital sensor 6 (input, pull-up) |
| D6 | `SENS4` | Digital sensor 4 (input, pull-up) |
| D7 | `OUTPUT_ENABLE` | `~OE` of both PCA9685 (active low) |
| D8 | `SENS5` | Digital sensor 5 (input, pull-up) |
| D9 | `PWM_LIDAR` | Lidar PWM, 20 kHz hardware PWM (Timer1 / OC1A) |
| D10 | `SENS3` | Digital sensor 3 (input, pull-up) |
| D11 | `SENS2` | Digital sensor 2 (input, pull-up) |
| D12 | `SENS1` | Digital sensor 1 (input, pull-up) |
| A1 | `POW_EN` | Buck converters enable (active high) |
| A2 | `SENS8` | Digital sensor 8 (input, pull-up) |
| A4 | `MASTER_I2C_DIO` | Hardware I2C data to the main computer |
| A5 | `MASTER_I2C_CLK` | Hardware I2C clock to the main computer |

### PCA9685 channel map

**PCA #1 (0x41)**

| Channel | Net |
|---------|-----|
| 0 | `LED_R` |
| 1 | `LED_G` |
| 2 | `LED_B` |
| 3 .. 9 | `SERVO1` .. `SERVO7` |
| 10 | `EXTPWM10` |
| 11 | `EXTPWM11` |

**PCA #2 (0x42)**

| Channel | Net |
|---------|-----|
| 0 .. 5 | `PUMP1` .. `PUMP6` |
| 6 .. 15 | `EV1` .. `EV10` |

Pumps and electrovalves are driven fully on or fully off; the RGB LED uses
the PWM duty. The servos are driven with a 500-2500 us pulse at 50 Hz.

## Build

PlatformIO is used for building and uploading.

```bash
pio run                    # build
pio run --target upload    # flash the Nano
pio device monitor         # 115200 bauds
```

The default environment targets a Nano with the new (115200 baud) bootloader.
For a board with the old bootloader, set `board = nanoatmega328` in
`platformio.ini`.

Serial debug output can be enabled by uncommenting `#define SERIAL_DEBUG` in
`src/main.cpp`.

## I2C API

Commands are sent to address `0x64` as:

| Byte | Description |
|------|-------------|
| 0 | Command code |
| 1 | Target number (servo, pump, sensor, ...) |
| 2+ | Payload (little endian) |

Answers to request-type commands are read back with `requestFrom()`.

### Commands

| Command | Code | Payload | Description |
|---------|------|---------|-------------|
| `CMD_GET_VERSION` | 0x01 | - | Returns `API_VERSION` (`uint8`) |
| `CMD_POWER_SERVOS` | 0x11 | `on` | Drives `POW_EN` (0 = off, 1 = on) |
| `CMD_MOVE_SERVO` | 0x12 | `target:u16, speed:u16` | Moves servo 1..7 (angle 0..180, speed in deg/s, 0 = immediate) |
| `CMD_GET_SERVO` | 0x13 | - | Returns the current angle (`int16`) |
| `CMD_READ_SENSOR` | 0x21 | - | Returns sensor 1..8 state (`uint8`, 1 = active) |
| `CMD_RGB_LED` | 0x41 | `mode:u8, r:u8, g:u8, b:u8` | RGB LED (modes 0 = solid, 1 = blink, 2 = rainbow) |
| `CMD_SET_PWM_LIDAR` | 0x51 | - | Lidar PWM duty 0..255 (in the target byte) |
| `CMD_SET_PUMP` | 0x71 | `state` | Pump 1..6 on/off |
| `CMD_SET_EV` | 0x72 | `state` | Electrovalve 1..10 on/off |
| `CMD_SET_PWM_EXT` | 0x73 | `duty` | `EXTPWM10` / `EXTPWM11` duty 0..255 |

### Backward compatibility

The previous board (Arduino Mega Pro) exposed stepper motors and a DC motor.
Those features do not exist on the CDFR 2027 board, but their command codes
are still accepted so that existing main-computer code keeps working:

| Command | Code | Behaviour |
|---------|------|-----------|
| `CMD_ENABLE_STEPPER` | 0x31 | Accepted, no effect |
| `CMD_DISABLE_STEPPER` | 0x32 | Accepted, no effect |
| `CMD_MOVE_STEPPER` | 0x33 | Accepted, no effect |
| `CMD_SET_STEPPER` | 0x34 | Accepted, no effect |
| `CMD_GET_STEPPER` | 0x35 | Returns 0 (`int32`) |
| `CMD_SET_STEPPER_SPEED` | 0x36 | Accepted, no effect |
| `CMD_MOVE_DC_MOTOR` | 0x61 | Accepted, no effect |
| `CMD_STOP_DC_MOTOR` | 0x62 | Accepted, no effect |

`CMD_POWER_SERVOS` used to switch a dedicated servo power pin; it now drives
the buck converters enable line (`POW_EN`), which powers the servo rail.

## Notes

- The PCA9685 `~OE` pin is pulled up on the board, so all outputs stay off
  until the firmware drives `OUTPUT_ENABLE` low.
- The lidar PWM is a hardware output: Timer1 / OC1A on D9, 20 kHz exact, no CPU
  load and no jitter.
- Both I2C buses are independent: a missing PCA9685 does not prevent the Nano
  from answering on the master bus.
