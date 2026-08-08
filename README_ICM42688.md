# ICM42688-P first-stage integration

This integration adds raw accelerometer and gyroscope acquisition only. It does
not calculate attitude, select a pitch axis, filter sensor data, close a balance
loop, or connect IMU data to either motor.

## Wiring and generated peripherals

| Signal | MSPM0G3507 pin | Configuration |
| --- | --- | --- |
| SCLK | PA12 | SPI0 controller output |
| SDO / MISO | PA13 | SPI0 POCI input |
| SDI / MOSI | PA14 | SPI0 PICO output |
| nCS | PA15 | software-controlled GPIO output, inactive high |
| INT1 | PA16 | GPIO input only; no interrupt is enabled |

SPI0 runs at 1 MHz, 8 bits, MSB first, CPOL=0 and CPHA=0. The electrical bus
has SCLK, MOSI, MISO and nCS. TI SysConfig calls the controller frame setting
`MOTO3` when nCS is deliberately controlled by a GPIO; `MOTO4` would require a
hardware SPI chip-select pin, and PA15 is not an SPI0 chip-select pin.

TIMG6 is the free timer used for the 500 Hz (2 ms) acquisition interrupt. Its
priority is 2, below the existing TIMG0 motor-control interrupt. Each 1 MHz
burst reads 12 data bytes and all SPI polling loops have finite timeouts. VOFA
transmission remains in the main loop at the existing approximately 50 Hz rate.

## Sensor setup

The driver follows TDK InvenSense `DS-000347`, ICM-42688-P Datasheet,
Revision 1.9:

- WHO_AM_I register `0x75`, expected value `0x47`.
- software reset through DEVICE_CONFIG `0x11`, followed by a 2 ms wait
  (the datasheet requires at least 1 ms).
- accelerometer range +/-4 g, gyroscope range +/-500 dps.
- accelerometer and gyroscope ODR 500 Hz, both in low-noise mode.
- 45 ms delay after enabling the gyroscope, including the 200 us no-write
  interval required for the OFF-to-ON transition.
- burst read from ACCEL_DATA_X1 `0x1F` through GYRO_DATA_Z0 `0x2A`.

Reference: <https://www.invensense.tdk.com/en-us/products/consumer/icm-42688-p>

Initialization failure is non-fatal to the rest of the application. A failed
SPI operation returns `false`, increments the error count, and marks the sample
invalid. A WHO_AM_I mismatch also leaves the IMU sampling timer stopped while
UART receive and the existing motor application continue normally.

## Telemetry selection

Set `TELEMETRY_OUTPUT_MODE` in `Config/telemetry_config.h`:

- `TELEMETRY_MODE_OFF`: no periodic transmit; UART receive remains active.
- `TELEMETRY_MODE_MOTOR`: preserves the original 16-channel motor stream.
- `TELEMETRY_MODE_IMU`: outputs 10 comma-separated fields plus CRLF:

| Channel | Value |
| --- | --- |
| I0-I2 | accel X/Y/Z in g |
| I3-I5 | gyro X/Y/Z in dps |
| I6 | latest sample valid (0 or 1) |
| I7 | WHO_AM_I in decimal (71 for `0x47`) |
| I8 | successful sample count |
| I9 | cumulative driver error count |

The current default is `TELEMETRY_MODE_IMU`.
