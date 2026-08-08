# ICM42688-P pitch attitude integration

This document describes pitch attitude estimation and mechanical-level
calibration on the ICM42688-P data path. The optional first-stage balance
controller consumes these verified signals only after an explicit `BAL ON`;
see `README_BALANCE_CONTROL.md` for its separate safety rules.

## Wiring and generated peripherals

| Signal | MSPM0G3507 pin | Configuration |
| --- | --- | --- |
| SCLK | PA12 | SPI0 controller output |
| SDO / MISO | PA13 | SPI0 POCI input |
| SDI / MOSI | PA14 | SPI0 PICO output |
| nCS | PA15 | software-controlled GPIO output, inactive high |
| INT1 | PA16 | GPIO input only; no interrupt is enabled |

SPI0 runs at 1 MHz, 8 bits, MSB first, CPOL=0 and CPHA=0. TI SysConfig uses
`MOTO3` because nCS is controlled by GPIO; the external electrical connection
still contains SCLK, MOSI, MISO and nCS.

TIMG6 triggers the deterministic path at 500 Hz (2 ms). Its ISR reads the
sensor, updates the attitude estimator and, only while balance mode owns the
motors, updates the balance controller and PWM. UART parsing, formatting and
VOFA transmission remain in the main loop at approximately 50 Hz.

## Sensor setup

The driver follows TDK InvenSense `DS-000347`, ICM-42688-P Datasheet,
Revision 1.9:

- WHO_AM_I register `0x75`, expected value `0x47`.
- software reset through DEVICE_CONFIG `0x11`, followed by a 2 ms wait.
- accelerometer range +/-4 g and gyroscope range +/-500 dps.
- accelerometer and gyroscope ODR 500 Hz, both in low-noise mode.
- 45 ms wait after enabling the gyroscope.
- 12-byte burst read from ACCEL_DATA_X1 `0x1F` through GYRO_DATA_Z0 `0x2A`.

Reference: <https://www.invensense.tdk.com/en-us/products/consumer/icm-42688-p>

Initialization and SPI failures remain non-fatal. They mark IMU data invalid
and increment the driver error counter without stopping UART command handling.

## Verified pitch axes and signs

Physical testing established that the installed gyroscope Y axis is the pitch
rate axis and that forward tilt is positive:

```text
pitch_rate_dps = IMU_PITCH_GYRO_SIGN * gyro_y_dps
IMU_PITCH_GYRO_SIGN = +1.0
```

The pitch acceleration angle uses the X-Z plane. Accelerometer Y is not used in
the angle formula:

```text
pitch_acc_raw_deg = atan2(accel_x_g, -accel_z_g) * 57.2957795
pitch_acc_deg = pitch_acc_raw_deg - pitch_zero_offset_deg
```

`pitch_acc_raw_deg` is the sensor-mounting angle before mechanical zero
correction. `pitch_rate_dps` is instantaneous pitch angular velocity.
`pitch_angle_deg` is the fused pitch estimate.

## Mechanical level calibration

At startup, `pitch_zero_offset_deg` uses `-7.2 deg` only as a temporary fallback
measured on the current assembly. Before final tuning, hold the vehicle at its
true mechanical upright position and send either command followed by a line
ending:

```text
LEVEL
IMUZERO
```

Both commands start the same non-blocking state machine. The state machine
accepts 300 samples whose acceleration norm is between 0.90 g and 1.10 g and
whose absolute pitch rate is below
5 dps. At 500 Hz, 300 accepted samples take about 0.6 seconds. The new zero is
their mean `pitch_acc_raw_deg`, and the complementary-filter state is
resynchronized to the corrected acceleration angle.

The calibration window is limited to 1000 sample ticks (about 2 seconds). If
motion or invalid IMU data prevents 300 acceptable samples, the runtime zero
and the previous `level_calibrated` state are preserved. UART parsing is never
blocked, and the commands do not start or move either motor.

## Complementary filter

The estimator uses `dt = 0.002 s` and `tau = 0.30 s`. Alpha is derived instead
of being hard-coded:

```text
alpha = tau / (tau + dt)
pitch_predict = pitch_angle + pitch_rate * dt
pitch_angle = alpha * pitch_predict + (1 - alpha) * pitch_acc
```

On the first valid numeric sample, `pitch_angle` is initialized directly from
`pitch_acc`, rather than converging from zero. When acceleration norm is outside
0.90-1.10 g, translational acceleration may have corrupted the acceleration
angle, so the estimator temporarily uses only `pitch_predict`. Manual LEVEL
completion is not required for `attitude_valid`; before LEVEL, the fallback
zero is used.

## Telemetry selection

`TELEMETRY_MODE_IMU` remains available in `Config/telemetry_config.h` for sensor
diagnostics. It sends 16 comma-separated fields followed by CRLF:

| Channel | Value |
| --- | --- |
| I0 | corrected acceleration pitch `pitch_acc_deg` |
| I1 | signed pitch rate `pitch_rate_dps` |
| I2 | complementary-filter pitch `pitch_angle_deg` |
| I3 | uncorrected acceleration pitch `pitch_acc_raw_deg` |
| I4 | active mechanical zero `pitch_zero_offset_deg` |
| I5 | `accel_x_g` |
| I6 | `accel_y_g` |
| I7 | `accel_z_g` |
| I8 | raw installed-axis rate `gyro_y_dps` |
| I9 | acceleration magnitude `accel_norm_g` |
| I10 | `attitude_valid` (0 or 1) |
| I11 | `level_calibrated` (0 or 1) |
| I12 | latest IMU sample valid (0 or 1) |
| I13 | WHO_AM_I in decimal (71 for `0x47`) |
| I14 | successful sensor sample count |
| I15 | cumulative sensor-driver error count |

`TELEMETRY_MODE_MOTOR` still retains the original motor PID telemetry. The
default build now selects `TELEMETRY_MODE_BALANCE`; switch back to IMU mode
when raw sensor and attitude diagnostics are needed.
