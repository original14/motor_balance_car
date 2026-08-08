# Balance-car project status

This project currently provides the full first cascaded balance-control path:

- 500 Hz ICM42688 sampling and pitch estimation;
- command-enabled 500 Hz balance PID/PD;
- original independent 100 Hz left/right encoder speed PID loops;
- 20 kHz TB6612 PWM generated only by the speed-control path;
- runtime balance and speed-PID tuning, VOFA telemetry and safety shutdowns.

The balance controller publishes a wheel-speed target in encoder counts per
10 ms. It does not publish PWM. The speed loops retain their original tuning,
math, measurement units and 100 Hz period.

Balance remains disabled after reset. Complete the motor/encoder direction tests
and LEVEL calibration before sending BAL ON. See
[README_BALANCE_CONTROL.md](README_BALANCE_CONTROL.md) and
[README_BALANCE_SPEED_CASCADE.md](README_BALANCE_SPEED_CASCADE.md).

Steering control and an additional vehicle-speed outer loop are not included.
No source/build validation should be interpreted as physical motor validation.
