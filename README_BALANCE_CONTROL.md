# First-stage balance control

This stage adds a directly driven 500 Hz balance PID/PD controller. It does not
add a speed outer loop or steering loop. The existing 100 Hz wheel-speed PID is
preserved, but it cannot write PWM while `MOTOR_APP_BALANCE` owns the motors.

## Control law and timing

The verified sign convention is forward pitch positive. The target defaults to
zero degrees:

```text
error = target_pitch - pitch_angle
P = kp * error
integral = integral + error * dt
I = ki * integral
D = -kd * pitch_rate
unclamped = P + I + D
balance_output = clamp(unclamped, -output_limit, +output_limit)
motor_command = BALANCE_OUTPUT_SIGN * balance_output
```

`D` uses the measured gyroscope pitch rate, not numerical angle
differentiation. For a constant target, `d(error)/dt = -pitch_rate`, which is
why the derivative term is `-kd * pitch_rate_dps`.

TIMG6 runs the complete ordered path every 2 ms: read ICM42688, update the
attitude estimator, update the balance controller, then update both motor PWM
commands. UART formatting and VOFA output never run in this ISR. TIMG0 still
runs encoder collection and the existing speed-control state machine at 100 Hz,
but its balance-state branch deliberately performs no PWM writes.

## Safe defaults and limits

All values are centralized in `Config/balance_config.h`:

- KP/KI/KD defaults: `0 / 0 / 0`; balance mode is also disabled at startup.
- accepted KP range: `0..60`.
- accepted KI range: `0..10`.
- accepted KD range: `0..5`.
- target range: `-5..+5 deg`, default `0 deg`.
- controller output limit: `+/-320` PWM command counts.
- integral-state limit: `+/-20 degree-seconds`.
- BAL ON start-angle limit: `+/-5 deg`.
- fall angle: `+/-25 deg`.
- `BALANCE_OUTPUT_SIGN`: `+1.0` until the physical feedback direction is
  confirmed.

The integral is cleared on disable, BAL OFF, STOP, fall/fault and whenever KI
changes. Conditional integration rejects a candidate integral update when the
output is saturated and the current error would push it farther into the same
saturation direction.

## Commands

Commands use spaces and require a line ending:

```text
BAL ON
BAL OFF
BAL KP 1.0
BAL KI 0.0
BAL KD 0.05
BAL PID 1.0 0.0 0.05
BAL PID?
BAL TARGET 0.0
```

New balance commands return a short `OK ...` response or `ERR`. Numeric input
must be finite, completely parsed and inside the configured limits; otherwise
the previous value is retained.

`BAL ON` is rejected unless all of these are true:

- the latest IMU and attitude are valid;
- LEVEL/IMUZERO has completed and no new level calibration is active;
- absolute pitch is at most 5 degrees;
- no STOP latch or existing motor fault is active;
- the current state is safe, or is the startup speed-PID test state with zero
  requested speed and zero applied PWM.

Successful BAL ON resets both speed controllers, stops existing motor output,
clears an earlier balance-fall fault and transfers exclusive PWM ownership to
the balance controller. BAL OFF resets both controllers, sets both targets and
PWM to zero, and enters the safe state. It does not clear a STOP latch. This
project has no ARM command, so after STOP the existing latch intentionally
requires a processor reset before any BAL ON can succeed.

While balance is active, invalid IMU/attitude data, loss of level calibration,
non-finite control data, or `|pitch_angle| > 25 deg` immediately disables and
resets the controller, sets both PWM outputs to zero, disables TB6612, records a
balance fault, and enters the safe state. It never restarts automatically;
physically right the vehicle and issue BAL ON again.

## VOFA balance mode

`TELEMETRY_MODE_BALANCE` is the default and sends these 14 fields at about
50 Hz:

| Channel | Value |
| --- | --- |
| I0 | target pitch |
| I1 | filtered pitch angle |
| I2 | pitch rate |
| I3 | angle error |
| I4 | KP |
| I5 | KI |
| I6 | KD |
| I7 | P output |
| I8 | I output |
| I9 | D output |
| I10 | clamped balance output before direction sign |
| I11 | applied signed motor command after `BALANCE_OUTPUT_SIGN` and rounding |
| I12 | balance enabled, 0 or 1 |
| I13 | latched balance fault, 0 or 1 |

The IMU and MOTOR telemetry modes remain available for separate diagnostics.

## First physical test: verify feedback direction before tuning

Keep the wheels suspended, restrain the chassis and keep motor power easy to
disconnect. Complete LEVEL, then use `KI=0`, `KD=0` and a very small KP. Do not
try to stand the vehicle yet.

1. Send BAL ON and tilt the body forward slightly. The wheels must rotate
   forward to chase the falling body.
2. Tilt it backward. The wheels must rotate backward.
3. If either response is reversed, use BAL OFF immediately and change only
   `BALANCE_OUTPUT_SIGN`. Do not use a negative KP to repair direction.

After direction is correct, keep KI and KD at zero and increase KP gradually.
Too little KP gives inadequate recovery; too much creates rapid oscillation or
violent correction. Once proportional recovery is clear, add KD gradually to
increase damping. Too much KD creates harsh braking, rapid reversals or motor
noise.

Tune a usable PD response before considering KI. Slow vehicle travel while
balancing should later be handled primarily by a slower speed PI loop that
adjusts target pitch. That speed outer loop is intentionally not implemented
in this stage.
