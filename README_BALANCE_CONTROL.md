# Cascaded balance control

The balance controller no longer writes motor PWM directly. It produces a
wheel-speed target which is consumed by the existing left/right speed PID loops.
The detailed ownership and timing design is in
[README_BALANCE_SPEED_CASCADE.md](README_BALANCE_SPEED_CASCADE.md).

## Control law and units

Forward pitch and pitch rate are positive. The target defaults to zero degrees:

```text
error = target_pitch - pitch_angle
P = kp * error
integral = integral + error * 0.002
I = ki * integral
D = -kd * pitch_rate
balanceSpeedTarget = clamp(P + I + D,
                           -BALANCE_SPEED_TARGET_LIMIT,
                           +BALANCE_SPEED_TARGET_LIMIT)

leftSpeedTarget  = BALANCE_OUTPUT_SIGN * balanceSpeedTarget
rightSpeedTarget = BALANCE_OUTPUT_SIGN * balanceSpeedTarget
```

`balanceSpeedTarget`, `leftSpeedTarget` and `rightSpeedTarget` use exactly the
same unit as `LSPD/RSPD`: encoder counts per 10 ms speed-control period. They are
not RPM, PWM counts or duty cycle. P, I and D are therefore also expressed in
encoder counts per 10 ms.

`D` uses the measured gyroscope pitch rate. For a fixed target,
`d(error)/dt = -pitch_rate`, avoiding noisy numerical angle differentiation.

## Timing and output ownership

- TIMG6, 500 Hz: read ICM42688, update attitude, run balance PID, publish the
  latest `balanceSpeedTarget`. It performs no normal motor PWM write.
- TIMG0, 100 Hz: read and clear both encoder deltas, select manual or balance
  speed targets, run the original two `SpeedPID_Update()` calls and write PWM
  through `TB6612_SetSignedPwm()`.
- Main loop, about 50 Hz telemetry trigger: parse UART commands and format VOFA
  output. No UART formatting runs in either control ISR.

PID_TEST and BALANCE share the same speed PID instances and execution function:

```text
PID_TEST: manual LSPD/RSPD -> existing speed PIDs -> PWM
BALANCE:  500 Hz balance target -> existing speed PIDs at 100 Hz -> PWM
```

`LSPD/RSPD` are rejected with `ERR` while BALANCE is active.

## Safe defaults and limits

- balance is OFF after reset; PID auto-start is disabled;
- KP/KI/KD defaults are `0 / 0 / 0`;
- target pitch range is `-5..+5 deg`;
- balance speed-target limit is `+/-300 encoder counts per 10 ms`;
- the limit matches the documented speed-PID test target and is 10% of the
  existing manual target acceptance limit of 3000;
- integral-state limit is `+/-20 degree-seconds`;
- BAL ON requires `|pitch| <= 5 deg`;
- fall protection triggers at `|pitch| > 25 deg`;
- `BALANCE_OUTPUT_SIGN` is `-1` for the current positive-forward speed
  convention and must still be confirmed with suspended wheels.

Conditional integration prevents further windup when the speed target is
saturated. Balance and both speed-PID integrators are reset on BAL OFF, STOP,
fall, invalid sensor data or control fault. Changing balance KI also clears its
stored integral.

## Commands

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

Existing `LKP/LKI/LKD`, `RKP/RKI/RKD`, `LPID/RPID`, `LSPD/RSPD` and `STOP`
commands are retained. Balance gains now map angle error to a wheel-speed target,
so parameters from the former direct-PWM architecture must not be copied.

`BAL ON` requires valid IMU and attitude data, completed LEVEL calibration, no
active calibration, a safe start angle, zero current manual speed/PWM and no
STOP or motor fault. `BAL OFF` immediately clears all targets, resets both
controllers, zeros PWM and disables TB6612. STOP remains latched until MCU reset.

During BALANCE, invalid IMU/attitude data, lost calibration, non-finite control
data, excessive pitch, direction mismatch or missing encoder feedback clears the
published speed target and both speed-PID histories before PWM is stopped.

## VOFA BALANCE channels

The default BALANCE stream sends 19 fields at about 50 Hz:

| Channel | Variable | Unit / meaning |
| --- | --- | --- |
| I0 | targetPitchDeg | deg |
| I1 | pitchAngleDeg | deg, filtered pitch |
| I2 | pitchRateDps | deg/s |
| I3 | angleErrorDeg | deg |
| I4 | balanceKp | speed-target counts/10 ms per deg |
| I5 | balanceKi | speed-target counts/10 ms per degree-second |
| I6 | balanceKd | speed-target counts/10 ms per deg/s |
| I7 | balancePOutput | encoder counts/10 ms |
| I8 | balanceIOutput | encoder counts/10 ms |
| I9 | balanceDOutput | encoder counts/10 ms |
| I10 | balanceSpeedTarget | raw clamped balance output, encoder counts/10 ms |
| I11 | leftSpeedTarget | signed target consumed by left speed PID |
| I12 | leftActualSpeed | left encoder delta during the last 10 ms |
| I13 | leftSpeedPidOutputPWM | signed applied PWM count |
| I14 | rightSpeedTarget | signed target consumed by right speed PID |
| I15 | rightActualSpeed | right encoder delta during the last 10 ms |
| I16 | rightSpeedPidOutputPWM | signed applied PWM count |
| I17 | balanceEnabled | 0 or 1 |
| I18 | balanceFault | latched balance fault, 0 or 1 |

I10 is updated at 500 Hz. I11-I16 reflect the most recent 100 Hz speed-PID
cycle, so a normal delay of up to one 10 ms period can appear between them.

## First hardware tuning sequence

1. Keep wheels suspended and the chassis restrained. Verify positive `LSPD` and
   `RSPD` make both wheels move forward and produce positive encoder deltas.
2. Complete LEVEL. Set balance KI and KD to zero and use a very small KP.
3. Enable BALANCE and tilt forward slightly. Both wheels must move forward to
   chase the chassis; a backward tilt must make both move backward.
4. If both directions are reversed, use BAL OFF and change only
   `BALANCE_OUTPUT_SIGN`. Do not make KP negative and do not make left/right
   targets opposite signs.
5. Increase KP gradually, then add KD for damping. Keep KI at zero initially.

The implementation and build do not prove motor direction or real-time margin;
both require controlled hardware validation before ground testing.
