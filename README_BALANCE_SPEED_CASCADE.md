# Balance-to-speed cascade design

## 1. Architecture before this change

```text
ICM42688 -> 500 Hz attitude -> 500 Hz balance PID -> PWM -> TB6612 -> motors
```

The 100 Hz left/right speed PIDs existed, but only PID_TEST used them. BALANCE
owned PWM directly.

## 2. Architecture after this change

```text
ICM42688 -> attitude -> balance PID -> wheel-speed target
                                      |
encoders -> left/right speed PIDs <----+
           -> PWM -> TB6612 -> motors
```

The cascade reuses the existing `leftPid`, `rightPid`, `runPidControl()` and
`SpeedPID_Update()` implementation. No second speed controller was added.

## 3. The 500 Hz balance path

TIMG6 executes every 2 ms. It reads ICM42688, updates the complementary-filter
attitude, runs `BalanceController_Update()` and publishes a bounded
`balanceSpeedTarget`. This path does not enable TB6612 or write normal PWM.
Emergency-stop calls remain permitted for sensor/fall safety.

## 4. The 100 Hz speed path

TIMG0 executes every 10 ms. It reads and clears each encoder interval delta,
selects the current target source, runs the original left/right speed PID and
writes the resulting PWM through TB6612. The speed PID period and formula remain
unchanged.

## 5. Speed and target units

`Encoder_GetAndClearDelta()` returns signed counts accumulated during the last
10 ms. `LSPD/RSPD`, speed-PID measurement, `balanceSpeedTarget`, and the active
left/right targets all use encoder counts per 10 ms. There is no RPM conversion.
The speed-PID output and TB6612 input use signed PWM timer counts.

## 6. PID_TEST and BALANCE sharing

```text
PID_TEST -> leftRequestedTarget/rightRequestedTarget -> runPidControl()
BALANCE  -> signed balanceSpeedTarget                -> runPidControl()
```

Only the target source changes. PID gains, integrator state type, output limits,
encoder measurements and final TB6612 calls are shared. Manual `LSPD/RSPD` is
rejected while BALANCE owns the target source.

## 7. Fault and stop handling

STOP, BAL OFF, a fall, invalid IMU/attitude data, non-finite output, direction
mismatch or missing encoder feedback clears the published balance target and
both active targets, resets both speed PIDs, resets/disables the balance
controller, writes zero PWM and disables TB6612 STBY. A previous speed target
cannot survive a balance fault.

## 8. VOFA channels

BALANCE mode uses I0-I18. I0-I10 describe attitude and balance calculation;
I11-I13 describe left target/feedback/PWM; I14-I16 describe right
target/feedback/PWM; I17-I18 report enable and fault state. The complete unit
table is in `README_BALANCE_CONTROL.md`.

## 9. Direction relationship

Both wheel targets have the same sign in the vehicle forward coordinate system.
Mirror mounting is handled by the existing `LEFT_MOTOR_REVERSE` and
`RIGHT_MOTOR_REVERSE` mappings, not by negating one target. With forward pitch
positive and `error = target - pitch`, `BALANCE_OUTPUT_SIGN=-1` converts the
negative forward-lean correction into a positive-forward speed target.

## 10. Hardware tuning order

First verify each existing speed PID and encoder sign with suspended wheels.
Then LEVEL the chassis, keep balance KI/KD at zero, start with very small KP and
confirm forward/backward chase direction. Tune KP for restoring force, add KD
for damping, and leave KI at zero until a stable PD response exists. Do not
ground-test until direction, STOP, BAL OFF, fall protection and current limits
have been verified with the chassis restrained.
