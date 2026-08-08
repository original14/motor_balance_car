#ifndef BALANCE_CONFIG_H
#define BALANCE_CONFIG_H

/* 500 Hz attitude and balance-control period. */
#define BALANCE_CONTROL_HZ                  500U
#define BALANCE_CONTROL_DT_S                  0.002f

/* Safe startup defaults: BALANCE remains disabled and all gains start at zero. */
#define BALANCE_KP_DEFAULT                    0.0f
#define BALANCE_KI_DEFAULT                    0.0f
#define BALANCE_KD_DEFAULT                    0.0f

#define BALANCE_KP_MIN                        0.0f
#define BALANCE_KP_MAX                       60.0f
#define BALANCE_KI_MIN                        0.0f
#define BALANCE_KI_MAX                       10.0f
#define BALANCE_KD_MIN                        0.0f
#define BALANCE_KD_MAX                        5.0f

#define BALANCE_TARGET_DEFAULT_DEG            0.0f
#define BALANCE_TARGET_MIN_DEG               (-5.0f)
#define BALANCE_TARGET_MAX_DEG                 5.0f

/*
 * Balance output is a wheel-speed target in encoder counts per 10 ms,
 * matching LSPD/RSPD and the existing 100 Hz speed PID. 300 is the existing
 * documented speed-PID test target and is 10% of MOTOR_TARGET_VALUE_LIMIT.
 */
#define BALANCE_SPEED_TARGET_LIMIT           300.0f
#define BALANCE_INTEGRAL_LIMIT                20.0f

#define BALANCE_START_MAX_ANGLE_DEG            5.0f
#define BALANCE_FALL_ANGLE_DEG                25.0f

/*
 * Forward pitch is positive while the controller error is target - pitch.
 * The minus sign converts the resulting negative correction into the existing
 * positive-forward wheel-speed convention. Verify this with suspended wheels.
 */
#define BALANCE_OUTPUT_SIGN                  (-1.0f)

#endif
