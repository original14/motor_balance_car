#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

/*==================== 程序运行模式 ====================*/
#define APP_MODE_ENCODER_TEST      0
#define APP_MODE_DIRECTION_TEST    1
#define APP_MODE_PID_TEST          2

/* 默认只允许手动读取编码器，电机绝对不动作。 */
#define APP_RUN_MODE APP_MODE_PID_TEST

/* 电机输出硬件总闸：只有改为 1 后，方向/PID 测试才可能输出。 */
#define MOTOR_OUTPUT_MASTER_ENABLE 1

/*==================== 电机与编码器方向 ====================*/
#define LEFT_MOTOR_REVERSE    0
#define RIGHT_MOTOR_REVERSE   1
#define LEFT_ENCODER_REVERSE  1
#define RIGHT_ENCODER_REVERSE 0

/*==================== 编码器参数 ====================*/
#define LEFT_ENCODER_CPR   60000.0f
#define RIGHT_ENCODER_CPR  60000.0f

/*==================== 控制与遥测周期 ====================*/
#define PID_CONTROL_FREQUENCY_HZ 100U
#define PID_CONTROL_PERIOD_S     0.01f
#define VOFA_OUTPUT_FREQUENCY_HZ 50U

/*==================== PWM ====================*/
#define MOTOR_PWM_FREQUENCY_HZ 20000U
#define MOTOR_PWM_TIMER_COUNTS  1600U
#define LEFT_MAX_PWM_COMMAND    1280.0f
#define RIGHT_MAX_PWM_COMMAND   1280.0f
#define LEFT_MIN_EFFECTIVE_PWM_COMMAND  0.0f
#define RIGHT_MIN_EFFECTIVE_PWM_COMMAND 0.0f

/*==================== 方向测试 ====================*/
#define DIRECTION_TEST_START_DELAY_MS 3000U
#define DIRECTION_TEST_DURATION_MS    1000U
#define DIRECTION_TEST_LEFT_PWM       240.0f
#define DIRECTION_TEST_RIGHT_PWM      240.0f

/*==================== PID 默认参数 ====================*/
#define LEFT_DEFAULT_KP  1.1f
#define LEFT_DEFAULT_KI  0.9f
#define LEFT_DEFAULT_KD  0.00f
#define RIGHT_DEFAULT_KP 1.1f
#define RIGHT_DEFAULT_KI 0.9f
#define RIGHT_DEFAULT_KD 0.00f
#define LEFT_DEFAULT_TARGET_VALUE  0.0f
#define RIGHT_DEFAULT_TARGET_VALUE 0.0f
#define PID_AUTO_START_ENABLE    0
#define PID_AUTO_START_DELAY_MS  3000U
/* 目标值直接应用；单位是每个 10 ms 控制周期的编码器计数，不使用斜坡。 */
#define TARGET_STOP_DEADBAND_VALUE 1.0f

/*==================== 命令与保护 ====================*/
#define MOTOR_TARGET_VALUE_LIMIT 3000.0f
#define SERIAL_RX_BUFFER_SIZE  256U
#define SERIAL_COMMAND_MAX_LEN 96U

#define PROTECTION_PWM_THRESHOLD_COMMAND 160.0f
#define DIRECTION_CHECK_MIN_TARGET_VALUE 100.0f
#define DIRECTION_MISMATCH_MIN_DELTA      50.0f
#define DIRECTION_FAULT_CYCLES           20U
#define NO_FEEDBACK_MAX_DELTA              0.0f
#define NO_FEEDBACK_FAULT_CYCLES         100U

#endif
