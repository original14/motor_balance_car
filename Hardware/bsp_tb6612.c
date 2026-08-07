#include "Hardware/bsp_tb6612.h"

#include <stdbool.h>
#include <stdint.h>

#include "Config/motor_config.h"
#include "ti_msp_dl_config.h"

static int32_t gAppliedPwm[2];
static bool gDriverEnabled;

static float clampPwm(MotorChannel motor, float command)
{
    float limit = (motor == MOTOR_LEFT) ? LEFT_MAX_PWM_COMMAND : RIGHT_MAX_PWM_COMMAND;
    if (limit > (float)MOTOR_PWM_TIMER_COUNTS) limit = (float)MOTOR_PWM_TIMER_COUNTS;
    if (command > limit) command = limit;
    if (command < -limit) command = -limit;
    return command;
}

static uint32_t setCompare(MotorChannel motor, float magnitude)
{
    uint32_t applied;
    uint32_t compare;
    uint_fast8_t index = (motor == MOTOR_LEFT) ? GPIO_MOTOR_PWM_C0_IDX : GPIO_MOTOR_PWM_C1_IDX;

    if (magnitude < 0.0f) magnitude = 0.0f;
    if (magnitude > (float)MOTOR_PWM_TIMER_COUNTS) magnitude = (float)MOTOR_PWM_TIMER_COUNTS;
    applied = (uint32_t)(magnitude + 0.5f);
    compare = MOTOR_PWM_TIMER_COUNTS - applied;
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, compare, index);
    return applied;
}

static void setDirection(MotorChannel motor, bool forward)
{
    uint32_t pin1 = (motor == MOTOR_LEFT) ? MOTOR_GPIO_AIN1_PIN : MOTOR_GPIO_BIN1_PIN;
    uint32_t pin2 = (motor == MOTOR_LEFT) ? MOTOR_GPIO_AIN2_PIN : MOTOR_GPIO_BIN2_PIN;
    bool reverse = (motor == MOTOR_LEFT) ? (LEFT_MOTOR_REVERSE != 0) : (RIGHT_MOTOR_REVERSE != 0);

    if (reverse) forward = !forward;
    DL_GPIO_clearPins(MOTOR_GPIO_PORT, pin1 | pin2);
    DL_GPIO_setPins(MOTOR_GPIO_PORT, forward ? pin1 : pin2);
}

void TB6612_Init(void)
{
    gDriverEnabled = false;
    gAppliedPwm[MOTOR_LEFT] = 0;
    gAppliedPwm[MOTOR_RIGHT] = 0;
    setCompare(MOTOR_LEFT, 0.0f);
    setCompare(MOTOR_RIGHT, 0.0f);
    DL_GPIO_clearPins(MOTOR_GPIO_PORT, MOTOR_GPIO_STBY_PIN | MOTOR_GPIO_AIN1_PIN |
        MOTOR_GPIO_AIN2_PIN | MOTOR_GPIO_BIN1_PIN | MOTOR_GPIO_BIN2_PIN);
    DL_TimerG_startCounter(MOTOR_PWM_INST);
}

void TB6612_EnableDriver(void)
{
#if MOTOR_OUTPUT_MASTER_ENABLE == 1
    gDriverEnabled = true;
    DL_GPIO_setPins(MOTOR_GPIO_PORT, MOTOR_GPIO_STBY_PIN);
#else
    TB6612_EmergencyStopAll();
#endif
}

void TB6612_DisableDriver(void)
{
    gDriverEnabled = false;
    setCompare(MOTOR_LEFT, 0.0f);
    setCompare(MOTOR_RIGHT, 0.0f);
    gAppliedPwm[MOTOR_LEFT] = 0;
    gAppliedPwm[MOTOR_RIGHT] = 0;
    DL_GPIO_clearPins(MOTOR_GPIO_PORT, MOTOR_GPIO_STBY_PIN);
}

void TB6612_SetSignedPwm(MotorChannel motor, float command)
{
    float magnitude;
    float minimum;
    uint32_t applied;
    if ((motor != MOTOR_LEFT) && (motor != MOTOR_RIGHT)) return;
#if MOTOR_OUTPUT_MASTER_ENABLE == 0
    (void)command;
    TB6612_EmergencyStopAll();
    return;
#endif
    if (!gDriverEnabled) {
        TB6612_StopMotor(motor);
        return;
    }
    command = clampPwm(motor, command);
    magnitude = (command < 0.0f) ? -command : command;
    minimum = (motor == MOTOR_LEFT) ? LEFT_MIN_EFFECTIVE_PWM_COMMAND : RIGHT_MIN_EFFECTIVE_PWM_COMMAND;
    if ((magnitude > 0.0f) && (magnitude < minimum)) magnitude = minimum;
    if (magnitude <= 0.0f) {
        TB6612_StopMotor(motor);
        return;
    }
    setDirection(motor, command > 0.0f);
    applied = setCompare(motor, magnitude);
    gAppliedPwm[motor] = (command < 0.0f) ? -(int32_t)applied : (int32_t)applied;
}

void TB6612_StopMotor(MotorChannel motor) { TB6612_CoastMotor(motor); }

void TB6612_CoastMotor(MotorChannel motor)
{
    uint32_t pins = (motor == MOTOR_LEFT) ? (MOTOR_GPIO_AIN1_PIN | MOTOR_GPIO_AIN2_PIN) :
        (MOTOR_GPIO_BIN1_PIN | MOTOR_GPIO_BIN2_PIN);
    if ((motor != MOTOR_LEFT) && (motor != MOTOR_RIGHT)) return;
    setCompare(motor, 0.0f);
    DL_GPIO_clearPins(MOTOR_GPIO_PORT, pins);
    gAppliedPwm[motor] = 0;
}

void TB6612_BrakeMotor(MotorChannel motor)
{
    uint32_t pins = (motor == MOTOR_LEFT) ? (MOTOR_GPIO_AIN1_PIN | MOTOR_GPIO_AIN2_PIN) :
        (MOTOR_GPIO_BIN1_PIN | MOTOR_GPIO_BIN2_PIN);
    if ((motor != MOTOR_LEFT) && (motor != MOTOR_RIGHT)) return;
    setCompare(motor, 0.0f);
    if (gDriverEnabled) DL_GPIO_setPins(MOTOR_GPIO_PORT, pins);
    gAppliedPwm[motor] = 0;
}

void TB6612_EmergencyStopAll(void)
{
    gDriverEnabled = false;
    setCompare(MOTOR_LEFT, 0.0f);
    setCompare(MOTOR_RIGHT, 0.0f);
    DL_GPIO_clearPins(MOTOR_GPIO_PORT, MOTOR_GPIO_STBY_PIN | MOTOR_GPIO_AIN1_PIN |
        MOTOR_GPIO_AIN2_PIN | MOTOR_GPIO_BIN1_PIN | MOTOR_GPIO_BIN2_PIN);
    gAppliedPwm[MOTOR_LEFT] = 0;
    gAppliedPwm[MOTOR_RIGHT] = 0;
}

int32_t TB6612_GetAppliedPwm(MotorChannel motor)
{
    return ((motor == MOTOR_LEFT) || (motor == MOTOR_RIGHT)) ? gAppliedPwm[motor] : 0;
}
