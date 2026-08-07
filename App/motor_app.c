#include "App/motor_app.h"

#include <math.h>
#include <stddef.h>

#include "Config/motor_config.h"
#include "Control/speed_pid.h"
#include "Hardware/bsp_encoder.h"
#include "ti_msp_dl_config.h"

typedef struct {
    volatile MotorAppState state;
    SpeedPID leftPid;
    SpeedPID rightPid;
    volatile float leftRequestedTarget;
    volatile float rightRequestedTarget;
    volatile float leftTarget;
    volatile float rightTarget;
    volatile int32_t leftDelta;
    volatile int32_t rightDelta;
    volatile uint32_t controlTicks;
    volatile uint32_t leftDirectionFaultCycles;
    volatile uint32_t rightDirectionFaultCycles;
    volatile uint32_t leftNoFeedbackCycles;
    volatile uint32_t rightNoFeedbackCycles;
    volatile bool telemetryDue;
    volatile bool directionTestDone;
    volatile bool stopLatched;
} MotorAppContext;

static MotorAppContext gApp;

static float absFloat(float value) { return (value < 0.0f) ? -value : value; }

static bool controlParametersValid(void)
{
    return isfinite(LEFT_MAX_PWM_COMMAND) && isfinite(RIGHT_MAX_PWM_COMMAND) &&
        (LEFT_MAX_PWM_COMMAND > 0.0f) && (RIGHT_MAX_PWM_COMMAND > 0.0f) &&
        (LEFT_MAX_PWM_COMMAND <= (float)MOTOR_PWM_TIMER_COUNTS) &&
        (RIGHT_MAX_PWM_COMMAND <= (float)MOTOR_PWM_TIMER_COUNTS) &&
        isfinite(MOTOR_TARGET_VALUE_LIMIT) && (MOTOR_TARGET_VALUE_LIMIT > 0.0f);
}

static void enterFault(void)
{
    gApp.state = MOTOR_APP_FAULT;
    gApp.stopLatched = true;
    gApp.leftRequestedTarget = 0.0f;
    gApp.rightRequestedTarget = 0.0f;
    gApp.leftTarget = 0.0f;
    gApp.rightTarget = 0.0f;
    SpeedPID_Reset(&gApp.leftPid);
    SpeedPID_Reset(&gApp.rightPid);
    TB6612_EmergencyStopAll();
}

static bool oppositeSign(float target, float feedbackDelta)
{
    return ((target > 0.0f) && (feedbackDelta < -DIRECTION_MISMATCH_MIN_DELTA)) ||
           ((target < 0.0f) && (feedbackDelta > DIRECTION_MISMATCH_MIN_DELTA));
}

static bool updateProtectionOne(float target, float feedbackDelta, int32_t pwm,
    volatile uint32_t *directionCycles, volatile uint32_t *noFeedbackCycles)
{
    if ((absFloat(target) >= DIRECTION_CHECK_MIN_TARGET_VALUE) &&
        (absFloat((float)pwm) >= PROTECTION_PWM_THRESHOLD_COMMAND) &&
        oppositeSign(target, feedbackDelta)) {
        (*directionCycles)++;
    } else {
        *directionCycles = 0U;
    }

    if ((absFloat(target) >= DIRECTION_CHECK_MIN_TARGET_VALUE) &&
        (absFloat((float)pwm) >= PROTECTION_PWM_THRESHOLD_COMMAND) &&
        (absFloat(feedbackDelta) <= NO_FEEDBACK_MAX_DELTA)) {
        (*noFeedbackCycles)++;
    } else {
        *noFeedbackCycles = 0U;
    }
    return (*directionCycles >= DIRECTION_FAULT_CYCLES) ||
           (*noFeedbackCycles >= NO_FEEDBACK_FAULT_CYCLES);
}

static void runPidControl(void)
{
    float leftOutput = 0.0f, rightOutput = 0.0f;
    gApp.leftTarget = gApp.leftRequestedTarget;
    gApp.rightTarget = gApp.rightRequestedTarget;

    if (absFloat(gApp.leftTarget) < TARGET_STOP_DEADBAND_VALUE) {
        SpeedPID_Reset(&gApp.leftPid);
        TB6612_StopMotor(MOTOR_LEFT);
    } else {
        leftOutput = SpeedPID_Update(&gApp.leftPid, gApp.leftTarget, (float)gApp.leftDelta);
    }
    if (absFloat(gApp.rightTarget) < TARGET_STOP_DEADBAND_VALUE) {
        SpeedPID_Reset(&gApp.rightPid);
        TB6612_StopMotor(MOTOR_RIGHT);
    } else {
        rightOutput = SpeedPID_Update(&gApp.rightPid, gApp.rightTarget, (float)gApp.rightDelta);
    }

    if (!isfinite(leftOutput) || !isfinite(rightOutput)) {
        enterFault();
        return;
    }
    if ((absFloat(gApp.leftTarget) < TARGET_STOP_DEADBAND_VALUE) &&
        (absFloat(gApp.rightTarget) < TARGET_STOP_DEADBAND_VALUE)) {
        TB6612_DisableDriver();
    } else {
        TB6612_EnableDriver();
        TB6612_SetSignedPwm(MOTOR_LEFT, leftOutput);
        TB6612_SetSignedPwm(MOTOR_RIGHT, rightOutput);
    }

    if (updateProtectionOne(gApp.leftTarget, (float)gApp.leftDelta,
            TB6612_GetAppliedPwm(MOTOR_LEFT),
            &gApp.leftDirectionFaultCycles, &gApp.leftNoFeedbackCycles) ||
        updateProtectionOne(gApp.rightTarget, (float)gApp.rightDelta,
            TB6612_GetAppliedPwm(MOTOR_RIGHT),
            &gApp.rightDirectionFaultCycles, &gApp.rightNoFeedbackCycles)) {
        enterFault();
    }
}

void MotorApp_Init(void)
{
    gApp.state = MOTOR_APP_SAFE;
    gApp.leftRequestedTarget = LEFT_DEFAULT_TARGET_VALUE;
    gApp.rightRequestedTarget = RIGHT_DEFAULT_TARGET_VALUE;
    gApp.leftTarget = 0.0f;
    gApp.rightTarget = 0.0f;
    gApp.leftDelta = gApp.rightDelta = 0;
    gApp.controlTicks = 0U;
    gApp.leftDirectionFaultCycles = gApp.rightDirectionFaultCycles = 0U;
    gApp.leftNoFeedbackCycles = gApp.rightNoFeedbackCycles = 0U;
    gApp.telemetryDue = false;
    gApp.directionTestDone = false;
    gApp.stopLatched = false;

    SpeedPID_Init(&gApp.leftPid, LEFT_DEFAULT_KP, LEFT_DEFAULT_KI, LEFT_DEFAULT_KD,
        PID_CONTROL_PERIOD_S);
    SpeedPID_Init(&gApp.rightPid, RIGHT_DEFAULT_KP, RIGHT_DEFAULT_KI, RIGHT_DEFAULT_KD,
        PID_CONTROL_PERIOD_S);
    SpeedPID_SetOutputLimits(&gApp.leftPid, -LEFT_MAX_PWM_COMMAND, LEFT_MAX_PWM_COMMAND);
    SpeedPID_SetOutputLimits(&gApp.rightPid, -RIGHT_MAX_PWM_COMMAND, RIGHT_MAX_PWM_COMMAND);
    TB6612_EmergencyStopAll();

    if (!controlParametersValid()) {
        enterFault();
    } else if (APP_RUN_MODE == APP_MODE_ENCODER_TEST) {
        gApp.state = MOTOR_APP_ENCODER_TEST;
    } else if ((APP_RUN_MODE == APP_MODE_DIRECTION_TEST) && MOTOR_OUTPUT_MASTER_ENABLE) {
        gApp.state = MOTOR_APP_DIRECTION_TEST;
    } else if ((APP_RUN_MODE == APP_MODE_PID_TEST) && MOTOR_OUTPUT_MASTER_ENABLE) {
        gApp.state = MOTOR_APP_PID_TEST;
    }

    NVIC_EnableIRQ(CONTROL_TIMER_INST_INT_IRQN);
    DL_TimerG_startCounter(CONTROL_TIMER_INST);
}

void MotorApp_Process(void)
{
    /* 非实时扩展点；当前实时控制固定在 100 Hz TIMG0 ISR。 */
}

void MotorApp_ControlTick(void)
{
    const uint32_t startTicks = (DIRECTION_TEST_START_DELAY_MS * PID_CONTROL_FREQUENCY_HZ) / 1000U;
    const uint32_t runTicks = (DIRECTION_TEST_DURATION_MS * PID_CONTROL_FREQUENCY_HZ) / 1000U;
    gApp.controlTicks++;
    gApp.leftDelta = Encoder_GetAndClearDelta(ENCODER_LEFT);
    gApp.rightDelta = Encoder_GetAndClearDelta(ENCODER_RIGHT);
    if (gApp.state == MOTOR_APP_ENCODER_TEST) {
        TB6612_EmergencyStopAll();
    } else if (gApp.state == MOTOR_APP_DIRECTION_TEST) {
        if (!gApp.directionTestDone && (gApp.controlTicks >= startTicks) &&
            (gApp.controlTicks < (startTicks + runTicks))) {
            TB6612_EnableDriver();
            TB6612_SetSignedPwm(MOTOR_LEFT, DIRECTION_TEST_LEFT_PWM);
            TB6612_SetSignedPwm(MOTOR_RIGHT, DIRECTION_TEST_RIGHT_PWM);
        } else if (gApp.controlTicks >= (startTicks + runTicks)) {
            TB6612_EmergencyStopAll();
            gApp.directionTestDone = true;
            gApp.state = MOTOR_APP_SAFE;
        }
    } else if ((gApp.state == MOTOR_APP_PID_TEST) && !gApp.stopLatched) {
#if PID_AUTO_START_ENABLE == 1
        if (gApp.controlTicks == ((PID_AUTO_START_DELAY_MS * PID_CONTROL_FREQUENCY_HZ) / 1000U)) {
            gApp.leftRequestedTarget = LEFT_DEFAULT_TARGET_VALUE;
            gApp.rightRequestedTarget = RIGHT_DEFAULT_TARGET_VALUE;
        }
#endif
        runPidControl();
    } else {
        TB6612_EmergencyStopAll();
    }

    if ((gApp.controlTicks % (PID_CONTROL_FREQUENCY_HZ / VOFA_OUTPUT_FREQUENCY_HZ)) == 0U) {
        gApp.telemetryDue = true;
    }
}

bool MotorApp_SetPidTunings(MotorChannel motor, float kp, float ki, float kd)
{
    uint32_t key;
    if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd) || kp < 0.0f || ki < 0.0f || kd < 0.0f)
        return false;
    key = __get_PRIMASK(); __disable_irq();
    if (motor == MOTOR_LEFT) SpeedPID_SetTunings(&gApp.leftPid, kp, ki, kd);
    else if (motor == MOTOR_RIGHT) SpeedPID_SetTunings(&gApp.rightPid, kp, ki, kd);
    else { if (key == 0U) __enable_irq(); return false; }
    if (key == 0U) __enable_irq();
    return true;
}

bool MotorApp_SetPidGain(MotorChannel motor, MotorPidGain gain, float value)
{
    SpeedPID *pid;
    uint32_t key;
    if (!isfinite(value) || (value < 0.0f) ||
        ((motor != MOTOR_LEFT) && (motor != MOTOR_RIGHT)) ||
        ((gain != MOTOR_PID_GAIN_KP) && (gain != MOTOR_PID_GAIN_KI) &&
         (gain != MOTOR_PID_GAIN_KD))) return false;

    key = __get_PRIMASK();
    __disable_irq();
    pid = (motor == MOTOR_LEFT) ? &gApp.leftPid : &gApp.rightPid;
    if (gain == MOTOR_PID_GAIN_KP) pid->kp = value;
    else if (gain == MOTOR_PID_GAIN_KI) pid->ki = value;
    else pid->kd = value;
    if (key == 0U) __enable_irq();
    return true;
}

bool MotorApp_SetTargetValue(MotorChannel motor, float targetValue)
{
    uint32_t key;
    if (!isfinite(targetValue) || (absFloat(targetValue) > MOTOR_TARGET_VALUE_LIMIT) ||
        (gApp.state != MOTOR_APP_PID_TEST) || gApp.stopLatched) return false;
    key = __get_PRIMASK(); __disable_irq();
    if (motor == MOTOR_LEFT) gApp.leftRequestedTarget = targetValue;
    else if (motor == MOTOR_RIGHT) gApp.rightRequestedTarget = targetValue;
    else { if (key == 0U) __enable_irq(); return false; }
    if (key == 0U) __enable_irq();
    return true;
}

void MotorApp_EmergencyStop(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    gApp.stopLatched = true;
    gApp.state = MOTOR_APP_SAFE;
    gApp.leftRequestedTarget = gApp.rightRequestedTarget = 0.0f;
    gApp.leftTarget = gApp.rightTarget = 0.0f;
    SpeedPID_Reset(&gApp.leftPid);
    SpeedPID_Reset(&gApp.rightPid);
    TB6612_EmergencyStopAll();
    if (key == 0U) __enable_irq();
}

MotorAppState MotorApp_GetState(void) { return gApp.state; }

void MotorApp_GetTelemetry(MotorTelemetry *telemetry)
{
    uint32_t key;
    if (telemetry == NULL) return;
    key = __get_PRIMASK(); __disable_irq();
    telemetry->leftTargetValue = gApp.leftTarget;
    telemetry->leftPidFeedbackDelta = gApp.leftDelta;
    telemetry->leftPwmOutput = TB6612_GetAppliedPwm(MOTOR_LEFT);
    telemetry->leftEncoderTotal = Encoder_GetTotalCount(ENCODER_LEFT);
    telemetry->leftKp = gApp.leftPid.kp;
    telemetry->leftKi = gApp.leftPid.ki;
    telemetry->leftKd = gApp.leftPid.kd;
    telemetry->leftIntegralOutput = SpeedPID_GetIntegralOutput(&gApp.leftPid);
    telemetry->rightTargetValue = gApp.rightTarget;
    telemetry->rightPidFeedbackDelta = gApp.rightDelta;
    telemetry->rightPwmOutput = TB6612_GetAppliedPwm(MOTOR_RIGHT);
    telemetry->rightEncoderTotal = Encoder_GetTotalCount(ENCODER_RIGHT);
    telemetry->rightKp = gApp.rightPid.kp;
    telemetry->rightKi = gApp.rightPid.ki;
    telemetry->rightKd = gApp.rightPid.kd;
    telemetry->rightIntegralOutput = SpeedPID_GetIntegralOutput(&gApp.rightPid);
    if (key == 0U) __enable_irq();
}

bool MotorApp_TakeTelemetryFlag(void)
{
    bool due;
    uint32_t key = __get_PRIMASK(); __disable_irq();
    due = gApp.telemetryDue;
    gApp.telemetryDue = false;
    if (key == 0U) __enable_irq();
    return due;
}

void CONTROL_TIMER_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(CONTROL_TIMER_INST) == DL_TIMER_IIDX_ZERO) {
        MotorApp_ControlTick();
    }
}
