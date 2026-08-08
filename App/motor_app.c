#include "App/motor_app.h"

#include <math.h>
#include <stddef.h>

#include "Config/balance_config.h"
#include "Config/motor_config.h"
#include "Control/balance_controller.h"
#include "Control/speed_pid.h"
#include "Hardware/bsp_encoder.h"
#include "ti_msp_dl_config.h"

typedef struct {
    volatile MotorAppState state;
    SpeedPID leftPid;
    SpeedPID rightPid;
    BalanceController balanceController;
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
    volatile bool balanceFault;
    volatile float balanceSpeedTarget;
} MotorAppContext;

static MotorAppContext gApp;

static float absFloat(float value) { return (value < 0.0f) ? -value : value; }

static bool controlParametersValid(void)
{
    return isfinite(LEFT_MAX_PWM_COMMAND) && isfinite(RIGHT_MAX_PWM_COMMAND) &&
        (LEFT_MAX_PWM_COMMAND > 0.0f) && (RIGHT_MAX_PWM_COMMAND > 0.0f) &&
        (LEFT_MAX_PWM_COMMAND <= (float)MOTOR_PWM_TIMER_COUNTS) &&
        (RIGHT_MAX_PWM_COMMAND <= (float)MOTOR_PWM_TIMER_COUNTS) &&
        isfinite(BALANCE_SPEED_TARGET_LIMIT) &&
        (BALANCE_SPEED_TARGET_LIMIT > 0.0f) &&
        (BALANCE_SPEED_TARGET_LIMIT <= MOTOR_TARGET_VALUE_LIMIT) &&
        ((BALANCE_OUTPUT_SIGN == 1.0f) || (BALANCE_OUTPUT_SIGN == -1.0f)) &&
        isfinite(MOTOR_TARGET_VALUE_LIMIT) && (MOTOR_TARGET_VALUE_LIMIT > 0.0f);
}

static void enterFault(void)
{
    bool wasBalance;
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    wasBalance = (gApp.state == MOTOR_APP_BALANCE);
    gApp.state = MOTOR_APP_FAULT;
    gApp.stopLatched = true;
    gApp.leftRequestedTarget = 0.0f;
    gApp.rightRequestedTarget = 0.0f;
    gApp.leftTarget = 0.0f;
    gApp.rightTarget = 0.0f;
    SpeedPID_Reset(&gApp.leftPid);
    SpeedPID_Reset(&gApp.rightPid);
    BalanceController_Disable(&gApp.balanceController);
    gApp.balanceSpeedTarget = 0.0f;
    if (wasBalance) gApp.balanceFault = true;
    TB6612_EmergencyStopAll();
    if (key == 0U) __enable_irq();
}

static void enterBalanceFault(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    gApp.balanceFault = true;
    gApp.state = MOTOR_APP_SAFE;
    gApp.leftRequestedTarget = 0.0f;
    gApp.rightRequestedTarget = 0.0f;
    gApp.leftTarget = 0.0f;
    gApp.rightTarget = 0.0f;
    SpeedPID_Reset(&gApp.leftPid);
    SpeedPID_Reset(&gApp.rightPid);
    BalanceController_Disable(&gApp.balanceController);
    gApp.balanceSpeedTarget = 0.0f;
    TB6612_EmergencyStopAll();
    if (key == 0U) __enable_irq();
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

static void runPidControl(float leftTarget, float rightTarget)
{
    float leftOutput = 0.0f, rightOutput = 0.0f;
    if (!isfinite(leftTarget) || !isfinite(rightTarget) ||
        (absFloat(leftTarget) > MOTOR_TARGET_VALUE_LIMIT) ||
        (absFloat(rightTarget) > MOTOR_TARGET_VALUE_LIMIT)) {
        enterFault();
        return;
    }
    gApp.leftTarget = leftTarget;
    gApp.rightTarget = rightTarget;

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
    gApp.balanceFault = false;
    gApp.balanceSpeedTarget = 0.0f;

    SpeedPID_Init(&gApp.leftPid, LEFT_DEFAULT_KP, LEFT_DEFAULT_KI, LEFT_DEFAULT_KD,
        PID_CONTROL_PERIOD_S);
    SpeedPID_Init(&gApp.rightPid, RIGHT_DEFAULT_KP, RIGHT_DEFAULT_KI, RIGHT_DEFAULT_KD,
        PID_CONTROL_PERIOD_S);
    SpeedPID_SetOutputLimits(&gApp.leftPid, -LEFT_MAX_PWM_COMMAND, LEFT_MAX_PWM_COMMAND);
    SpeedPID_SetOutputLimits(&gApp.rightPid, -RIGHT_MAX_PWM_COMMAND, RIGHT_MAX_PWM_COMMAND);
    BalanceController_Init(&gApp.balanceController);
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
    } else if (gApp.state == MOTOR_APP_BALANCE) {
        float balanceTarget = BALANCE_OUTPUT_SIGN * gApp.balanceSpeedTarget;
        runPidControl(balanceTarget, balanceTarget);
    } else if ((gApp.state == MOTOR_APP_PID_TEST) && !gApp.stopLatched) {
#if PID_AUTO_START_ENABLE == 1
        if (gApp.controlTicks == ((PID_AUTO_START_DELAY_MS * PID_CONTROL_FREQUENCY_HZ) / 1000U)) {
            gApp.leftRequestedTarget = LEFT_DEFAULT_TARGET_VALUE;
            gApp.rightRequestedTarget = RIGHT_DEFAULT_TARGET_VALUE;
        }
#endif
        runPidControl(gApp.leftRequestedTarget, gApp.rightRequestedTarget);
    } else {
        TB6612_EmergencyStopAll();
    }

    if ((gApp.controlTicks % (PID_CONTROL_FREQUENCY_HZ / VOFA_OUTPUT_FREQUENCY_HZ)) == 0U) {
        gApp.telemetryDue = true;
    }
}

void MotorApp_BalanceTick(bool imuValid, bool attitudeValid, bool levelCalibrated,
    float pitchAngleDeg, float pitchRateDps)
{
    float balanceSpeedTarget;

    if (gApp.state != MOTOR_APP_BALANCE) return;
    if (gApp.stopLatched || !imuValid || !attitudeValid || !levelCalibrated ||
        !isfinite(pitchAngleDeg) || !isfinite(pitchRateDps) ||
        (absFloat(pitchAngleDeg) > BALANCE_FALL_ANGLE_DEG)) {
        enterBalanceFault();
        return;
    }

    balanceSpeedTarget = BalanceController_Update(&gApp.balanceController,
        pitchAngleDeg, pitchRateDps, BALANCE_CONTROL_DT_S);
    if (!isfinite(balanceSpeedTarget)) {
        enterBalanceFault();
        return;
    }
    gApp.balanceSpeedTarget = balanceSpeedTarget;
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
    if (!isfinite(targetValue) ||
        (absFloat(targetValue) > MOTOR_TARGET_VALUE_LIMIT)) return false;
    key = __get_PRIMASK();
    __disable_irq();
    if ((gApp.state != MOTOR_APP_PID_TEST) || gApp.stopLatched) {
        if (key == 0U) __enable_irq();
        return false;
    }
    if (motor == MOTOR_LEFT) gApp.leftRequestedTarget = targetValue;
    else if (motor == MOTOR_RIGHT) gApp.rightRequestedTarget = targetValue;
    else { if (key == 0U) __enable_irq(); return false; }
    if (key == 0U) __enable_irq();
    return true;
}

bool MotorApp_EnableBalance(bool imuValid, bool attitudeValid, bool levelCalibrated,
    float pitchAngleDeg)
{
    bool speedStopped;
    bool stateAllowed;
    uint32_t key;

    if (!imuValid || !attitudeValid || !levelCalibrated || !isfinite(pitchAngleDeg) ||
        (absFloat(pitchAngleDeg) > BALANCE_START_MAX_ANGLE_DEG) ||
        (MOTOR_OUTPUT_MASTER_ENABLE != 1)) return false;

    key = __get_PRIMASK();
    __disable_irq();
    stateAllowed = (gApp.state == MOTOR_APP_SAFE) ||
        (gApp.state == MOTOR_APP_PID_TEST);
    speedStopped = (absFloat(gApp.leftRequestedTarget) < TARGET_STOP_DEADBAND_VALUE) &&
        (absFloat(gApp.rightRequestedTarget) < TARGET_STOP_DEADBAND_VALUE) &&
        (TB6612_GetAppliedPwm(MOTOR_LEFT) == 0) &&
        (TB6612_GetAppliedPwm(MOTOR_RIGHT) == 0);
    if (!stateAllowed || !speedStopped || gApp.stopLatched) {
        if (key == 0U) __enable_irq();
        return false;
    }

    gApp.leftRequestedTarget = 0.0f;
    gApp.rightRequestedTarget = 0.0f;
    gApp.leftTarget = 0.0f;
    gApp.rightTarget = 0.0f;
    SpeedPID_Reset(&gApp.leftPid);
    SpeedPID_Reset(&gApp.rightPid);
    TB6612_EmergencyStopAll();
    BalanceController_Enable(&gApp.balanceController);
    gApp.balanceSpeedTarget = 0.0f;
    gApp.balanceFault = false;
    gApp.state = MOTOR_APP_BALANCE;
    if (key == 0U) __enable_irq();
    return true;
}

void MotorApp_DisableBalance(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    BalanceController_Disable(&gApp.balanceController);
    gApp.balanceSpeedTarget = 0.0f;
    gApp.leftRequestedTarget = 0.0f;
    gApp.rightRequestedTarget = 0.0f;
    gApp.leftTarget = 0.0f;
    gApp.rightTarget = 0.0f;
    SpeedPID_Reset(&gApp.leftPid);
    SpeedPID_Reset(&gApp.rightPid);
    if (gApp.state != MOTOR_APP_FAULT) gApp.state = MOTOR_APP_SAFE;
    TB6612_EmergencyStopAll();
    if (key == 0U) __enable_irq();
}

bool MotorApp_SetBalancePid(float kp, float ki, float kd)
{
    bool updated;
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    updated = BalanceController_SetPID(&gApp.balanceController, kp, ki, kd);
    if (key == 0U) __enable_irq();
    return updated;
}

bool MotorApp_SetBalanceGain(BalancePidGain gain, float value)
{
    bool updated = false;
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    if (gain == BALANCE_PID_GAIN_KP) {
        updated = BalanceController_SetPID(&gApp.balanceController, value,
            gApp.balanceController.ki, gApp.balanceController.kd);
    } else if (gain == BALANCE_PID_GAIN_KI) {
        updated = BalanceController_SetPID(&gApp.balanceController,
            gApp.balanceController.kp, value, gApp.balanceController.kd);
    } else if (gain == BALANCE_PID_GAIN_KD) {
        updated = BalanceController_SetPID(&gApp.balanceController,
            gApp.balanceController.kp, gApp.balanceController.ki, value);
    }
    if (key == 0U) __enable_irq();
    return updated;
}

bool MotorApp_SetBalanceTarget(float targetAngleDeg)
{
    bool updated;
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    updated = BalanceController_SetTargetAngle(&gApp.balanceController, targetAngleDeg);
    if (key == 0U) __enable_irq();
    return updated;
}

bool MotorApp_IsBalanceEnabled(void)
{
    bool enabled;
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    enabled = (gApp.state == MOTOR_APP_BALANCE) && gApp.balanceController.enabled;
    if (key == 0U) __enable_irq();
    return enabled;
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
    BalanceController_Disable(&gApp.balanceController);
    gApp.balanceSpeedTarget = 0.0f;
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

void MotorApp_GetBalanceTelemetry(MotorBalanceTelemetry *telemetry)
{
    BalanceControllerTelemetry controllerTelemetry;
    uint32_t key;
    if (telemetry == NULL) return;

    key = __get_PRIMASK();
    __disable_irq();
    BalanceController_GetTelemetry(&gApp.balanceController, &controllerTelemetry);
    telemetry->targetPitchDeg = controllerTelemetry.target_angle_deg;
    telemetry->pitchAngleDeg = controllerTelemetry.pitch_angle_deg;
    telemetry->pitchRateDps = controllerTelemetry.pitch_rate_dps;
    telemetry->angleErrorDeg = controllerTelemetry.error_deg;
    telemetry->kp = controllerTelemetry.kp;
    telemetry->ki = controllerTelemetry.ki;
    telemetry->kd = controllerTelemetry.kd;
    telemetry->pOutput = controllerTelemetry.p_output;
    telemetry->iOutput = controllerTelemetry.i_output;
    telemetry->dOutput = controllerTelemetry.d_output;
    telemetry->balanceSpeedTarget = controllerTelemetry.output;
    telemetry->leftSpeedTarget = gApp.leftTarget;
    telemetry->leftActualSpeed = gApp.leftDelta;
    telemetry->leftSpeedPidOutputPwm = TB6612_GetAppliedPwm(MOTOR_LEFT);
    telemetry->rightSpeedTarget = gApp.rightTarget;
    telemetry->rightActualSpeed = gApp.rightDelta;
    telemetry->rightSpeedPidOutputPwm = TB6612_GetAppliedPwm(MOTOR_RIGHT);
    telemetry->enabled = (gApp.state == MOTOR_APP_BALANCE) &&
        controllerTelemetry.enabled;
    telemetry->fault = gApp.balanceFault;
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
