#include "Control/balance_controller.h"

#include <math.h>
#include <stddef.h>

#include "Config/balance_config.h"

static float clampValue(float value, float minimum, float maximum)
{
    if (value > maximum) return maximum;
    if (value < minimum) return minimum;
    return value;
}

static bool gainsValid(float kp, float ki, float kd)
{
    return isfinite(kp) && isfinite(ki) && isfinite(kd) &&
        (kp >= BALANCE_KP_MIN) && (kp <= BALANCE_KP_MAX) &&
        (ki >= BALANCE_KI_MIN) && (ki <= BALANCE_KI_MAX) &&
        (kd >= BALANCE_KD_MIN) && (kd <= BALANCE_KD_MAX);
}

void BalanceController_Init(BalanceController *controller)
{
    if (controller == NULL) return;
    controller->kp = BALANCE_KP_DEFAULT;
    controller->ki = BALANCE_KI_DEFAULT;
    controller->kd = BALANCE_KD_DEFAULT;
    controller->target_angle_deg = BALANCE_TARGET_DEFAULT_DEG;
    controller->output_limit = BALANCE_OUTPUT_LIMIT;
    controller->integral_limit = BALANCE_INTEGRAL_LIMIT;
    controller->enabled = false;
    BalanceController_Reset(controller);
}

void BalanceController_Reset(BalanceController *controller)
{
    if (controller == NULL) return;
    controller->pitch_angle_deg = 0.0f;
    controller->pitch_rate_dps = 0.0f;
    controller->error_deg = 0.0f;
    controller->p_output = 0.0f;
    controller->i_output = 0.0f;
    controller->d_output = 0.0f;
    controller->integral = 0.0f;
    controller->output_unclamped = 0.0f;
    controller->output = 0.0f;
}

void BalanceController_Enable(BalanceController *controller)
{
    if (controller == NULL) return;
    BalanceController_Reset(controller);
    controller->enabled = true;
}

void BalanceController_Disable(BalanceController *controller)
{
    if (controller == NULL) return;
    controller->enabled = false;
    BalanceController_Reset(controller);
}

bool BalanceController_SetPID(BalanceController *controller, float kp, float ki, float kd)
{
    if ((controller == NULL) || !gainsValid(kp, ki, kd)) return false;
    if (ki != controller->ki) {
        controller->integral = 0.0f;
        controller->i_output = 0.0f;
    }
    controller->kp = kp;
    controller->ki = ki;
    controller->kd = kd;
    return true;
}

bool BalanceController_SetTargetAngle(BalanceController *controller, float target_angle_deg)
{
    if ((controller == NULL) || !isfinite(target_angle_deg) ||
        (target_angle_deg < BALANCE_TARGET_MIN_DEG) ||
        (target_angle_deg > BALANCE_TARGET_MAX_DEG)) return false;
    controller->target_angle_deg = target_angle_deg;
    return true;
}

float BalanceController_Update(BalanceController *controller, float pitch_angle_deg,
    float pitch_rate_dps, float dt)
{
    float candidateIntegral;
    float candidateIOutput;
    float candidateOutput;
    bool drivesFurtherIntoSaturation;

    if ((controller == NULL) || !controller->enabled || !isfinite(pitch_angle_deg) ||
        !isfinite(pitch_rate_dps) || !isfinite(dt) || (dt <= 0.0f)) return 0.0f;

    controller->pitch_angle_deg = pitch_angle_deg;
    controller->pitch_rate_dps = pitch_rate_dps;
    controller->error_deg = controller->target_angle_deg - pitch_angle_deg;
    controller->p_output = controller->kp * controller->error_deg;
    controller->d_output = -controller->kd * pitch_rate_dps;

    candidateIntegral = clampValue(controller->integral +
        (controller->error_deg * dt), -controller->integral_limit,
        controller->integral_limit);
    candidateIOutput = controller->ki * candidateIntegral;
    candidateOutput = controller->p_output + candidateIOutput + controller->d_output;
    drivesFurtherIntoSaturation =
        ((candidateOutput > controller->output_limit) &&
         (controller->error_deg > 0.0f)) ||
        ((candidateOutput < -controller->output_limit) &&
         (controller->error_deg < 0.0f));
    if (!drivesFurtherIntoSaturation) controller->integral = candidateIntegral;

    controller->i_output = controller->ki * controller->integral;
    controller->output_unclamped =
        controller->p_output + controller->i_output + controller->d_output;
    controller->output = clampValue(controller->output_unclamped,
        -controller->output_limit, controller->output_limit);
    return controller->output;
}

void BalanceController_GetTelemetry(const BalanceController *controller,
    BalanceControllerTelemetry *telemetry)
{
    if ((controller == NULL) || (telemetry == NULL)) return;
    telemetry->kp = controller->kp;
    telemetry->ki = controller->ki;
    telemetry->kd = controller->kd;
    telemetry->target_angle_deg = controller->target_angle_deg;
    telemetry->pitch_angle_deg = controller->pitch_angle_deg;
    telemetry->pitch_rate_dps = controller->pitch_rate_dps;
    telemetry->error_deg = controller->error_deg;
    telemetry->p_output = controller->p_output;
    telemetry->i_output = controller->i_output;
    telemetry->d_output = controller->d_output;
    telemetry->output_unclamped = controller->output_unclamped;
    telemetry->output = controller->output;
    telemetry->enabled = controller->enabled;
}
