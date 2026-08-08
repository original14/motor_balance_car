#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <stdbool.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float target_angle_deg;
    float pitch_angle_deg;
    float pitch_rate_dps;
    float error_deg;
    float p_output;
    float i_output;
    float d_output;
    float integral;
    float output_unclamped;
    float output;
    float output_limit;
    float integral_limit;
    bool enabled;
} BalanceController;

typedef struct {
    float kp;
    float ki;
    float kd;
    float target_angle_deg;
    float pitch_angle_deg;
    float pitch_rate_dps;
    float error_deg;
    float p_output;
    float i_output;
    float d_output;
    float output_unclamped;
    float output;
    bool enabled;
} BalanceControllerTelemetry;

void BalanceController_Init(BalanceController *controller);
void BalanceController_Reset(BalanceController *controller);
void BalanceController_Enable(BalanceController *controller);
void BalanceController_Disable(BalanceController *controller);
bool BalanceController_SetPID(BalanceController *controller, float kp, float ki, float kd);
bool BalanceController_SetTargetAngle(BalanceController *controller, float target_angle_deg);
float BalanceController_Update(BalanceController *controller, float pitch_angle_deg,
    float pitch_rate_dps, float dt);
void BalanceController_GetTelemetry(const BalanceController *controller,
    BalanceControllerTelemetry *telemetry);

#endif
