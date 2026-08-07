#ifndef MOTOR_APP_H
#define MOTOR_APP_H

#include <stdbool.h>
#include <stdint.h>

#include "Hardware/bsp_tb6612.h"

typedef enum {
    MOTOR_APP_SAFE = 0,
    MOTOR_APP_ENCODER_TEST,
    MOTOR_APP_DIRECTION_TEST,
    MOTOR_APP_PID_TEST,
    MOTOR_APP_FAULT
} MotorAppState;

typedef enum {
    MOTOR_PID_GAIN_KP = 0,
    MOTOR_PID_GAIN_KI,
    MOTOR_PID_GAIN_KD
} MotorPidGain;

typedef struct {
    float leftTargetValue;
    int32_t leftPidFeedbackDelta;
    int32_t leftPwmOutput;
    int64_t leftEncoderTotal;
    float leftKp;
    float leftKi;
    float leftKd;
    float leftIntegralOutput;
    float rightTargetValue;
    int32_t rightPidFeedbackDelta;
    int32_t rightPwmOutput;
    int64_t rightEncoderTotal;
    float rightKp;
    float rightKi;
    float rightKd;
    float rightIntegralOutput;
} MotorTelemetry;

void MotorApp_Init(void);
void MotorApp_Process(void);
void MotorApp_ControlTick(void);
bool MotorApp_SetPidTunings(MotorChannel motor, float kp, float ki, float kd);
bool MotorApp_SetPidGain(MotorChannel motor, MotorPidGain gain, float value);
bool MotorApp_SetTargetValue(MotorChannel motor, float targetValue);
void MotorApp_EmergencyStop(void);
MotorAppState MotorApp_GetState(void);
void MotorApp_GetTelemetry(MotorTelemetry *telemetry);
bool MotorApp_TakeTelemetryFlag(void);

#endif
