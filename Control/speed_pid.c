#include "Control/speed_pid.h"

#include <stddef.h>

static float clampValue(float value, float minimum, float maximum)
{
    if (value > maximum) return maximum;
    if (value < minimum) return minimum;
    return value;
}

static float clampIntegralError(const SpeedPID *pid, float integralError)
{
    if ((pid == NULL) || (pid->ki <= 0.0f)) return 0.0f;
    return clampValue(integralError, pid->outputMin / pid->ki, pid->outputMax / pid->ki);
}

void SpeedPID_Init(SpeedPID *pid, float kp, float ki, float kd, float dt)
{
    if (pid == NULL) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->outputMin = -100.0f;
    pid->outputMax = 100.0f;
    SpeedPID_Reset(pid);
}

void SpeedPID_Reset(SpeedPID *pid)
{
    if (pid == NULL) return;
    pid->integralError = 0.0f;
    pid->previousMeasurement = 0.0f;
    pid->derivativeFiltered = 0.0f;
}

void SpeedPID_SetTunings(SpeedPID *pid, float kp, float ki, float kd)
{
    if (pid == NULL) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void SpeedPID_SetOutputLimits(SpeedPID *pid, float outputMin, float outputMax)
{
    if ((pid == NULL) || (outputMin >= outputMax)) return;
    pid->outputMin = outputMin;
    pid->outputMax = outputMax;
    pid->integralError = clampIntegralError(pid, pid->integralError);
}

float SpeedPID_Update(SpeedPID *pid, float target, float measurement)
{
    float error, derivative, candidateIntegralError, candidateIntegralOutput, output;
    const float derivativeAlpha = 0.20f;
    if ((pid == NULL) || (pid->dt <= 0.0f)) return 0.0f;

    error = target - measurement;
    derivative = -(measurement - pid->previousMeasurement) / pid->dt;
    pid->previousMeasurement = measurement;
    pid->derivativeFiltered += derivativeAlpha * (derivative - pid->derivativeFiltered);

    candidateIntegralError = clampIntegralError(pid, pid->integralError + error * pid->dt);
    candidateIntegralOutput = pid->ki * candidateIntegralError;
    output = pid->kp * error + candidateIntegralOutput + pid->kd * pid->derivativeFiltered;

    /* 条件积分 anti-windup：饱和且误差继续推向饱和方向时拒绝本次积分。 */
    if (!((output > pid->outputMax && error > 0.0f) ||
          (output < pid->outputMin && error < 0.0f))) {
        pid->integralError = candidateIntegralError;
    }
    output = pid->kp * error + SpeedPID_GetIntegralOutput(pid) +
        pid->kd * pid->derivativeFiltered;
    return clampValue(output, pid->outputMin, pid->outputMax);
}

float SpeedPID_GetIntegralOutput(const SpeedPID *pid)
{
    if (pid == NULL) return 0.0f;
    return clampValue(pid->ki * pid->integralError, pid->outputMin, pid->outputMax);
}
