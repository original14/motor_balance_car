#ifndef SPEED_PID_H
#define SPEED_PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float integralError;
    float previousMeasurement;
    float derivativeFiltered;
    float outputMin;
    float outputMax;
    float dt;
} SpeedPID;

void SpeedPID_Init(SpeedPID *pid, float kp, float ki, float kd, float dt);
void SpeedPID_Reset(SpeedPID *pid);
void SpeedPID_SetTunings(SpeedPID *pid, float kp, float ki, float kd);
void SpeedPID_SetOutputLimits(SpeedPID *pid, float outputMin, float outputMax);
float SpeedPID_Update(SpeedPID *pid, float target, float measurement);
float SpeedPID_GetIntegralOutput(const SpeedPID *pid);

#endif
