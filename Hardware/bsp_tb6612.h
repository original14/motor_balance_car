#ifndef BSP_TB6612_H
#define BSP_TB6612_H

#include <stdint.h>

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT
} MotorChannel;

void TB6612_Init(void);
void TB6612_SetSignedPwm(MotorChannel motor, float signedPwmCommand);
void TB6612_StopMotor(MotorChannel motor);
void TB6612_BrakeMotor(MotorChannel motor);
void TB6612_CoastMotor(MotorChannel motor);
void TB6612_EnableDriver(void);
void TB6612_DisableDriver(void);
void TB6612_EmergencyStopAll(void);
int32_t TB6612_GetAppliedPwm(MotorChannel motor);

#endif
