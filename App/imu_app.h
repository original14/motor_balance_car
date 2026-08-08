#ifndef IMU_APP_H
#define IMU_APP_H

#include <stdbool.h>

#include "Control/attitude_estimator.h"

bool IMUApp_Init(void);
void IMUApp_Process(void);
bool IMUApp_RequestLevelCalibration(void);
bool IMUApp_GetAttitude(AttitudeEstimate *estimate);
bool IMUApp_EnableBalance(void);
void IMUApp_DisableBalance(void);

#endif
