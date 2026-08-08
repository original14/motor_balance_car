#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include <stdbool.h>

typedef struct {
    float pitch_acc_raw_deg;
    float pitch_acc_deg;
    float pitch_rate_dps;
    float pitch_angle_deg;
    float pitch_zero_offset_deg;
    float accel_norm_g;
    bool attitude_initialized;
    bool attitude_valid;
    bool level_calibrated;
} AttitudeEstimate;

void AttitudeEstimator_Init(void);
void AttitudeEstimator_Update(float accel_x_g, float accel_y_g, float accel_z_g,
    float gyro_y_dps, bool imu_valid);
void AttitudeEstimator_RequestLevelCalibration(void);
bool AttitudeEstimator_GetLatest(AttitudeEstimate *estimate);
bool AttitudeEstimator_IsLevelCalibrationActive(void);

#endif
