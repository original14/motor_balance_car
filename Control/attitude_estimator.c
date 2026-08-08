#include "Control/attitude_estimator.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "Config/imu_config.h"

typedef struct {
    AttitudeEstimate estimate;
    float levelSum;
    uint32_t levelCount;
    uint32_t levelElapsedTicks;
    bool levelCalibrationActive;
} AttitudeEstimatorContext;

static AttitudeEstimatorContext gAttitude;

static bool accelNormIsUsable(float accelNorm)
{
    return (accelNorm >= ATTITUDE_ACCEL_NORM_MIN_G) &&
           (accelNorm <= ATTITUDE_ACCEL_NORM_MAX_G);
}

static void advanceCalibrationWindow(void)
{
    if (!gAttitude.levelCalibrationActive) return;
    gAttitude.levelElapsedTicks++;
    if (gAttitude.levelElapsedTicks >= IMU_LEVEL_CALIBRATION_MAX_TICKS) {
        gAttitude.levelCalibrationActive = false;
    }
}

void AttitudeEstimator_Init(void)
{
    gAttitude.estimate.pitch_acc_raw_deg = 0.0f;
    gAttitude.estimate.pitch_acc_deg = 0.0f;
    gAttitude.estimate.pitch_rate_dps = 0.0f;
    gAttitude.estimate.pitch_angle_deg = 0.0f;
    gAttitude.estimate.pitch_zero_offset_deg = IMU_DEFAULT_PITCH_ZERO_OFFSET_DEG;
    gAttitude.estimate.accel_norm_g = 0.0f;
    gAttitude.estimate.attitude_initialized = false;
    gAttitude.estimate.attitude_valid = false;
    gAttitude.estimate.level_calibrated = false;
    gAttitude.levelSum = 0.0f;
    gAttitude.levelCount = 0U;
    gAttitude.levelElapsedTicks = 0U;
    gAttitude.levelCalibrationActive = false;
}

void AttitudeEstimator_Update(float accel_x_g, float accel_y_g, float accel_z_g,
    float gyro_y_dps, bool imu_valid)
{
    float pitchPredictDeg;
    float alpha;
    bool accelUsable;

    advanceCalibrationWindow();
    if (!imu_valid || !isfinite(accel_x_g) || !isfinite(accel_y_g) ||
        !isfinite(accel_z_g) || !isfinite(gyro_y_dps)) {
        gAttitude.estimate.attitude_valid = false;
        return;
    }

    gAttitude.estimate.accel_norm_g =
        sqrtf((accel_x_g * accel_x_g) + (accel_y_g * accel_y_g) +
              (accel_z_g * accel_z_g));
    gAttitude.estimate.pitch_acc_raw_deg =
        atan2f(accel_x_g, -accel_z_g) * ATTITUDE_RAD_TO_DEG;
    gAttitude.estimate.pitch_rate_dps = IMU_PITCH_GYRO_SIGN * gyro_y_dps;
    gAttitude.estimate.pitch_acc_deg = gAttitude.estimate.pitch_acc_raw_deg -
        gAttitude.estimate.pitch_zero_offset_deg;

    if (!isfinite(gAttitude.estimate.accel_norm_g) ||
        !isfinite(gAttitude.estimate.pitch_acc_raw_deg) ||
        !isfinite(gAttitude.estimate.pitch_rate_dps) ||
        !isfinite(gAttitude.estimate.pitch_acc_deg)) {
        gAttitude.estimate.attitude_valid = false;
        return;
    }

    accelUsable = accelNormIsUsable(gAttitude.estimate.accel_norm_g);
    if (!gAttitude.estimate.attitude_initialized) {
        gAttitude.estimate.pitch_angle_deg = gAttitude.estimate.pitch_acc_deg;
        gAttitude.estimate.attitude_initialized = true;
    } else {
        alpha = ATTITUDE_COMPLEMENTARY_TAU_S /
            (ATTITUDE_COMPLEMENTARY_TAU_S + ATTITUDE_DT_S);
        pitchPredictDeg = gAttitude.estimate.pitch_angle_deg +
            (gAttitude.estimate.pitch_rate_dps * ATTITUDE_DT_S);
        if (accelUsable) {
            gAttitude.estimate.pitch_angle_deg = (alpha * pitchPredictDeg) +
                ((1.0f - alpha) * gAttitude.estimate.pitch_acc_deg);
        } else {
            gAttitude.estimate.pitch_angle_deg = pitchPredictDeg;
        }
    }

    gAttitude.estimate.attitude_valid =
        gAttitude.estimate.attitude_initialized &&
        isfinite(gAttitude.estimate.pitch_angle_deg);

    if (gAttitude.levelCalibrationActive && accelUsable &&
        (fabsf(gAttitude.estimate.pitch_rate_dps) < IMU_LEVEL_MAX_GYRO_DPS)) {
        gAttitude.levelSum += gAttitude.estimate.pitch_acc_raw_deg;
        gAttitude.levelCount++;
        if (gAttitude.levelCount >= IMU_LEVEL_CALIBRATION_SAMPLES) {
            gAttitude.estimate.pitch_zero_offset_deg =
                gAttitude.levelSum / (float)gAttitude.levelCount;
            gAttitude.estimate.pitch_acc_deg =
                gAttitude.estimate.pitch_acc_raw_deg -
                gAttitude.estimate.pitch_zero_offset_deg;
            gAttitude.estimate.pitch_angle_deg = gAttitude.estimate.pitch_acc_deg;
            gAttitude.estimate.level_calibrated = true;
            gAttitude.levelCalibrationActive = false;
        }
    }
}

void AttitudeEstimator_RequestLevelCalibration(void)
{
    gAttitude.levelSum = 0.0f;
    gAttitude.levelCount = 0U;
    gAttitude.levelElapsedTicks = 0U;
    gAttitude.levelCalibrationActive = true;
}

bool AttitudeEstimator_GetLatest(AttitudeEstimate *estimate)
{
    if (estimate == NULL) return false;
    *estimate = gAttitude.estimate;
    return estimate->attitude_valid;
}

bool AttitudeEstimator_IsLevelCalibrationActive(void)
{
    return gAttitude.levelCalibrationActive;
}
