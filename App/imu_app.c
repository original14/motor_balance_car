#include "App/imu_app.h"

#include <stddef.h>
#include <stdint.h>

#include "App/motor_app.h"
#include "Config/balance_config.h"
#include "Config/imu_config.h"
#include "Hardware/bsp_icm42688.h"
#include "ti_msp_dl_config.h"

#if (IMU_ODR_HZ != BALANCE_CONTROL_HZ) || \
    (ATTITUDE_UPDATE_HZ != BALANCE_CONTROL_HZ) || \
    (BALANCE_CONTROL_HZ != 500U)
#error "IMU, attitude estimator, and balance controller must all run at 500 Hz"
#endif

bool IMUApp_Init(void)
{
    ICM42688_Sample firstSample;
    bool initialized;

    AttitudeEstimator_Init();
    initialized = ICM42688_Init();
    if (!initialized) return false;

    if (ICM42688_GetLatestSample(&firstSample)) {
        AttitudeEstimator_Update(firstSample.accel_x_g, firstSample.accel_y_g,
            firstSample.accel_z_g, firstSample.gyro_y_dps, firstSample.valid);
    }
    NVIC_ClearPendingIRQ(IMU_SAMPLE_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(IMU_SAMPLE_TIMER_INST_INT_IRQN);
    DL_TimerG_startCounter(IMU_SAMPLE_TIMER_INST);
    return true;
}

void IMUApp_Process(void)
{
    /* The deterministic attitude and balance path runs in the 500 Hz timer ISR. */
}

bool IMUApp_RequestLevelCalibration(void)
{
    uint32_t key;
    if (!ICM42688_IsValid() || MotorApp_IsBalanceEnabled()) return false;
    key = __get_PRIMASK();
    __disable_irq();
    AttitudeEstimator_RequestLevelCalibration();
    if (key == 0U) __enable_irq();
    return true;
}

bool IMUApp_GetAttitude(AttitudeEstimate *estimate)
{
    bool valid;
    uint32_t key;
    if (estimate == NULL) return false;
    key = __get_PRIMASK();
    __disable_irq();
    valid = AttitudeEstimator_GetLatest(estimate);
    if (key == 0U) __enable_irq();
    return valid;
}

bool IMUApp_EnableBalance(void)
{
    AttitudeEstimate attitude;
    bool calibrationActive;
    bool imuValid = ICM42688_IsValid();
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    (void)AttitudeEstimator_GetLatest(&attitude);
    calibrationActive = AttitudeEstimator_IsLevelCalibrationActive();
    if (key == 0U) __enable_irq();
    if (calibrationActive) return false;
    return MotorApp_EnableBalance(imuValid, attitude.attitude_valid,
        attitude.level_calibrated, attitude.pitch_angle_deg);
}

void IMUApp_DisableBalance(void)
{
    MotorApp_DisableBalance();
}

void IMU_SAMPLE_TIMER_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(IMU_SAMPLE_TIMER_INST) == DL_TIMER_IIDX_ZERO) {
        AttitudeEstimate attitude;
        ICM42688_Sample sample = {0};
        bool imuValid = ICM42688_ReadSample(&sample) && sample.valid;

        AttitudeEstimator_Update(sample.accel_x_g, sample.accel_y_g,
            sample.accel_z_g, sample.gyro_y_dps, imuValid);
        (void)AttitudeEstimator_GetLatest(&attitude);
        MotorApp_BalanceTick(imuValid, attitude.attitude_valid,
            attitude.level_calibrated, attitude.pitch_angle_deg,
            attitude.pitch_rate_dps);
    }
}
