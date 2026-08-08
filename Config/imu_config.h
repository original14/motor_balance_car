#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

/* ICM42688-P operating point used by the first-stage raw data driver. */
#define IMU_ACCEL_RANGE_G                    4U
#define IMU_GYRO_RANGE_DPS                 500U
#define IMU_ODR_HZ                         500U
#define ICM42688_SPI_FREQUENCY_HZ      1000000U
#define ICM42688_SPI_TIMEOUT_ITERATIONS  10000U

/* DS-000347 register encodings for the operating point above. */
#define ICM42688_ACCEL_FS_SELECT              2U
#define ICM42688_GYRO_FS_SELECT               2U
#define ICM42688_ODR_SELECT                 0x0FU

/* DS-000347 Rev. 1.9 timing requirements, with margin where appropriate. */
#define ICM42688_SOFT_RESET_WAIT_US        2000U
#define ICM42688_POWER_MODE_WRITE_GUARD_US 200U
#define ICM42688_GYRO_STARTUP_WAIT_US      45000U

/* 16-bit sensor-register sensitivity at +/-4 g and +/-500 dps. */
#define ICM42688_ACCEL_LSB_PER_G            8192.0f
#define ICM42688_GYRO_LSB_PER_DPS             65.536f

#endif
