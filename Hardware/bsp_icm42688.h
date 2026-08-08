#ifndef BSP_ICM42688_H
#define BSP_ICM42688_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    uint32_t sample_count;
    bool valid;
} ICM42688_Sample;

bool ICM42688_Init(void);
bool ICM42688_ReadWhoAmI(uint8_t *who_am_i);
bool ICM42688_ReadSample(ICM42688_Sample *sample);
bool ICM42688_GetLatestSample(ICM42688_Sample *sample);
bool ICM42688_IsValid(void);
uint8_t ICM42688_GetWhoAmI(void);
uint32_t ICM42688_GetErrorCount(void);

#endif
