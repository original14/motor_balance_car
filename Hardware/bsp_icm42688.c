#include "Hardware/bsp_icm42688.h"

#include <stddef.h>

#include "Config/imu_config.h"
#include "Hardware/icm42688_registers.h"
#include "ti_msp_dl_config.h"

static volatile ICM42688_Sample gLatestSample;
static volatile uint32_t gErrorCount;
static volatile uint8_t gWhoAmI;
static volatile bool gInitialized;

static void delayMicroseconds(uint32_t microseconds)
{
    delay_cycles((CPUCLK_FREQ / 1000000U) * microseconds);
}

static bool waitForTxSpace(void)
{
    uint32_t timeout = ICM42688_SPI_TIMEOUT_ITERATIONS;
    while (DL_SPI_isTXFIFOFull(IMU_SPI_INST)) {
        if (--timeout == 0U) return false;
    }
    return true;
}

static bool waitForRxData(void)
{
    uint32_t timeout = ICM42688_SPI_TIMEOUT_ITERATIONS;
    while (DL_SPI_isRXFIFOEmpty(IMU_SPI_INST)) {
        if (--timeout == 0U) return false;
    }
    return true;
}

static bool waitForIdle(void)
{
    uint32_t timeout = ICM42688_SPI_TIMEOUT_ITERATIONS;
    while (DL_SPI_isBusy(IMU_SPI_INST)) {
        if (--timeout == 0U) return false;
    }
    return true;
}

static void drainRxFifo(void)
{
    uint32_t remaining = 8U;
    while (!DL_SPI_isRXFIFOEmpty(IMU_SPI_INST) && (remaining > 0U)) {
        (void)DL_SPI_receiveData8(IMU_SPI_INST);
        remaining--;
    }
}

static bool exchangeByte(uint8_t transmit, uint8_t *receive)
{
    if (!waitForTxSpace()) return false;
    DL_SPI_transmitData8(IMU_SPI_INST, transmit);
    if (!waitForRxData()) return false;
    if (receive != NULL) {
        *receive = DL_SPI_receiveData8(IMU_SPI_INST);
    } else {
        (void)DL_SPI_receiveData8(IMU_SPI_INST);
    }
    return true;
}

static bool writeRegister(uint8_t address, uint8_t value)
{
    bool ok;
    drainRxFifo();
    DL_GPIO_clearPins(IMU_GPIO_PORT, IMU_GPIO_CS_PIN);
    ok = exchangeByte((uint8_t)(address & 0x7FU), NULL);
    if (ok) ok = exchangeByte(value, NULL);
    if (ok) ok = waitForIdle();
    DL_GPIO_setPins(IMU_GPIO_PORT, IMU_GPIO_CS_PIN);
    if (!ok) drainRxFifo();
    return ok;
}

static bool readRegisters(uint8_t address, uint8_t *data, size_t length)
{
    size_t index;
    bool ok;
    if ((data == NULL) || (length == 0U)) return false;

    drainRxFifo();
    DL_GPIO_clearPins(IMU_GPIO_PORT, IMU_GPIO_CS_PIN);
    ok = exchangeByte((uint8_t)(address | ICM42688_SPI_READ_BIT), NULL);
    for (index = 0U; ok && (index < length); ++index) {
        ok = exchangeByte(0U, &data[index]);
    }
    if (ok) ok = waitForIdle();
    DL_GPIO_setPins(IMU_GPIO_PORT, IMU_GPIO_CS_PIN);
    if (!ok) drainRxFifo();
    return ok;
}

static int16_t combineSigned16(uint8_t high, uint8_t low)
{
    return (int16_t)(((uint16_t)high << 8) | (uint16_t)low);
}

static void copySample(ICM42688_Sample *destination, const volatile ICM42688_Sample *source)
{
    destination->accel_x_raw = source->accel_x_raw;
    destination->accel_y_raw = source->accel_y_raw;
    destination->accel_z_raw = source->accel_z_raw;
    destination->gyro_x_raw = source->gyro_x_raw;
    destination->gyro_y_raw = source->gyro_y_raw;
    destination->gyro_z_raw = source->gyro_z_raw;
    destination->accel_x_g = source->accel_x_g;
    destination->accel_y_g = source->accel_y_g;
    destination->accel_z_g = source->accel_z_g;
    destination->gyro_x_dps = source->gyro_x_dps;
    destination->gyro_y_dps = source->gyro_y_dps;
    destination->gyro_z_dps = source->gyro_z_dps;
    destination->sample_count = source->sample_count;
    destination->valid = source->valid;
}

static void recordError(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    gErrorCount++;
    gLatestSample.valid = false;
    if (key == 0U) __enable_irq();
}

static void publishSample(const ICM42688_Sample *sample)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    gLatestSample.accel_x_raw = sample->accel_x_raw;
    gLatestSample.accel_y_raw = sample->accel_y_raw;
    gLatestSample.accel_z_raw = sample->accel_z_raw;
    gLatestSample.gyro_x_raw = sample->gyro_x_raw;
    gLatestSample.gyro_y_raw = sample->gyro_y_raw;
    gLatestSample.gyro_z_raw = sample->gyro_z_raw;
    gLatestSample.accel_x_g = sample->accel_x_g;
    gLatestSample.accel_y_g = sample->accel_y_g;
    gLatestSample.accel_z_g = sample->accel_z_g;
    gLatestSample.gyro_x_dps = sample->gyro_x_dps;
    gLatestSample.gyro_y_dps = sample->gyro_y_dps;
    gLatestSample.gyro_z_dps = sample->gyro_z_dps;
    gLatestSample.sample_count = sample->sample_count;
    gLatestSample.valid = sample->valid;
    if (key == 0U) __enable_irq();
}

bool ICM42688_ReadWhoAmI(uint8_t *who_am_i)
{
    uint8_t value;
    if (who_am_i == NULL) {
        recordError();
        return false;
    }
    if (!readRegisters(ICM42688_REG_WHO_AM_I, &value, 1U)) {
        recordError();
        return false;
    }
    gWhoAmI = value;
    *who_am_i = value;
    return true;
}

bool ICM42688_ReadSample(ICM42688_Sample *sample)
{
    uint8_t raw[ICM42688_SAMPLE_REGISTER_BYTES];
    ICM42688_Sample next;
    uint32_t nextCount;
    if ((sample == NULL) || !gInitialized) {
        recordError();
        return false;
    }
    if (!readRegisters(ICM42688_REG_ACCEL_DATA_X1, raw, sizeof(raw))) {
        recordError();
        return false;
    }

    next.accel_x_raw = combineSigned16(raw[0], raw[1]);
    next.accel_y_raw = combineSigned16(raw[2], raw[3]);
    next.accel_z_raw = combineSigned16(raw[4], raw[5]);
    next.gyro_x_raw = combineSigned16(raw[6], raw[7]);
    next.gyro_y_raw = combineSigned16(raw[8], raw[9]);
    next.gyro_z_raw = combineSigned16(raw[10], raw[11]);
    next.accel_x_g = (float)next.accel_x_raw / ICM42688_ACCEL_LSB_PER_G;
    next.accel_y_g = (float)next.accel_y_raw / ICM42688_ACCEL_LSB_PER_G;
    next.accel_z_g = (float)next.accel_z_raw / ICM42688_ACCEL_LSB_PER_G;
    next.gyro_x_dps = (float)next.gyro_x_raw / ICM42688_GYRO_LSB_PER_DPS;
    next.gyro_y_dps = (float)next.gyro_y_raw / ICM42688_GYRO_LSB_PER_DPS;
    next.gyro_z_dps = (float)next.gyro_z_raw / ICM42688_GYRO_LSB_PER_DPS;
    nextCount = gLatestSample.sample_count + 1U;
    next.sample_count = nextCount;
    next.valid = true;
    publishSample(&next);
    *sample = next;
    return true;
}

bool ICM42688_Init(void)
{
    uint8_t who_am_i = 0U;
    ICM42688_Sample firstSample;

    gErrorCount = 0U;
    gWhoAmI = 0U;
    gInitialized = false;
    gLatestSample.sample_count = 0U;
    gLatestSample.valid = false;
    DL_GPIO_setPins(IMU_GPIO_PORT, IMU_GPIO_CS_PIN);

    if (!writeRegister(ICM42688_REG_DEVICE_CONFIG, ICM42688_DEVICE_SOFT_RESET)) {
        recordError();
        return false;
    }
    delayMicroseconds(ICM42688_SOFT_RESET_WAIT_US);

    if (!ICM42688_ReadWhoAmI(&who_am_i)) return false;
    if (who_am_i != ICM42688_WHO_AM_I_VALUE) {
        recordError();
        return false;
    }
    if (!writeRegister(ICM42688_REG_GYRO_CONFIG0, ICM42688_GYRO_CONFIG_VALUE)) {
        recordError();
        return false;
    }
    if (!writeRegister(ICM42688_REG_ACCEL_CONFIG0, ICM42688_ACCEL_CONFIG_VALUE)) {
        recordError();
        return false;
    }
    if (!writeRegister(ICM42688_REG_PWR_MGMT0, ICM42688_PWR_MGMT0_LN_VALUE)) {
        recordError();
        return false;
    }
    delayMicroseconds(ICM42688_POWER_MODE_WRITE_GUARD_US);
    delayMicroseconds(ICM42688_GYRO_STARTUP_WAIT_US - ICM42688_POWER_MODE_WRITE_GUARD_US);

    gInitialized = true;
    if (!ICM42688_ReadSample(&firstSample)) {
        gInitialized = false;
        return false;
    }

    return true;
}

bool ICM42688_GetLatestSample(ICM42688_Sample *sample)
{
    uint32_t key;
    if (sample == NULL) return false;
    key = __get_PRIMASK();
    __disable_irq();
    copySample(sample, &gLatestSample);
    if (key == 0U) __enable_irq();
    return sample->valid;
}

bool ICM42688_IsValid(void)
{
    bool valid;
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    valid = gInitialized && gLatestSample.valid;
    if (key == 0U) __enable_irq();
    return valid;
}

uint8_t ICM42688_GetWhoAmI(void)
{
    return gWhoAmI;
}

uint32_t ICM42688_GetErrorCount(void)
{
    uint32_t count;
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    count = gErrorCount;
    if (key == 0U) __enable_irq();
    return count;
}
