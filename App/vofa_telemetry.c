#include "App/vofa_telemetry.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "App/motor_app.h"
#include "Hardware/bsp_uart.h"

#define VOFA_TX_BUFFER_SIZE 256U

static bool appendChar(char *buffer, size_t *length, char value)
{
    if (*length >= VOFA_TX_BUFFER_SIZE) return false;
    buffer[(*length)++] = value;
    return true;
}

static bool appendUnsigned(char *buffer, size_t *length, uint64_t value)
{
    char reverse[21];
    size_t count = 0U;
    do {
        reverse[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);
    while (count > 0U) {
        if (!appendChar(buffer, length, reverse[--count])) return false;
    }
    return true;
}

static bool appendInteger(char *buffer, size_t *length, int64_t value)
{
    uint64_t magnitude;
    if (value < 0) {
        if (!appendChar(buffer, length, '-')) return false;
        magnitude = (uint64_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint64_t)value;
    }
    return appendUnsigned(buffer, length, magnitude);
}

static bool appendFixed2(char *buffer, size_t *length, float value)
{
    uint64_t scaled;
    if (!isfinite(value)) value = 0.0f;
    if (value < 0.0f) {
        if (!appendChar(buffer, length, '-')) return false;
        value = -value;
    }
    scaled = (uint64_t)(value * 100.0f + 0.5f);
    if (!appendUnsigned(buffer, length, scaled / 100U)) return false;
    if (!appendChar(buffer, length, '.')) return false;
    if (!appendChar(buffer, length, (char)('0' + ((scaled / 10U) % 10U)))) return false;
    return appendChar(buffer, length, (char)('0' + (scaled % 10U)));
}

static bool appendFixed4(char *buffer, size_t *length, float value)
{
    uint64_t scaled;
    if (!isfinite(value)) value = 0.0f;
    if (value < 0.0f) {
        if (!appendChar(buffer, length, '-')) return false;
        value = -value;
    }
    scaled = (uint64_t)(value * 10000.0f + 0.5f);
    if (!appendUnsigned(buffer, length, scaled / 10000U)) return false;
    if (!appendChar(buffer, length, '.')) return false;
    if (!appendChar(buffer, length, (char)('0' + ((scaled / 1000U) % 10U)))) return false;
    if (!appendChar(buffer, length, (char)('0' + ((scaled / 100U) % 10U)))) return false;
    if (!appendChar(buffer, length, (char)('0' + ((scaled / 10U) % 10U)))) return false;
    return appendChar(buffer, length, (char)('0' + (scaled % 10U)));
}

void VOFATelemetry_Process(void)
{
    static char buffer[VOFA_TX_BUFFER_SIZE];
    MotorTelemetry data;
    size_t length = 0U;
    bool ok = true;
    if (!MotorApp_TakeTelemetryFlag()) return;
    MotorApp_GetTelemetry(&data);
    ok &= appendFixed2(buffer, &length, data.leftTargetValue);
    ok &= appendChar(buffer, &length, ',');
    ok &= appendInteger(buffer, &length, data.leftPidFeedbackDelta);
    ok &= appendChar(buffer, &length, ',');
    ok &= appendInteger(buffer, &length, data.leftPwmOutput);
    ok &= appendChar(buffer, &length, ',');
    ok &= appendInteger(buffer, &length, data.leftEncoderTotal);
    ok &= appendChar(buffer, &length, ',');
    ok &= appendFixed4(buffer, &length, data.leftKp);
    ok &= appendChar(buffer, &length, ',');
    ok &= appendFixed4(buffer, &length, data.leftKi);
    ok &= appendChar(buffer, &length, ',');
    ok &= appendFixed4(buffer, &length, data.leftKd);
    ok &= appendChar(buffer, &length, ',');
    ok &= appendFixed2(buffer, &length, data.leftIntegralOutput);
    ok &= appendChar(buffer, &length, ',');
    ok &= appendFixed2(buffer, &length, data.rightTargetValue);
    ok &= appendChar(buffer, &length, ',');
    ok &= appendInteger(buffer, &length, data.rightPidFeedbackDelta);
    ok &= appendChar(buffer, &length, ',');
    ok &= appendInteger(buffer, &length, data.rightPwmOutput);
    ok &= appendChar(buffer, &length, ',');
    ok &= appendInteger(buffer, &length, data.rightEncoderTotal);
    ok &= appendChar(buffer, &length, ',');
    ok &= appendFixed4(buffer, &length, data.rightKp);
    ok &= appendChar(buffer, &length, ',');
    ok &= appendFixed4(buffer, &length, data.rightKi);
    ok &= appendChar(buffer, &length, ',');
    ok &= appendFixed4(buffer, &length, data.rightKd);
    ok &= appendChar(buffer, &length, ',');
    ok &= appendFixed2(buffer, &length, data.rightIntegralOutput);
    ok &= appendChar(buffer, &length, '\r');
    ok &= appendChar(buffer, &length, '\n');
    if (ok) UART_Write(buffer, length);
}
