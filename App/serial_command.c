#include "App/serial_command.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "App/imu_app.h"
#include "App/motor_app.h"
#include "Config/motor_config.h"
#include "Hardware/bsp_uart.h"

static char gLine[SERIAL_COMMAND_MAX_LEN + 1U];
static uint16_t gLineLength;
static bool gDiscardLine;
static uint32_t gCommandErrorCount;

#define BALANCE_RESPONSE_BUFFER_SIZE 128U

static bool appendResponseChar(char *buffer, size_t *length, char value)
{
    if (*length >= BALANCE_RESPONSE_BUFFER_SIZE) return false;
    buffer[(*length)++] = value;
    return true;
}

static bool appendResponseText(char *buffer, size_t *length, const char *text)
{
    while (*text != '\0') {
        if (!appendResponseChar(buffer, length, *text++)) return false;
    }
    return true;
}

static bool appendResponseUnsigned(char *buffer, size_t *length, uint32_t value)
{
    char reverse[10];
    size_t count = 0U;
    do {
        reverse[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);
    while (count > 0U) {
        if (!appendResponseChar(buffer, length, reverse[--count])) return false;
    }
    return true;
}

static bool appendResponseFloat4(char *buffer, size_t *length, float value)
{
    uint32_t scaled;
    if (!isfinite(value)) return false;
    if (value < 0.0f) {
        if (!appendResponseChar(buffer, length, '-')) return false;
        value = -value;
    }
    scaled = (uint32_t)(value * 10000.0f + 0.5f);
    if (!appendResponseUnsigned(buffer, length, scaled / 10000U)) return false;
    if (!appendResponseChar(buffer, length, '.')) return false;
    if (!appendResponseChar(buffer, length,
            (char)('0' + ((scaled / 1000U) % 10U)))) return false;
    if (!appendResponseChar(buffer, length,
            (char)('0' + ((scaled / 100U) % 10U)))) return false;
    if (!appendResponseChar(buffer, length,
            (char)('0' + ((scaled / 10U) % 10U)))) return false;
    return appendResponseChar(buffer, length, (char)('0' + (scaled % 10U)));
}

static void sendBalanceError(void)
{
    static const char response[] = "ERR\r\n";
    UART_Write(response, sizeof(response) - 1U);
}

static void sendBalanceSimpleOk(const char *command)
{
    char response[BALANCE_RESPONSE_BUFFER_SIZE];
    size_t length = 0U;
    bool ok = appendResponseText(response, &length, "OK BAL ") &&
        appendResponseText(response, &length, command) &&
        appendResponseText(response, &length, "\r\n");
    if (ok) UART_Write(response, length);
}

static void sendBalanceValueOk(const char *name, float value)
{
    char response[BALANCE_RESPONSE_BUFFER_SIZE];
    size_t length = 0U;
    bool ok = appendResponseText(response, &length, "OK BAL ") &&
        appendResponseText(response, &length, name) &&
        appendResponseChar(response, &length, '=') &&
        appendResponseFloat4(response, &length, value) &&
        appendResponseText(response, &length, "\r\n");
    if (ok) UART_Write(response, length);
}

static void sendBalancePidOk(void)
{
    char response[BALANCE_RESPONSE_BUFFER_SIZE];
    MotorBalanceTelemetry telemetry;
    size_t length = 0U;
    bool ok;
    MotorApp_GetBalanceTelemetry(&telemetry);
    ok = appendResponseText(response, &length, "OK BAL PID KP=") &&
        appendResponseFloat4(response, &length, telemetry.kp) &&
        appendResponseText(response, &length, " KI=") &&
        appendResponseFloat4(response, &length, telemetry.ki) &&
        appendResponseText(response, &length, " KD=") &&
        appendResponseFloat4(response, &length, telemetry.kd) &&
        appendResponseText(response, &length, "\r\n");
    if (ok) UART_Write(response, length);
}

static bool parseStrictFloat(const char *text, float *value)
{
    char *end;
    if ((text == NULL) || (value == NULL)) return false;
    *value = strtof(text, &end);
    return (end != text) && (*end == '\0') && isfinite(*value);
}

static bool parseBalancePid(const char *text, float *kp, float *ki, float *kd)
{
    char *end;
    *kp = strtof(text, &end);
    if ((end == text) || (*end != ' ') || !isfinite(*kp)) return false;
    while (*end == ' ') end++;
    text = end;
    *ki = strtof(text, &end);
    if ((end == text) || (*end != ' ') || !isfinite(*ki)) return false;
    while (*end == ' ') end++;
    text = end;
    *kd = strtof(text, &end);
    return (end != text) && (*end == '\0') && isfinite(*kd);
}

static bool executeBalanceLine(const char *line)
{
    MotorBalanceTelemetry telemetry;
    float kp, ki, kd, value;

    if (strcmp(line, "BAL ON") == 0) {
        if (!IMUApp_EnableBalance()) return false;
        sendBalanceSimpleOk("ON");
        return true;
    }
    if (strcmp(line, "BAL OFF") == 0) {
        IMUApp_DisableBalance();
        sendBalanceSimpleOk("OFF");
        return true;
    }
    if (strcmp(line, "BAL PID?") == 0) {
        sendBalancePidOk();
        return true;
    }
    if (strncmp(line, "BAL KP ", 7U) == 0) {
        if (!parseStrictFloat(line + 7, &value) ||
            !MotorApp_SetBalanceGain(BALANCE_PID_GAIN_KP, value)) return false;
        MotorApp_GetBalanceTelemetry(&telemetry);
        sendBalanceValueOk("KP", telemetry.kp);
        return true;
    }
    if (strncmp(line, "BAL KI ", 7U) == 0) {
        if (!parseStrictFloat(line + 7, &value) ||
            !MotorApp_SetBalanceGain(BALANCE_PID_GAIN_KI, value)) return false;
        MotorApp_GetBalanceTelemetry(&telemetry);
        sendBalanceValueOk("KI", telemetry.ki);
        return true;
    }
    if (strncmp(line, "BAL KD ", 7U) == 0) {
        if (!parseStrictFloat(line + 7, &value) ||
            !MotorApp_SetBalanceGain(BALANCE_PID_GAIN_KD, value)) return false;
        MotorApp_GetBalanceTelemetry(&telemetry);
        sendBalanceValueOk("KD", telemetry.kd);
        return true;
    }
    if (strncmp(line, "BAL PID ", 8U) == 0) {
        if (!parseBalancePid(line + 8, &kp, &ki, &kd) ||
            !MotorApp_SetBalancePid(kp, ki, kd)) return false;
        sendBalancePidOk();
        return true;
    }
    if (strncmp(line, "BAL TARGET ", 11U) == 0) {
        if (!parseStrictFloat(line + 11, &value) ||
            !MotorApp_SetBalanceTarget(value)) return false;
        MotorApp_GetBalanceTelemetry(&telemetry);
        sendBalanceValueOk("TARGET", telemetry.targetPitchDeg);
        return true;
    }
    return false;
}

static bool parsePid(const char *text, MotorChannel motor)
{
    char *end;
    float kp, ki, kd;
    kp = strtof(text, &end);
    if ((end == text) || (*end != ',') || !isfinite(kp)) return false;
    text = end + 1;
    ki = strtof(text, &end);
    if ((end == text) || (*end != ',') || !isfinite(ki)) return false;
    text = end + 1;
    kd = strtof(text, &end);
    if ((end == text) || (*end != '\0') || !isfinite(kd)) return false;
    return MotorApp_SetPidTunings(motor, kp, ki, kd);
}

static bool parseTarget(const char *text, MotorChannel motor)
{
    char *end;
    float targetValue = strtof(text, &end);
    if ((end == text) || (*end != '\0') || !isfinite(targetValue)) return false;
    return MotorApp_SetTargetValue(motor, targetValue);
}

static bool parsePidGain(const char *text, MotorChannel motor, MotorPidGain gain)
{
    char *end;
    float value = strtof(text, &end);
    if ((end == text) || (*end != '\0') || !isfinite(value)) return false;
    return MotorApp_SetPidGain(motor, gain, value);
}

static bool executeLine(const char *line)
{
    if (strcmp(line, "STOP") == 0) {
        MotorApp_EmergencyStop();
        return true;
    }
    if ((strcmp(line, "LEVEL") == 0) || (strcmp(line, "IMUZERO") == 0)) {
        return IMUApp_RequestLevelCalibration();
    }
    if (strncmp(line, "BAL", 3U) == 0) {
        bool ok = executeBalanceLine(line);
        if (!ok) sendBalanceError();
        return ok;
    }
    if (strncmp(line, "LPID,", 5U) == 0) return parsePid(line + 5, MOTOR_LEFT);
    if (strncmp(line, "RPID,", 5U) == 0) return parsePid(line + 5, MOTOR_RIGHT);
    if (strncmp(line, "LKP,", 4U) == 0)
        return parsePidGain(line + 4, MOTOR_LEFT, MOTOR_PID_GAIN_KP);
    if (strncmp(line, "LKI,", 4U) == 0)
        return parsePidGain(line + 4, MOTOR_LEFT, MOTOR_PID_GAIN_KI);
    if (strncmp(line, "LKD,", 4U) == 0)
        return parsePidGain(line + 4, MOTOR_LEFT, MOTOR_PID_GAIN_KD);
    if (strncmp(line, "RKP,", 4U) == 0)
        return parsePidGain(line + 4, MOTOR_RIGHT, MOTOR_PID_GAIN_KP);
    if (strncmp(line, "RKI,", 4U) == 0)
        return parsePidGain(line + 4, MOTOR_RIGHT, MOTOR_PID_GAIN_KI);
    if (strncmp(line, "RKD,", 4U) == 0)
        return parsePidGain(line + 4, MOTOR_RIGHT, MOTOR_PID_GAIN_KD);
    if (strncmp(line, "LSPD,", 5U) == 0) return parseTarget(line + 5, MOTOR_LEFT);
    if (strncmp(line, "RSPD,", 5U) == 0) return parseTarget(line + 5, MOTOR_RIGHT);
    return false;
}

void SerialCommand_Process(void)
{
    uint8_t byte;
    while (UART_ReadByte(&byte)) {
        if ((byte == '\r') || (byte == '\n')) {
            if (gDiscardLine) {
                gDiscardLine = false;
                gLineLength = 0U;
            } else if (gLineLength > 0U) {
                gLine[gLineLength] = '\0';
                if (!executeLine(gLine)) gCommandErrorCount++;
                gLineLength = 0U;
            }
        } else if (!gDiscardLine) {
            if (gLineLength < SERIAL_COMMAND_MAX_LEN) {
                gLine[gLineLength++] = (char)byte;
            } else {
                gDiscardLine = true;
                gLineLength = 0U;
                gCommandErrorCount++;
            }
        }
    }
}

uint32_t SerialCommand_GetErrorCount(void)
{
    return gCommandErrorCount + UART_GetRxOverflowCount();
}
