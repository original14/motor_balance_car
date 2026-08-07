#include "App/serial_command.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "App/motor_app.h"
#include "Config/motor_config.h"
#include "Hardware/bsp_uart.h"

static char gLine[SERIAL_COMMAND_MAX_LEN + 1U];
static uint16_t gLineLength;
static bool gDiscardLine;
static uint32_t gCommandErrorCount;

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
