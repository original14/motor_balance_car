#ifndef SERIAL_COMMAND_H
#define SERIAL_COMMAND_H
#include <stdint.h>
void SerialCommand_Process(void);
uint32_t SerialCommand_GetErrorCount(void);
#endif
