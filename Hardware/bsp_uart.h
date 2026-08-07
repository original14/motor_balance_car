#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void UART_Init(void);
bool UART_ReadByte(uint8_t *byte);
void UART_Write(const char *data, size_t length);
uint32_t UART_GetRxOverflowCount(void);

#endif
