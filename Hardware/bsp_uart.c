#include "Hardware/bsp_uart.h"

#include "Config/motor_config.h"
#include "ti_msp_dl_config.h"

static volatile uint8_t gRxBuffer[SERIAL_RX_BUFFER_SIZE];
static volatile uint16_t gRxHead;
static volatile uint16_t gRxTail;
static volatile uint32_t gRxOverflowCount;

void UART_Init(void)
{
    gRxHead = 0U;
    gRxTail = 0U;
    gRxOverflowCount = 0U;
    NVIC_EnableIRQ(DEBUG_UART_INST_INT_IRQN);
}

bool UART_ReadByte(uint8_t *byte)
{
    uint32_t key;
    if (byte == NULL) return false;
    key = __get_PRIMASK();
    __disable_irq();
    if (gRxTail == gRxHead) {
        if (key == 0U) __enable_irq();
        return false;
    }
    *byte = gRxBuffer[gRxTail];
    gRxTail = (uint16_t)((gRxTail + 1U) % SERIAL_RX_BUFFER_SIZE);
    if (key == 0U) __enable_irq();
    return true;
}

void UART_Write(const char *data, size_t length)
{
    size_t i;
    if (data == NULL) return;
    for (i = 0U; i < length; ++i) {
        while (DL_UART_Main_isTXFIFOFull(DEBUG_UART_INST)) {
            /* 控制环在高优先级定时器 ISR 中，不会被主循环发送阻塞。 */
        }
        DL_UART_Main_transmitData(DEBUG_UART_INST, (uint8_t)data[i]);
    }
}

uint32_t UART_GetRxOverflowCount(void)
{
    return gRxOverflowCount;
}

void DEBUG_UART_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(DEBUG_UART_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(DEBUG_UART_INST)) {
                uint8_t byte = DL_UART_Main_receiveData(DEBUG_UART_INST);
                uint16_t next = (uint16_t)((gRxHead + 1U) % SERIAL_RX_BUFFER_SIZE);
                if (next == gRxTail) {
                    gRxOverflowCount++;
                } else {
                    gRxBuffer[gRxHead] = byte;
                    gRxHead = next;
                }
            }
            break;
        default:
            break;
    }
}
