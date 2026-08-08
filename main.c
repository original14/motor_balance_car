#include "ti_msp_dl_config.h"

#include "App/imu_app.h"
#include "App/motor_app.h"
#include "App/serial_command.h"
#include "App/vofa_telemetry.h"
#include "Hardware/bsp_encoder.h"
#include "Hardware/bsp_tb6612.h"
#include "Hardware/bsp_uart.h"

int main(void)
{
    SYSCFG_DL_init();
    TB6612_Init();
    Encoder_Init();
    UART_Init();
    (void)IMUApp_Init();
    MotorApp_Init();
    while (1) {
        IMUApp_Process();
        SerialCommand_Process();
        MotorApp_Process();
        VOFATelemetry_Process();
        __WFI();
    }
}
