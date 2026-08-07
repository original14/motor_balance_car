#include "Hardware/bsp_encoder.h"

#include "Config/motor_config.h"
#include "ti_msp_dl_config.h"

typedef struct {
    volatile int64_t totalCount;
    volatile int32_t intervalDelta;
    volatile uint8_t previousAB;
    volatile uint32_t invalidTransitionCount;
} EncoderState;

static EncoderState gLeftEncoder;
static EncoderState gRightEncoder;
static uint16_t gRightPreviousQeiCount;
static int64_t gRightTotalCount;
static const int8_t gQuadratureTable[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static uint8_t readLeftAB(void)
{
    uint32_t pins = DL_GPIO_readPins(ENCODER_GPIO_PORT,
        ENCODER_GPIO_LEFT_A_PIN | ENCODER_GPIO_LEFT_B_PIN);
    return (uint8_t)(((pins & ENCODER_GPIO_LEFT_A_PIN) ? 2U : 0U) |
        ((pins & ENCODER_GPIO_LEFT_B_PIN) ? 1U : 0U));
}

static void updateLeftEncoder(void)
{
    uint8_t current = readLeftAB();
    uint8_t index = (uint8_t)((gLeftEncoder.previousAB << 2) | current);
    int8_t step = gQuadratureTable[index];

    if (LEFT_ENCODER_REVERSE) step = (int8_t)-step;
    if (step != 0) {
        gLeftEncoder.totalCount += step;
        gLeftEncoder.intervalDelta += step;
    } else if (current != gLeftEncoder.previousAB) {
        gLeftEncoder.invalidTransitionCount++;
    }
    gLeftEncoder.previousAB = current;
}

static void updateRightEncoder(void)
{
    uint16_t current = (uint16_t)DL_TimerG_getTimerCount(RIGHT_ENCODER_QEI_INST);
    uint16_t moduloDelta = (uint16_t)(current - gRightPreviousQeiCount);
    int32_t delta = (moduloDelta <= 32767U) ? (int32_t)moduloDelta :
        ((int32_t)moduloDelta - 65536);

    gRightPreviousQeiCount = current;
    if (RIGHT_ENCODER_REVERSE) delta = -delta;
    gRightTotalCount += delta;
    gRightEncoder.intervalDelta += delta;

    if ((DL_TimerG_getRawInterruptStatus(RIGHT_ENCODER_QEI_INST,
             DL_TIMERG_INTERRUPT_QEI_ERR_EVENT) &
            DL_TIMERG_INTERRUPT_QEI_ERR_EVENT) != 0U) {
        gRightEncoder.invalidTransitionCount++;
        DL_TimerG_clearInterruptStatus(
            RIGHT_ENCODER_QEI_INST, DL_TIMERG_INTERRUPT_QEI_ERR_EVENT);
    }
}

static uint32_t enterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void leaveCritical(uint32_t primask)
{
    if (primask == 0U) __enable_irq();
}

void Encoder_Init(void)
{
    /* SysConfig selects TIMG8; add pull-ups and hysteresis for encoder inputs. */
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_RIGHT_ENCODER_QEI_PHA_IOMUX,
        GPIO_RIGHT_ENCODER_QEI_PHA_IOMUX_FUNC,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_RIGHT_ENCODER_QEI_PHB_IOMUX,
        GPIO_RIGHT_ENCODER_QEI_PHB_IOMUX_FUNC,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    Encoder_ResetAll();
    DL_GPIO_clearInterruptStatus(ENCODER_GPIO_PORT,
        ENCODER_GPIO_LEFT_A_PIN | ENCODER_GPIO_LEFT_B_PIN);
    NVIC_EnableIRQ(ENCODER_GPIO_INT_IRQN);
}

int64_t Encoder_GetTotalCount(EncoderChannel encoder)
{
    int64_t value;
    uint32_t key;
    if ((encoder != ENCODER_LEFT) && (encoder != ENCODER_RIGHT)) return 0;
    key = enterCritical();
    if (encoder == ENCODER_LEFT) {
        value = gLeftEncoder.totalCount;
    } else {
        updateRightEncoder();
        value = gRightTotalCount;
    }
    leaveCritical(key);
    return value;
}

int32_t Encoder_GetAndClearDelta(EncoderChannel encoder)
{
    int32_t value;
    uint32_t key;
    if ((encoder != ENCODER_LEFT) && (encoder != ENCODER_RIGHT)) return 0;
    key = enterCritical();
    if (encoder == ENCODER_LEFT) {
        value = gLeftEncoder.intervalDelta;
        gLeftEncoder.intervalDelta = 0;
    } else {
        updateRightEncoder();
        value = gRightEncoder.intervalDelta;
        gRightEncoder.intervalDelta = 0;
    }
    leaveCritical(key);
    return value;
}

uint32_t Encoder_GetInvalidTransitionCount(EncoderChannel encoder)
{
    uint32_t value, key;
    if ((encoder != ENCODER_LEFT) && (encoder != ENCODER_RIGHT)) return 0U;
    key = enterCritical();
    if (encoder == ENCODER_LEFT) {
        value = gLeftEncoder.invalidTransitionCount;
    } else {
        updateRightEncoder();
        value = gRightEncoder.invalidTransitionCount;
    }
    leaveCritical(key);
    return value;
}

void Encoder_Reset(EncoderChannel encoder)
{
    uint32_t key;
    if ((encoder != ENCODER_LEFT) && (encoder != ENCODER_RIGHT)) return;
    key = enterCritical();
    if (encoder == ENCODER_LEFT) {
        gLeftEncoder.totalCount = 0;
        gLeftEncoder.intervalDelta = 0;
        gLeftEncoder.invalidTransitionCount = 0;
        gLeftEncoder.previousAB = readLeftAB();
    } else {
        gRightEncoder.intervalDelta = 0;
        gRightEncoder.invalidTransitionCount = 0;
        gRightTotalCount = 0;
        DL_TimerG_stopCounter(RIGHT_ENCODER_QEI_INST);
        DL_TimerG_setTimerCount(RIGHT_ENCODER_QEI_INST, 0U);
        DL_TimerG_clearInterruptStatus(
            RIGHT_ENCODER_QEI_INST, DL_TIMERG_INTERRUPT_QEI_ERR_EVENT);
        gRightPreviousQeiCount = 0U;
        DL_TimerG_startCounter(RIGHT_ENCODER_QEI_INST);
    }
    leaveCritical(key);
}

void Encoder_ResetAll(void)
{
    Encoder_Reset(ENCODER_LEFT);
    Encoder_Reset(ENCODER_RIGHT);
}

void GROUP1_IRQHandler(void)
{
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(ENCODER_GPIO_PORT,
        ENCODER_GPIO_LEFT_A_PIN | ENCODER_GPIO_LEFT_B_PIN);

    if ((status & (ENCODER_GPIO_LEFT_A_PIN | ENCODER_GPIO_LEFT_B_PIN)) != 0U) {
        updateLeftEncoder();
    }
    DL_GPIO_clearInterruptStatus(ENCODER_GPIO_PORT, status);
}
