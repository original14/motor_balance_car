#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdint.h>

typedef enum {
    ENCODER_LEFT = 0,
    ENCODER_RIGHT
} EncoderChannel;

void Encoder_Init(void);
int64_t Encoder_GetTotalCount(EncoderChannel encoder);
int32_t Encoder_GetAndClearDelta(EncoderChannel encoder);
uint32_t Encoder_GetInvalidTransitionCount(EncoderChannel encoder);
void Encoder_Reset(EncoderChannel encoder);
void Encoder_ResetAll(void);

#endif
