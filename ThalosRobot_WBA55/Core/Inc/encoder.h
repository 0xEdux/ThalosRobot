/*
 * encoder.h
 *
 *  Created on: 22 nov 2026
 *      Author: juanb
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "main.h"

typedef struct {
    uint16_t pin;
    uint16_t slots;

    volatile uint32_t buffer[4];
    volatile uint8_t buffer_idx;
    volatile uint8_t buffer_size;
    volatile uint32_t last_time;
} Encoder_Handler_t;

void Encoder_Init(Encoder_Handler_t *encoder, uint16_t pin, uint16_t slots);
void Encoder_OnExti(Encoder_Handler_t *encoder, uint16_t GPIO_Pin);
float Encoder_GetRPM(Encoder_Handler_t *encoder);


#endif /* INC_ENCODER_H_ */
