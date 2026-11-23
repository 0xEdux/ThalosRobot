/*
 * encoder.c
 *
 *  Created on: 22 nov 2026
 *      Author: juanb
 */

#include "encoder.h"

void Encoder_Init(Encoder_Handler_t *encoder, uint16_t pin, uint16_t slots) {
    if(encoder == NULL) return;

    encoder->buffer_idx = 0;
    encoder->buffer_size = 0;
    encoder->last_time = 0;
    encoder->pin = pin;
    encoder->slots = slots;
}

void Encoder_OnExti(Encoder_Handler_t *encoder, uint16_t GPIO_Pin) {
    if(encoder == NULL) return;

    uint32_t now = DWT->CYCCNT;

    if(GPIO_Pin == encoder->pin) {
        // Debounce filter 5ms
        if(encoder->buffer_size > 0 && (now - encoder->last_time) < (SystemCoreClock / 200U)) return;

        encoder->buffer[encoder->buffer_idx] = now;

        encoder->buffer_idx = (encoder->buffer_idx + 1) & 3;

        if(encoder->buffer_size < 4) encoder->buffer_size++;
        encoder->last_time = now;
    }
}

float Encoder_GetRPM(Encoder_Handler_t *encoder) {
    if(encoder == NULL) return 0;
    
    // Stop interrupts
    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    uint8_t n = encoder->buffer_size;
    uint8_t i = encoder->buffer_idx;
    uint32_t last_time = encoder->last_time;


    // "... & 3" -> "... % 4"
    uint32_t newT = encoder->buffer[(i + 3) & 3];
    uint32_t oldT = encoder->buffer[(i + 4 - n) & 3];

    // resume interrupts
    __set_PRIMASK(prim);

    // no pulses in 150ms gap -> stop

    if((DWT->CYCCNT - last_time) > (uint32_t)(SystemCoreClock * 0.15f)) {
        // Stop interrupts;
        prim = __get_PRIMASK();
        __disable_irq();
        encoder->buffer_size = 0;
        // Resume interrupts;
        __set_PRIMASK(prim);

        return 0.0f;
    }

    // return if less than two samples
    if (n < 2) return 0.0f;


    // RPM = [60 * F_cpu * (n - 1)] / [N_slots * Delta_ticks]
    return 60.0f * (float)SystemCoreClock * (n - 1) / ((float)encoder->slots * (float)(newT - oldT));
}
