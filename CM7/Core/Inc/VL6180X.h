/*
 * VL6180X.h
 *
 *  Created on: 24 nov 2026
 *      Author: juanb
 */

#ifndef INC_VL6180X_H_
#define INC_VL6180X_H_



#include <stddef.h>
#include <stdint.h>
#include "main.h"
#include "cmsis_os2.h"


typedef struct {
    I2C_HandleTypeDef *i2c;
    uint8_t address_7bit;
    osSemaphoreId_t ready_sem;   // binario, cuenta inicial 0; lo libera el EXTI de GPIO1
} VL6180X_t;

typedef struct {
    uint16_t distance_mm;        // 0xFFFF si la medicion no es valida
    uint8_t range_status;        // 0 = OK, 0xFF = sin dato
} VL6180X_Range_t;

// Inicializa la estructura y crea su semaforo. Llamar desde una tarea (o antes de osKernelStart).
HAL_StatusTypeDef VL6180X_Create(VL6180X_t *sensor, I2C_HandleTypeDef *i2c, uint8_t address_7bit);

HAL_StatusTypeDef VL6180X_SetAddress(VL6180X_t *sensor, uint8_t new_address_7bit);
HAL_StatusTypeDef VL6180X_Init(const VL6180X_t *sensor);

// Medicion en tres pasos
HAL_StatusTypeDef VL6180X_StartRange(const VL6180X_t *sensor);
HAL_StatusTypeDef VL6180X_WaitRangeReady(const VL6180X_t *sensor);
HAL_StatusTypeDef VL6180X_CollectRange(const VL6180X_t *sensor, VL6180X_Range_t *result);

// Medicion completa de un sensor
HAL_StatusTypeDef VL6180X_ReadRange(const VL6180X_t *sensor, VL6180X_Range_t *result);

// Medicion de varios sensores en paralelo
HAL_StatusTypeDef VL6180X_ReadRangeMulti(
    const VL6180X_t *const sensors[],
    VL6180X_Range_t results[],
    HAL_StatusTypeDef statuses[],
    size_t count
);

// Llamar desde HAL_GPIO_EXTI_Falling_Callback con el sensor cuyo GPIO1 disparo (ISR-safe)
void VL6180X_OnInterrupt(const VL6180X_t *sensor);



#endif /* INC_VL6180X_H_ */
