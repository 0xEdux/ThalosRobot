#ifndef VL6180X_H
#define VL6180X_H

#include "stm32h7xx_hal.h"
#define VL6180X_DEFAULT_ADDRESS 0x29

typedef struct {
    I2C_HandleTypeDef *i2c;
    uint8_t address_7bit;
} VL6180X_t;

HAL_StatusTypeDef VL6180X_ReadModelID(const VL6180X_t *sensor, uint8_t *model_id);

HAL_StatusTypeDef VL6180X_SetAddress( VL6180X_t *sensor, uint8_t new_address_7bit);

HAL_StatusTypeDef VL6180X_Init(const VL6180X_t *sensor);

#endif
