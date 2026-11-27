#ifndef AMG88XX_H
#define AMG88XX_H

#include "stm32wbaxx_hal.h"
#include "cmsis_os2.h"

#define AMG88XX_DEFAULT_ADDRESS 0x68U
#define AMG88XX_PIXEL_COUNT 64U

typedef struct{
    I2C_HandleTypeDef *i2c;
    osMutexId_t bus_mutex;
    uint8_t address_7bit;
} AMG88XX_t;

// las dos funciones se llaman desde una tarea RTOS
HAL_StatusTypeDef AMG88XX_Init(const AMG88XX_t *sensor);

// temperatura en centesimas
// 2500 = 25.00 C
HAL_StatusTypeDef AMG88XX_ReadPixels(
    const AMG88XX_t *sensor,
    int32_t temperatures[AMG88XX_PIXEL_COUNT]
);

#endif