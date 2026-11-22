#include "amg88XX.h"
#include <stddef.h>

// la matriz termia usa registros de 8 bits
// leemos sus 128 bytes de temperatura en una sola transf.

static int AMG88XX_IsValid(const AMG88XX_t *sensor){
    return sensor != NULL && sensor->i2c != NULL &&
        sensor->bus_mutex != NULL && 
        (sensor->address_7bit == 0x68U ||
        sensor->address_7bit == 0x69U);
}

static void AMG88XX_DelayMs(uint32_t ms){
    uint32_t ticks = (ms * osKernelGetTickFreq() + 999U) / 1000U;
    osDelay(ticks + 1U);
}

static HAL_StatusTypeDef AMG88XX_Write(
    const AMG88XX_t *sensor, 
    uint8_t reg, uint8_t value
){
    if(!AMG88XX_IsValid(sensor)) return HAL_ERROR;

    if(osMutexAcquire(sensor->bus_mutex, osWaitForever) != osOK){
        return HAL_BUSY;
    }

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
        sensor->i2c,
        (uint16_t)(sensor->address_7bit << 1),
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1U,
        50U
    );

    osMutexRelease(sensor->bus_mutex);
    return status;
}

static HAL_StatusTypeDef AMG88XX_Read(
    const AMG88XX_t *sensor,
    uint8_t reg,
    uint8_t *data,
    uint16_t size
){
    if(!AMG88XX_IsValid(sensor) || data == NULL || size == 0U){
        return HAL_ERROR;
    }

    if(osMutexAcquire(sensor->bus_mutex, osWaitForever) != osOK){
        return HAL_BUSY;
    }

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        sensor->i2c,
        (uint16_t)(sensor->address_7bit << 1),
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data, size, 50U
    );

    osMutexRelease(sensor->bus_mutex);
    return status;
}

HAL_StatusTypeDef AMG88XX_Init(const AMG88XX_t *sensor){
    if(!AMG88XX_IsValid(sensor)) return HAL_ERROR;

    AMG88XX_DelayMs(60U);

    static const struct {
        uint8_t reg;
        uint8_t value;
        uint16_t delay_ms;
    } settings[] = {
        {0x00, 0x00, 60}, // modo normal
        {0x00, 0x10, 10}, // modo sleep
        {0x00, 0x00, 60}, // salir de sleep a normal
        {0x01, 0x3F,  5}, // recargar ajustes tras despertar
        {0x01, 0x30,  5}, // reiniciar banderas
        {0x02, 0x00,  5}, // Sensor a 10 cuadros por segundo
        {0x03, 0x00, 250} // INT desactivado; esperar 2 cuadros
    };

    for(size_t i = 0; i < sizeof(settings) / sizeof(settings[0]); ++i){
        HAL_StatusTypeDef status = AMG88XX_Write(sensor, settings[i].reg, settings[i].value);

        if(status != HAL_OK) return status;

        AMG88XX_DelayMs(settings[i].delay_ms);
    }

    return HAL_OK;
}

HAL_StatusTypeDef AMG88XX_ReadPixels(
    const AMG88XX_t *sensor,
    int32_t temperatures[AMG88XX_PIXEL_COUNT]
){
    if(temperatures == NULL) return HAL_ERROR;

    uint8_t data[AMG88XX_PIXEL_COUNT * 2U];

    HAL_StatusTypeDef status = AMG88XX_Read(sensor, 0x80U, data, sizeof(data));

    if(status != HAL_OK)return status;

    for(uint32_t i = 0; i < AMG88XX_PIXEL_COUNT; ++i){
        uint16_t raw = (uint16_t)(data[2U * i] | ((uint16_t)data[2U * i + 1U] << 8));

        raw &= 0x0FFFU;

        // convertimos los 12 bits con signo
        int32_t value = (int32_t)raw;
        if((raw & 0x0800U) != 0U) value -= 4096;

        // cada unidad = 0.25 C
        temperatures[i] = value * 25;
    }

    return HAL_OK;
}
