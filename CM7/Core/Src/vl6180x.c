#include "vl6180x.h"
#include <stddef.h>
#include "cmsis_os2.h"

// Los VL6180 usan direccion de registro de 16 bits

extern osMutexId_t i2c1MutexHandle;

static int VL6180X_IsValid(const VL6180X_t *sensor){
    return sensor != NULL && sensor->i2c != NULL &&
        sensor->address_7bit >= 0x08U &&
        sensor->address_7bit <= 0x77U;
}

static HAL_StatusTypeDef VL6180X_Read(
    const VL6180X_t *sensor,
    uint16_t reg,
    uint8_t *value
){
    if(!VL6180X_IsValid(sensor) || value == NULL) return HAL_ERROR;

    if(osMutexAcquire(i2c1MutexHandle, osWaitForever) != osOK){
        return HAL_BUSY;
    }

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        sensor->i2c, (uint16_t)(sensor->address_7bit << 1),
        reg, I2C_MEMADD_SIZE_16BIT, value, 1U, 20U
    );

    osMutexRelease(i2c1MutexHandle);
    return status;
}

static HAL_StatusTypeDef VL6180X_Write(
    const VL6180X_t *sensor,
    uint16_t reg,
    uint8_t value
){
    if(!VL6180X_IsValid(sensor)) return HAL_ERROR;

    if(osMutexAcquire(i2c1MutexHandle, osWaitForever) != osOK){
        return HAL_BUSY;
    }

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
        sensor->i2c, (uint16_t)(sensor->address_7bit << 1),
        reg, I2C_MEMADD_SIZE_16BIT, &value, 1U, 20U
    );

    osMutexRelease(i2c1MutexHandle);
    return status;
}

static HAL_StatusTypeDef VL6180X_Wait(
    const VL6180X_t *sensor, 
    uint16_t reg,
    uint8_t mask,
    uint8_t expected
){
    uint32_t start = HAL_GetTick();

    for(;;){
        uint8_t value = 0;
        HAL_StatusTypeDef status = VL6180X_Read(sensor, reg, &value);

        if(status != HAL_OK) return status;
        if((value & mask) == expected) return HAL_OK;

        if((uint32_t)(HAL_GetTick() - start) >= 100U){
            return HAL_TIMEOUT;
        }

        osDelay(1U);
    }
}

HAL_StatusTypeDef VL6180X_SetAddress(
    VL6180X_t *sensor, 
    uint8_t new_address_7bit
){
    if(new_address_7bit < 0x08U || new_address_7bit > 0x77U){
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = VL6180X_Write(sensor, 0x0212U, new_address_7bit);

    if(status == HAL_OK) sensor->address_7bit = new_address_7bit;

    return status;
}

HAL_StatusTypeDef VL6180X_Init(const VL6180X_t *sensor){
    uint8_t id = 0;
    HAL_StatusTypeDef status = VL6180X_Read(sensor, 0x0000U, &id);

    if(status != HAL_OK) return status;
    if(id != 0xB4U) return HAL_ERROR;

    static const struct {
        uint16_t reg;
        uint8_t values;
    } settings[] = {
        // Ajustes obligatorios: AN4545 
        {0x0207, 0x01}, {0x0208, 0x01},
        {0x0096, 0x00}, {0x0097, 0xFD},
        {0x00E3, 0x01}, {0x00E4, 0x03},
        {0x00E5, 0x02}, {0x00E6, 0x01},
        {0x00E7, 0x03}, {0x00F5, 0x02},
        {0x00D9, 0x05}, {0x00DB, 0xCE},
        {0x00DC, 0x03}, {0x00DD, 0xF8},
        {0x009F, 0x00}, {0x00A3, 0x3C},
        {0x00B7, 0x00}, {0x00BB, 0x3C},
        {0x00B2, 0x09}, {0x00CA, 0x09},
        {0x0198, 0x01}, {0x01B0, 0x17},
        {0x01AD, 0x00}, {0x00FF, 0x05},
        {0x0100, 0x05}, {0x0199, 0x05},
        {0x01A6, 0x1B}, {0x01AC, 0x3E},
        {0x01A7, 0x1F}, {0x0030, 0x00},

        // Configuracion de distancia. 
        {0x0011, 0x10}, {0x010A, 0x30},
        {0x0031, 0xFF}, {0x002E, 0x01},
        {0x0014, 0x04}, {0x0015, 0x07},
        {0x0016, 0x00}
    };

    for(size_t i = 0; i < sizeof(settings) / sizeof(settings[0]); ++i){
        status = VL6180X_Write(sensor, settings[i].reg, settings[i].values);
        if(status != HAL_OK) return status;
    }

    return HAL_OK;
}

HAL_StatusTypeDef VL6180X_ReadRange(
    const VL6180X_t *sensor,
    VL6180X_Range_t *result
){
    if(result == NULL) return HAL_ERROR;

    result->distance_mm = 0xFFFFU;
    result->range_status = 0xFFU;

    // esperamos que este disponible uno y luego hacemos una medicion individual
    HAL_StatusTypeDef status = VL6180X_Wait(sensor, 0x004DU, 0x01U, 0x01U);
    if(status != HAL_OK) return status;
    
    status = VL6180X_Write(sensor, 0x0015U, 0x07);
    if(status != HAL_OK) return status;

    status = VL6180X_Write(sensor, 0x0018U, 0x01U);
    if(status != HAL_OK) return status;

    status = VL6180X_Wait(sensor, 0x004FU, 0xC7U, 0x04U);
    if(status != HAL_OK) return status;

    uint8_t raw_status = 0;
    uint8_t distance = 0;

    status = VL6180X_Read(sensor, 0x004DU, &raw_status);
    if(status != HAL_OK) return status;

    status = VL6180X_Read(sensor, 0x0062U, &distance);
    if(status != HAL_OK) return status;

    result->range_status = (uint8_t)((raw_status >> 4) & 0x0FU);

    if(result->range_status == 0U){
        result->distance_mm = distance;
    }

    return HAL_OK;
}
