#include "vl6180x.h"
#include <stddef.h>

// lee un byte del registro 0x0000
HAL_StatusTypeDef VL6180X_ReadModelID(
    const VL6180X_t *sensor, uint8_t *model_id
){

    if(sensor == NULL || sensor->i2c == NULL || 
        model_id == NULL || sensor->address_7bit > 0x7FU){
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(sensor->i2c, 
        (uint16_t)(sensor->address_7bit << 1), 
        0x0000U,
        I2C_MEMADD_SIZE_16BIT,
        model_id,
        1U,
        20U
    );
}

HAL_StatusTypeDef VL6180X_SetAddress(
    VL6180X_t *sensor,
    uint8_t new_address_7bit
){
    if(sensor == NULL || sensor->i2c == NULL || sensor->address_7bit > 0x7FU || 
        new_address_7bit < 0x08U || new_address_7bit > 0x77U){
            return HAL_ERROR;
    }

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
        sensor->i2c,
        (uint16_t)(sensor->address_7bit << 1),
        0x0212U,
        I2C_MEMADD_SIZE_16BIT,
        &new_address_7bit,
        1U,
        20U
    );

    if(status == HAL_OK){
        sensor->address_7bit = new_address_7bit;
    }

    return status;
}

static HAL_StatusTypeDef VL6180X_Write8(
    const VL6180X_t *sensor, 
    uint16_t reg, 
    uint8_t value
){
    return HAL_I2C_Mem_Write(
        sensor->i2c,
        (uint16_t)(sensor->address_7bit << 1),
        reg,
        I2C_MEMADD_SIZE_16BIT,
        &value,
        1U,
        20U);
}

// habilitamos el sensor con SHDN
// antes de miedir le asignamos su direccion
HAL_StatusTypeDef VL6180X_Init(const VL6180X_t *sensor){
    uint8_t id = 0;
    HAL_StatusTypeDef status = VL6180X_ReadModelID(sensor, &id);

    if(status != HAL_OK) return status;

    if(id != 0xB4U) return HAL_ERROR;

        static const struct {
        uint16_t reg;
        uint8_t value;

    } settings[] = {
        // AN4545 Mandatory : private registers
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

        // Configuracion para medir distancia. 
        {0x0011, 0x10}, // GPIO1: evento de muestra nueva. 
        {0x010A, 0x30}, // Periodo de promediado. 
        {0x0031, 0xFF}, // Recalibracion cada 255 mediciones. 
        {0x002E, 0x01}, // Solicitar recalibracion inicial. 
        {0x0014, 0x04}, // Evento de distancia lista. 
        {0x0015, 0x07}, // Limpiar eventos pendientes. 
        {0x0016, 0x00}, // Marcar inicializacion completada. 
    };

    for(size_t i = 0; i < sizeof(settings) / sizeof(settings[i]); ++i){
        status = VL6180X_Write8(sensor, settings[i].reg, settings[i].value);

        if(status != HAL_OK) return status;
    }

    return HAL_OK;
}
