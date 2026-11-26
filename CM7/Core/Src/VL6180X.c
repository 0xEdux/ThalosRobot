/*
 * vl6180x.c
 *
 * Driver del VL6180X en modo continuo, sobre i2c_bus (HAL + CMSIS-RTOS2).
 */

#include "VL6180X.h"

typedef struct {
    uint16_t reg;
    uint8_t value;
} VL6180X_RegValue_t;

/* Ajustes obligatorios: AN4545 (registros privados, sin nombre publico) */
static const VL6180X_RegValue_t an4545_mandatory[] = {
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
};

/* Configuracion de distancia (registros publicos) */
static const VL6180X_RegValue_t range_config[] = {
    {VL6180X_REG_SYSTEM_MODE_GPIO1,         VL6180X_GPIO1_MODE_INTERRUPT},
    {VL6180X_REG_READOUT_AVG_SAMPLE_PERIOD, VL6180X_AVG_SAMPLE_PERIOD},
    {VL6180X_REG_SYSRANGE_VHV_REPEAT_RATE,  VL6180X_VHV_REPEAT_RATE_CONT},
    {VL6180X_REG_SYSRANGE_VHV_RECAL,        VL6180X_VHV_RECAL_ENABLE},
    {VL6180X_REG_SYSTEM_INT_CONFIG_GPIO,    VL6180X_INT_CFG_RANGE_NEW_SAMPLE},
};

/* ------------------------------------------------------------------ */
/* Helpers                                                            */
/* ------------------------------------------------------------------ */

static HAL_StatusTypeDef i2c_to_hal(I2CStatus_t status) {
    switch(status) {
        case I2C_OK:          return HAL_OK;
        case I2C_ERR_TIMEOUT: return HAL_TIMEOUT;
        case I2C_ERR_BUSY:    return HAL_BUSY;
        default:              return HAL_ERROR;
    }
}

static HAL_StatusTypeDef VL6180X_Write(VL6180X_Handle_t *sensor, uint16_t reg, uint8_t value) {
    return i2c_to_hal(I2CBus_MemWrite(sensor->bus, sensor->address, reg,
                                      I2C_MEMADD_SIZE_16BIT, &value, 1, VL6180X_I2C_TIMEOUT_MS));
}

static HAL_StatusTypeDef VL6180X_Read(VL6180X_Handle_t *sensor, uint16_t reg, uint8_t *value) {
    return i2c_to_hal(I2CBus_MemRead(sensor->bus, sensor->address, reg,
                                     I2C_MEMADD_SIZE_16BIT, value, 1, VL6180X_I2C_TIMEOUT_MS));
}

static HAL_StatusTypeDef VL6180X_WriteTable(VL6180X_Handle_t *sensor, const VL6180X_RegValue_t *table, size_t count) {
    for(size_t i = 0; i < count; ++i) {
        HAL_StatusTypeDef status = VL6180X_Write(sensor, table[i].reg, table[i].value);
        if(status != HAL_OK) return status;
    }
    return HAL_OK;
}

/* ------------------------------------------------------------------ */
/* API                                                                */
/* ------------------------------------------------------------------ */

HAL_StatusTypeDef VL6180X_Init(VL6180X_Handle_t *sensor, I2CBus_t *bus, osSemaphoreId_t ready_sem,
                               uint16_t exti_pin, GPIO_TypeDef *ce_port, uint16_t ce_pin) {
    if(sensor == NULL || bus == NULL || ready_sem == NULL || ce_port == NULL) return HAL_ERROR;

    sensor->bus = bus;
    sensor->address = VL6180X_DEFAULT_ADDR;
    sensor->ready_sem = ready_sem;
    sensor->exti_pin = exti_pin;
    sensor->ce_port = ce_port;
    sensor->ce_pin = ce_pin;
    sensor->period_ms = 0;
    sensor->isSensor_start = 0;

    /* Sensor apagado: asi, despues de Init de todos, solo se despierta uno a la vez en Start */
    HAL_GPIO_WritePin(sensor->ce_port, sensor->ce_pin, GPIO_PIN_RESET);

    return HAL_OK;
}

HAL_StatusTypeDef VL6180X_SetAddress(VL6180X_Handle_t *sensor, uint8_t new_address) {
    if(sensor == NULL) return HAL_ERROR;
    if(new_address < VL6180X_ADDR_MIN || new_address > VL6180X_ADDR_MAX) return HAL_ERROR;

    /* Despierta solo este sensor; el resto sigue con CE en bajo desde Init */
    HAL_GPIO_WritePin(sensor->ce_port, sensor->ce_pin, GPIO_PIN_SET);
    osDelay(VL6180X_BOOT_DELAY_MS);

    /* Tras el reset responde en la direccion por defecto */
    sensor->address = VL6180X_DEFAULT_ADDR;

    if(new_address != VL6180X_DEFAULT_ADDR) {
        HAL_StatusTypeDef status = VL6180X_Write(sensor, VL6180X_REG_I2C_SLAVE_DEVICE_ADDRESS,
                                                 new_address & VL6180X_ADDR_MASK);
        if(status != HAL_OK) return status;
    }

    /* Desde aqui el sensor responde en la nueva direccion. CE se queda en alto */
    sensor->address = new_address;

    return HAL_OK;
}

HAL_StatusTypeDef VL6180X_Start(VL6180X_Handle_t *sensor, uint8_t address, uint16_t period_ms) {
    if(sensor == NULL) return HAL_ERROR;
    if(period_ms < VL6180X_PERIOD_MIN_MS || period_ms > VL6180X_PERIOD_MAX_MS) return HAL_ERROR;

    HAL_StatusTypeDef status = VL6180X_SetAddress(sensor, address);
    if(status != HAL_OK) return status;

    /* Verifica que hay un VL6180X en la nueva direccion */
    uint8_t id = 0;
    status = VL6180X_Read(sensor, VL6180X_REG_MODEL_ID, &id);
    if(status != HAL_OK) return status;
    if(id != VL6180X_MODEL_ID_EXPECTED) return HAL_ERROR;

    /* AN4545: los ajustes obligatorios solo se aplican si el sensor viene de reset */
    uint8_t fresh = 0;
    status = VL6180X_Read(sensor, VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET, &fresh);
    if(status != HAL_OK) return status;

    if(fresh == VL6180X_FRESH_OUT_OF_RESET) {
        status = VL6180X_WriteTable(sensor, an4545_mandatory,
                                    sizeof(an4545_mandatory) / sizeof(an4545_mandatory[0]));
        if(status != HAL_OK) return status;
    }

    status = VL6180X_WriteTable(sensor, range_config, sizeof(range_config) / sizeof(range_config[0]));
    if(status != HAL_OK) return status;

    /* Periodo entre mediciones: (valor + 1) * 10 ms */
    status = VL6180X_Write(sensor, VL6180X_REG_SYSRANGE_INTERMEAS_PERIOD,
                           (uint8_t)((period_ms / 10U) - 1U));
    if(status != HAL_OK) return status;

    status = VL6180X_Write(sensor, VL6180X_REG_SYSTEM_INT_CLEAR, VL6180X_INT_CLEAR_ALL);
    if(status != HAL_OK) return status;

    status = VL6180X_Write(sensor, VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET, VL6180X_CLEAR_FRESH_FLAG);
    if(status != HAL_OK) return status;

    /* Descarta cualquier aviso viejo antes de arrancar */
    while(osSemaphoreAcquire(sensor->ready_sem, 0) == osOK) { }

    status = VL6180X_Write(sensor, VL6180X_REG_SYSRANGE_START, VL6180X_START_CONTINUOUS);
    if(status != HAL_OK) return status;

    sensor->period_ms = period_ms;
    sensor->isSensor_start = 1;

    return HAL_OK;
}

HAL_StatusTypeDef VL6180X_ReadDistance(VL6180X_Handle_t *sensor, VL6180X_Range_t *result) {
    if(sensor == NULL || result == NULL) return HAL_ERROR;
    if(!sensor->isSensor_start) return HAL_ERROR;

    result->distance_mm = VL6180X_DISTANCE_INVALID;
    result->range_status = VL6180X_DATA_EMPTY;
    result->error_code = 0;

    /* Bloquea solo esta tarea hasta el EXTI de GPIO1 (muestra nueva) */
    if(osSemaphoreAcquire(sensor->ready_sem, sensor->period_ms + VL6180X_SAMPLE_MARGIN_MS) != osOK) {
        return HAL_TIMEOUT;
    }

    uint8_t raw_status = 0;
    uint8_t distance = 0;

    HAL_StatusTypeDef status = VL6180X_Read(sensor, VL6180X_REG_RESULT_RANGE_STATUS, &raw_status);
    if(status == HAL_OK) {
        status = VL6180X_Read(sensor, VL6180X_REG_RESULT_RANGE_VAL, &distance);
    }

    /* Limpiar siempre: si no, GPIO1 se queda asertado y no vuelve a haber flanco */
    HAL_StatusTypeDef clear_status = VL6180X_Write(sensor, VL6180X_REG_SYSTEM_INT_CLEAR, VL6180X_INT_CLEAR_ALL);

    if(status != HAL_OK) return status;
    if(clear_status != HAL_OK) return clear_status;

    result->error_code = (uint8_t)((raw_status >> VL6180X_RANGE_ERR_SHIFT) & VL6180X_RANGE_ERR_MASK);

    if(result->error_code == VL6180X_RANGE_ERR_NONE) {
        result->distance_mm = distance;
        result->range_status = VL6180X_DATA_OK;
    }

    return HAL_OK;
}

void VL6180X_OnExtiCb(VL6180X_Handle_t *sensor, uint16_t GPIO_PIN) {
    if(sensor == NULL || sensor->ready_sem == NULL) return;
    if(GPIO_PIN != sensor->exti_pin) return;

    osSemaphoreRelease(sensor->ready_sem);
}