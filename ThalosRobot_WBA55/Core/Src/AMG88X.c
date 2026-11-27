/*
 * AMG88X.c
 *
 *  Created on: 26 nov 2026
 *      Author: juanb
 */

#include "AMG88X.h"


typedef struct {
    uint8_t  reg;
    uint8_t  value;
    uint16_t delay_ms;
} AMG88XX_RegValue_t;

// Secuencia de arranque: reset inicial, INT apagado, frame rate se escribe aparte
static const AMG88XX_RegValue_t init_config[] = {
    { AMG88XX_REG_PCTL, AMG88XX_PCTL_NORMAL,    AMG88XX_BOOT_DELAY_MS },
    { AMG88XX_REG_RST,  AMG88XX_RST_INITIAL,    AMG88XX_RESET_DELAY_MS },
    { AMG88XX_REG_INTC, AMG88XX_INTC_DISABLED,  0},
};

// Helpers
static HAL_StatusTypeDef i2c_to_hal(I2CStatus_t status) {
    switch(status) {
        case I2C_OK:          return HAL_OK;
        case I2C_ERR_TIMEOUT: return HAL_TIMEOUT;
        case I2C_ERR_BUSY:    return HAL_BUSY;
        default:              return HAL_ERROR;
    }
}

static HAL_StatusTypeDef AMG88XX_WriteReg(AMG88XX_Handle_t *sensor, uint8_t reg, uint8_t value) {
    return i2c_to_hal(I2CBus_MemWrite(sensor->bus, sensor->address, reg, I2C_MEMADD_SIZE_8BIT, &value, 1,AMG88XX_I2C_TIMEOUT_MS));
}

static HAL_StatusTypeDef AMG88XX_ReadRegs(AMG88XX_Handle_t *sensor, uint8_t reg, uint8_t *data, uint16_t size) {
    return i2c_to_hal(I2CBus_MemRead(sensor->bus, sensor->address, reg, I2C_MEMADD_SIZE_8BIT, data, size, AMG88XX_I2C_TIMEOUT_MS));
}


static HAL_StatusTypeDef AMG88XX_WriteTable(AMG88XX_Handle_t *sensor, const AMG88XX_RegValue_t *table, uint8_t count) {
    for(uint8_t i = 0; i < count; ++i) {
        HAL_StatusTypeDef status = AMG88XX_WriteReg(sensor, table[i].reg, table[i].value);
        if(status != HAL_OK) return status;
        if(table[i].delay_ms != 0U) osDelay(table[i].delay_ms);
    }
    return HAL_OK;
}

static uint16_t cc_to_raw12(int32_t cc) {
    int32_t lsb = cc / AMG88XX_CENTI_PER_LSB;
    if(lsb >  2047) lsb =  2047;
    if(lsb < -2048) lsb = -2048;
    return (uint16_t)(lsb & 0x0FFF);
}

static HAL_StatusTypeDef AMG88XX_WriteLevel12(AMG88XX_Handle_t *sensor, uint8_t reg_low, uint8_t reg_high, int32_t cc) {
    uint16_t raw = cc_to_raw12(cc);
    HAL_StatusTypeDef status = AMG88XX_WriteReg(sensor, reg_low, (uint8_t)(raw & 0xFFU));
    if(status != HAL_OK) return status;
    return AMG88XX_WriteReg(sensor, reg_high, (uint8_t)((raw >> 8) & 0x0FU));
}

// API

HAL_StatusTypeDef AMG88XX_Init(AMG88XX_Handle_t *sensor, I2CBus_t *bus, uint8_t address, osSemaphoreId_t int_sem, uint16_t exti_pin) {
    if(sensor == NULL || bus == NULL) return HAL_ERROR;
    if(address != AMG88XX_ADDR_LOW && address != AMG88XX_ADDR_HIGH) return HAL_ERROR;

    sensor->bus = bus;
    sensor->address = address;
    sensor->int_sem = int_sem;
    sensor->exti_pin = exti_pin;
    sensor->period_ms = 0;
    sensor->next_tick = 0;
    sensor->isSensor_start = 0;

    return HAL_OK;
}

HAL_StatusTypeDef AMG88XX_Start(AMG88XX_Handle_t *sensor, uint8_t fps_reg) {
    if(sensor == NULL) return HAL_ERROR;
    if(fps_reg != AMG88XX_FPSC_10FPS && fps_reg != AMG88XX_FPSC_1FPS) return HAL_ERROR;

    HAL_StatusTypeDef status = AMG88XX_WriteTable(sensor, init_config, sizeof(init_config) / sizeof(init_config[0]));
    if(status != HAL_OK) return status;

    status = AMG88XX_WriteReg(sensor, AMG88XX_REG_FPSC, fps_reg);
    if(status != HAL_OK) return status;

    status = AMG88XX_WriteReg(sensor, AMG88XX_REG_RST, AMG88XX_RST_FLAG);
    if(status != HAL_OK) return status;

    if(sensor->int_sem != NULL) osSemaphoreAcquire(sensor->int_sem, 0);

    sensor->period_ms = (fps_reg == AMG88XX_FPSC_10FPS) ? AMG88XX_PERIOD_10FPS_MS : AMG88XX_PERIOD_1FPS_MS;

    // First frame after warm-up + timeout
    sensor->next_tick = osKernelGetTickCount() +
        (((AMG88XX_WARMUP_MS + sensor->period_ms) * osKernelGetTickFreq()) / 1000U);
    sensor->isSensor_start = 1;

    return HAL_OK;
}

HAL_StatusTypeDef AMG88XX_Stop(AMG88XX_Handle_t *sensor) {
    if(sensor == NULL) return HAL_ERROR;
    sensor->isSensor_start = 0;
    return AMG88XX_WriteReg(sensor, AMG88XX_REG_PCTL, AMG88XX_PCTL_SLEEP);
}

HAL_StatusTypeDef AMG88XX_ReadPixels(AMG88XX_Handle_t *sensor,
                                     int32_t temperatures[AMG88XX_PIXEL_COUNT]) {
    if(sensor == NULL || temperatures == NULL) return HAL_ERROR;
    if(!sensor->isSensor_start) return HAL_ERROR;

    // Sleep until next frame
    uint32_t now = osKernelGetTickCount();
    uint32_t period_ticks = (sensor->period_ms * osKernelGetTickFreq()) / 1000U;

    if((int32_t)(sensor->next_tick - now) > 0) {
        osDelay(sensor->next_tick - now);
        sensor->next_tick += period_ticks;
    } else {
        // Task late, resync withou delay
        sensor->next_tick = now + period_ticks;
    }

    uint8_t data[AMG88XX_PIXEL_BYTES];
    HAL_StatusTypeDef status = AMG88XX_ReadRegs(sensor, AMG88XX_REG_PIXEL_BASE,
                                                data, sizeof(data));
    if(status != HAL_OK) return status;

    for(uint32_t i = 0; i < AMG88XX_PIXEL_COUNT; ++i) {
        uint16_t raw = (uint16_t)(data[2U * i] | ((uint16_t)data[2U * i + 1U] << 8));
        raw &= 0x0FFFU;

        int32_t value = (int32_t)raw;
        if((raw & 0x0800U) != 0U) value -= 4096;

        temperatures[i] = value * AMG88XX_CENTI_PER_LSB;
    }

    return HAL_OK;
}

HAL_StatusTypeDef AMG88XX_ReadThermistor(AMG88XX_Handle_t *sensor, int32_t *centi_c) {
    if(sensor == NULL || centi_c == NULL) return HAL_ERROR;

    uint8_t data[2];
    HAL_StatusTypeDef status = AMG88XX_ReadRegs(sensor, AMG88XX_REG_TTHL, data, sizeof(data));
    if(status != HAL_OK) return status;

    // 12 bits sign + magnitud: 11 bit = sign, 0.0625 °C per LSB
    uint16_t raw = (uint16_t)(data[0] | ((uint16_t)data[1] << 8));
    int32_t mag = (int32_t)(raw & 0x07FFU);
    int32_t value = (mag * AMG88XX_THERM_CENTI_X100) / 100;

    *centi_c = ((raw & 0x0800U) != 0U) ? -value : value;
    return HAL_OK;
}

HAL_StatusTypeDef AMG88XX_EnableInterrupt(AMG88XX_Handle_t *sensor, AMG88XX_IntMode_t mode,
                                          int32_t upper_cc, int32_t lower_cc, int32_t hyst_cc) {
    if(sensor == NULL || sensor->int_sem == NULL) return HAL_ERROR;

    HAL_StatusTypeDef status;

    status = AMG88XX_WriteLevel12(sensor, AMG88XX_REG_INT_LVL_H_L, AMG88XX_REG_INT_LVL_H_H, upper_cc);
    if(status != HAL_OK) return status;
    status = AMG88XX_WriteLevel12(sensor, AMG88XX_REG_INT_LVL_L_L, AMG88XX_REG_INT_LVL_L_H, lower_cc);
    if(status != HAL_OK) return status;
    status = AMG88XX_WriteLevel12(sensor, AMG88XX_REG_INT_LVL_HYS_L, AMG88XX_REG_INT_LVL_HYS_H, hyst_cc);
    if(status != HAL_OK) return status;

    uint8_t intc = AMG88XX_INTC_EN_BIT;
    if(mode == AMG88XX_INT_MODE_ABSOLUTE) intc |= AMG88XX_INTC_ABS_BIT;

    status = AMG88XX_WriteReg(sensor, AMG88XX_REG_SCLR, AMG88XX_SCLR_ALL);
    if(status != HAL_OK) return status;

    osSemaphoreAcquire(sensor->int_sem, 0);
    return AMG88XX_WriteReg(sensor, AMG88XX_REG_INTC, intc);
}

HAL_StatusTypeDef AMG88XX_DisableInterrupt(AMG88XX_Handle_t *sensor) {
    if(sensor == NULL) return HAL_ERROR;
    return AMG88XX_WriteReg(sensor, AMG88XX_REG_INTC, AMG88XX_INTC_DISABLED);
}

HAL_StatusTypeDef AMG88XX_WaitInterrupt(AMG88XX_Handle_t *sensor, uint32_t timeout_ms) {
    if(sensor == NULL || sensor->int_sem == NULL) return HAL_ERROR;
    if(!sensor->isSensor_start) return HAL_ERROR;

    // Block task wait for interrupt
    uint32_t ticks = (timeout_ms == osWaitForever)
                   ? osWaitForever
                   : (timeout_ms * osKernelGetTickFreq()) / 1000U;

    if(osSemaphoreAcquire(sensor->int_sem, ticks) != osOK) return HAL_TIMEOUT;

    // Clear flag for sensor to free INT
    uint8_t stat = 0;
    HAL_StatusTypeDef status = AMG88XX_ReadRegs(sensor, AMG88XX_REG_STAT, &stat, 1);
    HAL_StatusTypeDef clear_status = AMG88XX_WriteReg(sensor, AMG88XX_REG_SCLR, AMG88XX_SCLR_ALL);

    if(status != HAL_OK) return status;
    if(clear_status != HAL_OK) return clear_status;

    return ((stat & AMG88XX_STAT_INTF) != 0U) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef AMG88XX_ReadInterruptTable(AMG88XX_Handle_t *sensor,
                                             uint8_t table[AMG88XX_INT_TABLE_BYTES]) {
    if(sensor == NULL || table == NULL) return HAL_ERROR;
    return AMG88XX_ReadRegs(sensor, AMG88XX_REG_INT_TABLE, table, AMG88XX_INT_TABLE_BYTES);
}

void AMG88XX_OnExtiCb(AMG88XX_Handle_t *sensor, uint16_t GPIO_PIN) {
    if(sensor == NULL || sensor->int_sem == NULL) return;
    if(GPIO_PIN != sensor->exti_pin) return;

    osSemaphoreRelease(sensor->int_sem);
}