/*
 * i2c_bus.c
 *
 *  Created on: 26 nov 2026
 *      Author: juanb
 */

 #include "i2c_bus.h"

I2CStatus_t I2CBus_Init(I2CBus_t *bus, I2C_HandleTypeDef *hi2c, osMutexId_t mutex, osSemaphoreId_t done_sem) {
    if(bus == NULL || hi2c == NULL) return I2C_ERR_BUS;

    bus->hi2c = hi2c;
    bus->mutex = mutex;
    bus->done_sem = done_sem;
    bus->error = HAL_I2C_ERROR_NONE;

    return I2C_OK;
}


static void i2c_recover(I2CBus_t *bus) {
    if(HAL_I2C_Master_Abort_IT(bus->hi2c, 0) == HAL_OK) {
        osSemaphoreAcquire(bus->done_sem, I2C_ABORT_TIMEOUT_MS);
    }

    if(HAL_I2C_GetState(bus->hi2c) != HAL_I2C_STATE_READY) {
        HAL_I2C_DeInit(bus->hi2c);
        HAL_I2C_Init(bus->hi2c);
    }

    while(osSemaphoreAcquire(bus->done_sem, 0) == osOK) { }
}

static I2CStatus_t wait_done(I2CBus_t *bus, uint32_t timeout_ms) {
    if(osSemaphoreAcquire(bus->done_sem, timeout_ms) != osOK) {
        i2c_recover(bus);
        return I2C_ERR_TIMEOUT;
    }

    if(bus->error != HAL_I2C_ERROR_NONE) {
        return (bus->error & HAL_I2C_ERROR_AF) ? I2C_ERR_NACK : I2C_ERR_BUS;
    }

    return I2C_OK;
}

I2CStatus_t I2CBus_MemRead(I2CBus_t *bus, uint16_t addr7, uint16_t reg, uint16_t reg_size, uint8_t *buf, uint16_t len, uint32_t timeout_ms) {
    if(bus == NULL || buf == NULL || len == 0) return I2C_ERR_BUS;

    if(osMutexAcquire(bus->mutex, timeout_ms) != osOK) return I2C_ERR_BUSY;

    bus->error = HAL_I2C_ERROR_NONE;
    osSemaphoreAcquire(bus->done_sem, 0);

    I2CStatus_t status;

    if(HAL_I2C_Mem_Read_IT(bus->hi2c, addr7 << 1, reg, reg_size, buf, len) != HAL_OK) {
        status = I2C_ERR_BUSY;
    } else {
        status = wait_done(bus, timeout_ms);
    }

    osMutexRelease(bus->mutex);

    return status;
}

I2CStatus_t I2CBus_MemWrite(I2CBus_t *bus, uint16_t addr7, uint16_t reg, uint16_t reg_size, const uint8_t *buf, uint16_t len, uint32_t timeout_ms) {
    if(bus == NULL || buf == NULL || len == 0) return I2C_ERR_BUS;

    if(osMutexAcquire(bus->mutex, timeout_ms) != osOK) return I2C_ERR_BUSY;

    bus->error = HAL_I2C_ERROR_NONE;
    osSemaphoreAcquire(bus->done_sem, 0);

    I2CStatus_t status;

    if(HAL_I2C_Mem_Write_IT(bus->hi2c, addr7 << 1, reg, reg_size, (uint8_t*)buf, len) != HAL_OK) {
        status = I2C_ERR_BUSY;
    } else {
        status = wait_done(bus, timeout_ms);
    }

    osMutexRelease(bus->mutex);

    return status;
}

void I2CBus_OnRxCpltCb(I2CBus_t *bus, I2C_HandleTypeDef *h) {
    if(bus == NULL || h == NULL)return;
    if(bus->hi2c != h)return;

    osSemaphoreRelease(bus->done_sem);
    return;
}

void I2CBus_OnTxCpltCb(I2CBus_t *bus, I2C_HandleTypeDef *h) {
    if(bus == NULL || h == NULL)return;
    if(bus->hi2c != h)return;

    osSemaphoreRelease(bus->done_sem);
    return;
}

void I2CBus_OnErrorCb(I2CBus_t *bus, I2C_HandleTypeDef *h) {
    if(bus == NULL || h == NULL)return;
    if(bus->hi2c != h)return;

    bus->error = HAL_I2C_GetError(h);
    osSemaphoreRelease(bus->done_sem);
    return;
}

void I2CBus_OnAbortCpltCb(I2CBus_t *bus, I2C_HandleTypeDef *h) {
    if(bus == NULL || bus->hi2c != h) return;
    osSemaphoreRelease(bus->done_sem);
}