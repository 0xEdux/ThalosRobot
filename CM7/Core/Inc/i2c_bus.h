/*
 * i2c_bus.h
 *
 *  Created on: 26 nov 2026
 *      Author: juanb
 */

#ifndef INC_I2C_BUS_H_
#define INC_I2C_BUS_H_

#include "main.h"
#include "cmsis_os2.h"

#define I2C_ABORT_TIMEOUT_MS  5U

typedef struct {
	I2C_HandleTypeDef *hi2c;
	osMutexId_t mutex;
	osSemaphoreId_t done_sem;
	volatile uint32_t error;
} I2CBus_t;

typedef enum {
	I2C_OK,
	I2C_ERR_TIMEOUT,
	I2C_ERR_NACK,
	I2C_ERR_BUSY,
	I2C_ERR_BUS
} I2CStatus_t;

I2CStatus_t I2CBus_Init(I2CBus_t *bus, I2C_HandleTypeDef *hi2c, osMutexId_t mutex, osSemaphoreId_t done_sem);
I2CStatus_t I2CBus_MemRead(I2CBus_t *bus, uint16_t addr7, uint16_t reg, uint16_t reg_size, uint8_t *buf, uint16_t len, uint32_t timeout_ms);
I2CStatus_t I2CBus_MemWrite(I2CBus_t *bus, uint16_t addr7, uint16_t reg, uint16_t reg_size, const uint8_t *buf, uint16_t len, uint32_t timeout_ms);

void I2CBus_OnRxCpltCb(I2CBus_t *bus, I2C_HandleTypeDef *h);
void I2CBus_OnTxCpltCb(I2CBus_t *bus, I2C_HandleTypeDef *h);
void I2CBus_OnErrorCb(I2CBus_t *bus, I2C_HandleTypeDef *h);
void I2CBus_OnAbortCpltCb(I2CBus_t *bus, I2C_HandleTypeDef *h);

#endif /* INC_I2C_BUS_H_ */
