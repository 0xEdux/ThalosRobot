/*
 * AMG88X.h
 *
 *  Created on: 26 nov 2026
 *      Author: juanb
 */

#ifndef INC_AMG88X_H_
#define INC_AMG88X_H_


#include "main.h"
#include "cmsis_os2.h"
#include "i2c_bus.h"
#include <stdint.h>
#include "FreeRTOS.h"

// Address
#define AMG88XX_ADDR_LOW            0x68U   /* AD_SELECT GND */
#define AMG88XX_ADDR_HIGH           0x69U   /* AD_SELECT VDD */

// Registers
#define AMG88XX_REG_PCTL            0x00U   // power control
#define AMG88XX_REG_RST             0x01U   // reset
#define AMG88XX_REG_FPSC            0x02U   // frame rate
#define AMG88XX_REG_INTC            0x03U   // interupt control
#define AMG88XX_REG_STAT            0x04U   // status only-read
#define AMG88XX_REG_SCLR            0x05U   // status clear
#define AMG88XX_REG_INT_LVL_H_L     0x08U   // upper threshold, low byte
#define AMG88XX_REG_INT_LVL_H_H     0x09U   // upper threshold, high nibble
#define AMG88XX_REG_INT_LVL_L_L     0x0AU   // low threshold, low byte
#define AMG88XX_REG_INT_LVL_L_H     0x0BU   // low threshold, high nibble
#define AMG88XX_REG_INT_LVL_HYS_L   0x0CU   // histéresis, low byte
#define AMG88XX_REG_INT_LVL_HYS_H   0x0DU   // histéresis, high nibble
#define AMG88XX_REG_TTHL            0x0EU   // termistor, low byte
#define AMG88XX_REG_TTHH            0x0FU   // termistor, high byte
#define AMG88XX_REG_INT_TABLE       0x10U   // 8 bytes: pexels that execute INT
#define AMG88XX_REG_PIXEL_BASE      0x80U   // temperature 128 bytes

// Registers values
#define AMG88XX_PCTL_NORMAL         0x00U
#define AMG88XX_PCTL_SLEEP          0x10U
#define AMG88XX_PCTL_STANDBY_60S    0x20U
#define AMG88XX_PCTL_STANDBY_10S    0x21U

#define AMG88XX_RST_FLAG            0x30U   // clear flags
#define AMG88XX_RST_INITIAL         0x3FU   // initial reset

#define AMG88XX_FPSC_10FPS          0x00U
#define AMG88XX_FPSC_1FPS           0x01U

#define AMG88XX_INTC_DISABLED       0x00U
#define AMG88XX_INTC_EN_BIT         0x01U   // bit 0: enable INT pin
#define AMG88XX_INTC_ABS_BIT        0x02U   // bit 1: 0 difference, 1 absolute

#define AMG88XX_STAT_OVF_IRS        0x02U   // overflow in pexel
#define AMG88XX_STAT_INTF           0x04U   // interrupt launch
#define AMG88XX_STAT_OVF_THS        0x08U   // termistor overflow
#define AMG88XX_SCLR_ALL            0x0EU   // clear 3 flags

// Matrix
#define AMG88XX_PIXEL_COUNT         64U
#define AMG88XX_PIXEL_BYTES         (AMG88XX_PIXEL_COUNT * 2U)
#define AMG88XX_INT_TABLE_BYTES     8U
#define AMG88XX_CENTI_PER_LSB       25      // 0.25 °C = 25 hundredths
#define AMG88XX_THERM_CENTI_X100    625     // 0.0625 °C = 6.25 hundredths

// Delays
#define AMG88XX_I2C_TIMEOUT_MS      50U
#define AMG88XX_BOOT_DELAY_MS       60U     /* power-on / sleep ouutput: >= 50 ms */
#define AMG88XX_RESET_DELAY_MS      5U      /* after reset: >= 2 ms */
#define AMG88XX_WARMUP_MS           100U    /* first square after reset */
#define AMG88XX_PERIOD_10FPS_MS     100U
#define AMG88XX_PERIOD_1FPS_MS      1000U


typedef enum {
    AMG88XX_INT_MODE_DIFFERENCE = 0,
    AMG88XX_INT_MODE_ABSOLUTE   = 1
} AMG88XX_IntMode_t;

typedef struct {
    I2CBus_t       *bus;
    uint8_t         address;        /* 7 bits */
    osSemaphoreId_t int_sem;        /* NULL if not using sem */
    uint16_t        exti_pin;
    uint32_t        period_ms;
    uint32_t        next_tick;
    uint8_t         isSensor_start;
} AMG88XX_Handle_t;

HAL_StatusTypeDef AMG88XX_Init(AMG88XX_Handle_t *sensor, I2CBus_t *bus, uint8_t address, osSemaphoreId_t int_sem, uint16_t exti_pin);

HAL_StatusTypeDef AMG88XX_Start(AMG88XX_Handle_t *sensor, uint8_t fps_reg);
HAL_StatusTypeDef AMG88XX_Stop(AMG88XX_Handle_t *sensor);

HAL_StatusTypeDef AMG88XX_ReadPixels(AMG88XX_Handle_t *sensor, int32_t temperatures[AMG88XX_PIXEL_COUNT]);
HAL_StatusTypeDef AMG88XX_ReadThermistor(AMG88XX_Handle_t *sensor, int32_t *centi_c);

HAL_StatusTypeDef AMG88XX_EnableInterrupt(AMG88XX_Handle_t *sensor, AMG88XX_IntMode_t mode, int32_t upper_cc, int32_t lower_cc, int32_t hyst_cc);
HAL_StatusTypeDef AMG88XX_DisableInterrupt(AMG88XX_Handle_t *sensor);
HAL_StatusTypeDef AMG88XX_WaitInterrupt(AMG88XX_Handle_t *sensor, uint32_t timeout_ms);
HAL_StatusTypeDef AMG88XX_ReadInterruptTable(AMG88XX_Handle_t *sensor, uint8_t table[AMG88XX_INT_TABLE_BYTES]);

/* Llamar desde HAL_GPIO_EXTI_Falling_Callback */
void AMG88XX_OnExtiCb(AMG88XX_Handle_t *sensor, uint16_t GPIO_PIN);

#endif /* INC_AMG88X_H_ */
