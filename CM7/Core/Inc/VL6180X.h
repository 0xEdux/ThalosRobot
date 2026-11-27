
#ifndef INC_VL6180X_H_
#define INC_VL6180X_H_

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "i2c_bus.h"
#include "main.h"

// Address I2C (7 bits)                                           */
#define VL6180X_DEFAULT_ADDR            0x29U   /* addres before  reset */
#define VL6180X_ADDR_MIN                0x08U
#define VL6180X_ADDR_MAX                0x77U

// Registers (16 bits)                                                */
#define VL6180X_REG_MODEL_ID                    0x0000U
#define VL6180X_REG_SYSTEM_MODE_GPIO1           0x0011U
#define VL6180X_REG_SYSTEM_INT_CONFIG_GPIO      0x0014U
#define VL6180X_REG_SYSTEM_INT_CLEAR            0x0015U
#define VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET   0x0016U
#define VL6180X_REG_SYSRANGE_START              0x0018U
#define VL6180X_REG_SYSRANGE_INTERMEAS_PERIOD   0x001BU
#define VL6180X_REG_SYSRANGE_VHV_RECAL          0x002EU
#define VL6180X_REG_SYSRANGE_VHV_REPEAT_RATE    0x0031U
#define VL6180X_REG_RESULT_RANGE_STATUS         0x004DU
#define VL6180X_REG_RESULT_RANGE_VAL            0x0062U
#define VL6180X_REG_READOUT_AVG_SAMPLE_PERIOD   0x010AU
#define VL6180X_REG_I2C_SLAVE_DEVICE_ADDRESS    0x0212U

// Values                                                            */
#define VL6180X_MODEL_ID_EXPECTED       0xB4U
#define VL6180X_FRESH_OUT_OF_RESET      0x01U
#define VL6180X_CLEAR_FRESH_FLAG        0x00U


#define VL6180X_GPIO1_MODE_INTERRUPT    0x10U   // GPIO1 as output low active
#define VL6180X_INT_CFG_RANGE_NEW_SAMPLE 0x04U 
#define VL6180X_INT_CLEAR_ALL           0x07U

#define VL6180X_AVG_SAMPLE_PERIOD       0x30U
#define VL6180X_VHV_REPEAT_RATE_CONT    0xFFU
#define VL6180X_VHV_RECAL_ENABLE        0x01U

#define VL6180X_START_CONTINUOUS        0x03U

#define VL6180X_ADDR_MASK               0x7FU
#define VL6180X_RANGE_ERR_SHIFT         4U
#define VL6180X_RANGE_ERR_MASK          0x0FU
#define VL6180X_RANGE_ERR_NONE          0U


// Delays
#define VL6180X_I2C_TIMEOUT_MS          20U
#define VL6180X_BOOT_DELAY_MS           5U      // after up CE befrore transmit */
#define VL6180X_PERIOD_MIN_MS           10U     // period between readings
#define VL6180X_PERIOD_MAX_MS           2550U
#define VL6180X_SAMPLE_MARGIN_MS        20U     // extra margin after EXTI

#define VL6180X_DISTANCE_INVALID        0xFFFFU

// Types
typedef struct {
    I2CBus_t *bus;
    uint8_t address;
    osSemaphoreId_t ready_sem;
    uint16_t exti_pin;
    GPIO_TypeDef *ce_port;
    uint16_t ce_pin;
    uint16_t period_ms;
    uint8_t isSensor_start;
} VL6180X_Handle_t;

typedef enum {
    VL6180X_DATA_OK,
    VL6180X_DATA_EMPTY,
} VL6180X_Data_Status_t;

typedef struct {
    uint16_t distance_mm;               // VL6180X_DISTANCE_INVALID if no reading
    VL6180X_Data_Status_t range_status; // OK / EMPTY
    uint8_t error_code;                 // raw error (0 = sin error)
} VL6180X_Range_t;

// API
HAL_StatusTypeDef VL6180X_Init(VL6180X_Handle_t *sensor, I2CBus_t *bus, osSemaphoreId_t ready_sem, uint16_t exti_pin, GPIO_TypeDef *ce_port, uint16_t ce_pin);

HAL_StatusTypeDef VL6180X_SetAddress(VL6180X_Handle_t *sensor, uint8_t new_address);


HAL_StatusTypeDef VL6180X_Start(VL6180X_Handle_t *sensor, uint8_t address, uint16_t period_ms);

HAL_StatusTypeDef VL6180X_ReadDistance(VL6180X_Handle_t *sensor, VL6180X_Range_t *result);

void VL6180X_OnExtiCb(VL6180X_Handle_t *sensor, uint16_t GPIO_PIN);

#endif /* INC_VL6180X_H_ */