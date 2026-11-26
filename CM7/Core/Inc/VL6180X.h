/*
 * VL6180X.h
 *
 * Driver del VL6180X sobre i2c_bus (HAL + CMSIS-RTOS2).
 * - Todo por interrupcion: I2C por IT y lectura lista por EXTI (GPIO1), sin polling.
 * - Modo continuo: el sensor mide solo y avisa con GPIO1 cuando hay muestra nueva.
 * - Pensado para varios sensores compartiendo el mismo bus I2C.
 */

#ifndef INC_VL6180X_H_
#define INC_VL6180X_H_

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "i2c_bus.h"
#include "main.h"

/* ------------------------------------------------------------------ */
/* Direcciones I2C (7 bits)                                           */
/* ------------------------------------------------------------------ */
#define VL6180X_DEFAULT_ADDR            0x29U   /* direccion tras cada reset */
#define VL6180X_ADDR_MIN                0x08U
#define VL6180X_ADDR_MAX                0x77U

/* Direcciones que se asignan a cada sensor del sistema */
#define VL6180X_ADDR_SENSOR_0           0x30U
#define VL6180X_ADDR_SENSOR_1           0x31U
#define VL6180X_ADDR_SENSOR_2           0x32U

/* ------------------------------------------------------------------ */
/* Registros (16 bits)                                                */
/* ------------------------------------------------------------------ */
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

/* ------------------------------------------------------------------ */
/* Valores                                                            */
/* ------------------------------------------------------------------ */
#define VL6180X_MODEL_ID_EXPECTED       0xB4U
#define VL6180X_FRESH_OUT_OF_RESET      0x01U
#define VL6180X_CLEAR_FRESH_FLAG        0x00U

#define VL6180X_GPIO1_MODE_INTERRUPT    0x10U   /* GPIO1 como salida de interrupcion, activo en bajo */
#define VL6180X_INT_CFG_RANGE_NEW_SAMPLE 0x04U  /* interrupcion de rango: muestra nueva lista */
#define VL6180X_INT_CLEAR_ALL           0x07U

#define VL6180X_AVG_SAMPLE_PERIOD       0x30U
#define VL6180X_VHV_REPEAT_RATE_CONT    0xFFU
#define VL6180X_VHV_RECAL_ENABLE        0x01U

#define VL6180X_START_CONTINUOUS        0x03U

#define VL6180X_ADDR_MASK               0x7FU
#define VL6180X_RANGE_ERR_SHIFT         4U
#define VL6180X_RANGE_ERR_MASK          0x0FU
#define VL6180X_RANGE_ERR_NONE          0U

/* ------------------------------------------------------------------ */
/* Tiempos                                                            */
/* ------------------------------------------------------------------ */
#define VL6180X_I2C_TIMEOUT_MS          20U
#define VL6180X_BOOT_DELAY_MS           5U      /* tras subir CE antes de hablarle */
#define VL6180X_PERIOD_MIN_MS           10U     /* periodo entre mediciones (pasos de 10 ms) */
#define VL6180X_PERIOD_MAX_MS           2550U
#define VL6180X_SAMPLE_MARGIN_MS        20U     /* margen extra sobre el periodo al esperar el EXTI */

#define VL6180X_DISTANCE_INVALID        0xFFFFU

/* ------------------------------------------------------------------ */
/* Tipos                                                              */
/* ------------------------------------------------------------------ */
typedef struct {
    I2CBus_t *bus;
    uint8_t address;            /* direccion 7 bits actual */
    osSemaphoreId_t ready_sem;  /* binario (max 1): lo libera el EXTI de GPIO1 cuando hay muestra */
    uint16_t exti_pin;          /* pin GPIO1 (interrupcion). Numero de pin unico por sensor */
    GPIO_TypeDef *ce_port;      /* puerto del pin CE/GPIO0 (apaga/enciende el sensor) */
    uint16_t ce_pin;
    uint16_t period_ms;         /* periodo del modo continuo */
    uint8_t isSensor_start;
} VL6180X_Handle_t;

typedef enum {
    VL6180X_DATA_OK,
    VL6180X_DATA_EMPTY,
} VL6180X_Data_Status_t;

typedef struct {
    uint16_t distance_mm;               /* VL6180X_DISTANCE_INVALID si no hay dato valido */
    VL6180X_Data_Status_t range_status; /* OK / EMPTY */
    uint8_t error_code;                 /* codigo crudo de RESULT__RANGE_STATUS (0 = sin error) */
} VL6180X_Range_t;

/* ------------------------------------------------------------------ */
/* API                                                                */
/* ------------------------------------------------------------------ */

/* Prepara el handle y deja el sensor apagado (CE en bajo).
 * Llamar a Init de TODOS los sensores antes de hacer Start de cualquiera. */
HAL_StatusTypeDef VL6180X_Init(VL6180X_Handle_t *sensor, I2CBus_t *bus, osSemaphoreId_t ready_sem,
                               uint16_t exti_pin, GPIO_TypeDef *ce_port, uint16_t ce_pin);

/* Enciende solo este sensor (CE en alto) y le asigna new_address. CE se queda en alto. */
HAL_StatusTypeDef VL6180X_SetAddress(VL6180X_Handle_t *sensor, uint8_t new_address);

/* SetAddress + configuracion + arranque del modo continuo.
 * Debe llamarse desde una tarea (usa osDelay y el bus). Uno por uno. */
HAL_StatusTypeDef VL6180X_Start(VL6180X_Handle_t *sensor, uint8_t address, uint16_t period_ms);

/* Espera (bloqueando solo la tarea) a la siguiente muestra y la lee. */
HAL_StatusTypeDef VL6180X_ReadDistance(VL6180X_Handle_t *sensor, VL6180X_Range_t *result);

/* Llamar desde HAL_GPIO_EXTI_Callback para cada sensor. */
void VL6180X_OnExtiCb(VL6180X_Handle_t *sensor, uint16_t GPIO_PIN);

#endif /* INC_VL6180X_H_ */