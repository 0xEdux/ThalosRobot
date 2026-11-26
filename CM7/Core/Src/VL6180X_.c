/*
 * VL6180X.c
 *
 *  Created on: 24 nov 2026
 *      Author: juanb
 */


#include "VL6180x.h"
#include <stddef.h>
#include "cmsis_os2.h"

// Los VL6180 usan direccion de registro de 16 bits
//
// Transporte por interrupciones (HAL_I2C_Mem_xxx_IT):
//  - i2c1MutexHandle    : serializa el acceso al bus (como antes)
//  - i2c1DoneSemHandle  : semaforo binario, cuenta inicial 0. Lo libera la ISR
//                         (callbacks de abajo) cuando termina la transferencia.
// Fin de medicion por interrupcion del sensor (GPIO1 -> EXTI, flanco de bajada):
//  - sensor->ready_sem : semaforo binario por sensor (se crea en VL6180X_Create).
//  - Si ready_sem es NULL se cae a polling del registro 0x004F.
//
// Requiere I2C1 event/error IRQ y el EXTI de cada GPIO1 habilitados en el NVIC,
// con prioridad numericamente >= configMAX_SYSCALL_INTERRUPT_PRIORITY.
// Todas las funciones (salvo VL6180X_OnInterrupt) se llaman desde una tarea.

extern osMutexId_t i2c1MutexHandle;
extern osSemaphoreId_t i2c1DoneSemHandle;

#define VL6180X_I2C_TIMEOUT_MS    20U
#define VL6180X_ABORT_TIMEOUT_MS  5U
#define VL6180X_READY_TIMEOUT_MS  100U
#define VL6180X_RANGE_TIMEOUT_MS  100U

// Solo se toca con el mutex tomado; la ISR lo escribe antes de liberar el semaforo.
static volatile uint8_t i2c1_error = 0U;

/* ---------- Callbacks de la HAL (ISR) ---------- */

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
    if(hi2c->Instance == I2C1) osSemaphoreRelease(i2c1DoneSemHandle);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c){
    if(hi2c->Instance == I2C1) osSemaphoreRelease(i2c1DoneSemHandle);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
    if(hi2c->Instance == I2C1){
        i2c1_error = 1U;
        osSemaphoreRelease(i2c1DoneSemHandle);
    }
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c){
    if(hi2c->Instance == I2C1) osSemaphoreRelease(i2c1DoneSemHandle);
}

/* ---------- Utilidades internas ---------- */

static int VL6180X_IsValid(const VL6180X_t *sensor){
    return sensor != NULL && sensor->i2c != NULL &&
        sensor->address_7bit >= 0x08U &&
        sensor->address_7bit <= 0x77U;
}

// Espera el fin de una transferencia IT. Se llama con el mutex tomado.
static HAL_StatusTypeDef VL6180X_WaitTransfer(
    const VL6180X_t *sensor,
    uint16_t addr
){
    if(osSemaphoreAcquire(i2c1DoneSemHandle, VL6180X_I2C_TIMEOUT_MS) != osOK){
        // Timeout: abortamos y esperamos a que la HAL termine de tocar el buffer
        if(HAL_I2C_Master_Abort_IT(sensor->i2c, addr) == HAL_OK){
            (void)osSemaphoreAcquire(i2c1DoneSemHandle, VL6180X_ABORT_TIMEOUT_MS);
        }
        return HAL_TIMEOUT;
    }

    return i2c1_error ? HAL_ERROR : HAL_OK;
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

    uint16_t addr = (uint16_t)(sensor->address_7bit << 1);

    // Limpia tokens viejos (por ejemplo, de un timeout anterior)
    while(osSemaphoreAcquire(i2c1DoneSemHandle, 0U) == osOK){}
    i2c1_error = 0U;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_IT(
        sensor->i2c, addr, reg, I2C_MEMADD_SIZE_16BIT, value, 1U
    );

    if(status == HAL_OK){
        status = VL6180X_WaitTransfer(sensor, addr);
    }

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

    uint16_t addr = (uint16_t)(sensor->address_7bit << 1);

    while(osSemaphoreAcquire(i2c1DoneSemHandle, 0U) == osOK){}
    i2c1_error = 0U;

    // &value es local: por eso NO se sale de la funcion hasta que termina la transferencia
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write_IT(
        sensor->i2c, addr, reg, I2C_MEMADD_SIZE_16BIT, &value, 1U
    );

    if(status == HAL_OK){
        status = VL6180X_WaitTransfer(sensor, addr);
    }

    osMutexRelease(i2c1MutexHandle);
    return status;
}

// Espera a que (reg & mask) == expected. Entre lecturas la tarea duerme.
static HAL_StatusTypeDef VL6180X_Wait(
    const VL6180X_t *sensor,
    uint16_t reg,
    uint8_t mask,
    uint8_t expected,
    uint32_t timeout_ms
){
    uint32_t start = HAL_GetTick();

    for(;;){
        uint8_t value = 0;
        HAL_StatusTypeDef status = VL6180X_Read(sensor, reg, &value);

        if(status != HAL_OK) return status;
        if((value & mask) == expected) return HAL_OK;

        if((uint32_t)(HAL_GetTick() - start) >= timeout_ms){
            return HAL_TIMEOUT;
        }

        osDelay(1U);
    }
}

/* ---------- API publica ---------- */

HAL_StatusTypeDef VL6180X_Create(
    VL6180X_t *sensor,
    I2C_HandleTypeDef *i2c,
    uint8_t address_7bit
){
    if(sensor == NULL || i2c == NULL) return HAL_ERROR;
    if(address_7bit < 0x08U || address_7bit > 0x77U) return HAL_ERROR;

    sensor->i2c = i2c;
    sensor->address_7bit = address_7bit;
    sensor->ready_sem = osSemaphoreNew(1U, 0U, NULL);

    return (sensor->ready_sem != NULL) ? HAL_OK : HAL_ERROR;
}

// ISR-safe: llamar desde HAL_GPIO_EXTI_Falling_Callback
void VL6180X_OnInterrupt(const VL6180X_t *sensor){
    if(sensor != NULL && sensor->ready_sem != NULL){
        osSemaphoreRelease(sensor->ready_sem);
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

// Paso 1: espera que el sensor este listo, limpia interrupciones y dispara una medicion.
HAL_StatusTypeDef VL6180X_StartRange(const VL6180X_t *sensor){
    HAL_StatusTypeDef status = VL6180X_Wait(
        sensor, 0x004DU, 0x01U, 0x01U, VL6180X_READY_TIMEOUT_MS
    );
    if(status != HAL_OK) return status;

    status = VL6180X_Write(sensor, 0x0015U, 0x07U);   // limpia interrupciones (GPIO1 vuelve a alto)
    if(status != HAL_OK) return status;

    // Descarta un flanco viejo antes de disparar
    if(sensor->ready_sem != NULL){
        while(osSemaphoreAcquire(sensor->ready_sem, 0U) == osOK){}
    }

    return VL6180X_Write(sensor, 0x0018U, 0x01U);     // inicia medicion individual
}

// Paso 2: espera a que la muestra este lista (interrupcion de GPIO1; polling si no hay semaforo).
HAL_StatusTypeDef VL6180X_WaitRangeReady(const VL6180X_t *sensor){
    if(sensor == NULL) return HAL_ERROR;

    if(sensor->ready_sem != NULL){
        return (osSemaphoreAcquire(sensor->ready_sem, VL6180X_RANGE_TIMEOUT_MS) == osOK)
            ? HAL_OK : HAL_TIMEOUT;
    }

    return VL6180X_Wait(sensor, 0x004FU, 0xC7U, 0x04U, VL6180X_RANGE_TIMEOUT_MS);
}

// Paso 3: lee estado y distancia.
HAL_StatusTypeDef VL6180X_CollectRange(
    const VL6180X_t *sensor,
    VL6180X_Range_t *result
){
    if(result == NULL) return HAL_ERROR;

    result->distance_mm = 0xFFFFU;
    result->range_status = 0xFFU;

    uint8_t raw_status = 0;
    uint8_t distance = 0;

    HAL_StatusTypeDef status = VL6180X_Read(sensor, 0x004DU, &raw_status);
    if(status != HAL_OK) return status;

    status = VL6180X_Read(sensor, 0x0062U, &distance);
    if(status != HAL_OK) return status;

    result->range_status = (uint8_t)((raw_status >> 4) & 0x0FU);

    if(result->range_status == 0U){
        result->distance_mm = distance;
    }

    return HAL_OK;
}

// Version de un solo sensor (misma firma que antes).
HAL_StatusTypeDef VL6180X_ReadRange(
    const VL6180X_t *sensor,
    VL6180X_Range_t *result
){
    if(result == NULL) return HAL_ERROR;

    result->distance_mm = 0xFFFFU;
    result->range_status = 0xFFU;

    HAL_StatusTypeDef status = VL6180X_StartRange(sensor);
    if(status != HAL_OK) return status;

    status = VL6180X_WaitRangeReady(sensor);
    if(status != HAL_OK) return status;

    return VL6180X_CollectRange(sensor, result);
}

// Version para varios sensores: dispara todos y luego espera/recoge.
// El tiempo total es ~ una medicion en lugar de n.
// results[i] queda con 0xFFFF/0xFF si ese sensor falla; status[i] guarda su codigo.
HAL_StatusTypeDef VL6180X_ReadRangeMulti(
    const VL6180X_t *const sensors[],
    VL6180X_Range_t results[],
    HAL_StatusTypeDef statuses[],
    size_t count
){
    if(sensors == NULL || results == NULL || statuses == NULL) return HAL_ERROR;

    HAL_StatusTypeDef overall = HAL_OK;

    for(size_t i = 0; i < count; ++i){
        results[i].distance_mm = 0xFFFFU;
        results[i].range_status = 0xFFU;
        statuses[i] = VL6180X_StartRange(sensors[i]);
    }

    for(size_t i = 0; i < count; ++i){
        if(statuses[i] == HAL_OK) statuses[i] = VL6180X_WaitRangeReady(sensors[i]);
        if(statuses[i] == HAL_OK) statuses[i] = VL6180X_CollectRange(sensors[i], &results[i]);
        if(statuses[i] != HAL_OK) overall = statuses[i];
    }

    return overall;
}