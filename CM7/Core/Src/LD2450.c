/*
 * LD2450.c
 *
 *  Created on: 16 nov 2026
 *      Author: juanb
 */


#include "LD2450.h"

int16_t LD2450_Decode(uint16_t raw) {
    int16_t mag = raw & 0x7FFF;
    return (raw & 0x8000) ? mag : -mag;
}



HAL_StatusTypeDef LD2450_Feed(LD2450_Handle_t *radar, uint8_t value) {
  if(radar == NULL || radar->initialized == 0) {
    return HAL_ERROR;
  }

  switch (radar->state) {
      case LD2450_IDLE:
        if (radar->byteCnt == 0 && value == 0xAA) radar->byteCnt++;
        else if (radar->byteCnt == 1 && value == 0xFF) radar->byteCnt++;
        else if (radar->byteCnt == 2 && value == 0x03) radar->byteCnt++;
        else if (radar->byteCnt == 3 && value == 0x00) {
          radar->state = LD2450_DATA;
          radar->byteCnt = 0;
        }
        else if (value == 0xAA) radar->byteCnt = 1;
        else radar->byteCnt = 0;

        break;

      case LD2450_DATA:
        radar->frameData[radar->byteCnt++] = value;
        if (radar->byteCnt == 24) {
          radar->byteCnt = 0;
          radar->state = LD2450_TAIL;
        }

        break;

      case LD2450_TAIL:
        if (radar->byteCnt == 0 && value == 0x55) radar->byteCnt++;
        else if (radar->byteCnt == 1 && value == 0xCC) {
          // Frame complete
          for (uint8_t i = 0; i < 3; i++) {

            uint8_t *p = &radar->frameData[i * 8]; 
            radar->targets.t[i].x = LD2450_Decode(p[1] << 8 | p[0]);
            radar->targets.t[i].y = LD2450_Decode(p[3] << 8 | p[2]);
            radar->targets.t[i].speed = LD2450_Decode(p[5] << 8 | p[4]);
            radar->targets.t[i].res =  p[7] << 8 | p[6];
          }

          LD2450_Msg_t msg = { .id = radar->id, .frame = radar->targets };

          if (osMessageQueuePut(radar->trackingQueueHandle, &msg, 0, 0) == osErrorResource) {
            LD2450_Msg_t old;
            osMessageQueueGet(radar->trackingQueueHandle, &old, NULL, 0);
            osMessageQueuePut(radar->trackingQueueHandle, &msg, 0, 0);
          }

          radar->state = LD2450_IDLE;
          radar->byteCnt = 0;
        } else {
          radar->state = LD2450_IDLE;
          radar->byteCnt = 0;
        }

        break;
        
      default:
        break;
  }
  
  return HAL_OK;
}

HAL_StatusTypeDef LD2450_Process(LD2450_Handle_t *radar) {

  if(radar == NULL || radar->initialized == 0) {
    return HAL_ERROR;
  }

  osSemaphoreAcquire(radar->rxSemHandle, osWaitForever);

  if(radar->error) {
    HAL_UART_AbortReceive(radar->huart);
    radar->head = 0;
    radar->tail = 0;
    radar->error = 0;
    radar->state = LD2450_IDLE;
    radar->byteCnt = 0;
    
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(radar->huart, radar->dmaBuf, LD2450_DMA_BUF_SIZE);

    if(status != HAL_OK ) {
      radar->initialized = 0;
    }

    return HAL_ERROR;
  }

  SCB_InvalidateDCache_by_Addr((uint32_t *) radar->dmaBuf, LD2450_DMA_BUF_SIZE);

  while (radar->tail != radar->head) {
    uint8_t value = radar->dmaBuf[radar->tail];
    radar->tail = (radar->tail + 1) % LD2450_DMA_BUF_SIZE;

    // Verify the data frame
    LD2450_Feed(radar, value);
  }

  return HAL_OK;
}

HAL_StatusTypeDef LD2450_Init(LD2450_Handle_t *radar, uint8_t id, UART_HandleTypeDef *huart, osMessageQueueId_t trackingQueueHandle, osSemaphoreId_t rxSemHandle) {
    if (radar == NULL || huart == NULL) {
        return HAL_ERROR;
    }

    radar->huart = huart;
    radar->trackingQueueHandle = trackingQueueHandle;
    radar->rxSemHandle = rxSemHandle;
    radar->id = id;

    radar->state = LD2450_IDLE;
    radar->initialized = 0;
    radar->head = 0;
    radar->tail = 0;
    radar->error = 0;
    radar->byteCnt = 0;

    for (int i = 0; i < LD2450_DMA_BUF_SIZE; i++) {
        radar->dmaBuf[i] = 0;
    }

    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)radar->dmaBuf, LD2450_DMA_BUF_SIZE);

    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(radar->huart, radar->dmaBuf, LD2450_DMA_BUF_SIZE);

    radar->initialized = 1;

    
    if (status != HAL_OK) {
        radar->initialized = 0;
        return status;
    }

    return HAL_OK;
}

HAL_StatusTypeDef LD2450_RxEventCb(LD2450_Handle_t *radar, UART_HandleTypeDef *huart,  uint16_t size) {
  if(radar == NULL || radar->initialized == 0 ||  huart == NULL) {
    return HAL_ERROR;
  }

  if(huart == radar->huart) {
      radar->head = size % LD2450_DMA_BUF_SIZE;
      osSemaphoreRelease(radar->rxSemHandle);
      return HAL_OK;
  }

  return HAL_ERROR;
}

HAL_StatusTypeDef LD2450_ErrorCb(LD2450_Handle_t *radar, UART_HandleTypeDef *huart) {
  if(radar == NULL || radar->initialized == 0 ||  huart == NULL) {
    return HAL_ERROR;
  }

  if(huart == radar->huart) {
      radar->error = 1;
      osSemaphoreRelease(radar->rxSemHandle);
      return HAL_OK;
  }

  return HAL_ERROR;
}
