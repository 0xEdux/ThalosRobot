/*
 * LD2450.h
 *
 *  Created on: 16 nov 2026
 *      Author: juanb
 */

#ifndef INC_LD2450_H_
#define INC_LD2450_H_

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "main.h"


#define LD2450_DMA_BUF_SIZE 128U       // 4 tramas del LD2450

typedef enum { LD2450_IDLE, LD2450_DATA, LD2450_TAIL } LD2450_States_t;

typedef struct { 
  int16_t x;
  int16_t y;
  int16_t speed;
  uint16_t res;
} LD2450_Target_t;

typedef struct { LD2450_Target_t t[3]; } LD2450_Frame_t;

typedef struct {
    uint8_t         id;
    LD2450_Frame_t  frame;
} LD2450_Msg_t;

typedef struct {
    uint8_t id;

    LD2450_States_t state;
    uint8_t byteCnt;
    uint8_t frameData[24];
    LD2450_Frame_t targets;

    osMessageQueueId_t trackingQueueHandle;
    osSemaphoreId_t rxSemHandle;

    UART_HandleTypeDef *huart;
    uint8_t dmaBuf[LD2450_DMA_BUF_SIZE] __attribute__((aligned(32)));

    volatile uint16_t head;
    volatile uint16_t tail;
    uint8_t error;

    uint8_t initialized;


} __attribute__((aligned(32))) LD2450_Handle_t;



int16_t LD2450_Decode(uint16_t raw);

HAL_StatusTypeDef LD2450_Feed(LD2450_Handle_t *radar, uint8_t value );
HAL_StatusTypeDef LD2450_Process(LD2450_Handle_t *radar);

HAL_StatusTypeDef LD2450_Init(LD2450_Handle_t *radar, uint8_t id, UART_HandleTypeDef *huart, osMessageQueueId_t trackingQueueHandle, osSemaphoreId_t rxSemHandle);

HAL_StatusTypeDef LD2450_RxEventCb(LD2450_Handle_t *radar, UART_HandleTypeDef *huart,  uint16_t size);
HAL_StatusTypeDef LD2450_ErrorCb(LD2450_Handle_t *radar, UART_HandleTypeDef *huart);

#endif /* INC_LD2450_H_ */
