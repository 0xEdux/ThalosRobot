/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "LD2450.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum { MOTORA, MOTORB } RobotMotors_t;
typedef enum { MOTOR_FORWARD, MOTOR_BACKWARD, MOTOR_BRAKE } MotorDirections_t;

typedef struct {
  LD2450_Handle_t       *radar;
  uint8_t                id;
  UART_HandleTypeDef    *huart;
  osMessageQueueId_t     queue;
  osSemaphoreId_t        sem;
} RadarTaskCfg_t;

typedef struct {
  int16_t x, y, speed;
  uint8_t src;             // bit0 = visto por R1, bit1 = visto por R2
} Fused_Target_t;

#define MAX_FUSED    6

typedef struct {
  uint32_t timestamp;
  uint8_t  count;
  Fused_Target_t t[MAX_FUSED];
} Fused_Msg_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define T_HEARTBEAT_MS      250U      // Cambio de estado del LED verde (Task Heartbeat)

#define DISTANCE_T   45   // mm entre radares
#define MAX_AGE_MS   150   // un frame más viejo se descarta
#define GATE_MM      300   // distancia máxima para considerar el mismo blanco

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
// #define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* Definitions for motionTask */
osThreadId_t motionTaskHandle;
const osThreadAttr_t motionTask_attributes = {
  .name = "motionTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for heartbeatTask */
osThreadId_t heartbeatTaskHandle;
const osThreadAttr_t heartbeatTask_attributes = {
  .name = "heartbeatTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for telemetryTask */
osThreadId_t telemetryTaskHandle;
const osThreadAttr_t telemetryTask_attributes = {
  .name = "telemetryTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for process1Trackin */
osThreadId_t process1TrackinHandle;
const osThreadAttr_t process1Trackin_attributes = {
  .name = "process1Trackin",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for process2Trackin */
osThreadId_t process2TrackinHandle;
const osThreadAttr_t process2Trackin_attributes = {
  .name = "process2Trackin",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for fusionLD */
osThreadId_t fusionLDHandle;
const osThreadAttr_t fusionLD_attributes = {
  .name = "fusionLD",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for trackingQueue */
osMessageQueueId_t trackingQueueHandle;
const osMessageQueueAttr_t trackingQueue_attributes = {
  .name = "trackingQueue"
};
/* Definitions for fusedQueue */
osMessageQueueId_t fusedQueueHandle;
const osMessageQueueAttr_t fusedQueue_attributes = {
  .name = "fusedQueue"
};
/* Definitions for telemetryMutex */
osMutexId_t telemetryMutexHandle;
const osMutexAttr_t telemetryMutex_attributes = {
  .name = "telemetryMutex"
};
/* Definitions for ldRxSem1 */
osSemaphoreId_t ldRxSem1Handle;
const osSemaphoreAttr_t ldRxSem1_attributes = {
  .name = "ldRxSem1"
};
/* Definitions for ldRxSem2 */
osSemaphoreId_t ldRxSem2Handle;
const osSemaphoreAttr_t ldRxSem2_attributes = {
  .name = "ldRxSem2"
};
/* USER CODE BEGIN PV */

RadarTaskCfg_t cfg1, cfg2;

LD2450_Handle_t radar1;
LD2450_Handle_t radar2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
void StartMotionTask(void *argument);
void StartHeartbeathTask(void *argument);
void StartTelemetryTask(void *argument);
void StartProcessTracking1(void *argument);
void StartProcessTracking2(void *argument);
void StartFusionLD(void *argument);

/* USER CODE BEGIN PFP */

uint8_t normalize(const LD2450_Msg_t *m, int16_t x_offset, uint8_t src, Fused_Target_t *out) {
    uint8_t n = 0;

    for(int i = 0; i < 3; i++) {
      const LD2450_Target_t *t = &m->frame.t[i];
      if(!target_valid(t)) continue;

      out[n].x = t->x + x_offset;
      out[n].y = t->y;
      out[n].speed = t->speed;
      out[n].src = src;

      n++;
    }

    return n;
}

void push(LD2450_Msg_t *out, uint8_t *count, const LD2450_Target_t *t) {
  if(*count >= 3) return;
  out->frame.t[*count] = *t;

  (*count)++;
}

void RadarTask_Run(RadarTaskCfg_t *config) {
  // Cast the argument to the radar task configuration structure
  RadarTaskCfg_t *c = (RadarTaskCfg_t *)config;

  // Initialize the radar peripheral; enter error handler if initialization fails
  if (LD2450_Init(c->radar, c->id, c->huart, c->queue, c->sem) != HAL_OK)
  {
      Error_Handler();
  }

  /* Infinite loop */
  for (;;)
  {
      // Process radar data; if it fails, delay execution to free up CPU
      if (LD2450_Process(c->radar) != HAL_OK)
      {
          osDelay(100);
      }
  }
}

void set_motor_dir(RobotMotors_t motor, MotorDirections_t dir) {
	switch (dir) {
		case MOTOR_FORWARD:
			if(motor == MOTORA) {
	      HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
			} else if(motor == MOTORB) {
        HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, GPIO_PIN_RESET);
      }
			break;

		case MOTOR_BACKWARD:
    	if(motor == MOTORA) {
	      HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_SET);
			} else if(motor == MOTORB) {
        HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, GPIO_PIN_SET);
      }
			break;
		case MOTOR_BRAKE:
			if(motor == MOTORA) {
	      HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_SET);
			} else if(motor == MOTORB) {
        HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, GPIO_PIN_SET);
      }
			break;
		default:
			break;
	}
}

void UART_Print(const char *msg){
	osMutexAcquire(telemetryMutexHandle, osWaitForever);
	HAL_UART_Transmit(&huart3, (const uint8_t *)msg, (uint16_t)strlen(msg), 100);
	osMutexRelease(telemetryMutexHandle);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of telemetryMutex */
  telemetryMutexHandle = osMutexNew(&telemetryMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */

  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of ldRxSem1 */
  ldRxSem1Handle = osSemaphoreNew(1, 0, &ldRxSem1_attributes);

  /* creation of ldRxSem2 */
  ldRxSem2Handle = osSemaphoreNew(1, 0, &ldRxSem2_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of trackingQueue */
  trackingQueueHandle = osMessageQueueNew (6, sizeof(LD2450_Msg_t), &trackingQueue_attributes);

  /* creation of fusedQueue */
  fusedQueueHandle = osMessageQueueNew (8, sizeof(Fused_Msg_t), &fusedQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  cfg1 = (RadarTaskCfg_t){ &radar1, 1, &huart2, trackingQueueHandle, ldRxSem1Handle };
  cfg2 = (RadarTaskCfg_t){ &radar2, 2, &huart4, trackingQueueHandle, ldRxSem2Handle };
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of motionTask */
  motionTaskHandle = osThreadNew(StartMotionTask, NULL, &motionTask_attributes);

  /* creation of heartbeatTask */
  heartbeatTaskHandle = osThreadNew(StartHeartbeathTask, NULL, &heartbeatTask_attributes);

  /* creation of telemetryTask */
  telemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL, &telemetryTask_attributes);

  /* creation of process1Trackin */
  process1TrackinHandle = osThreadNew(StartProcessTracking1, NULL, &process1Trackin_attributes);

  /* creation of process2Trackin */
  process2TrackinHandle = osThreadNew(StartProcessTracking2, NULL, &process2Trackin_attributes);

  /* creation of fusionLD */
  fusionLDHandle = osThreadNew(StartFusionLD, NULL, &fusionLD_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 256000;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 256000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, MOTOR_IN3_Pin|MOTOR_IN1_Pin|MOTOR_IN2_Pin|MOTOR_IN4_Pin
                          |LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOTOR_IN3_Pin MOTOR_IN1_Pin MOTOR_IN2_Pin MOTOR_IN4_Pin
                           LD3_Pin */
  GPIO_InitStruct.Pin = MOTOR_IN3_Pin|MOTOR_IN1_Pin|MOTOR_IN2_Pin|MOTOR_IN4_Pin
                          |LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(B1_EXTI_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(B1_EXTI_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  LD2450_RxEventCb(&radar1, huart, Size);
  LD2450_RxEventCb(&radar2, huart, Size);
}

// overrun ó framing: reload
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  LD2450_ErrorCb(&radar1, huart);
  LD2450_ErrorCb(&radar2, huart);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartMotionTask */
/**
  * @brief  Function implementing the mottionTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMotionTask */
void StartMotionTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    set_motor_dir(MOTORA, MOTOR_FORWARD);
    set_motor_dir(MOTORB, MOTOR_BACKWARD);

  /* Infinite loop */
  for(;;)
  {

	  for(int ccr = 0; ccr < 990; ccr += 5) {
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr);
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ccr);
		  osDelay(100);
	  }

	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartHeartbeathTask */
/**
* @brief Function implementing the heartbeatTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHeartbeathTask */
void StartHeartbeathTask(void *argument)
{
  /* USER CODE BEGIN StartHeartbeathTask */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	  osDelay(T_HEARTBEAT_MS);
    UART_Print("Heartbeat!!\r\n");
  }
  /* USER CODE END StartHeartbeathTask */
}

/* USER CODE BEGIN Header_StartTelemetryTask */
/**
* @brief Function implementing the telemetryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTelemetryTask */
void StartTelemetryTask(void *argument)
{
  /* USER CODE BEGIN StartTelemetryTask */
  /* Infinite loop */

  Fused_Msg_t m;
  char line[160];
  for(;;)
  {
    osStatus_t status = osMessageQueueGet(fusedQueueHandle, &m, NULL, 200);
    if (status == osOK) {
      size_t n = 0;

      for (int i = 0; i < m.count; i++) {
        const Fused_Target_t *t = &m.t[i];

        n += snprintf(line + n, sizeof(line) - n,
                      "T%d x=%d y=%d v=%d src=%u | ",
                      i + 1, t->x, t->y, t->speed, (unsigned)t->src);
        if (n >= sizeof(line) - 3) break;
      }

      if (n == 0) {
        snprintf(line, sizeof(line), "sin blancos\r\n");
      } else {
        n -= 3;
        snprintf(line + n, sizeof(line) - n, "\r\n");
      }
      UART_Print(line);
    } else if (status == osErrorTimeout) {
      UART_Print("radar sin datos\r\n");
    }
  }
  /* USER CODE END StartTelemetryTask */
}

/* USER CODE BEGIN Header_StartProcessTracking1 */
/**
* @brief Function implementing the process1Trackin thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartProcessTracking1 */
void StartProcessTracking1(void *argument)
{
  /* USER CODE BEGIN StartProcessTracking1 */
  /* Infinite loop */
  RadarTask_Run(&cfg1);
  /* USER CODE END StartProcessTracking1 */
}

/* USER CODE BEGIN Header_StartProcessTracking2 */
/**
* @brief Function implementing the process2Trackin thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartProcessTracking2 */
void StartProcessTracking2(void *argument)
{
  /* USER CODE BEGIN StartProcessTracking2 */
  /* Infinite loop */
  RadarTask_Run(&cfg2);
  /* USER CODE END StartProcessTracking2 */
}

/* USER CODE BEGIN Header_StartFusionLD */
/**
* @brief Function implementing the fusionLD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFusionLD */
void StartFusionLD(void *argument)
{
  /* USER CODE BEGIN StartFusionLD */
  /* Infinite loop */


  LD2450_Msg_t m;
  LD2450_Msg_t latest[2];

  // Bandera para verificar data inicial
  uint8_t have[2] = {0, 0};

  for (;;)
  {
    osStatus_t status = osMessageQueueGet(trackingQueueHandle, &m, NULL, 200);
    if (status != osOK) continue;
    if (m.id < 1 || m.id > 2) continue;

    latest[m.id - 1] = m;
    have[m.id - 1] = 1;

    uint32_t now = osKernelGetTickCount();
    uint8_t fresh[2];
    for (int r = 0; r < 2; r++) {
      fresh[r] = have[r] && ((now - latest[r].timestamp) < MAX_AGE_MS);
    }

    Fused_Target_t a[3], b[3];
    uint8_t na = fresh[0] ? normalize(&latest[0], -(DISTANCE_T / 2), 0x01, a) : 0;
    uint8_t nb = fresh[1] ? normalize(&latest[1],  (DISTANCE_T / 2), 0x02, b) : 0;
    uint8_t used_b[3] = {0, 0, 0};

    Fused_Msg_t out = { .timestamp = now, .count = 0 };

    // Cada blanco de R1: buscar el más cercano de R2
    for (int i = 0; i < na; i++) {
      int best = -1;
      int32_t best_d2 = (int32_t)GATE_MM * GATE_MM;

      for (int j = 0; j < nb; j++) {
        if (used_b[j]) continue;
        int32_t dx = a[i].x - b[j].x;
        int32_t dy = a[i].y - b[j].y;
        int32_t d2 = dx * dx + dy * dy;
        if (d2 < best_d2) { best_d2 = d2; best = j; }
      }

      Fused_Target_t *o = &out.t[out.count++];
      if (best >= 0) {
        used_b[best] = 1;
        o->x     = (a[i].x + b[best].x) / 2;
        o->y     = (a[i].y + b[best].y) / 2;
        o->speed = (a[i].speed + b[best].speed) / 2;
        o->src   = 0x03;
      } else {
        *o = a[i];
      }
    }

    // Los de R2 que no se asociaron pasan solos
    for (int j = 0; j < nb; j++) {
      if (!used_b[j]) out.t[out.count++] = b[j];
    }

    osMessageQueuePut(fusedQueueHandle, &out, 0, 0);
  }
  /* USER CODE END StartFusionLD */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
