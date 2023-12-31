/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CO_app_STM32.h"
#include "OD.h"
#include "tim.h"
#include "comment.h"
#include "can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
volatile uint16_t rpm = 0; // volatile use to let gdp get value
volatile int timer_counter = 0;
volatile int encoder_raw_data = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */
/* Definitions for RPM_CalcuateTas */
osThreadId_t RPM_CalcuateTasHandle;
const osThreadAttr_t RPM_CalcuateTas_attributes = {
    .name = "RPM_CalcuateTas",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
/* Definitions for canopenTask */
osThreadId_t canopenTaskHandle;
const osThreadAttr_t canopenTask_attributes = {
    .name = "canopenTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityRealtime,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartRPM_Calcuate(void *argument);
void canopen_task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of RPM_CalcuateTas */
  RPM_CalcuateTasHandle = osThreadNew(StartRPM_Calcuate, NULL, &RPM_CalcuateTas_attributes);

  /* creation of canopenTask */
  canopenTaskHandle = osThreadNew(canopen_task, NULL, &canopenTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartRPM_Calcuate */
/**
 * @brief  Function implementing the RPM_CalcuateTas thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartRPM_Calcuate */
void StartRPM_Calcuate(void *argument)
{
  /* USER CODE BEGIN StartRPM_Calcuate */
  /* Infinite loop */

  for (;;)
  {
    CO_TPDOsendRequest(&canopenNodeSTM32->canOpenStack->TPDO[0]);
    osDelay(100);
  }
  /* USER CODE END StartRPM_Calcuate */
}

/* USER CODE BEGIN Header_canopen_task */
/**
 * @brief Function implementing the canopenTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_canopen_task */
void canopen_task(void *argument)
{
  /* USER CODE BEGIN canopen_task */
  CANopenNodeSTM32 canOpenNodeSTM32;
  canOpenNodeSTM32.CANHandle = &hcan;
  canOpenNodeSTM32.HWInitFunction = MX_CAN_Init;
  canOpenNodeSTM32.timerHandle = &htim17;
  canOpenNodeSTM32.desiredNodeID = 18;
  canOpenNodeSTM32.baudrate = 125;
  canopen_app_init(&canOpenNodeSTM32);
  /* Infinite loop */
  for (;;)
  {
    canopen_app_process();
    vTaskDelay(1);
  }
  /* USER CODE END canopen_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim == canopenNodeSTM32->timerHandle)
  {
    canopen_app_interrupt();
  }
  if (htim->Instance == TIM6)
  {
    if (timer_counter == 1)
    {
      encoder_raw_data = (int)((TIM2->CNT) >> 2);
      if (encoder_raw_data < 0) // prevent over flow
      {
        encoder_raw_data *= -1;
      }
      if (encoder_raw_data > 10000000)
      {
        timer_counter = 0;
        TIM2->CNT &= 0x0;
      }
      else
      {
        rpm = encoder_raw_data;
        OD_PERSIST_COMM.x6000_rpm_data = rpm /2;
        OD_set_u16(OD_find(OD, 0x6000), 0x000, OD_PERSIST_COMM.x6000_rpm_data, false);
        timer_counter = 0;
        TIM2->CNT &= 0x0;
      }
    }
    else
    {
      timer_counter += 1;
    }
  }
  /* USER CODE END Callback 1 */
}
/* USER CODE END Application */
