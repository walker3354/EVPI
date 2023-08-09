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
#include "stdlib.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CO_app_STM32.h"
#include "OD.h"
#include "tim.h"
#include "comment.h"
#include "can.h"
#include "pid.h"
#include "adc.h"
#define portTICK_RATE_MS portTICK_PERIOD_MS
#define portTICK_PERIOD_MS ((TickType_t)1000 / configTICK_RATE_HZ)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
/* Definitions for PIDTask */
osThreadId_t PIDTaskHandle;
const osThreadAttr_t PIDTask_attributes = {
    .name = "PIDTask",
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
volatile double CCR_value = 0;
volatile double current_voltage = 0;
volatile double current_angle = 0;
volatile double db_target_angle = 0;
int volatge_counter = 0;
int error_counter = 0;

bool PID_lock = false;
bool init_flag = false;
int init_counter = 0;

void analyze_volatge(void);
void set_db_target_angle(double temp);
void steering_prevent(void);
void PID_prevent(void);
void PID_Error_handler(void);
void init_temp_buffer(void);

double voltage_temp[10] = {0};
double error_temp[10] = {0};

/* USER CODE END FunctionPrototypes */

void StartPIDTask(void *argument);
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
  /* creation of PIDTask */
  PIDTaskHandle = osThreadNew(StartPIDTask, NULL, &PIDTask_attributes);

  /* creation of canopenTask */
  canopenTaskHandle = osThreadNew(canopen_task, NULL, &canopenTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartPIDTask */
/**
 * @brief  Function implementing the PIDTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartPIDTask */
void StartPIDTask(void *argument)
{
  /* USER CODE BEGIN StartPIDTask */
  TIM2->CCR2 = (int)CCR_value; // CCR_value bigger the rpm will highter 0~100
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_GPIO_WritePin(CW_CCW_control_GPIO_Port, CW_CCW_control_Pin, 1);
  analyze_volatge();
  set_db_target_angle(35.0);
  PID_TypeDef TPID;
  PID(&TPID, &current_angle, &CCR_value, &db_target_angle, 1, 0.15, 0, _PID_P_ON_E, _PID_CD_DIRECT); // input output target
  PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&TPID, 100);
  PID_SetOutputLimits(&TPID, -70, 70);
  /* Infinite loop */
  for (;;)
  {
    analyze_volatge();
    PID_Compute(&TPID);
    PID_prevent();
    if (current_angle < db_target_angle)
    {
      HAL_GPIO_WritePin(CW_CCW_control_GPIO_Port, CW_CCW_control_Pin, 0);
    }
    else
    {
      HAL_GPIO_WritePin(CW_CCW_control_GPIO_Port, CW_CCW_control_Pin, 1);
    }
    if (PID_lock == false)
    {
      CCR_value = abs(CCR_value);
      TIM2->CCR2 = ((int)CCR_value);
    }
    else
    {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
    osDelay(100);
  }
  /* USER CODE END StartPIDTask */
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
  canOpenNodeSTM32.desiredNodeID = 17;
  canOpenNodeSTM32.baudrate = 125;
  canopen_app_init(&canOpenNodeSTM32);
  int16_t pre_target_angle = OD_PERSIST_COMM.x6001_target_angle;
  db_target_angle = OD_PERSIST_COMM.x6001_target_angle;
  /* Infinite loop */
  for (;;)
  {
    canopen_app_process();
    if (OD_PERSIST_COMM.x6000_current_angle != pre_target_angle)
    {
      pre_target_angle = OD_PERSIST_COMM.x6001_target_angle;
      db_target_angle = (double)OD_PERSIST_COMM.x6001_target_angle;
    }
    vTaskDelay(1);
  }
  /* USER CODE END canopen_task */
}
/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void analyze_volatge(void)
{
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1);
  current_voltage = HAL_ADC_GetValue(&hadc1);
  current_voltage = HAL_ADC_GetValue(&hadc1);
  current_voltage = ((current_voltage / 4096) * 3.3);
  current_angle = (((current_voltage)*2 * 22.5) - 45);
  current_angle = current_angle / 1;
  steering_prevent();
  if (init_counter > 2)
  {
    init_flag = true;
  }
  else
  {
    init_counter += 1;
  }
}

void steering_prevent(void)
{
  voltage_temp[volatge_counter] = current_voltage;
  volatge_counter = (volatge_counter + 1) % 10;
  if (init_flag)
  {
    if (voltage_temp[volatge_counter] > 2.5 && voltage_temp[volatge_counter] < 0.25) // out of range 4.5v ~ 0.5
    {
      PID_Error_handler();
    }

    if (abs(*(voltage_temp) - *(voltage_temp + 9)) < 0.02 && abs(current_angle - db_target_angle) > 3) // static voltage 0.03
    {
      PID_Error_handler();
    }
    if (voltage_temp[volatge_counter] - voltage_temp[(volatge_counter == 0 ? 9 : volatge_counter - 1)] > 1) // voltage difference too high
    {
      PID_Error_handler();
    }
  }
}
void PID_prevent(void)
{
  error_temp[error_counter] = abs(current_angle - db_target_angle);
  if (init_flag)
  {
    if (abs(current_angle - db_target_angle) > 10 && abs(*(error_temp) - *(error_temp + 9)) < 1) // 3 error too low
    {
      PID_Error_handler();
    }
  }
  error_counter = (error_counter + 1) % 10;
}

void set_db_target_angle(double temp)
{
  db_target_angle = temp;
}

void PID_Error_handler(void)
{
  TIM2->CCR2 = 0;
  PID_lock = true;
}

/* USER CODE END Application */
