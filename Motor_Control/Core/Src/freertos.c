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
#include "pid.h"
#include "adc.h"
#define Virtual_target_angle -20
#define Timer_ARR 5
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
PID_TypeDef TPID;
volatile double CCR_value = 0;
volatile double current_voltage = 0;
volatile double current_angle = 0;
volatile double db_target_angle = 0;

int voltage_counter = 0;
volatile double previous_voltage = 0;
volatile double previos_error = 0;

bool protection_lock = false;
bool PID_safe_lock = false;

void TPDO_tarnsmit(uint8_t status, int16_t angle);
void analyze_voltage(void);
void PID_Error_handler(void);
void CW_CCW_deect(void);
void PID_init(void);
void PID_protection(void);
double abs(double a);
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
  PID_init();
  /* Infinite loop */
  for (;;)
  {
    analyze_voltage();
    CW_CCW_deect();
    PID_Compute(&TPID);
    CCR_value = abs(CCR_value);
    PID_protection();
    if (PID_safe_lock == false)
    {
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
void analyze_voltage(void)
{
  if (voltage_counter == 0)
  {
    previous_voltage = current_voltage;
    previos_error = abs(current_angle - db_target_angle);
  }
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1);
  current_voltage = HAL_ADC_GetValue(&hadc1);
  current_voltage = ((current_voltage / 4096) * 3.3);
  current_angle = (int)(((current_voltage * 2) - 1.6) * 25 - 25); // min_voltage => 1.6  mid_voltage => 2.6 Max_voltage => 4.4  25degree/1Voltage
  if (voltage_counter == 4)
  {
    protection_lock = true;
  }
  else
  {
    protection_lock = false;
  }
  voltage_counter = (voltage_counter + 1) % Timer_ARR;
}

void PID_protection(void)
{
  if (current_voltage * 2 > 4.2 || current_voltage * 2 < 1.8) // outof voltage range 4.4v ~ 1.6v
  {
    PID_Error_handler();
    TPDO_tarnsmit(2, 99); // 2.reach limit
  }
  if ((abs(current_voltage - previous_voltage) > 1 || abs(current_voltage - previous_voltage) < 0.03) && abs(current_angle - db_target_angle) > 5 && previous_voltage != 0 && protection_lock) // voltage difference too hight and static voltage
  {
    PID_Error_handler();
    TPDO_tarnsmit(3, 99); // 3. Angle reader abnormal
  }
  if (abs((current_angle - db_target_angle) - previos_error) < 2 && abs(current_angle - db_target_angle) > 5 && previos_error != 0 && protection_lock) // angle error
  {
    PID_Error_handler();
    TPDO_tarnsmit(4, 99); // 4. Error not decrease
  }
  else
  {
    TPDO_tarnsmit(0, current_angle); // 0.normal output
  }
}

void PID_init(void)
{
  TIM2->CCR2 = 0; // CCR_value bigger the rpm will highter 0~100
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_GPIO_WritePin(CW_CCW_control_GPIO_Port, CW_CCW_control_Pin, 1);
  osDelay(500);
  analyze_voltage();
  db_target_angle = Virtual_target_angle;
  PID(&TPID, &current_angle, &CCR_value, &db_target_angle, 1, 0.15, 0, _PID_P_ON_E, _PID_CD_DIRECT); // input output target
  PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&TPID, 100);
  PID_SetOutputLimits(&TPID, -70, 70);
}
void CW_CCW_deect(void)
{
  if (current_angle < db_target_angle)
  {
    HAL_GPIO_WritePin(CW_CCW_control_GPIO_Port, CW_CCW_control_Pin, 0);
  }
  else
  {
    HAL_GPIO_WritePin(CW_CCW_control_GPIO_Port, CW_CCW_control_Pin, 1);
  }
}
void PID_Error_handler(void)
{
  TIM2->CCR2 = 0;
  PID_safe_lock = true;
}
double abs(double a)
{
  if (a < 0)
  {
    return (a * -1);
  }
  return a;
}
void TPDO_tarnsmit(uint8_t status, int16_t angle)
{
  OD_PERSIST_COMM.x6000_current_angle = angle;
  OD_PERSIST_COMM.x6003_status = status;
  OD_set_u16(OD_find(OD, 0x6000), 0x000, OD_PERSIST_COMM.x6000_current_angle, false); // change_obj, index change , value,false
  OD_set_u8(OD_find(OD, 0x6003), 0x000, OD_PERSIST_COMM.x6003_status, false);
  CO_TPDOsendRequest(&canopenNodeSTM32->canOpenStack->TPDO[0]);
}
/* USER CODE END Application */
