/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ChassisControl.h"
#include "comm_manager.h"
#include "statemachine.h"
#include "gimbal.h"
#include "config.h"
#include "fdcan.h"
#include "bsp_zdt.h"
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static volatile uint32_t g_zdt_init_fail_mask = 0;

static HAL_StatusTypeDef ZDT_EnableBeforeTasks(uint8_t addr)
{
  for (uint8_t retry = 0; retry < 3; retry++)
  {
    if (ZDT_Enable(addr) == HAL_OK)
    {
      HAL_Delay(10);
      return HAL_OK;
    }
    HAL_Delay(2);
  }

  g_zdt_init_fail_mask |= (1UL << addr);
  return HAL_ERROR;
}

static void ZDT_PreTaskInit(void)
{
  ZDT_Init(&hfdcan1);

#if !CONFIG_STEPPER_USE_UART1
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    g_zdt_init_fail_mask |= (1UL << 31);
    return;
  }
#endif

#if CONFIG_USE_CHASSIS
  for (uint8_t addr = 1; addr <= 4; addr++)
    ZDT_EnableBeforeTasks(addr);
#endif

#if CONFIG_USE_GIMBAL
  ZDT_EnableBeforeTasks(5);
  ZDT_EnableBeforeTasks(6);
#endif
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* 在创建任何业务任务前启动总线并依次使能全部 ZDT 电机。 */
  ZDT_PreTaskInit();

#if CONFIG_USE_GIMBAL
  /*
   * 必须在创建任何 FreeRTOS 任务之前执行：
   * 调度器启动前创建任务会屏蔽低优先级中断，此后 HAL_Delay 无法依靠
   * TIM8（优先级 15）推进时基。
   */
  Gimbal_GripperInit();
  Gimbal_Gripper(CONFIG_GRIPPER_OPEN_PULSE_US);
#if CONFIG_GRIPPER_STARTUP_TEST
HAL_Delay(CONFIG_GRIPPER_STARTUP_TEST_HOLD_MS);
Gimbal_Gripper(CONFIG_GRIPPER_CLOSE_PULSE_US);
HAL_Delay(CONFIG_GRIPPER_STARTUP_TEST_HOLD_MS);
#endif
#endif
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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
#if CONFIG_USE_CHASSIS
  Chassis_TaskInit();
#endif
  Comm_InitTask();
#if CONFIG_USE_GIMBAL
  Gimbal_InitTask();                  /* 启动云台电机 J4310 角度闭环 (1kHz PID) */
#endif
  SM_Init();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

