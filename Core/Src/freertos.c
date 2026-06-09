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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "kinematics.h"
#include "remote.h"
#include "OLED.h"
#include "bluetooth.h"
#include "key.h"
#include "camera.h"
#include "usart.h"
#include <stdio.h>

#include "tim.h"
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
/* Definitions for PID_Task */
osThreadId_t PID_TaskHandle;
const osThreadAttr_t PID_Task_attributes = {
  .name = "PID_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Bluetooth_Task */
osThreadId_t Bluetooth_TaskHandle;
const osThreadAttr_t Bluetooth_Task_attributes = {
  .name = "Bluetooth_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Camera_Task */
osThreadId_t Camera_TaskHandle;
const osThreadAttr_t Camera_Task_attributes = {
  .name = "Camera_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for KeyTask */
osThreadId_t KeyTaskHandle;
const osThreadAttr_t KeyTask_attributes = {
  .name = "KeyTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for UpComm_Task */
osThreadId_t UpComm_TaskHandle;
const osThreadAttr_t UpComm_Task_attributes = {
  .name = "UpComm_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for xCmdQueue */
osMessageQueueId_t xCmdQueueHandle;
const osMessageQueueAttr_t xCmdQueue_attributes = {
  .name = "xCmdQueue"
};
/* Definitions for xStatusQueue */
osMessageQueueId_t xStatusQueueHandle;
const osMessageQueueAttr_t xStatusQueue_attributes = {
  .name = "xStatusQueue"
};
/* Definitions for xCameraQueue */
osMessageQueueId_t xCameraQueueHandle;
const osMessageQueueAttr_t xCameraQueue_attributes = {
  .name = "xCameraQueue"
};
/* Definitions for xUpCommQueue */
osMessageQueueId_t xUpCommQueueHandle;
const osMessageQueueAttr_t xUpCommQueue_attributes = {
  .name = "xUpCommQueue"
};
/* Definitions for xCamMutex */
osMutexId_t xCamMutexHandle;
const osMutexAttr_t xCamMutex_attributes = {
  .name = "xCamMutex"
};
/* Definitions for xPidSemaphore */
osSemaphoreId_t xPidSemaphoreHandle;
const osSemaphoreAttr_t xPidSemaphore_attributes = {
  .name = "xPidSemaphore"
};
/* Definitions for xBtReadySem */
osSemaphoreId_t xBtReadySemHandle;
const osSemaphoreAttr_t xBtReadySem_attributes = {
  .name = "xBtReadySem"
};
/* Definitions for xUpCommSem */
osSemaphoreId_t xUpCommSemHandle;
const osSemaphoreAttr_t xUpCommSem_attributes = {
  .name = "xUpCommSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartPID_Task(void *argument);
void StartBluetooth_Task(void *argument);
void StartCamera_Task(void *argument);
void StartKeyTask(void *argument);
void StartUpComm_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of xCamMutex */
  xCamMutexHandle = osMutexNew(&xCamMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of xPidSemaphore */
  xPidSemaphoreHandle = osSemaphoreNew(1, 0, &xPidSemaphore_attributes);

  /* creation of xBtReadySem */
  xBtReadySemHandle = osSemaphoreNew(1, 0, &xBtReadySem_attributes);

  /* creation of xUpCommSem */
  xUpCommSemHandle = osSemaphoreNew(1, 0, &xUpCommSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of xCmdQueue */
  xCmdQueueHandle = osMessageQueueNew (1, sizeof(uint16_t), &xCmdQueue_attributes);

  /* creation of xStatusQueue */
  xStatusQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &xStatusQueue_attributes);

  /* creation of xCameraQueue */
  xCameraQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &xCameraQueue_attributes);

  /* creation of xUpCommQueue */
  xUpCommQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &xUpCommQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of PID_Task */
  PID_TaskHandle = osThreadNew(StartPID_Task, NULL, &PID_Task_attributes);

  /* creation of Bluetooth_Task */
  Bluetooth_TaskHandle = osThreadNew(StartBluetooth_Task, NULL, &Bluetooth_Task_attributes);

  /* creation of Camera_Task */
  Camera_TaskHandle = osThreadNew(StartCamera_Task, NULL, &Camera_Task_attributes);

  /* creation of KeyTask */
  KeyTaskHandle = osThreadNew(StartKeyTask, NULL, &KeyTask_attributes);

  /* creation of UpComm_Task */
  UpComm_TaskHandle = osThreadNew(StartUpComm_Task, NULL, &UpComm_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartPID_Task */
/**
  * @brief  Function implementing the PID_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartPID_Task */
void StartPID_Task(void *argument)
{
  /* USER CODE BEGIN StartPID_Task */
  /* 初始化电机和编码器 */
  Motor_Init();
  Encoder_Init();
  Kinematics_ResetPose(0, 0, 0);
  Remote_Init();

  /* 按键控制状态 */
  static uint8_t motor_toggle_state = 0;  /* 0=停止, 1=运行 */
  static uint8_t key0_last_state = 0;    /* KEY0 上次状态 */
  static uint8_t key1_last_state = 0;    /* KEY1 上次状态 */
  static uint8_t key_forward_active = 0;  /* 按键前进激活标志 */

  /* VOFA 发送缓冲 */
  static char vofa_buf[256];
  static char up_buf[16];
  static int32_t last_sent_num = -1;
  uint32_t vofa_last_send = 0;

  /* 初始化显示 */
  OLED_Init();
  OLED_Clear();
  OLED_ShowString(0, 0, "Chassis Ready", OLED_8X16);
  OLED_Update();

  /* 主循环 */
  for(;;)
  {
    /* 更新编码器 */
    Motor_EncoderUpdate();

    /* 获取遥控器数据 */
    RemoteData_t* remote = Remote_GetData();

    /* 遥控器数字同步到上层板：数字变化时通过 UART4 转发 */
    if (remote->receive_number != last_sent_num) {
      last_sent_num = remote->receive_number;
      int up_len = sprintf(up_buf, "%ld\r\n", (long)remote->receive_number);
      HAL_UART_Transmit(&huart4, (uint8_t*)up_buf, up_len, 50);
    }

    /* 检查按键状态 - 按一下toggle开/关 */
    uint8_t key0_pressed = KEY_IsPressed(KEY_ID_0);
    uint8_t key1_pressed = KEY_IsPressed(KEY_ID_1);

    /* KEY0: 前进/停止 toggle */
    if (key0_pressed && !key0_last_state) {
      if (motor_toggle_state == 0) {
        motor_toggle_state = 1;
        key_forward_active = 1;  /* 标记按键前进激活 */
        Kinematics_SetTarget(KEY_FORWARD_SPEED, 0, 0);  /* 前进用 vx */
      } else {
        motor_toggle_state = 0;
        key_forward_active = 0;  /* 取消前进标记 */
        Kinematics_SetTarget(0, 0, 0);
      }
    }
    key0_last_state = key0_pressed;

    /* KEY1: 后退/停止 toggle */
    if (key1_pressed && !key1_last_state) {
      if (motor_toggle_state == 0) {
        motor_toggle_state = 1;
        key_forward_active = 0;  /* 取消前进标记 */
        Kinematics_SetTarget(KEY_BACKWARD_SPEED, 0, 0);  /* 后退用 vx */
      } else {
        motor_toggle_state = 0;
        Kinematics_SetTarget(0, 0, 0);
      }
    }
    key1_last_state = key1_pressed;

    /* 速度控制逻辑：遥控器优先，但摇杆中间时不再保留残留目标 */
    if (!Remote_IsConnected()) {
      /* 遥控器未连接，使用按键设置；若未进入按键模式则清零 */
      if (motor_toggle_state == 0) {
        Kinematics_SetTarget(0, 0, 0);
      }
    } else if (remote->vx == 0 && remote->vy == 0 && remote->omega == 0) {
      /* 遥控器连接但摇杆回中：只有按键未接管时才清零，避免残留非零目标 */
      if (motor_toggle_state == 0) {
        Kinematics_SetTarget(0, 0, 0);
      }
    } else {
      /* 遥控器连接且摇杆有输入，使用遥控器数据 */
      motor_toggle_state = 0;
      key_forward_active = 0;
      Kinematics_SetTarget(remote->vx, remote->vy, remote->omega);
    }

    /* 获取轮子目标速度 */
    WheelSpeedMm_t* wheel = Kinematics_GetTargetWheelSpeed();

    /* 设置电机速度 */
    Motor_SetSpeed(wheel->left_front, wheel->right_front, wheel->left_back, wheel->right_back);

    /* PID控制 */
    Motor_ControlLoop();

    /* 更新里程计 */
    Kinematics_UpdatePosition(0.001f);  /* 1ms */

    /* 发送 VOFA+ 调试数据 (20ms间隔) */
    uint32_t now = osKernelGetTickCount();
    if (now - vofa_last_send >= 20) {
      vofa_last_send = now;
      ChassisMotorState_t* state = Motor_GetState();

      /* 读取原始编码器读书用于调试 */
      uint16_t enc1 = __HAL_TIM_GET_COUNTER(&htim2);
      uint16_t enc2 = __HAL_TIM_GET_COUNTER(&htim3);
      uint16_t enc3 = __HAL_TIM_GET_COUNTER(&htim4);
      uint16_t enc4 = __HAL_TIM_GET_COUNTER(&htim8);

      int len = sprintf(vofa_buf,
        "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
        (int)wheel->left_front,
        (int)wheel->right_front,
        (int)wheel->left_back,
        (int)wheel->right_back,
        (int)state->feedback_left_front,
        (int)state->feedback_right_front,
        (int)state->feedback_left_back,
        (int)state->feedback_right_back,
        (int)enc1,
        (int)enc2,
        (int)enc3,
        (int)enc4,
        (int)Motor_GetMode()
      );
      // 然后发送实际数据
      HAL_UART_Transmit(&huart1, (uint8_t*)vofa_buf, len, 100);
    }

    /* 显示状态 (降低刷新率) */
    static uint32_t last_display = 0;
    if (now - last_display > 100) {
      last_display = now;
      OLED_ClearArea(0, 0, 128, 64);
      ChassisMotorState_t* state = Motor_GetState();
      /* 显示遥控器目标速度和旋转速度 */
      OLED_Printf(0, 0, OLED_6X8, "Vx:%+d Vy:%+d W:%+d", (int)remote->vx, (int)remote->vy, (int)remote->omega);
      /* 显示四个电机的PWM值 */
      OLED_Printf(0, 8, OLED_6X8, "LF:%+4d RF:%+4d", state->PWM_left_front, state->PWM_right_front);
      OLED_Printf(0, 16, OLED_6X8, "LB:%+4d RB:%+4d", state->PWM_left_back, state->PWM_right_back);

      /* 显示相机调试信息 */
      uint32_t rx_count, raw_len;
      uint8_t valid, idle_flag;
      idle_flag = Camera_GetDebugInfo(&rx_count, &raw_len, &valid);

      OLED_Printf(0, 24, OLED_6X8, "RX:%d Idle:%d M:%d", rx_count, idle_flag, Motor_GetMode());

      if (raw_len > 0) {
        OLED_Printf(0, 32, OLED_6X8, "Raw:%s", Camera_GetRawData());
      } else {
        OLED_Printf(0, 32, OLED_6X8, "Raw:None");
      }

      /* 显示解析后的颜色 */
      if (valid) {
        OLED_Printf(0, 40, OLED_6X8, "C1:%s", Camera_GetColor1());
        OLED_Printf(64, 40, OLED_6X8, "C2:%s", Camera_GetColor2());
      } else {
        OLED_Printf(0, 40, OLED_6X8, "Color:Waiting");
      }

      OLED_Printf(0, 50, OLED_6X8, "Num:%d", remote->receive_number);

      OLED_Update();
    }

    /* LED控制逻辑 */
    /* 遥控器连接且有摇杆输入时，PE5(LED_2)置低电平 */
    if (Remote_IsConnected()) {
      /* 检查是否有非零速度指令（摇杆有输入） */
      if (remote->vx != 0 || remote->vy != 0 || remote->omega != 0) {
        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);  /* PE5置低 */
      } else {
        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);   /* PE5置高 */
      }
    } else {
      HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);      /* 遥控器未连接，PE5置高 */
    }

    /* KEY0按下前进时，PB5(LED_1)置低电平 */
    if (key_forward_active) {
      HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);  /* PB5置低 */
    } else {
      HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);   /* PB5置高 */
    }

    osDelay(10);
  }
  /* USER CODE END StartPID_Task */
}

/* USER CODE BEGIN Header_StartBluetooth_Task */
/**
* @brief Function implementing the Bluetooth_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBluetooth_Task */
void StartBluetooth_Task(void *argument)
{
  /* USER CODE BEGIN StartBluetooth_Task */
  /* 初始化蓝牙模块 */
  BT_Init();

  /* 主循环 - 等待蓝牙数据 */
  for(;;)
  {
    /* 等待新数据或超时 */
    if (BT_WaitData(100)) {
      /* 有新数据到达，已经在中断中处理 */
    }

    osDelay(1);
  }
  /* USER CODE END StartBluetooth_Task */
}

/* USER CODE BEGIN Header_StartCamera_Task */
/**
* @brief Function implementing the Camera_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCamera_Task */
void StartCamera_Task(void *argument)
{
  /* USER CODE BEGIN StartCamera_Task */
  /* 初始化相机数据接收 */
  Camera_Init();

  /* 主循环 */
  for(;;)
  {
    /* 处理接收到的数据 */
    Camera_Process();

    /* 发送颜色数据到蓝牙（遥控器） */
    static uint32_t last_send = 0;
    uint32_t now = osKernelGetTickCount();
    if (now - last_send >= 100) {  /* 100ms发送一次 */
      last_send = now;
      Camera_SendToBluetooth(&huart2);
    }

    osDelay(10);  /* 10ms周期 */
  }
  /* USER CODE END StartCamera_Task */
}

/* USER CODE BEGIN Header_StartKeyTask */
/**
* @brief Function implementing the KeyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKeyTask */
void StartKeyTask(void *argument)
{
  /* USER CODE BEGIN StartKeyTask */
  /* 初始化按键模块 */
  KEY_Init();

  /* 主循环 - 按键扫描 */
  for(;;)
  {
    KEY_Scan();
    osDelay(20);  /* 20ms 扫描周期 */
  }
  /* USER CODE END StartKeyTask */
}

/* USER CODE BEGIN Header_StartUpComm_Task */
/**
* @brief Function implementing the UpComm_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUpComm_Task */
void StartUpComm_Task(void *argument)
{
  /* USER CODE BEGIN StartUpComm_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUpComm_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

