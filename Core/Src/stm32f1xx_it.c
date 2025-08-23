/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "stdio.h"
#include "motor.h"
#include "super_PC.h"
#include "pid.h"

#include "MPU6050.h"
#include "mpuiic.h"
#include "Delay.h"

#include "FreeRTOS.h"//包含相关头文件
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
short Encode1Count = 0;
short Encode2Count = 0;
float Motor1Speed = 0.00;
float Motor2Speed = 0.00;
uint16_t TimerCount = 0;


uint8_t Usart1_ReadBuf[256];	//串口1 缓冲数组
uint8_t Usart1_ReadCount = 0;	//串口1 接收字节计数

extern tPid pidMotor1Speed;
extern tPid pidMotor2Speed;
extern uint8_t g_ucUsart3ReceiveData;  //保存串口三接收的数据
extern uint8_t g_ucUsart2ReceiveData;  //保存串口二接收的数据

extern float Mileage;//里程数 单位cm

//int16_t AX, AY, AZ;
//int16_t	GX, GY, GZ;			//定义用于存放各个数据的变量
//float dt = 0.01f;  // 10ms采样间隔
//float pitch = 0.0f,
//			roll = 0.0f,
//			yaw = 0.0f; // 俯仰角 横滚角 航向角
////uint8_t mpu_init_sign = 0;	//初次检测的GX, GY, GZ作为平衡值
////int16_t	GX_1, GY_1, GZ_1;
extern tPid pidMPU6050YawMovement;  //利用6050偏航角 进行姿态控制的PID参数

extern uint8_t g_ucMode;//当前模式变量

extern osMessageQId myQueueModeHandle;  //声明一下 模式消息队列的句柄
extern char Usart3String[50];


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim3;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY1_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
 if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_RXNE))//判断huart1 是否读到字节
  {
		if(Usart1_ReadCount >= 255) Usart1_ReadCount = 0;
		HAL_UART_Receive(&huart1,&Usart1_ReadBuf[Usart1_ReadCount++],1,1000);
  }
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY2_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/*******************
*  @brief  定时器回调函数
*  @param  
*  @return  
*
*******************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim1)//htim1  500HZ 2ms中断一次
	{
		TimerCount++;//每次进入中断 、中断计数变量递增
		if(TimerCount %5 == 0)//每10ms 执行一次
		{
			Encode1Count = -(short)__HAL_TIM_GET_COUNTER(&htim4);//获得当前编码器计数值并赋值 (short):是类型转化 -:是因为电机对侧安装
			Encode2Count = (short)__HAL_TIM_GET_COUNTER(&htim2);
			__HAL_TIM_SET_COUNTER(&htim4,0);//每次获得编码器计数值后都清零，这样每次计数值就是变化量
			__HAL_TIM_SET_COUNTER(&htim2,0);
		
			/* 电机速度速度 = 编码器计数值*编码器读取频率/减速比/编码器线数/4倍频 */
			Motor1Speed = (float)Encode1Count*100/9.6/11/4;
			Motor2Speed = (float)Encode2Count*100/9.6/11/4;
		
//			MPU6050_GetData(NULL ,NULL ,NULL , &GX, &GY, &GZ);		//获取MPU6050的数据
//			
//			pitch += GX * dt;																		//计算俯仰角、横滚角、航向角
//			roll -= GY * dt;
//			yaw += GZ * dt;
		}
		if(TimerCount %10 == 0)//每20ms执行一次
		{
			/*里程 += 时间*电机速度*周长*/
		   Mileage += 0.02*Motor1Speed*22;
		   /*控制电机转速*/
		   Motor_Set(PID_realize(&pidMotor1Speed,Motor1Speed),PID_realize(&pidMotor2Speed,Motor2Speed));
		   TimerCount=0;
		}
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

//	char buf[10]; 
//sprintf(buf ,"%d\n",GPIO_Pin); 
//HAL_UART_Transmit(&huart2, ( uint8_t *)"enter exit\n",10, 100); // 阻塞发送，确保发完
//HAL_UART_Transmit(&huart2, ( uint8_t *)buf, strlen(buf), 100);   // 按实际长度发
		static uint32_t lastTick = 0;
    uint32_t now = HAL_GetTick();

	BaseType_t xHigherPriorityTaskWoken = pdPASS;  // 用于中断后的任务调度标志 pdFALSE表示不进行任务调度
	uint8_t currentMode = 0;  // 当前模式
	uint8_t newMode = 0;      // 新模式
	if(GPIO_Pin == KEY1_Pin) //判断一下那个引脚触发中断
	{

			HAL_UART_Transmit_IT(&huart2,( uint8_t *)"pin1\n",strlen("pin1\n"));
			mpuiic_Delayus(100);
		if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_SET)//判断KEY1引脚仍为高电平
		{

			// 获取当前模式：从消息队列中读取模式（使用xQueueReceiveFromISR）
			if (xQueueReceiveFromISR(myQueueModeHandle, &currentMode, &xHigherPriorityTaskWoken) != pdPASS)
			{
		sprintf((char *)Usart3String,"recv_fail%d",(int)1);//显示超声波数据
		HAL_UART_Transmit_IT(&huart2,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String));//阻塞式发送通过串口三输出字符 strlen:计算字符串大小
					currentMode = 0;  // 如果队列为空，则默认为模式0
			}
				
			// 根据当前模式和按键动作更新模式
			if (currentMode == 5) 
			{
					newMode = 0;  // 如果当前模式是6，按键按下后切换到1
			}
			else{
				
					newMode = currentMode + 1;  // 否则模式加1
			}
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);  // 反转LED

     // 将新模式发送到消息队列（使用xQueueSendFromISR）
     xQueueSendFromISR(myQueueModeHandle, &newMode, &xHigherPriorityTaskWoken);  // 将新模式发送到队列
//			
		}
	}
	if(GPIO_Pin == KEY2_Pin) //判断一下那个引脚触发中断
	{
			mpuiic_Delayus(100);
		if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)//判断KEY1引脚仍为低电平
		{
			
			// 重置模式为0
			newMode = 0;
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);  // 反转LED
			// 将新模式发送到消息队列
			//xQueueOverwriteFromISR 中断中写入消息队列数据-且强制覆盖原有数据
			xQueueOverwriteFromISR(myQueueModeHandle, &newMode, &xHigherPriorityTaskWoken);  // 将新模式发送到队列
			
		}
	}
	
}

/* USER CODE END 1 */
