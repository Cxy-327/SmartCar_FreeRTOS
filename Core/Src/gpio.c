/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */
extern uint8_t g_ucMode; 

#include "FreeRTOS.h"//包含相关头文件
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

extern osMessageQId myQueueModeHandle;  //声明一下 模式消息队列的句柄


/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OLED_SDA_Pin|SCL_6050_Pin|SDA_6050_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AIN1_Pin|BIN1_Pin|HC_SR04_Trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin */
  GPIO_InitStruct.Pin = HW_OUT_1_Pin|HC_SR04_Echo_Pin|HW_OUT_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin */
  GPIO_InitStruct.Pin = HW_OUT_3_Pin|HW_OUT_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = OLED_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OLED_SDA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin
                           PBPin */
  GPIO_InitStruct.Pin = AIN1_Pin|BIN1_Pin|HC_SR04_Trig_Pin|SCL_6050_Pin
                          |SDA_6050_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = KEY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OLED_SCL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	
//	BaseType_t xHigherPriorityTaskWoken = pdFALSE;  // 用于中断后的任务调度标志 pdFALSE表示不进行任务调度
//	uint8_t currentMode = 0;  // 当前模式
//	uint8_t newMode = 0;      // 新模式
//	if(GPIO_Pin == KEY1_Pin) //判断一下那个引脚触发中断
//	{
//		/*注意现在是在外部中断 要调用HAL_Delay，会使用Systick定时器中断 所以Systick优先级要高于外部中断*/
//		HAL_Delay(10);//延时消抖 
//		if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_SET)//判断KEY1引脚仍为高电平
//		{
//			
//			// 获取当前模式：从消息队列中读取模式（使用xQueueReceiveFromISR）
//			if (xQueueReceiveFromISR(myQueueModeHandle, &currentMode, &xHigherPriorityTaskWoken) != pdPASS)
//			{
//					currentMode = 0;  // 如果队列为空，则默认为模式0
//			}
//				
//			// 根据当前模式和按键动作更新模式
//			if (currentMode == 4) 
//			{
//					newMode = 1;  // 如果当前模式是6，按键按下后切换到1
//			}
//			else{
//				
//					newMode = currentMode + 1;  // 否则模式加1
//			}
//      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);  // 反转LED

//     // 将新模式发送到消息队列（使用xQueueSendFromISR）
//     xQueueSendFromISR(myQueueModeHandle, &newMode, &xHigherPriorityTaskWoken);  // 将新模式发送到队列
//			
//		}
//	}
//	if(GPIO_Pin == KEY2_Pin) //判断一下那个引脚触发中断
//	{
//		HAL_Delay(10);//延时消抖
//		if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)//判断KEY1引脚仍为低电平
//		{
//			
//			// 重置模式为0
//			newMode = 0;
//			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);  // 反转LED
//			// 将新模式发送到消息队列
//			//xQueueOverwriteFromISR 中断中写入消息队列数据-且强制覆盖原有数据
//			xQueueOverwriteFromISR(myQueueModeHandle, &newMode, &xHigherPriorityTaskWoken);  // 将新模式发送到队列
//			
//		}
//	}
//	
//}

/* USER CODE END 2 */
