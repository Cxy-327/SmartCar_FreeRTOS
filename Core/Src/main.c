/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "cmsis_os.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "motor.h"
#include "pid.h"
#include "MPU6050.h"
#include "Uart.h"

#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

extern tPid pidMotor1Speed;//声明电机1PID速度控制结构体类型变量
extern tPid pidMotor2Speed;
extern tPid pidFollow;    //定距离跟随PID
extern tPid pidMPU6050YawMovement;  //利用6050偏航角 进行姿态控制的PID
extern uint8_t Usart1_ReadBuf[255];	//串口1 缓冲数组
extern float p,i,d,a,b;//使用JSON时候使用的变量
extern uint8_t OledString[50];//OLED显示使用的字符串数组
extern float Mileage;//里程数

uint8_t g_ucUsart3ReceiveData;  //保存串口三接收的数据

extern char Usart3String[50];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  /* We should never get here as control is now taken by the scheduler */
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//开启定时器1 通道1 PWM输出
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);//开启定时器1 通道4 PWM输出
//	Motor_Set((int)0,(int)0);

  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);//开启定时器2
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);//开启定时器4
  HAL_TIM_Base_Start_IT(&htim2);				//开启定时器2中断
  HAL_TIM_Base_Start_IT(&htim4);        //开启定时器4中断

  HAL_TIM_Base_Start_IT(&htim1);        //开启定时器1中断

  __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);	//开启串口一接收中断
	
	HAL_UART_Receive_IT(&huart3,&g_ucUsart3ReceiveData,1);  //串口三接收数据
	motorPidSetSpeed(0.0,0.0);
//	printf("init\n");
//  osDelay(500);//延时0.5秒 6050上电稳定后初始化
//  MPU_Init(); //初始化MPU6050
//  while(MPU_Init()!=0);
//  while(mpu_dmp_init()!=0);
//  while(MPU_Init()!=0);//初始化MPU6050模块的MPU 注意初始化阶段不要移动小车
  for(uint8_t i = 0; i < 3 ; i++)
	{
//		printf ("going to MPU init\n");
		uint8_t mpu_init_result = MPU_Init(); // 保存函数返回值	//你妈的 这里卡住了[底层调用的hal_delay该为MPU IIC里的delay]
//		printf ("MPU init over");
		if(mpu_init_result == 0)//初始化成功
		{
			printf("In main.c:MPU_Init() was initialized successfully\n");//串口1 输出初始化成功 注意！ 这个要写再串口1初始化之后	
			sprintf((char*)Usart3String, "In main.c:MPU_Init() was initialized successfully\r\n");//蓝牙输出 初始化成功
		  HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小	
			break;
		}
		//上面的MPU_Init能够成功了	
		
		if(i == 3-1 )//说明初始化失败
		{
			printf("In main.c:MPU_Init() failed to initialize last return value: %d\n", mpu_init_result);//串口1 输出初始化失败 注意！ 这个要写再串口1初始化之后 初始化失败及最后一次返回值
			sprintf((char*)Usart3String, "In main.c:MPU_Init() failed to initialize last return value: %d\n", mpu_init_result);//蓝牙输出 初始化失败
		  HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小
		}
	}
	
//  while(mpu_dmp_init()!=0);//mpu6050,dmp初始化
  for(uint8_t i = 0; i < 3 ; i++)
	{
		//printf("go to mpu_dmp_init\n");
		uint8_t mpu_dmp_init_result = mpu_dmp_init();//这里卡住了
		//printf("mpu_dmp_init over\n");
		if(mpu_dmp_init_result == 0)//初始化成功
		{
			printf("In main.c:mpu_dmp_init() was initialized successfully\n");//串口1 输出初始化成功 注意！ 这个要写再串口1初始化之后	
			sprintf((char*)Usart3String, "In main.c:mpu_dmp_init() was initialized successfully\r\n");//蓝牙输出 初始化成功
		  HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小
			break;
		}
		if(i == 3-1 )//说明初始化失败
		{
			printf("In main.c: mpu_dmp_init() failed to initialize, last return value: %d\n", mpu_dmp_init_result);//串口1 输出初始化失败 注意！ 这个要写再串口1初始化之后
			sprintf((char*)Usart3String, "In main.c: mpu_dmp_init() failed to initialize, last return value: %d\n", mpu_dmp_init_result);//蓝牙输出 初始化成功
		  HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小
		}
	}

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

//***********MPU6050初始化***************//
//	osDelay(500);//延时0.5秒 6050上电稳定后初始化
//  MPU_Init(); //初始化MPU6050
//  while(MPU_Init()!=0);
//  while(mpu_dmp_init()!=0);
	
  /* USER CODE END 2 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  /* USER CODE BEGIN Callback 0 */
//	if(htim == &htim1)//htim1  500HZ 2ms中断一次
//	{
//		TimerCount++;//每次进入中断 、中断计数变量递增
//		if(TimerCount %5 == 0)//每10ms 执行一次
//		{
//			Encode1Count = -(short)__HAL_TIM_GET_COUNTER(&htim4);//获得当前编码器计数值并赋值 (short):是类型转化 -:是因为电机对侧安装
//			Encode2Count = (short)__HAL_TIM_GET_COUNTER(&htim2);
//			__HAL_TIM_SET_COUNTER(&htim4,0);//每次获得编码器计数值后都清零，这样每次计数值就是变化量
//			__HAL_TIM_SET_COUNTER(&htim2,0);
//		
//			/* 电机速度速度 = 编码器计数值*编码器读取频率/减速比/编码器线数/4倍频 */
//			Motor1Speed = (float)Encode1Count*100/9.6/11/4;
//			Motor2Speed = (float)Encode2Count*100/9.6/11/4;
//		}
//		if(TimerCount %10 == 0)//每20ms执行一次
//		{
//			/*里程 += 时间*电机速度*周长*/
//		   Mileage += 0.02*Motor1Speed*22;
//		   /*控制电机转速*/
//		   Motor_Set(PID_realize(&pidMotor1Speed,Motor1Speed),PID_realize(&pidMotor2Speed,Motor2Speed));
//		   TimerCount=0;
//		}
//	}

//  /* USER CODE END Callback 0 */
//  if (htim->Instance == TIM3) {
//    HAL_IncTick();
//  }
//  /* USER CODE BEGIN Callback 1 */

//  /* USER CODE END Callback 1 */
//}

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

#ifdef  USE_FULL_ASSERT
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
