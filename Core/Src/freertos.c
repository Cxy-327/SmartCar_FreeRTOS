/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "motor.h"
#include "tim.h"
#include "super_PC.h"
#include "pid.h"
#include "cJSON.h"
#include "usart.h"
#include "adc.h"
#include "gpio.h"
#include "HC_SR04.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "MPU6050.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern short Encode1Count;//电机编码器计数值
extern short Encode2Count ;
extern float Motor1Speed;//电机速度 转/s
extern float Motor2Speed;


extern tPid pidMotor1Speed;	//声明电机PID速度控制结构体变量
extern tPid pidMotor2Speed;

uint8_t OledString[50];//OLED显示使用的字符串数组

extern uint8_t  Usart1_ReadBuf[255];//串口一 缓冲数据组
cJSON *cJsonData ,*cJsonVlaue;
float p=0,i=0,d=0,a=0;//使用CJSON时使用的变量

float Mileage;//里程数


extern tPid pidHW_Tracking;//红外循迹的PID
uint8_t g_ucaHW_Read[4] = {0};//保存红外对管电平的数组
//int8_t g_cThisState = 0;//这次状态
//int8_t g_cLastState = 0; //上次状态
float g_cThisState = 0;//这次状态
float g_cLastState = 0; //上次状态

float g_fHW_PID_Out;//红外对管PID计算输出速度
float g_fHW_PID_Out1;//电机1的最后循迹PID控制速度
float g_fHW_PID_Out2;//电机2的最后循迹PID控制速度

extern uint8_t g_ucUsart3ReceiveData;  //保存串口三接收的数据

char Usart3String[50];
float g_fHC_SR04_Read;
extern tPid pidFollow;    //定距离跟随PID结构体类型变量
float g_fFollow_PID_Out;

float pitch,roll,yaw; // 俯仰角 横滚角 航向角

extern tPid pidMPU6050YawMovement;
float  g_fMPU6050YawMovePidOut = 0.00f; //姿态PID运算输出
float  g_fMPU6050YawMovePidOut1 = 0.00f; //第一个电机控制输出
float  g_fMPU6050YawMovePidOut2 = 0.00f; //第一个电机控制输出

uint8_t g_ucMode = 0; //小车运动模式标志位

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
osThreadId StopTaskHandle;
osThreadId LedTaskHandle;
osThreadId OledTaskHandle;
osThreadId MultiModeTaskHandle;
osMessageQId myQueueModeHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartStopTask(void const * argument);
void StartLedTask(void const * argument);
void StartOledTask(void const * argument);
void StartMultiModeTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	OLED_Init();
	OLED_Clear();
	PID_init();		//初始化，设置目标值
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

  /* Create the queue(s) */
  /* definition and creation of myQueueMode */
  osMessageQDef(myQueueMode, 1, uint8_t);
  myQueueModeHandle = osMessageCreate(osMessageQ(myQueueMode), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of StopTask */
  osThreadDef(StopTask, StartStopTask, osPriorityHigh, 0, 128);
  StopTaskHandle = osThreadCreate(osThread(StopTask), NULL);

  /* definition and creation of LedTask */
//  osThreadDef(LedTask, StartLedTask, osPriorityNormal, 0, 128);
//  LedTaskHandle = osThreadCreate(osThread(LedTask), NULL);

  /* definition and creation of OledTask */
  osThreadDef(OledTask, StartOledTask, osPriorityNormal, 0, 156);
  OledTaskHandle = osThreadCreate(osThread(OledTask), NULL);

  /* definition and creation of MultiModeTask */
  osThreadDef(MultiModeTask, StartMultiModeTask, osPriorityAboveNormal, 0, 128);
  MultiModeTaskHandle = osThreadCreate(osThread(MultiModeTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartStopTask */
/**
  * @brief  Function implementing the StopTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartStopTask */
void StartStopTask(void const * argument)
{
  /* USER CODE BEGIN StartStopTask */
		uint8_t receivedMode;  // 定义临时变量 用来存储读取到的当前模式 
//	int Motor1Pwm = 0, Motor2Pwm = 0;
  /* Infinite loop */
  for(;;)
  {
		 // xQueuePeek从队列查看数据 但是不将数据移除。
		//不等待队列中的数据，队列为空 则赋值receivedMode 为零
		if (xQueuePeek(myQueueModeHandle, &receivedMode, 0) == errQUEUE_EMPTY)
		{
			receivedMode = 0; //返回errQUEUE_EMPTY 表示 消息队列为空 赋值receivedMode 为零
		}
		if(receivedMode == 0)//如果当前模式为0 则设置小车速度为零
		{
			motorPidSetSpeed(0,0);			//设置小车速度为0	
		}
		
		osDelay(10);
		
  }
  /* USER CODE END StartStopTask */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
* @brief Function implementing the LedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedTask */
void StartLedTask(void const * argument)
{
  /* USER CODE BEGIN StartLedTask */
	/* Infinite loop */
  for(;;)
  {
		
		/******************
*   电机实际速度(定时器回调函数中，完成计算)
******************/		
		

//		sprintf((char*)OledString,"s1:%.2f s2:%.2f",Motor1Speed,Motor2Speed);
//		OLED_ShowString(1,1,(char*)OledString);

//		sprintf((char*)OledString,"Mil:%.2f",Mileage);
//		OLED_ShowString(2,1,(char*)OledString);

//		sprintf((char*)OledString,"V:%.2f",adcGetBatteryVoltage());
//		OLED_ShowString(3,1,(char*)OledString);
//		
//		osDelay(10);
		
		
/******************
*   电机速度控制（简单判断->PID算法）		
******************/		
		
//		if(Motor1Speed>3.1) Motor1Pwm--;
//		if(Motor1Speed<2.9) Motor1Pwm++;
//		if(Motor2Speed>3.1) Motor2Pwm--;
//		if(Motor2Speed<2.9) Motor2Pwm++;

/******************
*   PID设速，上位机速度曲线显示、调试
******************/		
//		Motor_Set(PID_realize(&pidMotor1Speed ,Motor1Speed),Motor2Pwm);
//	
//		ANO_DT_Send_F2(Motor1Speed*100,3.0*100,Motor2Speed*100,3.0*100);
		
		
/******************
*   通过CJSON，帮助PID调试
******************/		
//	if(Usart_WaitReasFinish() == 0)//是否接收完毕
//	{
//		cJsonData  = cJSON_Parse((const char *)Usart1_ReadBuf);
//		if(cJSON_GetObjectItem(cJsonData,"p") !=NULL)
//		{
//			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"p");	
//		    p = cJsonVlaue->valuedouble;
//			pidMotor1Speed.Kp = p;
//		}
//		if(cJSON_GetObjectItem(cJsonData,"i") !=NULL)
//		{
//			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"i");	
//		    i = cJsonVlaue->valuedouble;
//			pidMotor1Speed.Ki = i;
//		}
//		if(cJSON_GetObjectItem(cJsonData,"d") !=NULL)
//		{
//			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"d");	
//		    d = cJsonVlaue->valuedouble;
//			pidMotor1Speed.Kd = d;
//		}
//		if(cJSON_GetObjectItem(cJsonData,"a") !=NULL)
//		{
//		
//			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"a");	
//		    a = cJsonVlaue->valuedouble;
//			pidMotor1Speed.target_val =a;
//		}
//		if(cJsonData != NULL){
//		  cJSON_Delete(cJsonData);//释放空间、但是不能删除cJsonVlaue不然会 出现异常错误
//		}
//		memset(Usart1_ReadBuf,0,255);//清空接收buf，注意这里不能使用strlen	
//	}

//	OLED_ShowNum (4,1,p,1);
//	OLED_ShowNum (4,3,i,1);
//	OLED_ShowNum (4,5,d,1);
//	OLED_ShowNum (4,7,a,1);

/******************
*   PID置速，控制小车运动
******************/		
//		motorPidSetSpeed(3.0,3.0);


/******************
*   红外对管循迹
******************/		

//if(READ_HW_OUT_1 == 0&&READ_HW_OUT_2 == 0&&READ_HW_OUT_3 == 0&&READ_HW_OUT_4 == 0 )
//	{
////		printf("应该前进\r\n");
//		motorPidSetSpeed(1,1);//前运动
//	}
//	if(READ_HW_OUT_1 == 0&&READ_HW_OUT_2 == 1&&READ_HW_OUT_3 == 0&&READ_HW_OUT_4 == 0 )
//	{
////		printf("应该右转\r\n");
//		motorPidSetSpeed(0.5,3);//右边运动
//	}
//	if(READ_HW_OUT_1 == 1&&READ_HW_OUT_2 == 0&&READ_HW_OUT_3 == 0&&READ_HW_OUT_4 == 0 )
//	{
////		printf("快速右转\r\n");
//		motorPidSetSpeed(0.2,5);//快速右转
//	}
//	if(READ_HW_OUT_1 == 0&&READ_HW_OUT_2 == 0&&READ_HW_OUT_3 == 1&&READ_HW_OUT_4 == 0 )
//	{
////		printf("应该左转\r\n");
//		motorPidSetSpeed(3,0.5);//左边运动
//	}
//	if(READ_HW_OUT_1 == 0&&READ_HW_OUT_2 == 0&&READ_HW_OUT_3 == 0&&READ_HW_OUT_4 == 1 )
//	{
////		printf("快速左转\r\n");
//		motorPidSetSpeed(5,0.2);//快速左转
//	}		
	

/******************
*   红外对管循迹+PID
******************/		

//	g_ucaHW_Read[0] = READ_HW_OUT_1;//读取红外对管状态、这样相比于写在if里面更高效
//	g_ucaHW_Read[1] = READ_HW_OUT_2;
//	g_ucaHW_Read[2] = READ_HW_OUT_3;
//	g_ucaHW_Read[3] = READ_HW_OUT_4;

//	if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
//	{
////		printf("应该前进\r\n");//注释掉更加高效，减少无必要程序执行
//		g_cThisState = 0;//前进
//	}
//	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )//使用else if更加合理高效
//	{
////		printf("应该右转\r\n");
//		g_cThisState = -0.6;//应该右转
//	}
//	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
//	{
////		printf("快速右转\r\n");
//		g_cThisState = -1.8;//快速右转
//	}
//	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0)
//	{
////		printf("快速右转\r\n");
//		g_cThisState = -2.5;//快速右转
//	}
//	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 0 )
//	{
////		printf("应该左转\r\n");
//		g_cThisState = 0.6;//应该左转	
//	}
//	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 1 )
//	{
////		printf("快速左转\r\n");
//		g_cThisState = 1.8;//快速左转
//	}
//	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 1)
//	{
////	    printf("快速左转\r\n");
//		g_cThisState = 2.5;//快速左转
//	}
//	g_fHW_PID_Out = PID_realize(&pidHW_Tracking,g_cThisState);//PID计算输出目标速度 这个速度，会和基础速度加减

//	g_fHW_PID_Out1 = 2 + g_fHW_PID_Out;//电机1速度=基础速度+循迹PID输出速度
//	g_fHW_PID_Out2 = 2 - g_fHW_PID_Out;//电机1速度=基础速度-循迹PID输出速度
//	if(g_fHW_PID_Out1 >5) g_fHW_PID_Out1 =5;//进行限幅 限幅速度在0-5之间
//	if(g_fHW_PID_Out1 <0) g_fHW_PID_Out1 =0;
//	if(g_fHW_PID_Out2 >5) g_fHW_PID_Out2 =5;
//	if(g_fHW_PID_Out2 <0) g_fHW_PID_Out2 =0;
//	
//	if(g_cThisState != g_cLastState)//如何这次状态不等于上次状态、就进行改变目标速度和控制电机、在定时器中依旧定时控制电机
//	{
//		motorPidSetSpeed(g_fHW_PID_Out1,g_fHW_PID_Out2);//通过计算的速度控制电机
//	}
//
//	g_cLastState = g_cThisState;//保存上次红外对管状态
	
//		osDelay(10);


/******************
*   通过串口三接收的数据改变小车运动状态
*
*		电脑通过串口助手进行控制 ->  先设置好蓝牙连接，手机通过蓝牙控制
******************/		
//		sprintf((char*)OledString,"s1:%.2f s2:%.2f",Motor1Speed,Motor2Speed);
//		OLED_ShowString(1,1,(char*)OledString);		//速度显示
		
		
		
/******************
*   超声波模块测距 -> 超声波避障 
******************/		
//****************测距，发送至蓝牙*****************//
//		sprintf((char *)Usart3String,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());//显示超声波数据
//		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//通过串口三输出字符 strlen:计算字符串大小	
//		sprintf((char *)OledString ,"HC_SR04:%.2f",HC_SR04_Read());//显示超声波数据
//		OLED_ShowString(3,1,(char*)Usart3String);		//速度显示

		
//**************避障功能********************//
//避障逻辑
//	if(HC_SR04_Read() > 25)//前方无障碍物
//	{
//		motorPidSetSpeed(1,1);//前运动
//		HAL_Delay(100);
//	}
//	else{	//前方有障碍物
//		motorPidSetSpeed(-1,1);//右边运动 原地	
//		HAL_Delay(500);
//		if(HC_SR04_Read() > 25)//右边无障碍物
//		{
//			motorPidSetSpeed(1,1);//前运动
//			HAL_Delay(100);
//		}
//		else{//右边有障碍物
//			motorPidSetSpeed(1,-1);//左边运动 原地
//			HAL_Delay(1000);
//			if(HC_SR04_Read() >25)//左边无障碍物
//			{
//				 motorPidSetSpeed(1,1);//前运动
//				HAL_Delay(100);
//			}
//			else{
//				motorPidSetSpeed(-1,-1);//后运动
//				HAL_Delay(1000);
//				motorPidSetSpeed(-1,1);//右边运动
//				HAL_Delay(50);
//			}
//		}
//	}
//		
//		osDelay(5);
		

/******************
*   超声波PID跟随 (可设置 开始跟随的距离、目标距离 )
******************/		

//    g_fHC_SR04_Read=HC_SR04_Read();//读取前方障碍物距离
//	if(g_fHC_SR04_Read < 20){  //如果前 X cm 有东西就启动跟随
//		g_fFollow_PID_Out = PID_realize(&pidFollow,g_fHC_SR04_Read);//PID计算输出目标速度 这个速度，会和基础速度加减
//		if(g_fFollow_PID_Out > 3) g_fFollow_PID_Out = 3;//对输出速度限幅
//		if(g_fFollow_PID_Out < -3) g_fFollow_PID_Out = -3;
//		motorPidSetSpeed(g_fFollow_PID_Out,g_fFollow_PID_Out);//速度作用与电机上
//	}
//	else motorPidSetSpeed(0,0);//如果前面 X cm 没有东西就停止
//	HAL_Delay(10);//读取超声波传感器不能过快



/******************
*   MPU6050  读取姿态角 -> 用于直线、直角转弯控制
******************/		
////*****************姿态角读取**************************

//   	sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);//显示6050数据
//		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//通过串口三输出字符 strlen:计算字符串大小	
//////   printf ("%s\n",Usart3String);
//		sprintf((char *)OledString,"p:%.1f  r:%.1f ",pitch,roll);//显示6050数据 俯仰角 横滚角
//		OLED_ShowString (1,1,OledString);
//		
//		sprintf((char *)OledString,"y:%.1f  ",yaw);//显示6050数据  航向角
//		OLED_ShowString (2,1,OledString);

//		for(uint8_t i = 0; i < 20 ; i++)
//		{
//			if(mpu_dmp_get_data(&pitch,&roll,&yaw) == 0)//获得MPU6050数据成功
//			{
//				strcpy((char *)Usart3String,"mpu_dmp_get_data() Data for success\n");
//			HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//通过串口三输出字符 strlen:计算字符串大小	
//				printf("mpu_dmp_get_data() Data for success\n");//串口1 输出数据获取成功 注意！ 这个要写再串口1初始化之后		
//			  break;
//			}
//						
//			if(i == 20-1 )//i已经等于(20-1)次 还没有退出循环说明 说明输出读取失败
//			{
//				strcpy((char *)Usart3String,"mpu_dmp_get_data() Data acquisition failure\n\n");
//			HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//通过串口三输出字符 strlen:计算字符串大小	
//				
//				printf("mpu_dmp_get_data()  Data acquisition failure\n");//串口1 输出数据获取失败 注意！ 这个要写再串口1初始化之后	
//			}
//		}
		
		
		
//////////////		sprintf((char *)OledString,"p:%.2f r:%.2f \r\n",pitch,roll);//显示6050数据 俯仰角 横滚角
//////////////		OLED_ShowString (1,1,OledString);
//////////////		
//////////////		sprintf((char *)OledString,"y:%.2f  \r\n",yaw);//显示6050数据  航向角
//////////////		OLED_ShowString (2,1,OledString);

////////////mpu_dmp_get_data(&pitch,&roll,&yaw);//返回值:0,DMP成功解出欧拉角
////////////    while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //这个可以解决经常读不出数据的问题


////////////		MPU6050_UpdateAttitude(dt);  // 更新姿态角
////////////   	sprintf((char *)Usart3String,"pitch:%.1f roll:%.1f yaw:%.1f\r\n",pitch,roll,yaw);//显示6050数据
////////////		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//通过串口三输出字符 strlen:计算字符串大小	
////////////   
////////////		sprintf((char *)OledString,"p:%.1f  r:%.1f ",pitch,roll);//显示6050数据 俯仰角 横滚角
////////////		OLED_ShowString (1,1,OledString);
////////////		
////////////		sprintf((char *)OledString,"y:%.1f  ",yaw);//显示6050数据  航向角
////////////		OLED_ShowString (2,1,OledString);

		g_fMPU6050YawMovePidOut = PID_realize(&pidMPU6050YawMovement,yaw);//PID计算输出目标速度 这个速度，会和基础速度加减

			g_fMPU6050YawMovePidOut1 = 1.5 + g_fMPU6050YawMovePidOut;//基础速度加减PID输出速度
			g_fMPU6050YawMovePidOut2 = 1.5 - g_fMPU6050YawMovePidOut;
			if(g_fMPU6050YawMovePidOut1 >3.5) g_fMPU6050YawMovePidOut1 =3.5;//进行限幅
			if(g_fMPU6050YawMovePidOut1 <0) g_fMPU6050YawMovePidOut1 =0;
			if(g_fMPU6050YawMovePidOut2 >3.5) g_fMPU6050YawMovePidOut2 =3.5;
			if(g_fMPU6050YawMovePidOut2 <0) g_fMPU6050YawMovePidOut2 =0;
			motorPidSetSpeed(g_fMPU6050YawMovePidOut1,g_fMPU6050YawMovePidOut2);

		osDelay(1);

  }
  /* USER CODE END StartLedTask */
}

/* USER CODE BEGIN Header_StartOledTask */
/**
* @brief Function implementing the OledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOledTask */
void StartOledTask(void const * argument)
{
  /* USER CODE BEGIN StartOledTask */
		uint8_t readMode;  // 定义临时变量 用来存储读取到的当前模式
	/* Infinite loop */
  for(;;)
  {
		if (xQueuePeek(myQueueModeHandle, &readMode, 0) == errQUEUE_EMPTY)
		{
			readMode = 0;   //如果返回errQUEUE_EMPTY 就是消息队列为空就赋值 readMode = 0
		}


		sprintf((char *)OledString,"Mode:%d",readMode);//显示g_ucMode 当前模式
		OLED_ShowString (1,1,(char*)OledString);
		
		sprintf((char *)Usart3String,"Mode:%d\n",readMode);//蓝牙APP显示
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小

		//OLED显示
//		sprintf((char*)OledString,"v1:%.2f v2:%.2f",Motor1Speed,Motor2Speed);//显示速度
//		OLED_ShowString (2,1,(char*)OledString);
		
		sprintf((char*)OledString, "L:%.2f", Mileage);//显示里程
		OLED_ShowString (1,8,(char*)OledString);
		
		sprintf((char*)OledString, "U:%.2fV", adcGetBatteryVoltage());//显示电池电压
		OLED_ShowString (2,9,(char*)OledString);
		
		sprintf((char *)OledString,"HC:%.2f",HC_SR04_Read());//显示超声波数据
		OLED_ShowString (2,1,(char*)OledString);
		
		sprintf((char *)OledString,"p:%.2f r:%.2f",pitch,roll);//显示6050数据 俯仰角 横滚角
		OLED_ShowString (3,1,OledString);
	
		sprintf((char *)OledString,"y:%.2f",yaw);//显示6050数据  航向角
		OLED_ShowString (4,1,OledString);
		
			
		
	//蓝牙APP显示
		sprintf((char*)Usart3String, "v1:%.2fv2:%.2f\n", Motor1Speed,Motor2Speed);//显示速度
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小
		//阻塞方式发送可以保证数据发送完毕，中断发送不一定可以保证数据已经发送完毕才启动下一次发送
		sprintf((char*)Usart3String, "Mileage:%.2f\n", Mileage);//显示里程
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小
		
		sprintf((char*)Usart3String, "U:%.2fV\n", adcGetBatteryVoltage());//显示电池电压
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小
		
		sprintf((char *)Usart3String,"HC_SR04:%.2fcm\n",HC_SR04_Read());//显示超声波数据
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小
		
		sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\n",pitch,roll,yaw);//显示6050数据 俯仰角 横滚角 航向角
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//通过串口三输出字符 strlen:计算字符串大小	


		for(uint8_t i = 0; i < 20 ; i++)
		{
			if(mpu_dmp_get_data(&pitch,&roll,&yaw) == 0)//获得MPU6050数据成功
			{
				strcpy((char *)Usart3String,"mpu_dmp_get_data() Data for success\n");
			HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),100);//通过串口三输出字符 strlen:计算字符串大小	
				printf("mpu_dmp_get_data() Data for success\n");//串口1 输出数据获取成功 注意！ 这个要写再串口1初始化之后		
			  break;
			}
						
			if(i == 20-1 )//i已经等于(20-1)次 还没有退出循环说明 说明输出读取失败
			{
				strcpy((char *)Usart3String,"mpu_dmp_get_data() Data acquisition failure\n\n");
			HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),100);//通过串口三输出字符 strlen:计算字符串大小	
				
				printf("mpu_dmp_get_data()  Data acquisition failure\n");//串口1 输出数据获取失败 注意！ 这个要写再串口1初始化之后	
			}
		}

    osDelay(10);
  }
  /* USER CODE END StartOledTask */
}

/* USER CODE BEGIN Header_StartMultiModeTask */
/**
* @brief Function implementing the MultiModeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMultiModeTask */
void StartMultiModeTask(void const * argument)
{
  /* USER CODE BEGIN StartMultiModeTask */
	uint8_t readMode;  // 定义临时变量 用来存储读取到的当前模式	
  /* Infinite loop */
  for(;;)
  {
		if (xQueuePeek(myQueueModeHandle, &readMode, 0) == errQUEUE_EMPTY)
		{
			readMode = 0;   //如果返回errQUEUE_EMPTY 就是消息队列为空就赋值 readMode = 0
		}

		if(readMode == 1)
		{
			//PID红外循迹
			g_ucaHW_Read[0] = READ_HW_OUT_1;//读取红外对管状态、这样相比于写在if里面更高效
			g_ucaHW_Read[1] = READ_HW_OUT_2;
			g_ucaHW_Read[2] = READ_HW_OUT_3;
			g_ucaHW_Read[3] = READ_HW_OUT_4;

			if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
			{
		//		printf("应该前进\r\n");//注释掉更加高效，减少无必要程序执行
				g_cThisState = 0;//前进
			}
			else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )//使用else if更加合理高效
			{
		//		printf("应该右转\r\n");
				g_cThisState = -0.6;//应该右转
			}
			else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
			{
		//		printf("快速右转\r\n");
				g_cThisState = -1.8;//快速右转
			}
			else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0)
			{
		//		printf("快速右转\r\n");
				g_cThisState = -2.5;//快速右转
			}
			else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 0 )
			{
		//		printf("应该左转\r\n");
				g_cThisState = 0.6;//应该左转	
			}
			else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 1 )
			{
		//		printf("快速左转\r\n");
				g_cThisState = 1.8;//快速左转
			}
			else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 1)
			{
		//	    printf("快速左转\r\n");
				g_cThisState = 2.5;//快速左转
			}
			g_fHW_PID_Out = PID_realize(&pidHW_Tracking,g_cThisState);//PID计算输出目标速度 这个速度，会和基础速度加减

			g_fHW_PID_Out1 = 2 + g_fHW_PID_Out;//电机1速度=基础速度+循迹PID输出速度
			g_fHW_PID_Out2 = 2 - g_fHW_PID_Out;//电机1速度=基础速度-循迹PID输出速度
			if(g_fHW_PID_Out1 >5) g_fHW_PID_Out1 =5;//进行限幅 限幅速度在0-5之间
			if(g_fHW_PID_Out1 <0) g_fHW_PID_Out1 =0;
			if(g_fHW_PID_Out2 >5) g_fHW_PID_Out2 =5;
			if(g_fHW_PID_Out2 <0) g_fHW_PID_Out2 =0;
			
			if(g_cThisState != g_cLastState)//如何这次状态不等于上次状态、就进行改变目标速度和控制电机、在定时器中依旧定时控制电机
			{
				motorPidSetSpeed(g_fHW_PID_Out1,g_fHW_PID_Out2);//通过计算的速度控制电机
			}

			g_cLastState = g_cThisState;//保存上次红外对管状态
			
		}

		else if(readMode == 2)
		{
			
			//***************遥控模式***********************//
			//遥控模式的控制在串口三的中断里面
		}		

		else if(readMode == 3)
		{
			//超声波避障
			if(HC_SR04_Read() > 25)//前方无障碍物
			{
				motorPidSetSpeed(1,1);//前运动
				osDelay(100);
			}
			else{	//前方有障碍物
				motorPidSetSpeed(-1,1);//右边运动 原地	
				osDelay(500);
				if(HC_SR04_Read() > 25)//右边无障碍物
				{
					motorPidSetSpeed(1,1);//前运动
					osDelay(100);
				}
				else{//右边有障碍物
					motorPidSetSpeed(1,-1);//左边运动 原地
					osDelay(1000);
					if(HC_SR04_Read() >25)//左边无障碍物
					{
						 motorPidSetSpeed(1,1);//前运动
						osDelay(100);
					}
					else{
						motorPidSetSpeed(-1,-1);//后运动
						osDelay(1000);
						motorPidSetSpeed(-1,1);//右边运动
						osDelay(50);
					}
				}
			}
		}
		else if(readMode == 4)
		{
			//PID超声波跟随
			g_fHC_SR04_Read=HC_SR04_Read();//读取前方障碍物距离
			if(g_fHC_SR04_Read < 20)
			{  //如果前 X cm 有东西就启动跟随
			g_fFollow_PID_Out = PID_realize(&pidFollow,g_fHC_SR04_Read);//PID计算输出目标速度 这个速度，会和基础速度加减
			if(g_fFollow_PID_Out > 3) g_fFollow_PID_Out = 3;//对输出速度限幅
			if(g_fFollow_PID_Out < -3) g_fFollow_PID_Out = -3;
			motorPidSetSpeed(g_fFollow_PID_Out,g_fFollow_PID_Out);//速度作用与电机上
			}
			else motorPidSetSpeed(0,0);//如果前面 X cm 没有东西就停止
			osDelay(10);//读取超声波传感器不能过快
		}

		else if(readMode == 5)		
		{
			//MPU6050 姿态角数据，PID转向控制
			//读取数据
				for(uint8_t i = 0; i < 20 ; i++)
			{
				if(mpu_dmp_get_data(&pitch,&roll,&yaw) == 0)//获得MPU6050数据成功
				{
					printf("mpu_dmp_get_data() Data for success\n");//串口1 输出数据获取成功 注意！ 这个要写再串口1初始化之后		
					break;
				}
							
				if(i == 20-1 )//i已经等于(20-1)次 还没有退出循环说明 说明输出读取失败
				{
					
					printf("mpu_dmp_get_data()  Data acquisition failure\n");//串口1 输出数据获取失败 注意！ 这个要写再串口1初始化之后	
				}
			}
			//姿态角数据发送到蓝牙
			sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\n",pitch,roll,yaw);//显示6050数据 俯仰角 横滚角 航向角
			HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//通过串口三输出字符 strlen:计算字符串大小	

			//PID转向控制
			g_fMPU6050YawMovePidOut = PID_realize(&pidMPU6050YawMovement,yaw);//PID计算输出目标速度 这个速度，会和基础速度加减

			g_fMPU6050YawMovePidOut1 = 1.5 + g_fMPU6050YawMovePidOut;//基础速度加减PID输出速度
			g_fMPU6050YawMovePidOut2 = 1.5 - g_fMPU6050YawMovePidOut;
			if(g_fMPU6050YawMovePidOut1 >3.5) g_fMPU6050YawMovePidOut1 =3.5;//进行限幅
			if(g_fMPU6050YawMovePidOut1 <0) g_fMPU6050YawMovePidOut1 =0;
			if(g_fMPU6050YawMovePidOut2 >3.5) g_fMPU6050YawMovePidOut2 =3.5;
			if(g_fMPU6050YawMovePidOut2 <0) g_fMPU6050YawMovePidOut2 =0;
			motorPidSetSpeed(g_fMPU6050YawMovePidOut1,g_fMPU6050YawMovePidOut2);
		}
    osDelay(1);
  }
  /* USER CODE END StartMultiModeTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

