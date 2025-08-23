#include "pid.h"
//定义一个结构体类型变量
tPid pidMotor1Speed;//电机1速度PID闭环参数
tPid pidMotor2Speed;//电机2速度PID闭环参数
tPid pidHW_Tracking;//红外循迹的PID结构体类型变量

tPid pidOpenmv_Tracking;//openmv摄像头的PID结构体类型变量

tPid pidFollow;    //定距离跟随PID结构体类型变量
tPid pidMPU6050YawMovement;  //利用6050偏航角 进行姿态控制的PID

//给结构体类型变量赋初值
void PID_init()
{
	pidMotor1Speed.actual_val=0.0;
	pidMotor1Speed.target_val=0.00;
	pidMotor1Speed.err=0.0;
	pidMotor1Speed.err_last=0.0;
	pidMotor1Speed.err_sum=0.0;
	pidMotor1Speed.Kp=10;
	pidMotor1Speed.Ki=3;
	pidMotor1Speed.Kd=0;
	
	pidMotor2Speed.actual_val=0.0;
	pidMotor2Speed.target_val=0.00;
	pidMotor2Speed.err=0.0;
	pidMotor2Speed.err_last=0.0;
	pidMotor2Speed.err_sum=0.0;
	pidMotor2Speed.Kp=10;
	pidMotor2Speed.Ki=3;
	pidMotor2Speed.Kd=0;
	
	pidHW_Tracking.actual_val=0.0;
	pidHW_Tracking.target_val=0.00;//红外循迹PID 的目标值为0
	pidHW_Tracking.err=0.0;
	pidHW_Tracking.err_last=0.0;
	pidHW_Tracking.err_sum=0.0;
	pidHW_Tracking.Kp=-2;
	pidHW_Tracking.Ki=0;
	pidHW_Tracking.Kd=1;
//	
//	pidOpenmv_Tracking.actual_val=0.0;
//	pidOpenmv_Tracking.target_val=0.00;//摄像头PID 的目标值为0
//	pidOpenmv_Tracking.err=0.0;
//	pidOpenmv_Tracking.err_last=0.0;
//	pidOpenmv_Tracking.err_sum=0.0;
//	pidOpenmv_Tracking.Kp=-0.4;//摄像头循迹PID的K和d先调整的小一些
//	pidOpenmv_Tracking.Ki=0;
//	pidOpenmv_Tracking.Kd=0.10;
//	
//	
//	
	pidFollow.actual_val=0.0;
	pidFollow.target_val=10;//定距离跟随 目标距离
	pidFollow.err=0.0;
	pidFollow.err_last=0.0;
	pidFollow.err_sum=0.0;
	pidFollow.Kp=-0.5;//定距离跟随的Kp大小通过估算PID输入输出数据，确定大概大小，然后在调试
	pidFollow.Ki=-0.001;//Ki小一些
	pidFollow.Kd=0;
	
	pidMPU6050YawMovement.actual_val=0.0;
	pidMPU6050YawMovement.target_val=0.00;//设定姿态目标值
	pidMPU6050YawMovement.err=0.0;
	pidMPU6050YawMovement.err_last=0.0;
	pidMPU6050YawMovement.err_sum=0.0;
	pidMPU6050YawMovement.Kp=0.02;//6050航向角PID运动控制 
	pidMPU6050YawMovement.Ki=0;
	pidMPU6050YawMovement.Kd=0.1;
	
}
//比例p调节控制函数
float P_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//传递真实值
	pid->err = pid->target_val - pid->actual_val;//当前误差=目标值-真实值
	//比例控制调节   输出=Kp*当前误差
	pid->actual_val = pid->Kp*pid->err;
	return pid->actual_val;
}
//比例P 积分I 控制函数
float PI_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//传递真实值
	pid->err = pid->target_val - pid->actual_val;//当前误差=目标值-真实值
	pid->err_sum += pid->err;//误差累计值 = 当前误差累计和
	//使用PI控制 输出=Kp*当前误差+Ki*误差累计值
	pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->err_sum;
	
	return pid->actual_val;
}
// PID控制函数
float PID_realize(tPid * pid,float actual_val)
{
	pid->actual_val = actual_val;//传递真实值
	pid->err = pid->target_val - pid->actual_val;////当前误差=目标值-真实值
	pid->err_sum += pid->err;//误差累计值 = 当前误差累计和
	//使用PID控制 输出 = Kp*当前误差  +  Ki*误差累计值 + Kd*(当前误差-上次误差)
	pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->err_sum + pid->Kd*(pid->err - pid->err_last);
	//保存上次误差: 这次误差赋值给上次误差
	pid->err_last = pid->err;
	
	return pid->actual_val;
}

