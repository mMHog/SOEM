/*
 * @Author: your name
 * @Date: 2021-04-10 15:36:48
 * @LastEditTime: 2021-04-13 18:47:18
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /SOEM/test/linux/pid_interface/include/pid.h
 */
#ifndef _PID_H
#define _PID_H

#include <stdio.h>
#include <stdlib.h>
#include "./config.h"
struct _pid
{
	double SetSpeed;	//定义设定值
	double ActualSpeed; //定义实际值
	double err;			//定义偏差值
	double err_last;	//定义上一个偏差值
	double Kp, Ki, Kd;	//定义比例、积分、微分系数
	double voltage;		//定义电压值（控制执行器的变量）
	double integral;	//定义积分值
	double umax;
	double umin;
} pid[slave_num];

void PID_init(double init_x, int i, int mode)
{
	printf("PID_init begin \n");
	pid[i].SetSpeed = init_x;
	pid[i].ActualSpeed = init_x;
	pid[i].err = 0.0;
	pid[i].err_last = 0.0;
	pid[i].voltage = 0.0;
	pid[i].integral = 0.0;
	if (mode == 0)
	{
		pid[i].Kp = Kp[i];
		pid[i].Ki = Ki[i];
		pid[i].Kd = Kd[i];
	}
	else
	{
		pid[i].Kp = Kp_v[i];
		pid[i].Ki = Ki_v[i];
		pid[i].Kd = Kd_v[i];
	}
	printf("PID_init end \n");
}

double PID_realize(double speed, double current, int i, int mode)
{
	int index;
	pid[i].SetSpeed = speed;
	if (mode == 1) pid[i].ActualSpeed = current;
	pid[i].err = pid[i].SetSpeed - pid[i].ActualSpeed;
	if (abs(pid[i].err) > max_err)
	{
		if (pid[i].err>0) pid[i].err = max_err;
		else pid[i].err = -max_err;
	}
	if (abs(pid[i].err) > min_err_i) //积分分离过程
	{
		index = 0;
	}
	else
	{
		index = 1;
		pid[i].integral += pid[i].err;
	}
	if (abs(pid[i].err) < min_err && mode==0){
		return speed;
	}
	//	pid.voltage=pid.Kp*pid.err+index*pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
	pid[i].voltage = pid[i].Kp * pid[i].err + index * pid[i].Ki * pid[i].integral / 2 + pid[i].Kd * (pid[i].err - pid[i].err_last); //梯形积分
	pid[i].err_last = pid[i].err;
	if (mode == 0)
	{
		pid[i].ActualSpeed += pid[i].voltage * 1.0;
	}
	else
	{
		pid[i].ActualSpeed = pid[i].voltage * 1.0;
	}
	return pid[i].ActualSpeed;
}

#endif // !_PID_H
