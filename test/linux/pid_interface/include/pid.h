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

#include<stdio.h>
#include<stdlib.h>
struct _pid{
	double SetSpeed; //定义设定值
	double ActualSpeed; //定义实际值
	double err; //定义偏差值
	double err_last; //定义上一个偏差值
	double Kp,Ki,Kd; //定义比例、积分、微分系数
	double voltage; //定义电压值（控制执行器的变量）
	double integral; //定义积分值
	double umax;
	double umin;
}pid[18];
 
void PID_init(int i){
	printf("PID_init begin \n");
	pid[i].SetSpeed=0.0;
	pid[i].ActualSpeed=0.0;
	pid[i].err=0.0;
	pid[i].err_last=0.0;
	pid[i].voltage=0.0;
	pid[i].integral=0.0;
	pid[i].Kp=0.02;
	pid[i].Ki=0.0; //注意，和上几次相比，这里加大了积分环节的值
	pid[i].Kd=0.002;
	pid[i].umax=400;
	pid[i].umin=-200;
	printf("PID_init end \n");
}
 
double PID_realize(double speed,int i){
	int index;
	pid[i].SetSpeed=speed;
	pid[i].err=pid[i].SetSpeed-pid[i].ActualSpeed;
	if(pid[i].ActualSpeed>pid[i].umax) //灰色底色表示抗积分饱和的实现
	{
		if(abs(pid[i].err)>200) //蓝色标注为积分分离过程
		{
		index=0;
		}else{
			index=1;
			if(pid[i].err<0)
			{
			pid[i].integral+=pid[i].err;
			}
		}
	}else if(pid[i].ActualSpeed<pid[i].umin){
		if(abs(pid[i].err)>200) //积分分离过程
		{
			index=0;
		}else{
			index=1;
			if(pid[i].err>0)
			{
				pid[i].integral+=pid[i].err;
			}
		}
	}else{
			if(abs(pid[i].err)>200) //积分分离过程
			{
				index=0;
			}else{
				index=1;
				pid[i].integral+=pid[i].err;
			}
		}
//	pid.voltage=pid.Kp*pid.err+index*pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
pid[i].voltage=pid[i].Kp*pid[i].err+index*pid[i].Ki*pid[i].integral/2+pid[i].Kd*(pid[i].err-pid[i].err_last);//梯形积分
	pid[i].err_last=pid[i].err;
	pid[i].ActualSpeed=pid[i].voltage*1.0;
	return pid[i].ActualSpeed;
}

#endif // !_PID_H