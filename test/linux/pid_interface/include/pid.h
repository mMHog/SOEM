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
}pid;
 
void PID_init(){
	printf("PID_init begin \n");
	pid.SetSpeed=0.0;
	pid.ActualSpeed=0.0;
	pid.err=0.0;
	pid.err_last=0.0;
	pid.voltage=0.0;
	pid.integral=0.0;
	pid.Kp=0.02;
	pid.Ki=0.01; //注意，和上几次相比，这里加大了积分环节的值
	pid.Kd=0.02;
	pid.umax=400;
	pid.umin=-200;
	printf("PID_init end \n");
}
 
double PID_realize(double speed){
	int index;
	pid.SetSpeed=speed;
	pid.err=pid.SetSpeed-pid.ActualSpeed;
	if(pid.ActualSpeed>pid.umax) //灰色底色表示抗积分饱和的实现
	{
		if(abs(pid.err)>200) //蓝色标注为积分分离过程
		{
		index=0;
		}else{
			index=1;
			if(pid.err<0)
			{
			pid.integral+=pid.err;
			}
		}
	}else if(pid.ActualSpeed<pid.umin){
		if(abs(pid.err)>200) //积分分离过程
		{
			index=0;
		}else{
			index=1;
			if(pid.err>0)
			{
				pid.integral+=pid.err;
			}
		}
	}else{
			if(abs(pid.err)>200) //积分分离过程
			{
				index=0;
			}else{
				index=1;
				pid.integral+=pid.err;
			}
		}
//	pid.voltage=pid.Kp*pid.err+index*pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
pid.voltage=pid.Kp*pid.err+index*pid.Ki*pid.integral/2+pid.Kd*(pid.err-pid.err_last);//梯形积分
	pid.err_last=pid.err;
	pid.ActualSpeed=pid.voltage*1.0;
	return pid.ActualSpeed;
}