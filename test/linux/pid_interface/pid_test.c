/*
 * @Author: your name
 * @Date: 2021-04-10 14:58:23
 * @LastEditTime: 2021-04-15 15:17:59
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /SOEM/test/linux/pid_interface/pid_test.c
 */
#include "pid.h"
#include "inter.h"
 
int main(){
	// PID_init(0);
	// int count=0;
	// while(count<500)
	// {
	// double speed=PID_realize(1.57,0);
	// printf("%f\n",speed);
	// count++;
	// }
    // count=0;
	// while(count<500)
	// {
	// double speed=PID_realize(0,0);
	// printf("%f\n",speed);
	// count++;
	// }
	inter_init(0,0);
	for (size_t i = 0; i < 200; i++)
	{
		inter_realize(1, 1, 0);
	}
	return 0;

}