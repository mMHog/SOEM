/*
 * @Author: your name
 * @Date: 2021-04-10 14:01:38
 * @LastEditTime: 2021-04-26 17:26:06
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /SOEM/test/linux/pid_interface/include/data_handle.h
 */
#ifndef _DATA_HANDLE_H
#define _DATA_HANDLE_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <math.h>

// long int zero_position[18] = {94879,-216914,268775,2558795,519740,190464,-341512,250790,110862,-2372377,-1973210,-225506,265421,266114,209112,152844,165471,2524202};
long int zero_position[18] = {-28129,979562,230434,-131607,2427951,-1176797,  
							  -56737,67609,302859,-4939445,-7512,-3319151,  
							  -89460,-1812698,-413453,-11044222,1574536,-379825};
long int incpdeg[18] = {-53521,65238,-59738,-28017,-25486,-15969,
						-53521,65238,-59738,-28017,-25486,-15969,
						-55324,55706,-55708,-31147,-26970,26970};

long int deg2inc(double deg, int i)
{
	return (long int)(deg * incpdeg[i]) + zero_position[i];
}

long int rad2inc(double rad, int i)
{
	return (long int)(rad * 180 / 3.1415926 * incpdeg[i]) + zero_position[i];
}
long int rad2inc2(double rad, int i)
{
	return (long int)(rad * 180 / 3.1415926 * incpdeg[i]);
}
double inc2rad(long int inc, int i)
{
	return (((double)(inc - zero_position[i])) / incpdeg[i]) * 3.1415926 / 180;
}
double inc2rad2(long int inc, int i)
{
	return ((double)(inc) / incpdeg[i]) * 3.1415926 / 180;
}
int data_process(double *outputx, double x2, double x1, double v, double a, double delta)
{
	double x = x2 - x1;
	int neg = 0;
	if (x < 0)
	{
		x = -x;
		neg = 1;
	}
	double t1, t2;
	t1 = v / a;
	t2 = x / v - t1;
	outputx[0] = 0;
	if (t2 <= 0)
	{
		t1 = sqrt(x / a);
		t2 = 0;
		v = a * t1;
	}
	int n1 = t1 / delta, n2 = t2 / delta;
	outputx[n1] = a * t1 * t1 / 2;
	outputx[n1 + n2] = outputx[n1] + v * t2;
	outputx[n1 * 2 + n2] = x;
	for (int i = 1; i < n1; ++i)
	{
		outputx[i] = a * i * i * delta * delta / 2;
		outputx[n1 + n2 + i] = outputx[n1 + n2] + v * i * delta - outputx[i];
	}
	for (int i = 1; i < n2; ++i)
	{
		outputx[n1 + i] = outputx[n1] + v * i * delta;
	}
	if (neg == 1)
	{
		for (int i = 0; i < n1 * 2 + n2 + 1; ++i)
		{
			outputx[i] = -outputx[i];
		}
	}
	for (int i = 0; i < n1 * 2 + n2 + 1; ++i)
	{
		outputx[i] = outputx[i] + x1;
	}
	return (n1 * 2 + n2 + 1);
}

#endif // !_DATA_HANDLE_H
