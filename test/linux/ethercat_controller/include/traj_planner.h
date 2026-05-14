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

#include "common.h"

// long int zero_position[18] = {94879,-216914,268775,2558795,519740,190464,-341512,250790,110862,-2372377,-1973210,-225506,265421,266114,209112,152844,165471,2524202};

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

struct inter
{
    double x, v, a;
    double xf, vf, af;
    double target;
    double step;
    int num, current;
    double a0, a1, a2, a3, a4, a5;
} inter[18];

void inter_init(double init_x, int i)
{
    //printf("inter_init begin \n");

    inter[i].x = init_x;
    inter[i].v = 0.0;
    inter[i].a = 0.0;
    inter[i].xf = init_x;
    inter[i].vf = 0.0;
    inter[i].af = 0.0;
    inter[i].step = 0.004;
    inter[i].num = 0;
    //inter[i].current = 0;
    inter[i].a0 = 0.0;
    inter[i].a1 = 0.0;
    inter[i].a2 = 0.0;
    inter[i].a3 = 0.0;
    inter[i].a4 = 0.0;
    inter[i].a5 = 0.0;

    //printf("inter_init end \n");
}

double inter_realize(double target, int num, int i)
{
    if (target - inter[i].x < 0.0001 && target - inter[i].x > -0.0001)
    {
        inter[i].x = target;
        inter[i].v = 0;
        inter[i].a = 0;
        //printf("%lf %lf %lf \n", inter[i].x, inter[i].v, inter[i].a);
        return inter[i].x;
    }

    double t = inter[i].step;
    double tt =t*num;
    inter[i].xf = target;
    inter[i].a0 = inter[i].x;
    inter[i].a1 = inter[i].v;
    inter[i].a2 = inter[i].a / 2;
    inter[i].a3 = (20 * (inter[i].xf - inter[i].x) - (8 * inter[i].vf + 12 * inter[i].v) * tt - (3 * inter[i].a - 2 * inter[i].af) * tt * tt) / (2 * tt * tt * tt);
    inter[i].a4 = (30 * (inter[i].x - inter[i].xf) + (14 * inter[i].vf + 16 * inter[i].v) * tt + (3 * inter[i].a - 2 * inter[i].af) * tt * tt) / (2 * tt * tt * tt * tt);
    inter[i].a5 = (12 * (inter[i].xf - inter[i].x) - (6 * inter[i].vf + 6 * inter[i].v) * tt - (inter[i].a - inter[i].af) * tt * tt) / (2 * tt * tt * tt * tt * tt);
    inter[i].x = inter[i].a0 + t * (inter[i].a1 + t * (inter[i].a2 + t * (inter[i].a3 + t * (inter[i].a4 + inter[i].a5 * t))));
    inter[i].v = inter[i].a1 + t * (2 * inter[i].a2 + t * (3 * inter[i].a3 + t * (4 * inter[i].a4 + 5 * inter[i].a5 * t)));
    inter[i].a = 2 * inter[i].a2 + t * (6 * inter[i].a3 + t * (12 * inter[i].a4 + 20 * inter[i].a5 * t));
    //printf("%lf %lf %lf \n", inter[i].x, inter[i].v, inter[i].a);
    return inter[i].x;
}

#endif // !_DATA_HANDLE_H
