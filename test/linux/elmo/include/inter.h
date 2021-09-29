/*
 * @Author: your name
 * @Date: 2021-04-15 13:00:10
 * @LastEditTime: 2021-04-15 17:06:29
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /SOEM/test/linux/pid_interface/include/inter.h
 */
#ifndef _INTER_H
#define _INTER_H

#include <stdio.h>
#include <stdlib.h>
#include "./config.h"
struct inter
{
    double x, v, a;
    double xf, vf, af;
    double target;
    double step;
    int num, current;
    double a0, a1, a2, a3, a4, a5;
} inter[7];

void inter_init(double init_x, int i)
{
    printf("inter_init begin \n");

    inter[i].x = init_x;
    inter[i].v = 0.0;
    inter[i].a = 0.0;
    inter[i].xf = init_x;
    inter[i].vf = 0.0;
    inter[i].af = 0.0;
    inter[i].step = step;
    inter[i].num = 0;
    //inter[i].current = 0;
    inter[i].a0 = 0.0;
    inter[i].a1 = 0.0;
    inter[i].a2 = 0.0;
    inter[i].a3 = 0.0;
    inter[i].a4 = 0.0;
    inter[i].a5 = 0.0;

    printf("inter_init end \n");
}

double inter_realize(double target, int num, int i)
{
    if (abs(target - inter[i].x) < min_err)
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

#endif // !_PID_H