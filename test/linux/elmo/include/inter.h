/*
 * @Author: your name
 * @Date: 2021-04-15 13:00:10
 * @LastEditTime: 2021-11-02 16:29:09
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
    int x, v, a;
    int xf, vf, af;
    int target;
    double step;
    int num, current;
    double a0, a1, a2, a3, a4, a5;
} inter[slave_num];

void inter_init(int init_x, int i)
{
    printf("inter_init begin \n");

    inter[i].x = init_x;
    inter[i].v = 0;
    inter[i].a = 0;
    inter[i].xf = init_x;
    inter[i].vf = 0;
    inter[i].af = 0;
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

int inter_realize(int target, int num_left, int i,int feedback, int mode)
{
    inter[i].x = feedback;
    double t = inter[i].step;
    double tt = t * num_left;
    if ((mode == 1 || mode==2) && (num_left==0 || abs(target - inter[i].x) < 10*min_err))
    {        
        tt=inter[i].step;
    }
    if (abs(target - inter[i].x) < min_err || (num_left == 0 && mode == 0))
    {
        inter[i].x = target;
        inter[i].v = 0;
        inter[i].a = 0;
        //printf("%d %d %d \n", inter[i].x, inter[i].v, inter[i].a);
        if (mode == 0)
            return inter[i].x;
        else
            return 0;
    }
    
    inter[i].xf = target;
    inter[i].a0 = inter[i].x;
    inter[i].a1 = inter[i].v;
    inter[i].a2 = inter[i].a / 2;
    inter[i].a3 = (20 * (inter[i].xf - inter[i].x) - (8 * inter[i].vf + 12 * inter[i].v) * tt - (3 * inter[i].a - 2 * inter[i].af) * tt * tt) / (2 * tt * tt * tt);
    inter[i].a4 = (30 * (inter[i].x - inter[i].xf) + (14 * inter[i].vf + 16 * inter[i].v) * tt + (3 * inter[i].a - 2 * inter[i].af) * tt * tt) / (2 * tt * tt * tt * tt);
    inter[i].a5 = (12 * (inter[i].xf - inter[i].x) - (6 * inter[i].vf + 6 * inter[i].v) * tt - (inter[i].a - inter[i].af) * tt * tt) / (2 * tt * tt * tt * tt * tt);
    inter[i].x = inter[i].a0 + t * (inter[i].a1 + t * (inter[i].a2 + t * (inter[i].a3 + t * (inter[i].a4 + inter[i].a5 * t))));
    inter[i].v = inter[i].a1 + t * (2 * inter[i].a2 + t * (3 * inter[i].a3 + t * (4 * inter[i].a4 + 5 * t * inter[i].a5)));
    inter[i].a = 2 * inter[i].a2 + t * (6 * inter[i].a3 + t * (12 * inter[i].a4 + 20 * inter[i].a5 * t));
    //printf("%lf %lf %lf \n", inter[i].x, inter[i].v, inter[i].a);
    //double com=(inter[i].x-feedback)/step;
    //if(com>0.05*inter[i].v && inter[i].v>0) com=0.05*inter[i].v;
    //else if(com<0.05*inter[i].v && inter[i].v<0) com=0.05*inter[i].v;
    if(mode==2) {inter[i].v = Kpp * (target - inter[i].x)/tt;inter[i].x+=inter[i].v*tt;}
    if (mode == 0)
        return inter[i].x;
    else
    {
        if(inter[i].v>max_v[i]) {inter[i].a = 0;return inter[i].v=max_v[i];}
        else if(inter[i].v<min_v[i]) {inter[i].a = 0;return inter[i].v=min_v[i];}
        else return inter[i].v;
    }
        //return inter[i].v;
    // return inter[i].v;
}

#endif // !_PID_H
