#ifndef _CONFIG_H
#define _CONFIG_H

// 0: weizhi pid 1: sudu pid 2:duoxiangshi
#define MODE 1

int max[] = {1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000};
int min[] = {-1000000, -1000000, -1000000, -1000000, -1000000, -1000000, -1000000};

double Kp[] = {0.002, 0.002, 0.002, 0.002, 0.002, 0.002, 0.002};
double Ki[] = {0, 0, 0, 0, 0, 0, 0};
double Kd[] = {0.002, 0.002, 0.002, 0.002, 0.002, 0.002, 0.002};

double Kp_v[] = {2, 2, 2, 2, 2, 2, 2};
double Ki_v[] = {0, 0, 0, 0, 0, 0, 0};
double Kd_v[] = {2, 2, 2, 2, 2, 2, 2};

double max_err=3000;
double min_err_i = 200;
double min_err = 50;

double step=0.001;
int step_num = 2000;

#endif