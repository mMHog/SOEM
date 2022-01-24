/*
 * @Author: your name
 * @Date: 2021-04-12 13:17:53
 * @LastEditTime: 2021-04-26 16:38:26
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /SOEM/test/linux/pid_interface/mems_client.c
 */
#include <sys/shm.h>
#include <unistd.h>
#include <string.h>
#include <sys/ipc.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "ethercat.h"

#define  RANGE 0.2

typedef struct
{
    int control_word;
    double position_command;
    double positon_feedback;
} Transfer;
double fitter(double a,double b,double x){
    if (x>a)
    {
        return a;
    }else if (x<b)
    {
        return b;
    }else
    {
        return x;
    }
    
    
}
int main()
{
    double pi=3.1415926;
    key_t key = ftok("/dev/shm/myshm344", 0);
    int shm_id = shmget(key, 0x400000, IPC_CREAT | 0666);
    double upper[6] = {(170.0/180.0) * pi,
                  (90.0/180.0) * pi,
                  (160.0/180) * pi,
                  (360.0/180.0) * pi,
                  (125.0/180.0) * pi,
                  (360.0/180.0) * pi};
    double lower[6] = {(-170.0/180.0) * pi,
                  (-90.0/180.0) * pi,
                  (-80.0/180.0) * pi,
                  (-360.0/180.0) * pi,
                  (-125.0/180.0) * pi,
                  (-360.0/180.0) * pi};

    Transfer *tran = (Transfer *)shmat(shm_id, NULL, 0);

    srand((unsigned)time(NULL));
    while (1)
    {
        for (size_t i = 0; i < 6; i++)
        {
            tran[i].control_word = 1;
            tran[i].position_command = fitter(upper[i]/5,lower[i]/5,tran[i].positon_feedback+(RANGE*2*rand()/(RAND_MAX+1.0)-RANGE));
            printf("%lf ",tran[i].position_command);
        }
        printf("\n");
        usleep(5000000);
    }
    shmdt(tran);

    return 0;
}