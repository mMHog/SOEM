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
#include <time.h>
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>

#define USECS_PER_SEC     1000000

int osal_usleep (unsigned int usec)
{
   struct timespec ts;
   ts.tv_sec = usec / USECS_PER_SEC;
   ts.tv_nsec = (usec % USECS_PER_SEC) * 1000;
   /* usleep is deprecated, use nanosleep instead */
   return nanosleep(&ts, NULL);
}


#define SERVO_NUMBER 18

#define WELD (1<<4)
#define DRIVE (1<<6)
#define RETRO (1<<5)
#define GAS (1<<7)

typedef struct
{
    int control_word;
    double position_command;
    double positon_feedback;
} Transfer;
typedef struct
{
    int command;
    int feedback;
    double Icommand;
    double Ucommand;
    double Ifeedback;
    double Ufeedback;
} WTransfer;
int main()
{
    key_t key = ftok("/dev/shm/myshm344", 0);
    int shm_id = shmget(key, 0x400000, IPC_CREAT | 0666);
    //double *p = (double *)shmat(shm_id, NULL, 0);
    // double p;
    int q;

    // Transfer *tran = (Transfer *)shmat(shm_id, NULL, 0);
    // WTransfer *wtran=(WTransfer *)(tran+SERVO_NUMBER);
    WTransfer *wtran=(WTransfer *)shmat(shm_id, NULL, 0);;

    // wtran->Icommand=50;
    // wtran->Ucommand=10;

    while (1)
    {
        printf("control: ");
        scanf("%d", &q);
        if(q==1){
          wtran->Icommand=120;
          wtran->Ucommand=16;
          wtran->command = WELD;
          osal_usleep(1000000);
          wtran->Icommand=100;
        }else if(q==2){
          wtran->command = DRIVE;
        }else if(q==3){
          wtran->command = RETRO;
        }else if(q==4){
          wtran->command = GAS;
        }else if(q==0){
          wtran->command = 0;
          wtran->Icommand=0;
          wtran->Ucommand=0;
        }
        // printf("%lf\n",wtran->positon_feedback);
    }
    //int a;scanf("%d",&a);
    //struct timespec tv;
    //for (size_t i = 0; i < 100; i++)
    //{
        //p[0] = 1.0 * i;
        //clock_gettime(CLOCK_MONOTONIC, &tv);
        //printf("%lf : %ld.%ld\n", p[0], tv.tv_sec, tv.tv_nsec);
        //sleep(1);
    //}

    //memset(p, 1, sizeof(int));
    //printf("%d %d %d %d .\n", p[0], p[1], p[2], p[3]);
    shmdt(wtran);

    return 0;
}