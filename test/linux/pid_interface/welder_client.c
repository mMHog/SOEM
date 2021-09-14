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


#define SERVO_NUMBER 18

#define WELD (1<<4)
#define DRIVE (1<<6)
#define RETRO (1<<5)

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

    Transfer *tran = (Transfer *)shmat(shm_id, NULL, 0);
    WTransfer *wtran=(WTransfer *)(tran+SERVO_NUMBER);

    wtran->Icommand=150;
    wtran->Ucommand=10;

    while (1)
    {
        printf("control: ");
        scanf("%d", &q);
        if(q==1){
          wtran->command = WELD|DRIVE;
        }else if(q==0){
          wtran->command = 0;
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
    shmdt(tran);

    return 0;
}