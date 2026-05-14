/*
 * @Author: your name
 * @Date: 2021-04-12 13:17:25
 * @LastEditTime: 2021-04-14 12:07:58
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /SOEM/test/linux/pid_interface/mems_server.c
 */

#include <sys/shm.h>
#include <unistd.h>
#include <string.h>
#include <sys/ipc.h>
#include <stdio.h>
#include <time.h>

typedef struct
{
    int control_word;
    double position_command;
    double positon_feedback;
} Transfer;
int main()
{
    key_t key = ftok("/dev/shm/myshm344", 0);
    int shm_id = shmget(key, 0x400000, IPC_CREAT | 0666);
    //double *p = (double *)shmat(shm_id, NULL, 0);
    //double p;

    Transfer *tran = (Transfer *)shmat(shm_id, NULL, 0);

    while (1)
    {
        //scanf("%lf", &p);
        //tran[3].position_command = p;
        printf("%lf\n", tran[3].position_command);
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