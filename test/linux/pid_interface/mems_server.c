/*
 * @Author: your name
 * @Date: 2021-04-12 13:17:25
 * @LastEditTime: 2021-04-13 15:19:41
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

int main()
{
    key_t key = ftok("/dev/shm/myshm344", 0);
    int shm_id = shmget(key, 0x400000, 0666);
    double *p = (double *)shmat(shm_id, NULL, 0);
    int pre = (int)p[0];
    struct timespec tv;
    while (1)
    {
        if ((int)p[0] != pre)
        {
            pre = (int)p[0];
            clock_gettime(CLOCK_MONOTONIC, &tv);
            printf("%lf : %ld.%ld\n", p[0], tv.tv_sec, tv.tv_nsec);
            /* code */
        }
    }

    //printf("%lf %lf %lf %lf .\n", p[0], p[1], p[2], p[3]);
    shmdt(p);

    return 0;
}