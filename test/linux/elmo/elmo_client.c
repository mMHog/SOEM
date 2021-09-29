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

typedef struct
{
    int position_command;
    const int positon_feedback;
} Transfer;
int main()
{
    key_t key = ftok("/dev/shm/myshm344", 0);
    int shm_id = shmget(key, 0x400000, IPC_CREAT | 0666);
    Transfer *tran = (Transfer *)shmat(shm_id, NULL, 0);

    int p;
    while (1)
    {
        printf("%d\n",tran[0].positon_feedback);
        printf("target: ");
        scanf("%d", &p);
        tran[0].position_command = p;
    }
    shmdt(tran);

    return 0;
}