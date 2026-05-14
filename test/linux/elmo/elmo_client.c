/*
 * @Author: your name
 * @Date: 2021-04-12 13:17:53
 * @LastEditTime: 2021-11-12 20:51:34
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
#include <stdlib.h>
typedef struct
{
    int position_command;
    int velocity_command;
    int torque_command;
    const int position_feedback;
    const int velocity_feedback;
    const int torque_feedback;
    int position_enable;
    int operation_mode;
    int operation_mode_display;
} Transfer;

int main()
{
    int num = 15;
    char base[100] =  "sudo  ~/hw/SOEMv7/build/test/linux/elmo/elmo enp7s0"; //网卡名，用ifcongig查询

    key_t key = ftok("/dev/shm/myshm344", 0);
    int shm_id = shmget(key, 0x400000, IPC_CREAT | 0666);
    Transfer *tran = (Transfer *)shmat(shm_id, NULL, 0);
    char arg[100] = "";
    char command[200] = "";
    system(base);

    while (1)
    {
        for (int i = 0; i < num; i++)
        {
            strcat(arg, " ");
            if (tran[i].operation_mode_display == 0)
                strcat(arg, "0");
            if (tran[i].operation_mode_display == 1)
                strcat(arg, "1");
            if (tran[i].operation_mode_display == 2)
                strcat(arg, "2");
        }
        strcat(command, base);
        strcat(command, arg);
        // printf("%s %s %d\n", command, arg, tran[0].operation_mode);
        system(command);
        command[0] = arg[0] = '\0';
    }

    shmdt(tran);

    return 0;
}
