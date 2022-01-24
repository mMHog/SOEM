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
#include <stdlib.h>

//结构体，必须
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
    const int operation_mode_display;
} Transfer;

int main()
{
    //以下三行，初始化
    key_t key = ftok("/dev/shm/myshm344", 0);
    int shm_id = shmget(key, 0x400000, IPC_CREAT | 0666);
    Transfer *tran = (Transfer *)shmat(shm_id, NULL, 0);

    int p,i;
    char c[2];
    while (1)
    {
        printf("%d %d %d\n", i, tran[i].position_feedback, tran[i].operation_mode_display);
        printf("Input(p,v,t,c): ");
        scanf("%s %d %d", c,&i, &p);


        if (c[0] == 'p')
        {
            //调用位置接口
            tran[i].velocity_command = 50000;
            tran[i].position_command = p;
            tran[i].position_enable = 1;//位置指令使能信号
        }
        if (c[0] == 'v')
        {
            //调用速度接口
            tran[i].velocity_command = p;
        }
        if (c[0] == 't')
        {
            //调用力矩接口
            tran[i].torque_command = p;
        }
        if (c[0] == 'c')
        {
            //改变控制模式，0：位置，1：速度，2：力矩，调用后需要等待一秒
            tran[i].operation_mode = p;
        }        
    }
    shmdt(tran);

    return 0;
}
