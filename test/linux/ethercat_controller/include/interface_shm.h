//
// Created by ralph on 5/28/25.
//

#ifndef INTERFACE_SHM_H
#define INTERFACE_SHM_H

#include "common.h"

key_t key;
int shm_id;



typedef struct
{
    int control_word;
    double position_command;
    double positon_feedback;
} Transfer;

Transfer* tran;

int is_shm_init=0;

void shm_init()
{
    key = ftok("/dev/shm/myshm344", 0);
    shm_id = shmget(key, 0x400000, IPC_CREAT | 0666);
    tran = (Transfer*)shmat(shm_id, NULL, 0);
    return 0;
}

int data_receive_shm()
{
    if (!is_shm_init)
    {
        shm_init();
    }
    return 0;
}
#endif //INTERFACE_SHM_H
