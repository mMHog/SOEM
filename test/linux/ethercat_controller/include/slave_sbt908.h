//
// Created by root on 5/30/25.
//

#ifndef SBT908_SLAVE_H
#define SBT908_SLAVE_H

#include "common.h"
#include "ec_slave.h"

typedef struct PACKED
{
    int32 force[10];
} sbt908_TPdo;


typedef struct
{
    int fs_mode[10];
    int fs_limit_raw;
} sbt908_data_interface_command;

typedef struct
{
    int fs_raw[10];
    double fs_data[10];
} sbt908_data_interface_feedback;


int sbt908_init(ec_slave_s ec_slave_sbt908)
{
    sbt908_TPdo** ptrs_T = (sbt908_TPdo**)ec_slave_sbt908.Tpdo_p;
    for (int i = 0; i < ec_slave_sbt908.slave_mun; i++)
    {
        ptrs_T[i] = (sbt908_TPdo*)(ec_slave[i + 1 + ec_slave_sbt908.index_start].inputs);
    }
    return 0;
}

int sbt908_config(ec_slave_s ec_slave_sbt908)
{
    return 0;
}

int sbt908_pdo(ec_slave_s ec_slave_sbt908)
{
    sbt908_TPdo** ptrs_T = (sbt908_TPdo**)ec_slave_sbt908.Tpdo_p;
    for (int i = 0; i < 10; i++)
    {
        ((sbt908_data_interface_feedback*)ec_slave_sbt908.hw_interface_feedback)->fs_raw[i]=ptrs_T[0]->force[i];
        ((sbt908_data_interface_feedback*)ec_slave_sbt908.hw_interface_feedback)->fs_data[i]=0.01*(ptrs_T[0]->force[i]);
    }
    return 0;
}

int sbt908_screen_log(ec_slave_s ec_slave_sbt908,int mode)
{
    for (int i = 0; i < 10; i++)
    {
        if (((sbt908_data_interface_command*)ec_slave_sbt908.hw_interface_command)->fs_mode[i]!=0)
        {
            if (mode==1)
                printf("%.2f\t",  ((sbt908_data_interface_feedback*)ec_slave_sbt908.hw_interface_feedback)->fs_data[i]);
            else if (mode==2)
                printf("%d\t",  ((sbt908_data_interface_feedback*)ec_slave_sbt908.hw_interface_feedback)->fs_raw[i]);

        }
    }
    return 0;
}
int sbt908_data_log(ec_slave_s ec_slave_sbt908,FILE* log_f)
{
    for (int i = 0; i < 10; i++)
    {
        if (((sbt908_data_interface_command*)ec_slave_sbt908.hw_interface_command)->fs_mode[i]!=0)
        {
                fprintf(log_f,"%.6f\t",  ((sbt908_data_interface_feedback*)ec_slave_sbt908.hw_interface_feedback)->fs_data[i]);
        }
    }
    return 0;
}
int sbt908_raw_log(ec_slave_s ec_slave_sbt908,FILE* log_f_raw)
{
    for (int i = 0; i < 10; i++)
    {
        if (((sbt908_data_interface_command*)ec_slave_sbt908.hw_interface_command)->fs_mode[i]!=0)
        {
            fprintf(log_f_raw,"%d\t",  ((sbt908_data_interface_feedback*)ec_slave_sbt908.hw_interface_feedback)->fs_raw[i]);
        }
    }
    return 0;
}
int sbt908_safe_check(ec_slave_s ec_slave_sbt908)
{
    sbt908_TPdo** ptrs_T = (sbt908_TPdo**)ec_slave_sbt908.Tpdo_p;
    for (int i = 0; i < ec_slave_sbt908.slave_mun; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            if (abs(ptrs_T[i]->force[j])>((sbt908_data_interface_command*)ec_slave_sbt908.hw_interface_command)->fs_limit_raw)
            {
                printf("ERROR Force Sensor %d is %d\n",j,ptrs_T[i]->force[j]);
                return -1;
            }
        }
    }
    return 0;
}
int sbt908_safe_handle(ec_slave_s ec_slave_sbt908)
{
    return 0;
}
#endif //SBT908_SLAVE_H
