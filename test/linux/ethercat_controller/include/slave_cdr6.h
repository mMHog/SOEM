//
// Created by ralph on 5/28/25.
//

#ifndef CONTROLLER_CDR_H
#define CONTROLLER_CDR_H

#include "common.h"
#include "ec_slave.h"
#include "cia402.h"

typedef struct PACKED
{
    uint16 control_word;
    int32 target_position;
    int32 target_velocity;
    int16 target_torque;
    int8 op_mode;
} cdr6_RPdo;

typedef struct PACKED
{
    uint16 status_word;
    int32 actual_position;
    int32 actual_velocity;
    int16 actual_torque;
    int8 op_mode_display;
} cdr6_TPdo;


typedef struct
{
    int mode[6];
    double target_position[6];
    int position_limit_upper_raw[6];
    int position_limit_lower_raw[6];
    int velocity_limit_raw;
    int torque_limit_raw;
    int mtr_zero_raw[6];
} cdr6_data_interface_command;

typedef struct
{
    double actual_position[6];
    double actual_velocity[6];
    double actual_torque[6];
    int target_position_raw[6];
    int actual_position_raw[6];
    int actual_velocity_raw[6];
    int actual_torque_raw[6];
} cdr6_data_interface_feedback;


int cdr6_init(ec_slave_s ec_slave_cdr6)
{
    cdr6_RPdo** ptrs_R = (cdr6_RPdo**)ec_slave_cdr6.Rpdo_p;
    cdr6_TPdo** ptrs_T = (cdr6_TPdo**)ec_slave_cdr6.Tpdo_p;
    for (int i = 0; i < ec_slave_cdr6.slave_mun; i++)
    {
        ptrs_R[i] = (cdr6_RPdo*)(ec_slave[i+1 + ec_slave_cdr6.index_start].outputs);
        ptrs_T[i] = (cdr6_TPdo*)(ec_slave[i+1 + ec_slave_cdr6.index_start].inputs);
        ((cdr6_data_interface_feedback*)ec_slave_cdr6.hw_interface_feedback)->target_position_raw[i]=ptrs_R[i]->target_position = ptrs_T[i]->actual_position;
        ((cdr6_data_interface_command*)ec_slave_cdr6.hw_interface_command)->target_position[i]=4.79369e-5*(double)(ptrs_T[i]->actual_position-((cdr6_data_interface_command*)ec_slave_cdr6.hw_interface_command)->mtr_zero_raw[i]);
        ptrs_R[i]->target_velocity = 0;
        ptrs_R[i]->target_torque = 0;
        ptrs_R[i]->op_mode = ((cdr6_data_interface_command*)ec_slave_cdr6.hw_interface_command)->mode[i];
        if (ptrs_R[i]->op_mode != 0)
        {
            servo_init(&ptrs_R[i]->control_word, &ptrs_T[i]->status_word);
        }
    }
    return 0;
}

int cdr6_config(ec_slave_s ec_slave_cdr6)
{
    return 0;
}

int cdr6_pdo(ec_slave_s ec_slave_cdr6)
{
    cdr6_RPdo** ptrs_R = (cdr6_RPdo**)ec_slave_cdr6.Rpdo_p;
    cdr6_TPdo** ptrs_T = (cdr6_TPdo**)ec_slave_cdr6.Tpdo_p;
    for (int i = 0; i < ec_slave_cdr6.slave_mun; i++)
    {
        if (((cdr6_data_interface_command*)ec_slave_cdr6.hw_interface_command)->mode[i] != 0)
        {
            ((cdr6_data_interface_feedback*)ec_slave_cdr6.hw_interface_feedback)->actual_position_raw[i]=ptrs_T[i]->actual_position;
            ((cdr6_data_interface_feedback*)ec_slave_cdr6.hw_interface_feedback)->actual_velocity_raw[i]=ptrs_T[i]->actual_velocity;
            ((cdr6_data_interface_feedback*)ec_slave_cdr6.hw_interface_feedback)->actual_torque_raw[i]=ptrs_T[i]->actual_torque;
            ((cdr6_data_interface_feedback*)ec_slave_cdr6.hw_interface_feedback)->actual_position[i]=4.79369e-5*(double)(ptrs_T[i]->actual_position-((cdr6_data_interface_command*)ec_slave_cdr6.hw_interface_command)->mtr_zero_raw[i]);
            ((cdr6_data_interface_feedback*)ec_slave_cdr6.hw_interface_feedback)->actual_velocity[i]=4.79369e-5*(double)ptrs_T[i]->actual_velocity;
            ((cdr6_data_interface_feedback*)ec_slave_cdr6.hw_interface_feedback)->actual_torque[i]=0.00064*(double)ptrs_T[i]->actual_torque;
            ptrs_R[i]->target_position=(int)(2.0861e+04*((cdr6_data_interface_command*)ec_slave_cdr6.hw_interface_command)->target_position[i])+((cdr6_data_interface_command*)ec_slave_cdr6.hw_interface_command)->mtr_zero_raw[i];
            ((cdr6_data_interface_feedback*)ec_slave_cdr6.hw_interface_feedback)->target_position_raw[i]= ptrs_R[i]->target_position;
        }
    }
    return 0;
}


int cdr6_screen_log(ec_slave_s ec_slave_cdr6,int mode)
{
    for (int i = 0; i < ec_slave_cdr6.slave_mun; i++)
    {
        if (((cdr6_data_interface_command*)ec_slave_cdr6.hw_interface_command)->mode[i] != 0)
        {
            if (mode==1)
                printf("%.2f\t%.2f\t%.2f\t%.2f\t ", ((cdr6_data_interface_command*)ec_slave_cdr6.hw_interface_command)->target_position[i]*r2d
                    , ((cdr6_data_interface_feedback*)ec_slave_cdr6.hw_interface_feedback)->actual_position[i]*r2d
                    , ((cdr6_data_interface_feedback*)ec_slave_cdr6.hw_interface_feedback)->actual_velocity[i]*r2d
                    , ((cdr6_data_interface_feedback*)ec_slave_cdr6.hw_interface_feedback)->actual_torque[i]);
            else if (mode==2)
                printf("%d\t%d\t%d\t%d\t ",  ((cdr6_data_interface_feedback*)ec_slave_cdr6.hw_interface_feedback)->target_position_raw[i]
                    , ((cdr6_data_interface_feedback*)ec_slave_cdr6.hw_interface_feedback)->actual_position_raw[i]
                    , ((cdr6_data_interface_feedback*)ec_slave_cdr6.hw_interface_feedback)->actual_velocity_raw[i]
                    , ((cdr6_data_interface_feedback*)ec_slave_cdr6.hw_interface_feedback)->actual_torque_raw[i]);
        }
    }
    return 0;
}
int cdr6_data_log(ec_slave_s ec_slave_cdr6,FILE* log_f)
{
    for (int i = 0; i < ec_slave_cdr6.slave_mun; i++)
    {
        if (((cdr6_data_interface_command*)ec_slave_cdr6.hw_interface_command)->mode[i] != 0)
        {
               fprintf(log_f,"%.6f\t%.6f\t%.6f\t%.6f\t ", ((cdr6_data_interface_command*)ec_slave_cdr6.hw_interface_command)->target_position[i]
                    , ((cdr6_data_interface_feedback*)ec_slave_cdr6.hw_interface_feedback)->actual_position[i]
                    , ((cdr6_data_interface_feedback*)ec_slave_cdr6.hw_interface_feedback)->actual_velocity[i]
                    , ((cdr6_data_interface_feedback*)ec_slave_cdr6.hw_interface_feedback)->actual_torque[i]);
        }
    }
    return 0;
}
int cdr6_raw_log(ec_slave_s ec_slave_cdr6,FILE* log_f_raw)
{
    for (int i = 0; i < ec_slave_cdr6.slave_mun; i++)
    {
        if (((cdr6_data_interface_command*)ec_slave_cdr6.hw_interface_command)->mode[i] != 0)
        {
            fprintf(log_f_raw,"%d\t%d\t%d\t%d\t ",  ((cdr6_data_interface_feedback*)ec_slave_cdr6.hw_interface_feedback)->target_position_raw[i]
                    , ((cdr6_data_interface_feedback*)ec_slave_cdr6.hw_interface_feedback)->actual_position_raw[i]
                    , ((cdr6_data_interface_feedback*)ec_slave_cdr6.hw_interface_feedback)->actual_velocity_raw[i]
                    , ((cdr6_data_interface_feedback*)ec_slave_cdr6.hw_interface_feedback)->actual_torque_raw[i]);
        }
    }
    return 0;
}

int cdr6_safe_check(ec_slave_s ec_slave_cdr6)
{
    cdr6_RPdo** ptrs_R = (cdr6_RPdo**)ec_slave_cdr6.Rpdo_p;
    cdr6_TPdo** ptrs_T = (cdr6_TPdo**)ec_slave_cdr6.Tpdo_p;

    for (int i = 0; i < ec_slave_cdr6.slave_mun; i++)
    {
        if (((cdr6_data_interface_command*)ec_slave_cdr6.hw_interface_command)->mode[i] != 0)
        {
            if (ptrs_T[i]->actual_position>((cdr6_data_interface_command*)ec_slave_cdr6.hw_interface_command)->position_limit_upper_raw[i]
                ||ptrs_T[i]->actual_position<((cdr6_data_interface_command*)ec_slave_cdr6.hw_interface_command)->position_limit_lower_raw[i])
            {
                printf("ERROR Motor encoder %d is %d\n",i,ptrs_T[i]->actual_position);
                return -1;
            }
            if (abs(ptrs_T[i]->actual_velocity)>((cdr6_data_interface_command*)ec_slave_cdr6.hw_interface_command)->velocity_limit_raw)
            {
                printf("ERROR Motor encoder velocity %d is %d\n",i,ptrs_T[i]->actual_velocity);
                return -1;
            }
            if (abs(ptrs_T[i]->actual_torque)>((cdr6_data_interface_command*)ec_slave_cdr6.hw_interface_command)->torque_limit_raw)
            {
                printf("ERROR Motor encoder torque %d is %d\n",i,ptrs_T[i]->actual_torque);
                return -1;
            }
        }
    }
    return 0;
}
int cdr6_safe_handle(ec_slave_s ec_slave_cdr6)
{
    cdr6_RPdo** ptrs_R = (cdr6_RPdo**)ec_slave_cdr6.Rpdo_p;
    cdr6_TPdo** ptrs_T = (cdr6_TPdo**)ec_slave_cdr6.Tpdo_p;
    for (int i = 0; i < ec_slave_cdr6.slave_mun; i++)
    {
        servo_stop(&ptrs_R[i]->control_word, &ptrs_T[i]->status_word);
    }
    return 0;
}

#endif //CONTROLLER_CDR_H