//
// Created by ralph on 5/28/25.
//

#ifndef CONTROLLER_AC_H
#define CONTROLLER_AC_H

#include "common.h"
#include "ec_slave.h"

typedef struct PACKED
{
    uint32 position;
} ac58_TPdo;


typedef struct
{
    int ac_encoder_zero_raw;
    int ac_encoder_limit_upper_raw;
    int ac_encoder_limit_lower_raw;
} ac58_data_interface_command;

typedef struct
{
    double ac_encoder_rad;
    unsigned int ac_encoder_raw;
} ac58_data_interface_feedback;


int ac58_init(ec_slave_s ec_slave_ac58)
{
    ac58_TPdo** ptrs_T = (ac58_TPdo**)ec_slave_ac58.Tpdo_p;
    for (int i = 0; i < ec_slave_ac58.slave_mun; i++)
    {
        ptrs_T[i] = (ac58_TPdo*)(ec_slave[i + 1 + ec_slave_ac58.index_start].inputs);
    }
    return 0;
}

int ac58_config(ec_slave_s ec_slave_ac58)
{
    return 0;
}



int ac58_pdo(ec_slave_s ec_slave_ac58)
{
    ac58_TPdo** ptrs_T = (ac58_TPdo**)ec_slave_ac58.Tpdo_p;

    ((ac58_data_interface_feedback*)ec_slave_ac58.hw_interface_feedback)->ac_encoder_raw=ptrs_T[0]->position;
    ((ac58_data_interface_feedback*)ec_slave_ac58.hw_interface_feedback)->ac_encoder_rad=1.49727e-6*(int)(ptrs_T[0]->position-((ac58_data_interface_command*)ec_slave_ac58.hw_interface_command)->ac_encoder_zero_raw);
    return 0;
}

int ac58_screen_log(ec_slave_s ec_slave_ac58,int mode)
{
    if (mode==1)
        printf("%.2f\t", ((ac58_data_interface_feedback*)ec_slave_ac58.hw_interface_feedback)->ac_encoder_rad*r2d);
    else if (mode==2)
        printf("%d\t", ((ac58_data_interface_feedback*)ec_slave_ac58.hw_interface_feedback)->ac_encoder_raw);
    return 0;
}
int ac58_data_log(ec_slave_s ec_slave_ac58,FILE* log_f)
{
    fprintf(log_f,"%.6f\t", ((ac58_data_interface_feedback*)ec_slave_ac58.hw_interface_feedback)->ac_encoder_rad);
    return 0;
}
int ac58_raw_log(ec_slave_s ec_slave_ac58,FILE* log_f_raw)
{
    fprintf(log_f_raw,"%d\t", ((ac58_data_interface_feedback*)ec_slave_ac58.hw_interface_feedback)->ac_encoder_raw);
    return 0;
}

int ac58_safe_check(ec_slave_s ec_slave_ac58)
{
    // ac58_TPdo** ptrs_T = (ac58_TPdo**)ec_slave_ac58.Tpdo_p;
    // for (int i = 0; i < ec_slave_ac58.slave_mun; i++)
    // {
    //     if (ptrs_T[i]->position>ac_encoder_limit_upper[i]||ptrs_T[i]->position<ac_encoder_limit_lower[i])
    //     {
    //         printf("ERROR AC encoder %d is %d\n",i,ptrs_T[i]->position);
    //         return -1;
    //     }
    // }
    return 0;
}
int ac58_safe_handle(ec_slave_s ec_slave_cdr6)
{
    return 0;
}
#endif //CONTROLLER_AC_H
