//
// Created by ralph on 5/28/25.
//

#ifndef CONFIG_H
#define CONFIG_H

#include "common.h"
#include "slave_cdr6.h"
#include "slave_ac58.h"
#include "slave_sbt908.h"
#include "interface_shm.h"


int screen_log_mode=1;
int screen_event_log_on=1;
int data_log_on=1;
int raw_log_on=1;
int event_log_on=0;
int var_log_on=1;

char* ifname = "eno1";
int cycle_time = 2000;
/* Slave Distributed Clock Configuration */
boolean dcsync_enable = TRUE;

#define MTR_MODE {8,8,0,0,0,0}
#define MTR_LIMIT_UPPER {13000000,13000000,0,0,0,0}
#define MTR_LIMIT_LOWER {-13000000,-13000000,0,0,0,0}
#define MTR_VELOCITY_LIMIT 10000000
#define MTR_TORQUE_LIMIT 2000
#define MTR_ZERO {1248826,102379,0,0,0,0}

#define AC_ENCODER_LIMIT_UPPER_1 2500000
#define AC_ENCODER_LIMIT_LOWER_1 1560000
#define ENCODER_ZERO_1 2016865

#define AC_ENCODER_LIMIT_UPPER_2 970000
#define AC_ENCODER_LIMIT_LOWER_2 90000
#define ENCODER_ZERO_2 527830

#define FORCE_SENSOR_MODE {1,1,1,1,1,1,0,0,0,0}
#define FORCE_SENSOR_LIMIT 1000

int (*data_receive)() = data_receive_shm;


cdr6_RPdo* cdr6_RPdo_p[6];
cdr6_TPdo* cdr6_TPdo_p[6];
cdr6_data_interface_command cdr6_data_interface_command_p={MTR_MODE,{0.0},MTR_LIMIT_UPPER,MTR_LIMIT_LOWER,
    MTR_VELOCITY_LIMIT,MTR_TORQUE_LIMIT,MTR_ZERO};
cdr6_data_interface_feedback cdr6_data_interface_feedback_p;


ac58_TPdo* ac58_TPdo_p_1[1];

ac58_data_interface_command ac58_data_interface_command_p_1={ENCODER_ZERO_1,
    AC_ENCODER_LIMIT_UPPER_1,AC_ENCODER_LIMIT_LOWER_1};
ac58_data_interface_feedback ac58_data_interface_feedback_p_1;


ac58_TPdo* ac58_TPdo_p_2[1];
ac58_data_interface_command ac58_data_interface_command_p_2={ENCODER_ZERO_2,
    AC_ENCODER_LIMIT_UPPER_2,AC_ENCODER_LIMIT_LOWER_2};
ac58_data_interface_feedback ac58_data_interface_feedback_p_2;


sbt908_TPdo* sbt908_TPdo_p[1];
sbt908_data_interface_command sbt908_data_interface_command_p={FORCE_SENSOR_MODE,FORCE_SENSOR_LIMIT};
sbt908_data_interface_feedback sbt908_data_interface_feedback_p;

ec_slave_s ec_slave_config_s[] = {
    {
        0, 6, "CDR6", cdr6_init, cdr6_config, cdr6_pdo, cdr6_screen_log, cdr6_data_log,cdr6_raw_log,
        cdr6_safe_check,cdr6_safe_handle,cdr6_RPdo_p, cdr6_TPdo_p,
        &cdr6_data_interface_command_p, &cdr6_data_interface_feedback_p
    },
    {
        0, 1, "AC58_1", ac58_init, ac58_config,  ac58_pdo, ac58_screen_log,ac58_data_log,ac58_raw_log,
        ac58_safe_check,ac58_safe_handle,NULL, ac58_TPdo_p_1, &ac58_data_interface_command_p_1,
        &ac58_data_interface_feedback_p_1
    },
{
    0, 1, "AC58_2", ac58_init, ac58_config,  ac58_pdo, ac58_screen_log,ac58_data_log,ac58_raw_log,
    ac58_safe_check,ac58_safe_handle,NULL, ac58_TPdo_p_2, &ac58_data_interface_command_p_2,
    &ac58_data_interface_feedback_p_2
},
    {
        0, 1, "SBT908", sbt908_init, sbt908_config,  sbt908_pdo, sbt908_screen_log,sbt908_data_log,sbt908_raw_log,
        sbt908_safe_check,sbt908_safe_handle,NULL, sbt908_TPdo_p,
        &sbt908_data_interface_command_p,
        &sbt908_data_interface_feedback_p
    }
};

double* control_var[2]={cdr6_data_interface_command_p.target_position+1,cdr6_data_interface_command_p.target_position};


#endif //CONFIG_H
