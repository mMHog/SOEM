//
// Created by ralph on 25-5-28.
//

#ifndef EC_SLAVE_H
#define EC_SLAVE_H

typedef struct ec_slave_s
{
    int index_start;
    int slave_mun;
    char* desc;
    int (*init_fun)(struct ec_slave_s ec_slave);
    int (*config_fun)(struct ec_slave_s ec_slave);
    int (*pdo_fun)(struct ec_slave_s ec_slave);
    int (*screen_log)(struct ec_slave_s ec_slave,int mode);
    int (*data_log)(struct ec_slave_s ec_slave,FILE* log_f);
    int (*raw_log)(struct ec_slave_s ec_slave,FILE* log_f_raw);
    int (*safe_check)(struct ec_slave_s ec_slave);
    int (*safe_handle)(struct ec_slave_s ec_slave);
    void* Rpdo_p;
    void* Tpdo_p;
    void* hw_interface_command;
    void* hw_interface_feedback;
} ec_slave_s;

int slave_num = 0;

int device_num = 0;

int ec_slave_init(ec_slave_s* ec_slave_config)
{
    for (int i = 0; i < device_num; i++)
    {
        ec_slave_config[i].index_start = slave_num;
        slave_num += ec_slave_config[i].slave_mun;
        ec_slave_config[i].init_fun(ec_slave_config[i]);
    }
    if (slave_num != ec_slavecount)
    {
        printf("Slave Config ERROR!");
    }
    return 0;
}

int ec_slave_config(ec_slave_s* ec_slave_config)
{
    for (int i = 0; i < device_num; i++)
    {
        ec_slave_config[i].config_fun(ec_slave_config[i]);
    }
    return 0;
}


int ec_slave_pdo(ec_slave_s* ec_slave_config)
{
    for (int i = 0; i < device_num; i++)
    {
        ec_slave_config[i].pdo_fun(ec_slave_config[i]);
    }
    return 0;
}

int ec_slave_commend(double* alg_ref,double** hw_command_interface,int num)
{
    for (int i = 0; i < num; i++)
    {
        *(hw_command_interface[i])=alg_ref[i];
    }
    return 0;
}
#endif //EC_SLAVE_H
