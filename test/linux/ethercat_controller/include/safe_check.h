//
// Created by root on 2026/4/28.
//

#ifndef SOEM_SAFE_CHECK_H
#define SOEM_SAFE_CHECK_H
#include "common.h"
#include "config.h"

int ec_safe_check(ec_slave_s* ec_slave_config)
{

    for (int i = 0; i < device_num; i++)
    {
        if (ec_slave_config[i].safe_check(ec_slave_config[i])!=0)
        {
            return -1;
        }
    }
    return 0;
}

int ec_safe_handle(ec_slave_s* ec_slave_config)
{

    for (int i = 0; i < device_num; i++)
    {
        ec_slave_config[i].safe_handle(ec_slave_config[i]);
    }
    exit(0);
}
#endif //SOEM_SAFE_CHECK_H