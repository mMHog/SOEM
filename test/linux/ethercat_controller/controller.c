/*
 * @Author: your name
 * @Date: 2021-04-15 13:51:33
 * @LastEditTime: 2021-05-11 16:50:53
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /SOEM/test/linux/pid_interface/inter_interface.h
 */
#include "controller.h"


int main(int argc, char* argv[])
{
    if (argc > 1)
    {
        screen_log_mode = atoi(argv[1]);
    }
    printf("SOEM (Simple Open EtherCAT Master)\nDC-sync test\n");

    ec_dc_rt_init();
    ec_check_init();
    ec_start(ifname);

    log_close();
    printf("End program\n");

    return (0);
}
