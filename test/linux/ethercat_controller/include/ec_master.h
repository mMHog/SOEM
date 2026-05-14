//
// Created by ralph on 5/28/25.
//

#ifndef INTERFACE_EC_H
#define INTERFACE_EC_H

#include "config.h"
#include "alg.h"
#include "safe_check.h"
#include "ctr_log.h"

int8 buf8;
int16 buf16;
int32 buf32;

#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500

#define stack64k (64 * 1024)

struct sched_param schedp;
char IOmap[4096];
pthread_t thread1, thread2;
struct timeval tv, t1, t2;
int dorun = 0;
int deltat, tmax = 0;
int64 toff, gl_delta;
int DCdiff;
int os;
uint8 ob;
uint16 ob2;
uint8* digout = 0;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;

int all_init=0;
uint8 currentgroup = 0;

#define READ(slaveId, idx, sub, buf, comment)                                                                                                                        \
    {                                                                                                                                                                \
        buf = 0;                                                                                                                                                     \
        int __s = sizeof(buf);                                                                                                                                       \
        int __ret = ec_SDOread(slaveId, idx, sub, FALSE, &__s, &buf, EC_TIMEOUTRXM);                                                                                 \
        printf("Slave: %d - Read at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, (unsigned int)buf, comment); \
    }

#define WRITE(slaveId, idx, sub, buf, value, comment)                                                                                         \
    {                                                                                                                                         \
        int __s = sizeof(buf);                                                                                                                \
        buf = value;                                                                                                                          \
        int __ret = ec_SDOwrite(slaveId, idx, sub, FALSE, __s, &buf, EC_TIMEOUTRXM);                                                          \
        printf("Slave: %d - Write at 0x%04x:%d => wkc: %d; data: 0x%.*x\t{%s}\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, comment); \
    }

#define CHECKERROR(slaveId)                                                                                                                                                                       \
    {                                                                                                                                                                                             \
        ecx_readstate(&ecx_context);                                                                                                                                                              \
        printf("EC> \"%s\" %x - %x [%s] \n", (char *)ec_elist2string(), ec_slave[slaveId].state, ec_slave[slaveId].ALstatuscode, (char *)ec_ALstatuscode2string(ec_slave[slaveId].ALstatuscode)); \
    }

static int slave_dc_config(uint16 slave)
{
    //  ec_dcsync0(slave,   active,           cycletime,  calc and copy time)
    ec_dcsync0(slave, dcsync_enable, 4000000U, 1220000U);
    printf("ec_dcsync0 called on slave %u\n", slave);
    return 0;
}

void ec_start(char* ifname)
{
    int cnt, i, oloop, iloop;

    printf("Starting DC-sync test\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname);
        /* find and auto-config slaves */
        if (ec_config_init(FALSE) > 0) // == ec_config_init + ec_config_map
        {
            printf("%d slaves found and configured.\n", ec_slavecount);

            // PO2SOconfig is for registering a hook function that will be called when the slave does the transition
            // between Pre-OP and Safe-OP.
            if ((ec_slavecount >= 1))
            {
                for (cnt = 1; cnt <= ec_slavecount; cnt++)
                {
                    printf("Found %s at position %d\n", ec_slave[cnt].name, cnt);
                    ec_slave[cnt].PO2SOconfig = &slave_dc_config;
                }
            }

            /* Locate DC slaves, measure propagation delays. */
            ec_configdc();

            ec_config_map(&IOmap);

            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

            /* read indevidual slave state and store in ec_slave[] */
            ec_readstate();
            for (cnt = 1; cnt <= ec_slavecount; cnt++)
            {
                /* BEGIN USER CODE */

                printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d\n",
                       cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                       ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);

                /* END USER CODE */
            }
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);

            printf("Request operational state for all slaves\n");
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* request OP state for all slaves */
            ec_writestate(0);
            /* activate cyclic process data */
            dorun = 1;
            /* wait for all slaves to reach OP state */
            ec_statecheck(0, EC_STATE_OPERATIONAL, 5 * EC_TIMEOUTSTATE);
            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0))
                oloop = 1;
            if (oloop > 8)
                oloop = 8;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0))
                iloop = 1;
            if (iloop > 8)
                iloop = 8;
            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                printf("Operational state reached for all slaves.\n");
                inOP = TRUE;

                log_init();
                /* acyclic loop 5000 x 20ms = 10s */


                device_num = sizeof(ec_slave_config_s) / sizeof(ec_slave_config_s[0]);
                osal_usleep(100000);
                ec_slave_init(ec_slave_config_s);
                all_init = 1;
                printf("All Slave is Ready!\n");
                while (1)
                {
                    ec_slave_config(ec_slave_config_s);
                }
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for (i = 1; i <= ec_slavecount; i++)
                {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode,
                               ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
            printf("Request safe operational state for all slaves\n");
            ec_slave[0].state = EC_STATE_SAFE_OP;
            /* request SAFE_OP state for all slaves */
            ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End DC-sync test, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n", ifname);
    }
}

/* add ns to timespec */
void add_timespec(struct timespec* ts, int64 addtime)
{
    int64 sec, nsec;

    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;
    if (ts->tv_nsec > NSEC_PER_SEC)
    {
        nsec = ts->tv_nsec % NSEC_PER_SEC;
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
        ts->tv_nsec = nsec;
    }
}

/* PI calculation to get linux time synced to DC time */
void ec_sync(int64 reftime, int64 cycletime, int64* offsettime)
{
    static int64 integral = 0;
    int64 delta;
    /* set linux sync point 50us later than DC sync, just as example */
    delta = (reftime - 50000) % cycletime;
    if (delta > (cycletime / 2))
    {
        delta = delta - cycletime;
    }
    if (delta > 0)
    {
        integral++;
    }
    if (delta < 0)
    {
        integral--;
    }
    *offsettime = -(delta / 100) - (integral / 20);
    gl_delta = delta;
}

/* RT EtherCAT thread */
OSAL_THREAD_FUNC_RT ecatthread(void* ptr)
{
    struct timespec ts, tleft;
    int ht;
    int64 cycletime;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
    ts.tv_nsec = ht * 1000000;
    cycletime = *(int*)ptr * 1000; /* cycletime in ns */
    toff = 0;
    dorun = 0;
    int count = 0;
                                                                                                                                                                                ec_send_processdata();
    while (1)
    {
        /* calculate next cycle start */
        add_timespec(&ts, cycletime + toff);
        /* wait to cycle start */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
        /* BEGIN USER CODE */
        wkc = ec_receive_processdata(EC_TIMEOUTRET);

        if (dorun > 0)
        {
            if (all_init)
            {
                ec_slave_pdo(ec_slave_config_s);//接收pdo数据，更新硬件接口反馈信息
                alg_data_handle(&alg_ref,&alg_var);//根据硬件接口反馈，更新算法系统状态信息
                alg_control(&alg_ref,&alg_var);//根据算法状态，调用控制规划器，更新规划值
                if (alg_safe_check(&alg_ref,&alg_var)==0)
                {
                    ec_slave_commend(alg_ref.q_r,control_var,2);//给pdo赋值的核心函数
                }

                ec_log_f(ec_slave_config_s);//记录日志

                if (ec_safe_check(ec_slave_config_s)!=0)
                {
                    ec_safe_handle(ec_slave_config_s);
                }
                cycle++;
            }
            if (ec_slave[0].hasdc)
            {
                /* calulate toff to get linux time and DC synced */
                ec_sync(ec_DCtime, cycletime, &toff);
            }
            ec_send_processdata();
            /* END USER CODE */
        }
        count++;
    }
}

OSAL_THREAD_FUNC ecatcheck()
{
    int slave;

    while (1)
    {
        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                printf("\n");
            }
            /* one or more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state > EC_STATE_NONE)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n", slave);
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[slave].state == EC_STATE_NONE)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (ec_slave[slave].state == EC_STATE_NONE)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n", slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}


int ec_dc_rt_init()
{
    /* create RT thread */
    osal_thread_create_rt(&thread1, stack64k * 2, &ecatthread, (void*)&cycle_time);
    return 0;
}

int ec_check_init()
{
    /* create thread to handle slave error handling in OP */
    osal_thread_create(&thread2, stack64k * 4, &ecatcheck, NULL);
    return 0;
}


#endif //INTERFACE_EC_H
