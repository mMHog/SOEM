/*
 * @Author: your name
 * @Date: 2021-04-15 13:51:33
 * @LastEditTime: 2021-05-11 16:50:53
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /SOEM/test/linux/pid_interface/inter_interface.h
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <math.h>
#include <sys/shm.h>
#include <unistd.h>
#include <string.h>
#include <sys/ipc.h>

#include "ethercat.h"
#include "data_handle.h"
#include "inter.h"

#define WELD (1<<4)
#define DRIVE (1<<6)
#define RETRO (1<<5)
#define CHECKGAS (1<<7)

#define V 54.61333
#define A 5.46133

#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500
#define SERVO_NUMBER 1
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
uint8 *digout = 0;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
//  int enable[SERVO_NUMBER] = {1};
//int enable[SERVO_NUMBER] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
// int enable[SERVO_NUMBER] = {0, 0, 0, 0, 0, 1,0, 0, 0, 0, 0, 1};
int all_enable;
int w_enable;
uint8 currentgroup = 0;

key_t key;
int shm_id;

/* Slave Distributed Clock Configuration */
boolean dcsync_enable = TRUE;
typedef struct PACKED
{
    uint16 dout;
    int16 aout1;
    int16 aout2;
} WRPdo;
typedef struct PACKED
{
    uint16 din;
    int16 ain1;
    int16 ain2;
} WTPdo;

typedef struct
{
    int control_word;
    double position_command;
    double positon_feedback;
} Transfer;

typedef struct
{
    int command;
    int feedback;
    double Icommand;
    double Ucommand;
    double Ifeedback;
    double Ufeedback;
} WTransfer;

WTransfer *wtran;
WRPdo *wcommend[1];
WTPdo *wfeedback[1];
// RPdo *commend[SERVO_NUMBER];
// TPdo *feedback[SERVO_NUMBER];
// int servo_init(int i)
// {
//     commend[i]->target_velocity = 0;
//     commend[i]->target_torque = 0;
//     commend[i]->op_mode = 8;

//     commend[i]->control_word = 128;
//     osal_usleep(100000);
//     printf("c 128 s %d\n", feedback[i]->status_word);
//     commend[i]->control_word = 6;
//     osal_usleep(100000);
//     printf("c 6 s %d\n", feedback[i]->status_word);
//     commend[i]->control_word = 7;
//     osal_usleep(100000);
//     printf("c 7 s %d\n", feedback[i]->status_word);
//     commend[i]->target_position = feedback[i]->actual_position;
//     commend[i]->control_word = 15;
//     osal_usleep(100000);
//     printf("c 15 s %d\n", feedback[i]->status_word);
//     if (feedback[i]->status_word != 34615 && feedback[i]->status_word != 49975)
//     {
//         printf("Fail to enable joint %d\n", i + 1);
//         exit(0);
//     }

//     osal_usleep(100000);
//     return feedback[i]->actual_position;
// }
// int servo_pause(int i)
// {
//     commend[i]->control_word = 7;
//     printf("c 7 s %d\n", feedback[i]->status_word);
//     return feedback[i]->actual_position;
// }
// int servo_continue(int i)
// {
//     commend[i]->target_position = feedback[i]->actual_position;
//     commend[i]->control_word = 15;
//     printf("c 15 s %d\n", feedback[i]->status_word);
//     return feedback[i]->actual_position;
// }
// int servo_stop(int i)
// {
//     commend[i]->control_word = 128;
//     printf("c 128 s %d\n", feedback[i]->status_word);
//     return feedback[i]->actual_position;
// }
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

void redtest(char *ifname)
{
    int cnt, i, oloop, iloop;
    // int init_position[SERVO_NUMBER] = {0};
    // init_position[0] += 0;

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
                /* acyclic loop 5000 x 20ms = 10s */

                key = ftok("/dev/shm/myshm345", 0);
                shm_id = shmget(key, 0x400000, IPC_CREAT | 0666);
                wtran = (WTransfer *)shmat(shm_id, NULL, 0);
                // for (int i = 0; i < SERVO_NUMBER; i++)
                // {
                //     commend[i] = (RPdo *)(ec_slave[i + 1].outputs);
                //     feedback[i] = (TPdo *)(ec_slave[i + 1].inputs);
                //     tran[i].control_word = enable[i];
                //     if (enable[i] == 1)
                //     {

                //         // init_position[i] = servo_init(i);
                //         // tran[i].position_command = inc2rad(init_position[i], i);
                //         // tran[i].positon_feedback = inc2rad(init_position[i], i);

                //         // inter_init(inc2rad(init_position[i], i), i);
                //         // double outputx[1000000];
                //         // int num;
                //         // num = data_process(outputx, rad2inc(0, i), init_position[i], 200, 10, 0.1);
                //         // for (int j = 0; j < num; ++j)
                //         // {
                //         //    commend[i]->target_position = (int)outputx[j];
                //         //    osal_usleep(50);
                //         // }
                //         // osal_usleep(1000000);

                //         //commend[i]->op_mode = 8;
                //         enable[i] = 2;
                //     }
                // }
                wcommend[0] = (WRPdo *)(ec_slave[1].outputs);
                wfeedback[0] = (WTPdo *)(ec_slave[1].inputs);
                


                // wcommend[0]->empty1=0;
                // wcommend[0]->empty2=0;
                // wcommend[0]->empty3=0;
                // wcommend[0]->empty4=0;
                // wcommend[0]->empty5=0;
                // wfeedback[0]->empty1=0;
                // wfeedback[0]->empty2=0;
                // wfeedback[0]->empty3=0;
                // wfeedback[0]->empty4=0;
                // wfeedback[0]->empty5=0;
                wtran->command=wcommend[0]->dout=0;
                wtran->feedback=wfeedback[0]->din=0;
                wtran->Icommand=wcommend[0]->aout1=0;
                wtran->Ucommand=wcommend[0]->aout2=0;
                wtran->Ifeedback=wfeedback[0]->ain1=0;
                wtran->Ufeedback=wfeedback[0]->ain2=0;

                w_enable=1;
                all_enable = 1;
                printf("all to ready\n");

                //double load_data[18];
                //double outputx[1000000];
                //int num;
                // for (int k = 0; k < SERVO_NUMBER; ++k)
                // {
                //   printf("%d to zero\n", k);
                //   num=data_process(outputx,rad2inc(0,k+1),init_position[k],200,10,0.1);
                //   for (int i = 0; i < num; ++i)
                //   {
                //     commend[k]->target_position=(int)outputx[i];
                //     osal_usleep(50);
                //   }
                // }
                // num = data_process(outputx, rad2inc(0, 0), init_position[0], 200, 10, 0.1);
                // for (int i = 0; i < num; ++i)
                // {
                //    commend[0]->target_position = (int)outputx[i];
                //    osal_usleep(50);
                // }
                // osal_usleep(1000000);
                // enable[0] = 1;
                while (1)
                {
                    // for (int i = 0; i < SERVO_NUMBER; i++)
                    // {
                    //     if (tran[i].control_word == 0 && enable[i] != 0)
                    //     {
                    //         printf("stop%d", i);
                    //         // servo_pause(i);
                    //         enable[i] = 0;
                    //     }
                    //     if (tran[i].control_word == 1 && enable[i] == 0)
                    //     {
                    //         printf("enable%d", i);
                    //         // init_position[i] = servo_init(i);
                    //         // tran[i].position_command = inc2rad(init_position[i], i);
                    //         // tran[i].positon_feedback = inc2rad(init_position[i], i);
                    //         // inter_init(inc2rad(init_position[i], i), i);
                    //         //commend[i]->op_mode = 8;
                    //         enable[i] = 2;
                    //     }
                    // }
                }

                // for(i = 1; i <= 5000; i++)
                // {
                //     /* BEGIN USER CODE */

                //    printf("Processdata cycle %5d , Wck %3d, DCtime %12ld, dt %12ld, O:",
                //       dorun, wkc , ec_DCtime, gl_delta);
                //    for(j = 0 ; j < oloop; j++)
                //    {
                //       printf(" %2.2x", *(ec_slave[0].outputs + j));
                //    }
                //    printf(" I:");
                //    for(j = 0 ; j < iloop; j++)
                //    {
                //       printf(" %2.2x", *(ec_slave[0].inputs + j));
                //    }
                //    printf("\r");
                //    fflush(stdout);
                //    osal_usleep(20000);

                //    /* END USER CODE */

                // }
                dorun = 0;
                inOP = FALSE;
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
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
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
void add_timespec(struct timespec *ts, int64 addtime)
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
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
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
OSAL_THREAD_FUNC_RT ecatthread(void *ptr)
{
    struct timespec ts, tleft;
    int ht;
    int64 cycletime;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
    ts.tv_nsec = ht * 1000000;
    cycletime = *(int *)ptr * 1000; /* cycletime in ns */
    toff = 0;
    dorun = 0;
    int count = 0;
    // double speed, c;
    //double v = 0.1 / 2;
    ec_send_processdata();
    while (1)
    {
        /* calculate next cycle start */
        add_timespec(&ts, cycletime + toff);
        /* wait to cycle start */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
        if (dorun > 0)
        {
            /* BEGIN USER CODE */
            wkc = ec_receive_processdata(EC_TIMEOUTRET);
            // for (size_t i = 0; i < SERVO_NUMBER; i++)
            // {
            //     if (all_enable == 1 && enable[i] == 2)
            //     {
            //         //inter[i].x = inc2rad(feedback[i]->actual_position, i);
            //         // tran[i].positon_feedback = inc2rad(feedback[i]->actual_position, i);
            //         // p = pid[i].ActualSpeed;
            //         // c = tran[i].position_command;
            //         // if ((c - p) >= v)
            //         // {
            //         //     c = p + v;
            //         // }
            //         // else if ((p - c) >= v)
            //         // {
            //         //     c = p - v;
            //         // }
            //         // speed = inter_realize(c, 150, i);

            //         //    speed = PID_realize(c, i);

            //         // commend[i]->target_position = rad2inc(speed, i);
            //         //commend[i]->target_velocity = 100 * (long int)(speed * 180 / 3.1415926 * incpdeg[i]);
            //         //commend[i]->target_torque = -speed*10000;

            //         commend[i]->dout=WELD|DRIVE;
            //         // commend[i]->dout|=RETRO;

            //         commend[i]->aout1=(int)0*A;
            //         commend[i]->aout2=(int)0*V;

            //         count++;
            //         printf("Success:%d Ready:%d Weld:%d Drive:%d Retro:%d Ifeedback:%.2lf Ufeedback:%.2lf Icommand:%.2lf Ucommand:%.2lf\n", feedback[i]->din&1, (feedback[i]->din&(2))>>1, (commend[i]->dout&(1<<4))>>4, (commend[i]->dout&(1<<6))>>6, (commend[i]->dout&(1<<5))>>5, feedback[i]->ain1/A, feedback[i]->ain2/V, commend[i]->aout1/A, commend[i]->aout2/V);
            //         // printf("%d %d\n", feedback[i]->ain1,feedback[i]->ain2 );
            //         // printf("%ld %lf %lf %lf\n", i + 1, inc2rad(feedback[i]->actual_position, i), c, speed);
            //     }
            // }
            // if (all_enable==1)printf("\n");
            if(all_enable == 1 && w_enable==1){
                wcommend[0]->dout=wtran->command;
                //if ((wfeedback[0]->din&(4))>>2==0) wcommend[0]->dout=0;

                wtran->feedback=wfeedback[0]->din;

                wcommend[0]->aout1=(int)wtran->Icommand*A;
                wcommend[0]->aout2=(int)wtran->Ucommand*V;

                wtran->Ifeedback=wfeedback[0]->ain1/A;
                wtran->Ufeedback=wfeedback[0]->ain2/V;

                //printf("C:%d Success:%d Ready:%d Weld:%d Drive:%d Retro:%d Ifeedback:%.2lf Ufeedback:%.2lf Icommand:%.2lf Ucommand:%.2lf\n", (wfeedback[0]->din&(4))>>2, wfeedback[0]->din&1, (wfeedback[0]->din&(2))>>1, (wcommend[0]->dout&(1<<4))>>4, (wcommend[0]->dout&(1<<6))>>6, (wcommend[0]->dout&(1<<5))>>5, wfeedback[0]->ain1/A, wfeedback[0]->ain2/V, wcommend[0]->aout1/A, wcommend[0]->aout2/V);
                printf("Command:%d Feedback:%d Ifeedback:%.2lf Ufeedback:%.2lf Icommand:%.2lf Ucommand:%.2lf\n", wcommend[0]->dout, wfeedback[0]->din, wfeedback[0]->ain1/A, wfeedback[0]->ain2/V, wcommend[0]->aout1/A, wcommend[0]->aout2/V);
            }
            count++;
            dorun = 1;
            if (ec_slave[0].hasdc)
            {
                /* calulate toff to get linux time and DC synced */
                ec_sync(ec_DCtime, cycletime, &toff);
            }
            ec_send_processdata();
            /* END USER CODE */
        }
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
            /* one ore more slaves are not responding */
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

#define stack64k (64 * 1024)

int main(int argc, char *argv[])
{
    int ctime;

    printf("SOEM (Simple Open EtherCAT Master)\nDC-sync test\n");

    if (argc > 2)
    {
        dorun = 0;
        ctime = atoi(argv[2]);

        /* create RT thread */
        osal_thread_create_rt(&thread1, stack64k * 2, &ecatthread, (void *)&ctime);

        /* create thread to handle slave error handling in OP */
        osal_thread_create(&thread2, stack64k * 4, &ecatcheck, NULL);

        /* start acyclic part */
        redtest(argv[1]);
    }
    else
    {
        printf("Usage: dcsync_test ifname cycletime\nifname = eth0 for example\ncycletime in us\n");
    }

    printf("End program\n");

    return (0);
}
