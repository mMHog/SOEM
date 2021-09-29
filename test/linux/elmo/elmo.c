#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <sys/shm.h>

#include "ethercat.h"
#include "./include/config.h"
#include "./include/inter.h"
#include "./include/pid.h"

#define EC_TIMEOUTMON 500

char IOmap[4096];  //定义映射内存区
pthread_t thread1; //创建线程
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

typedef struct
{
    int32 position;
    uint32 digital;
    uint16 status;
} PositionOut;
typedef struct
{
    int32 velocity;
    uint16 status;
} VelocityOut;
typedef struct
{
    int32 position;
    uint32 digital;
    uint16 status;
} PositionIn;

typedef struct
{
    int position_command;
    int positon_feedback;
} Transfer;

Transfer *tran;

int all_enable;
/**
 * helper macros
 */
//定义了读写检查三个宏
#define READ(slaveId, idx, sub, buf, comment)                                                                                                                        \
    {                                                                                                                                                                \
        buf = 0;                                                                                                                                                     \
        int __s = sizeof(buf);                                                                                                                                       \
        int __ret = ecx_SDOread(&ecx_context, slaveId, idx, sub, FALSE, &__s, &buf, EC_TIMEOUTRXM);                                                                  \
        printf("Slave: %d - Read at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, (unsigned int)buf, comment); \
    }

#define WRITE(slaveId, idx, sub, buf, feedbackue, comment)                                                                                    \
    {                                                                                                                                         \
        int __s = sizeof(buf);                                                                                                                \
        buf = feedbackue;                                                                                                                     \
        int __ret = ecx_SDOwrite(&ecx_context, slaveId, idx, sub, FALSE, __s, &buf, EC_TIMEOUTRXM);                                           \
        printf("Slave: %d - Write at 0x%04x:%d => wkc: %d; data: 0x%.*x\t{%s}\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, comment); \
    }

#define CHECKERROR(slaveId)                                                                                                                                                                                    \
    {                                                                                                                                                                                                          \
        ecx_readstate(&ecx_context);                                                                                                                                                                           \
        printf("EC> \"%s\" %x - %x [%s] \n", (char *)ecx_elist2string(&ecx_context), ec_slave[slaveId].state, ec_slave[slaveId].ALstatuscode, (char *)ec_ALstatuscode2string(ec_slave[slaveId].ALstatuscode)); \
    }

void simpletest(char *ifname)
{
    int i, oloop, iloop, chk;
    needlf = FALSE;
    inOP = FALSE;

    uint32 buf32;
    uint16 buf16;
    uint8 buf8;
    // struct PositionOut *command2;

    printf("Starting simple test\n");

    //初始化SOEM，绑定网口
    /* initialise SOEM, bind socket to ifname */
    if (ecx_init(&ecx_context, ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname);
        /* find and auto-config slaves */

        //发现并配置从站
        /** network discovery */
        if (ecx_config_init(&ecx_context, FALSE) > 0)
        {
            printf("%d slaves found and configured.\n", ec_slavecount);

            for (int i = 1; i <= ec_slavecount; i++)
            {
                printf("Slave %d has CA? %s\n", i, ec_slave[i].CoEdetails & ECT_COEDET_SDOCA ? "true" : "false");

                /** CompleteAccess disabled for Elmo driver */
                ec_slave[i].CoEdetails ^= ECT_COEDET_SDOCA;
            }

            ecx_statecheck(&ecx_context, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

            //设置控制器运行模式等参数
            /** set PDO mapping */
            /** opMode: 8  => Position profile */
            for (int i = 1; i <= ec_slavecount; i++)
            {
                if (MODE == 1)
                {
                    WRITE(i, 0x6060, 0, buf8, 9, "OpMode");
                }
                else
                {
                    WRITE(i, 0x6060, 0, buf8, 8, "OpMode");
                }
                READ(i, 0x6061, 0, buf8, "OpMode display");

                READ(i, 0x1c12, 0, buf32, "rxPDO:0");
                READ(i, 0x1c13, 0, buf32, "txPDO:0");

                READ(i, 0x1c12, 1, buf32, "rxPDO:1");
                READ(i, 0x1c13, 1, buf32, "txPDO:1");
            }

            int32 ob2;
            int os;
            for (int i = 1; i <= ec_slavecount; i++)
            {
                os = sizeof(ob2);
                if (MODE == 1)
                {
                    ob2 = 0x16010001;
                }
                else
                {
                    ob2 = 0x16000001;
                }
                ecx_SDOwrite(&ecx_context, i, 0x1c12, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);
                os = sizeof(ob2);
                ob2 = 0x1a000001;
                ecx_SDOwrite(&ecx_context, i, 0x1c13, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);

                READ(i, 0x1c12, 0, buf32, "rxPDO:0");
                READ(i, 0x1c13, 0, buf32, "txPDO:0");

                READ(i, 0x1c12, 1, buf32, "rxPDO:1");
                READ(i, 0x1c13, 1, buf32, "txPDO:1");
            }

            //输入输出物理映射，group=0代表所有
            /** if CA disable => automapping works */
            ecx_config_map_group(&ecx_context, &IOmap, 0);

            key_t key = ftok("/dev/shm/myshm344", 0);
            int shm_id = shmget(key, 0x400000, IPC_CREAT | 0666);
            tran = (Transfer *)shmat(shm_id, NULL, 0);

            //定义输入输出指针
            PositionOut *command[ec_slavecount];
            VelocityOut *command_v[ec_slavecount];
            PositionIn *feedback[ec_slavecount];
            int cache[ec_slavecount];
            int init_position[ec_slavecount];
            for (int i = 1; i <= ec_slavecount; i++)
            {
                /* cyclic loop for two slaves*/
                os = sizeof(ob2);
                feedback[i - 1] = (PositionIn *)(ec_slave[i].inputs);
                READ(i, 0x6064, 0, buf32, "rxPDO:0");
                if (MODE == 1)
                {
                    command_v[i - 1] = (VelocityOut *)(ec_slave[i].outputs);
                    command_v[i - 1]->velocity = 0;
                    init_position[i - 1] = buf32;
                }
                else
                {
                    command[i - 1] = (PositionOut *)(ec_slave[i].outputs);
                    init_position[i - 1] = command[i - 1]->position = buf32;
                }
                cache[i - 1] = tran[i - 1].position_command = tran[i - 1].positon_feedback = buf32;
                // printf("test: %d %d %d %d\n", init_position[i - 1], command[i - 1]->position, tran[i - 1].position_command, tran[i - 1].positon_feedback);
                if (MODE == 0)
                {
                    PID_init(init_position[i - 1], i - 1,0);
                }
                else if (MODE == 1)
                {
                    PID_init(init_position[i - 1], i - 1,1);
                }
                else if (MODE == 2)
                {
                    inter_init(init_position[i - 1], i - 1);
                }
                // usleep(100000);
            }

            // show slave info
            for (int i = 1; i <= ec_slavecount; i++)
            {
                printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
                       i, ec_slave[i].name, ec_slave[i].Obits, ec_slave[i].Ibits,
                       ec_slave[i].state, ec_slave[i].pdelay, ec_slave[i].hasdc);
            }

            /** disable heartbeat alarm */
            for (int i = 1; i <= ec_slavecount; i++)
            {
                READ(i, 0x10F1, 2, buf32, "Heartbeat?");
                WRITE(i, 0x10F1, 2, buf32, 1, "Heartbeat");

                WRITE(i, 0x60c2, 1, buf8, 2, "Time period");
                WRITE(i, 0x2f75, 0, buf16, 2, "Interpolation timeout");
            }

            printf("Slaves mapped, state to SAFE_OP.\n");

            /* wait for all slaves to reach SAFE_OP state */
            ecx_statecheck(&ecx_context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

            /** old SOEM code, inactive */
            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0))
                oloop = 1;
            if (oloop > 20)
                oloop = 8;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0))
                iloop = 1;
            if (iloop > 20)
                iloop = 8;

            printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

            printf("Request operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);

            /** going operational */
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one feedbackid process data to make outputs in slaves happy*/
            ecx_send_processdata(&ecx_context);
            ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);

            for (int i = 1; i <= ec_slavecount; i++)
            {
                READ(i, 0x6083, 0, buf32, "Profile acceleration");
                READ(i, 0x6084, 0, buf32, "Profile deceleration");
                READ(i, 0x6085, 0, buf32, "Quick stop deceleration");
            }

            /* request OP state for all slaves */
            ecx_writestate(&ecx_context, 0);
            chk = 40;
            /* wait for all slaves to reach OP state */
            do
            {
                ecx_send_processdata(&ecx_context);
                ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
                ecx_statecheck(&ecx_context, 0, EC_STATE_OPERATIONAL, 50000);
                // for (int i = 1; i <= ec_slavecount; i++)
                // {
                //     command[i - 1]->position = feedback[i - 1]->position;
                // }
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                printf("Operational state reached for all slaves.\n");
                // wkc_count = 0;
                inOP = TRUE;

                /**
                 * Drive state machine transistions
                 *   0 -> 6 -> 7 -> 15
                 */

                for (int i = 1; i <= ec_slavecount; i++)
                {
                    READ(i, 0x6041, 0, buf16, "*status word*");
                    if (buf16 == 0x218)
                    {
                        WRITE(i, 0x6040, 0, buf16, 128, "*control word*");
                        usleep(100000);
                        READ(i, 0x6041, 0, buf16, "*status word*");
                    }

                    WRITE(i, 0x6040, 0, buf16, 0, "*control word*");
                    usleep(100000);
                    READ(i, 0x6041, 0, buf16, "*status word*");

                    WRITE(i, 0x6040, 0, buf16, 6, "*control word*");
                    usleep(100000);
                    READ(i, 0x6041, 0, buf16, "*status word*");

                    WRITE(i, 0x6040, 0, buf16, 7, "*control word*");
                    usleep(100000);
                    READ(i, 0x6041, 0, buf16, "*status word*");

                    WRITE(i, 0x6040, 0, buf16, 15, "*control word*");
                    usleep(100000);
                    READ(i, 0x6041, 0, buf16, "*status word*");

                    CHECKERROR(i);
                    READ(i, 0x1a0b, 0, buf8, "OpMode Display");

                    READ(i, 0x1001, 0, buf8, "Error");
                }
                int reachedInitial = 0;
                double speed, c;
                int timestep = 400;
                while (1)
                {
                    for (int i = 1; i <= ec_slavecount; i++)
                    {
                        ecx_send_processdata(&ecx_context);
                        wkc = ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);

                        if (wkc >= expectedWKC)
                        {
                            if (MODE == 1)
                            {
                                switch (command_v[i - 1]->status)
                                {
                                case 0:
                                    command_v[i - 1]->status = 6;
                                    break;
                                case 6:
                                    command_v[i - 1]->status = 7;
                                    break;
                                case 7:
                                    command_v[i - 1]->status = 15;
                                    break;
                                case 128:
                                    command_v[i - 1]->status = 0;
                                    break;
                                default:
                                    if (feedback[i - 1]->status >> 3 & 0x01)
                                    {
                                        READ(1, 0x1001, 0, buf8, "Error");
                                        command_v[i - 1]->status = 128;
                                        reachedInitial = 0;
                                    }
                                }
                            }
                            else
                            {
                                switch (command[i - 1]->status)
                                {
                                case 0:
                                    command[i - 1]->status = 6;
                                    break;
                                case 6:
                                    command[i - 1]->status = 7;
                                    break;
                                case 7:
                                    command[i - 1]->status = 15;
                                    break;
                                case 128:
                                    command[i - 1]->status = 0;
                                    break;
                                default:
                                    if (feedback[i - 1]->status >> 3 & 0x01)
                                    {
                                        READ(1, 0x1001, 0, buf8, "Error");
                                        command[i - 1]->status = 128;
                                        reachedInitial = 0;
                                    }
                                }
                            }
                            /** we wait to be in ready-to-run mode and with initial feedbackue reached */
                            if (reachedInitial == 0 && (feedback[i - 1]->status & 0x0fff) == 0x0237)
                            {
                                reachedInitial = 1;
                            }

                            if ((feedback[i - 1]->status & 0x0fff) == 0x0237 && reachedInitial)
                            {
                                tran[i - 1].positon_feedback = feedback[i - 1]->position;
                                if (tran[i - 1].position_command < max[i - 1] && tran[i - 1].position_command > min[i - 1])
                                {
                                    cache[i - 1] = c = tran[i - 1].position_command;
                                }
                                else
                                {
                                    tran[i - 1].position_command = c = cache[i - 1];
                                }
                                if (MODE == 0)
                                {
                                    speed = PID_realize(c, feedback[i - 1]->position, i - 1, 0);
                                    command[i - 1]->position = (int)speed;
                                }
                                else if (MODE == 1)
                                {
                                    speed = PID_realize(c, feedback[i - 1]->position, i - 1, 1);
                                    command_v[i - 1]->velocity = (int)speed;
                                }
                                else if (MODE == 2)
                                {
                                    speed = inter_realize(c, step_num, i - 1);
                                    command[i - 1]->position = (int)speed;
                                }
                                printf("index: %d feed: %d targrt: %d motion: %d\n", i, feedback[i - 1]->position, (int)c, (int)speed);
                            }
                            needlf = TRUE;
                        }
                        usleep(timestep);
                    }
                    // printf("\n");
                }
                inOP = FALSE;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ecx_readstate(&ecx_context);
                for (i = 1; i <= ec_slavecount; i++)
                {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }

            printf("\nRequest init state for all slaves\n");
            for (int i = 1; i <= ec_slavecount; i++)
            {
                WRITE(i, 0x10F1, 2, buf32, 0, "Heartbeat");
            }

            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            ecx_writestate(&ecx_context, 0);
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End simple test, close socket\n");
        /* stop SOEM, close socket */
        ecx_close(&ecx_context);
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n", ifname);
    }
}

void *ecatcheck()
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
            ecx_readstate(&ecx_context);
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ecx_writestate(&ecx_context, slave);
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ecx_writestate(&ecx_context, slave);
                    }
                    else if (ec_slave[slave].state > 0)
                    {
                        if (ecx_reconfig_slave(&ecx_context, slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n", slave);
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ecx_statecheck(&ecx_context, slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (!ec_slave[slave].state)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (!ec_slave[slave].state)
                    {
                        if (ecx_recover_slave(&ecx_context, slave, EC_TIMEOUTMON))
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
                printf(".");
        }
        usleep(250);
    }
}

int main(int argc, char *argv[])
{
    // int iret1;
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

    if (argc > 1)
    {
        /* create thread to handle slave error handling in OP */
        pthread_create(&thread1, NULL, &ecatcheck, (void(*)) & ctime); // (void) &ctime
        /* start cyclic part */
        simpletest(argv[1]);
    }
    else
    {
        printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
    }

    printf("End program\n");
    return (0);
}
