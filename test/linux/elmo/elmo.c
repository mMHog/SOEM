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
#define NSEC_PER_SEC 1000000000

pthread_t thread1; //创建线程
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
FILE *fp;
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
    uint16 torque;
    uint16 status;
} TorqueOut;
typedef struct
{
    int32 position;
    uint32 digital;
    uint16 status;
} PositionIn;

typedef struct
{
    int position_command;
    int velocity_command;
    int torque_command;
    int position_feedback;
    int velocity_feedback;
    int torque_feedback;
    int position_enable;
    int operation_mode;
    int operation_mode_display;
} Transfer;

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
static int slave_dc_config(uint16 slave)
{
    //  ec_dcsync0(slave,   active,           cycletime,  calc and copy time)
    ec_dcsync0(slave, TRUE, step * 1000 * 1000000, 0.3 * step * 1000 * 1000000);
    printf("ec_dcsync0 called on slave %u\n", slave);
    usleep(100000);
    return 0;
}
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
void simpletest(char *ifname)
{
    int i, oloop, iloop, chk;
    needlf = FALSE;
    inOP = FALSE;
    Transfer *tran;
    uint32 buf32;
    uint16 buf16;
    uint8 buf8;
    char IOmap[4096]; //定义映射内存区
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
                ec_slave[i].PO2SOconfig = &slave_dc_config;
            }

            ecx_statecheck(&ecx_context, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
            //设置控制器运行模式等参数
            /** set PDO mapping */
            /** opMode: 8  => Position profile */
            for (int i = 1; i <= ec_slavecount; i++)
            {
                if (op_mode[MODE[i - 1]] == 0)
                {
                    WRITE(i, 0x6060, 0, buf8, 8, "OpMode");
                }
                else if (op_mode[MODE[i - 1]] == 1)
                {
                    WRITE(i, 0x6060, 0, buf8, 9, "OpMode");
                }
                else if (op_mode[MODE[i - 1]] == 2)
                {
                    WRITE(i, 0x6060, 0, buf8, 10, "OpMode");
                }
                usleep(50000);
                WRITE(i, 0x6007, 0, buf16, 0, "Quick Stop Disabled");
                usleep(50000);
            }

            int32 ob2;
            int os;
            for (int i = 1; i <= ec_slavecount; i++)
            {
                os = sizeof(ob2);
                if (op_mode[MODE[i - 1]] == 0)
                {
                    ob2 = 0x16000001;
                }
                else if (op_mode[MODE[i - 1]] == 1)
                {
                    ob2 = 0x16010001;
                }
                else if (op_mode[MODE[i - 1]] == 2)
                {
                    ob2 = 0x16020001;
                }
                printf("Config PDO of index %d\n",i);
                ecx_SDOwrite(&ecx_context, i, 0x1c12, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);
                usleep(50000);
                os = sizeof(ob2);
                ob2 = 0x1a000001;
                ecx_SDOwrite(&ecx_context, i, 0x1c13, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);
                usleep(50000);
            }

            //输入输出物理映射，group=0代表所有
            /** if CA disable => automapping works */
            ecx_config_map_group(&ecx_context, &IOmap, 0);
            ec_configdc();

            key_t key = ftok("/dev/shm/myshm344", 0);
            int shm_id = shmget(key, 0x400000, IPC_CREAT | 0666);
            tran = (Transfer *)shmat(shm_id, NULL, 0);

            //定义输入输出指针
            PositionOut *command_p[ec_slavecount];
            VelocityOut *command_v[ec_slavecount];
            TorqueOut *command_t[ec_slavecount];
            PositionIn *feedback[ec_slavecount];
            int cache_p[ec_slavecount];
            int cache_v[ec_slavecount];
            int cache_t[ec_slavecount];
            int init_position[ec_slavecount];
            double plan, p[ec_slavecount], v[ec_slavecount], t[ec_slavecount];
            int num_left[ec_slavecount];
            for (int i = 1; i <= ec_slavecount; i++)
            {
                /* cyclic loop for two slaves*/
                os = sizeof(ob2);
                feedback[i - 1] = (PositionIn *)(ec_slave[i].inputs);
                READ(i, 0x6064, 0, buf32, "rxPDO:0");
                if (op_mode[MODE[i - 1]] == 0)
                {
                    command_p[i - 1] = (PositionOut *)(ec_slave[i].outputs);
                    init_position[i - 1] = command_p[i - 1]->position = buf32;
                }
                else if (op_mode[MODE[i - 1]] == 1)
                {
                    command_v[i - 1] = (VelocityOut *)(ec_slave[i].outputs);
                    command_v[i - 1]->velocity = 0;
                    init_position[i - 1] = buf32;
                }
                else if (op_mode[MODE[i - 1]] == 2)
                {
                    command_t[i - 1] = (TorqueOut *)(ec_slave[i].outputs);
                    command_t[i - 1]->torque = 0;
                    init_position[i - 1] = buf32;
                }
                p[i - 1] = cache_p[i - 1] = tran[i - 1].position_command = tran[i - 1].position_feedback = buf32;
                v[i - 1] = cache_v[i - 1] = tran[i - 1].velocity_command = tran[i - 1].velocity_feedback = 0;
                t[i - 1] = cache_v[i - 1] = tran[i - 1].torque_command = tran[i - 1].torque_feedback = 0;
                // tran[i - 1].operation_mode = -1;
                tran[i - 1].position_enable = 0;
                // tran[i - 1].operation_mode_display = op_mode[MODE[i-1]];
                num_left[i - 1] = 0;
                // tran[i - 1].motion_mode = op_mode[MODE[i-1]];
                // init_func[MODE[i-1]](init_position[i - 1], i - 1);
                // printf("test: %d %d %d %d\n", init_position[i - 1], command_p[i - 1]->position, tran[i - 1].position_command, tran[i - 1].position_feedback);
                if (MODE[i - 1] == 0)
                {
                    PID_init(init_position[i - 1], i - 1, 0);
                }
                else if (MODE[i - 1] == 1)
                {
                    PID_init(init_position[i - 1], i - 1, 1);
                }
                else if (MODE[i - 1] == 2)
                {
                    inter_init(init_position[i - 1], i - 1);
                }
                else if (MODE[i - 1] == 3)
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
            //for (int i = 1; i <= ec_slavecount; i++)
            //{
                // READ(i, 0x10F1, 2, buf32, "Heartbeat?");
                // WRITE(i, 0x10F1, 2, buf32, 1, "Heartbeat");

                //WRITE(i, 0x60c2, 1, buf8, 2, "Time period");
                //usleep(50000);
                //WRITE(i, 0x2f75, 0, buf16, 2, "Interpolation timeout");
                //usleep(50000);
            //}

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

            //for (int i = 1; i <= ec_slavecount; i++)
            //{
                // WRITE(i, 0x6081, 0, buf32, 10000, "Time period");
                //WRITE(i, 0x6081, 0, buf32, 50000, "Velocity");
                //WRITE(i, 0x6083, 0, buf32, 1000, "Profile acceleration");
                //WRITE(i, 0x6084, 0, buf32, 1000, "Profile deceleration");
                // WRITE(i, 0x6084, 0, buf32, 100000, "Quick stop deceleration");
                // WRITE(i, 0x6085, 0, buf32, 1000000, "Time period");
                //READ(i, 0x6083, 0, buf32, "Profile acceleration");
                //READ(i, 0x6084, 0, buf32, "Profile deceleration");
                //READ(i, 0x6085, 0, buf32, "Quick stop deceleration");
            //}

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
                //     command_p[i - 1]->position = feedback[i - 1]->position;
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

                int reachedInitial[ec_slavecount];

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
                    tran[i - 1].operation_mode = -1;
                    reachedInitial[i - 1]=0;
                }
                int timestep = step * 1000000;
                // int count=0;
                struct timespec ts0, ts;
                clock_gettime(CLOCK_MONOTONIC, &ts0);
                // int64 time_last;
                int count=0;
                int cycle=0;
                int op_change_num = 0;
                char op_des[3]={'p','v','t'};
                fprintf(fp,"cycle\tindex\tservo_sts\tservo_ctrl\tcheck_sts\top_mode\tpos_feedback\tpos_command\tvel_command\ttor_command\tmotion_type\tmotion_plan\tt_lft\tinter_x\tinter_v\tinter_a\n");
                while (1)
                {
                    wkc = ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
                    printf("%ld\n",ec_DCtime);
                    printf(" c:%-13ds_sts\ts_ctrl\tck_sts\top_md\tpos_feedback\tpos_command\tvel_command\ttor_command\tmotion_plan\tt_lft(ms)\tinter_x\t\tinter_v\t\tinter_a\n",cycle++);
                    
                    if (op_change_num == ec_slavecount)
                    {
                        break;
                    }
                    op_change_num = 0;
                    for (int i = 1; i <= ec_slavecount; i++)
                    {
                        printf(" index: %d\t",i);
                        if (tran[i - 1].operation_mode != -1)
                        {
                            tran[i - 1].operation_mode_display=tran[i - 1].operation_mode;
                            op_change_num++;
                        }

                        if (wkc >= expectedWKC)
                        {
                            fprintf(fp,"%d\t%d\t",cycle,i);
                            printf("%X\t",feedback[i - 1]->status);
                            fprintf(fp,"%X\t",feedback[i - 1]->status);
                            if (op_mode[MODE[i - 1]] == 0)
                            {
                                printf("%d\t",command_p[i - 1]->status);
                                fprintf(fp,"%d\t",command_p[i - 1]->status);
                                switch (command_p[i - 1]->status)
                                {
                                case 0:
                                    if ((feedback[i - 1]->status & 0x0fff) == 0x0250) command_p[i - 1]->status = 6;
                                    break;
                                case 6:
                                    if ((feedback[i - 1]->status & 0x0fff) == 0x0231) command_p[i - 1]->status = 7;
                                    break;
                                case 7:
                                    if ((feedback[i - 1]->status & 0x0fff) == 0x0233) command_p[i - 1]->status = 15;
                                    break;
                                case 128:
                                    if ((feedback[i - 1]->status >> 3 & 0x01) == 0) command_p[i - 1]->status = 0;
                                    break;
                                default:
                                    if ((feedback[i - 1]->status >> 5 & 0x01)==0)
                                    {                                        
                                      command_p[i - 1]->status = 6;
                                    }
                                    if (feedback[i - 1]->status >> 3 & 0x01)
                                    {
                                        READ(1, 0x1001, 0, buf8, "Error");
                                        command_p[i - 1]->status = 128;
                                        reachedInitial[i - 1] = 0;
                                    }
                                }
                            }
                            else if (op_mode[MODE[i - 1]] == 1)
                            {
                                printf("%d\t",command_v[i - 1]->status);
                                fprintf(fp,"%d\t",command_v[i - 1]->status);
                                switch (command_v[i - 1]->status)
                                {
                                case 0:
                                    if ((feedback[i - 1]->status & 0x0fff) == 0x0250) command_v[i - 1]->status = 6;
                                    break;
                                case 6:
                                    if ((feedback[i - 1]->status & 0x0fff) == 0x0231) command_v[i - 1]->status = 7;
                                    break;
                                case 7:
                                    if ((feedback[i - 1]->status & 0x0fff) == 0x0233) command_v[i - 1]->status = 15;
                                    break;
                                case 128:
                                    if ((feedback[i - 1]->status >> 3 & 0x01) == 0) command_v[i - 1]->status = 0;
                                    break;
                                default:
                                    if ((feedback[i - 1]->status >> 5 & 0x01)==0)
                                    {                                        
                                      command_v[i - 1]->status = 6;
                                    }
                                    if (feedback[i - 1]->status >> 3 & 0x01)
                                    {
                                        READ(1, 0x1001, 0, buf8, "Error");
                                        command_v[i - 1]->status = 128;
                                        reachedInitial[i - 1] = 0;
                                    }
                                }
                            }
                            else if (op_mode[MODE[i - 1]] == 2)
                            {
                                printf("%d\t",command_t[i - 1]->status);
                                fprintf(fp,"%d\t",command_t[i - 1]->status);
                                switch (command_t[i - 1]->status)
                                {
                                case 0:
                                    if ((feedback[i - 1]->status & 0x0fff) == 0x0250) command_t[i - 1]->status = 6;
                                    break;
                                case 6:
                                    if ((feedback[i - 1]->status & 0x0fff) == 0x0231) command_t[i - 1]->status = 7;
                                    break;
                                case 7:
                                    if ((feedback[i - 1]->status & 0x0fff) == 0x0233) command_t[i - 1]->status = 15;
                                    break;
                                case 128:
                                    if ((feedback[i - 1]->status >> 3 & 0x01) == 0) command_t[i - 1]->status = 0;
                                    break;
                                default:
                                    if ((feedback[i - 1]->status >> 5 & 0x01)==0)
                                    {                                        
                                      command_t[i - 1]->status = 6;
                                    }
                                    if (feedback[i - 1]->status >> 3 & 0x01)
                                    {
                                        READ(1, 0x1001, 0, buf8, "Error");
                                        command_t[i - 1]->status = 128;
                                        reachedInitial[i - 1] = 0;
                                    }
                                }
                            }
                            /** we wait to be in ready-to-run mode and with initial feedbackue reached */
                            if (reachedInitial[i - 1] == 0 && (feedback[i - 1]->status & 0x0fff) == 0x0237)
                            {
                                reachedInitial[i - 1] = 1;
                            }

                            if ((feedback[i - 1]->status & 0x0fff) == 0x0237 && reachedInitial[i - 1])
                            {
                                printf("ok\t");
                                fprintf(fp,"ok\t");
                                tran[i - 1].position_feedback = feedback[i - 1]->position;
                                if (op_mode[MODE[i - 1]] == 0 || MODE[i-1]==3)
                                {
                                    if (tran[i - 1].position_enable && (tran[i - 1].position_command != cache_p[i - 1] || tran[i - 1].position_command != cache_p[i - 1]))
                                    {
                                        if (tran[i - 1].position_command < max_p[i - 1] && tran[i - 1].position_command > min_p[i - 1] && tran[i - 1].velocity_command < max_v[i - 1] && tran[i - 1].velocity_command > min_v[i - 1])
                                        {
                                            cache_p[i - 1] = p[i - 1] = tran[i - 1].position_command;
                                            cache_v[i - 1] = v[i - 1] = tran[i - 1].velocity_command;
                                            if (tran[i - 1].velocity_command < 0)
                                                tran[i - 1].velocity_command = -tran[i - 1].velocity_command;
                                            if (tran[i - 1].velocity_command == 0)
                                                num_left[i - 1] = 0;
                                            else
                                                num_left[i - 1] = abs(((1.0*(tran[i - 1].position_command - feedback[i - 1]->position)) / (step * tran[i - 1].velocity_command)));
                                                //num_left[i - 1] = 2;
                                        }
                                        else
                                        {
                                            tran[i - 1].position_command = p[i - 1] = cache_p[i - 1];
                                            tran[i - 1].velocity_command = v[i - 1] = cache_v[i - 1];
                                        }
                                        tran[i - 1].position_enable = 0;
                                    }
                                    //c = p[i - 1];
                                }
                                if (op_mode[MODE[i - 1]] == 1 && MODE[i-1]!=3)
                                {
                                    if (tran[i - 1].velocity_command < max_v[i - 1] && tran[i - 1].velocity_command > min_v[i - 1])
                                        cache_v[i - 1] = v[i - 1] = tran[i - 1].velocity_command;
                                    else
                                        tran[i - 1].velocity_command = v[i - 1] = cache_v[i - 1];
                                    //c = v[i - 1];
                                }
                                if (op_mode[MODE[i - 1]] == 2)
                                {
                                    if (tran[i - 1].torque_command < max_t[i - 1] && tran[i - 1].torque_command > min_t[i - 1])
                                        cache_t[i - 1] = t[i - 1] = tran[i - 1].torque_command;
                                    else
                                        tran[i - 1].torque_command = t[i - 1] = cache_t[i - 1];
                                    //c = t[i - 1];
                                }
                                if (MODE[i - 1] == 0)
                                {
                                    plan = PID_realize(p[i - 1], feedback[i - 1]->position, i - 1, 0);
                                    command_p[i - 1]->position = (int)plan;
                                    //c = p[i - 1];
                                }
                                else if (MODE[i - 1] == 1)
                                {
                                    plan = PID_realize(p[i - 1], feedback[i - 1]->position, i - 1, 1);
                                    command_v[i - 1]->velocity = (int)plan;
                                    //c = p[i - 1];
                                }
                                else if (MODE[i - 1] == 2)
                                {
                                    plan = inter_realize(p[i - 1], num_left[i - 1], i - 1, feedback[i - 1]->position, 0);
                                    if (num_left[i - 1] > 0)
                                        num_left[i - 1]--;
                                    command_p[i - 1]->position = (int)plan;
                                    // command_p[i - 1]->position=100*(int32)(sin((count++) / 100.) * (100000));
                                }
                                else if (MODE[i - 1] == 3)
                                {
                                    plan = inter_realize(p[i - 1], num_left[i - 1], i - 1, feedback[i - 1]->position, 2);//moshi 1:duoxiangshi 2:xianxing
                                    command_v[i - 1]->velocity = (int)plan;
                                    if (num_left[i - 1] > 0)
                                        num_left[i - 1]--;
                                    //c = p[i - 1];
                                    // command_p[i - 1]->position=100*(int32)(sin((count++) / 100.) * (100000));
                                }
                                else if (MODE[i - 1] == 4)
                                {
                                    plan = v[i - 1];
                                    command_v[i - 1]->velocity = (int)plan;
                                    // command_p[i - 1]->position=100*(int32)(sin((count++) / 100.) * (100000));
                                }
                                else if (MODE[i - 1] == 5)
                                {
                                    plan = t[i - 1];
                                    command_t[i - 1]->torque = (int16)plan;
                                    // command_p[i - 1]->position=100*(int32)(sin((count++) / 100.) * (100000));
                                }
                                // printf("index: %d feed: %d targrt: %d motion: %d\n", i, feedback[i - 1]->position, (int)c, (int)plan);

                                printf("%c\t%-16d",op_des[tran[i - 1].operation_mode_display], feedback[i - 1]->position);
                                printf("%-16d%-16d%-16d",tran[i - 1].position_command, tran[i - 1].velocity_command, tran[i - 1].torque_command);
                                printf("%c:%-14d%-16d%-16d%-16d%-16d\n",op_des[op_mode[MODE[i - 1]]],(int)plan,(int)(num_left[i - 1]*(step*1000)),(int)inter[i-1].x,(int)inter[i-1].v,(int)inter[i-1].a);
                                fprintf(fp,"%d\t%d\t",MODE[i - 1], feedback[i - 1]->position);
                                fprintf(fp,"%d\t%d\t%d\t",tran[i - 1].position_command, tran[i - 1].velocity_command, tran[i - 1].torque_command);
                                fprintf(fp,"%c\t%d\t%d\t%d\t%d\t%d\n",op_des[op_mode[MODE[i - 1]]],(int)plan,(int)(num_left[i - 1]),(int)inter[i-1].x,(int)inter[i-1].v,(int)inter[i-1].a);
                                // time_last = ec_DCtime;
                            }
                            else 
                            {
                                printf("fault, please ctrl-c\n");
                                fprintf(fp,"fault\n");
                            }
                            needlf = TRUE;
                        }

                        // usleep(timestep);
                    }
                    ecx_send_processdata(&ecx_context);
                    add_timespec(&ts0, timestep * 1000);
                    clock_gettime(CLOCK_MONOTONIC, &ts);
                    if ((ts.tv_sec == ts0.tv_sec && ts.tv_nsec >= ts0.tv_nsec) || ts.tv_sec > ts0.tv_sec) printf("cycle too short, cycle lost\n");
                    while (1)
                    {
                        clock_gettime(CLOCK_MONOTONIC, &ts);
                        if ((ts.tv_sec == ts0.tv_sec && ts.tv_nsec >= ts0.tv_nsec) || ts.tv_sec > ts0.tv_sec){
                            break;
                        }
                    }
                    if (reachedInitial == 0)
                    {
                        count++;
                    }
                    if (count > to / step)
                    {
                        break;
                    }
                    printf("\n");
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
                // WRITE(i, 0x10F1, 2, buf32, 0, "Heartbeat");
            }

            // ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            // ecx_writestate(&ecx_context, 0);
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End simple test, close socket\n");
        /* stop SOEM, close socket */
        // ecx_close(&ecx_context);
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
        usleep(10000);
    }
}

int main(int argc, char *argv[])
{
    time_t rawtime;
    time(&rawtime);
    char str[128];
    struct tm *tinfo=localtime(&rawtime);
    strftime(str,sizeof(str),"/home/liwenshuo/hw/SOEMv7/log/elmo-%Y-%m-%d-%H:%M:%S.log",tinfo);
    // int iret1;
    fp=fopen(str, "a");
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");
    if (argc > 1)
    {
        /* create thread to handle slave error handling in OP */
        pthread_create(&thread1, NULL, &ecatcheck, (void(*)) & ctime); // (void) &ctime
        /* start cyclic part */
        for (int i = 0; i < argc - 2; i++)
        {
            MODE[i] = default_mode[atoi(argv[2 + i])];
        }
        simpletest(argv[1]);
    }
    else
    {
        printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
    }
    fclose(fp);
    printf("End program\n");
    return (0);
}
