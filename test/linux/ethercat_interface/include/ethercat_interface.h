/** \file
 * \brief Example code for Simple Open EtherCAT master with Distributed Clock Sync
 * 
 * Usage : dcsync_test [ifname1] [cycletime]
 * ifname is NIC interface, f.e. eth0
 * cycletime in us, f.e. 1000
 *
 * This is a script based on redundancy test to run EtherCAT master with DC Sync.
 *
 * (c)Arthur Ketels 2008
 *
 * modified by Stefano Dalla Gasperina 2020
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

#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500
#define SERVO_NUMBER 6
#define COMMAND_CONTROL 0
#define RT_CONTROL 1
#define PID_CONTROL 2

#include "ethercat.h"
#include "data_handle.h"
#include "pid.h"
//#include "rt.h"

struct sched_param schedp;
char IOmap[4096];
pthread_t thread1, thread2;
struct timeval tv, t1, t2;
int dorun = 0;
int enable = 0;
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
uint8 currentgroup = 0;
int mode=COMMAND_CONTROL;

/* Slave Distributed Clock Configuration */
boolean dcsync_enable = TRUE;
typedef struct PACKED{
    uint16 control_word;
    int32 target_position;
    int32 target_velocity;
    int16 target_torque;
    int8 op_mode;
} RPdo;
typedef struct PACKED{
    uint16 status_word;
    int32 actual_position;
    int32 actual_velocity;
    int16 actual_torque;
    int8 op_mode_display;
} TPdo;
RPdo *commend[SERVO_NUMBER];
TPdo *feedback[SERVO_NUMBER];
int cycle=0;
double target[SERVO_NUMBER]={0};
double target_pre[SERVO_NUMBER]={0};
int init_position[SERVO_NUMBER];
double outputx[SERVO_NUMBER][1000000];
int num[SERVO_NUMBER]={0},i[SERVO_NUMBER]={0},finish[SERVO_NUMBER]={0},waiting[SERVO_NUMBER]={0};
int online=0;
int rt_func();



#define READ(slaveId, idx, sub, buf, comment)    \
    {   \
        buf=0;  \
        int __s = sizeof(buf);    \
        int __ret = ec_SDOread(slaveId, idx, sub, FALSE, &__s, &buf, EC_TIMEOUTRXM);   \
        printf("Slave: %d - Read at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, (unsigned int)buf, comment);    \
     }

#define WRITE(slaveId, idx, sub, buf, value, comment) \
    {   \
        int __s = sizeof(buf);  \
        buf = value;    \
        int __ret = ec_SDOwrite(slaveId, idx, sub, FALSE, __s, &buf, EC_TIMEOUTRXM);  \
        printf("Slave: %d - Write at 0x%04x:%d => wkc: %d; data: 0x%.*x\t{%s}\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, comment);    \
    }

#define CHECKERROR(slaveId)   \
{   \
    ecx_readstate(&ecx_context);\
    printf("EC> \"%s\" %x - %x [%s] \n", (char*)ec_elist2string(), ec_slave[slaveId].state, ec_slave[slaveId].ALstatuscode, (char*)ec_ALstatuscode2string(ec_slave[slaveId].ALstatuscode));    \
}

static int slave_dc_config(uint16 slave)
{
//  ec_dcsync0(slave,   active,           cycletime,  calc and copy time)
    ec_dcsync0(slave,   dcsync_enable,    4000000U,   1220000U);
    printf("ec_dcsync0 called on slave %u\n",slave);
    return 0;
}


void redtest(char *ifname)
{
   int cnt, i, oloop, iloop;
   //int j;
   //int a;

    //uint32 buf32;
    //uint16 buf16;
    //uint8 buf8;
   //int init_position[SERVO_NUMBER];init_position[0]=0;




   printf("Starting DC-sync test\n");

   /* initialise SOEM, bind socket to ifname */
   if (ec_init(ifname))
   {
      printf("ec_init on %s succeeded.\n",ifname);
      /* find and auto-config slaves */
      if (ec_config_init(FALSE) > 0)   // == ec_config_init + ec_config_map
      {
         printf("%d slaves found and configured.\n",ec_slavecount);

         // PO2SOconfig is for registering a hook function that will be called when the slave does the transition
         // between Pre-OP and Safe-OP.
         if ((ec_slavecount >= 1)) {
             for (cnt = 1; cnt <= ec_slavecount; cnt++) {
                     printf("Found %s at position %d\n", ec_slave[cnt].name, cnt);
                     ec_slave[cnt].PO2SOconfig = &slave_dc_config;
             }
         }

         /* Locate DC slaves, measure propagation delays. */
         ec_configdc();

         ec_config_map(&IOmap);

         /* wait for all slaves to reach SAFE_OP state */
         ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);

         /* read indevidual slave state and store in ec_slave[] */
         ec_readstate();
         for(cnt = 1; cnt <= ec_slavecount ; cnt++)
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
         ec_statecheck(0, EC_STATE_OPERATIONAL,  5 * EC_TIMEOUTSTATE);
         oloop = ec_slave[0].Obytes;
         if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
         if (oloop > 8) oloop = 8;
         iloop = ec_slave[0].Ibytes;
         if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
         if (iloop > 8) iloop = 8;
         if (ec_slave[0].state == EC_STATE_OPERATIONAL )
         {
            printf("Operational state reached for all slaves.\n");
            inOP = TRUE;
            /* acyclic loop 5000 x 20ms = 10s */
            for (int i = 0; i < SERVO_NUMBER; i++)
            {
                commend[i] = (RPdo *)(ec_slave[i+1].outputs);
                feedback[i] = (TPdo *)(ec_slave[i+1].inputs);

                commend[i]->target_velocity=0;
                commend[i]->target_torque=0;
                commend[i]->op_mode=8;

                commend[i]->control_word=128;
                osal_usleep(10000);
                printf("c 128 s %d\n", feedback[i]->status_word);
                commend[i]->control_word=6;
                osal_usleep(10000);
                printf("c 6 s %d\n", feedback[i]->status_word);
                commend[i]->control_word=7;
                osal_usleep(10000);
                printf("c 7 s %d\n", feedback[i]->status_word);

                init_position[i]=feedback[i]->actual_position;
                target[i]=target_pre[i]=inc2rad(init_position[i],i+1);
                commend[i]->target_position=feedback[i]->actual_position;
                commend[i]->control_word=15;
                osal_usleep(10000);
                printf("c 15 s %d\n", feedback[i]->status_word);
                finish[i]=1;
                osal_usleep(10000);
            }
            
            printf("all to ready\n");
            enable=1;

            //double load_data[18];

            // while (enable==1)
            // {
            //    enable=1;
               
            // }

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


            // scanf("%lf %lf %lf %lf %lf %lf",&load_data[0],&load_data[1],&load_data[2],&load_data[3],&load_data[4],&load_data[5]);
            // scanf("%lf %lf %lf %lf %lf %lf",&load_data[6],&load_data[7],&load_data[8],&load_data[9],&load_data[10],&load_data[11]);
            // scanf("%lf %lf %lf %lf %lf %lf",&load_data[12],&load_data[13],&load_data[14],&load_data[15],&load_data[16],&load_data[17]);
            // for (int k = 0; k < SERVO_NUMBER; ++k)
            // {
            //   printf("%d to init\n", k);
            //   num=data_process(outputx,rad2inc(load_data[k],k+1),rad2inc(0,k+1),200,10,0.1);
            //   for (int i = 0; i < num; ++i)
            //   {
            //     commend[k]->target_position=(int)outputx[i];
            //     osal_usleep(50);
            //   }
            //   printf("%lf \n",inc2rad(feedback[k]->actual_position,k+1));
            // }




            // for (int i = 0; i < 1440048; ++i)
            // {
            //   scanf("%lf %lf %lf %lf %lf %lf",&load_data[0],&load_data[1],&load_data[2],&load_data[3],&load_data[4],&load_data[5]);
            //   scanf("%lf %lf %lf %lf %lf %lf",&load_data[6],&load_data[7],&load_data[8],&load_data[9],&load_data[10],&load_data[11]);
            //   scanf("%lf %lf %lf %lf %lf %lf",&load_data[12],&load_data[13],&load_data[14],&load_data[15],&load_data[16],&load_data[17]);
            //   commend[0]->target_position=rad2inc(load_data[0],1);
            //   commend[1]->target_position=rad2inc(load_data[1],2);
            //   commend[2]->target_position=rad2inc(load_data[2],3);
            //   commend[3]->target_position=rad2inc(load_data[3],4);
            //   commend[4]->target_position=rad2inc(load_data[4],5);
            //   commend[5]->target_position=rad2inc(load_data[5],6);
            //   commend[6]->target_position=rad2inc(load_data[6],7);
            //   commend[7]->target_position=rad2inc(load_data[7],8);
            //   commend[8]->target_position=rad2inc(load_data[8],9);
            //   commend[9]->target_position=rad2inc(load_data[9],10);
            //   commend[10]->target_position=rad2inc(load_data[10],11);
            //   commend[11]->target_position=rad2inc(load_data[11],12);
            //   commend[12]->target_position=rad2inc(load_data[12],13);
            //   commend[13]->target_position=rad2inc(load_data[13],14);
            //   commend[14]->target_position=rad2inc(load_data[14],15);
            //   commend[15]->target_position=rad2inc(load_data[15],16);
            //   commend[16]->target_position=rad2inc(load_data[16],17);
            //   commend[17]->target_position=rad2inc(load_data[17],18);

            //   /*printf("%lf ",inc2rad(feedback[0]->actual_position,1));
            //   printf("%lf ",inc2rad(feedback[1]->actual_position,2));
            //   printf("%lf ",inc2rad(feedback[2]->actual_position,3));
            //   printf("%lf ",inc2rad(feedback[3]->actual_position,4));
            //   printf("%lf ",inc2rad(feedback[4]->actual_position,5));
            //   printf("%lf ",inc2rad(feedback[5]->actual_position,6));
            //   printf("%lf ",inc2rad(feedback[6]->actual_position,7));
            //   printf("%lf ",inc2rad(feedback[7]->actual_position,8));
            //   printf("%lf ",inc2rad(feedback[8]->actual_position,9));
            //   printf("%lf ",inc2rad(feedback[9]->actual_position,10));
            //   printf("%lf ",inc2rad(feedback[10]->actual_position,11));
            //   printf("%lf ",inc2rad(feedback[11]->actual_position,12));
            //   printf("%lf ",inc2rad(feedback[12]->actual_position,13));
            //   printf("%lf ",inc2rad(feedback[13]->actual_position,14));
            //   printf("%lf ",inc2rad(feedback[14]->actual_position,15));
            //   printf("%lf ",inc2rad(feedback[15]->actual_position,16));
            //   printf("%lf ",inc2rad(feedback[16]->actual_position,17));
            //   printf("%lf\n",inc2rad(feedback[17]->actual_position,18));*/

            //  osal_usleep(50);
            // }
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
            //dorun = 0;
            //inOP = FALSE;
         }
         else
         {
            printf("Not all slaves reached operational state.\n");
             ec_readstate();
             for(i = 1; i<=ec_slavecount ; i++)
             {
                 if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                 {
                     printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                         i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                 }
             }
         }
         //printf("Request safe operational state for all slaves\n");
         //ec_slave[0].state = EC_STATE_SAFE_OP;
         /* request SAFE_OP state for all slaves */
         //ec_writestate(0);
      }
      else
      {
         printf("No slaves found!\n");
      }
      printf("End DC-sync test, close socket\n");
      /* stop SOEM, close socket */
      //ec_close();
   }
   else
   {
      printf("No socket connection on %s\nExcecute as root\n",ifname);
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
   if ( ts->tv_nsec > NSEC_PER_SEC )
   {
      nsec = ts->tv_nsec % NSEC_PER_SEC;
      ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
      ts->tv_nsec = nsec;
   }
}

/* PI calculation to get linux time synced to DC time */
void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime)
{
   static int64 integral = 0;
   int64 delta;
   /* set linux sync point 50us later than DC sync, just as example */
   delta = (reftime - 50000) % cycletime;
   if(delta> (cycletime / 2)) { delta= delta - cycletime; }
   if(delta>0){ integral++; }
   if(delta<0){ integral--; }
   *offsettime = -(delta / 100) - (integral / 20);
   gl_delta = delta;
}

/* RT EtherCAT thread */
OSAL_THREAD_FUNC_RT ecatthread(void *ptr)
{
   struct timespec   ts, tleft;
   int ht;
   int64 cycletime;

   clock_gettime(CLOCK_MONOTONIC, &ts);
   ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
   ts.tv_nsec = ht * 1000000;
   cycletime = *(int*)ptr * 1000; /* cycletime in ns */
   toff = 0;
   dorun = 0;
   ec_send_processdata();
   while(1)
   {
      /* calculate next cycle start */
      add_timespec(&ts, cycletime + toff);
      /* wait to cycle start */
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
      if (dorun>0)
      {
         /* BEGIN USER CODE */
         wkc = ec_receive_processdata(EC_TIMEOUTRET);
         
         for (size_t jj = 0; jj < SERVO_NUMBER; jj++)
         {
           if (mode==COMMAND_CONTROL)
           {
            if (enable>0 &&target[jj]!=target_pre[jj] && (online==1 || (i[jj] == num[jj])))
            {
                  target_pre[jj]=target[jj];
                  num[jj]=data_process(outputx[jj],rad2inc(target[jj],1),feedback[jj]->actual_position,20000,100,0.1);
                  i[jj]=0;finish[jj]=0;
            }
            if (enable>0 && /*cycle<=25000 &&*/ i[jj] < num[jj])
            {
               commend[jj]->target_position=(int)outputx[jj][i[jj]];i[jj]+=1;
               if(target[jj]==target_pre[jj]){finish[jj]=(i[jj] == num[jj]);}
               //printf("%d ",feedback[jj]->actual_position);
            }
           }else if (mode==RT_CONTROL)
           {
              if (enable>0)
              {
              rt_func();
              }
           }else if (mode==PID_CONTROL)
           {
              if (enable>0)
              {
            //   setpoint[0]=rad2inc(1,1);
            //   processValue[0]=feedback[0]->actual_position;
            //   PIDRegulation(processValue);
            //   commend[0]->target_position+=result[0]/1000;                 
              }
           }
         }
         dorun=1;
         /* if we have some digital output, cycle */
         if( digout ) *digout = (uint8) ((dorun / 16) & 0xff);

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

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
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
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > EC_STATE_NONE)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (ec_slave[slave].state == EC_STATE_NONE)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(ec_slave[slave].state == EC_STATE_NONE)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

#define stack64k (64 * 1024)

// int main(int argc, char *argv[])
// {
//    int ctime;

//    printf("SOEM (Simple Open EtherCAT Master)\nDC-sync test\n");

//    if (argc > 2)
//    {
//       dorun = 0;
//       ctime = atoi(argv[2]);

//       /* create RT thread */
//       osal_thread_create_rt(&thread1, stack64k * 2, &ecatthread, (void*) &ctime);

//       /* create thread to handle slave error handling in OP */
//       osal_thread_create(&thread2, stack64k * 4, &ecatcheck, NULL);

//       /* start acyclic part */
//       redtest(argv[1]);
//    }
//    else
//    {
//       printf("Usage: dcsync_test ifname cycletime\nifname = eth0 for example\ncycletime in us\n");
//    }

//    printf("End program\n");

//    return (0);
// }
int start_ethercat()
{
   int ctime;

   printf("SOEM (Simple Open EtherCAT Master)\nDC-sync test\n");

   if (1)
   {
      dorun = 0;
      ctime = 4000;

      /* create RT thread */
      osal_thread_create_rt(&thread1, stack64k * 2, &ecatthread, (void*) &ctime);

      /* create thread to handle slave error handling in OP */
      osal_thread_create(&thread2, stack64k * 4, &ecatcheck, NULL);

      /* start acyclic part */
      redtest("eno1");
   }
   else
   {
      printf("Usage: dcsync_test ifname cycletime\nifname = eth0 for example\ncycletime in us\n");
   }

   printf("End program\n");

   return (0);
}
int close_ethercat(){

    dorun = 0;
    inOP = FALSE;
    ec_slave[0].state = EC_STATE_SAFE_OP;
         /* request SAFE_OP state for all slaves */
    ec_writestate(0);
    ec_close();
   printf("EXIT\n");
    return 0;
}
int wait(){
   for (size_t jj = 0; jj < SERVO_NUMBER; jj++)
   {
    while (!finish[jj]){
        }      
   }
    return 0;
}
int single_joint_command(int jj, double rad){
    finish[jj]=0;
    target[jj]=rad;
    return 0;
}
double feed(int jj){
    return inc2rad(feedback[jj]->actual_position,jj+1);
}
double joint_command(double *commands){
   for (size_t jj = 0; jj < SERVO_NUMBER; jj++)
   {
    single_joint_command(jj,commands[jj]);      
   }
   wait();
   return 0;
}
int command_rt(double* rad){   
   for (size_t jj = 0; jj < SERVO_NUMBER; jj++)
   {
    commend[0]->target_position=rad2inc(rad[jj],jj+1);      
   }
    return 0;
}
double feedback_rt(int jj){
    return feed(jj);
}
int rt_func(){
   double commands[SERVO_NUMBER];
   for (size_t i = 0; i < 10000; i++)
   {
   for (size_t jj = 0; jj < SERVO_NUMBER; jj++)
   {
    commands[jj]=feedback_rt(jj)+inc2rad(1000,jj+1);     
   }
   command_rt(commands);
   }
   return 0;
}
