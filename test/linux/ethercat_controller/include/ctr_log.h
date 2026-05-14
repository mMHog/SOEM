//
// Created by ralph on 5/28/25.
//

#ifndef CTR_LOG_H
#define CTR_LOG_H

#include "config.h"
#include "alg.h"
#include <stdio.h>
#include <stdarg.h>
#include <time.h>

FILE* log_fp;
FILE* log_fp_raw;
FILE* log_fp_event;
FILE* log_fp_var;

int log_status=0;

int log_init()
{
    time_t rawtime;
    time(&rawtime);
    char str[128];
    struct tm* tinfo = localtime(&rawtime);
    if (data_log_on==1)
    {
        strftime(str, sizeof(str), "/home/ralph/SOEM/log/%Y-%m-%d-%H:%M:%S.log", tinfo);
        log_fp = fopen(str, "a");
    }
    if (raw_log_on==1)
    {
        strftime(str, sizeof(str), "/home/ralph/SOEM/log/%Y-%m-%d-%H:%M:%S-raw.log", tinfo);
        log_fp_raw = fopen(str, "a");
    }
    if (event_log_on==1)
    {
        strftime(str, sizeof(str), "/home/ralph/SOEM/log/%Y-%m-%d-%H:%M:%S-event.log", tinfo);
        log_fp_event = fopen(str, "a");
    }
    if (var_log_on==1)
    {
        strftime(str, sizeof(str), "/home/ralph/SOEM/log/%Y-%m-%d-%H:%M:%S-var.log", tinfo);
        log_fp_var = fopen(str, "a");
    }
    return 0;
}

int log_close()
{
    if (log_fp!=NULL)
    {
        fclose(log_fp);
    }
    if (log_fp_raw!=NULL)
    {
        fclose(log_fp_raw);
    }
    if (log_fp_event!=NULL)
    {
        fclose(log_fp_event);
    }
    if (log_fp_var!=NULL)
    {
        fclose(log_fp_var);
    }
    return 0;
}


int ec_log_f(ec_slave_s* ec_slave_config)
{
    if (screen_log_mode>=1)
    {
        printf("%.3f\|\t",0.000001*cycle_time*cycle);
        for (int i = 0; i < device_num; i++)
        {
            ec_slave_config[i].screen_log(ec_slave_config[i],screen_log_mode);
        }
        alg_screen_log(&alg_ref,&alg_var,screen_log_mode);
        printf("\n");
    }
    if (data_log_on==1)
    {
        fprintf(log_fp,"%ld\t",cycle);
        for (int i = 0; i < device_num; i++)
        {
            ec_slave_config[i].data_log(ec_slave_config[i],log_fp);
        }
        fprintf(log_fp,"\n");
    }
    if (raw_log_on==1)
    {
        fprintf(log_fp_raw,"%ld\t",cycle);
        for (int i = 0; i < device_num; i++)
        {
            ec_slave_config[i].raw_log(ec_slave_config[i],log_fp_raw);
        }
        fprintf(log_fp_raw,"\n");
    }
    if (var_log_on==1)
    {
        if (cycle==0)
        {
            alg_log_info(log_fp_var);
        }
        fprintf(log_fp_var,"%ld\t",cycle);
        alg_log(&alg_ref,&alg_var,log_fp_var);
        fprintf(log_fp_var,"\n");
    }
    return 0;
}
#define LOG_INFO(fmt, ...)  ec_event_log("INFO", fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) ec_event_log("ERROR", fmt, ##__VA_ARGS__)

void ec_event_log(const char *level, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    time_t t = time(NULL);
    struct tm *tm_info = localtime(&t);

    if (event_log_on==1)
    {
        fprintf(log_fp_event,"[%02d:%02d:%02d][%s][%ld]:\t",
              tm_info->tm_hour,
              tm_info->tm_min,
              tm_info->tm_sec,
              level,cycle);

        vfprintf(log_fp_event,format, args);
        fprintf(log_fp_event,"\n");
    }
    if (screen_event_log_on==1)
    {
        printf("[%s][%ld]:\t", level,cycle);

        vprintf(format, args);
        printf("\n");

    }
    va_end(args);
}
#endif //CTR_LOG_H