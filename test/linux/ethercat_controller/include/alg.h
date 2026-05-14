//
// Created by root on 2026/5/7.
//

#ifndef SOEM_ALG_H
#define SOEM_ALG_H

#include "config.h"

typedef struct
{
    double q[2];
    double v_q[2];
    double tau_q[2];
    double theta[3];
    double d_theta;
    double theta_2e;
    double q2_err;
    double x_e[3];
    double fs[6];

    double q_r[2];
    double theta_r[2];
    double d_q_r[2];
    double dd_q_r[2];
    double d_theta_r[2];
    double dd_theta_r[2];
    double tau_r[2];

    double theta_0[2];
} alg_var_s;
alg_var_s alg_var;

typedef struct
{
    double q_r[2];
    double theta_r[2];
    double d_q_r[2];
    double dd_q_r[2];
    double d_theta_r[2];
    double dd_theta_r[2];
    double tau_r[2];

} alg_ref_s;
alg_ref_s alg_ref;

int alg_data_handle(alg_ref_s* alg_r,alg_var_s* alg_v)
{
    alg_v->q[0]=cdr6_data_interface_feedback_p.actual_position[1];
    alg_v->q[1]=cdr6_data_interface_feedback_p.actual_position[0];
    alg_v->v_q[0]=cdr6_data_interface_feedback_p.actual_velocity[1];
    alg_v->v_q[1]=cdr6_data_interface_feedback_p.actual_velocity[0];
    alg_v->tau_q[0]=cdr6_data_interface_feedback_p.actual_torque[1];
    alg_v->tau_q[1]=cdr6_data_interface_feedback_p.actual_torque[0];

    alg_v->theta[0]=-0.01*alg_v->q[0];
    alg_v->theta[1]=ac58_data_interface_feedback_p_1.ac_encoder_rad;
    alg_v->theta[2]=-ac58_data_interface_feedback_p_2.ac_encoder_rad;
    alg_v->d_theta=alg_v->theta[2]-alg_v->theta[1];

    alg_v->theta_2e=2*asin(-0.00106344* alg_v->q[1]);

    alg_v->x_e[0]=0.506*sin(alg_v->theta[0])+0.16*sin(alg_v->theta[0]+alg_v->theta[1])+0.24*sin(alg_v->theta[0]+alg_v->theta[1]+alg_v->theta[2]);
    alg_v->x_e[1]=0.506*cos(alg_v->theta[0])+0.16*cos(alg_v->theta[0]+alg_v->theta[1])+0.24*cos(alg_v->theta[0]+alg_v->theta[1]+alg_v->theta[2]);
    alg_v->x_e[2]=alg_v->theta[0]+alg_v->theta[1]+alg_v->theta[2];

    alg_v->fs[0]=sbt908_data_interface_feedback_p.fs_data[0];
    alg_v->fs[1]=sbt908_data_interface_feedback_p.fs_data[1];
    alg_v->fs[2]=sbt908_data_interface_feedback_p.fs_data[2];
    alg_v->fs[3]=sbt908_data_interface_feedback_p.fs_data[3];
    alg_v->fs[4]=sbt908_data_interface_feedback_p.fs_data[4];
    alg_v->fs[5]=sbt908_data_interface_feedback_p.fs_data[5];

    if (cycle==0)
    {
        alg_v->q_r[0]=alg_v->q[0];
        alg_v->q_r[1]=alg_v->q[1];
        alg_v->theta_r[0]=alg_v->theta[0];
        alg_v->theta_r[1]=alg_v->theta[1];

        alg_r->q_r[0]=alg_v->q[0];
        alg_r->q_r[1]=alg_v->q[1];
        alg_r->theta_r[0]=alg_v->theta[0];
        alg_r->theta_r[1]=alg_v->theta[1];

        alg_v->theta_0[0]=alg_v->theta[0];
        alg_v->theta_0[1]=alg_v->theta[1];

        alg_v->q2_err=-940.3445*sin(alg_v->theta[1])-alg_v->q[1];

        alg_v->d_q_r[0]=0.0;
        alg_v->dd_q_r[0]=0.0;
        alg_v->d_theta_r[0]=0.0;
        alg_v->dd_theta_r[0]=0.0;
        alg_v->d_q_r[1]=0.0;
        alg_v->dd_q_r[1]=0.0;
        alg_v->d_theta_r[1]=0.0;
        alg_v->dd_theta_r[1]=0.0;

        alg_r->d_q_r[0]=0.0;
        alg_r->dd_q_r[0]=0.0;
        alg_r->d_theta_r[0]=0.0;
        alg_r->dd_theta_r[0]=0.0;
        alg_r->d_q_r[1]=0.0;
        alg_r->dd_q_r[1]=0.0;
        alg_r->d_theta_r[1]=0.0;
        alg_r->dd_theta_r[1]=0.0;

        alg_v->tau_r[0]=0.0;
        alg_v->tau_r[1]=0.0;
        alg_r->tau_r[0]=0.0;
        alg_r->tau_r[1]=0.0;
    }
    return 0;
}

int alg_control(alg_ref_s* alg_r,alg_var_s* alg_v)
{


    // 这里计算alg_r->q_r
     // if (alg_r->q_r[0]>0)
     //     alg_r->q_r[0]=alg_v->q_r[0]-0.001;
     // if (alg_r->q_r[1]>0)
     //     alg_r->q_r[1]=alg_v->q_r[1]-0.001;
    // 这里计算alg_r->q_r

    // alg_r->theta_r[0]=-0.01* alg_r->q_r[0];
    // alg_r->theta_r[1]=asin(-0.00106344* (alg_r->q_r[1]+alg_v->q2_err));

    //这里计算alg_r->theta_r

    // if (alg_r->theta_r[0]<20/r2d)
    //     alg_r->theta_r[0]=alg_v->theta_r[0]+0.00005;
    // if (alg_r->theta_r[1]<20/r2d)
    //     alg_r->theta_r[1]=alg_v->theta_r[1]+0.00005;

    double t=0.002*cycle;
    if (t<5)
    {
        alg_r->theta_r[1]=alg_v->theta_0[1]*cos(2*3.14/20*t);
    }
    else if (t<7)
    {
        alg_r->theta_r[1]=-2*3.14/20*alg_v->theta_0[1]*(t-5);
    }
    else if (t<9)
    {
        alg_r->theta_r[1]=-2*3.14/20*alg_v->theta_0[1]*2-2*3.14/20*alg_v->theta_0[1]*(t-7)+2*3.14/20*alg_v->theta_0[1]/4*(t-7)*(t-7);
    }
    // alg_r->theta_r[0]=0*alg_v->theta_0[0]*cos(2*3.14/20*t);
    // alg_r->theta_r[1]=alg_v->theta_0[1]*cos(2*3.14/15*t);

    //这里计算alg_r->theta_r
    alg_r->q_r[0]=-100.0 * alg_r->theta_r[0];
    alg_r->q_r[1]=-940.3445*sin(alg_r->theta_r[1])-alg_v->q2_err;

    alg_v->q_r[0]=alg_r->q_r[0];
    alg_v->q_r[1]=alg_r->q_r[1];
    alg_v->theta_r[0]=alg_r->theta_r[0];
    alg_v->theta_r[1]=alg_r->theta_r[1];
    alg_v->d_q_r[0]=alg_r->d_q_r[0];
    alg_v->dd_q_r[0]=alg_r->dd_q_r[0];
    alg_v->d_theta_r[0]=alg_r->d_theta_r[0];
    alg_v->dd_theta_r[0]=alg_r->dd_theta_r[0];
    alg_v->d_q_r[1]=alg_r->d_q_r[1];
    alg_v->dd_q_r[1]=alg_r->dd_q_r[1];
    alg_v->d_theta_r[1]=alg_r->d_theta_r[1];
    alg_v->dd_theta_r[1]=alg_r->dd_theta_r[1];
    alg_v->tau_r[0]=alg_r->tau_r[0];
    alg_v->tau_r[1]=alg_r->tau_r[1];
    alg_v->tau_r[1]=alg_r->tau_r[1];
    return 0;
}

int  alg_screen_log(alg_ref_s* alg_r,alg_var_s* alg_v,int mode)
{
    if(mode==3)
    printf("%.2f\t%.2f\|\t%.2f\t%.2f\t%.2f\|\t%.2f\t%.2f\t%.2f\|\t%.2f\t%.2f\t%.2f\|\t%.2f\t%.2f\|\t%.4f\t%.4f\t%.2f\|\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f",
        alg_r->theta_r[0]*r2d, alg_v->theta[0]*r2d,
        alg_r->theta_r[1]*r2d, alg_v->theta[1]*r2d,alg_v->q2_err,
        alg_v->theta[1]*r2d,alg_v->theta[2]*r2d,alg_v->d_theta*r2d,
        alg_v->theta_2e*r2d,alg_v->theta[1]*r2d+alg_v->theta[2]*r2d,alg_v->theta_2e*r2d-alg_v->theta[1]*r2d-alg_v->theta[2]*r2d,
        alg_v->tau_q[0],alg_v->tau_q[1],
        alg_v->x_e[0],alg_v->x_e[1],alg_v->x_e[2]*r2d,
        alg_v->fs[0],alg_v->fs[1],alg_v->fs[2],alg_v->fs[3],alg_v->fs[4],alg_v->fs[5]);
    return 0;
}

int alg_log_info(FILE* log_fp_v)
{
    fprintf(log_fp_v,"alg_r->theta_r[0],alg_r->theta_r[1],alg_r->q_r[0],alg_r->q_r[1],"
                     "alg_v->q[0],alg_v->q[1],alg_v->v_q[0],alg_v->v_q[1],alg_v->tau_q[0],alg_v->tau_q[1],"
                     "alg_v->theta[0],alg_v->theta[1],alg_v->theta[2],alg_v->d_theta,alg_v->theta_2e,alg_v->x_e[0],alg_v->x_e[1],alg_v->x_e[2],"
                     "alg_v->fs[0],alg_v->fs[1],alg_v->fs[2],alg_v->fs[3],alg_v->fs[4],alg_v->fs[5]"
                     "2*alg_v->theta_2e*r2d,alg_v->theta[1]*r2d+alg_v->theta[2]*r2d,2*alg_v->theta_2e*r2d-alg_v->theta[1]*r2d-alg_v->theta[2]*r2d,alg_v->q2_err\n");
    return 0;
}

int  alg_log(alg_ref_s* alg_r,alg_var_s* alg_v,FILE* log_fp_v)
{
    fprintf(log_fp_v,"%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f",
        alg_r->theta_r[0],alg_r->theta_r[1],alg_r->q_r[0],alg_r->q_r[1],
        alg_v->q[0],alg_v->q[1],alg_v->v_q[0],alg_v->v_q[1],alg_v->tau_q[0],alg_v->tau_q[1],
        alg_v->theta[0],alg_v->theta[1],alg_v->theta[2],alg_v->d_theta,alg_v->theta_2e,alg_v->x_e[0],alg_v->x_e[1],alg_v->x_e[2],
        alg_v->fs[0],alg_v->fs[1],alg_v->fs[2],alg_v->fs[3],alg_v->fs[4],alg_v->fs[5],
        2*alg_v->theta_2e,alg_v->theta[1]+alg_v->theta[2],2*alg_v->theta_2e-alg_v->theta[1]-alg_v->theta[2],alg_v->q2_err);
    return 0;

}

int alg_safe_check(alg_ref_s* alg_r,alg_var_s* alg_v)
{
    if (fabs(alg_r->theta_r[0]*r2d)>80||fabs(alg_r->theta_r[1])>0.6716)
    {
        return -1;
    }
    return 0;
}
#endif //SOEM_ALG_H