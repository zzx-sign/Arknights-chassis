/**
 ******************************************************************************
 * @file     pid.c
 * @author   sevenfite (based on HNU RobotMaster)
 * @brief    PID控制器实现
 ******************************************************************************
 */
#include "pid.h"
#include <string.h>

#define abs_val(x) ((x > 0) ? x : -x)
#define is_same_sign(a, b) ((a >= 0 && b >= 0) || (a < 0 && b < 0))

/*============================================
 * 优化环节实现
 *============================================*/

/* 梯形积分 */
static void f_Trapezoid_Intergral(PIDInstance *pid)
{
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
}

/* 变速积分 */
static void f_Changing_Integration_Rate(PIDInstance *pid)
{
    if (is_same_sign(pid->Err, pid->Iout))
    {
        if (abs_val(pid->Err) <= pid->CoefB)
            return;
        if (abs_val(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - abs_val(pid->Err) + pid->CoefB) / pid->CoefA;
        else
            pid->ITerm = 0;
    }
}

/* 积分分离 */
static void f_Integral_Separate(PIDInstance *pid)
{
    if (pid->Intergral_Separate <= 0)
        return;

    if (abs_val(pid->Err) > pid->Intergral_Separate)
    {
        pid->Iout = 0;
        pid->ITerm = 0;
    }
}

/* 积分限幅 */
static void f_Integral_Limit(PIDInstance *pid)
{
    float temp_Iout = pid->Iout + pid->ITerm;
    float temp_Output = pid->Pout + pid->Iout + pid->Dout;

    if (abs_val(temp_Output) > pid->MaxOut)
    {
        if ((pid->Ki > 0 && is_same_sign(pid->Err, pid->Iout)) ||
            (pid->Ki < 0 && (!is_same_sign(pid->Err, pid->Iout))))
        {
            pid->ITerm = 0;
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

/* 微分先行 */
static void f_Derivative_On_Measurement(PIDInstance *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

/* 微分滤波 */
static void f_Derivative_Filter(PIDInstance *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

/* 输出滤波 */
static void f_Output_Filter(PIDInstance *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

/* 时间间隔限幅 */
static void f_DeltaT_Limit(PIDInstance *pid)
{
    if (pid->dt > pid->DeltaT_Limit_Max)
        pid->dt = pid->DeltaT_Limit_Max;
    if (pid->dt < pid->DeltaT_Limit_Min)
        pid->dt = pid->DeltaT_Limit_Min;
}

/* 输出限幅 */
static void f_Output_Limit(PIDInstance *pid)
{
    if (pid->Output > pid->MaxOut)
        pid->Output = pid->MaxOut;
    if (pid->Output < -(pid->MaxOut))
        pid->Output = -(pid->MaxOut);
}

/*============================================
 * 公共函数
 *============================================*/

/**
 * @brief 初始化PID
 */
void PID_Init(PIDInstance *pid, PID_Init_Config_s *config)
{
    memset(pid, 0, sizeof(PIDInstance));
    memcpy(pid, config, sizeof(PID_Init_Config_s));

    /* 获取初始时间 */
    pid->lastTime = get_system_time_ms();
}

/**
 * @brief PID计算
 */
float PID_Calculate(PIDInstance *pid, float measure, float ref)
{
    /* 获取时间间隔 */
    uint32_t now = get_system_time_ms();
    pid->dt = (float)(now - pid->lastTime) / 1000.0f;
    pid->lastTime = now;

    if (pid->Improve & PID_DeltaT_Limit)
        f_DeltaT_Limit(pid);

    /* 保存上次的测量值和误差 */
    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    /* 死区外才计算 */
    if (abs_val(pid->Err) > pid->DeadBand)
    {
        /* 基础PID计算 */
        pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err * pid->dt;
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;

        /* 优化环节 */
        if (pid->Improve & PID_Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        if (pid->Improve & PID_ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);
        if (pid->Improve & PID_Integral_Separate)
            f_Integral_Separate(pid);
        if (pid->Improve & PID_Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        if (pid->Improve & PID_DerivativeFilter)
            f_Derivative_Filter(pid);
        if (pid->Improve & PID_Integral_Limit)
            f_Integral_Limit(pid);

        /* 累加积分 */
        pid->Iout += pid->ITerm;

        /* 计算输出 */
        pid->Output = pid->Pout + pid->Iout + pid->Dout;

        /* 输出滤波 */
        if (pid->Improve & PID_OutputFilter)
            f_Output_Filter(pid);

        /* 输出限幅 */
        f_Output_Limit(pid);
    }
    else
    {
        pid->Output = pid->Last_Output;
        pid->ITerm = 0;
    }

    /* 保存当前数据 */
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;

    return pid->Output;
}

/**
 * @brief 重置PID
 */
void PID_Reset(PIDInstance *pid)
{
    pid->Err = 0;
    pid->Last_Err = 0;
    pid->Pout = 0;
    pid->Iout = 0;
    pid->ITerm = 0;
    pid->Dout = 0;
    pid->Output = 0;
    pid->Last_Output = 0;
    pid->Last_Dout = 0;
    pid->Last_ITerm = 0;
}

/**
 * @brief 设置PID参数
 */
void PID_SetParams(PIDInstance *pid, float kp, float ki, float kd, float maxOut)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->MaxOut = maxOut;
}
