/**
 ******************************************************************************
 * @file     pid.h
 * @author   sevenfite (based on HNU RobotMaster)
 * @date     2025/5/10
 * @brief    PID控制器头文件
 ******************************************************************************
 * @attention
 * 参照了湖南大学RobotMaster跃鹿战队的代码(author Wang Hongxi)
 ******************************************************************************
 */
#ifndef _PID_CONTROLLER_H
#define _PID_CONTROLLER_H

#include <stdint.h>
#include "config.h"

/*============================================
 * PID结构体
 *============================================*/
typedef struct
{
    /* 基础参数 */
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float DeadBand;

    /* 优化参数 */
    uint8_t Improve;
    float IntegralLimit;
    float CoefA;              /* 变速积分参数A */
    float CoefB;              /* 变速积分参数B: ITerm = Err*((A-abs(err)+B)/A) when B<|err|<A+B */
    float Output_LPF_RC;       /* 输出滤波器 RC = 1/omegac */
    float Derivative_LPF_RC;   /* 微分滤波器系数 */
    float Intergral_Separate;  /* 积分分离值 */
    float DeltaT_Limit_Max;    /* 时间间隔限幅最大值 */
    float DeltaT_Limit_Min;    /* 时间间隔限幅最小值 */

    /* 计算用变量 */
    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;
    float Last_ITerm;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;

    float Output;
    float Last_Output;
    float Last_Dout;

    float Ref;

    uint32_t lastTime;
    float dt;

} PIDInstance;

/* PID初始化配置结构体 */
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float DeadBand;

    uint8_t Improve;
    float IntegralLimit;
    float CoefA;
    float CoefB;
    float Output_LPF_RC;
    float Derivative_LPF_RC;
    float Intergral_Separate;
    float DeltaT_Limit_Max;
    float DeltaT_Limit_Min;
} PID_Init_Config_s;

/*============================================
 * 函数声明
 *============================================*/

/**
 * @brief 初始化PID实例
 */
void PID_Init(PIDInstance *pid, PID_Init_Config_s *config);

/**
 * @brief 计算PID输出
 * @param measure 反馈值
 * @param ref 设定值
 * @return PID计算输出
 */
float PID_Calculate(PIDInstance *pid, float measure, float ref);

/**
 * @brief 重置PID
 */
void PID_Reset(PIDInstance *pid);

/**
 * @brief 设置PID参数
 */
void PID_SetParams(PIDInstance *pid, float kp, float ki, float kd, float maxOut);

/* 时间获取接口 (需适配到你的系统) */
extern uint32_t get_system_time_ms(void);

#endif /* _PID_CONTROLLER_H */
