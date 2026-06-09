/**
 * @file motor.h
 * @brief 麦轮底盘电机驱动头文件
 *
 * 支持开环PWM和速度闭环两种模式
 */

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"
#include "config.h"
#include "pid.h"
#include <stdint.h>

/*============================================
 * 硬件配置
 *============================================*/

/* TIM1 PWM通道映射 */
#define MOTOR_PWM_CHANNEL_1  TIM_CHANNEL_1  /* PE9  - 电机1 */
#define MOTOR_PWM_CHANNEL_2  TIM_CHANNEL_2  /* PE11 - 电机2 */
#define MOTOR_PWM_CHANNEL_3  TIM_CHANNEL_3  /* PE13 - 电机3 */
#define MOTOR_PWM_CHANNEL_4  TIM_CHANNEL_4  /* PE14 - 电机4 */

/* 编码器TIM */
#define MOTOR1_ENCODER_TIM   &htim2
#define MOTOR2_ENCODER_TIM   &htim3
#define MOTOR3_ENCODER_TIM   &htim4
#define MOTOR4_ENCODER_TIM   &htim8

/*============================================
 * 电机定义
 *============================================*/
#define MOTOR_COUNT          4

enum MotorId
{
    MOTOR_ID_LEFT_FRONT = 0,
    MOTOR_ID_RIGHT_FRONT = 1,
    MOTOR_ID_LEFT_BACK = 2,
    MOTOR_ID_RIGHT_BACK = 3
};

enum MotorMode
{
    MOTOR_MODE_OPEN_LOOP_PWM = 0,  /* 开环PWM模式 */
    MOTOR_MODE_SPEED_LOOP = 1,     /* 速度闭环模式 */
};

/*============================================
 * 电机控制块
 *============================================*/
typedef struct _MotorControlBlock
{
    /* 硬件配置 */
    TIM_HandleTypeDef* PWM_TIM;      /* PWM定时器 */
    TIM_HandleTypeDef* encoder_TIM;  /* 编码器定时器 */
    uint32_t pwm_channel;            /* PWM通道 */

    /* PID */
    PIDInstance pid;

    /* 电机状态 */
    enum MotorMode mode;
    int8_t encoder_sign;             /* 编码器方向修正 (+1/-1) */
    uint16_t encoder_one_round;     /* 一圈编码器脉冲数 */

    /* 速度数据 (mm/s) */
    int16_t target_speed;           /* 目标速度 */
    int16_t feedback_speed;         /* 反馈速度 */
    int16_t PWM;                    /* PWM输出值 */

    /* 距离数据 (mm) */
    float distance;                 /* 累计距离 */
    float distance_coefficient;     /* 距离校正系数 */

    /* 编码器数据 */
    uint16_t last_encoder_cnt;
    uint8_t first_read;             /* 首次读取标志 */

} MotorControlBlock_t;

/*============================================
 * 全局状态
 *============================================*/
typedef struct
{
    /* 四个电机目标速度 */
    int16_t left_front_speed;
    int16_t right_front_speed;
    int16_t left_back_speed;
    int16_t right_back_speed;

    /* 四个电机反馈速度 */
    int16_t feedback_left_front;
    int16_t feedback_right_front;
    int16_t feedback_left_back;
    int16_t feedback_right_back;

    /* PWM值 */
    int16_t PWM_left_front;
    int16_t PWM_right_front;
    int16_t PWM_left_back;
    int16_t PWM_right_back;

    /* 距离 */
    int32_t distance_left_front;
    int32_t distance_right_front;
    int32_t distance_left_back;
    int32_t distance_right_back;

    /* 总距离 */
    int32_t total_distance;

} ChassisMotorState_t;

/*============================================
 * 函数声明
 *============================================*/

/* 初始化 */
void Motor_Init(void);
void Encoder_Init(void);

/* 速度控制 (mm/s) */
void Motor_SetSpeed(int16_t lf, int16_t rf, int16_t lb, int16_t rb);
void Motor_SetSpeed_Real(int16_t lf, int16_t rf, int16_t lb, int16_t rb);
void Motor_Stop(void);
void Motor_StopSmooth(void);

/* 模式切换 */
void Motor_SetMode(enum MotorMode mode);
enum MotorMode Motor_GetMode(void);

/* 速度读取 */
int16_t Motor_GetFeedbackSpeed(enum MotorId motor_id);
ChassisMotorState_t* Motor_GetState(void);

/* PID控制循环 (定时调用) */
void Motor_ControlLoop(void);

/* 编码器更新 (定时调用) */
void Motor_EncoderUpdate(void);

/* 单位换算 */
float Motor_mm_s_to_rad_s(int16_t mm_s);
int16_t Motor_rad_s_to_mm_s(float rad_s);

#endif /* __MOTOR_H__ */
