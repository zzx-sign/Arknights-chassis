/**
 * @file kinematics.h
 * @brief 麦克纳姆轮运动学头文件
 *
 * 麦轮布局:
 *   左前轮(1)      右前轮(3)
 *      \            /
 *       \    ^y     /
 *        \   |     /
 *    135°  \ | 45°/
 *         [O]-----> x
 *    45°  / |     \
 *        /  | 135° \
 *       /   v       \
 *   左后轮(0)      右后轮(2)
 *
 * 单位: vx/vy/omega = mm/s (统一使用 mm/s，omega 内部转换为 rad/s)
 */

#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "config.h"
#include <stdint.h>

/*============================================
 * 底盘运动状态 (单位: mm/s)
 *============================================*/
typedef struct
{
    float vx;             /* X方向速度 mm/s */
    float vy;             /* Y方向速度 mm/s */
    float omega;          /* 旋转角速度 rad/s (内部使用) */
} ChassisSpeed_t;

/*============================================
 * 轮子速度 (rad/s)
 *============================================*/
typedef struct
{
    float left_front;     /* 左前轮 rad/s */
    float right_front;    /* 右前轮 rad/s */
    float left_back;     /* 左后轮 rad/s */
    float right_back;     /* 右后轮 rad/s */
} WheelSpeedRad_t;

/*============================================
 * 轮子速度 (mm/s)
 *============================================*/
typedef struct
{
    int16_t left_front;   /* 左前轮 mm/s */
    int16_t right_front;  /* 右前轮 mm/s */
    int16_t left_back;    /* 左后轮 mm/s */
    int16_t right_back;   /* 右后轮 mm/s */
} WheelSpeedMm_t;

/*============================================
 * 位置和角度
 *============================================*/
typedef struct
{
    float x;              /* X坐标 m */
    float y;              /* Y坐标 m */
    float theta;          /* 角度 rad */
} ChassisPose_t;

/*============================================
 * 函数声明
 *============================================*/

/**
 * @brief 运动学逆解: 底盘速度 -> 轮子角速度
 * @param chassis 底盘速度 (vx, vy, omega)
 * @param wheel 输出轮子角速度 (rad/s)
 */
void Kinematics_SolveInverse(ChassisSpeed_t* chassis, WheelSpeedRad_t* wheel);

/**
 * @brief 运动学正解: 轮子角速度 -> 底盘速度
 * @param wheel 轮子角速度 (rad/s)
 * @param chassis 输出底盘速度
 */
void Kinematics_SolveForward(WheelSpeedRad_t* wheel, ChassisSpeed_t* chassis);

/**
 * @brief 限幅底盘速度
 * @param chassis 底盘速度指针
 */
void Kinematics_LimitSpeed(ChassisSpeed_t* chassis);

/**
 * @brief 设置底盘目标速度 (mm/s)
 * @param vx X方向速度 mm/s
 * @param vy Y方向速度 mm/s
 * @param omega_mm_s 旋转等效速度 mm/s (内部自动转换为 rad/s)
 */
void Kinematics_SetTarget(float vx, float vy, float omega_mm_s);

/**
 * @brief 获取当前底盘速度
 */
ChassisSpeed_t* Kinematics_GetCurrentSpeed(void);

/**
 * @brief 更新底盘位置 (积分)
 * @param dt 时间间隔 s
 */
void Kinematics_UpdatePosition(float dt);

/**
 * @brief 获取当前底盘位置
 */
ChassisPose_t* Kinematics_GetPose(void);

/**
 * @brief 重置底盘位置
 */
void Kinematics_ResetPose(float x, float y, float theta);

/**
 * @brief 获取轮子目标速度 (mm/s)
 */
WheelSpeedMm_t* Kinematics_GetTargetWheelSpeed(void);

/**
 * @brief 获取底盘目标速度
 */
ChassisSpeed_t* Kinematics_GetTargetSpeed(void);

#endif /* __KINEMATICS_H__ */
