/**
 * @file remote.h
 * @brief 遥控器解析头文件
 *
 * 解析遥控器/手柄输入，转换为底盘控制指令
 */

#ifndef __REMOTE_H__
#define __REMOTE_H__

#include "config.h"
#include "kinematics.h"
#include <stdint.h>

/*============================================
 * 遥控器类型
 *============================================*/
enum RemoteType
{
    REMOTE_TYPE_NONE = 0,
    REMOTE_TYPE_BLUETOOTH,     /* 蓝牙遥控 */
    REMOTE_TYPE_RESERVED,      /* 保留 */
    REMOTE_TYPE_KEYBOARD       /* 键盘控制 */
};

/*============================================
 * 遥控数据
 *============================================*/
typedef struct
{
    /* 遥杆/按键数据 */
    int16_t ch0;   /* 左摇杆X */
    int16_t ch1;   /* 左摇杆Y */
    int16_t ch2;   /* 右摇杆X */
    int16_t ch3;   /* 右摇杆Y */
    uint8_t s1;    /* 开关1 */
    uint8_t s2;    /* 开关2 */

    /* 原始摇杆百分比值 (-100 ~ +100) */
    int16_t raw_left_x;    /* 左摇杆X原始值 */
    int16_t raw_left_y;    /* 左摇杆Y原始值 */
    int16_t raw_right_x;   /* 右摇杆X原始值 */
    int16_t raw_right_y;   /* 右摇杆Y原始值 */

    /* 解析后的速度指令 (单位: mm/s) */
    float vx;      /* X方向速度 mm/s */
    float vy;      /* Y方向速度 mm/s */
    float omega;   /* 旋转等效速度 mm/s */

    /* 控制模式 */
    uint8_t mode;  /* 0=停止, 1=正常, 2=调试 */

    /* 连接状态 */
    uint8_t connected;
    uint32_t last_update_time;

    /* 接收到的数字 */
    int32_t receive_number;

} RemoteData_t;

/*============================================
 * 函数声明
 *============================================*/

/**
 * @brief 遥控器初始化
 */
void Remote_Init(void);

/**
 * @brief 解析遥控器数据
 */
void Remote_Parse(void);

/**
 * @brief 更新遥控器数据
 */
void Remote_Update(int16_t ch0, int16_t ch1, int16_t ch2, int16_t ch3, uint8_t s1, uint8_t s2);

/**
 * @brief 获取遥控器数据
 */
RemoteData_t* Remote_GetData(void);

/**
 * @brief 检查遥控器连接状态
 */
uint8_t Remote_IsConnected(void);

/**
 * @brief 设置遥控器类型
 */
void Remote_SetType(enum RemoteType type);

/**
 * @brief 获取遥控器类型
 */
enum RemoteType Remote_GetType(void);

/**
 * @brief 蓝牙模式直接更新速度
 */
void Remote_UpdateBt(int16_t vx, int16_t vy, int16_t omega);

#endif /* __REMOTE_H__ */
