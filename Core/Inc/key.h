/**
 * @file key.h
 * @brief 按键处理模块头文件
 *
 * 使用板载按键手动控制底盘
 */

#ifndef __KEY_H__
#define __KEY_H__

#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================
 * 按键定义
 *============================================*/
enum KeyId
{
    KEY_ID_0 = 0,    /* KEY0 */
    KEY_ID_1 = 1,    /* KEY1 */
    KEY_COUNT = 2
};

/*============================================
 * 按键控制速度 (mm/s)
 *============================================*/
#define KEY_FORWARD_SPEED       600   /* 前进速度 */
#define KEY_BACKWARD_SPEED     -600   /* 后退速度 */

/*============================================
 * 函数声明
 *============================================*/

/**
 * @brief 初始化按键模块
 */
void KEY_Init(void);

/**
 * @brief 扫描按键 (定时调用，周期20ms)
 */
void KEY_Scan(void);

/**
 * @brief 检查按键是否短按
 */
uint8_t KEY_IsShortPress(enum KeyId key_id);

/**
 * @brief 检查按键是否长按
 */
uint8_t KEY_IsLongPress(enum KeyId key_id);

/**
 * @brief 检查按键是否按下 (包含长按和短按)
 */
uint8_t KEY_IsPressed(enum KeyId key_id);

#ifdef __cplusplus
}
#endif

#endif /* __KEY_H__ */
