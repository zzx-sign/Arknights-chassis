/**
 * @file camera.h
 * @brief 树莓派相机数据处理头文件
 */

#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* 函数声明 */

/* 初始化 */
void Camera_Init(void);

/* 数据处理 */
void Camera_Process(void);

/* IDLE中断回调 */
void Camera_IdleCallback(void);

/* 复位 */
void Camera_Reset(void);

/* 数据获取 */
const char* Camera_GetColor1(void);
const char* Camera_GetColor2(void);
const char* Camera_GetColor3(void);
uint8_t Camera_IsValid(void);

/* 调试用 */
const char* Camera_GetRawData(void);
uint32_t Camera_GetRawDataLen(void);
uint32_t Camera_GetRxCount(void);

/* 调试用：获取状态信息 */
uint8_t Camera_GetDebugInfo(uint32_t* pRxCount, uint32_t* pRawLen, uint8_t* pValid);

/* 数据发送 */
void Camera_SendToBluetooth(UART_HandleTypeDef* huart);

/* 根据当前解析到的颜色刷新 LED (red->PB0, blue->PB2, yellow->PF12) */
void Camera_UpdateLedByColor(void);

/* 关闭所有颜色 LED */
void Camera_ClearLeds(void);

#endif /* __CAMERA_H__ */
