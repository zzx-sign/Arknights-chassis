/**
 * @file bluetooth.h
 * @brief 蓝牙数据接收模块头文件
 *
 * 使用空闲中断 + DMA 接收蓝牙数据
 */

#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

#include <stdint.h>
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================
 * 函数声明
 *============================================*/

/**
 * @brief 初始化蓝牙模块
 */
void BT_Init(void);

/**
 * @brief DMA接收完成回调
 */
void BT_RxCpltCallback(void);

/**
 * @brief 空闲中断回调
 */
void BT_IdleCallback(void);

/**
 * @brief 发送数据到蓝牙
 */
void BT_SendData(const uint8_t* data, uint16_t len);

/**
 * @brief 等待新数据
 * @param timeout_ms 超时时间(ms)
 * @return 1:有数据, 0:超时
 */
int BT_WaitData(uint32_t timeout_ms);

/**
 * @brief 获取蓝牙接收统计信息
 */
void BT_GetStats(uint32_t* rx_count, uint32_t* parse_ok, uint32_t* parse_fail, uint32_t* last_rx_time);

/**
 * @brief 更新按钮计数
 */
void BT_UpdateButton(uint8_t value);

/**
 * @brief 获取按钮计数
 */
uint32_t BT_GetButtonCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __BLUETOOTH_H__ */
