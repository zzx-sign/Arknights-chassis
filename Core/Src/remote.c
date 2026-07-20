/**
 * @file remote.c
 * @brief 遥控器数据模块
 *
 * 当前仅支持蓝牙文本协议 (joystick, / number, / button,)，
 * 由 bluetooth.c::parse_text_frame() 直接调用 Remote_UpdateBt()
 * 或填入 receive_number/receive_seq。
 *
 * 历史遗留的摇杆解析链路 (parse_bluetooth / Remote_Update) 已移除。
 */

#include "remote.h"
#include "cmsis_os.h"
#include <string.h>

/*============================================
 * 静态变量
 *============================================*/

/* 遥控器类型 */
static enum RemoteType remote_type = REMOTE_TYPE_BLUETOOTH;

/* 遥控器数据 */
static RemoteData_t remote_data;

/*============================================
 * 公共函数
 *============================================*/

/**
 * @brief 遥控器初始化
 */
void Remote_Init(void)
{
    memset(&remote_data, 0, sizeof(RemoteData_t));
    remote_data.connected = 0;
    remote_data.mode = 0;
}

/**
 * @brief 更新遥控器数据
 *
 * 注: 蓝牙文本协议由 bluetooth.c 直接调用 Remote_UpdateBt()，
 * 此函数保留接口用于将来可能接入的 DBUS/SBUS 等原始通道数据。
 */
void Remote_Update(int16_t ch0, int16_t ch1, int16_t ch2, int16_t ch3, uint8_t s1, uint8_t s2)
{
    remote_data.ch0 = ch0;
    remote_data.ch1 = ch1;
    remote_data.ch2 = ch2;
    remote_data.ch3 = ch3;
    remote_data.s1 = s1;
    remote_data.s2 = s2;
    remote_data.connected = 1;
    remote_data.last_update_time = osKernelGetTickCount();
}

/**
 * @brief 获取遥控器数据
 */
RemoteData_t* Remote_GetData(void)
{
    return &remote_data;
}

/**
 * @brief 检查遥控器连接状态
 */
uint8_t Remote_IsConnected(void)
{
    /* 检查超时 */
    uint32_t now = osKernelGetTickCount();
    if (now - remote_data.last_update_time > 500) {
        remote_data.connected = 0;
    }
    return remote_data.connected;
}

/**
 * @brief 设置遥控器类型
 */
void Remote_SetType(enum RemoteType type)
{
    remote_type = type;
}

/**
 * @brief 获取遥控器类型
 */
enum RemoteType Remote_GetType(void)
{
    return remote_type;
}

/**
 * @brief 蓝牙模式直接更新速度 (绕过摇杆解析)
 * @param vx X方向速度 mm/s
 * @param vy Y方向速度 mm/s
 * @param omega 旋转等效速度 mm/s
 */
void Remote_UpdateBt(int16_t vx, int16_t vy, int16_t omega)
{
    remote_data.vx = vx;
    remote_data.vy = vy;
    remote_data.omega = omega;
    remote_data.connected = 1;
    remote_data.last_update_time = osKernelGetTickCount();
    remote_data.mode = 1;  /* 正常模式 */
}