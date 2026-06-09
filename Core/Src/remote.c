/**
 * @file remote.c
 * @brief 遥控器解析实现
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

/* 解析参数 (需要根据实际调整) */
#define REMOTE_DEADZONE     100     /* 摇杆死区 */
#define MAX_JOYSTICK_VALUE  660     /* 摇杆最大值 */

/* 速度映射参数 - 统一使用 mm/s */
#define MAX_VX_MM_S        1000.0f   /* 最大X速度 mm/s */
#define MAX_VY_MM_S        1000.0f   /* 最大Y速度 mm/s */
#define MAX_OMEGA_MM_S     500.0f    /* 最大旋转等效速度 mm/s */

/*============================================
 * 内部函数
 *============================================*/

/**
 * @brief 摇杆死区处理
 */
static int16_t apply_deadzone(int16_t value)
{
    if (value > REMOTE_DEADZONE) {
        return value - REMOTE_DEADZONE;
    } else if (value < -REMOTE_DEADZONE) {
        return value + REMOTE_DEADZONE;
    }
    return 0;
}

/**
 * @brief 限幅
 */
static int16_t clamp_int16(int16_t val, int16_t min_val, int16_t max_val)
{
    if (val > max_val) return max_val;
    if (val < min_val) return min_val;
    return val;
}

/**
 * @brief 根据遥控器类型解析数据
 */
static void parse_bluetooth(void)
{
    int16_t lx, ly, rx, ry;

    /* 读取摇杆值 */
    lx = apply_deadzone(remote_data.ch0);
    ly = apply_deadzone(remote_data.ch1);
    rx = apply_deadzone(remote_data.ch2);
    ry = apply_deadzone(remote_data.ch3);

    /* 归一化到 [-1, 1] */
    float norm_lx = (float)lx / (MAX_JOYSTICK_VALUE - REMOTE_DEADZONE);
    float norm_ly = (float)ly / (MAX_JOYSTICK_VALUE - REMOTE_DEADZONE);
    float norm_rx = (float)rx / (MAX_JOYSTICK_VALUE - REMOTE_DEADZONE);
    float norm_ry = (float)ry / (MAX_JOYSTICK_VALUE - REMOTE_DEADZONE);

    /* 限幅 */
    norm_lx = clamp_int16((int16_t)(norm_lx * 1000), -1000, 1000) / 1000.0f;
    norm_ly = clamp_int16((int16_t)(norm_ly * 1000), -1000, 1000) / 1000.0f;
    norm_rx = clamp_int16((int16_t)(norm_rx * 1000), -1000, 1000) / 1000.0f;
    norm_ry = clamp_int16((int16_t)(norm_ry * 1000), -1000, 1000) / 1000.0f;

    /* 计算速度 (统一使用 mm/s)
     * 左手摇杆: 前后左右移动
     * 右手摇杆: 旋转
     */
    remote_data.vx = norm_lx * MAX_VX_MM_S;
    remote_data.vy = -norm_ly * MAX_VY_MM_S;  /* 摇杆Y轴取反 */
    remote_data.omega = -norm_rx * MAX_OMEGA_MM_S;  /* 旋转等效速度 mm/s */

    /* 判断模式 */
    if (remote_data.s1 == 1 && remote_data.s2 == 1) {
        /* 正常模式 */
        remote_data.mode = 1;
    } else if (remote_data.s1 == 3 && remote_data.s2 == 3) {
        /* 停止模式 */
        remote_data.mode = 0;
        remote_data.vx = 0;
        remote_data.vy = 0;
        remote_data.omega = 0;
    } else {
        /* 其他状态 */
        remote_data.mode = 2;
    }
}

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
 * @brief 解析遥控器数据
 */
void Remote_Parse(void)
{
    switch (remote_type) {
        case REMOTE_TYPE_BLUETOOTH:
            parse_bluetooth();
            break;

        case REMOTE_TYPE_KEYBOARD:
            /* 键盘控制 - 可后续扩展 */
            break;

        default:
            break;
    }
}

/**
 * @brief 更新遥控器数据
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

    /* 解析数据 */
    Remote_Parse();
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
