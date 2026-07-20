/**
 * @file bluetooth.c
 * @brief 蓝牙数据接收模块 - 使用空闲中断 + DMA
 */

#include "bluetooth.h"
#include "usart.h"
#include "cmsis_os.h"
#include "remote.h"
#include "main.h"
#include "config.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/*============================================
 * 常量定义
 *============================================*/
#define BT_RX_BUFFER_SIZE  128
#define BT_TX_DEBUG_EN     1       /* 调试回传开关 */

/*============================================
 * 静态变量
 *============================================*/
static uint8_t bt_rx_buffer[BT_RX_BUFFER_SIZE];
static osSemaphoreId_t bt_data_sem = NULL;
static uint32_t bt_rx_count = 0;        /* 接收计数 */
static uint32_t bt_parse_success = 0;   /* 解析成功计数 */
static uint32_t bt_parse_fail = 0;      /* 解析失败计数 */
static uint32_t bt_last_rx_time = 0;    /* 上次接收时间 */
static uint32_t bt_button_count = 0;     /* 按钮按下计数 */

/*============================================
 * 内部函数
 *============================================*/

/**
 * @brief 解析文本格式遥控数据
 * 格式1: joystick,<right_x>,<left_y>,<left_x>,<right_y>\r\n
 *       每个值范围: -100 ~ +100
 *
 * 格式2: button,<value>\r\n
 *       按钮按下事件
 *
 * 映射关系:
 *   right_x -> omega (旋转)  右摇杆X
 *   left_y  -> vy     (前进/后退) 左摇杆Y
 *   left_x  -> vx     (左右) 左摇杆X
 *   right_y -> 备用
 */
static void parse_text_frame(const char* data, uint16_t len)
{
    /* 确保以null结尾 */
    static char line[BT_RX_BUFFER_SIZE];
    if (len >= BT_RX_BUFFER_SIZE) len = BT_RX_BUFFER_SIZE - 1;
    memcpy(line, data, len);
    line[len] = '\0';

    /* 去除末尾的 ] 或 \r\n */
    for (int i = len - 1; i >= 0; i--) {
        if (line[i] == ']' || line[i] == '\r' || line[i] == '\n') {
            line[i] = '\0';
        } else {
            break;
        }
    }

    /* 查找 "button," 开头 */
    if (strncmp(line, "button,", 7) == 0) {
        int num_value = atoi(line + 7);
        RemoteData_t* remote = Remote_GetData();
        remote->receive_number = num_value;
        remote->receive_seq++;            /* 每收到一帧 button, 算一次新事件 */
        return;
    }

    /* 查找 "number," 开头 */
    if (strncmp(line, "number,", 7) == 0) {
        int num_value = atoi(line + 7);
        RemoteData_t* remote = Remote_GetData();
        remote->receive_number = num_value;
        remote->receive_seq++;            /* 每收到一帧 number, 算一次新事件 */
        return;
    }

    /* 查找 "joystick," 开头（兼容带[]和不带[]的格式） */
    const char* prefix1 = "[joystick,";
    const char* prefix2 = "joystick,";
    const char* ptr = NULL;

    if (strncmp(line, prefix1, strlen(prefix1)) == 0) {
        ptr = line + strlen(prefix1);
    } else if (strncmp(line, prefix2, strlen(prefix2)) == 0) {
        ptr = line + strlen(prefix2);
    } else {
        bt_parse_fail++;
        return;  /* 不是遥控数据 */
    }

    /* 解析4个int值 */
    int right_x, left_y, left_x, right_y;

    if (sscanf(ptr, "%d,%d,%d,%d", &right_x, &left_y, &left_x, &right_y) != 4) {
        bt_parse_fail++;
        return;  /* 解析失败 */
    }

    /* 限幅到 -100 ~ +100 */
    right_x = (right_x > 100) ? 100 : (right_x < -100) ? -100 : right_x;
    left_y  = (left_y  > 100) ? 100 : (left_y  < -100) ? -100 : left_y;
    left_x  = (left_x  > 100) ? 100 : (left_x  < -100) ? -100 : left_x;
    right_y = (right_y > 100) ? 100 : (right_y < -100) ? -100 : right_y;

    /* 保存原始摇杆值 */
    RemoteData_t* remote = Remote_GetData();
    remote->raw_right_x = right_x;
    remote->raw_left_y = left_y;
    remote->raw_left_x = left_x;
    remote->raw_right_y = right_y;

    /* 映射到速度单位 mm/s
     * 根据 config.h 中的 MAX_LINEAR_SPEED_MM_S / MAX_ANGULAR_SPEED_MM_S
     * 摇杆值 ±100 对应最大速度
     */
    int16_t vx    = (int16_t)((left_x  * MAX_LINEAR_SPEED_MM_S)  / 100.0f);   /* vx: 左右移动 */
    int16_t vy    = (int16_t)((-left_y * MAX_LINEAR_SPEED_MM_S)  / 100.0f);   /* vy: 前进后退 (Y轴取反) */
    int16_t omega = (int16_t)((-right_x * MAX_ANGULAR_SPEED_MM_S) / 100.0f);  /* omega: 旋转 (取反，右摇杆左拉→左转) */

    /* 更新遥控数据 */
    Remote_UpdateBt(vx, vy, omega);

    bt_parse_success++;

#if BT_TX_DEBUG_EN
    /* 发送回传确认 (调试用) */
    char ack_buf[64];
    int ack_len = sprintf(ack_buf, "ACK:%d,%d,%d,%d\r\n", right_x, left_y, left_x, right_y);
    BT_SendData((uint8_t*)ack_buf, ack_len);
#endif
}

/*============================================
 * 公共函数
 *============================================*/

/**
 * @brief 初始化蓝牙模块
 */
void BT_Init(void)
{
    /* 创建信号量 */
    bt_data_sem = osSemaphoreNew(1, 0, NULL);

    /* 使能空闲中断检测 */
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

    /* 测试发送：验证TX是否正常工作 */
    uint8_t test_msg[] = "BT Init OK\r\n";
    HAL_UART_Transmit(&huart2, test_msg, sizeof(test_msg)-1, 500);

    /* 启动 DMA 接收 */
    HAL_UART_Receive_DMA(&huart2, bt_rx_buffer, BT_RX_BUFFER_SIZE);
}

/**
 * @brief 蓝牙接收完成回调 (DMA中断中调用)
 */
void BT_RxCpltCallback(void)
{
    /* 空实现 - 由空闲中断统一处理 */
}

/**
 * @brief 空闲中断回调 (USART中断中调用)
 */
void BT_IdleCallback(void)
{
    /* 获取接收长度 */
    uint16_t len = BT_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);

    if (len > 0 && len < BT_RX_BUFFER_SIZE) {
        /* 停止 DMA */
        HAL_UART_DMAStop(&huart2);

        bt_rx_count++;
        bt_last_rx_time = osKernelGetTickCount();

        /* 解析文本格式数据 */
        parse_text_frame((const char*)bt_rx_buffer, len);

        /* 清除缓冲区 */
        memset(bt_rx_buffer, 0, len);

        /* 重新启动 DMA */
        HAL_UART_Receive_DMA(&huart2, bt_rx_buffer, BT_RX_BUFFER_SIZE);
    }
}

/**
 * @brief 发送数据到蓝牙
 */
void BT_SendData(const uint8_t* data, uint16_t len)
{
    /* 停止DMA接收以避免冲突 */
    HAL_UART_DMAStop(&huart2);

    /* 使用较长超时，9600波特率传输 len 字节需要约 len*10/9600 秒 */
    uint32_t timeout = (len * 10 * 1000) / 9600 + 50;  /* 计算实际需要的ms + 50ms余量 */
    if (timeout < 100) timeout = 100;

    HAL_UART_Transmit(&huart2, (uint8_t*)data, len, timeout);

    /* 重新启动DMA接收 */
    HAL_UART_Receive_DMA(&huart2, bt_rx_buffer, BT_RX_BUFFER_SIZE);
}

/**
 * @brief 等待新数据
 */
int BT_WaitData(uint32_t timeout_ms)
{
    if (bt_data_sem == NULL) return 0;

    return osSemaphoreAcquire(bt_data_sem, timeout_ms) == osOK;
}

/**
 * @brief 获取蓝牙接收统计信息
 */
void BT_GetStats(uint32_t* rx_count, uint32_t* parse_ok, uint32_t* parse_fail, uint32_t* last_rx_time)
{
    if (rx_count) *rx_count = bt_rx_count;
    if (parse_ok) *parse_ok = bt_parse_success;
    if (parse_fail) *parse_fail = bt_parse_fail;
    if (last_rx_time) *last_rx_time = bt_last_rx_time;
}

/**
 * @brief 更新按钮计数
 */
void BT_UpdateButton(uint8_t value)
{
    if (value == 1) {
        bt_button_count++;
    }
}

/**
 * @brief 获取按钮计数
 */
uint32_t BT_GetButtonCount(void)
{
    return bt_button_count;
}
