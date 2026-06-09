/**
 * @file camera.c
 * @brief 树莓派相机数据处理 - 颜色识别结果解析
 */

#include "camera.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"

/* 接收缓冲区大小 - 扩大缓冲区以支持更多数据 */
#define CAMERA_RX_BUFFER_SIZE  128

/* 接收缓冲区 */
static uint8_t camera_rx_buffer[CAMERA_RX_BUFFER_SIZE];

/* 解析后的颜色数据 */
static char color1[16] = {0};
static char color2[16] = {0};
static uint8_t color_valid = 0;

/* 调试用数据 */
static char raw_data[64] = {0};
static char display_raw[64] = {0};  /* 用于显示的原始数据 */
static uint32_t display_raw_len = 0;  /* display_raw的长度 */
static uint32_t raw_data_len = 0;
static uint32_t rx_count = 0;

/* IDLE标志 */
static volatile uint8_t idle_flag = 0;

/* 环形缓冲读写指针（累计计数）*/
static uint32_t read_cnt = 0;       /* 已处理的字节数 */
static uint32_t write_cnt = 0;      /* 已写入的字节数 */
static uint16_t last_write_idx = 0; /* 上次的写入位置 */

/* USART3句柄声明 */
extern UART_HandleTypeDef huart3;

/**
 * @brief 初始化相机数据接收
 */
void Camera_Init(void)
{
    memset(camera_rx_buffer, 0, sizeof(camera_rx_buffer));
    memset(color1, 0, sizeof(color1));
    memset(color2, 0, sizeof(color2));
    memset(raw_data, 0, sizeof(raw_data));
    raw_data_len = 0;
    color_valid = 0;
    idle_flag = 0;
    read_cnt = 0;
    write_cnt = 0;
    last_write_idx = 0;

    HAL_UART_Receive_DMA(&huart3, camera_rx_buffer, CAMERA_RX_BUFFER_SIZE);

    /* 使能IDLE中断 */
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
}

/**
 * @brief IDLE中断回调 - 记录帧结束
 */
void Camera_IdleCallback(void)
{
    /* 获取当前DMA写入位置 */
    uint16_t current_cnt = __HAL_DMA_GET_COUNTER(huart3.hdmarx);
    uint16_t current_write_idx = CAMERA_RX_BUFFER_SIZE - current_cnt;

    /* 计算新增的字节数（考虑环形缓冲） */
    if (current_write_idx >= last_write_idx) {
        write_cnt += (current_write_idx - last_write_idx);
    } else {
        /* DMA循环了一圈 */
        write_cnt += (CAMERA_RX_BUFFER_SIZE - last_write_idx + current_write_idx);
    }
    last_write_idx = current_write_idx;

    idle_flag = 1;
}

/**
 * @brief 处理接收到的数据
 */
void Camera_Process(void)
{
    if (!idle_flag) return;
    idle_flag = 0;

    /* 计算当前帧长度 */
    uint32_t frame_len = write_cnt - read_cnt;

    /* 先保存原始数据用于显示（从缓冲区复制，去掉\r\n） */
    if (frame_len > 0 && frame_len < sizeof(display_raw)) {
        uint32_t copy_len = frame_len;

        /* 从源缓冲区检查并去掉结尾的\r\n */
        for (uint32_t i = frame_len; i > 0; i--) {
            uint16_t idx = (read_cnt + i - 1) % CAMERA_RX_BUFFER_SIZE;
            if (camera_rx_buffer[idx] == '\n' || camera_rx_buffer[idx] == '\r') {
                copy_len = i - 1;
            } else {
                break;
            }
        }

        for (uint32_t i = 0; i < copy_len; i++) {
            uint16_t idx = (read_cnt + i) % CAMERA_RX_BUFFER_SIZE;
            display_raw[i] = camera_rx_buffer[idx];
        }
        display_raw[copy_len] = '\0';
        display_raw_len = copy_len;
    }

    /* 处理所有新数据 */
    while (read_cnt < write_cnt) {
        uint16_t idx = read_cnt % CAMERA_RX_BUFFER_SIZE;
        uint8_t data = camera_rx_buffer[idx];
        read_cnt++;

        /* 检查换行符 */
        if (data == '\n' || data == '\r') {
            if (raw_data_len > 0) {
                raw_data[raw_data_len] = '\0';
                rx_count++;

                /* 解析数据 */
                const char* space = memchr(raw_data, ' ', raw_data_len);
                if (space != NULL) {
                    size_t len1 = space - raw_data;
                    if (len1 < sizeof(color1)) {
                        strncpy(color1, raw_data, len1);
                        color1[len1] = '\0';
                    }
                    size_t len2 = raw_data_len - len1 - 1;
                    if (len2 > 0 && len2 < sizeof(color2)) {
                        strncpy(color2, space + 1, len2);
                        color2[len2] = '\0';
                    }
                    color_valid = 1;
                } else {
                    if (raw_data_len < sizeof(color1)) {
                        strncpy(color1, raw_data, raw_data_len);
                        color1[raw_data_len] = '\0';
                    }
                    memset(color2, 0, sizeof(color2));
                    color_valid = 1;
                }
            }
            raw_data_len = 0;
        } else {
            if (raw_data_len < sizeof(raw_data) - 1) {
                raw_data[raw_data_len++] = data;
            }
        }
    }
}

/**
 * @brief 获取第一个颜色
 */
const char* Camera_GetColor1(void)
{
    return color1;
}

/**
 * @brief 获取第二个颜色
 */
const char* Camera_GetColor2(void)
{
    return color2;
}

/**
 * @brief 检查颜色数据是否有效
 */
uint8_t Camera_IsValid(void)
{
    return color_valid;
}

/**
 * @brief 获取原始接收数据
 */
const char* Camera_GetRawData(void)
{
    return display_raw;
}

/**
 * @brief 获取原始数据长度
 */
uint32_t Camera_GetRawDataLen(void)
{
    return display_raw_len;
}

/**
 * @brief 获取接收计数
 */
uint32_t Camera_GetRxCount(void)
{
    return rx_count;
}

/**
 * @brief 发送颜色数据到蓝牙
 */
void Camera_SendToBluetooth(UART_HandleTypeDef* huart)
{
    if (!color_valid) return;

    char tx_buf[48];
    int len = sprintf(tx_buf, "color:%s %s\r\n", color1, color2);
    HAL_UART_Transmit(huart, (uint8_t*)tx_buf, len, 100);
}

/**
 * @brief 获取调试信息
 */
uint8_t Camera_GetDebugInfo(uint32_t* pRxCount, uint32_t* pRawLen, uint8_t* pValid)
{
    if (pRxCount) *pRxCount = rx_count;
    if (pRawLen) *pRawLen = display_raw_len;  /* 使用保存的原始数据长度 */
    if (pValid) *pValid = color_valid;
    return idle_flag;
}

/**
 * @brief 强制重新初始化接收
 */
void Camera_Reset(void)
{
    memset(camera_rx_buffer, 0, CAMERA_RX_BUFFER_SIZE);
    memset(raw_data, 0, sizeof(raw_data));
    memset(display_raw, 0, sizeof(display_raw));
    raw_data_len = 0;
    display_raw_len = 0;
    idle_flag = 0;
    read_cnt = 0;
    write_cnt = 0;
    last_write_idx = 0;
    HAL_UART_Receive_DMA(&huart3, camera_rx_buffer, CAMERA_RX_BUFFER_SIZE);
}
