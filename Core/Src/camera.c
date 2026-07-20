/**
 * @file camera.c
 * @brief 树莓派相机数据处理 - 颜色识别结果解析
 */

#include "camera.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"
#include "strings.h"
#include "ctype.h"
#include "stdio.h"

/* 接收缓冲区大小 - 扩大缓冲区以支持更多数据 */
#define CAMERA_RX_BUFFER_SIZE  128

/* 接收缓冲区 */
static uint8_t camera_rx_buffer[CAMERA_RX_BUFFER_SIZE];

/* 解析后的颜色数据 */
static char color1[16] = {0};
static char color2[16] = {0};
static uint8_t color_valid = 0;

/*============================================
 * 颜色 → LED 控制
 *============================================*/

/* 已点亮的颜色掩码 (bit0=Red, bit1=Blue, bit2=Yellow) */
#define COLOR_LED_RED    (1u << 0)
#define COLOR_LED_BLUE   (1u << 1)
#define COLOR_LED_YELLOW (1u << 2)

static uint8_t color_led_mask = 0;

/**
 * @brief 大小写不敏感字符串比较 (忽略空字符)
 */
static int color_strcmp_ci(const char* a, const char* b)
{
    if (a == NULL || b == NULL) return -1;
    while (*a && *b) {
        unsigned char ca = (unsigned char)*a++;
        unsigned char cb = (unsigned char)*b++;
        if (tolower(ca) != tolower(cb)) return -1;
    }
    /* 两者同时到末尾才算相等 */
    return (*a == '\0' && *b == '\0') ? 0 : -1;
}

/**
 * @brief 将一个颜色字符串映射到 LED 标志位
 * @return COLOR_LED_RED / COLOR_LED_BLUE / COLOR_LED_YELLOW, 不识别返回 0
 */
static uint8_t color_to_led_flag(const char* color)
{
    if (color == NULL || color[0] == '\0') return 0;

    if (color_strcmp_ci(color, "red")    == 0) return COLOR_LED_RED;
    if (color_strcmp_ci(color, "blue")   == 0) return COLOR_LED_BLUE;
    if (color_strcmp_ci(color, "yellow") == 0) return COLOR_LED_YELLOW;
    return 0;
}

/**
 * @brief 根据 color_led_mask 刷新 LED GPIO
 *        只有在 mask 变化时才操作 GPIO, 避免无谓翻转
 */
static void apply_color_led(uint8_t new_mask)
{
    if (new_mask == color_led_mask) return;
    color_led_mask = new_mask;

    HAL_GPIO_WritePin(LED_Red_GPIO_Port,    LED_Red_Pin,
        (new_mask & COLOR_LED_RED)    ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_Blue_GPIO_Port,   LED_Blue_Pin,
        (new_mask & COLOR_LED_BLUE)   ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_Yellow_GPIO_Port, LED_Yellow_Pin,
        (new_mask & COLOR_LED_YELLOW) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief 公共 API: 根据当前解析到的 color1/color2 刷新 LED
 *        - 任一为 red/blue/yellow 即点亮对应 LED
 *        - 同时识别到多个不同颜色, 同时点亮
 *        - 未识别到任何有效颜色, 三个 LED 全灭
 */
void Camera_UpdateLedByColor(void)
{
    uint8_t mask = 0;

    if (color_valid) {
        mask |= color_to_led_flag(color1);
        mask |= color_to_led_flag(color2);
    }

    apply_color_led(mask);
}

/**
 * @brief 复位 LED 状态 (颜色失效时调用)
 */
void Camera_ClearLeds(void)
{
    apply_color_led(0);
}

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

                /* 解析数据 - 按逗号拆成两个颜色，整帧覆盖 color1/color2 */
                const char* comma = memchr(raw_data, ',', raw_data_len);
                size_t c1_len = (comma != NULL) ? (size_t)(comma - raw_data) : raw_data_len;

                /* 第二个颜色起始位置：逗号之后第一个非空字符 */
                const char* c2_start = raw_data + c1_len;
                if (comma != NULL) {
                    c2_start = comma + 1;
                }
                size_t c2_len = raw_data_len - (size_t)(c2_start - raw_data);

                /* 写入 color1 */
                if (c1_len >= sizeof(color1)) c1_len = sizeof(color1) - 1;
                memcpy(color1, raw_data, c1_len);
                color1[c1_len] = '\0';

                /* 写入 color2（即使为空也写，保证清掉旧值）*/
                if (c2_len >= sizeof(color2)) c2_len = sizeof(color2) - 1;
                memcpy(color2, c2_start, c2_len);
                color2[c2_len] = '\0';

                color_valid = 1;
                Camera_UpdateLedByColor();
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
    color_valid = 0;
    color1[0] = '\0';
    color2[0] = '\0';
    Camera_ClearLeds();
    HAL_UART_Receive_DMA(&huart3, camera_rx_buffer, CAMERA_RX_BUFFER_SIZE);
}
