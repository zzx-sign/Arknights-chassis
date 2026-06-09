/**
 * @file key.c
 * @brief 按键处理模块
 *
 * 使用板载按键手动控制底盘
 */

#include "key.h"
#include "gpio.h"
#include "cmsis_os.h"
#include "motor.h"
#include "kinematics.h"

/*============================================
 * 常量定义
 *============================================*/
#define KEY_CHECK_INTERVAL_MS    20      /* 按键扫描间隔 */
#define KEY_DEBOUNCE_TIME_MS     10      /* 消抖时间 */

/* 按键状态 */
typedef enum
{
    KEY_STATE_IDLE = 0,      /* 空闲 */
    KEY_STATE_PRESSED,       /* 按下 */
    KEY_STATE_SHORT_PRESS,    /* 短按释放 */
    KEY_STATE_LONG_PRESS,     /* 长按 */
    KEY_STATE_RELEASED        /* 释放 */
} KeyState_t;

/*============================================
 * 按键定义
 *============================================*/
typedef struct
{
    GPIO_TypeDef* port;      /* GPIO端口 */
    uint16_t pin;            /* GPIO引脚 */
    KeyState_t state;        /* 当前状态 */
    uint32_t press_time;     /* 按下时刻 */
    uint32_t last_check;     /* 上次检测时间 */
} Key_t;

/*============================================
 * 静态变量
 *============================================*/
static Key_t keys[KEY_COUNT];

static uint8_t key_init_flag = 0;

/*============================================
 * 内部函数
 *============================================*/

/**
 * @brief 读取按键电平
 */
static uint8_t key_read_pin(Key_t* key)
{
    return HAL_GPIO_ReadPin(key->port, key->pin) == GPIO_PIN_RESET ? 1 : 0;
}

/**
 * @brief 处理单个按键
 */
static void key_process(Key_t* key)
{
    uint32_t now = osKernelGetTickCount();
    uint8_t current_level = key_read_pin(key);

    switch (key->state) {
        case KEY_STATE_IDLE:
            if (current_level == 1) {
                key->state = KEY_STATE_PRESSED;
                key->press_time = now;
            }
            break;

        case KEY_STATE_PRESSED:
            if (current_level == 1) {
                if (now - key->press_time > KEY_DEBOUNCE_TIME_MS) {
                    key->state = KEY_STATE_LONG_PRESS;
                }
            } else {
                key->state = KEY_STATE_RELEASED;
            }
            break;

        case KEY_STATE_LONG_PRESS:
            if (current_level == 0) {
                key->state = KEY_STATE_RELEASED;
            }
            break;

        case KEY_STATE_RELEASED:
            if (now - key->press_time < 300) {
                /* 短按 */
                key->state = KEY_STATE_SHORT_PRESS;
            } else {
                /* 长按释放 */
                key->state = KEY_STATE_IDLE;
            }
            break;

        case KEY_STATE_SHORT_PRESS:
            key->state = KEY_STATE_IDLE;
            break;

        default:
            key->state = KEY_STATE_IDLE;
            break;
    }
}

/*============================================
 * 公共函数
 *============================================*/

/**
 * @brief 初始化按键模块
 */
void KEY_Init(void)
{
    if (key_init_flag) return;

    /* 初始化按键列表 */
    keys[KEY_ID_0].port = KEY0_GPIO_Port;
    keys[KEY_ID_0].pin = KEY0_Pin;
    keys[KEY_ID_0].state = KEY_STATE_IDLE;

    keys[KEY_ID_1].port = KEY1_GPIO_Port;
    keys[KEY_ID_1].pin = KEY1_Pin;
    keys[KEY_ID_1].state = KEY_STATE_IDLE;

    key_init_flag = 1;
}

/**
 * @brief 扫描按键 (定时调用)
 */
void KEY_Scan(void)
{
    for (int i = 0; i < KEY_COUNT; i++) {
        key_process(&keys[i]);
    }
}

/**
 * @brief 检查按键是否短按
 */
uint8_t KEY_IsShortPress(enum KeyId key_id)
{
    if (key_id >= KEY_COUNT) return 0;
    return keys[key_id].state == KEY_STATE_SHORT_PRESS;
}

/**
 * @brief 检查按键是否长按
 */
uint8_t KEY_IsLongPress(enum KeyId key_id)
{
    if (key_id >= KEY_COUNT) return 0;
    return keys[key_id].state == KEY_STATE_LONG_PRESS;
}

/**
 * @brief 检查按键是否按下 (包含长按和短按)
 */
uint8_t KEY_IsPressed(enum KeyId key_id)
{
    if (key_id >= KEY_COUNT) return 0;
    return (keys[key_id].state == KEY_STATE_LONG_PRESS) ||
           (keys[key_id].state == KEY_STATE_PRESSED);
}
