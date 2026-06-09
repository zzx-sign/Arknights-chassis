/**
 * @file config.h
 * @brief 项目配置头文件
 *
 * 包含机械参数、PID参数、宏定义等
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <stdint.h>
#include <stdbool.h>

/*============================================
 * 版本信息
 *============================================*/
#define VERSION "v1.0.0"
#define PROJECT_NAME "Arknights-Chassis"

/*============================================
 * 机械参数 (需要根据实际测量调整)
 *============================================*/

/* 轮子参数 */
#define WHEEL_DIAMETER_MM      127.0f      /* 轮子直径 mm */
#define WHEEL_RADIUS_M         (WHEEL_DIAMETER_MM / 2000.0f)  /* 轮子半径 m */

/* 麦轮底盘轮距参数 (用户提供: 365.5mm) */
#define WHEELBASE_DISTANCE_MM  365.5f      /* 前后轮中心距 mm */
#define WHEELBASE_HALF_A       (WHEELBASE_DISTANCE_MM / 4.0f)  /* 半前后轮距 m */
#define WHEELBASE_HALF_B       (WHEELBASE_DISTANCE_MM / 4.0f)  /* 半左右轮距 m */
#define WHEELBASE_ROTATION_RADIUS_MM ((WHEELBASE_HALF_A + WHEELBASE_HALF_B) * 1000.0f)  /* 等效旋转半径 mm */

/* 编码器参数 */
#define ENCODER_PPR            3692         /* 编码器每转脉冲数 (需确认) */
#define ENCODER_STANDARD_PPR   3692        /* 标准一圈脉冲数 */

/*============================================
 * PWM与速度映射表 (需要标定)
 *============================================*/
#define MOTOR_SPEED_TABLE_SIZE  7

/* PWM限幅 */
#define MAX_PWM                 3600        /* 最大PWM，约为Period的50%以保留死区 */
#define SPEED_COMPENSATION      1.0f        /* 速度补偿系数 */

/*============================================
 * 常用宏
 *============================================*/
#define max(a, b)               ((a) > (b) ? (a) : (b))
#define min(a, b)               ((a) < (b) ? (a) : (b))
#define limit(x, a, b)          min(max(x, a), b)

/* 16位整数绝对值 */
static inline int16_t abs16(int16_t data_in)
{
    return data_in < 0 ? -data_in : data_in;
}

/* 线性插值 */
static inline float lerpf(float a, float b, float t)
{
    return a + (b - a) * t;
}

/* 范围内检测 */
#define inRange(val, a, b) (((a) <= (b)) ? ((val) >= (a) && (val) <= (b)) : ((val) >= (b) && (val) <= (a)))

/*============================================
 * 单位换算
 *============================================*/
#define ENCODE_OF_ONE_MM    (ENCODER_STANDARD_PPR / (WHEEL_DIAMETER_MM * 3.14159f))

/**
 * @brief 将每0.1秒的脉冲数换算为 mm/s
 */
static inline int32_t pulses_to_mm_per_s(int16_t pulses_per_0_1s)
{
    float mm = ((float)pulses_per_0_1s) / ENCODE_OF_ONE_MM;
    return (int32_t)(mm * 10.0f);
}

/**
 * @brief 将 mm/s 换算为每0.1秒的脉冲数
 */
static inline int16_t mm_per_s_to_pulses(int32_t mm_per_s)
{
    float pulses = ((float)mm_per_s) / 10.0f * ENCODE_OF_ONE_MM;
    if (pulses > 32767.0f) return 32767;
    if (pulses < -32768.0f) return -32768;
    return (int16_t)pulses;
}

/*============================================
 * PID默认参数
 *============================================*/
#define DEFAULT_KP             0.0f
#define DEFAULT_KI             0.0f
#define DEFAULT_KD             0.0f
#define DEFAULT_MAX_OUT         6500.0f
#define DEFAULT_DEAD_BAND       5.0f
#define DEFAULT_INTEGRAL_LIMIT 1000.0f

/* PID优化环节 */
#define PID_IMPROVE_NONE                   (0x00)
#define PID_Integral_Limit                 (0x01 << 0)  /* 积分限幅 */
#define PID_Derivative_On_Measurement      (0x01 << 1)  /* 微分先行 */
#define PID_Trapezoid_Intergral           (0x01 << 2)  /* 梯形积分 */
#define PID_DeltaT_Limit                  (0x01 << 3)  /* 时间间隔限幅 */
#define PID_OutputFilter                  (0x01 << 4)  /* 输出滤波器 */
#define PID_ChangingIntegrationRate        (0x01 << 5)  /* 变速积分 */
#define PID_DerivativeFilter               (0x01 << 6)  /* 微分滤波器 */
#define PID_Integral_Separate              (0x01 << 7)  /* 积分分离 */

/*============================================
 * 控制参数
 *============================================*/
#define MOTOR_CMD_SLEW_STEP_SPEED_LOOP     200    /* 速度环目标变化步进 */
#define MOTOR_CMD_SLEW_STEP_OPEN_LOOP      600    /* 开环目标变化步进 */
#define MOTOR_STOP_SLEW_STEP               250    /* 停止时步进 */

/* 最大速度限制 (mm/s) - omega 使用等效切向速度表示 */
#define MAX_LINEAR_SPEED_MM_S   2000.0f    /* 最大线速度 mm/s */
#define MAX_ANGULAR_SPEED_MM_S  1000.0f    /* 最大旋转等效速度 mm/s (表示旋转时的切向线速度) */

/*============================================
 * 状态类型
 *============================================*/
typedef enum
{
    CHASSIS_OK,
    CHASSIS_BUSY,
    CHASSIS_STANDBY,
    CHASSIS_EXIT
} Chassis_StatusTypeDef;

#define CHASSIS_READY  CHASSIS_OK
#define CHASSIS_BLOCKED CHASSIS_STANDBY
#define CHASSIS_RUNNING CHASSIS_BUSY

#endif /* __CONFIG_H__ */
