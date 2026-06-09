/**
 * @file motor.c
 * @brief 麦轮底盘电机驱动实现
 */

#include "motor.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdlib.h>

#include "tim.h"

/*============================================
 * 静态变量
 *============================================*/

/* 电机控制块 */
static MotorControlBlock_t motor_cb[MOTOR_COUNT];

/* 全局状态 */
static ChassisMotorState_t motor_state;

/* 目标速度缓存 (用于斜坡变化) */
static int16_t target_speed_cache[MOTOR_COUNT];
static int16_t applied_speed[MOTOR_COUNT];

/* 缓停标志 */
static uint8_t smooth_stop_flag = 0;

/*============================================
 * 编码器方向配置 (需要根据实际调整)
 *============================================*/
static const int8_t encoder_sign[MOTOR_COUNT] = {
    1,    /* 左前轮: 向前为正 */
    1,    /* 右前轮: 向前为正 */
    1,    /* 左后轮: 向前为正 */
    1     /* 右后轮: 向前为正 */
};

/* 距离校正系数 (需要实测调整) */
static const float distance_coef[MOTOR_COUNT] = {
    0.90f,  /* 左前轮 */
    0.94f,  /* 右前轮 */
    0.85f,  /* 左后轮 */
    0.91f   /* 右后轮 */
};

/* 一圈编码器脉冲数 (需要实测) */
static const uint16_t encoder_ppr[MOTOR_COUNT] = {
    3692,  /* 左前轮 */
    3692,  /* 右前轮 */
    3692,  /* 左后轮 */
    3692   /* 右后轮 */
};

/*============================================
 * 内部函数
 *============================================*/

/**
 * @brief 获取系统时间 (ms)
 */
uint32_t get_system_time_ms(void)
{
    return osKernelGetTickCount();
}

/**
 * @brief 限幅
 */
static int16_t clamp_pwm(int16_t val)
{
    return limit(val, -MAX_PWM, MAX_PWM);
}

/*============================================
 * 公共函数
 *============================================*/

/**
 * @brief 设置PWM输出
 */
static void pwm_set_compare(MotorControlBlock_t* motor, int16_t pwm)
{
    TIM_HandleTypeDef* tim = motor->PWM_TIM;
    uint32_t ch = motor->pwm_channel;

    pwm = clamp_pwm((int16_t)(pwm * SPEED_COMPENSATION));

    /* 获取电机索引 (0-3) */
    uint8_t idx = motor - motor_cb;

    /* 方向引脚映射表 */
    GPIO_TypeDef* ain_port[4] = {
        Driver1AIN1_GPIO_Port, Driver2AIN1_GPIO_Port,
        Driver3AIN1_GPIO_Port, Driver4AIN1_GPIO_Port
    };
    uint16_t ain1_pin[4] = {
        Driver1AIN1_Pin, Driver2AIN1_Pin,
        Driver3AIN1_Pin, Driver4AIN1_Pin
    };
    uint16_t ain2_pin[4] = {
        Driver1AIN2_Pin, Driver2AIN2_Pin,
        Driver3AIN2_Pin, Driver4AIN2_Pin
    };

    GPIO_PinState ain1_state, ain2_state;

    /* 正转 */
    if (pwm > 0) {
        ain1_state = GPIO_PIN_SET;
        ain2_state = GPIO_PIN_RESET;
        __HAL_TIM_SET_COMPARE(tim, ch, (uint16_t)pwm);
    }
    /* 反转 */
    else if (pwm < 0) {
        ain1_state = GPIO_PIN_RESET;
        ain2_state = GPIO_PIN_SET;
        __HAL_TIM_SET_COMPARE(tim, ch, (uint16_t)(-pwm));
    }
    /* 停止 */
    else {
        ain1_state = GPIO_PIN_RESET;
        ain2_state = GPIO_PIN_RESET;
        __HAL_TIM_SET_COMPARE(tim, ch, 0);
    }

    /* 设置方向引脚 */
    HAL_GPIO_WritePin(ain_port[idx], ain1_pin[idx], ain1_state);
    HAL_GPIO_WritePin(ain_port[idx], ain2_pin[idx], ain2_state);
}

/**
 * @brief 读取编码器差值
 */
static int32_t read_encoder_delta(MotorControlBlock_t* motor)
{
    uint16_t current_cnt = (uint16_t)(__HAL_TIM_GET_COUNTER(motor->encoder_TIM));
    int32_t raw_delta = (int32_t)current_cnt - (int32_t)motor->last_encoder_cnt;

    /* 处理溢出 */
    if (raw_delta < -32768) {
        raw_delta += 65536;
    } else if (raw_delta > 32767) {
        raw_delta -= 65536;
    }

    motor->last_encoder_cnt = current_cnt;

    /* 应用编码器方向和校正系数 */
    return raw_delta * motor->encoder_sign;
}

/**
 * @brief 斜坡限幅
 */
static int16_t ramp_toward(int16_t current, int16_t target, int16_t max_step)
{
    if (target > current + max_step) {
        return current + max_step;
    }
    if (target < current - max_step) {
        return current - max_step;
    }
    return target;
}

/*============================================
 * 公共函数
 *============================================*/

/**
 * @brief 电机初始化
 */
void Motor_Init(void)
{
    /* 清零状态 */
    memset(&motor_state, 0, sizeof(ChassisMotorState_t));
    memset(target_speed_cache, 0, sizeof(target_speed_cache));
    memset(applied_speed, 0, sizeof(applied_speed));
    smooth_stop_flag = 0;

    /* 初始化电机控制块 */
    TIM_HandleTypeDef* pwm_tim = &htim1;
    uint32_t pwm_channels[MOTOR_COUNT] = {
        MOTOR_PWM_CHANNEL_1,
        MOTOR_PWM_CHANNEL_2,
        MOTOR_PWM_CHANNEL_3,
        MOTOR_PWM_CHANNEL_4
    };

    TIM_HandleTypeDef* encoder_tims[MOTOR_COUNT] = {
        MOTOR1_ENCODER_TIM,
        MOTOR2_ENCODER_TIM,
        MOTOR3_ENCODER_TIM,
        MOTOR4_ENCODER_TIM
    };

    for (int i = 0; i < MOTOR_COUNT; i++) {
        MotorControlBlock_t* motor = &motor_cb[i];

        motor->PWM_TIM = pwm_tim;
        motor->pwm_channel = pwm_channels[i];
        motor->encoder_TIM = encoder_tims[i];
        motor->encoder_sign = encoder_sign[i];
        motor->encoder_one_round = encoder_ppr[i];
        motor->distance_coefficient = distance_coef[i];
        motor->mode = MOTOR_MODE_OPEN_LOOP_PWM;
        motor->target_speed = 0;
        motor->feedback_speed = 0;
        motor->PWM = 0;
        motor->distance = 0;
        motor->first_read = 1;

        /* 初始化PID */
        PID_Init_Config_s pid_config = {
            .Kp = 2.0f,       /* 测试用 */
            .Ki = 0.1f,      /* 积分系数 - 先设为0测试 */
            .Kd = 0.0f,       /* 微分系数 */
            .MaxOut = 3000,    /* PID输出限幅 (PWM范围，需匹配新的MAX_PWM) */
            .DeadBand = 15,   /* 死区：对齐编码器最小速度台阶，避免零附近抖动 */
            .Improve = PID_OutputFilter | PID_Integral_Limit,
            .IntegralLimit = 3000,  /* 积分限幅 */
            .Intergral_Separate = 10.0f,  /* 误差超过该值才分离积分 */
            .Output_LPF_RC = 0.08f,
        };
        PID_Init(&motor->pid, &pid_config);
    }

    /* 启动PWM */
    HAL_TIM_PWM_Start(pwm_tim, MOTOR_PWM_CHANNEL_1);
    HAL_TIM_PWM_Start(pwm_tim, MOTOR_PWM_CHANNEL_2);
    HAL_TIM_PWM_Start(pwm_tim, MOTOR_PWM_CHANNEL_3);
    HAL_TIM_PWM_Start(pwm_tim, MOTOR_PWM_CHANNEL_4);

    osDelay(10);

    /* 拉高STBY引脚，使能电机驱动器 */
    HAL_GPIO_WritePin(STBY1_GPIO_Port, STBY1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(STBY2_GPIO_Port, STBY2_Pin, GPIO_PIN_SET);
}

/**
 * @brief 编码器初始化
 */
void Encoder_Init(void)
{
    TIM_HandleTypeDef* encoder_tims[MOTOR_COUNT] = {
        MOTOR1_ENCODER_TIM,
        MOTOR2_ENCODER_TIM,
        MOTOR3_ENCODER_TIM,
        MOTOR4_ENCODER_TIM
    };

    for (int i = 0; i < MOTOR_COUNT; i++) {
        HAL_TIM_Encoder_Start(encoder_tims[i], TIM_CHANNEL_ALL);
        /* 记录初始值 */
        motor_cb[i].last_encoder_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(encoder_tims[i]);
    }
}

/**
 * @brief 设置速度 (mm/s)
 */
void Motor_SetSpeed(int16_t lf, int16_t rf, int16_t lb, int16_t rb)
{
    target_speed_cache[MOTOR_ID_LEFT_FRONT] = lf;
    target_speed_cache[MOTOR_ID_RIGHT_FRONT] = rf;
    target_speed_cache[MOTOR_ID_LEFT_BACK] = lb;
    target_speed_cache[MOTOR_ID_RIGHT_BACK] = rb;

    /* 清除缓停标志 */
    if (lf != 0 || rf != 0 || lb != 0 || rb != 0) {
        smooth_stop_flag = 0;
    }

    /* 更新状态 */
    motor_state.left_front_speed = lf;
    motor_state.right_front_speed = rf;
    motor_state.left_back_speed = lb;
    motor_state.right_back_speed = rb;
}

/**
 * @brief 设置真实速度 (确保速度环模式)
 */
void Motor_SetSpeed_Real(int16_t lf, int16_t rf, int16_t lb, int16_t rb)
{
    if (Motor_GetMode() != MOTOR_MODE_SPEED_LOOP) {
        Motor_SetMode(MOTOR_MODE_SPEED_LOOP);
    }
    Motor_SetSpeed(lf, rf, lb, rb);
}

/**
 * @brief 停止电机
 */
void Motor_Stop(void)
{
    smooth_stop_flag = 0;
    Motor_SetSpeed(0, 0, 0, 0);
    for (int i = 0; i < MOTOR_COUNT; i++) {
        applied_speed[i] = 0;
    }
}

/**
 * @brief 缓停
 */
void Motor_StopSmooth(void)
{
    if (Motor_GetMode() != MOTOR_MODE_SPEED_LOOP) {
        Motor_SetMode(MOTOR_MODE_SPEED_LOOP);
    }
    smooth_stop_flag = 1;
    Motor_SetSpeed(0, 0, 0, 0);
}

/**
 * @brief 设置电机模式
 */
void Motor_SetMode(enum MotorMode mode)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_cb[i].mode = mode;
    }
}

/**
 * @brief 获取电机模式
 */
enum MotorMode Motor_GetMode(void)
{
    return motor_cb[0].mode;
}

/**
 * @brief 获取反馈速度
 */
int16_t Motor_GetFeedbackSpeed(enum MotorId motor_id)
{
    if (motor_id >= MOTOR_COUNT) return 0;
    return motor_cb[motor_id].feedback_speed;
}

/**
 * @brief 获取全局状态
 */
ChassisMotorState_t* Motor_GetState(void)
{
    return &motor_state;
}

/**
 * @brief 编码器更新
 */
void Motor_EncoderUpdate(void)
{
    static uint32_t last_time_hight = 0;
    static uint32_t last_time_low = 0;
    static uint8_t call_flag = 0;

    uint32_t now = osKernelGetTickCount();

    /* 首次调用或超时 */
    if (call_flag == 0 || (now - last_time_hight > 1000)) {
        call_flag = 1;
        last_time_hight = now;
        return;
    }

    int32_t interval = now - last_time_hight;
    if (interval <= 0) interval = 1;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        MotorControlBlock_t* motor = &motor_cb[i];

        int32_t delta = read_encoder_delta(motor);

        /* 转换为标准脉冲数 */
        float standard_pulses = (float)delta / (float)motor->encoder_one_round * ENCODER_STANDARD_PPR;

        /* 转换为距离 */
        float delta_distance_mm = standard_pulses / ENCODE_OF_ONE_MM;

        /* 更新距离 */
        motor->distance += delta_distance_mm * motor->distance_coefficient;

        /* 计算速度 (已经是脉冲/秒，不需要再*1000) */
        float pulses_per_s = standard_pulses * 1000.0f / (float)interval;
        motor->feedback_speed = (int16_t)pulses_to_mm_per_s((int16_t)(pulses_per_s / 10.0f));
    }

    last_time_hight = now;
}

/**
 * @brief 控制循环 (定时调用)
 */
void Motor_ControlLoop(void)
{
    int16_t step;

    if (smooth_stop_flag && Motor_GetMode() == MOTOR_MODE_SPEED_LOOP) {
        step = MOTOR_STOP_SLEW_STEP;
    } else if (Motor_GetMode() == MOTOR_MODE_SPEED_LOOP) {
        step = MOTOR_CMD_SLEW_STEP_SPEED_LOOP;
    } else {
        step = MOTOR_CMD_SLEW_STEP_OPEN_LOOP;
    }

    /* 斜坡逼近目标 */
    for (int i = 0; i < MOTOR_COUNT; i++) {
        applied_speed[i] = ramp_toward(applied_speed[i], target_speed_cache[i], step);
        motor_cb[i].target_speed = applied_speed[i];
    }

    /* 零目标时先缓慢收尾，避免PWM在阈值边界处被硬切到0后又恢复 */
    if (abs16(target_speed_cache[0]) <= 25 && abs16(target_speed_cache[1]) <= 25 &&
        abs16(target_speed_cache[2]) <= 25 && abs16(target_speed_cache[3]) <= 25) {
        for (int i = 0; i < MOTOR_COUNT; i++) {
            MotorControlBlock_t* motor = &motor_cb[i];
            int16_t set_pwm = 0;

            applied_speed[i] = ramp_toward(applied_speed[i], 0, MOTOR_STOP_SLEW_STEP);
            motor->target_speed = applied_speed[i];

            if (abs16(applied_speed[i]) <= 1) {
                motor->PWM = 0;
                motor->pid.Iout = 0;
                motor->pid.ITerm = 0;
                motor->pid.Output = 0;
                motor->pid.Last_Output = 0;
                pwm_set_compare(motor, 0);
            } else {
                if (motor->mode == MOTOR_MODE_OPEN_LOOP_PWM) {
                    set_pwm = motor->target_speed;
                } else {
                    set_pwm = (int16_t)PID_Calculate(&motor->pid, motor->feedback_speed, motor->target_speed);
                }

                set_pwm = clamp_pwm(set_pwm);
                motor->PWM = set_pwm;
                pwm_set_compare(motor, set_pwm);
            }
        }
    } else {
        /* PID控制 */
        for (int i = 0; i < MOTOR_COUNT; i++) {
            MotorControlBlock_t* motor = &motor_cb[i];
            int16_t set_pwm = 0;

            if (motor->mode == MOTOR_MODE_OPEN_LOOP_PWM) {
                /* 开环模式: 直接输出 */
                set_pwm = motor->target_speed;
            } else {
                /* 速度环模式: 纯PID控制 */
                set_pwm = (int16_t)PID_Calculate(&motor->pid, motor->feedback_speed, motor->target_speed);
            }

            set_pwm = clamp_pwm(set_pwm);
            motor->PWM = set_pwm;

            /* 输出PWM */
            pwm_set_compare(motor, set_pwm);
        }
    }

    /* 更新全局状态 */
    motor_state.feedback_left_front = motor_cb[MOTOR_ID_LEFT_FRONT].feedback_speed;
    motor_state.feedback_right_front = motor_cb[MOTOR_ID_RIGHT_FRONT].feedback_speed;
    motor_state.feedback_left_back = motor_cb[MOTOR_ID_LEFT_BACK].feedback_speed;
    motor_state.feedback_right_back = motor_cb[MOTOR_ID_RIGHT_BACK].feedback_speed;

    motor_state.PWM_left_front = motor_cb[MOTOR_ID_LEFT_FRONT].PWM;
    motor_state.PWM_right_front = motor_cb[MOTOR_ID_RIGHT_FRONT].PWM;
    motor_state.PWM_left_back = motor_cb[MOTOR_ID_LEFT_BACK].PWM;
    motor_state.PWM_right_back = motor_cb[MOTOR_ID_RIGHT_BACK].PWM;

    motor_state.distance_left_front = (int32_t)motor_cb[MOTOR_ID_LEFT_FRONT].distance;
    motor_state.distance_right_front = (int32_t)motor_cb[MOTOR_ID_RIGHT_FRONT].distance;
    motor_state.distance_left_back = (int32_t)motor_cb[MOTOR_ID_LEFT_BACK].distance;
    motor_state.distance_right_back = (int32_t)motor_cb[MOTOR_ID_RIGHT_BACK].distance;

    motor_state.total_distance = (motor_state.distance_left_front +
                                 motor_state.distance_right_front +
                                 motor_state.distance_left_back +
                                 motor_state.distance_right_back) / 4;
}

/**
 * @brief 单位换算: mm/s -> rad/s
 */
float Motor_mm_s_to_rad_s(int16_t mm_s)
{
    return (float)mm_s / 1000.0f / WHEEL_RADIUS_M;
}

/**
 * @brief 单位换算: rad/s -> mm/s
 */
int16_t Motor_rad_s_to_mm_s(float rad_s)
{
    return (int16_t)(rad_s * 1000.0f * WHEEL_RADIUS_M);
}
