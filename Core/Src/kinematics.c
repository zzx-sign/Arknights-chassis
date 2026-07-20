/**
 * @file kinematics.c
 * @brief 麦克纳姆轮运动学实现
 */

#include "kinematics.h"
#include <math.h>

/*============================================
 * 静态变量
 *============================================*/

/* 目标底盘速度 */
static ChassisSpeed_t target_chassis_speed;

/* 当前底盘速度 */
static ChassisSpeed_t current_chassis_speed;

/* 底盘位置 */
static ChassisPose_t chassis_pose;

/* 目标轮子速度 (mm/s) */
static WheelSpeedMm_t target_wheel_speed_mm;

/*============================================
 * 公共函数
 *============================================*/

/**
 * @brief 运动学逆解: 底盘速度 -> 轮子角速度
 *
 * 逆运动学矩阵:
 * [W0]   [ 1  -1  -(a+b)] [vx ]
 * [W1] = [ 1   1   (a+b)] [vy ]
 * [W2]   [ 1  -1   (a+b)] [omega]
 * [W3]   [ 1   1  -(a+b)] [   ]
 *
 * Wi = (vx +/- vy +/- (a+b)*omega) / r
 */
void Kinematics_SolveInverse(ChassisSpeed_t* chassis, WheelSpeedRad_t* wheel)
{
    float vx = chassis->vx;
    float vy = chassis->vy;
    float omega = chassis->omega;

    float a_plus_b = WHEELBASE_HALF_A + WHEELBASE_HALF_B;
    float r = WHEEL_RADIUS_M;

    /* 修正：前进时所有轮子同向 */
    wheel->left_front  = ( vx - vy - a_plus_b * omega) / r;
    wheel->right_front = (-vx - vy + a_plus_b * omega) / r;
    wheel->left_back   = (-
            vx - vy - a_plus_b * omega) / r;
    wheel->right_back  = ( vx - vy + a_plus_b * omega) / r;
}

/**
 * @brief 运动学正解: 轮子角速度 -> 底盘速度
 *
 * 正运动学矩阵:
 * vx = (W0 + W1 + W2 + W3) * r / 4
 * vy = (-W0 + W1 - W2 + W3) * r / 4
 * omega = (-W0 + W1 + W2 - W3) * r / (4*(a+b))
 */
void Kinematics_SolveForward(WheelSpeedRad_t* wheel, ChassisSpeed_t* chassis)
{
    float r = WHEEL_RADIUS_M;
    float a_plus_b = WHEELBASE_HALF_A + WHEELBASE_HALF_B;

    chassis->vx = (wheel->left_front - wheel->right_front -
                   wheel->left_back + wheel->right_back) * r / 4.0f;

    chassis->vy = (wheel->left_front + wheel->right_front +
                  wheel->left_back + wheel->right_back) * r / 4.0f;

    chassis->omega = (wheel->left_front + wheel->right_front -
                     wheel->left_back - wheel->right_back) * r / (4.0f * a_plus_b);
}

/**
 * @brief 限幅底盘速度
 */
void Kinematics_LimitSpeed(ChassisSpeed_t* chassis)
{
    /* 限幅线速度 */
    float max_linear = MAX_LINEAR_SPEED_MM_S / 1000.0f;
    float speed = sqrtf(chassis->vx * chassis->vx + chassis->vy * chassis->vy);
    if (speed > max_linear) {
        float scale = max_linear / speed;
        chassis->vx *= scale;
        chassis->vy *= scale;
    }

    /* 限幅角速度 (内部是 rad/s，限幅值需要转换) */
    /* max_omega_rad = MAX_ANGULAR_SPEED_MM_S / WHEELBASE_ROTATION_RADIUS_MM */
    float max_omega = MAX_ANGULAR_SPEED_MM_S / WHEELBASE_ROTATION_RADIUS_MM;
    if (chassis->omega > max_omega) {
        chassis->omega = max_omega;
    } else if (chassis->omega < -max_omega) {
        chassis->omega = -max_omega;
    }
}

/**
 * @brief 设置底盘目标速度
 * @param vx X方向速度 mm/s
 * @param vy Y方向速度 mm/s
 * @param omega_mm_s 旋转等效速度 mm/s (等效切向线速度)
 */
void Kinematics_SetTarget(float vx, float vy, float omega_mm_s)
{
    /* 将 mm/s 转换为内部使用的 m/s */
    target_chassis_speed.vx = vx / 1000.0f;
    target_chassis_speed.vy = vy / 1000.0f;

    /* 将旋转等效速度 (mm/s) 转换为角速度 (rad/s) */
    /* omega_rad = omega_mm / rotation_radius_mm */
    target_chassis_speed.omega = omega_mm_s / WHEELBASE_ROTATION_RADIUS_MM;

    /* 限幅 */
    Kinematics_LimitSpeed(&target_chassis_speed);

    /* 计算轮子目标速度 */
    WheelSpeedRad_t wheel_rad;
    Kinematics_SolveInverse(&target_chassis_speed, &wheel_rad);

    /* 转换为 mm/s */
    float w_mm[4];
    w_mm[0] = wheel_rad.left_front  * 1000.0f * WHEEL_RADIUS_M;
    w_mm[1] = wheel_rad.right_front * 1000.0f * WHEEL_RADIUS_M;
    w_mm[2] = wheel_rad.left_back   * 1000.0f * WHEEL_RADIUS_M;
    w_mm[3] = wheel_rad.right_back  * 1000.0f * WHEEL_RADIUS_M;

    /* 麦轮 IK 非保模，几何上 wheel_speed = vx ± vy ± (a+b)/r * omega，
     * 极限拉杆时单轮需求可达线速度上限的 ~3 倍。
     * 若不缩放，PID 会在某轮先饱和，导致实际运动方向偏离摇杆方向。
     * 在转 int16 之前做等比缩放，保证运动方向不变。 */
    float max_abs = 0.0f;
    for (int i = 0; i < 4; i++) {
        float a = w_mm[i] < 0.0f ? -w_mm[i] : w_mm[i];
        if (a > max_abs) max_abs = a;
    }
    float scale = 1.0f;
    if (max_abs > MAX_WHEEL_SPEED_MM_S) {
        scale = MAX_WHEEL_SPEED_MM_S / max_abs;
        for (int i = 0; i < 4; i++) w_mm[i] *= scale;
    }

    target_wheel_speed_mm.left_front  = (int16_t)w_mm[0];
    target_wheel_speed_mm.right_front = (int16_t)w_mm[1];
    target_wheel_speed_mm.left_back   = (int16_t)w_mm[2];
    target_wheel_speed_mm.right_back  = (int16_t)w_mm[3];
}

/**
 * @brief 获取当前底盘速度
 */
ChassisSpeed_t* Kinematics_GetCurrentSpeed(void)
{
    return &current_chassis_speed;
}

/**
 * @brief 更新底盘位置 (积分)
 */
void Kinematics_UpdatePosition(float dt)
{
    /* 更新线速度 (低通滤波) */
    float alpha = 0.3f;
    current_chassis_speed.vx = alpha * target_chassis_speed.vx + (1 - alpha) * current_chassis_speed.vx;
    current_chassis_speed.vy = alpha * target_chassis_speed.vy + (1 - alpha) * current_chassis_speed.vy;
    current_chassis_speed.omega = alpha * target_chassis_speed.omega + (1 - alpha) * current_chassis_speed.omega;

    /* 积分位置 */
    float v = sqrtf(current_chassis_speed.vx * current_chassis_speed.vx +
                    current_chassis_speed.vy * current_chassis_speed.vy);

    chassis_pose.x += current_chassis_speed.vx / 1000.0f * dt;
    chassis_pose.y += current_chassis_speed.vy / 1000.0f * dt;
    chassis_pose.theta += current_chassis_speed.omega * dt;

    /* 角度归一化 */
    while (chassis_pose.theta > 3.14159f) chassis_pose.theta -= 2.0f * 3.14159f;
    while (chassis_pose.theta < -3.14159f) chassis_pose.theta += 2.0f * 3.14159f;
}

/**
 * @brief 获取当前底盘位置
 */
ChassisPose_t* Kinematics_GetPose(void)
{
    return &chassis_pose;
}

/**
 * @brief 重置底盘位置
 */
void Kinematics_ResetPose(float x, float y, float theta)
{
    chassis_pose.x = x;
    chassis_pose.y = y;
    chassis_pose.theta = theta;

    target_chassis_speed.vx = 0;
    target_chassis_speed.vy = 0;
    target_chassis_speed.omega = 0;

    current_chassis_speed.vx = 0;
    current_chassis_speed.vy = 0;
    current_chassis_speed.omega = 0;
}

/**
 * @brief 获取轮子目标速度
 */
WheelSpeedMm_t* Kinematics_GetTargetWheelSpeed(void)
{
    return &target_wheel_speed_mm;
}

/**
 * @brief 获取底盘目标速度
 */
ChassisSpeed_t* Kinematics_GetTargetSpeed(void)
{
    return &target_chassis_speed;
}
