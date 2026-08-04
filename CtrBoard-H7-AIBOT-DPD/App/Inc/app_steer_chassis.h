/**
 * @file app_steer_chassis.h
 * @brief ROBSTRIDE 四轮转向底盘应用层接口。
 *
 * App 层职责：
 * - 遥控映射和底盘速度指令生成。
 * - 四轮转向运动学解算（atan2）。
 * - 安全停机判断。
 * - 10ms 任务调度。
 *
 * App 层不负责：
 * - 直接调用 HAL / FDCAN（由 BSP 层负责）。
 * - ROBSTRIDE 协议打包/解包（由 Middleware 层负责）。
 */

#ifndef APP_STEER_CHASSIS_H
#define APP_STEER_CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "bsp_can_robostride.h"
#include "bsp_rc.h"
#include "robstride_motor.h"

/* ========================================================================
   底盘几何参数
   ======================================================================== */

/** @brief 轴距半长 — x 方向(前后), 目前机体的短边 (m)。 */
#define APP_STEER_HALF_WHEEL_BASE_M             0.1675f   //a
/** @brief 轮距半长 — y 方向(左右), 目前机体长边 (m)。 */
#define APP_STEER_HALF_TRACK_WIDTH_M            0.200425f //b
/** @brief 轮半径 (m)，后续轮毂电机使用。 */
#define APP_STEER_CHASSIS_WHEEL_RADIUS_M        0.07500f
/** @brief 驱动减速比，后续轮毂电机使用。 */
#define APP_STEER_CHASSIS_DRIVE_GEAR_RATIO      1.0f


#define APP_DS_RS_UM_MOTOR_SPEED_MIN_RPM       (-200)
#define APP_DS_RS_UM_MOTOR_SPEED_MAX_RPM       (200)

/* 轮毂参数 */
#define APP_DS_RS_UM_WHEEL_RADIUS_M             (0.075f)
#define APP_DS_RS_UM_REDUCTION_RATIO            (1.0f)
#define M_PI 3.14159265358979323846f
/* 轮毂允许的最大线速度：约 1.5708 m/s */
#define APP_DS_RS_UM_HUB_SPEED_MAX_MPS                                      \
    ((float)APP_DS_RS_UM_MOTOR_SPEED_MAX_RPM * 2.0f * (float)M_PI           \
     * APP_DS_RS_UM_WHEEL_RADIUS_M                                          \
     / (60.0f * APP_DS_RS_UM_REDUCTION_RATIO))

/* ========================================================================
   速度限幅
   ======================================================================== */

/** @brief 遥控映射后的最大 x 方向速度 (m/s)。 */
#define APP_STEER_CHASSIS_MAX_VX_MPS            1.5708f
/** @brief 遥控映射后的最大 y 方向速度 (m/s)。 */
#define APP_STEER_CHASSIS_MAX_VY_MPS            1.5708f
/** @brief 遥控映射后的最大 yaw 角速度 (rad/s)。 */
#define APP_STEER_CHASSIS_MAX_WZ_RAD_S           2.1654f

/* ========================================================================
   舵向安全摆角限幅
   ======================================================================== */

/** @brief 舵向折叠限幅: [-PI/2, +PI/2] rad。
    atan2f 全圆输出超出此范围时，折叠到对侧并反转轮毂电机补偿方向。 */
#define APP_STEER_ANGLE_FOLD_LIMIT_RAD          1.570796327f   /* PI/2 */

/* ========================================================================
   任务周期
   ======================================================================== */

/* ========================================================================
   舵向电机软件零偏 (上板实测后修改)
   ======================================================================== */

#define APP_STEER_FL_ZERO_OFFSET_RAD            0.0316486359f
#define APP_STEER_RL_ZERO_OFFSET_RAD            -0.03663452f
#define APP_STEER_RR_ZERO_OFFSET_RAD            0.00556182861f
#define APP_STEER_FR_ZERO_OFFSET_RAD            -0.015919196854f

/* ========================================================================
   API 函数
   ======================================================================== */
/**
 * @brief 归零四个舵向模块
 */
void app_steer_chassis_zero_offset_init(steer_chassis_t *chassis);

/**
 * @brief 初始化四个舵向模块并执行 CSP 启动流程。
 */
void app_steer_chassis_init(void);

/**
 * @brief 遥控输入 → 右手系 vx/vy/wz → 四轮运动学 → 发送。
 *
 * RC 映射 + 逆运动学 + 发送 合并为一个函数。
 *
 * @param filter 滤波后的遥控输入，NULL 时忽略。
 */
void app_steer_chassis_run(const RC_Filter_t *filter);

void app_steer_chassis_rc_control(const RC_Filter_t *filter);

/**
 * @brief 串口直接速度控制（绕过 RC 映射，直接给定 vx/vy/wz）。
 * @param vx_mps   X 轴线速度 (m/s)
 * @param vy_mps   Y 轴线速度 (m/s)
 * @param wz_rad_s Z 轴角速度 (rad/s)
 */
void app_steer_chassis_run_direct(float vx_mps, float vy_mps, float wz_rad_s);

/**
 * @brief 获取当前四轮转向底盘状态的只读指针。
 * @return 静态底盘状态指针。
 */
const steer_chassis_t *app_steer_chassis_get_state(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_STEER_CHASSIS_H */
