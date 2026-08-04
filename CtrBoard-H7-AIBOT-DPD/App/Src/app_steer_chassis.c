/**
 * @file app_steer_chassis.c
 * @brief ROBSTRIDE 四轮转向底盘应用层实现。
 */

#include "app_steer_chassis.h"
#include "app_wheel_task.h"
#include "bsp_rc.h"
#include <math.h>
#include "stm32h7xx_hal.h"

/** @brief ROBSTRIDE 四轮转向底盘的静态状态对象。 */
steer_chassis_t s_robstride_steer_chassis;

/* ========================================================================
   内部工具函数
   ======================================================================== */

static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) { return min_value; }
    if (value > max_value) { return max_value; }
    return value;
}

static float deadzone(float value)
{
    if ((value >= -BSP_RC_STICK_DEADZONE) && (value <= BSP_RC_STICK_DEADZONE)) return 0.0f;
    return value;
}

/**
 * @brief 将 atan2f 全圆输出 [-PI, +PI] 折叠到半圆 [-PI/2, +PI/2]。
 *
 * 折叠规则:
 *   raw >  +PI/2  →  folded = raw - PI,   reverse = 1 (轮毂反转)
 *   raw <  -PI/2  →  folded = raw + PI,   reverse = 1
 *   否则         →  folded = raw,         reverse = 0
 *
 * 折叠后角度与原始角度物理指向相同（车轮旋转对称），通过轮毂反转补偿方向。
 *
 * @param raw_angle    atan2f(vy, vx) 原始输出，范围 [-PI, +PI]。
 * @param reverse_flag [out] 轮毂反转标志: 0=正转, 1=反转。
 * @return 折叠后的舵向角，范围 [-PI/2, +PI/2]。
 */
static float steer_angle_fold(float raw_angle, uint8_t *reverse_flag)
{
    if (raw_angle > APP_STEER_ANGLE_FOLD_LIMIT_RAD) {
        *reverse_flag = 1U;
        return raw_angle - (APP_STEER_ANGLE_FOLD_LIMIT_RAD * 2.0f);
    } else if (raw_angle < -APP_STEER_ANGLE_FOLD_LIMIT_RAD) {
        *reverse_flag = 1U;
        return raw_angle + (APP_STEER_ANGLE_FOLD_LIMIT_RAD * 2.0f);
    } else {
        *reverse_flag = 0U;
        return raw_angle;
    }
}

/**
 * @brief 将遥控输入[0,1600]线性映射到[0,max_vel]。
 *
 * @param max_vel 最大输出速度。
 * @param rc_var  遥控输入值，期望范围为[0,1600]。
 *
 * @return 缩放后的速度，范围为[0,max_vel]。
 */
static float app_rc_var_to_velocity(float max_vel, float rc_var)
{
    float speed_scale;

    /* 最大速度非法保护 */
    if (max_vel <= 0.0f)
    {
        return 0.0f;
    }
    else
    {
        /* 继续处理 */
    }

    /* 遥控输入下限保护 */
    if (rc_var <= 0.0f)
    {
        rc_var = 0.0f;
    }
    /* 遥控输入上限保护 */
    else if (rc_var >= BSP_RC_VAR_MAX_VALUE)
    {
        rc_var = BSP_RC_VAR_MAX_VALUE;
    }
    else
    {
        /* 输入处于正常范围，不处理 */
    }

    /* 得到范围为[0,1]的缩放比例 */
    speed_scale = rc_var / BSP_RC_VAR_MAX_VALUE;

    /* 返回范围为[0,max_vel]的速度 */
    return max_vel * speed_scale;
}
/* ========================================================================
   四舵轮逆运动学（右手系，显式展开）
   ========================================================================

   坐标系: x+=前(轴距,短边) y+=左(轮距,长边) wz+=逆时针

   坐标:
     FL: x=+base/2, y=+track/2    RL: x=-base/2, y=+track/2
     RR: x=-base/2, y=-track/2    FR: x=+base/2, y=-track/2

   公式:
     vx_i = vx - wz * y_i
     vy_i = vy + wz * x_i
     angle = atan2f(vy_i, vx_i)
   ======================================================================== */

/**
 * @brief 遥控 → 右手系 vx/vy/wz → 四轮运动学 → 叠加零偏 → 发送。
 *
 * 遥控映射:
 *   ch_lx>0 (上推) → vx>0 (前进)
 *   ch_ly>0 (右推) → vy<0 (右手系左移为正)
 *   ch_ry>0 (右推) → wz<0 (右手系逆时针为正)
 * */
/* ========================================================================
   核心：四轮逆运动学执行（RC 映射与安全停机已在上层完成）
   ======================================================================== */
static void steer_chassis_execute(float vx, float vy, float wz)
{
    /* ---- 角速度分量 ---- */
    float fl_vx, fl_vy, rl_vx, rl_vy, rr_vx, rr_vy, fr_vx, fr_vy;
    float fl_angle_rad, rl_angle_rad, rr_angle_rad, fr_angle_rad;
    float fl_hub_speed_mps, rl_hub_speed_mps, rr_hub_speed_mps, fr_hub_speed_mps;
    uint8_t fl_hub_reverse, rl_hub_reverse, rr_hub_reverse, fr_hub_reverse;

    /* 1. 四轮逆运动学（显式） */

    /* FL: x=+base/2, y=+track/2 */
    fl_vx = vx - wz * APP_STEER_HALF_TRACK_WIDTH_M;
    fl_vy = vy + wz * APP_STEER_HALF_WHEEL_BASE_M;

    /* RL: x=-base/2, y=+track/2 */
    rl_vx = vx - wz * APP_STEER_HALF_TRACK_WIDTH_M;
    rl_vy = vy - wz * APP_STEER_HALF_WHEEL_BASE_M;

    /* RR: x=-base/2, y=-track/2 */
    rr_vx = vx + wz * APP_STEER_HALF_TRACK_WIDTH_M;
    rr_vy = vy - wz * APP_STEER_HALF_WHEEL_BASE_M;

    /* FR: x=+base/2, y=-track/2 */
    fr_vx = vx + wz * APP_STEER_HALF_TRACK_WIDTH_M;
    fr_vy = vy + wz * APP_STEER_HALF_WHEEL_BASE_M;

    /* 2. 四轮: 速度幅值 + atan2f + 角度折叠 + 轮毂反转标志 */

    /* ---- FL ---- */
    fl_hub_speed_mps = sqrtf(fl_vx * fl_vx + fl_vy * fl_vy);
    if (fl_vy == 0.0f && fl_vx == 0.0f) {
        fl_angle_rad   = 0.0f;
        fl_hub_reverse = 0U;
    } else {
        float fl_raw = atan2f(fl_vy, fl_vx);
        fl_angle_rad = steer_angle_fold(fl_raw, &fl_hub_reverse);
    }

    /* ---- RL ---- */
    rl_hub_speed_mps = sqrtf(rl_vx * rl_vx + rl_vy * rl_vy);
    if (rl_vy == 0.0f && rl_vx == 0.0f) {
        rl_angle_rad   = 0.0f;
        rl_hub_reverse = 0U;
    } else {
        float rl_raw = atan2f(rl_vy, rl_vx);
        rl_angle_rad = steer_angle_fold(rl_raw, &rl_hub_reverse);
    }

    /* ---- RR ---- */
    rr_hub_speed_mps = sqrtf(rr_vx * rr_vx + rr_vy * rr_vy);
    if (rr_vy == 0.0f && rr_vx == 0.0f) {
        rr_angle_rad   = 0.0f;
        rr_hub_reverse = 0U;
    } else {
        float rr_raw = atan2f(rr_vy, rr_vx);
        rr_angle_rad = steer_angle_fold(rr_raw, &rr_hub_reverse);
    }

    /* ---- FR ---- */
    fr_hub_speed_mps = sqrtf(fr_vx * fr_vx + fr_vy * fr_vy);
    if (fr_vy == 0.0f && fr_vx == 0.0f) {
        fr_angle_rad   = 0.0f;
        fr_hub_reverse = 0U;
    } else {
        float fr_raw = atan2f(fr_vy, fr_vx);
        fr_angle_rad = steer_angle_fold(fr_raw, &fr_hub_reverse);
    }

    /* 3. 叠加零偏 → 写入目标角（电机正角=CCW） */
    s_robstride_steer_chassis.modules[APP_STEER_MODULE_FL].robstride_target_angle_rad =
        fl_angle_rad
        + s_robstride_steer_chassis.modules[APP_STEER_MODULE_FL].steer_zero_offset_rad;

    s_robstride_steer_chassis.modules[APP_STEER_MODULE_RL].robstride_target_angle_rad =
        rl_angle_rad
        + s_robstride_steer_chassis.modules[APP_STEER_MODULE_RL].steer_zero_offset_rad;

    s_robstride_steer_chassis.modules[APP_STEER_MODULE_RR].robstride_target_angle_rad =
        rr_angle_rad
        + s_robstride_steer_chassis.modules[APP_STEER_MODULE_RR].steer_zero_offset_rad;

    s_robstride_steer_chassis.modules[APP_STEER_MODULE_FR].robstride_target_angle_rad =
        fr_angle_rad
        + s_robstride_steer_chassis.modules[APP_STEER_MODULE_FR].steer_zero_offset_rad;

    /* ---- 存储轮毂控制参数（由 app_wheel_task_run 下发到 RS485 轮毂电机） ---- */
    s_robstride_steer_chassis.modules[APP_STEER_MODULE_FL].hub_target_speed_mps = fl_hub_speed_mps;
    s_robstride_steer_chassis.modules[APP_STEER_MODULE_FL].hub_reverse_flag    = fl_hub_reverse;

    s_robstride_steer_chassis.modules[APP_STEER_MODULE_RL].hub_target_speed_mps = rl_hub_speed_mps;
    s_robstride_steer_chassis.modules[APP_STEER_MODULE_RL].hub_reverse_flag    = rl_hub_reverse;

    s_robstride_steer_chassis.modules[APP_STEER_MODULE_RR].hub_target_speed_mps = rr_hub_speed_mps;
    s_robstride_steer_chassis.modules[APP_STEER_MODULE_RR].hub_reverse_flag    = rr_hub_reverse;

    s_robstride_steer_chassis.modules[APP_STEER_MODULE_FR].hub_target_speed_mps = fr_hub_speed_mps;
    s_robstride_steer_chassis.modules[APP_STEER_MODULE_FR].hub_reverse_flag    = fr_hub_reverse;

    /* 度数也更新 */
    s_robstride_steer_chassis.modules[APP_STEER_MODULE_FL].robstride_target_angle_deg =
        s_robstride_steer_chassis.modules[APP_STEER_MODULE_FL].robstride_target_angle_rad * 57.29578f;
    s_robstride_steer_chassis.modules[APP_STEER_MODULE_RL].robstride_target_angle_deg =
        s_robstride_steer_chassis.modules[APP_STEER_MODULE_RL].robstride_target_angle_rad * 57.29578f;
    s_robstride_steer_chassis.modules[APP_STEER_MODULE_RR].robstride_target_angle_deg =
        s_robstride_steer_chassis.modules[APP_STEER_MODULE_RR].robstride_target_angle_rad * 57.29578f;
    s_robstride_steer_chassis.modules[APP_STEER_MODULE_FR].robstride_target_angle_deg =
        s_robstride_steer_chassis.modules[APP_STEER_MODULE_FR].robstride_target_angle_rad * 57.29578f;

    /* 4. 发送：先 CAN 舵向位置指令，再 RS485 轮毂速度指令 */
    extern volatile uint8_t s_steerwheel_chassis_task_20ms_flag;
    if (s_steerwheel_chassis_task_20ms_flag == 1U)
    {
        extern volatile uint8_t GPIOA_PIN0_FLAG;
        s_steerwheel_chassis_task_20ms_flag = 0U;
        HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_0);
        GPIOA_PIN0_FLAG = !GPIOA_PIN0_FLAG;
        bsp_can_robostride_send_steer_chassis_position(&s_robstride_steer_chassis);
    }

    app_wheel_task_run(&s_robstride_steer_chassis);
}

/* ========================================================================
   遥控映射 → 运动学
   ======================================================================== */
void app_steer_chassis_run(const RC_Filter_t *filter)
{
    float ch_lx, ch_ly, ch_ry, ch_rx;
    float vx, vy, wz;

    if (filter == NULL) return;

    /* 1. 遥控(原左手系)输入 → 标准右手系 vx/vy/wz */
    ch_lx = deadzone(filter->ch_lx);
    ch_ly = deadzone(filter->ch_ly);
    ch_ry = deadzone(filter->ch_ry);
    ch_rx = deadzone(filter->ch_rx);

    float tmp_steer_chassis_max_vx_mps;
    float tmp_steer_chassis_max_vy_mps;
    float tmp_steer_chassis_max_wz_rad_s;

    extern RC_Channels_t     g_rc;

    tmp_steer_chassis_max_vx_mps = app_rc_var_to_velocity(APP_STEER_CHASSIS_MAX_VX_MPS,g_rc.vra);
    tmp_steer_chassis_max_vy_mps = app_rc_var_to_velocity(APP_STEER_CHASSIS_MAX_VY_MPS,g_rc.vra);
    tmp_steer_chassis_max_wz_rad_s = app_rc_var_to_velocity(APP_STEER_CHASSIS_MAX_WZ_RAD_S,g_rc.vrb);

    vx =  clampf(ch_rx / (float)BSP_RC_STICK_MAX, -1.0f, 1.0f)
          * tmp_steer_chassis_max_vx_mps;

    vy = clampf(-ch_ly / (float)BSP_RC_STICK_MAX, -1.0f, 1.0f)
          * tmp_steer_chassis_max_vy_mps;

    wz = clampf(-ch_ry / (float)BSP_RC_STICK_MAX, -1.0f, 1.0f)
          * tmp_steer_chassis_max_wz_rad_s;

    /* 2. 安全停机 */
    {
        extern volatile uint8_t g_emergency_stop_flag;
        extern RC_Channels_t     g_rc;

        if (g_emergency_stop_flag==1 || g_rc.lost_flag
            || (g_rc.sw_st[eRC_SW_A].curr == eRC_POS_DOWN)) {
            vx = 0.0f; vy = 0.0f; wz = 0.0f;
        }
    }

    steer_chassis_execute(vx, vy, wz);
}

/* ========================================================================
   串口直接速度控制（绕过 RC 映射，保留运动学）
   ======================================================================== */
void app_steer_chassis_run_direct(float vx_mps, float vy_mps, float wz_rad_s)
{
    extern volatile uint8_t g_emergency_stop_flag;
    extern RC_Channels_t     g_rc;

    /* 硬件急停始终生效 */
    if (g_emergency_stop_flag == 1) {
        vx_mps = 0.0f; vy_mps = 0.0f; wz_rad_s = 0.0f;
    }
    /* RC 在线时，SWA 下档也作为急停（即使处于串口控制模式） */
    if (!g_rc.lost_flag && g_rc.sw_st[eRC_SW_A].curr == eRC_POS_DOWN) {
        vx_mps = 0.0f; vy_mps = 0.0f; wz_rad_s = 0.0f;
    }

    steer_chassis_execute(vx_mps, vy_mps, wz_rad_s);
}

/* ========================================================================
   兼容旧接口 / 测试接口
   ======================================================================== */

void app_steer_chassis_rc_control(const RC_Filter_t *filter)
{
    app_steer_chassis_run(filter);
}


/* ========================================================================
   零偏 / 初始化 / 状态
   ======================================================================== */

void app_steer_chassis_zero_offset_init(steer_chassis_t *chassis)
{
    chassis->modules[APP_STEER_MODULE_FL].steer_zero_offset_rad =
        APP_STEER_FL_ZERO_OFFSET_RAD;
    chassis->modules[APP_STEER_MODULE_RL].steer_zero_offset_rad =
        APP_STEER_RL_ZERO_OFFSET_RAD;
    chassis->modules[APP_STEER_MODULE_RR].steer_zero_offset_rad =
        APP_STEER_RR_ZERO_OFFSET_RAD;
    chassis->modules[APP_STEER_MODULE_FR].steer_zero_offset_rad =
        APP_STEER_FR_ZERO_OFFSET_RAD;
}

void app_steer_chassis_init(void)
{
    bsp_can_robostride_init_steer_chassis(&s_robstride_steer_chassis);
    app_steer_chassis_zero_offset_init(&s_robstride_steer_chassis);
    bsp_can_robostride_start_steer_chassis(&s_robstride_steer_chassis);
}

const steer_chassis_t *app_steer_chassis_get_state(void)
{
    return &s_robstride_steer_chassis;
}
