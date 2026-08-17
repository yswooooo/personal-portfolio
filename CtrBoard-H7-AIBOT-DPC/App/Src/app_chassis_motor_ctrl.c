/**
  ******************************************************************************
  * @file    app_chassis_motor_ctrl.c
  * @brief   LD2-RS 初始化、差速解算和遥控映射实现
  *
  * @details Init 顺序配置速度模式、速度源、加减速和零速目标；
  *          运行期非阻塞读写由 app_ld2rs_task 模块负责。
  ******************************************************************************
  */

#include "app_chassis_motor_ctrl.h"
#include "app_config.h"
#include "bsp_vofa.h"
#include "bsp_rc.h"
#include "stm32h7xx_hal.h"

ld2rs_motor_ctrl_t g_ld2rs_motor_ctrl_m1;
ld2rs_motor_ctrl_t g_ld2rs_motor_ctrl_m2;

/* 电机控制参数 — 上电一次性写入驱动器 */
motor_ctrl_param_t g_ld2rs_motor_param_m1 = {
    .speed = {
        .kp = 810u,
        .ti = 45u
    }
};
motor_ctrl_param_t g_ld2rs_motor_param_m2 = {
    .speed = {
        .kp = 810u,
        .ti = 45u
    }
};

/**
  * @brief 通过电机控制上下文写入一个 LD2-RS 寄存器。
  * @param[in] motor_ctrl 电机控制上下文。
  * @param[in] reg_addr   寄存器地址。
  * @param[in] reg_value  待写入的 16 bit 寄存器值。
  * @param[in] reg_name   寄存器名称；当前仅为调用侧保留的调试标识。
  * @return 底层 ld2_motor_write_reg() 返回的 Modbus RTU 状态。
  */
static modbus_rtu_status_t app_chassis_ld2rs_motor_ctrl_write_reg(ld2rs_motor_ctrl_t *motor_ctrl,
                                     uint16_t reg_addr,
                                     uint16_t reg_value,
                                     const char *reg_name)
{
    modbus_rtu_status_t status;

    status = ld2_motor_write_reg(motor_ctrl->ld2_motor, reg_addr, reg_value);
    return status;
}

/* Exported ld2rs functions --------------------------------------------------------*/

/** @copydoc app_chassis_ld2rs_motor_ctrl_init */
void app_chassis_ld2rs_motor_ctrl_init(ld2rs_motor_ctrl_t *motor_ctrl, ld2_motor_handle_t *ld2_motor,
                    int16_t ref_speed_rpm, uint8_t motor_id)
{
    modbus_rtu_status_t status;
    uint16_t reg_value;

    if (motor_ctrl == NULL) {
        return;
    }

    motor_ctrl->ld2_motor                = ld2_motor;
    motor_ctrl->last_speed_update_tick_ms = 0u;
    motor_ctrl->last_monitor_tick_ms     = 0u;
    motor_ctrl->ref_speed_rpm            = ref_speed_rpm;
    motor_ctrl->motor_id              = motor_id;
    motor_ctrl->is_speed_mode_ok         = 0u;
    motor_ctrl->is_zero_speed_ok         = 0u;
    motor_ctrl->is_speed_source_ok          = 0u;
    motor_ctrl->is_accel_decel_ok        = 0u;

    /* Step 1: 确认或设置速度模式 (Pr0.01 = 1) */
    status = ld2_motor_read_reg(motor_ctrl->ld2_motor,
                                 LD2_MOTOR_REG_CONTROL_MODE, &reg_value);
    if (status != MODBUS_RTU_OK) {
        return;  /* 所有 flag 为 0 */
    }

    if (reg_value != LD2_MOTOR_MODE_SPEED) {
        status = app_chassis_ld2rs_motor_ctrl_write_reg(motor_ctrl, LD2_MOTOR_REG_CONTROL_MODE,
                                  LD2_MOTOR_MODE_SPEED, "Pr0.01");
        if (status != MODBUS_RTU_OK) {
            return;
        }
    }
    motor_ctrl->is_speed_mode_ok = 1u;

    /* Step 2: 写零速 — 必须在切速度源之前，防止旧 Pr3.04 导致抽搐 */
    status = app_chassis_ld2rs_motor_ctrl_write_reg(motor_ctrl, LD2_MOTOR_REG_SPEED_TARGET, 0u, "Pr3.04");
    motor_ctrl->is_zero_speed_ok = (status == MODBUS_RTU_OK) ? 1u : 0u;

    /* Step 3: 确认或设置内部速度源 (Pr3.00 = 1) */
    status = ld2_motor_read_reg(motor_ctrl->ld2_motor,
                                 LD2_MOTOR_REG_SPEED_SOURCE, &reg_value);
    if (status != MODBUS_RTU_OK) {
        /* 非致命, 继续 */
    } else if (reg_value != LD2_MOTOR_SPEED_SRC_INTERNAL) {
        status = app_chassis_ld2rs_motor_ctrl_write_reg(motor_ctrl, LD2_MOTOR_REG_SPEED_SOURCE,
                                  LD2_MOTOR_SPEED_SRC_INTERNAL, "Pr3.00");
        if (status != MODBUS_RTU_OK) {
            return;
        }
    }
    motor_ctrl->is_speed_source_ok = 1u;

    /* Step 4: 设置加减速时间 */
    status = ld2_motor_set_accel_decel(motor_ctrl->ld2_motor,
                                        APP_MOTOR_CTRL_ACCEL_MS_PER_1000RPM,
                                        APP_MOTOR_CTRL_DECEL_MS_PER_1000RPM);
    motor_ctrl->is_accel_decel_ok = (status == MODBUS_RTU_OK) ? 1u : 0u;
}

/** @copydoc app_chassis_ld2rs_motor_ctrl_pi_init */
void app_chassis_ld2rs_motor_ctrl_pi_init(ld2_motor_handle_t *dev, const motor_ctrl_param_t *motor_param)
{
    uint16_t reg_value;

    if ((dev == NULL) || (motor_param == NULL)) {
        return;
    }

    /* 1. 写 Kp (Pr1.01) */
    if (ld2_motor_write_reg(dev, LD2_MOTOR_REG_SPEED_KP, motor_param->speed.kp) != MODBUS_RTU_OK) {
        return;
    }

    /* 2. 写 Ti (Pr1.02) */
    if (ld2_motor_write_reg(dev, LD2_MOTOR_REG_SPEED_TI, motor_param->speed.ti) != MODBUS_RTU_OK) {
        return;
    }

    /* 3. 读回 Kp 验证 */
    if (ld2_motor_read_reg(dev, LD2_MOTOR_REG_SPEED_KP, &reg_value) != MODBUS_RTU_OK) {
        return;
    }
    if (reg_value != motor_param->speed.kp) {
        return;
    }

    /* 4. 读回 Ti 验证 */
    if (ld2_motor_read_reg(dev, LD2_MOTOR_REG_SPEED_TI, &reg_value) != MODBUS_RTU_OK) {
        return;
    }
    if (reg_value != motor_param->speed.ti) {
        return;
    }
}

/** @copydoc app_chassis_ld2rs_motor_ctrl_param_read_back */
void app_chassis_ld2rs_motor_ctrl_param_read_back(ld2_motor_handle_t *dev)
{
    uint16_t reg_value;

    if (dev == NULL) {
        return;
    }

    if (ld2_motor_read_reg(dev, LD2_MOTOR_REG_BAUD_RATE, &reg_value) == MODBUS_RTU_OK) {
        dev->baud_rate_setting = reg_value;
    }
}

/* Exported diff chassis functions ----------------------------------------------*/

/** @copydoc app_diff_drive_compute */
void app_diff_drive_compute(const float *linear_velocity_mps, const float *angular_velocity_radps,
                       int16_t *left_rpm_out, int16_t *right_rpm_out)
{
    float half_track_mm = APP_CHASSIS_WHEEL_TRACK_MM / 2.0f;
    float scale;    /* 线速度 → rpm 转换系数 */
    float left_rpm;     /* 左轮转速 (rpm, 浮点) */
    float right_rpm;    /* 右轮转速 (rpm, 浮点) */

    /* 1. 运动学分解 */
    left_rpm  = (*linear_velocity_mps) * 1000.0f - (*angular_velocity_radps) * half_track_mm;
    right_rpm = (*linear_velocity_mps) * 1000.0f + (*angular_velocity_radps) * half_track_mm;

    /* 2. 线速度 → 转速 (rpm) */
    scale = (60.0f * APP_CHASSIS_GEAR_RATIO)
        / (APP_MATH_TWO_PI_F * APP_CHASSIS_WHEEL_RADIUS_MM);
    left_rpm  *= scale;
    right_rpm *= scale;

    /* 3. 等比例限幅：任一超限则两轮同步压缩 */
    app_diff_drive_limit_rpm(&left_rpm, &right_rpm, (float)APP_CHASSIS_SPEED_LIMIT_RPM);

    /* 4. 输出整型转速，叠加极性 (M1/M2 对装时方向相反) */
    *left_rpm_out  = (int16_t)left_rpm  * APP_MOTOR_CTRL_M1_POLARITY;
    *right_rpm_out = (int16_t)right_rpm * APP_MOTOR_CTRL_M2_POLARITY;
}

/** @copydoc app_diff_drive_limit_rpm */
void app_diff_drive_limit_rpm(float *left_rpm_out, float *right_rpm_out, float max_rpm)
{
    float max_abs_rpm;   /* 两轮绝对值最大值 */
    float scale;    /* 缩放系数 */

    /* 异常保护：参数为空或限制无效则清零 */
    if ((left_rpm_out == NULL) || (right_rpm_out == NULL) || (max_rpm <= 0.0f)) {
        if (left_rpm_out  != NULL) *left_rpm_out  = 0.0f;
        if (right_rpm_out != NULL) *right_rpm_out = 0.0f;
        return;
    }

    /* 取两轮绝对值较大者 */
    max_abs_rpm = fabsf(*left_rpm_out);
    if (fabsf(*right_rpm_out) > max_abs_rpm) {
        max_abs_rpm = fabsf(*right_rpm_out);
    }

    /* 无需限幅 */
    if (max_abs_rpm <= max_rpm) {
        return;
    }

    /* 等比例缩放：scale < 1，正负号自动保留 */
    if (max_abs_rpm > 1.0e-6f) {  /* 防除零 */
        scale     = max_rpm / max_abs_rpm;
        *left_rpm_out  *= scale;
        *right_rpm_out *= scale;
    }
}
/* Exported steering wheel functions ----------------------------------------------*/


/* Exported RC business functions ----------------------------------------------*/

/** @copydoc app_rc_channels_check_lost */
void app_rc_channels_check_lost(RC_Channels_t *channels)
{
    channels->now_tick = HAL_GetTick();
    if ((channels->now_tick - channels->last_tick) > BSP_RC_TIMEOUT_MS) {
        channels->lost_flag = 1u;
    } else {
        channels->lost_flag = 0u;
    }
}

/** @copydoc app_diff_chassis_motor_ctrl_rc_map_to_chassis */
void app_diff_chassis_motor_ctrl_rc_map_to_chassis(const RC_Filter_t *filter, RC_ChassisCmd_t *command)
{
    if (BSP_RC_STICK_IN_DEADZONE(filter->ch_rx, filter->ch_ry)) {
        command->fLinearVel = 0.0f;
        command->fAngularVel = 0.0f;
        return;
    }

    /* 仅输出摇杆归一化方向量 [-1, 1], 物理速度由油门决定 */
    command->fLinearVel  = filter->ch_rx / (float)BSP_RC_STICK_MAX;
    command->fAngularVel = APP_CHASSIS_CMD_POLARITY * filter->ch_ry / (float)BSP_RC_STICK_MAX;
}






