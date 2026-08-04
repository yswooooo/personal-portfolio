/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   应用层业务配置宏
  *
  * @details 集中管理所有可调业务参数。
  *          修改此文件后重新编译即可生效，无需改动其他代码。
  ******************************************************************************
  */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* LD2-RS 通信参数 -----------------------------------------------------------*/

/** @brief 电机1 Modbus 站号 (与驱动器 Pr5.28 一致) */
#define APP_LD2_MOTOR_SLAVE_ID_M1           1u

/** @brief 电机2 Modbus 站号 (与驱动器 Pr5.28 一致) */
#define APP_LD2_MOTOR_SLAVE_ID_M2           2u

/** @brief 当前参与 RS485 轮询调度的 LD2-RS 电机数量 */
#define APP_LD2_MOTOR_COUNT                 2u

/** @brief LD2-RS 电机轮询调度器预留最大数量 (n 台) */
#define APP_LD2_MOTOR_MAX_COUNT             3u

/** @brief Modbus 通信超时 (ms) */
#define APP_LD2_MOTOR_TIMEOUT_MS            100u

/** @brief LD2-RS 连续无应答离线确认次数 */
#define APP_LD2_MOTOR_OFFLINE_CONFIRM_COUNT 3u

/** @brief Modbus RTU 帧间隔 (ms) — 需 ≥ 3.5 字符 (115200bps ≈ 0.33ms) */
#define APP_MODBUS_RTU_INTERFRAME_MS        1u

/* 编码器反馈参数 ------------------------------------------------------------*/

/** @brief M17 编码器分辨率 */
#define APP_LD2_ENCODER_BITS                17U

#if APP_LD2_ENCODER_BITS == 17U
#define LD2_ENCODER_COUNTS_PER_REV          (131072.0f)
#elif APP_LD2_ENCODER_BITS == 23U
#define LD2_ENCODER_COUNTS_PER_REV          (8388608.0f)
#else
#error "APP_LD2_ENCODER_BITS must be 17U or 23U"
#endif

/** @brief 编码器反馈方向极性 */
#define APP_LD2_ENCODER_POLARITY_M1         (1)
#define APP_LD2_ENCODER_POLARITY_M2         (1)

/* 速度控制参数 --------------------------------------------------------------*/

/**
  * @brief 电机1 参考目标转速 (rpm)
  * @attention + 逆时针，- 顺时针
  */

#define APP_MOTOR_CTRL_REF_SPEED_RPM_M1      -0

/**
  * @brief 电机2 参考目标转速 (rpm)
  */
#define APP_MOTOR_CTRL_REF_SPEED_RPM_M2      -0

/** @brief 加速时间 (ms/1000rpm), Pr3.12 */
#define APP_MOTOR_CTRL_ACCEL_MS_PER_1000RPM  250u

/** @brief 减速时间 (ms/1000rpm), Pr3.13 */
#define APP_MOTOR_CTRL_DECEL_MS_PER_1000RPM  250u

/* 状态机时序参数 ------------------------------------------------------------*/
/** @brief 通信超时判定阈值 (ms) — 超过此时间无成功通信则进入 ERROR */
#define APP_MOTOR_CTRL_COMM_TIMEOUT_MS             500u

/** @brief 最大重试次数 — 超过后进入 STOP (不可恢复) */
#define APP_MOTOR_CTRL_MAX_RETRY_COUNT             3u

/** @brief ERROR 状态重试间隔 (ms) */
#define APP_MOTOR_CTRL_ERROR_RETRY_DELAY_MS        500u

/** @brief 上电启动延时 (ms) — 等待驱动器上电稳态 */
#define APP_SYSTEM_START_DELAY_MS              200u

/* 安全限幅 ------------------------------------------------------------------*/

/**
  * @brief 初期测试转速上限 (rpm)
  *
  * @note  初次上电测试时的安全限幅，防止因配置错误导致飞车。
  *        正式部署后可适当放宽或移除此限制。
  */
#define APP_CHASSIS_SPEED_LIMIT_RPM             4500

/* VOFA+ JustFloat 模式 -------------------------------------------------------*/

/**
  * @brief VOFA+ JustFloat 波形模式开关
  *        1 = 仅发 JustFloat 二进制帧（VOFA+ 看波形用）
  *        0 = 关闭 JustFloat 帧（串口助手调试用）
  */
#define APP_VOFA_JUSTFLOAT_ENABLE       1u
#define APP_VOFA_JUSTFLOAT_PERIOD_MS			20u

/* 二轮差速底盘参数 -----------------------------------------------------------*/

/** @brief 轮距 — 两轮中心间距 (mm) */
#define APP_CHASSIS_WHEEL_TRACK_MM              657.0f

/** @brief 车轮半径 (mm) */
#define APP_CHASSIS_WHEEL_RADIUS_MM             75.0f

/** @brief 齿轮减速比 — 电机:车轮 = 20:1 */
#define APP_CHASSIS_GEAR_RATIO                  20.0f

/** @brief  差速轮解算极性控制 */
#define APP_CHASSIS_CMD_POLARITY    -1

/** @brief 电机方向极性  */
#define APP_MOTOR_CTRL_M1_POLARITY                 1
#define APP_MOTOR_CTRL_M2_POLARITY                 -1

/** @brief 底盘物理最大线速度 (m/s): v_max = RPM × 2π × R / (Gear × 60) */
#define APP_CHASSIS_MAX_LINEAR_VEL_MPS \
    ((float)(APP_CHASSIS_SPEED_LIMIT_RPM) / (float)(APP_CHASSIS_GEAR_RATIO) \
     * 2.0f * 3.14159265f * (APP_CHASSIS_WHEEL_RADIUS_MM) / 60000.0f)

/** @brief 底盘物理最大角速度 (rad/s): ω_max = 2 × v_max / (Track / 1000) */
#define APP_CHASSIS_MAX_ANGULAR_VEL_RPS \
    (2.0f * APP_CHASSIS_MAX_LINEAR_VEL_MPS / ((APP_CHASSIS_WHEEL_TRACK_MM) / 1000.0f))

#endif /* APP_CONFIG_H */

