/**
  ******************************************************************************
  * @file    app_chassis_motor_ctrl.h
  * @brief   电机控制接口 (应用层)
  *
  * @details 提供 LD2-RS 初始化参数、二轮差速解算、遥控映射以及 VOFA 观测数据结构。
  *          非阻塞通信状态机由 app_ld2rs_task 模块负责推进。
  ******************************************************************************
  */

#ifndef APP_MOTOR_CTRL_H
#define APP_MOTOR_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "ld2_motor.h"
#include "bsp_rc.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @brief LD2-RS 电机控制配置与初始化结果。
  */
typedef struct {  
    ld2_motor_handle_t *ld2_motor;        /**< 对应的 LD2-RS 协议设备句柄。 */
    uint32_t  last_speed_update_tick_ms;   /**< 最近一次速度更新时刻，HAL tick，ms。 */
    uint32_t  last_monitor_tick_ms;       /**< 最近一次监控数据更新时间，HAL tick，ms。 */
    int16_t   ref_speed_rpm;             /**< 当前目标电机转速，rpm。 */
    uint8_t   motor_id;               /**< 电机编号，1=M1，2=M2。 */
    uint8_t   is_speed_mode_ok;         /**< 速度模式配置结果：0=失败，1=成功。 */
    uint8_t   is_zero_speed_ok;         /**< 初始零速写入结果：0=失败，1=成功。 */
    uint8_t   is_speed_source_ok;          /**< 内部速度源配置结果：0=失败，1=成功。 */
    uint8_t   is_accel_decel_ok;        /**< 加减速参数写入结果：0=失败，1=成功。 */
} ld2rs_motor_ctrl_t;

/**
  * @brief LD2-RS 第一速度环 PI 参数。
  */
typedef struct {
    uint16_t kp; /**< 第一速度环比例增益，对应 Pr1.01。 */
    uint16_t ti; /**< 第一速度环积分时间常数，对应 Pr1.02。 */
} motor_speed_pi_t;

/**
  * @brief 上电写入 LD2-RS 驱动器的电机控制参数集合。
  */
typedef struct {
    motor_speed_pi_t speed; /**< 第一速度环 PI 参数。 */
} motor_ctrl_param_t;

/** @brief M1 驱动器上电配置参数。 */
extern motor_ctrl_param_t g_ld2rs_motor_param_m1;

/** @brief M2 驱动器上电配置参数。 */
extern motor_ctrl_param_t g_ld2rs_motor_param_m2;

/**
  * @brief 读取驱动器通信参数并保存到设备句柄。
  * @param[in,out] dev LD2-RS 设备句柄；成功读取的参数写回该对象。
  * @note  dev 为 NULL 或通信失败时不修改相应字段。
  */
void app_chassis_ld2rs_motor_ctrl_param_read_back(ld2_motor_handle_t *dev);

/**
  * @brief  VOFA+ JustFloat 转速通道数据
  *
  * @details 由 MotorRuntime 状态机每周期写入, main.c VOFA 发送读取。
  *          Ref=给定转速, Fdb=驱动器反馈实际转速, Error=给定-反馈。
  */
typedef struct {
    float m1_ref_speed_rpm;          /**< M1 目标电机转速，rpm。 */
    float m1_feedback_speed_rpm;          /**< M1 驱动器滤波反馈转速，rpm。 */
    float m1_speed_error_rpm;        /**< M1 目标转速减反馈转速，rpm。 */
    float m1_read_rtt_ms;    /**< M1 最近一次读事务往返时间，ms。 */
    float m1_write_rtt_ms;   /**< M1 最近一次写事务往返时间，ms。 */
    float m1_cycle_ms;      /**< M1 最近一次完整闭环周期，ms。 */
    float m2_ref_speed_rpm;          /**< M2 目标电机转速，rpm。 */
    float m2_feedback_speed_rpm;          /**< M2 驱动器滤波反馈转速，rpm。 */
    float m2_speed_error_rpm;        /**< M2 目标转速减反馈转速，rpm。 */
    float m2_read_rtt_ms;    /**< M2 最近一次读事务往返时间，ms。 */
    float m2_write_rtt_ms;   /**< M2 最近一次写事务往返时间，ms。 */
    float m2_cycle_ms;      /**< M2 最近一次完整闭环周期，ms。 */
    float m1_offline_total_count; /**< M1 无有效应答累计次数。 */
    float m1_offline_confirmed;   /**< M1 离线确认标志：0=在线，1=离线。 */
    float m2_offline_total_count; /**< M2 无有效应答累计次数。 */
    float m2_offline_confirmed;   /**< M2 离线确认标志：0=在线，1=离线。 */
    float m1_encoder_position_counts; /**< M1 PrB.24 电机侧累计位置，count。 */
    float m2_encoder_position_counts; /**< M2 PrB.24 电机侧累计位置，count。 */
    float m1_encoder_estimated_speed_rpm; /**< M1 DWT 差分电机转速，rpm。 */
    float m2_encoder_estimated_speed_rpm; /**< M2 DWT 差分电机转速，rpm。 */
    float est_error_m1_rpm;               /**< M1 DWT 估算转速减驱动器反馈转速，rpm。 */
    float est_error_m2_rpm;               /**< M2 DWT 估算转速减驱动器反馈转速，rpm。 */
} vofa_motor_info_t;

/** @brief VOFA+ JustFloat 电机观测通道数据。 */
extern vofa_motor_info_t g_vofa_speed;

/** @brief M1 电机控制上下文。 */
extern ld2rs_motor_ctrl_t g_ld2rs_motor_ctrl_m1;

/** @brief M2 电机控制上下文。 */
extern ld2rs_motor_ctrl_t g_ld2rs_motor_ctrl_m2;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化电机控制 — 一次性配置速度模式/速度源/加减速/零速停止
  *
  * @details 顺序执行以下步骤，每步结果均通过 USART1 打印：
  *          1. 读 Pr0.01，确认/设置为速度模式 (1)
  *          2. 读 Pr3.00，确认/设置为内部速度源 (1)
  *          3. 写 Pr3.12 / Pr3.13 加减速时间
  *          4. 写 Pr3.04 = 0 确保初始停止
  *
  * @param[out] motor_ctrl   初始化后的电机控制上下文。
  * @param[in]  ld2_motor    LD2-RS 设备句柄。
  * @param[in]  ref_speed_rpm 初始目标电机转速，rpm。
  * @param[in]  motor_id     电机编号，1=M1，2=M2。
  * @note  motor_ctrl 为 NULL 时直接返回；通信失败时对应配置结果标志保持为 0。
  */
void app_chassis_ld2rs_motor_ctrl_init(ld2rs_motor_ctrl_t *motor_ctrl, ld2_motor_handle_t *ld2_motor,
                    int16_t ref_speed_rpm, uint8_t motor_id);

/**
  * @brief 写入第一速度环 PI 参数并读回校验。
  * @param[in,out] dev         LD2-RS 设备句柄。
  * @param[in]     motor_param 待写入的速度环参数。
  * @note  该函数执行阻塞式协议访问，仅在初始化阶段调用；参数为空或任一步失败时直接返回。
  */
void app_chassis_ld2rs_motor_ctrl_pi_init(ld2_motor_handle_t *dev, const motor_ctrl_param_t *motor_param);

/**
  * @brief  二轮差速模型 — 线速度+角速度 → 左右轮转速
  * @param[in]  linear_velocity_mps    底盘中心线速度，m/s。
  * @param[in]  angular_velocity_radps 底盘角速度，rad/s，逆时针为正。
  * @param[out] left_rpm_out            左侧电机目标转速，rpm。
  * @param[out] right_rpm_out           右侧电机目标转速，rpm。
  */
void app_diff_drive_compute(const float *linear_velocity_mps, const float *angular_velocity_radps,
                       int16_t *left_rpm_out, int16_t *right_rpm_out);

/**
  * @brief  等比例限幅 — 任一超限则两轮同步缩放，保持转弯半径不变
  * @param[in,out] left_rpm_out  左侧电机转速，rpm。
  * @param[in,out] right_rpm_out 右侧电机转速，rpm。
  * @param[in]     max_rpm       允许的最大转速绝对值，rpm。
  * @note  参数无效时将可写的输出置零；未超限时保持输入比例不变。
  */
void app_diff_drive_limit_rpm(float *left_rpm_out, float *right_rpm_out, float max_rpm);

/**
  * @brief  遥控掉线检测 — >200ms 无帧则置 lost_flag = 1
  * @param[in,out] channels 遥控通道数据；函数更新时间并维护 lost_flag。
  */
void app_rc_channels_check_lost(RC_Channels_t *channels);

/**
  * @brief  滤波后的遥控模拟通道映射为底盘速度指令
  * @param[in]  filter  滤波后的遥控模拟通道。
  * @param[out] command 底盘线速度与角速度指令。
  */
void app_diff_chassis_motor_ctrl_rc_map_to_chassis(const RC_Filter_t *filter, RC_ChassisCmd_t *command);

#ifdef __cplusplus
}
#endif

#endif /* APP_MOTOR_CTRL_H */



