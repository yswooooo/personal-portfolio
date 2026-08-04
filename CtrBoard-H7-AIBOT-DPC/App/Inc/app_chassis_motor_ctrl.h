/**
  ******************************************************************************
  * @file    motor_ctrl.h
  * @brief   电机控制接口 (应用层)
  *
  * @details 线性流程：
  *          app_chassis_ld2rs_motor_ctrl_init() — 一次性配置速度模式 + 速度源 + 加减速 + 写零速。
  *          MotorCtrl_Task() — 主循环中周期读状态、写速度、打印度量。
  *          当前无状态机，后续可按需扩展。
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
  * @brief  电机控制上下文 (精简版，无状态机)
  */
typedef struct {  
    ld2_motor_handle_t *ld2_motor;        /**< LD2-RS 设备句柄                    */
    uint32_t  last_speed_update_tick_ms;   /**< 最后一次速度更新时刻 (ms)          */
    uint32_t  last_monitor_tick_ms;       /**< 最后一次监控打印时刻 (ms)          */
    int16_t   ref_speed_rpm;             /**< 目标转速 (rpm), Debug 下可改写      */
    uint8_t   motor_id;               /**< 电机编号 (1/2), 用于打印前缀        */
    uint8_t   is_speed_mode_ok;         /**< 速度模式确认: 0=失败 1=成功        */
    uint8_t   is_zero_speed_ok;         /**< 零速写入: 0=失败 1=成功            */
    uint8_t   is_speed_source_ok;          /**< 内部速度源确认: 0=失败 1=成功      */
    uint8_t   is_accel_decel_ok;        /**< 加减速写入: 0=失败 1=成功          */
} ld2rs_motor_ctrl_t;

/**
  * @brief  速度环 PI 参数 (Pr1.01 Kp + Pr1.02 Ti)
  */
typedef struct {
    uint16_t kp;         /**< 第1速度环增益 Pr1.01            */
    uint16_t ti;         /**< 第1速度环积分时间常数 Pr1.02    */
} motor_speed_pi_t;

/**
  * @brief  电机控制参数
  */
typedef struct {
    motor_speed_pi_t speed;   /**< 速度环 PI */
} motor_ctrl_param_t;

extern motor_ctrl_param_t g_ld2rs_motor_param_m1;
extern motor_ctrl_param_t g_ld2rs_motor_param_m2;

void app_chassis_ld2rs_motor_ctrl_param_read_back(ld2_motor_handle_t *dev);

/**
  * @brief  VOFA+ JustFloat 转速通道数据
  *
  * @details 由 MotorRuntime 状态机每周期写入, main.c VOFA 发送读取。
  *          Ref=给定转速, Fdb=驱动器反馈实际转速, Error=给定-反馈。
  */
typedef struct {
    float m1_ref_speed_rpm;          /**< M1 给定转速 (rpm)                 */
    float m1_feedback_speed_rpm;          /**< M1 反馈实际转速 (rpm)             */
    float m1_speed_error_rpm;        /**< M1 转速误差 (rpm)                 */
    float m1_read_rtt_ms;    /**< M1 读事务往返耗时 (ms)            */
    float m1_write_rtt_ms;   /**< M1 写事务往返耗时 (ms)            */
    float m1_cycle_ms;      /**< M1 完整闭环周期 (ms)              */
    float m2_ref_speed_rpm;          /**< M2 给定转速 (rpm)                 */
    float m2_feedback_speed_rpm;          /**< M2 反馈实际转速 (rpm)             */
    float m2_speed_error_rpm;        /**< M2 转速误差 (rpm)                 */
    float m2_read_rtt_ms;    /**< M2 读事务往返耗时 (ms)            */
    float m2_write_rtt_ms;   /**< M2 写事务往返耗时 (ms)            */
    float m2_cycle_ms;      /**< M2 完整闭环周期 (ms)              */
    float m1_offline_total_count; /**< M1 offline no-response total count */
    float m1_offline_confirmed;   /**< M1 offline confirmed flag          */
    float m2_offline_total_count; /**< M2 offline no-response total count */
    float m2_offline_confirmed;   /**< M2 offline confirmed flag          */
    float m1_encoder_position_counts; /**< M1 encoder feedback position     */
    float m2_encoder_position_counts; /**< M2 encoder feedback position     */
} vofa_motor_info_t;

extern vofa_motor_info_t g_vofa_speed;

extern ld2rs_motor_ctrl_t g_ld2rs_motor_ctrl_m1;
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
  * @param  motor_ctrl         电机控制上下文指针
  * @param  ld2_motor      LD2-RS 设备句柄
  * @param  ref_speed_rpm 初始目标转速 (rpm)
  * @param  motor_id   电机编号 (1 或 2), 用于打印前缀
  */
void app_chassis_ld2rs_motor_ctrl_init(ld2rs_motor_ctrl_t *motor_ctrl, ld2_motor_handle_t *ld2_motor,
                    int16_t ref_speed_rpm, uint8_t motor_id);

void app_chassis_ld2rs_motor_ctrl_pi_init(ld2_motor_handle_t *dev, const motor_ctrl_param_t *motor_param);

/**
  * @brief  二轮差速模型 — 线速度+角速度 → 左右轮转速
  * @param  linear_velocity_mps   底盘线速度 (mm/s)
  * @param  angular_velocity_radps  底盘角速度 (rad/s, 逆时针为正)
  * @param  left_rpm_out     左轮转速输出 (rpm)
  * @param  right_rpm_out    右轮转速输出 (rpm)
  */
void app_diff_drive_compute(const float *linear_velocity_mps, const float *angular_velocity_radps,
                       int16_t *left_rpm_out, int16_t *right_rpm_out);

/**
  * @brief  等比例限幅 — 任一超限则两轮同步缩放，保持转弯半径不变
  * @param  left_rpm_out   左轮转速 (输入兼输出)
  * @param  right_rpm_out  右轮转速 (输入兼输出)
  * @param  max_rpm    最大允许转速绝对值 (rpm)
  */
void app_diff_drive_limit_rpm(float *left_rpm_out, float *right_rpm_out, float max_rpm);

/**
  * @brief  遥控掉线检测 — >200ms 无帧则置 lost_flag = 1
  * @param  channels 遥控通道数据指针
  */
void app_rc_channels_check_lost(RC_Channels_t *channels);

/**
  * @brief  滤波后的遥控模拟通道映射为底盘速度指令
  * @param  filter  滤波后的遥控模拟通道
  * @param  command 底盘速度指令输出
  */
void app_diff_chassis_motor_ctrl_rc_map_to_chassis(const RC_Filter_t *filter, RC_ChassisCmd_t *command);

#ifdef __cplusplus
}
#endif

#endif /* APP_MOTOR_CTRL_H */



