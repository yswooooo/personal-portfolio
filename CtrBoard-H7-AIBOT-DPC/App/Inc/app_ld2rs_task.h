/**
  ******************************************************************************
  * @file    app_ld2rs_task.h
  * @brief   非阻塞 LD2-RS 电机控制任务
  *
  * @details 一条 app_ld2rs_task_run() 完成 RC→差速→M1/M2 Modbus 状态机。
  *          主循环中直接调用, 零 CPU 死等。
  ******************************************************************************
  */

#ifndef APP_LD2RS_TASK_H
#define APP_LD2RS_TASK_H

#include <stdbool.h>
#include <stdint.h>
#include "app_config.h"
#include "app_encoder_dev.h"
#include "app_chassis_motor_ctrl.h"
#include "ld2_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/

/**
  * @brief 单个电机 Modbus 读写事务的执行阶段。
  */
typedef enum {
    APP_LD2RS_TASK_PHASE_IDLE = 0,    /**< 空闲阶段，准备开始新的闭环事务。 */
    APP_LD2RS_TASK_PHASE_READ_REQ,   /**< 等待总线空闲并发起读寄存器请求。 */
    APP_LD2RS_TASK_PHASE_WAIT_READ,  /**< 等待读事务完成、超时或异常。 */
    APP_LD2RS_TASK_PHASE_READ_DONE,  /**< 读事务结束，准备写入目标速度。 */
    APP_LD2RS_TASK_PHASE_WRITE_REQ,  /**< 等待总线空闲并发起速度写请求。 */
    APP_LD2RS_TASK_PHASE_WAIT_WRITE, /**< 等待写事务完成、超时或异常。 */
    APP_LD2RS_TASK_PHASE_WRITE_DONE, /**< 写事务结束，完成本次闭环周期。 */
} motor_ctrl_phase_t;

/**
  * @brief 电机状态机交替执行的读寄存器事务类型。
  */
typedef enum {
    APP_LD2RS_READ_TRANSACTION_STATUS_SPEED = 0, /**< 读取状态字和驱动器滤波转速。 */
    APP_LD2RS_READ_TRANSACTION_ENCODER_POSITION, /**< 读取 PrB.24 电机侧累计位置。 */
} motor_read_transaction_t;

/**
  * @brief 单台 LD2-RS 电机的非阻塞通信、控制和编码器估算运行时。
  * @note  由 app_ld2rs_task_run() 在主循环中推进；每台电机使用独立实例。
  */
typedef struct {
    uint8_t tx_frame[8];             /**< Modbus RTU 请求帧缓冲区，byte。 */
    uint8_t rx_frame[32];            /**< Modbus RTU 应答帧缓冲区，byte。 */

    uint16_t status_word;            /**< 驱动器状态字原始值。 */
    uint16_t actual_speed_rpm;       /**< 驱动器返回的实际转速原始 16 bit 数据，rpm。 */
    int32_t encoder_position_counts; /**< PrB.24 电机侧累计位置，count。 */

    app_encoder_sample_t encoder_sample;          /**< 最近一次有效编码器采样。 */
    app_encoder_estimator_t encoder_estimator;    /**< DWT 差分速度与角度估算器。 */

    motor_ctrl_phase_t phase;                        /**< 当前 Modbus 事务阶段。 */
    motor_read_transaction_t read_transaction;       /**< 当前读事务类型。 */
    uint8_t read_retry_count;                        /**< 当前读事务的逻辑重试次数。 */
    uint8_t write_retry_count;                       /**< 当前写事务的逻辑重试次数。 */

    ld2rs_motor_ctrl_t *motor_ctrl;  /**< 应用层电机控制上下文。 */
    ld2_motor_handle_t *motor_dev;   /**< LD2-RS 协议设备句柄。 */

    int16_t target_speed_rpm;        /**< 本周期准备写入驱动器的目标转速，rpm。 */

    uint32_t tx_start_tick_ms;       /**< 当前事务开始时刻，HAL tick，ms。 */
    uint32_t read_rtt_ms;            /**< 最近一次成功读事务往返时间，ms。 */
    uint32_t write_rtt_ms;           /**< 最近一次成功写事务往返时间，ms。 */
    uint32_t cycle_start_tick_ms;    /**< 当前闭环周期开始时刻，HAL tick，ms。 */
    uint32_t cycle_elapsed_ms;       /**< 最近一次完整闭环周期耗时，ms。 */

    uint8_t offline_miss_count;      /**< 连续无有效应答次数。 */
    uint8_t is_offline_confirmed;    /**< 离线确认标志：0=在线，1=已确认离线。 */
    uint32_t offline_total_count;    /**< 上电后的无有效应答累计次数。 */

    uint8_t force_zero_speed;        /**< 安全零速标志：1 时强制目标转速为 0 rpm。 */
} motor_fsm_t;

/**
  * @brief 多电机共享 RS485 总线的轮询调度器。
  */
typedef struct {
    motor_fsm_t *motor_fsm_table[APP_LD2_MOTOR_MAX_COUNT]; /**< 参与轮询的电机 FSM 指针表。 */
    uint8_t motor_count;                                  /**< 实际参与轮询的电机数量。 */
    uint8_t current_index;                                /**< 当前获得总线使用权的电机索引。 */
} motor_fsm_scheduler_t;

/**
  * @brief 对外发布的单台电机运动反馈。
  */
typedef struct {
    float motor_speed_rpm; /**< 电机转子估算转速，rpm。 */
    float wheel_speed_rpm; /**< 经过减速比后的车轮估算转速，rpm。 */
    float wheel_speed_mps; /**< 车轮线速度，m/s。 */
} app_ld2rs_speed_feedback_t;

/* Exported instances --------------------------------------------------------*/

/** @brief M1 电机非阻塞通信与运动反馈运行时。 */
extern motor_fsm_t g_motor_fsm_m1;

/** @brief M2 电机非阻塞通信与运动反馈运行时。 */
extern motor_fsm_t g_motor_fsm_m2;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief 初始化两台电机的 FSM 运行时和 RS485 轮询调度器。
  * @note  系统启动阶段调用一次，调用后才能执行 app_ld2rs_task_run()。
  */
void app_ld2rs_task_init(void);

/**
  * @brief 单步执行遥控映射、电机通信状态机和编码器速度估算。
  * @note  在 main() 的 while(1) 中持续调用；函数内部不进行阻塞等待。
  */
void app_ld2rs_task_run(void);

/**
  * @brief 获取指定电机最近一次有效的编码器差分运动反馈。
  * @param[in]  motor_number 电机编号，1 表示 M1，2 表示 M2。
  * @param[out] feedback     运动反馈输出，包含电机 rpm、车轮 rpm 和车轮 m/s。
  * @return true 表示对应估算器已有有效速度；false 表示参数错误或结果尚未有效。
  */
bool app_ld2rs_task_get_speed_feedback(
    uint8_t motor_number,
    app_ld2rs_speed_feedback_t *feedback);

/**
  * @brief 读取指定电机 FSM 中驱动器返回的实际转速。
  * @param[in] motor 电机 FSM 运行时指针。
  * @return 驱动器实际转速，rpm；motor 为 NULL 时返回 0。
  */
int16_t app_ld2rs_status_getter(const motor_fsm_t *motor);

#ifdef __cplusplus
}
#endif

#endif /* APP_LD2RS_TASK_H */


