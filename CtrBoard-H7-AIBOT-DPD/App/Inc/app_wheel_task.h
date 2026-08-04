/**
  ******************************************************************************
  * @file    app_wheel_task.h
  * @brief   四轮轮毂电机非阻塞控制任务接口
  *
  * @details 基于 UART2 RS485 (g_rs485_bus2) + Modbus RTU，使用中断收发。
  *          - 4 个 DS/RS/UM 轮毂电机轮询调度
  *          - 每电机：READ D0.000 (实际转速) → WRITE F0.3.018 (目标转速)
  *          - 目标转速来自四舵轮运动学计算结果，单位由 m/s 转为 rpm
  *
  *          电机 ID 与位置：
  *            01 = 左前轮 (FL), 02 = 左后轮 (RL),
  *            03 = 右后轮 (RR), 04 = 右前轮 (FR)
  ******************************************************************************
  */

#ifndef APP_WHEEL_TASK_H
#define APP_WHEEL_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "ds_rs_um_motor.h"
#include "bsp_can_robostride.h"

/* Exported defines ----------------------------------------------------------*/

/** @brief 轮毂电机数量 */
#define APP_WHEEL_COUNT                     4u

/** @brief 轮询调度器最大支持电机数 */
#define APP_WHEEL_MAX_COUNT                 4u

/** @brief 测试目标转速 (rpm) */
#define APP_WHEEL_TEST_SPEED_RPM            100

/** @brief 正转保持时间 (ms) */
#define APP_WHEEL_TEST_FORWARD_TIME_MS      5000u

/** @brief 反转保持时间 (ms) */
#define APP_WHEEL_TEST_BACKWARD_TIME_MS     5000u

/** @brief 失能命令发送后的等待时间 (ms)，确保所有电机收到 */
#define APP_WHEEL_TEST_DISABLE_TIME_MS      500u

/** @brief 单个 Modbus 事务超时 (ms) */
#define APP_WHEEL_TIMEOUT_MS                100u

/** @brief 读/写逻辑重试次数 */
#define APP_WHEEL_MAX_RETRY                 3u

/** @brief 离线确认阈值（连续无应答次数） */
#define APP_WHEEL_OFFLINE_CONFIRM_COUNT     3u

/** @brief 各轮机械安装方向极性。
 *  若某轮实际转向与参考坐标系相反，将该轮极性改为 -1。
 *  当前 03/04（右后轮、右前轮）与 01/02 反向，故右轮取 -1。 */
#define APP_WHEEL_FL_POLARITY               (+1)
#define APP_WHEEL_RL_POLARITY               (+1)
#define APP_WHEEL_RR_POLARITY               (-1)
#define APP_WHEEL_FR_POLARITY               (-1)

/* Exported types ------------------------------------------------------------*/

/** @brief 测试阶段 */
typedef enum {
    APP_WHEEL_TEST_PHASE_FORWARD = 0,   /**< 正转阶段 */
    APP_WHEEL_TEST_PHASE_BACKWARD,      /**< 反转阶段 */
    APP_WHEEL_TEST_PHASE_DISABLE,       /**< 失能阶段 */
    APP_WHEEL_TEST_PHASE_DONE           /**< 测试完成，停止调度 */
} app_wheel_test_phase_t;

/** @brief 单电机 FSM 状态 */
typedef enum {
    APP_WHEEL_FSM_STATE_IDLE = 0,
    APP_WHEEL_FSM_STATE_READ_REQ,
    APP_WHEEL_FSM_STATE_WAIT_READ,
    APP_WHEEL_FSM_STATE_READ_DONE,
    APP_WHEEL_FSM_STATE_WRITE_REQ,
    APP_WHEEL_FSM_STATE_WAIT_WRITE,
    APP_WHEEL_FSM_STATE_WRITE_DONE,
} app_wheel_fsm_state_t;

/** @brief 电机运行时状态 */
typedef struct {
    ds_rs_um_motor_handle_t *dev;       /**< DS/RS/UM 电机句柄              */
    app_wheel_fsm_state_t   state;      /**< FSM 状态                       */
    int8_t   polarity;                  /**< 机械安装方向极性               */

    uint8_t  tx_frame[8];               /**< Modbus 发送帧缓冲区            */
    uint8_t  rx_frame[32];              /**< Modbus 接收帧缓冲区            */

    int16_t  target_speed_rpm;          /**< 当前目标转速 (含极性)          */
    int16_t  feedback_speed_rpm;        /**< 最近一次回读的实际转速         */
    uint16_t status_word;               /**< 最近一次回读的状态字           */

    uint8_t  read_retry_count;          /**< 读阶段逻辑重试计数             */
    uint8_t  write_retry_count;         /**< 写阶段逻辑重试计数             */
    uint8_t  offline_miss_count;        /**< 连续无应答计数                 */
    uint8_t  is_offline_confirmed;      /**< 已确认离线标志                 */
    uint8_t  disable_done;              /**< 已完成失能写入标志             */

    uint32_t tx_start_tick_ms;          /**< 本次事务发起时刻               */
} app_wheel_fsm_t;

/** @brief FSM 调度器 */
typedef struct {
    app_wheel_fsm_t *table[APP_WHEEL_MAX_COUNT];
    uint8_t count;
    uint8_t current_index;
} app_wheel_scheduler_t;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化四轮轮毂电机控制任务
  * @note   调用前需先完成 bsp_rs485_bus2_init() 和 ds_rs_um_motor_init()。
  */
void app_wheel_task_init(void);

/**
  * @brief  四轮轮毂电机控制任务入口
  * @note   由 app_steer_chassis_run() 在刷新 hub_target_speed_mps 后调用，零阻塞。
  */
void app_wheel_task_run(const steer_chassis_t *steer_chassis);

/**
  * @brief  获取四个轮毂电机当前给定转速
  * @param  target_speed_rpm 输出数组，顺序为 FL、RL、RR、FR
  */
void app_wheel_task_get_target_speed_rpm(int16_t target_speed_rpm[APP_WHEEL_COUNT]);

/**
  * @brief  获取当前测试阶段（调试用）
  */
app_wheel_test_phase_t app_wheel_task_get_phase(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_WHEEL_TASK_H */
