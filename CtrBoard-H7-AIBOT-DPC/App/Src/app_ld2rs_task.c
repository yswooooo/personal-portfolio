/**
  ******************************************************************************
  * @file    ld2rs_task.c
  * @brief   非阻塞 LD2-RS 电机控制 — Modbus 状态机 + 差速解算 + VOFA 数据填充
  *
  * @details 每电机状态机 (7 阶段):
  *   IDLE → READ_REQ → WAIT_READ → READ_DONE
  *        → WRITE_REQ → WAIT_WRITE → WRITE_DONE → IDLE
  *   READ_REQ / WAIT_READ 内由读子事务区分状态速度与编码器位置。
  *
  *   调度器按电机表顺序轮询, 每次只推进当前 index 对应的 FSM。只有当前
  *   FSM 完成一次 Modbus 事务并释放 RS485 总线后, 才切换到下一台电机。
  *
  *   一次 Modbus 0x03 读 2 个连续寄存器 (PrB.05 运行状态 + PrB.06 未滤波速度):
  *     - 寄存器 1 → 报警位检测 (ALARM/RDY/RUN)
  *     - 寄存器 2 → VOFA 反馈转速
  *   另读 PrB.24 两个寄存器获取编码器位置; 该观测读失败时不阻塞速度写入。
  *
  *   重试分层:
  *     - BSP 层: 每事务最多 BSP_RS485_MAX_RETRY (3) 次物理重发
  *     - 本层: 读/写各自最多 APP_LD2RS_TASK_MAX_RETRY (3) 次逻辑重试
  *
  *   两电机共享一条 RS485 总线, 通过 BSP 状态机互斥串行。
  ******************************************************************************
  */

#include "app_ld2rs_task.h"
#include "app_encoder_speed.h"
#include "bsp_rs485.h"
#include "bsp_dwt.h"
#include "app_chassis_motor_ctrl.h"
#include "modbus_rtu.h"
#include "ld2_motor.h"
#include "bsp_rc.h"
#include "app_config.h"
#include "stm32h7xx_hal.h"

/* ------------------------------------------------------------------ */
/* 本地常量                                                             */
/* ------------------------------------------------------------------ */
#define APP_LD2RS_TASK_MODBUS_READ_QTY_TWO    0x0002u  /**< 一次读 2 个寄存器 (连续)    */
#define APP_LD2RS_TASK_MAX_RETRY           3u       /**< MotorRuntime 层最大逻辑重试  */

/* ------------------------------------------------------------------ */
/* 电机控制阶段                                                        */
/* ------------------------------------------------------------------ */
typedef enum {
    APP_LD2RS_TASK_PHASE_IDLE = 0,   /**< 空闲, 等周期计时器          */
    APP_LD2RS_TASK_PHASE_READ_REQ,   /**< 发送当前读子事务请求        */
    APP_LD2RS_TASK_PHASE_WAIT_READ,  /**< 等待当前读子事务应答        */
    APP_LD2RS_TASK_PHASE_READ_DONE,  /**< 本轮反馈读取完成            */
    APP_LD2RS_TASK_PHASE_WRITE_REQ,  /**< 发送目标转速写入请求        */
    APP_LD2RS_TASK_PHASE_WAIT_WRITE, /**< 等待目标转速写入应答        */
    APP_LD2RS_TASK_PHASE_WRITE_DONE, /**< 本轮读写完成                */
} motor_ctrl_phase_t;

typedef enum {
    APP_LD2RS_READ_TRANSACTION_STATUS_SPEED = 0,
    APP_LD2RS_READ_TRANSACTION_ENCODER_POSITION,
} motor_read_transaction_t;

typedef enum {
    APP_MOTOR_FSM_STEP_NONE = 0,
    APP_MOTOR_FSM_STEP_STARTED,
    APP_MOTOR_FSM_STEP_WAITING,
    APP_MOTOR_FSM_STEP_DONE_RELEASED,
    APP_MOTOR_FSM_STEP_ERROR_RELEASED
} app_motor_fsm_step_result_t;

/* ------------------------------------------------------------------ */
/* 电机运行时状态 (在 motor_ctrl_t 基础上追加)                           */
/* ------------------------------------------------------------------ */
typedef struct {
    /* Modbus 帧缓冲区 */
    uint8_t             tx_frame[8];
    uint8_t             rx_frame[32];

    /* 读回的 2 个寄存器值 */
    uint16_t            status_word;          /**< PrB.05 运行状态       */
    uint16_t            actual_speed_rpm;           /**< PrB.06 未滤波转速     */
    int32_t             encoder_position_counts;   /**< PrB.24 编码器反馈位置 */

    /* 编码器差分估计 */
    app_encoder_sample_t encoder_sample;             /**< 最近一次有效编码器样本 */
    app_encoder_speed_estimator_t speed_estimator;   /**< 编码器速度估算状态     */

    /* 状态机 */
    motor_ctrl_phase_t       phase;             /**< 当前阶段              */
    motor_read_transaction_t read_transaction;  /**< 当前读子事务          */
    uint8_t                  read_retry_count;  /**< 当前读子事务重试计数  */
    uint8_t                  write_retry_count; /**< 写阶段逻辑重试计数    */

    /* 指向电机控制上下文的句柄 */
    ld2rs_motor_ctrl_t        *motor_ctrl;
    ld2_motor_handle_t *motor_dev;

    /* 目标转速 (由差速解算结果赋值) */
    int16_t             target_speed_rpm;

    /* 诊断: 事务耗时打桩 */
    uint32_t            tx_start_tick_ms;     /**< 本次 TX 发起时刻 (打桩) */
    uint32_t            read_rtt_ms;       /**< 读事务往返耗时 (ms)     */
    uint32_t            write_rtt_ms;      /**< 写事务往返耗时 (ms)     */
    uint32_t            cycle_start_tick_ms;  /**< 本闭环起始时刻          */
    uint32_t            cycle_elapsed_ms;  /**< 完整闭环耗时 (ms)       */

    uint8_t             offline_miss_count;    /**< 连续事务无应答次数      */
    uint8_t             is_offline_confirmed;  /**< 连续无应答确认离线      */
    uint32_t            offline_total_count;   /**< 历史事务无应答总次数    */

    uint8_t             force_zero_speed;      /**< READ重试耗尽→强制零速, 跨advance存活 */
} motor_fsm_t;

typedef struct {
    motor_fsm_t *motor_fsm_table[APP_LD2_MOTOR_MAX_COUNT];
    uint8_t      motor_count;
    uint8_t      current_index;
} motor_fsm_scheduler_t;

static motor_fsm_t s_motor_fsm_m1;
static motor_fsm_t s_motor_fsm_m2;
static motor_fsm_scheduler_t s_motor_fsm_scheduler;

/* ------------------------------------------------------------------ */
/* 前向声明                                                            */
/* ------------------------------------------------------------------ */
static void app_motor_fsm_init(motor_fsm_t *fsm,
                               ld2rs_motor_ctrl_t *motor_ctrl, ld2_motor_handle_t *motor_dev);
static void app_motor_fsm_sync_vofa_offline(const motor_fsm_t *fsm);
static void app_motor_fsm_mark_response_ok(motor_fsm_t *fsm);
static void app_motor_fsm_mark_no_response(motor_fsm_t *fsm);
static app_motor_fsm_step_result_t app_motor_fsm_step(motor_fsm_t *fsm,
                                                      bsp_rs485_state_t bus_state);

static void app_encoder_sample_publish(app_encoder_sample_t *target,
                                       const app_encoder_sample_t *source)
{
    if ((target == NULL) || (source == NULL)) {
        return;
    }

    target->counts = source->counts;
    target->timestamp_us = source->timestamp_us;
    target->sequence = source->sequence;
}

static void app_ld2rs_speed_feedback_copy(
    app_ld2rs_speed_feedback_t *feedback,
    const app_encoder_speed_estimator_t *estimator)
{
    feedback->motor_speed_rpm = estimator->motor_speed_rpm;
    feedback->wheel_speed_rpm = estimator->wheel_speed_rpm;
    feedback->wheel_speed_mps = estimator->wheel_speed_mps;
}

static void app_encoder_runtime_reset(
    app_encoder_sample_t *sample,
    app_encoder_speed_estimator_t *estimator)
{
    *sample = (app_encoder_sample_t){0};
    *estimator = (app_encoder_speed_estimator_t){0};
}

static void app_motor_fsm_scheduler_init(motor_fsm_scheduler_t *scheduler,
                                         uint8_t motor_count);
static void app_motor_fsm_scheduler_step(motor_fsm_scheduler_t *scheduler,
                                         bsp_rs485_state_t bus_state);
static void app_motor_fsm_scheduler_advance(motor_fsm_scheduler_t *scheduler);

/* ------------------------------------------------------------------ */
/* Modbus 帧组装助手 (原地, 不依赖 modbus_rtu.c 静态函数)                */
/* ------------------------------------------------------------------ */

/** @brief 将 uint16_t 以大端写入 buf[idx..idx+1] */
static void app_ld2rs_task_put_u16(uint8_t *buffer, uint16_t value_u16)
{
    buffer[0] = (uint8_t)(value_u16 >> 8);
    buffer[1] = (uint8_t)(value_u16 & 0xFFu);
}

/** @brief 从 buf[idx..idx+1] 以大端读取 uint16_t */
static uint16_t app_ld2rs_task_get_u16(const uint8_t *buffer)
{
    return (uint16_t)(((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1]);
}

/**
  * @brief  组装 0x03 读寄存器请求帧 (8 字节)
  * @param  frame_buffer     输出帧缓冲区 (8 字节)
  * @param  slave_id  从机站号
  * @param  reg_addr 起始寄存器地址
  * @param  quantity     读取数量 (1~125)
  */
static void app_ld2rs_task_build_read_req(uint8_t *frame_buffer, uint8_t slave_id,
                              uint16_t reg_addr, uint16_t quantity)
{
    uint16_t crc_value;

    frame_buffer[0] = slave_id;
    frame_buffer[1] = MODBUS_RTU_FC_READ_HOLDING;
    app_ld2rs_task_put_u16(&frame_buffer[2], reg_addr);
    app_ld2rs_task_put_u16(&frame_buffer[4], quantity);
    crc_value = modbus_rtu_crc16(frame_buffer, 6u);
    frame_buffer[6] = (uint8_t)(crc_value & 0xFFu);
    frame_buffer[7] = (uint8_t)(crc_value >> 8);
}

/**
  * @brief  组装 0x06 写单个寄存器请求帧 (8 字节)
  */
static void app_ld2rs_task_build_write_req(uint8_t *frame_buffer, uint8_t slave_id,
                               uint16_t reg_addr, uint16_t value_u16)
{
    uint16_t crc_value;

    frame_buffer[0] = slave_id;
    frame_buffer[1] = MODBUS_RTU_FC_WRITE_SINGLE;
    app_ld2rs_task_put_u16(&frame_buffer[2], reg_addr);
    app_ld2rs_task_put_u16(&frame_buffer[4], value_u16);
    crc_value = modbus_rtu_crc16(frame_buffer, 6u);
    frame_buffer[6] = (uint8_t)(crc_value & 0xFFu);
    frame_buffer[7] = (uint8_t)(crc_value >> 8);
}

/**
  * @brief  校验 Modbus 应答帧 (函数码感知长度检查)
  *
  * @details 0x03 响应 (读): ID(1)+FC(1)+字节数(1)+数据(≥2)+CRC(2) ≥ 7
  *          0x06 响应 (写): ID(1)+FC(1)+地址(2)+值(2)+CRC(2) = 6
  *
  * @param  rx_buffer               接收帧缓冲区
  * @param  rx_len          实际接收长度
  * @param  expected_slave_id 期望站号
  * @param  expected_func    期望功能码 (0x03 或 0x06)
  * @retval 1 有效, 0 无效
  */
static uint8_t app_ld2rs_task_validate_resp(const uint8_t *rx_buffer, uint16_t rx_len,
                                uint8_t expected_slave_id,
                                uint8_t expected_func)
{
    uint16_t min_len;
    uint16_t crc_value;

    /* 函数码感知最小长度 */
    if (expected_func == MODBUS_RTU_FC_READ_HOLDING) {
        min_len = 7u;  /* ID + FC + byteCnt + 2×data + CRC */
    } else if (expected_func == MODBUS_RTU_FC_WRITE_SINGLE) {
        min_len = 8u;  /* ID + FC + addrH + addrL + valH + valL + CRC */
    } else {
        min_len = 5u;  /* 通用最小值 */
    }

    if (rx_len < min_len) {
        return 0u;
    }

    /* CRC (全帧 CRC == 0 即正确) */
    crc_value = modbus_rtu_crc16(rx_buffer, rx_len);
    if (crc_value != 0u) {
        return 0u;
    }

    /* 站号 */
    if (rx_buffer[0] != expected_slave_id) {
        return 0u;
    }

    /* 功能码 (含异常码检测) */
    if ((rx_buffer[1] & 0x80u) != 0u) {
        return 0u; /* 异常应答 */
    }
    if (rx_buffer[1] != expected_func) {
        return 0u;
    }

    return 1u;
}

/* ------------------------------------------------------------------ */
/* MotorRuntime                                                       */
/* ------------------------------------------------------------------ */

/**
  * @brief  初始化电机运行时
  */
static void app_motor_fsm_init(motor_fsm_t *fsm,
                              ld2rs_motor_ctrl_t *motor_ctrl, ld2_motor_handle_t *motor_dev)
{
    fsm->motor_ctrl              = motor_ctrl;
    fsm->motor_dev             = motor_dev;
    fsm->phase           = APP_LD2RS_TASK_PHASE_IDLE;
    fsm->read_transaction = APP_LD2RS_READ_TRANSACTION_STATUS_SPEED;
    fsm->target_speed_rpm     = 0;
    fsm->read_retry_count   = 0u;
    fsm->write_retry_count  = 0u;
    fsm->status_word        = 0u;
    fsm->actual_speed_rpm         = 0u;
    fsm->encoder_position_counts = 0;
    app_encoder_runtime_reset(&fsm->encoder_sample,
                              &fsm->speed_estimator);
    fsm->tx_start_tick_ms   = 0u;
    fsm->read_rtt_ms     = 0u;
    fsm->write_rtt_ms    = 0u;
    fsm->cycle_start_tick_ms = 0u;
    fsm->cycle_elapsed_ms = 0u;
    fsm->offline_miss_count   = 0u;
    fsm->is_offline_confirmed = 0u;
    fsm->offline_total_count  = 0u;
    fsm->force_zero_speed     = 0u;
    app_motor_fsm_sync_vofa_offline(fsm);
}

static void app_motor_fsm_sync_vofa_offline(const motor_fsm_t *fsm)
{
    if ((fsm == NULL) || (fsm->motor_ctrl == NULL)) {
        return;
    }

    if (fsm->motor_ctrl->motor_id == 1u) {
        g_vofa_speed.m1_offline_total_count = (float)fsm->offline_total_count;
        g_vofa_speed.m1_offline_confirmed = (float)fsm->is_offline_confirmed;
    } else if (fsm->motor_ctrl->motor_id == 2u) {
        g_vofa_speed.m2_offline_total_count = (float)fsm->offline_total_count;
        g_vofa_speed.m2_offline_confirmed = (float)fsm->is_offline_confirmed;
    }
}

static void app_motor_fsm_mark_response_ok(motor_fsm_t *fsm)
{
    if ((fsm == NULL) || (fsm->motor_ctrl == NULL)) {
        return;
    }

    fsm->offline_miss_count = 0u;
    fsm->is_offline_confirmed = 0u;
    app_motor_fsm_sync_vofa_offline(fsm);
}

static void app_motor_fsm_mark_no_response(motor_fsm_t *fsm)
{
    if ((fsm == NULL) || (fsm->motor_ctrl == NULL)) {
        return;
    }

    fsm->offline_total_count++;
    if (fsm->offline_miss_count < APP_LD2_MOTOR_OFFLINE_CONFIRM_COUNT) {
        fsm->offline_miss_count++;
    }
    if (fsm->offline_miss_count >= APP_LD2_MOTOR_OFFLINE_CONFIRM_COUNT) {
        fsm->is_offline_confirmed = 1u;
    }
    app_motor_fsm_sync_vofa_offline(fsm);
}

static uint8_t app_motor_fsm_safety_stop_active(void)
{
    extern volatile uint8_t g_emergency_stop_flag;

    return (uint8_t)(g_emergency_stop_flag
        || g_rc.lost_flag
        || (g_rc.sw_st[eRC_SW_A].curr == eRC_POS_DOWN));
}

static app_motor_fsm_step_result_t app_motor_fsm_handle_read_failure(
    motor_fsm_t *fsm,
    app_motor_fsm_step_result_t released_result)
{
    fsm->read_retry_count++;
    if (fsm->read_retry_count < APP_LD2RS_TASK_MAX_RETRY) {
        fsm->phase = APP_LD2RS_TASK_PHASE_READ_REQ;
        return released_result;
    }

    if (fsm->read_transaction == APP_LD2RS_READ_TRANSACTION_STATUS_SPEED) {
        fsm->force_zero_speed = 1u;
        fsm->phase = APP_LD2RS_TASK_PHASE_WRITE_REQ;
        return released_result;
    }

    fsm->phase = APP_LD2RS_TASK_PHASE_READ_DONE;
    return APP_MOTOR_FSM_STEP_NONE;
}

/**
  * @brief  单步推进电机状态机 (主循环每轮调用)
  *
  * @note   仅当 BSP 总线空闲时才能发起新 TX。
  *         BSP 层自动处理物理层超时重试 (最多 3 次)。
  *         状态速度与编码器读子事务分别最多执行 3 次 APP 逻辑尝试。
  *         编码器重试耗尽仅放弃本轮观测, 保留旧值并继续速度写入。
  *
  * @param  fsm       电机运行时句柄
  * @param  bus_state 总线当前状态 (由上层 app_ld2rs_task_run 统一 Poll 一次获得)
  */
static app_motor_fsm_step_result_t app_motor_fsm_step(motor_fsm_t *fsm,
                                                      bsp_rs485_state_t bus_state)
{
    uint32_t now_tick;

    if (fsm == NULL || fsm->motor_ctrl == NULL) {
        return APP_MOTOR_FSM_STEP_NONE;
    }

    now_tick = HAL_GetTick();

    switch (fsm->phase) {

    /* ------------------------------------------------------------ */
    case APP_LD2RS_TASK_PHASE_IDLE:

        {
            fsm->cycle_start_tick_ms = now_tick;  /**< 打桩: 闭环起始时刻 */
            fsm->read_retry_count    = 0u;      /**< 新周期复位             */
            fsm->write_retry_count   = 0u;
            fsm->force_zero_speed    = 0u;      /**< 新周期清除强制零速      */
            fsm->read_transaction = APP_LD2RS_READ_TRANSACTION_STATUS_SPEED;
            fsm->phase = APP_LD2RS_TASK_PHASE_READ_REQ;
        }
        break;

    /* ------------------------------------------------------------ */
    case APP_LD2RS_TASK_PHASE_READ_REQ:
        {
            bsp_rs485_status_t start_status;
            uint16_t read_register;

            if (bus_state != BSP_RS485_STATE_IDLE) {
                return APP_MOTOR_FSM_STEP_WAITING;
            }

            if (fsm->read_transaction
                == APP_LD2RS_READ_TRANSACTION_STATUS_SPEED) {
                read_register = LD2_MOTOR_REG_RUN_STATUS;//RS485地址
            } else if (fsm->read_transaction
                       == APP_LD2RS_READ_TRANSACTION_ENCODER_POSITION) {
                if (app_motor_fsm_safety_stop_active()) {
                    fsm->phase = APP_LD2RS_TASK_PHASE_READ_DONE;
                    break;
                }
                read_register = LD2_MOTOR_REG_ENCODER_POSITION_H;//RS485地址
            } else {
                fsm->force_zero_speed = 1u;
                fsm->phase = APP_LD2RS_TASK_PHASE_WRITE_REQ;
                return APP_MOTOR_FSM_STEP_NONE;
            }
            //组请求返回帧
            app_ld2rs_task_build_read_req(
                fsm->tx_frame,
                fsm->motor_dev->slave_id,
                read_register,
                APP_LD2RS_TASK_MODBUS_READ_QTY_TWO);

            start_status = bsp_rs485_start_tx(
                &g_rs485_bus,
                fsm->tx_frame, sizeof(fsm->tx_frame),
                fsm->rx_frame, sizeof(fsm->rx_frame),
                fsm->motor_dev->timeout_ms);

            if (start_status == BSP_RS485_STATUS_OK) {
                fsm->tx_start_tick_ms = HAL_GetTick();
                fsm->phase = APP_LD2RS_TASK_PHASE_WAIT_READ;
                return APP_MOTOR_FSM_STEP_STARTED;
            }
            if (start_status == BSP_RS485_STATUS_ERR_BUSY) {
                return APP_MOTOR_FSM_STEP_WAITING;
            }

            return app_motor_fsm_handle_read_failure(
                fsm, APP_MOTOR_FSM_STEP_ERROR_RELEASED);
        }

    /* ------------------------------------------------------------ */
    case APP_LD2RS_TASK_PHASE_WAIT_READ:
        if ((fsm->read_transaction
             == APP_LD2RS_READ_TRANSACTION_ENCODER_POSITION)
            && app_motor_fsm_safety_stop_active()) {
            bsp_rs485_cancel_transaction(&g_rs485_bus);
            fsm->phase = APP_LD2RS_TASK_PHASE_READ_DONE;
            return APP_MOTOR_FSM_STEP_NONE;
        }

        if (bus_state == BSP_RS485_STATE_DONE) {
            uint8_t response_valid;

            response_valid = app_ld2rs_task_validate_resp(
                fsm->rx_frame,
                g_rs485_bus.rx_len,
                fsm->motor_dev->slave_id,
                MODBUS_RTU_FC_READ_HOLDING);
            response_valid = (uint8_t)(response_valid
                && (g_rs485_bus.rx_len == 9u)
                && (fsm->rx_frame[2] == 4u));

            if (response_valid) {
                if (fsm->read_transaction
                    == APP_LD2RS_READ_TRANSACTION_STATUS_SPEED) {
                    /* [3-4]=PrB.05 状态, [5-6]=PrB.06 未滤波速度 */
                    fsm->status_word =
                        app_ld2rs_task_get_u16(&fsm->rx_frame[3]);
                    fsm->actual_speed_rpm =
                        app_ld2rs_task_get_u16(&fsm->rx_frame[5]);
                    fsm->read_rtt_ms =
                        HAL_GetTick() - fsm->tx_start_tick_ms;

                    if (fsm->motor_ctrl->motor_id == 1u) {
                        g_vofa_speed.m1_feedback_speed_rpm =
                            (float)((int16_t)fsm->actual_speed_rpm);
                        g_vofa_speed.m1_read_rtt_ms =
                            (float)fsm->read_rtt_ms;
                    } else {
                        g_vofa_speed.m2_feedback_speed_rpm =
                            (float)((int16_t)fsm->actual_speed_rpm);
                        g_vofa_speed.m2_read_rtt_ms =
                            (float)fsm->read_rtt_ms;
                    }

                    app_motor_fsm_mark_response_ok(fsm);
                    fsm->read_retry_count = 0u;
                    fsm->read_transaction =
                        APP_LD2RS_READ_TRANSACTION_ENCODER_POSITION;
                    fsm->phase = APP_LD2RS_TASK_PHASE_READ_REQ;
                } else if (fsm->read_transaction
                           == APP_LD2RS_READ_TRANSACTION_ENCODER_POSITION) {
                    uint16_t encoder_position_high;
                    uint16_t encoder_position_low;
                    uint32_t encoder_position_raw;
                    app_encoder_sample_t new_sample;

                    encoder_position_high =
                        app_ld2rs_task_get_u16(&fsm->rx_frame[3]);
                    encoder_position_low =
                        app_ld2rs_task_get_u16(&fsm->rx_frame[5]);
                    encoder_position_raw =
                        ((uint32_t)encoder_position_high << 16)
                        | (uint32_t)encoder_position_low;
                    fsm->encoder_position_counts =
                        (int32_t)encoder_position_raw;

                    new_sample.counts = fsm->encoder_position_counts;
                    new_sample.timestamp_us = g_rs485_bus.rx_timestamp_us;
                    new_sample.sequence = fsm->encoder_sample.sequence + 1u;
                    app_encoder_sample_publish(&fsm->encoder_sample,
                                               &new_sample);

                    if (fsm->motor_ctrl->motor_id == 1u) {
                        g_vofa_speed.m1_encoder_position_counts =
                            (float)fsm->encoder_position_counts;
                    } else {
                        g_vofa_speed.m2_encoder_position_counts =
                            (float)fsm->encoder_position_counts;
                    }

                    fsm->read_retry_count = 0u;
                    fsm->phase = APP_LD2RS_TASK_PHASE_READ_DONE;
                } else {
                    fsm->force_zero_speed = 1u;
                    fsm->phase = APP_LD2RS_TASK_PHASE_WRITE_REQ;
                    bsp_rs485_ack_done(&g_rs485_bus);
                    return APP_MOTOR_FSM_STEP_NONE;
                }

                bsp_rs485_ack_done(&g_rs485_bus);
                return APP_MOTOR_FSM_STEP_DONE_RELEASED;
            }

            {
                app_motor_fsm_step_result_t result;

                result = app_motor_fsm_handle_read_failure(
                    fsm, APP_MOTOR_FSM_STEP_DONE_RELEASED);
                bsp_rs485_ack_done(&g_rs485_bus);
                return result;
            }
        } else if ((bus_state == BSP_RS485_STATE_TIMEOUT)
                   || (bus_state == BSP_RS485_STATE_ERROR)) {
            app_motor_fsm_step_result_t result;

            if ((bus_state == BSP_RS485_STATE_TIMEOUT)
                && (fsm->read_transaction
                    == APP_LD2RS_READ_TRANSACTION_STATUS_SPEED)) {
                app_motor_fsm_mark_no_response(fsm);
            }

            result = app_motor_fsm_handle_read_failure(
                fsm, APP_MOTOR_FSM_STEP_ERROR_RELEASED);
            bsp_rs485_ack_done(&g_rs485_bus);
            return result;
        } else {
            return APP_MOTOR_FSM_STEP_WAITING;
        }
        break;

    /* ------------------------------------------------------------ */
    case APP_LD2RS_TASK_PHASE_READ_DONE:
        /* 安全停机 / 报警检查由 app_ld2rs_task_run 统一处理。 */
        fsm->phase = APP_LD2RS_TASK_PHASE_WRITE_REQ;
        break;

    /* ------------------------------------------------------------ */
    case APP_LD2RS_TASK_PHASE_WRITE_REQ:
        if (bus_state != BSP_RS485_STATE_IDLE) {
            return APP_MOTOR_FSM_STEP_WAITING;
        }
        {
            int16_t ref_speed_rpm = fsm->target_speed_rpm;

            /* 限幅 */
            if (ref_speed_rpm > APP_CHASSIS_SPEED_LIMIT_RPM) {
                ref_speed_rpm = APP_CHASSIS_SPEED_LIMIT_RPM;
            } else if (ref_speed_rpm < (-APP_CHASSIS_SPEED_LIMIT_RPM)) {
                ref_speed_rpm = (int16_t)(-APP_CHASSIS_SPEED_LIMIT_RPM);
            }

            /* 填充 VOFA 给定转速 */
            if (fsm->motor_ctrl->motor_id == 1u) {
                g_vofa_speed.m1_ref_speed_rpm  = (float)ref_speed_rpm;
                g_vofa_speed.m1_speed_error_rpm = g_vofa_speed.m1_ref_speed_rpm - g_vofa_speed.m1_feedback_speed_rpm;
            } else {
                g_vofa_speed.m2_ref_speed_rpm  = (float)ref_speed_rpm;
                g_vofa_speed.m2_speed_error_rpm = g_vofa_speed.m2_ref_speed_rpm - g_vofa_speed.m2_feedback_speed_rpm;
            }

            app_ld2rs_task_build_write_req(fsm->tx_frame, fsm->motor_dev->slave_id,
                               LD2_MOTOR_REG_SPEED_TARGET, (uint16_t)ref_speed_rpm);
        }
        if (bsp_rs485_start_tx(&g_rs485_bus,
                              fsm->tx_frame, sizeof(fsm->tx_frame),
                              fsm->rx_frame, sizeof(fsm->rx_frame),
                              fsm->motor_dev->timeout_ms) == BSP_RS485_STATUS_OK) {
            fsm->tx_start_tick_ms = HAL_GetTick();  /**< 打桩: 写 TX 发起 */
            fsm->phase = APP_LD2RS_TASK_PHASE_WAIT_WRITE;
            return APP_MOTOR_FSM_STEP_STARTED;
        }
        break;

    /* ------------------------------------------------------------ */
    case APP_LD2RS_TASK_PHASE_WAIT_WRITE:
        if (bus_state == BSP_RS485_STATE_DONE) {
            if (app_ld2rs_task_validate_resp(fsm->rx_frame, g_rs485_bus.rx_len,
                                 fsm->motor_dev->slave_id,
                                 MODBUS_RTU_FC_WRITE_SINGLE)) {
                /* 打桩: 写 RTT + VOFA */
                fsm->write_rtt_ms = HAL_GetTick() - fsm->tx_start_tick_ms;
                if (fsm->motor_ctrl->motor_id == 1u) {
                    g_vofa_speed.m1_write_rtt_ms = (float)fsm->write_rtt_ms;
                } else {
                    g_vofa_speed.m2_write_rtt_ms = (float)fsm->write_rtt_ms;
                }
                app_motor_fsm_mark_response_ok(fsm);
                fsm->write_retry_count = 0u;
            } else {
                /* 写应答校验失败 → 逻辑重试 */
                fsm->write_retry_count++;
                if (fsm->write_retry_count >= APP_LD2RS_TASK_MAX_RETRY) {
                    fsm->phase = APP_LD2RS_TASK_PHASE_WRITE_DONE;
                } else {
                    bsp_rs485_ack_done(&g_rs485_bus);
                    fsm->phase = APP_LD2RS_TASK_PHASE_WRITE_REQ;
                    return APP_MOTOR_FSM_STEP_DONE_RELEASED;
                }
            }
            bsp_rs485_ack_done(&g_rs485_bus);
            fsm->phase = APP_LD2RS_TASK_PHASE_WRITE_DONE;
            return APP_MOTOR_FSM_STEP_DONE_RELEASED;
        } else if (bus_state == BSP_RS485_STATE_TIMEOUT) {
            app_motor_fsm_mark_no_response(fsm);
            fsm->write_retry_count++;
            if (fsm->write_retry_count >= APP_LD2RS_TASK_MAX_RETRY) {
                fsm->phase = APP_LD2RS_TASK_PHASE_WRITE_DONE;
            } else {
                bsp_rs485_ack_done(&g_rs485_bus);
                fsm->phase = APP_LD2RS_TASK_PHASE_WRITE_REQ;
                return APP_MOTOR_FSM_STEP_ERROR_RELEASED;
            }

            bsp_rs485_ack_done(&g_rs485_bus);
            return APP_MOTOR_FSM_STEP_ERROR_RELEASED;
        } else if (bus_state == BSP_RS485_STATE_ERROR) {
            fsm->write_retry_count++;
            if (fsm->write_retry_count >= APP_LD2RS_TASK_MAX_RETRY) {
                fsm->phase = APP_LD2RS_TASK_PHASE_WRITE_DONE;
            } else {
                fsm->phase = APP_LD2RS_TASK_PHASE_WRITE_REQ;
            }
            bsp_rs485_ack_done(&g_rs485_bus);
            return APP_MOTOR_FSM_STEP_ERROR_RELEASED;
        } else {
            return APP_MOTOR_FSM_STEP_WAITING;
        }
        break;

    /* ------------------------------------------------------------ */
    case APP_LD2RS_TASK_PHASE_WRITE_DONE:
        /* 打桩: 完整闭环耗时 → VOFA */
        fsm->cycle_elapsed_ms = now_tick - fsm->cycle_start_tick_ms;
        if (fsm->motor_ctrl->motor_id == 1u) {
            g_vofa_speed.m1_cycle_ms     = (float)fsm->cycle_elapsed_ms;
        } else {
            g_vofa_speed.m2_cycle_ms     = (float)fsm->cycle_elapsed_ms;
        }
        /* 更新计时器, 本轮完成 */
        fsm->motor_ctrl->last_speed_update_tick_ms = now_tick;
        fsm->phase = APP_LD2RS_TASK_PHASE_IDLE;
        break;

    default:
        fsm->phase = APP_LD2RS_TASK_PHASE_IDLE;
        break;
    }

    return APP_MOTOR_FSM_STEP_NONE;
}

static void app_motor_fsm_scheduler_init(motor_fsm_scheduler_t *scheduler,
                                         uint8_t motor_count)
{
    uint8_t index;

    if (scheduler == NULL) {
        return;
    }

    for (index = 0u; index < APP_LD2_MOTOR_MAX_COUNT; index++) {
        scheduler->motor_fsm_table[index] = NULL;
    }

    scheduler->motor_count = motor_count;
    if (scheduler->motor_count > APP_LD2_MOTOR_MAX_COUNT) {
        scheduler->motor_count = APP_LD2_MOTOR_MAX_COUNT;
    }

    scheduler->current_index = 0u;

#if (APP_LD2_MOTOR_MAX_COUNT >= 1u)
    if ((APP_LD2_MOTOR_COUNT >= 1u) && (scheduler->motor_count >= 1u)) {
        scheduler->motor_fsm_table[0] = &s_motor_fsm_m1;
    }
#endif

#if (APP_LD2_MOTOR_MAX_COUNT >= 2u)
    if ((APP_LD2_MOTOR_COUNT >= 2u) && (scheduler->motor_count >= 2u)) {
        scheduler->motor_fsm_table[1] = &s_motor_fsm_m2;
    }
#endif
}

static void app_motor_fsm_scheduler_advance(motor_fsm_scheduler_t *scheduler)
{
    if ((scheduler == NULL) || (scheduler->motor_count == 0u)) {
        return;
    }

    scheduler->current_index = (uint8_t)((scheduler->current_index + 1u)
                               % scheduler->motor_count);
}

static void app_motor_fsm_scheduler_step(motor_fsm_scheduler_t *scheduler,
                                         bsp_rs485_state_t bus_state)
{
    motor_fsm_t *current_fsm;
    app_motor_fsm_step_result_t step_result;

    if ((scheduler == NULL) || (scheduler->motor_count == 0u)) {
        return;
    }

    if (scheduler->motor_count > APP_LD2_MOTOR_MAX_COUNT) {
        scheduler->motor_count = APP_LD2_MOTOR_MAX_COUNT;
    }

    if (scheduler->current_index >= scheduler->motor_count) {
        scheduler->current_index = 0u;
    }

    current_fsm = scheduler->motor_fsm_table[scheduler->current_index];
    if (current_fsm == NULL) {
        app_motor_fsm_scheduler_advance(scheduler);
        return;
    }

    if (((bus_state == BSP_RS485_STATE_DONE)
         || (bus_state == BSP_RS485_STATE_TIMEOUT)
         || (bus_state == BSP_RS485_STATE_ERROR))
        && (current_fsm->phase != APP_LD2RS_TASK_PHASE_WAIT_READ)
        && (current_fsm->phase != APP_LD2RS_TASK_PHASE_WAIT_WRITE)) {
        bsp_rs485_ack_done(&g_rs485_bus);
        return;
    }

    step_result = app_motor_fsm_step(current_fsm, bus_state);

    if ((step_result == APP_MOTOR_FSM_STEP_DONE_RELEASED)
        || (step_result == APP_MOTOR_FSM_STEP_ERROR_RELEASED)) {
        app_motor_fsm_scheduler_advance(scheduler);
    }
}

/* ------------------------------------------------------------------ */
/* 公开接口                                                            */
/* ------------------------------------------------------------------ */

/**
  * @brief  LD2-RS 电机控制统一任务入口
  *
  * @details 每轮调用执行:
  *          1. 统一 Poll 一次 BSP 总线状态
  *          2. RC 掉线检查 + 通道映射 → 差速解算 → 左右轮 rpm
  *          3. M1/M2 各自状态机推进 (非阻塞, 共享 Poll 结果)
  *
  * @note    main.c while(1) 中直接调用, 零 CPU 死等。
  */
void app_ld2rs_task_run(void)
{
    bsp_rs485_state_t bus_state;

    /* 保持 DWT 32 位计数器的软件扩展时间线连续。 */
    (void)BSP_DWT_GetTickUs();

    /* 统一 Poll 一次, 两个电机共享结果 (避免重复调用) */
    bus_state = bsp_rs485_poll(&g_rs485_bus);

    /* 差速解算 (M1=左轮, M2=右轮) */
    app_rc_channels_check_lost(&g_rc);
    app_diff_chassis_motor_ctrl_rc_map_to_chassis(&g_rc_filter, &g_rc_chassis);

    /* VRA/VRB 油门: 旋钮控制最大速度, SWB/SWC 解锁 */
    {
        float vra_throttle = (float)g_rc.vra / 1600.0f;
        float vrb_throttle = (float)g_rc.vrb / 1600.0f;

        if (g_rc.sw_st[eRC_SW_B].curr == eRC_POS_UP) {
            g_rc_chassis.fLinearVel *=
                vra_throttle * APP_CHASSIS_MAX_LINEAR_VEL_MPS;
        } else {
            g_rc_chassis.fLinearVel = 0.0f;
        }

        if (g_rc.sw_st[eRC_SW_C].curr == eRC_POS_UP) {
            g_rc_chassis.fAngularVel *=
                vrb_throttle * APP_CHASSIS_MAX_ANGULAR_VEL_RPS;
        } else {
            g_rc_chassis.fAngularVel = 0.0f;
        }
    }

    app_diff_drive_compute(&g_rc_chassis.fLinearVel, &g_rc_chassis.fAngularVel,
                      &g_rc_chassis.i16LeftRpm,  &g_rc_chassis.i16RightRpm);

    /* 目标转速赋给各电机运行时 */
    s_motor_fsm_m1.target_speed_rpm = g_rc_chassis.i16LeftRpm;
    s_motor_fsm_m2.target_speed_rpm = g_rc_chassis.i16RightRpm;

    /* FSM 层强制零速: READ 重试耗尽标志, 优先级高于 RC 但低于安全停机 */
    if (s_motor_fsm_m1.force_zero_speed) {
        s_motor_fsm_m1.target_speed_rpm = 0;
    }
    if (s_motor_fsm_m2.force_zero_speed) {
        s_motor_fsm_m2.target_speed_rpm = 0;
    }

    /* 安全停机: 优先级最高, 最终覆盖所有 */
    {
        extern volatile uint8_t g_emergency_stop_flag;
        extern RC_Channels_t     g_rc;

        if (g_emergency_stop_flag || g_rc.lost_flag
            || (g_rc.sw_st[eRC_SW_A].curr == eRC_POS_DOWN)) {
            s_motor_fsm_m1.target_speed_rpm = 0;
            s_motor_fsm_m2.target_speed_rpm = 0;
        }
    }

    app_motor_fsm_scheduler_step(&s_motor_fsm_scheduler, bus_state);

    app_encoder_speed_update(&s_motor_fsm_m1.encoder_sample,
                             &s_motor_fsm_m1.speed_estimator,
                             APP_LD2_ENCODER_POLARITY_M1);
    app_encoder_speed_update(&s_motor_fsm_m2.encoder_sample,
                             &s_motor_fsm_m2.speed_estimator,
                             APP_LD2_ENCODER_POLARITY_M2);
}

bool app_ld2rs_task_get_speed_feedback(
    uint8_t motor_number,
    app_ld2rs_speed_feedback_t *feedback)
{
    const app_encoder_speed_estimator_t *estimator;

    if (feedback == NULL) {
        return false;
    }

    if (motor_number == 1u) {
        estimator = &s_motor_fsm_m1.speed_estimator;
    } else if (motor_number == 2u) {
        estimator = &s_motor_fsm_m2.speed_estimator;
    } else {
        return false;
    }

    if (estimator->speed_valid == 0u) {
        return false;
    }

    app_ld2rs_speed_feedback_copy(feedback, estimator);
    return true;
}

/**
  * @brief  初始化电机运行时 (main.c 启动阶段调用一次)
  */
void app_ld2rs_task_init(void)
{
    app_motor_fsm_init(&s_motor_fsm_m1, &g_ld2rs_motor_ctrl_m1, &g_ld2rs_dev_m1);
    app_motor_fsm_init(&s_motor_fsm_m2, &g_ld2rs_motor_ctrl_m2, &g_ld2rs_dev_m2);

    app_motor_fsm_scheduler_init(&s_motor_fsm_scheduler, APP_LD2_MOTOR_COUNT);

    /* 轮询调度从 index 0 开始，即 M1 先尝试发起 READ 事务。 */
    s_motor_fsm_m1.cycle_start_tick_ms = HAL_GetTick();
    s_motor_fsm_m1.read_transaction =
        APP_LD2RS_READ_TRANSACTION_STATUS_SPEED;
    s_motor_fsm_m1.phase = APP_LD2RS_TASK_PHASE_READ_REQ;
}





