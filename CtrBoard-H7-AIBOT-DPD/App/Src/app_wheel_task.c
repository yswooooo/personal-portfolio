/**
  ******************************************************************************
  * @file    app_wheel_task.c
  * @brief   四轮轮毂电机非阻塞控制任务实现
  *
  * @details 基于 UART2 RS485 (g_rs485_bus2) + Modbus RTU，使用中断收发。
  *          采用与 app_ld2rs_task.c 相同的分层设计：
  *          - BSP 层：RS485 半双工非阻塞状态机 (bsp_rs485.c)
  *          - Middleware 层：Modbus RTU 协议 + DS/RS/UM 电机寄存器封装
  *          - APP 层（本文件）：4 电机轮询调度 + 测试流程 + 安装极性
  *
  *          单电机 FSM：
  *          IDLE → READ_REQ → WAIT_READ → READ_DONE → WRITE_REQ → WAIT_WRITE → WRITE_DONE → IDLE
  *          - READ：0x03 读 D0.000（实际转速），闭环确认当前转速
  *          - WRITE：0x06 写 F0.3.018（目标转速）；测试结束阶段改写 F0.1.000（使能）
  *
  *          调度器：
  *          4 电机共享一条 RS485 总线，按 FL → RL → RR → FR 顺序轮询。
  *          只有当前电机事务完成并释放总线后，才 advance 到下一台。
  *
  *          运行期由 app_steer_chassis_run() 传入四舵轮运动学结果：
  *          hub_target_speed_mps → rpm，并写入 F0.3.018 目标转速。
  ******************************************************************************
  */

#include "app_wheel_task.h"
#include "app_steer_chassis.h"
#include "bsp_rs485_bus2.h"
#include "modbus_rtu.h"
#include "app_config.h"
#include "stm32h7xx_hal.h"

/* Private defines -----------------------------------------------------------*/
#define APP_WHEEL_MODBUS_READ_QTY_ONE   0x0001u  /**< 单次读 1 个保持寄存器   */
#define APP_WHEEL_PI                    3.14159265358979323846f

/* Private types -------------------------------------------------------------*/

/**
  * @brief  单电机 FSM 单步推进结果
  */
typedef enum {
    APP_WHEEL_FSM_STEP_NONE = 0,          /**< 本步无状态变化                */
    APP_WHEEL_FSM_STEP_STARTED,           /**< 成功发起 TX                   */
    APP_WHEEL_FSM_STEP_WAITING,           /**< 等待总线完成/超时             */
    APP_WHEEL_FSM_STEP_DONE_RELEASED,     /**< 事务完成，总线已释放          */
    APP_WHEEL_FSM_STEP_ERROR_RELEASED     /**< 事务异常，总线已释放          */
} app_wheel_fsm_step_result_t;

/* Private variables ---------------------------------------------------------*/

static app_wheel_fsm_t        s_wheel_fsm_fl;        /**< 左前轮 FSM 实例       */
static app_wheel_fsm_t        s_wheel_fsm_rl;        /**< 左后轮 FSM 实例       */
static app_wheel_fsm_t        s_wheel_fsm_rr;        /**< 右后轮 FSM 实例       */
static app_wheel_fsm_t        s_wheel_fsm_fr;        /**< 右前轮 FSM 实例       */
static app_wheel_scheduler_t  s_wheel_scheduler;     /**< 4 电机轮询调度器      */
static app_wheel_test_phase_t s_test_phase;          /**< 当前测试阶段          */
static uint32_t               s_test_phase_start_tick; /**< 当前阶段起始 tick    */

/* Private function prototypes -----------------------------------------------*/

static void app_wheel_fsm_init(app_wheel_fsm_t *fsm, ds_rs_um_motor_handle_t *dev, int8_t polarity);
static app_wheel_fsm_step_result_t app_wheel_fsm_step(app_wheel_fsm_t *fsm, bsp_rs485_state_t bus_state);
static void app_wheel_scheduler_init(app_wheel_scheduler_t *scheduler, uint8_t count);
static void app_wheel_scheduler_step(app_wheel_scheduler_t *scheduler, bsp_rs485_state_t bus_state);
static void app_wheel_scheduler_advance(app_wheel_scheduler_t *scheduler);

static void app_wheel_put_u16(uint8_t *buffer, uint16_t value);
static uint16_t app_wheel_get_u16(const uint8_t *buffer);
static void app_wheel_build_read_req(uint8_t *frame, uint8_t slave_id, uint16_t reg_addr);
static void app_wheel_build_write_req(uint8_t *frame, uint8_t slave_id,
                                      uint16_t reg_addr, uint16_t value);
static uint8_t app_wheel_validate_resp(const uint8_t *rx_buffer, uint16_t rx_len,
                                       uint8_t expected_slave_id, uint8_t expected_func);
static int16_t app_wheel_mps_to_rpm(float speed_mps, uint8_t reverse_flag, int8_t polarity);
static void app_wheel_apply_steer_chassis(const steer_chassis_t *steer_chassis);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  将 uint16_t 以大端序写入缓冲区（Modbus 网络字节序）
  */
static void app_wheel_put_u16(uint8_t *buffer, uint16_t value)
{
    buffer[0] = (uint8_t)(value >> 8);
    buffer[1] = (uint8_t)(value & 0xFFu);
}

/**
  * @brief  从缓冲区以大端序读取 uint16_t（Modbus 网络字节序）
  */
static uint16_t app_wheel_get_u16(const uint8_t *buffer)
{
    return (uint16_t)(((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1]);
}

/**
  * @brief  组装 Modbus 0x03 读保持寄存器请求帧（8 字节）
  * @param  frame     输出帧缓冲区，长度 8
  * @param  slave_id  从机站号
  * @param  reg_addr  寄存器地址
  */
static void app_wheel_build_read_req(uint8_t *frame, uint8_t slave_id, uint16_t reg_addr)
{
    uint16_t crc;
    frame[0] = slave_id;
    frame[1] = MODBUS_RTU_FC_READ_HOLDING;
    app_wheel_put_u16(&frame[2], reg_addr);
    app_wheel_put_u16(&frame[4], APP_WHEEL_MODBUS_READ_QTY_ONE);
    crc = modbus_rtu_crc16(frame, 6u);
    frame[6] = (uint8_t)(crc & 0xFFu);
    frame[7] = (uint8_t)(crc >> 8);
}

/**
  * @brief  组装 Modbus 0x06 写单个寄存器请求帧（8 字节）
  * @param  frame     输出帧缓冲区，长度 8
  * @param  slave_id  从机站号
  * @param  reg_addr  寄存器地址
  * @param  value     写入值
  */
static void app_wheel_build_write_req(uint8_t *frame, uint8_t slave_id,
                                      uint16_t reg_addr, uint16_t value)
{
    uint16_t crc;
    frame[0] = slave_id;
    frame[1] = MODBUS_RTU_FC_WRITE_SINGLE;
    app_wheel_put_u16(&frame[2], reg_addr);
    app_wheel_put_u16(&frame[4], value);
    crc = modbus_rtu_crc16(frame, 6u);
    frame[6] = (uint8_t)(crc & 0xFFu);
    frame[7] = (uint8_t)(crc >> 8);
}

/**
  * @brief  校验 Modbus 应答帧
  *
  * @details 检查内容：最小长度、CRC、站号、功能码、异常码。
  *          0x03 应答最小 7 字节；0x06 应答最小 8 字节。
  *
  * @param  rx_buffer           接收帧缓冲区
  * @param  rx_len              接收帧长度
  * @param  expected_slave_id   期望站号
  * @param  expected_func       期望功能码
  * @retval 1 校验通过，0 校验失败
  */
static uint8_t app_wheel_validate_resp(const uint8_t *rx_buffer, uint16_t rx_len,
                                       uint8_t expected_slave_id, uint8_t expected_func)
{
    uint16_t min_len;
    uint16_t crc;

    if (expected_func == MODBUS_RTU_FC_READ_HOLDING) {
        min_len = 7u;  /* ID + FC + byteCnt + 2×data + CRC */
    } else {
        min_len = 8u;  /* ID + FC + addrH + addrL + valH + valL + CRC */
    }

    if (rx_len < min_len) {
        return 0u;
    }

    crc = modbus_rtu_crc16(rx_buffer, rx_len);
    if (crc != 0u) {
        return 0u;
    }

    if (rx_buffer[0] != expected_slave_id) {
        return 0u;
    }

    if ((rx_buffer[1] & 0x80u) != 0u) {
        return 0u;  /* 异常应答 */
    }

    if (rx_buffer[1] != expected_func) {
        return 0u;
    }

    return 1u;
}

/**
  * @brief  将轮毂线速度 m/s 转换为驱动器目标转速 rpm。
  *
  * @details app_steer_chassis_run() 计算得到的是轮心线速度幅值。
  *          reverse_flag 决定轮毂方向，polarity 补偿左右侧机械安装方向。
  */
static int16_t app_wheel_mps_to_rpm(float speed_mps, uint8_t reverse_flag, int8_t polarity)
{
    float signed_speed_mps;
    float wheel_rpm;
    int32_t rpm_i32;

    if (speed_mps < 0.0f) {
        speed_mps = -speed_mps;
    }

    signed_speed_mps = speed_mps * ((reverse_flag != 0u) ? -1.0f : 1.0f);
    wheel_rpm = signed_speed_mps
              * 60.0f
              * APP_STEER_CHASSIS_DRIVE_GEAR_RATIO
              / (2.0f * APP_WHEEL_PI * APP_STEER_CHASSIS_WHEEL_RADIUS_M);
    wheel_rpm *= (float)polarity;

    rpm_i32 = (wheel_rpm >= 0.0f) ? (int32_t)(wheel_rpm + 0.5f)
                                  : (int32_t)(wheel_rpm - 0.5f);

    // if (rpm_i32 > DS_RS_UM_MOTOR_SPEED_MAX_RPM) {
    //     rpm_i32 = DS_RS_UM_MOTOR_SPEED_MAX_RPM;
    // } else if (rpm_i32 < DS_RS_UM_MOTOR_SPEED_MIN_RPM) {
    //     rpm_i32 = DS_RS_UM_MOTOR_SPEED_MIN_RPM;
    // }

    return (int16_t)rpm_i32;
}

/**
  * @brief  根据舵轮运动学结果刷新四个轮毂 FSM 的目标转速。
  */
static void app_wheel_apply_steer_chassis(const steer_chassis_t *steer_chassis)
{
    if (steer_chassis == NULL) {
        s_wheel_fsm_fl.target_speed_rpm = 0;
        s_wheel_fsm_rl.target_speed_rpm = 0;
        s_wheel_fsm_rr.target_speed_rpm = 0;
        s_wheel_fsm_fr.target_speed_rpm = 0;
        return;
    }
    {
            s_wheel_fsm_fl.target_speed_rpm =
                    app_wheel_mps_to_rpm(steer_chassis->modules[APP_STEER_MODULE_FL].hub_target_speed_mps,
                                                                steer_chassis->modules[APP_STEER_MODULE_FL].hub_reverse_flag,
                                                                s_wheel_fsm_fl.polarity);
            s_wheel_fsm_rl.target_speed_rpm =
                    app_wheel_mps_to_rpm(steer_chassis->modules[APP_STEER_MODULE_RL].hub_target_speed_mps,
                                                                steer_chassis->modules[APP_STEER_MODULE_RL].hub_reverse_flag,
                                                                s_wheel_fsm_rl.polarity);
            s_wheel_fsm_rr.target_speed_rpm =
                    app_wheel_mps_to_rpm(steer_chassis->modules[APP_STEER_MODULE_RR].hub_target_speed_mps,
                                                                steer_chassis->modules[APP_STEER_MODULE_RR].hub_reverse_flag,
                                                                s_wheel_fsm_rr.polarity);
            s_wheel_fsm_fr.target_speed_rpm =
                    app_wheel_mps_to_rpm(steer_chassis->modules[APP_STEER_MODULE_FR].hub_target_speed_mps,
                                                                steer_chassis->modules[APP_STEER_MODULE_FR].hub_reverse_flag,
                                                                s_wheel_fsm_fr.polarity);
    }

    static float fl_target_speed_rpm_abs = 0.0f;
    static float rl_target_speed_rpm_abs = 0.0f;
    static float rr_target_speed_rpm_abs = 0.0f;
    static float fr_target_speed_rpm_abs = 0.0f;

    static float max_target_speed_rpm_abs = 0.0f;
    static float speed_scale = 1.0f;

    /* 保存四轮转速绝对值 */
    fl_target_speed_rpm_abs =
        fabsf((float)s_wheel_fsm_fl.target_speed_rpm);

    rl_target_speed_rpm_abs =
        fabsf((float)s_wheel_fsm_rl.target_speed_rpm);

    rr_target_speed_rpm_abs =
        fabsf((float)s_wheel_fsm_rr.target_speed_rpm);

    fr_target_speed_rpm_abs =
        fabsf((float)s_wheel_fsm_fr.target_speed_rpm);

        /* 找出最大转速模 */
    max_target_speed_rpm_abs = fl_target_speed_rpm_abs;

    max_target_speed_rpm_abs =
        fmaxf(max_target_speed_rpm_abs, rl_target_speed_rpm_abs);

    max_target_speed_rpm_abs =
        fmaxf(max_target_speed_rpm_abs, rr_target_speed_rpm_abs);

    max_target_speed_rpm_abs =
        fmaxf(max_target_speed_rpm_abs, fr_target_speed_rpm_abs);
        /* 计算四轮统一限幅比例 */
    if (max_target_speed_rpm_abs <= (float)DS_RS_UM_MOTOR_SPEED_MAX_RPM)
    {
        speed_scale = 1.0f;
    }
    else
    {
        /*
        * 进入此分支时最大转速必然大于200 rpm，
        * 因此分母不可能为0。
        */
        speed_scale =
            (float)DS_RS_UM_MOTOR_SPEED_MAX_RPM
            / max_target_speed_rpm_abs;
    }

    /* 四个带符号RPM同比例缩放 */
    s_wheel_fsm_fl.target_speed_rpm =
        (int16_t)((float)s_wheel_fsm_fl.target_speed_rpm * speed_scale);

    s_wheel_fsm_rl.target_speed_rpm =
        (int16_t)((float)s_wheel_fsm_rl.target_speed_rpm * speed_scale);

    s_wheel_fsm_rr.target_speed_rpm =
        (int16_t)((float)s_wheel_fsm_rr.target_speed_rpm * speed_scale);

    s_wheel_fsm_fr.target_speed_rpm =
        (int16_t)((float)s_wheel_fsm_fr.target_speed_rpm * speed_scale);
						
}

/**
  * @brief  初始化单个电机 FSM 实例
  * @param  fsm       电机 FSM 实例
  * @param  dev       DS/RS/UM 电机设备句柄
  * @param  polarity  机械安装极性（预留，后续运动学接入时使用）
  */
static void app_wheel_fsm_init(app_wheel_fsm_t *fsm, ds_rs_um_motor_handle_t *dev, int8_t polarity)
{
    if (fsm == NULL) {
        return;
    }

    fsm->dev                  = dev;
    fsm->state                = APP_WHEEL_FSM_STATE_IDLE;
    fsm->polarity             = polarity;
    fsm->target_speed_rpm     = 0;
    fsm->feedback_speed_rpm   = 0;
    fsm->status_word          = 0u;
    fsm->read_retry_count     = 0u;
    fsm->write_retry_count    = 0u;
    fsm->offline_miss_count   = 0u;
    fsm->is_offline_confirmed = 0u;
    fsm->disable_done         = 0u;
    fsm->tx_start_tick_ms     = 0u;
}

/**
  * @brief  单步推进电机 FSM
  *
  * @details 状态转换：
  *          IDLE → READ_REQ → WAIT_READ → READ_DONE → WRITE_REQ → WAIT_WRITE → WRITE_DONE → IDLE
  *
  *          仅在 BSP 总线 IDLE 时发起新 TX；WAIT 阶段由上层统一传入 bus_state。
  *          应答校验失败/超时后，读/写各自最多重试 APP_WHEEL_MAX_RETRY 次。
  *
  * @param  fsm        电机 FSM 实例
  * @param  bus_state  当前 RS485 总线状态（由上层统一 Poll 得到）
  * @retval APP_WHEEL_FSM_STEP_* 单步结果
  */
static app_wheel_fsm_step_result_t app_wheel_fsm_step(app_wheel_fsm_t *fsm,
                                                      bsp_rs485_state_t bus_state)
{
    uint32_t now_tick;
    bsp_rs485_handle_t *bus = bsp_rs485_bus2_get_handle();

    if ((fsm == NULL) || (fsm->dev == NULL) || (bus == NULL)) {
        return APP_WHEEL_FSM_STEP_NONE;
    }

    now_tick = HAL_GetTick();

    switch (fsm->state) {

    /* IDLE：新周期开始，复位重试计数并进入读请求 */
    case APP_WHEEL_FSM_STATE_IDLE:
        fsm->read_retry_count  = 0u;
        fsm->write_retry_count = 0u;
        fsm->state = APP_WHEEL_FSM_STATE_READ_REQ;
        break;

    /* READ_REQ：总线空闲时组装并发送 0x03 读 D0.000 请求 */
    case APP_WHEEL_FSM_STATE_READ_REQ:
        if (bus_state != BSP_RS485_STATE_IDLE) {
            return APP_WHEEL_FSM_STEP_WAITING;
        }
        app_wheel_build_read_req(fsm->tx_frame, fsm->dev->slave_id,
                                 DS_RS_UM_MOTOR_REG_ACTUAL_SPEED);
        if (bsp_rs485_start_tx(bus,
                               fsm->tx_frame, sizeof(fsm->tx_frame),
                               fsm->rx_frame, sizeof(fsm->rx_frame),
                               APP_WHEEL_TIMEOUT_MS) == BSP_RS485_STATUS_OK) {
            fsm->tx_start_tick_ms = now_tick;
            fsm->state = APP_WHEEL_FSM_STATE_WAIT_READ;
            return APP_WHEEL_FSM_STEP_STARTED;
        }
        break;

    /* WAIT_READ：等待从机应答，校验后提取实际转速 */
    case APP_WHEEL_FSM_STATE_WAIT_READ:
        if (bus_state == BSP_RS485_STATE_DONE) {
            if (app_wheel_validate_resp(bus->rx_buffer, bus->rx_len,
                                        fsm->dev->slave_id,
                                        MODBUS_RTU_FC_READ_HOLDING)) {
                /* 0x03 应答：byte[2]=字节数，byte[3..4]=寄存器值 */
                fsm->feedback_speed_rpm = (int16_t)app_wheel_get_u16(&bus->rx_buffer[3]);
                fsm->read_retry_count   = 0u;
                fsm->offline_miss_count = 0u;
                fsm->is_offline_confirmed = 0u;
                fsm->state = APP_WHEEL_FSM_STATE_READ_DONE;
            } else {
                /* 校验失败，逻辑重试 */
                fsm->read_retry_count++;
                if (fsm->read_retry_count >= APP_WHEEL_MAX_RETRY) {
                    fsm->state = APP_WHEEL_FSM_STATE_WRITE_REQ;
                } else {
                    fsm->state = APP_WHEEL_FSM_STATE_READ_REQ;
                }
            }
            bsp_rs485_ack_done(bus);
            return APP_WHEEL_FSM_STEP_DONE_RELEASED;
        } else if ((bus_state == BSP_RS485_STATE_TIMEOUT)
                   || (bus_state == BSP_RS485_STATE_ERROR)) {
            /* 超时/错误：记录离线，逻辑重试 */
            fsm->offline_miss_count++;
            if (fsm->offline_miss_count >= APP_WHEEL_OFFLINE_CONFIRM_COUNT) {
                fsm->is_offline_confirmed = 1u;
            }
            fsm->read_retry_count++;
            if (fsm->read_retry_count >= APP_WHEEL_MAX_RETRY) {
                fsm->state = APP_WHEEL_FSM_STATE_WRITE_REQ;
            } else {
                fsm->state = APP_WHEEL_FSM_STATE_READ_REQ;
            }
            bsp_rs485_ack_done(bus);
            return APP_WHEEL_FSM_STEP_ERROR_RELEASED;
        }
        return APP_WHEEL_FSM_STEP_WAITING;

    /* READ_DONE：读完成，准备写目标速度 */
    case APP_WHEEL_FSM_STATE_READ_DONE:
        fsm->state = APP_WHEEL_FSM_STATE_WRITE_REQ;
        break;

    /* WRITE_REQ：根据当前测试阶段决定写速度还是写使能 */
    case APP_WHEEL_FSM_STATE_WRITE_REQ:
        if (bus_state != BSP_RS485_STATE_IDLE) {
            return APP_WHEEL_FSM_STEP_WAITING;
        }
        {
            uint16_t reg_addr;
            uint16_t reg_value;

            if (s_test_phase == APP_WHEEL_TEST_PHASE_DISABLE) {
                /* 测试结束阶段：写 F0.1.000 = 0 失能电机 */
                reg_addr = DS_RS_UM_MOTOR_REG_ENABLE;
                reg_value = DS_RS_UM_MOTOR_ENABLE_OFF;
            } else {
                /* 正常运行阶段：写 F0.3.018 目标转速 */
                reg_addr = DS_RS_UM_MOTOR_REG_TARGET_SPEED;
                reg_value = (uint16_t)fsm->target_speed_rpm;
            }

            app_wheel_build_write_req(fsm->tx_frame, fsm->dev->slave_id,
                                      reg_addr, reg_value);
        }
        if (bsp_rs485_start_tx(bus,
                               fsm->tx_frame, sizeof(fsm->tx_frame),
                               fsm->rx_frame, sizeof(fsm->rx_frame),
                               APP_WHEEL_TIMEOUT_MS) == BSP_RS485_STATUS_OK) {
            fsm->tx_start_tick_ms = now_tick;
            fsm->state = APP_WHEEL_FSM_STATE_WAIT_WRITE;
            return APP_WHEEL_FSM_STEP_STARTED;
        }
        break;

    /* WAIT_WRITE：等待写应答，成功后标记失能完成（若处于 DISABLE 阶段） */
    case APP_WHEEL_FSM_STATE_WAIT_WRITE:
        if (bus_state == BSP_RS485_STATE_DONE) {
            if (app_wheel_validate_resp(bus->rx_buffer, bus->rx_len,
                                        fsm->dev->slave_id,
                                        MODBUS_RTU_FC_WRITE_SINGLE)) {
                fsm->write_retry_count  = 0u;
                fsm->offline_miss_count = 0u;
                fsm->is_offline_confirmed = 0u;
                if (s_test_phase == APP_WHEEL_TEST_PHASE_DISABLE) {
                    fsm->disable_done = 1u;
                }
            } else {
                fsm->write_retry_count++;
            }
            bsp_rs485_ack_done(bus);
            fsm->state = APP_WHEEL_FSM_STATE_WRITE_DONE;
            return APP_WHEEL_FSM_STEP_DONE_RELEASED;
        } else if ((bus_state == BSP_RS485_STATE_TIMEOUT)
                   || (bus_state == BSP_RS485_STATE_ERROR)) {
            fsm->offline_miss_count++;
            if (fsm->offline_miss_count >= APP_WHEEL_OFFLINE_CONFIRM_COUNT) {
                fsm->is_offline_confirmed = 1u;
            }
            fsm->write_retry_count++;
            bsp_rs485_ack_done(bus);
            fsm->state = APP_WHEEL_FSM_STATE_WRITE_DONE;
            return APP_WHEEL_FSM_STEP_ERROR_RELEASED;
        }
        return APP_WHEEL_FSM_STEP_WAITING;

    /* WRITE_DONE：本电机本轮结束，返回 IDLE 等待下次调度 */
    case APP_WHEEL_FSM_STATE_WRITE_DONE:
        fsm->state = APP_WHEEL_FSM_STATE_IDLE;
        break;

    default:
        fsm->state = APP_WHEEL_FSM_STATE_IDLE;
        break;
    }

    return APP_WHEEL_FSM_STEP_NONE;
}

/**
  * @brief  初始化 FSM 调度器
  * @param  scheduler  调度器实例
  * @param  count      实际电机数量（不超过 APP_WHEEL_MAX_COUNT）
  */
static void app_wheel_scheduler_init(app_wheel_scheduler_t *scheduler, uint8_t count)
{
    uint8_t i;

    if (scheduler == NULL) {
        return;
    }

    for (i = 0u; i < APP_WHEEL_MAX_COUNT; i++) {
        scheduler->table[i] = NULL;
    }

    scheduler->count = (count > APP_WHEEL_MAX_COUNT) ? APP_WHEEL_MAX_COUNT : count;
    scheduler->current_index = 0u;

    scheduler->table[0] = &s_wheel_fsm_fl;
    scheduler->table[1] = &s_wheel_fsm_rl;
    scheduler->table[2] = &s_wheel_fsm_rr;
    scheduler->table[3] = &s_wheel_fsm_fr;
}

/**
  * @brief  调度器游标前进到下一台电机
  */
static void app_wheel_scheduler_advance(app_wheel_scheduler_t *scheduler)
{
    if ((scheduler == NULL) || (scheduler->count == 0u)) {
        return;
    }
    scheduler->current_index = (uint8_t)((scheduler->current_index + 1u) % scheduler->count);
}

/**
  * @brief  调度器单步推进
  *
  * @details 每次只推进 current_index 指向的电机。
  *          若总线处于 DONE/TIMEOUT/ERROR 但当前 FSM 不在 WAIT 态，
  *          说明总线残留状态需要释放，直接 ack_done 即可。
  *          当 FSM 返回 DONE/ERROR_RELEASED 时，advance 到下一台电机。
  *
  * @param  scheduler  调度器实例
  * @param  bus_state  当前 RS485 总线状态
  */
static void app_wheel_scheduler_step(app_wheel_scheduler_t *scheduler,
                                     bsp_rs485_state_t bus_state)
{
    app_wheel_fsm_t *current_fsm;
    app_wheel_fsm_step_result_t result;
    bsp_rs485_handle_t *bus = bsp_rs485_bus2_get_handle();

    if ((scheduler == NULL) || (scheduler->count == 0u)) {
        return;
    }

    if (scheduler->current_index >= scheduler->count) {
        scheduler->current_index = 0u;
    }

    current_fsm = scheduler->table[scheduler->current_index];
    if (current_fsm == NULL) {
        app_wheel_scheduler_advance(scheduler);
        return;
    }

    /* 总线完成/超时/错误但当前 FSM 不在等待态时，释放总线 */
    if (((bus_state == BSP_RS485_STATE_DONE)
         || (bus_state == BSP_RS485_STATE_TIMEOUT)
         || (bus_state == BSP_RS485_STATE_ERROR))
        && (current_fsm->state != APP_WHEEL_FSM_STATE_WAIT_READ)
        && (current_fsm->state != APP_WHEEL_FSM_STATE_WAIT_WRITE)) {
        bsp_rs485_ack_done(bus);
        return;
    }

    result = app_wheel_fsm_step(current_fsm, bus_state);
    if ((result == APP_WHEEL_FSM_STEP_DONE_RELEASED)
        || (result == APP_WHEEL_FSM_STEP_ERROR_RELEASED)) {
        app_wheel_scheduler_advance(scheduler);
    }
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化四轮轮毂电机控制任务
  *
  * @details 初始化 4 电机 FSM 并注册到调度器，
  *          让 FL 优先尝试发起第一次 READ 事务。
  */
void app_wheel_task_init(void)
{
    s_test_phase = APP_WHEEL_TEST_PHASE_DONE;
    s_test_phase_start_tick = HAL_GetTick();

    app_wheel_fsm_init(&s_wheel_fsm_fl, &g_ds_rs_um_wheel_fl, APP_WHEEL_FL_POLARITY);
    app_wheel_fsm_init(&s_wheel_fsm_rl, &g_ds_rs_um_wheel_rl, APP_WHEEL_RL_POLARITY);
    app_wheel_fsm_init(&s_wheel_fsm_rr, &g_ds_rs_um_wheel_rr, APP_WHEEL_RR_POLARITY);
    app_wheel_fsm_init(&s_wheel_fsm_fr, &g_ds_rs_um_wheel_fr, APP_WHEEL_FR_POLARITY);

    app_wheel_scheduler_init(&s_wheel_scheduler, APP_WHEEL_COUNT);

    /* 让第一个电机立即尝试发起读事务，缩短首次闭环延迟 */
    s_wheel_fsm_fl.state = APP_WHEEL_FSM_STATE_READ_REQ;
}

/**
  * @brief  四轮轮毂电机控制任务入口
  *
  * @details 主循环每轮调用：
  *          1. 统一 Poll 一次 UART2 RS485 总线状态。
  *          2. 将 steer_chassis 中的四轮 mps 幅值转换为有符号 rpm。
  *          3. 推进轮询调度器，处理当前电机的一个 Modbus 事务。
  */
void app_wheel_task_run(const steer_chassis_t *steer_chassis)
{
    bsp_rs485_handle_t *bus = bsp_rs485_bus2_get_handle();
    bsp_rs485_state_t bus_state;

    bus_state = bsp_rs485_poll(bus);

    app_wheel_apply_steer_chassis(steer_chassis);

    /* ---- 调度器推进 ---- */
    app_wheel_scheduler_step(&s_wheel_scheduler, bus_state);
}

/**
  * @brief  获取四个轮毂电机当前给定转速
  * @param  target_speed_rpm 输出数组，顺序为 FL、RL、RR、FR
  */
void app_wheel_task_get_target_speed_rpm(int16_t target_speed_rpm[APP_WHEEL_COUNT])
{
    if (target_speed_rpm == NULL) {
        return;
    }

    target_speed_rpm[APP_STEER_MODULE_FL] = s_wheel_fsm_fl.target_speed_rpm;
    target_speed_rpm[APP_STEER_MODULE_RL] = s_wheel_fsm_rl.target_speed_rpm;
    target_speed_rpm[APP_STEER_MODULE_RR] = s_wheel_fsm_rr.target_speed_rpm;
    target_speed_rpm[APP_STEER_MODULE_FR] = s_wheel_fsm_fr.target_speed_rpm;
}

/**
  * @brief  获取当前测试阶段
  * @retval 当前 APP_WHEEL_TEST_PHASE_* 阶段
  */
app_wheel_test_phase_t app_wheel_task_get_phase(void)
{
    return s_test_phase;
}
