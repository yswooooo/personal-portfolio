/**
 * @file bsp_can_robostride.c
 * @brief ROBSTRIDE 舵向电机 CAN BSP 实现。
 *
 * BSP 层职责：
 * - 调用 HAL_FDCAN 初始化和收发。
 * - 在 CAN RX 中断中分发到 Middleware 层 robstride_motor_parse_rx_frame()。
 * - 管理 steer_chassis_t 的模块绑定和 TX 轮询调度。
 *
 * BSP 层不负责：
 * - 协议打包/解包（由 Middleware/robstride_motor 负责）。
 * - 运动学解算（由 App 层负责）。
 */

#include "bsp_can_robostride.h"

#include <string.h>

#include "fdcan.h"

/**
 * @brief Maximum RX FIFO0 frames drained per FDCAN1 ISR.
 *
 * This value is aligned with the CubeMX RxFifo0ElmtsNbr = 16 setting and
 * prevents an abnormal interrupt storm from keeping the ISR in an endless
 * drain loop.
 */
#define BSP_FDCAN1_RX_FIFO0_DRAIN_MAX      16U

/** @brief FDCAN1 RX FIFO0 message-lost event counter for debug observation. */
volatile uint32_t g_fdcan1_rx_fifo0_msg_lost_count = 0U;

/** @brief FDCAN1 RX FIFO0 full event counter for debug observation. */
volatile uint32_t g_fdcan1_rx_fifo0_full_count = 0U;

/** @brief FDCAN1 RX FIFO0 drain-limit hit counter for debug observation. */
volatile uint32_t g_fdcan1_rx_fifo0_drain_limit_count = 0U;

/* ========================================================================
   BSP 内部状态
   ======================================================================== */

/** @brief App 层底盘对象指针，供 ISR 直接写回反馈。 */
 steer_chassis_t *s_chassis_ptr;

/**
 * @brief 按 motor_id 在 chassis 中查找模块。
 * @return 模块指针，未找到返回 NULL。
 */
static steer_module_t *bsp_find_module_by_motor_id(uint8_t motor_id)
{
    uint8_t i;

    if (s_chassis_ptr == NULL) {
        return NULL;
    }
    for (i = 0U; i < APP_STEER_MODULE_COUNT; i++) 
    {
        if (s_chassis_ptr->modules[i].robstride_motor_id == motor_id) 
        {
            return &s_chassis_ptr->modules[i];
        }
    }
    return NULL;
}

/* ========================================================================
   公共发送函数（BSP 层唯一直接调 HAL_FDCAN 发送的地方）
   ======================================================================== */

/**
 * @brief 将已打包的 extended_id + 8 字节数据通过 FDCAN1 发送。
 *
 * 使用 Classic CAN Extended Frame 格式：
 * - 29-bit 扩展 ID
 * - DLC = 8 bytes
 * - BRS OFF, FD Format OFF
 *
 * @param ext_id 由 Middleware 打包函数返回的 29 位扩展 ID。
 * @param data   由 Middleware 打包函数填充的 8 字节数据。
 * @return 1 表示已放入 TX FIFO，0 表示 FIFO 满或发送失败。
 */
volatile uint32_t g_fdcan1_tx_request_count = 0U;
volatile uint32_t g_fdcan1_tx_enqueue_ok_count = 0U;
volatile uint32_t g_fdcan1_tx_fifo_full_count = 0U;
volatile uint32_t g_fdcan1_tx_add_fail_count = 0U;
volatile uint32_t g_fdcan1_tx_last_hal_error = 0U;

static uint8_t bsp_can_robostride_send_frame(uint32_t ext_id, const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef tx_header;

    if (data == NULL) {
        return 0U;
    }

    (void)memset(&tx_header, 0, sizeof(tx_header));

    tx_header.Identifier          = ext_id;
    tx_header.IdType              = FDCAN_EXTENDED_ID;
    tx_header.TxFrameType         = FDCAN_DATA_FRAME;
    tx_header.DataLength          = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch       = FDCAN_BRS_OFF;
    tx_header.FDFormat            = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker       = 0U;

    g_fdcan1_tx_request_count++;

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U) {
        g_fdcan1_tx_fifo_full_count++;
        return 0U;
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, (uint8_t *)data) == HAL_OK) {
        g_fdcan1_tx_enqueue_ok_count++;
        return 1U;
    }

    g_fdcan1_tx_add_fail_count++;
    g_fdcan1_tx_last_hal_error = HAL_FDCAN_GetError(&hfdcan1);
    return 0U;
}

/* ========================================================================
   FDCAN 初始化
   ======================================================================== */

/**
 * @brief 初始化 FDCAN1 过滤器并启动外设。
 *
 * 过滤器不过滤任何 ID（mask=0），所有扩展帧均进入 RX FIFO0。
 * 电机区分完全依靠扩展 ID 中的 motor CAN ID 字段。
 */
void bsp_can_robostride_init(void)
{
    FDCAN_FilterTypeDef filter_config;

    (void)memset(&filter_config, 0, sizeof(filter_config));
    filter_config.IdType       = FDCAN_EXTENDED_ID;
    filter_config.FilterIndex  = 0U;
    filter_config.FilterType   = FDCAN_FILTER_MASK;
    filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter_config.FilterID1    = 0x00000000U;
    filter_config.FilterID2    = 0x00000000U;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter_config) != HAL_OK) {
        return;
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK) {
        return;
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        return;
    }

    (void)HAL_FDCAN_ActivateNotification(&hfdcan1,
                                         FDCAN_IT_RX_FIFO0_NEW_MESSAGE
                                         | FDCAN_IT_RX_FIFO0_MESSAGE_LOST
                                         | FDCAN_IT_RX_FIFO0_FULL,
                                         0U);
}

/* ========================================================================
   舵向模块初始化
   ======================================================================== */

/**
 * @brief 初始化单个舵向模块结构体。
 */
static void bsp_can_robostride_init_module(steer_module_t *module,
                                           steer_module_index_e index,
                                           uint8_t motor_id)
{
    if (module == NULL) {
        return;
    }
    (void)memset(module, 0, sizeof(*module));
    module->index              = index;
    module->robstride_motor_id = motor_id;
}

/**
 * @brief 初始化四个舵向模块，绑定电机 CAN ID。
 */
void bsp_can_robostride_init_steer_chassis(steer_chassis_t *chassis)
{
    if (chassis == NULL) {
        return;
    }

    (void)memset(chassis, 0, sizeof(*chassis));
    chassis->robstride_next_tx_index = APP_STEER_MODULE_FL;
    s_chassis_ptr = chassis;

    bsp_can_robostride_init_module(&chassis->modules[APP_STEER_MODULE_FL],
                                   APP_STEER_MODULE_FL,
                                   ROBSTRIDE_STEER_MOTOR_CAN_ID_FL);
    bsp_can_robostride_init_module(&chassis->modules[APP_STEER_MODULE_RL],
                                   APP_STEER_MODULE_RL,
                                   ROBSTRIDE_STEER_MOTOR_CAN_ID_RL);
    bsp_can_robostride_init_module(&chassis->modules[APP_STEER_MODULE_RR],
                                   APP_STEER_MODULE_RR,
                                   ROBSTRIDE_STEER_MOTOR_CAN_ID_RR);
    bsp_can_robostride_init_module(&chassis->modules[APP_STEER_MODULE_FR],
                                   APP_STEER_MODULE_FR,
                                   ROBSTRIDE_STEER_MOTOR_CAN_ID_FR);
}

/* ========================================================================
   启动配置（上电一次，允许阻塞）
   ======================================================================== */

/**
 * @brief 阻塞等待帧间隔满足。
 */
static void bsp_can_robostride_wait_tx_interval(steer_chassis_t *chassis)
{
    while ((HAL_GetTick() - chassis->robstride_last_tx_tick_ms) < ROBSTRIDE_CAN_TX_INTERVAL_MS) {
        /* 阻塞等待，仅在上电初始化阶段使用 */
    }
}

/**
 * @brief 发送一帧并更新 tick。
 */
static uint8_t bsp_can_robostride_tx_and_tick(steer_chassis_t *chassis,
                                              steer_module_t *module,
                                              uint32_t ext_id,
                                              const uint8_t data[8])
{
    uint32_t now_tick;

    if (bsp_can_robostride_send_frame(ext_id, data) == 0U) {
        return 0U;
    }

    now_tick = HAL_GetTick();
    module->robstride_last_tx_tick_ms      = now_tick;
    chassis->robstride_last_tx_tick_ms     = now_tick;
    return 1U;
}

/**
 * @brief PP 启动流程：每电机 10 步。
 *
 * ① mode=0x12 → 写 run_mode=1 (PP 位置模式)
 * ② mode=0x03 → 使能电机
 * ③ mode=0x12 → 写 EPScan_time=3 (20ms)
 * ④ mode=0x18 → 使能周期反馈上报
 * ⑤ mode=0x12 → 写 vel_max (PP 最大速度)
 * ⑥ mode=0x12 → 写 acc_set (PP 加速度)
 * ⑦ mode=0x12 → 写 loc_ref (初始位置)
 * ⑧ mode=0x12 → 写 spd_kp / loc_kp
 * ⑨ mode=0x12 → 写 zero_sta=1 (-π~π)
 * ⑩ mode=0x16 → 保存全部参数到 Flash
 */
void bsp_can_robostride_start_steer_chassis(steer_chassis_t *chassis)
{
    uint8_t i;
    uint8_t tx_data[8];

    if (chassis == NULL) {
        return;
    }

    for (i = 0U; i < APP_STEER_MODULE_COUNT; i++) {
        steer_module_t *module = &chassis->modules[i];
        uint8_t motor_id = module->robstride_motor_id;
        uint32_t ext_id;

        /* ① 写 PP 模式 (run_mode=1) */
        bsp_can_robostride_wait_tx_interval(chassis);
        ext_id = robstride_motor_pack_run_mode_pp(motor_id, tx_data);
        bsp_can_robostride_tx_and_tick(chassis, module, ext_id, tx_data);

//        /* ② 使能电机 */
         bsp_can_robostride_wait_tx_interval(chassis);
         ext_id = robstride_motor_pack_enable(motor_id, tx_data);
         bsp_can_robostride_tx_and_tick(chassis, module, ext_id, tx_data);


        /* ④ 使能周期反馈上报 */

        bsp_can_robostride_wait_tx_interval(chassis);
        ext_id = robstride_motor_pack_enable_report(motor_id, tx_data);
        bsp_can_robostride_tx_and_tick(chassis, module, ext_id, tx_data);

        /* ③ 设置上报周期为 20ms (EPScan_time=3, 1→10ms, 每+1+5ms) */
        bsp_can_robostride_wait_tx_interval(chassis);
        ext_id = robstride_motor_pack_epscan_time(motor_id, 3U, tx_data);
        bsp_can_robostride_tx_and_tick(chassis, module, ext_id, tx_data);

        /* ④ 写 PP 最大速度 */
        bsp_can_robostride_wait_tx_interval(chassis);
        ext_id = robstride_motor_pack_vel_max(motor_id,
                                              ROBSTRIDE_PP_DEFAULT_VEL_MAX_RAD_S,
                                              tx_data);
        bsp_can_robostride_tx_and_tick(chassis, module, ext_id, tx_data);

        /* ⑤ 写 PP 加速度 */
        bsp_can_robostride_wait_tx_interval(chassis);
        ext_id = robstride_motor_pack_acc_set(motor_id,
                                              ROBSTRIDE_PP_DEFAULT_ACC_SET_RAD_S2,
                                              tx_data);
        bsp_can_robostride_tx_and_tick(chassis, module, ext_id, tx_data);

        /* ⑥ 写初始位置 loc_ref=0（上电回零） */
        bsp_can_robostride_wait_tx_interval(chassis);
        ext_id = robstride_motor_pack_position_ref(motor_id, module->steer_zero_offset_rad, tx_data);
        bsp_can_robostride_tx_and_tick(chassis, module, ext_id, tx_data);

        /* ⑦ 记录 PP 参数到 debug_param (已在 ④⑤ 写入电机) */
        module->debug_param.vel_max = ROBSTRIDE_PP_DEFAULT_VEL_MAX_RAD_S;
        module->debug_param.acc_set = ROBSTRIDE_PP_DEFAULT_ACC_SET_RAD_S2;

        /* ⑧ 写 PID 参数 (电流环 + 速度环 + 位置环) */
        // module->debug_param.cur_kp = 0.125f;
        // bsp_can_robostride_wait_tx_interval(chassis);
        // ext_id = robstride_motor_pack_cur_kp(motor_id, module->debug_param.cur_kp, tx_data);
        // bsp_can_robostride_tx_and_tick(chassis, module, ext_id, tx_data);

        // module->debug_param.cur_ki = 0.0158f;
        // bsp_can_robostride_wait_tx_interval(chassis);
        // ext_id = robstride_motor_pack_cur_ki(motor_id, module->debug_param.cur_ki, tx_data);
        // bsp_can_robostride_tx_and_tick(chassis, module, ext_id, tx_data);

        module->debug_param.spd_kp = 180.0f;
        bsp_can_robostride_wait_tx_interval(chassis);
        ext_id = robstride_motor_pack_spd_kp(motor_id, module->debug_param.spd_kp, tx_data);
        bsp_can_robostride_tx_and_tick(chassis, module, ext_id, tx_data);

        // module->debug_param.spd_ki = 0.1f;
        // bsp_can_robostride_wait_tx_interval(chassis);
        // ext_id = robstride_motor_pack_spd_ki(motor_id, module->debug_param.spd_ki, tx_data);
        // bsp_can_robostride_tx_and_tick(chassis, module, ext_id, tx_data);

        module->debug_param.loc_kp = 150.0f;
        bsp_can_robostride_wait_tx_interval(chassis);
        ext_id = robstride_motor_pack_loc_kp(motor_id, module->debug_param.loc_kp, tx_data);
        bsp_can_robostride_tx_and_tick(chassis, module, ext_id, tx_data);

        /* ⑨ 设置零点标志位 -π~π */
        bsp_can_robostride_wait_tx_interval(chassis);
        ext_id = robstride_motor_pack_zero_sta(motor_id, ROBSTRIDE_DEFAULT_ZERO_STA_MODE, tx_data);
        bsp_can_robostride_tx_and_tick(chassis, module, ext_id, tx_data);

        /* ⑩ 保存全部参数到 Flash (type 22, 掉电不丢) */
        bsp_can_robostride_wait_tx_interval(chassis);
        ext_id = robstride_motor_pack_save_params(motor_id, tx_data);
        bsp_can_robostride_tx_and_tick(chassis, module, ext_id, tx_data);
    }

    chassis->robstride_next_tx_index = APP_STEER_MODULE_FL;
}

/* ========================================================================
   停止命令
   ======================================================================== */

void bsp_can_robostride_stop_steer_chassis(steer_chassis_t *chassis,
                                           uint8_t clear_error)
{
    uint8_t i;
    uint8_t tx_data[8];

    if (chassis == NULL) {
        return;
    }

    for (i = 0U; i < APP_STEER_MODULE_COUNT; i++) {
        steer_module_t *module = &chassis->modules[i];
        uint32_t ext_id;

        /* ① 发送停止命令（mode=0x04） */
        while ((HAL_GetTick() - chassis->robstride_last_tx_tick_ms) < ROBSTRIDE_CAN_TX_INTERVAL_MS) {
        }

        ext_id = robstride_motor_pack_stop(module->robstride_motor_id, clear_error, tx_data);
        if (bsp_can_robostride_send_frame(ext_id, tx_data) != 0U) {
            uint32_t now_tick = HAL_GetTick();
            module->robstride_last_tx_tick_ms  = now_tick;
            chassis->robstride_last_tx_tick_ms = now_tick;
        }

        /*
         * ② 切回空闲模式（对照官方例程 Disenable_Motor）：
         *    Set_RobStride_Motor_parameter(0X7005, move_control_mode, Set_mode);
         */
        while ((HAL_GetTick() - chassis->robstride_last_tx_tick_ms) < ROBSTRIDE_CAN_TX_INTERVAL_MS) {
        }

        ext_id = robstride_motor_pack_run_mode(module->robstride_motor_id,
                                               ROBSTRIDE_RUN_MODE_IDLE, tx_data);
        if (bsp_can_robostride_send_frame(ext_id, tx_data) != 0U) {
            uint32_t now_tick = HAL_GetTick();
            module->robstride_last_tx_tick_ms  = now_tick;
            chassis->robstride_last_tx_tick_ms = now_tick;
        }
    }
}

/* ========================================================================
   一次性改 CAN ID（出厂 0x7F → new_id）
   ======================================================================== */

void bsp_can_robostride_set_motor_id_once(uint8_t old_id, uint8_t new_id)
{
    uint8_t tx_data[8];
    uint32_t ext_id;
    uint32_t now_tick;

    /*
     * ① 发 0x07 改 CAN ID (old_id → new_id)
     *    FDCAN1 必须在之前已初始化（bsp_can_robostride_init）
     */
    ext_id = robstride_motor_pack_set_can_id(old_id, new_id, tx_data);
    (void)bsp_can_robostride_send_frame(ext_id, tx_data);

    /* 等 1ms 帧间隔 */
    now_tick = HAL_GetTick();
    while ((HAL_GetTick() - now_tick) < 1U) {}

    /*
     * ② 发 0x16 保存到 Flash（用新 ID 寻址）
     *    改完 ID 后电机立即以新 ID 响应，所以保存帧要发给新 ID
     */
    ext_id = robstride_motor_pack_save_params(new_id, tx_data);
    (void)bsp_can_robostride_send_frame(ext_id, tx_data);

    /* 此时电机已改为 new_id 并保存。断电后不丢失。 */
}

/* ========================================================================
   运行期轮询发送
   ======================================================================== */

/**
 * @brief 按 FL→RL→RR→FR 顺序轮询发送舵向目标角度。
 *
 * 每次调用最多发送 1 帧；运行期绝不阻塞。
 */

void bsp_can_robostride_send_steer_chassis_position(steer_chassis_t *chassis)
{
    uint8_t  i;
    uint8_t  tx_data[8];

    if (chassis == NULL) {
        return;
    }

    //  static uint32_t last_tick_4_motor = 0;
    //  /*4个电机的总发送周期间隔为20ms*/
    // if((HAL_GetTick() - last_tick_4_motor >  ROBSTRIDE_CAN_TX_INTERVAL_MS_4_MOTOR))
    // {
        // last_tick_4_motor = HAL_GetTick();
    
    /* for 循环依次检查 4 个电机，距上次发送 ≥1ms 才发 */
      for (i = 0U; i < APP_STEER_MODULE_COUNT; i++) 
			{
            steer_module_t *m = &chassis->modules[i];
            uint32_t now_tick = HAL_GetTick();
            uint32_t ext_id;

//            if ((now_tick - m->robstride_last_tx_tick_ms) < ROBSTRIDE_CAN_TX_INTERVAL_MS) {
//                continue;
//            }

            ext_id = robstride_motor_pack_position_ref(m->robstride_motor_id,
                                                    m->robstride_target_angle_rad, tx_data);
            if (bsp_can_robostride_send_frame(ext_id, tx_data) != 0U) {
                m->robstride_last_tx_tick_ms = now_tick;
            // }
					
        }
    }
 
}

/* ========================================================================
   HAL FDCAN RX FIFO0 中断回调
   ======================================================================== */

/**
 * @brief FDCAN1 RX FIFO0 新消息中断回调。
 *
 * 中断上下文约束：
 * - 不做舵轮运动学、速度斜坡、printf、阻塞等待、大量循环。
 * - 只做帧验证 + 调 Middleware 解包。
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo0_its)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t               rx_data[8];
    uint32_t              drain_count = 0U;

    /* 只处理 FDCAN1 的 RX FIFO0 新消息中断 */
    if (hfdcan != &hfdcan1) {
        return;
    }

    if ((rx_fifo0_its & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U) {
        g_fdcan1_rx_fifo0_msg_lost_count++;
    }

    if ((rx_fifo0_its & FDCAN_IT_RX_FIFO0_FULL) != 0U) {
        g_fdcan1_rx_fifo0_full_count++;
    }

    while ((HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
           && (drain_count < BSP_FDCAN1_RX_FIFO0_DRAIN_MAX)) {
        drain_count++;

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
            break;
        }

    /* 只接受扩展帧 + 数据帧 + DLC=8 */
        if ((rx_header.IdType != FDCAN_EXTENDED_ID)
            || (rx_header.RxFrameType != FDCAN_DATA_FRAME)
            || (rx_header.DataLength != FDCAN_DLC_BYTES_8)) {
            continue;
        }

    /* 调 Middleware 解包（内部 switch-case 判断通信类型） */
        if (robstride_motor_parse_rx_frame(rx_header.Identifier, rx_data) != 0U) 
        {
            uint8_t motor_id = (uint8_t)((rx_header.Identifier >> ROBSTRIDE_CAN_ID_SHIFT)
                                         & ROBSTRIDE_CAN_ID_MASK);
            uint32_t now_tick = HAL_GetTick();

        /* 更新 Middleware 缓存的时间戳 */
            robstride_motor_set_last_update_ms(motor_id, now_tick);

        /* 直接写回 chassis 对应模块的 feedback */
            steer_module_t *module = bsp_find_module_by_motor_id(motor_id);
            if (module != NULL) 
            {
                const robstride_motor_feedback_t *fb;
                fb = robstride_motor_get_feedback(motor_id);
                if (fb != NULL) {
                    module->feedback = *fb;
                    module->feedback.last_update_ms = now_tick;

                /* 软件标零: 原始角 - 零偏 = 计算出来的理论车体舵角 */
                    module->feedback.position_new_rad =
                        module->feedback.position_raw_rad - module->steer_zero_offset_rad;
                    module->feedback.position_new_deg =
                        module->feedback.position_new_rad * (180.0f / 3.141592653589793f);
                }
            }
        }

    }

    if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
        g_fdcan1_rx_fifo0_drain_limit_count++;
    }
}
