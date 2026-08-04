/**
 * @file robstride_motor.c
 * @brief ROBSTRIDE RS00 私有协议 Middleware 实现。
 *
 * Middleware 层约束：
 * - 不包含 HAL / BSP / App 头文件。
 * - 不处理底盘 vx/vy/wz。
 * - 不知道舵轮几何结构。
 * - 打包/解包全部在此层完成。
 */

#include "robstride_motor.h"

#include <string.h>

/* ========================================================================
   内部常量
   ======================================================================== */

/** @brief 2π，用于欧拉角归一化 */
#define ROBSTRIDE_TWO_PI  6.283185307f
/** @brief π */
#define ROBSTRIDE_PI       3.141592654f
/** @brief rad → deg */
#define ROBSTRIDE_RAD_TO_DEG  57.29577951f

/* ========================================================================
   反馈缓存
   ======================================================================== */

/** @brief 4 电机反馈缓存，以 motor_id-1 为索引。 */
static robstride_motor_feedback_t s_feedback_cache[ROBSTRIDE_MOTOR_MAX_COUNT];

/* ========================================================================
   内部工具函数
   ======================================================================== */

/**
 * @brief 从两个字节读取大端 uint16。
 *
 * ROBSTRIDE 反馈帧中 position/speed/torque 按大端 uint16 编码。
 * 对应说明书 4.4.2 节官方打包代码：
 *   tx_data[0] = float_to_uint(val, min, max, 16) >> 8;  // 高字节在前
 *   tx_data[1] = float_to_uint(val, min, max, 16);        // 低字节在后
 */
static uint16_t robstride_u16_be(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

/**
 * @brief 将 uint16 编码值线性映射到物理量。
 *
 * 说明书 4.4 节编码公式的逆运算：
 *   physical = (raw / 65535) * (max - min) + min
 */
static float robstride_uint_to_float(uint16_t raw, float min_val, float max_val)
{
    return ((float)raw) * (max_val - min_val) / ROBSTRIDE_ENCODE_MAX_VAL + min_val;
}

/**
 * @brief 将角度归一化到欧拉角范围 [-π, +π)。
 */
static float robstride_normalize_euler(float angle_rad)
{
    while (angle_rad >= ROBSTRIDE_PI) {
        angle_rad -= ROBSTRIDE_TWO_PI;
    }
    while (angle_rad < -ROBSTRIDE_PI) {
        angle_rad += ROBSTRIDE_TWO_PI;
    }
    return angle_rad;
}

/**
 * @brief 将 motor_id (1~4) 映射到反馈缓存索引 (0~3)。
 * @return 0~3 有效索引，无效时返回 0xFF。
 */
static uint8_t robstride_motor_id_to_index(uint8_t motor_id)
{
    /* 出厂默认 0x7F 映射到 FL 作为 fallback；正式部署后此分支不应被触发。 */
    if (motor_id == 0x7FU) { return 0U; }

    if (motor_id < 1U || motor_id > ROBSTRIDE_MOTOR_MAX_COUNT) {
        return 0xFFU;
    }
    return (uint8_t)(motor_id - 1U);
}

/* ========================================================================
   扩展 ID 组装
   ======================================================================== */

/**
 * @brief 组装 29-bit 扩展 ID。
 *
 * 对应说明书 struct exCanIdInfo：
 *   bits [7:0]   = motor_id
 *   bits [15:8]  = ROBSTRIDE_MASTER_CAN_ID (0xFD)
 *   bits [23:16] = 0x00
 *   bits [28:24] = comm_type
 *   bits [31:29] = 0 (reserved)
 */
uint32_t robstride_motor_build_ext_id(uint8_t motor_id, uint8_t comm_type)
{
    uint32_t ext_id;

    ext_id = ((uint32_t)motor_id & ROBSTRIDE_MOTOR_ID_MASK)
           | (((uint32_t)ROBSTRIDE_MASTER_CAN_ID & ROBSTRIDE_CAN_ID_MASK) << ROBSTRIDE_CAN_ID_SHIFT)
           | (((uint32_t)comm_type & ROBSTRIDE_COMM_TYPE_MASK) << ROBSTRIDE_COMM_TYPE_SHIFT);

    return ext_id & ROBSTRIDE_EXT_ID_MASK;
}

/* ========================================================================
   打包函数
   ======================================================================== */

/**
 * @brief 组装参数写入帧 data[8] 的公共部分。
 *
 * 参数写入帧（mode = 0x12）格式（说明书 4.4.5 节）：
 *   Byte0~1: 参数索引 (uint16, 小端，直接 memcpy)
 *   Byte2~3: 保留 (0x0000)
 *   Byte4~7: 参数值 (float, IEEE 754, 小端)
 */
static void robstride_pack_param_frame(uint16_t param_index, const void *value, uint8_t value_size,
                                       uint8_t data[8])
{
    (void)memset(data, 0, 8U);
    (void)memcpy(&data[0], &param_index, sizeof(param_index));
    if ((value != NULL) && (value_size > 0U) && (value_size <= 4U)) {
        (void)memcpy(&data[4], value, value_size);
    }
}

uint32_t robstride_motor_pack_enable(uint8_t motor_id, uint8_t data[8])
{
    uint32_t ext_id;

    ext_id = robstride_motor_build_ext_id(motor_id, ROBSTRIDE_COMM_ENABLE);
    (void)memset(data, 0, 8U);
    /* 使能帧数据域全零 */
    return ext_id;
}

uint32_t robstride_motor_pack_stop(uint8_t motor_id, uint8_t clear_error, uint8_t data[8])
{
    uint32_t ext_id;

    ext_id = robstride_motor_build_ext_id(motor_id, ROBSTRIDE_COMM_STOP);
    (void)memset(data, 0, 8U);
    data[0] = clear_error;
    return ext_id;
}

uint32_t robstride_motor_pack_run_mode_pp(uint8_t motor_id, uint8_t data[8])
{
    uint32_t ext_id;
    uint8_t run_mode = ROBSTRIDE_RUN_MODE_PP;

    ext_id = robstride_motor_build_ext_id(motor_id, ROBSTRIDE_COMM_SET_SINGLE_PARAM);
    (void)memset(data, 0, 8U);
    data[0] = (uint8_t)(ROBSTRIDE_PARAM_RUN_MODE & 0xFFU);
    data[1] = (uint8_t)((ROBSTRIDE_PARAM_RUN_MODE >> 8) & 0xFFU);
    data[4] = run_mode;
    return ext_id;
}

uint32_t robstride_motor_pack_run_mode(uint8_t motor_id, uint8_t run_mode, uint8_t data[8])
{
    uint32_t ext_id;

    ext_id = robstride_motor_build_ext_id(motor_id, ROBSTRIDE_COMM_SET_SINGLE_PARAM);
    (void)memset(data, 0, 8U);
    data[0] = (uint8_t)(ROBSTRIDE_PARAM_RUN_MODE & 0xFFU);
    data[1] = (uint8_t)((ROBSTRIDE_PARAM_RUN_MODE >> 8) & 0xFFU);
    data[4] = run_mode;
    return ext_id;
}

uint32_t robstride_motor_pack_vel_max(uint8_t motor_id, float vel_max_rad_s, uint8_t data[8])
{
    uint32_t ext_id;

    ext_id = robstride_motor_build_ext_id(motor_id, ROBSTRIDE_COMM_SET_SINGLE_PARAM);
    robstride_pack_param_frame(ROBSTRIDE_PARAM_VEL_MAX, &vel_max_rad_s, sizeof(vel_max_rad_s), data);
    return ext_id;
}

uint32_t robstride_motor_pack_acc_set(uint8_t motor_id, float acc_rad_s2, uint8_t data[8])
{
    uint32_t ext_id;

    ext_id = robstride_motor_build_ext_id(motor_id, ROBSTRIDE_COMM_SET_SINGLE_PARAM);
    robstride_pack_param_frame(ROBSTRIDE_PARAM_ACC_SET, &acc_rad_s2, sizeof(acc_rad_s2), data);
    return ext_id;
}

uint32_t robstride_motor_pack_epscan_time(uint8_t motor_id, uint16_t epscan_time, uint8_t data[8])
{
    uint32_t ext_id;

    ext_id = robstride_motor_build_ext_id(motor_id, ROBSTRIDE_COMM_SET_SINGLE_PARAM);
    robstride_pack_param_frame(ROBSTRIDE_PARAM_EPSCAN_TIME, &epscan_time, sizeof(epscan_time), data);
    return ext_id;
}

uint32_t robstride_motor_pack_zero_sta(uint8_t motor_id, uint8_t mode, uint8_t data[8])
{
    uint32_t ext_id;

    ext_id = robstride_motor_build_ext_id(motor_id, ROBSTRIDE_COMM_SET_SINGLE_PARAM);
    (void)memset(data, 0, 8U);
    data[0] = (uint8_t)(ROBSTRIDE_PARAM_ZERO_STA & 0xFFU);
    data[1] = (uint8_t)((ROBSTRIDE_PARAM_ZERO_STA >> 8) & 0xFFU);
    data[4] = mode;
    return ext_id;
}

uint32_t robstride_motor_pack_position_ref(uint8_t motor_id, float position_rad, uint8_t data[8])
{
    uint32_t ext_id;

    ext_id = robstride_motor_build_ext_id(motor_id, ROBSTRIDE_COMM_SET_SINGLE_PARAM);
    robstride_pack_param_frame(ROBSTRIDE_PARAM_LOC_REF, &position_rad, sizeof(position_rad), data);
    return ext_id;
}

uint32_t robstride_motor_pack_enable_report(uint8_t motor_id, uint8_t data[8])
{
    uint32_t ext_id;

    /*
     * 0x18 上报使能帧 data 格式（对照官方例程 RobStride_Motor_MotorModeSet）：
     *   data[0] = 0x01
     *   data[1] = 0x02
     *   data[2] = 0x03
     *   data[3] = 0x04
     *   data[4] = 0x05
     *   data[5] = 0x06
     *   data[6] = F_CMD  (0x00 = 关闭, 0x01 = 开启 10ms 周期上报)
     *   data[7] = 0x00  （固定填充，RS00 说明书确认）
     */
    ext_id = robstride_motor_build_ext_id(motor_id, ROBSTRIDE_COMM_SET_REPORT);
    data[0] = 0x01U;
    data[1] = 0x02U;
    data[2] = 0x03U;
    data[3] = 0x04U;
    data[4] = 0x05U;
    data[5] = 0x06U;
    data[6] = ROBSTRIDE_REPORT_10MS;
    data[7] = 0x00U;
    return ext_id;
}

uint32_t robstride_motor_pack_set_can_id(uint8_t old_id, uint8_t new_id, uint8_t data[8])
{
    uint32_t ext_id;

    /*
     * 0x07 改 CAN ID 帧（说明书 4.1 节 type 7）：
     *   bits[28:24] = 0x07
     *   bits[23:16] = new_id
     *   bits[15:8]  = master (0xFD)
     *   bits[7:0]   = old_id（当前 CAN ID，用于寻址）
     * Data[8]: 全 0
     */
    ext_id = (((uint32_t)0x07U & ROBSTRIDE_COMM_TYPE_MASK) << ROBSTRIDE_COMM_TYPE_SHIFT)
           | (((uint32_t)new_id & 0xFFU) << 16U)
           | (((uint32_t)ROBSTRIDE_MASTER_CAN_ID & ROBSTRIDE_CAN_ID_MASK) << ROBSTRIDE_CAN_ID_SHIFT)
           | ((uint32_t)old_id & ROBSTRIDE_MOTOR_ID_MASK);

    (void)memset(data, 0, 8U);
    return ext_id;
}

uint32_t robstride_motor_pack_save_params(uint8_t motor_id, uint8_t data[8])
{
    uint32_t ext_id;

    /*
     * 0x16 保存参数帧（说明书 4.1 节 type 22）：
     *   Data: {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}
     *   保存后 CAN ID 掉电不丢失。
     */
    ext_id = robstride_motor_build_ext_id(motor_id, ROBSTRIDE_COMM_SAVE_PARAMS);
    data[0] = 0x01U;
    data[1] = 0x02U;
    data[2] = 0x03U;
    data[3] = 0x04U;
    data[4] = 0x05U;
    data[5] = 0x06U;
    data[6] = 0x07U;
    data[7] = 0x08U;
    return ext_id;
}

uint32_t robstride_motor_pack_cur_kp(uint8_t motor_id, float value, uint8_t data[8])
{
    uint32_t ext_id = robstride_motor_build_ext_id(motor_id, ROBSTRIDE_COMM_SET_SINGLE_PARAM);
    robstride_pack_param_frame(ROBSTRIDE_PARAM_CUR_KP, &value, sizeof(value), data);
    return ext_id;
}

uint32_t robstride_motor_pack_cur_ki(uint8_t motor_id, float value, uint8_t data[8])
{
    uint32_t ext_id = robstride_motor_build_ext_id(motor_id, ROBSTRIDE_COMM_SET_SINGLE_PARAM);
    robstride_pack_param_frame(ROBSTRIDE_PARAM_CUR_KI, &value, sizeof(value), data);
    return ext_id;
}

uint32_t robstride_motor_pack_spd_kp(uint8_t motor_id, float value, uint8_t data[8])
{
    uint32_t ext_id = robstride_motor_build_ext_id(motor_id, ROBSTRIDE_COMM_SET_SINGLE_PARAM);
    robstride_pack_param_frame(ROBSTRIDE_PARAM_SPD_KP, &value, sizeof(value), data);
    return ext_id;
}

uint32_t robstride_motor_pack_spd_ki(uint8_t motor_id, float value, uint8_t data[8])
{
    uint32_t ext_id = robstride_motor_build_ext_id(motor_id, ROBSTRIDE_COMM_SET_SINGLE_PARAM);
    robstride_pack_param_frame(ROBSTRIDE_PARAM_SPD_KI, &value, sizeof(value), data);
    return ext_id;
}

uint32_t robstride_motor_pack_loc_kp(uint8_t motor_id, float value, uint8_t data[8])
{
    uint32_t ext_id = robstride_motor_build_ext_id(motor_id, ROBSTRIDE_COMM_SET_SINGLE_PARAM);
    robstride_pack_param_frame(ROBSTRIDE_PARAM_LOC_KP, &value, sizeof(value), data);
    return ext_id;
}

uint32_t robstride_motor_pack_read_param(uint8_t motor_id, uint16_t param_index, uint8_t data[8])
{
    uint32_t ext_id;

    /*
     * 0x11 读单个参数帧：
     *   data[0:1] = param_index (uint16 小端)
     *   data[2:7] = 0
     */
    ext_id = robstride_motor_build_ext_id(motor_id, ROBSTRIDE_COMM_READ_SINGLE_PARAM);
    (void)memset(data, 0, 8U);
    data[0] = (uint8_t)(param_index & 0xFFU);
    data[1] = (uint8_t)((param_index >> 8) & 0xFFU);
    return ext_id;
}

/* ========================================================================
   解包函数
   ======================================================================== */

/**
 * @brief 解析一帧 ROBSTRIDE 反馈并写入缓存。
 *
 * 反馈帧 ID 布局（Motor→Host，RS00 说明书 4.1 节）：
 *   bits [28:24] = 通信类型 (0x02 / 0x18)
 *   bits [23:22] = 模式状态 (0=Reset, 1=Cali, 2=Motor)
 *   bits [21:16] = 故障码 (6 bits)
 *   bits [15:8]  = 源电机 CAN ID
 *   bits [7:0]   = 主机 CAN ID (0xFD)
 *
 * 反馈帧 8 字节数据布局（大端 uint16 编码）：
 *   Byte0~1: 位置 [0~65535] → [-12.57, 12.57] rad
 *   Byte2~3: 速度 [0~65535] → [-33.0,  33.0]  rad/s
 *   Byte4~5: 力矩 [0~65535] → [-14.0,  14.0]  Nm
 *   Byte6~7: 温度 uint16 × 0.1 = °C
 */
uint8_t robstride_motor_parse_rx_frame(uint32_t extended_id, const uint8_t data[8])
{
    uint8_t  comm_type;
    uint8_t  motor_id;
    uint8_t  cache_index;
    uint16_t raw_position;
    uint16_t raw_speed;
    uint16_t raw_torque;
    robstride_motor_feedback_t *fb;

    if (data == NULL) {
        return 0U;
    }

    /* 提取通信类型 bits [28:24] */
    comm_type = (uint8_t)((extended_id >> ROBSTRIDE_COMM_TYPE_SHIFT) & ROBSTRIDE_COMM_TYPE_MASK);

    switch (comm_type) {
    case ROBSTRIDE_COMM_FEEDBACK:       /* 0x02 周期反馈 */
    case ROBSTRIDE_COMM_SET_REPORT:     /* 0x18 上报帧反馈 */
        break;
    default:
        /* 未知通信类型，不处理 */
        return 0U;
    }

    /* 提取电机 CAN ID bits [15:8] */
    motor_id = (uint8_t)((extended_id >> ROBSTRIDE_CAN_ID_SHIFT) & ROBSTRIDE_CAN_ID_MASK);

    /* 映射到缓存索引 */
    cache_index = robstride_motor_id_to_index(motor_id);
    if (cache_index == 0xFFU) {
        return 0U;
    }

    fb = &s_feedback_cache[cache_index];

    /*
     * 从扩展 ID 提取故障码和模式状态（不在 data 中）：
     *   bits [21:16] = 故障码 (6 bits)
     *     Bit16: 欠压   Bit17: 三相电流  Bit18: 过温
     *     Bit19: 磁编码 Bit20: 堵转过载  Bit21: 未标定
     *   bits [23:22] = 模式状态
     *     0 = Reset, 1 = Cali, 2 = Motor 运行
     */
    fb->fault_code = (uint8_t)((extended_id >> 16) & 0x3FU);
    fb->mode_state = (uint8_t)((extended_id >> 22) & 0x03U);

    /* ---- 解析 8 字节反馈数据 ---- */

    /* Byte0~1: 位置 (uint16 大端 → rad) */
    raw_position = robstride_u16_be(&data[0]);
    fb->position_raw_rad = robstride_uint_to_float(raw_position,
                                               ROBSTRIDE_P_MIN_RAD,
                                               ROBSTRIDE_P_MAX_RAD);

    /* 归一化到欧拉角 [-π, +π) */
    fb->position_raw_rad = robstride_normalize_euler(fb->position_raw_rad);
		fb->position_raw_deg = fb->position_raw_rad * (180.0f / 3.141592653589793f);

    /* Byte2~3: 速度 (uint16 大端 → rad/s) */
    raw_speed = robstride_u16_be(&data[2]);
    fb->speed_rad_s = robstride_uint_to_float(raw_speed,
                                              ROBSTRIDE_V_MIN_RAD_S,
                                              ROBSTRIDE_V_MAX_RAD_S);

    /* Byte4~5: 力矩 (uint16 大端 → Nm)，V1 暂不解码暴露到 App */
    raw_torque = robstride_u16_be(&data[4]);
    fb->torque_nm = robstride_uint_to_float(raw_torque,
                                            ROBSTRIDE_T_MIN_NM,
                                            ROBSTRIDE_T_MAX_NM);

    /*
     * Byte6~7: 温度 uint16 × 0.1 = °C（对照官方例程：
     *   Pos_Info.Temp = (DataFrame[6]<<8|DataFrame[7]) * 0.1;）
     * 故障码在扩展 ID bit[21:16]，不在 data 中。
     */
    fb->temperature_c = (int16_t)((float)(robstride_u16_be(&data[6])) * 0.1f);

    /* 更新元信息 */
    fb->motor_id = motor_id;
    /* last_update_ms 不在此设置，由调用方（BSP ISR）写入 HAL_GetTick() */

    return 1U;
}

/* ========================================================================
   反馈访问器
   ======================================================================== */

const robstride_motor_feedback_t *robstride_motor_get_feedback(uint8_t motor_id)
{
    uint8_t cache_index;

    cache_index = robstride_motor_id_to_index(motor_id);
    if (cache_index == 0xFFU) {
        return NULL;
    }

    return &s_feedback_cache[cache_index];
}

void robstride_motor_set_last_update_ms(uint8_t motor_id, uint32_t tick_ms)
{
    uint8_t cache_index;

    cache_index = robstride_motor_id_to_index(motor_id);
    if (cache_index == 0xFFU) {
        return;
    }

    s_feedback_cache[cache_index].last_update_ms = tick_ms;
}
