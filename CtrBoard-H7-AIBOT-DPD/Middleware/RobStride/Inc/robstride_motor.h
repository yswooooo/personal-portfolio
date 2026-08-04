/**
 * @file robstride_motor.h
 * @brief ROBSTRIDE RS00 舵向电机私有协议 Middleware 层。
 *
 * 本层只负责 ROBSTRIDE 协议打包/解包和反馈缓存维护。
 * - 不包含 HAL / BSP / App 头文件。
 * - 不处理底盘 vx/vy/wz 运动学。
 * - 不知道舵轮几何结构。
 */

#ifndef ROBSTRIDE_MOTOR_H
#define ROBSTRIDE_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========================================================================
   扩展 ID 位域定义
   ========================================================================

   对应 RS00 说明书 4.4 节官方 struct exCanIdInfo：

       struct exCanIdInfo {
           uint32_t id:8;      // bits [7:0]   电机 CAN ID
           uint32_t data:16;   // bits [23:8]  主机 CAN ID / 电机 CAN ID + 状态
           uint32_t mode:5;    // bits [28:24] 通信类型
           uint32_t res:3;     // bits [31:29] 保留，必须为 0
       };

   29-bit 扩展 ID 组装：
       ext_id = (id & 0xFF) | ((data & 0xFFFF) << 8) | ((mode & 0x1F) << 24)

   Host → Motor (下发命令)：
       id[7:0]    = 目标电机 CAN ID
       data[15:8] = 主机 CAN ID (0xFD)
       data[23:16]= 0x00
       mode[28:24]= 通信类型

   Motor → Host (反馈上报)：
       id[7:0]    = 主机 CAN ID (0xFD)
       data[15:8] = 源电机 CAN ID
       data[23:16]= 状态位
       mode[28:24]= 0x02 (周期反馈) 或 0x18 (上报帧反馈)
   ======================================================================== */

/** @brief 29-bit 扩展 ID 掩码（res 位清零） */
#define ROBSTRIDE_EXT_ID_MASK               0x1FFFFFFFUL

/** @brief 通信类型 mode 字段：bits [28:24]（5 bits） */
#define ROBSTRIDE_COMM_TYPE_SHIFT           24U
#define ROBSTRIDE_COMM_TYPE_MASK            0x1FU

/** @brief data 字段中主机/电机 CAN ID 所在位：bits [15:8] */
#define ROBSTRIDE_CAN_ID_SHIFT              8U
#define ROBSTRIDE_CAN_ID_MASK               0xFFU

/** @brief 电机 CAN ID 字段：bits [7:0] */
#define ROBSTRIDE_MOTOR_ID_SHIFT            0U
#define ROBSTRIDE_MOTOR_ID_MASK             0xFFU

/* ========================================================================
   协议常量
   ======================================================================== */

/** @brief 主机 CAN ID */
#define ROBSTRIDE_MASTER_CAN_ID             0xFDU

/** @brief 最大电机数量 */
#define ROBSTRIDE_MOTOR_MAX_COUNT           4U

/* ---- 通信类型 (mode) ---- */
#define ROBSTRIDE_COMM_FEEDBACK             0x02U   /**< 周期反馈帧 (Motor→Host)         */
#define ROBSTRIDE_COMM_ENABLE               0x03U   /**< 使能电机 (Host→Motor)          */
#define ROBSTRIDE_COMM_STOP                 0x04U   /**< 停止/复位电机 (Host→Motor)     */
#define ROBSTRIDE_COMM_SET_SINGLE_PARAM     0x12U   /**< 写单个参数 (Host→Motor)        */
#define ROBSTRIDE_COMM_READ_SINGLE_PARAM    0x11U   /**< 读单个参数 (Host→Motor)        */
#define ROBSTRIDE_COMM_SET_REPORT           0x18U   /**< 配置上报 / 上报反馈 (双向)     */
#define ROBSTRIDE_COMM_SAVE_PARAMS          0x16U   /**< 保存参数到 Flash (Host→Motor)  */

/* ---- 参数索引 ---- */
#define ROBSTRIDE_PARAM_RUN_MODE            0x7005U /**< 运行模式: 0=空闲 1=PP 2=速度 3=电流 5=CSP */
#define ROBSTRIDE_PARAM_LOC_REF             0x7016U /**< 位置给定 (float rad)              */
#define ROBSTRIDE_PARAM_LIMIT_SPD           0x7017U /**< CSP 速度限幅 (float rad/s)       */
#define ROBSTRIDE_PARAM_LIMIT_CUR           0x7018U /**< 电流限幅 (float A)               */
#define ROBSTRIDE_PARAM_VEL_MAX             0x7024U /**< PP 最大速度 (float rad/s)        */
#define ROBSTRIDE_PARAM_ACC_SET             0x7025U /**< PP 加速度 (float rad/s²)         */
#define ROBSTRIDE_PARAM_EPSCAN_TIME         0x7026U /**< 主动上报周期: 1=10ms, +1递增5ms */
#define ROBSTRIDE_PARAM_ZERO_STA           0x7029U /**< 零点标志位: 0=0~2π, 1=-π~π   */

/* ---- zero_sta 模式 ---- */
#define ROBSTRIDE_ZERO_STA_0_2PI           0U      /**< 0 ~ 2π              */
#define ROBSTRIDE_ZERO_STA_NEG_PI_PI       1U      /**< -π ~ π             */
#define ROBSTRIDE_DEFAULT_ZERO_STA_MODE    ROBSTRIDE_ZERO_STA_NEG_PI_PI//ROBSTRIDE_ZERO_STA_NEG_PI_PI

/* ---- 调试用参数索引 (0x2xxx 范围，说明书参数表) ---- */
#define ROBSTRIDE_PARAM_CUR_KP              0x2012U /**< 电流 Kp             */
#define ROBSTRIDE_PARAM_CUR_KI              0x2013U /**< 电流 Ki             */
#define ROBSTRIDE_PARAM_SPD_KP              0x2014U /**< 速度 Kp             */
#define ROBSTRIDE_PARAM_SPD_KI              0x2015U /**< 速度 Ki             */
#define ROBSTRIDE_PARAM_LOC_KP              0x2016U /**< 位置 Kp             */
#define ROBSTRIDE_PARAM_PP_VEL_MAX          0x201FU /**< PP 最大速度         */
#define ROBSTRIDE_PARAM_PP_ACC_SET          0x2020U /**< PP 加速度           */

/* ---- 运行模式值 ---- */
#define ROBSTRIDE_RUN_MODE_IDLE             0U
#define ROBSTRIDE_RUN_MODE_PP               1U
#define ROBSTRIDE_RUN_MODE_SPEED            2U
#define ROBSTRIDE_RUN_MODE_CURRENT          3U
#define ROBSTRIDE_RUN_MODE_CSP              5U

/* ---- 编码范围 (RS00 说明书 4.4 节) ---- */
#define ROBSTRIDE_P_MIN_RAD                 (-12.57f)
#define ROBSTRIDE_P_MAX_RAD                 12.57f
#define ROBSTRIDE_V_MIN_RAD_S              (-33.0f)
#define ROBSTRIDE_V_MAX_RAD_S               33.0f
#define ROBSTRIDE_T_MIN_NM                 (-14.0f)
#define ROBSTRIDE_T_MAX_NM                  14.0f

/* ---- 编码位数 ---- */
#define ROBSTRIDE_ENCODE_BITS               16
#define ROBSTRIDE_ENCODE_MAX_VAL            65535.0f

/* ---- 上报配置 F_CMD ---- */
#define ROBSTRIDE_REPORT_DISABLE            0x00U
#define ROBSTRIDE_REPORT_10MS               0x01U

/* ---- 默认 PP 轨迹参数 ---- */
#define ROBSTRIDE_PP_DEFAULT_VEL_MAX_RAD_S      20.0f
#define ROBSTRIDE_PP_DEFAULT_ACC_SET_RAD_S2     100.0f

/* ========================================================================
   调试数据结构
   ======================================================================== */

/**
 * @brief 舵向电机 PID / PP 参数（调试用，从电机读回）。
 */
typedef struct {
    float cur_kp;    /**< 电流 Kp              */
    float cur_ki;    /**< 电流 Ki              */
    float spd_kp;    /**< 速度 Kp              */
    float spd_ki;    /**< 速度 Ki              */
    float loc_kp;    /**< 位置 Kp              */
    float vel_max;   /**< PP 最大速度 (rad/s)   */
    float acc_set;   /**< PP 加速度 (rad/s²)    */
} robstride_debug_param_t;

/* ========================================================================
   反馈数据结构
   ======================================================================== */

/**
 * @brief 单个 ROBSTRIDE 电机反馈缓存。
 *
 * 由 robstride_motor_parse_rx_frame() 写入（CAN 中断上下文），
 * 由 robstride_motor_get_feedback() 读取（主循环上下文）。
 * position_rad 已归一化到欧拉角范围 [-π, +π)。
 */
typedef struct {
    float    position_raw_rad; /**< 电机原始反馈角度 (rad)              */
    float  position_raw_deg; /**< 电机原始反馈角度 (°)                 */

    float    position_new_rad; /**< 标零后车体舵角 (rad)                */
    float   position_new_deg; /**< 标零后车体舵角 (°)                  */

    // float    position_rad;     /**< 当前位置，欧拉角 [-π, +π) rad       */
    // float    position_deg;     /**< 当前位置，角度 [°]  (position_rad 的度数) */
    float    speed_rad_s;      /**< 当前转速 (rad/s)                   */
    float    torque_nm;        /**< 当前力矩 (Nm)                      */
    int16_t  temperature_c;    /**< 温度 (°C)                          */
    uint8_t  motor_id;         /**< 电机 CAN ID                        */
    uint8_t  fault_code;       /**< 故障码 (ID bit[21:16]), 0=正常     */
    uint8_t  mode_state;       /**< 模式状态 (ID bit[23:22]) 0=复位 1=标定 2=运行 */
    uint32_t last_update_ms;   /**< 最近一次更新时刻 (HAL_GetTick)     */
} robstride_motor_feedback_t;

/* ========================================================================
   API 函数
   ======================================================================== */

/**
 * @brief 组装 ROBSTRIDE 扩展 ID。
 * @param motor_id 电机 CAN ID，填入 bits [7:0]。
 * @param comm_type 通信类型，填入 bits [28:24]。
 * @return 29-bit 扩展 ID（res 位已清零）。
 */
uint32_t robstride_motor_build_ext_id(uint8_t motor_id, uint8_t comm_type);

/* ---- 打包函数：返回 extended_id + 填充 data[8]，不调用 HAL ---- */

uint32_t robstride_motor_pack_enable(uint8_t motor_id, uint8_t data[8]);
uint32_t robstride_motor_pack_stop(uint8_t motor_id, uint8_t clear_error, uint8_t data[8]);
uint32_t robstride_motor_pack_run_mode_pp(uint8_t motor_id, uint8_t data[8]);
uint32_t robstride_motor_pack_run_mode(uint8_t motor_id, uint8_t run_mode, uint8_t data[8]);
uint32_t robstride_motor_pack_vel_max(uint8_t motor_id, float vel_max_rad_s, uint8_t data[8]);
uint32_t robstride_motor_pack_acc_set(uint8_t motor_id, float acc_rad_s2, uint8_t data[8]);
uint32_t robstride_motor_pack_position_ref(uint8_t motor_id, float position_rad, uint8_t data[8]);
uint32_t robstride_motor_pack_enable_report(uint8_t motor_id, uint8_t data[8]);
uint32_t robstride_motor_pack_set_can_id(uint8_t old_id, uint8_t new_id, uint8_t data[8]);
uint32_t robstride_motor_pack_save_params(uint8_t motor_id, uint8_t data[8]);

/* ---- PID / PP 参数专用写入 ---- */

uint32_t robstride_motor_pack_cur_kp(uint8_t motor_id, float value, uint8_t data[8]);
uint32_t robstride_motor_pack_cur_ki(uint8_t motor_id, float value, uint8_t data[8]);
uint32_t robstride_motor_pack_spd_kp(uint8_t motor_id, float value, uint8_t data[8]);
uint32_t robstride_motor_pack_spd_ki(uint8_t motor_id, float value, uint8_t data[8]);
uint32_t robstride_motor_pack_loc_kp(uint8_t motor_id, float value, uint8_t data[8]);

/* ---- 上报周期 ---- */

uint32_t robstride_motor_pack_epscan_time(uint8_t motor_id, uint16_t epscan_time, uint8_t data[8]);

/* ---- 零点标志位 ---- */

uint32_t robstride_motor_pack_zero_sta(uint8_t motor_id, uint8_t mode, uint8_t data[8]);

/* ---- 读参数（调试用） ---- */

/**
 * @brief 组装读单个参数帧（mode=0x11）。
 * @param motor_id  电机 CAN ID。
 * @param param_index 参数索引。
 * @param data[8]   输出：填充后的 8 字节数据。
 * @return 29-bit 扩展 ID。
 */
uint32_t robstride_motor_pack_read_param(uint8_t motor_id, uint16_t param_index, uint8_t data[8]);

/* ---- 解包函数：含 switch-case ---- */

/**
 * @brief 解析一帧 ROBSTRIDE 反馈数据。
 *
 * 内部使用 switch-case 判断 communication_type：
 * - 0x02 (ROBSTRIDE_COMM_FEEDBACK)：周期反馈
 * - 0x18 (ROBSTRIDE_COMM_SET_REPORT)：上报帧反馈
 * 其他通信类型直接忽略。
 *
 * @param extended_id 29 位扩展 CAN ID。
 * @param data 8 字节 CAN 数据。
 * @return 1 表示成功解析并更新反馈缓存，0 表示忽略/不处理。
 */
uint8_t robstride_motor_parse_rx_frame(uint32_t extended_id, const uint8_t data[8]);

/* ---- 反馈访问器 ---- */

/**
 * @brief 获取指定电机的反馈缓存只读指针。
 * @param motor_id 电机 CAN ID (1~4)。
 * @return 反馈缓存指针；motor_id 无效时返回 NULL。
 */
const robstride_motor_feedback_t *robstride_motor_get_feedback(uint8_t motor_id);

/**
 * @brief 更新反馈缓存的 last_update_ms。
 *
 * Middleware 不依赖 HAL，无法自己获取 tick。
 * 由 BSP 层在收到反馈帧后调用，传入 HAL_GetTick() 的值。
 *
 * @param motor_id 电机 CAN ID (1~4)。
 * @param tick_ms  当前系统 tick (ms)。
 */
void robstride_motor_set_last_update_ms(uint8_t motor_id, uint32_t tick_ms);

#ifdef __cplusplus
}
#endif

#endif /* ROBSTRIDE_MOTOR_H */
