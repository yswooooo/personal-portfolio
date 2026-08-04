/**
 * @file bsp_can_robostride.h
 * @brief ROBSTRIDE 舵向电机 CAN BSP 接口。
 *
 * BSP 层职责：
 * - 调用 HAL_FDCAN 发送/接收。
 * - 在 CAN RX 中断中分发到 Middleware 层。
 * - 维护 steer_chassis_t 底盘对象（模块绑定、TX 调度游标）。
 *
 * BSP 层不负责：
 * - ROBSTRIDE 协议打包/解包（由 Middleware 层负责）。
 * - 运动学解算（由 App 层负责）。
 */

#ifndef BSP_CAN_ROBOSTRIDE_H
#define BSP_CAN_ROBOSTRIDE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "robstride_motor.h"

/* ========================================================================
   电机 CAN ID 宏（预留，依据实际电机配置修改）
   ======================================================================== */

/** @brief  ROBSTRIDE CAN ID */
#define ROBSTRIDE_STEER_MOTOR_CAN_ID_FL             0x01U
/** @brief  ROBSTRIDE CAN ID */
#define ROBSTRIDE_STEER_MOTOR_CAN_ID_RL             0x02U
/** @brief  ROBSTRIDE CAN ID */
#define ROBSTRIDE_STEER_MOTOR_CAN_ID_RR             0x03U
/** @brief  ROBSTRIDE CAN ID */
#define ROBSTRIDE_STEER_MOTOR_CAN_ID_FR             0x04U

/* ========================================================================
   CAN 发送间隔
   ======================================================================== */

/** @brief ROBSTRIDE CAN 连续两帧之间的最小发送间隔 (ms) */
#define ROBSTRIDE_CAN_TX_INTERVAL_MS                1U
#define ROBSTRIDE_CAN_TX_INTERVAL_MS_4_MOTOR                20U
/* ========================================================================
   枚举与结构体
   ======================================================================== */

/**
 * @brief 四个舵向模块的索引。
 *
 * 顺序：左前 → 左后 → 右后 → 右前（逆时针）。
 * 用于 steer_module_t 内部数组索引，与电机 CAN ID 无关。
 */
typedef enum {
    APP_STEER_MODULE_FL = 0, /**< 左前舵向模块。 */
    APP_STEER_MODULE_RL,     /**< 左后舵向模块。 */
    APP_STEER_MODULE_RR,     /**< 右后舵向模块。 */
    APP_STEER_MODULE_FR,     /**< 右前舵向模块。 */
    APP_STEER_MODULE_COUNT   /**< 舵向模块数量。 */
} steer_module_index_e;

/**
 * @brief 车体坐标系下的底盘速度指令。
 *
 * 坐标约定：x 正前方，y 左侧，wz 逆时针为正。
 */
typedef struct {
    float vx_mps;    /**< 底盘 x 方向速度指令 (m/s)。       */
    float vy_mps;    /**< 底盘 y 方向速度指令 (m/s)。       */
    float wz_rad_s;  /**< 底盘 yaw 角速度指令 (rad/s)。     */
} steer_chassis_command_t;

/**
 * @brief 单个 ROBSTRIDE 舵向模块状态。
 */
typedef struct {
    steer_module_index_e        index;           /**< 舵向模块逻辑索引。                */
    uint8_t                     robstride_motor_id; /**< ROBSTRIDE 电机 CAN ID (1~4)。 */
    float                       robstride_target_angle_rad; /**< 目标角度 (rad)。    */
    float                       robstride_target_angle_deg; /**< 目标角度 (°)。      */
    uint32_t                    robstride_last_tx_tick_ms;  /**< 最近发送 tick (ms)。  */
    robstride_motor_feedback_t  feedback;        /**< 最新反馈数据（由 ISR 写入）       */
    robstride_debug_param_t     debug_param;     /**< 写的 PID/PP 参数（init 写入）  */
    float                       steer_zero_offset_rad;      /**< 软件零偏 (rad)            */
    float                       hub_target_speed_mps;       /**< 轮毂目标速度幅值 (m/s)    */
    uint8_t                     hub_reverse_flag;           /**< 轮毂反转: 0=正转 1=反转  */
} steer_module_t;

/**
 * @brief ROBSTRIDE BSP 使用的四轮转向底盘状态。
 */
typedef struct {
    steer_module_t          modules[APP_STEER_MODULE_COUNT]; /**< 四个舵向模块。       */
    steer_chassis_command_t target_command;                  /**< 最近一次底盘速度目标。 */
    uint32_t                robstride_last_tx_tick_ms;       /**< 最近一次任意模块 CAN 发送 tick。 */
    uint8_t                 robstride_next_tx_index;         /**< 下一次优先发送的模块索引。 */
} steer_chassis_t;

/* ========================================================================
   BSP API
   ======================================================================== */

/**
 * @brief 初始化 FDCAN1 过滤器、全局过滤、启动外设、使能 RX FIFO0 中断。
 *
 * 过滤器不过滤任何 ID（mask=0），所有扩展帧均进入 RX FIFO0。
 * 电机区分完全依靠扩展 ID 中的 motor CAN ID 字段。
 */
void bsp_can_robostride_init(void);

/**
 * @brief 初始化四个舵向模块结构体，绑定电机 CAN ID。
 * @param chassis 待初始化的底盘对象。
 */
void bsp_can_robostride_init_steer_chassis(steer_chassis_t *chassis);

/**
 * @brief 上电初始化流程：对四个电机依次发送 PP 模式→使能→EPScan→上报使能→限幅→PID→零点→保存。
 * @param chassis 已初始化的底盘对象。
 *
 * 初始化阶段允许阻塞等待帧间隔，保证启动配置帧完整发出。
 * 流程（每电机 10 步）：
 *   ① mode=0x12 → 写 run_mode=1 (PP)
 *   ② mode=0x03 → 使能电机
 *   ③ mode=0x12 → 写 EPScan_time=3 (20ms)
 *   ④ mode=0x18 → 使能周期反馈上报
 *   ⑤ mode=0x12 → 写 vel_max
 *   ⑥ mode=0x12 → 写 acc_set
 *   ⑦ mode=0x12 → 写 loc_ref
 *   ⑧ mode=0x12 → 写 spd_kp / loc_kp
 *   ⑨ mode=0x12 → 写 zero_sta=-π~π
 *   ⑩ mode=0x16 → 保存全部参数到 Flash
 */
void bsp_can_robostride_start_steer_chassis(steer_chassis_t *chassis);

/**
 * @brief 向四个舵向电机发送停止命令。
 * @param chassis 已初始化的底盘对象。
 * @param clear_error 非零表示请求清除错误。
 */
/**
 * @brief 一次性改电机 CAN ID 并保存到 Flash。
 *
 * 只能接一台电机时使用。发送 0x07 改 ID + 0x16 保存。
 * 改完后断电，接下一台电机，改参数后重新上电。
 *
 * @param old_id 当前 CAN ID（出厂默认 0x7F）。
 * @param new_id 新 CAN ID（1~4）。一次只改一台。
 */
void bsp_can_robostride_set_motor_id_once(uint8_t old_id, uint8_t new_id);

void bsp_can_robostride_stop_steer_chassis(steer_chassis_t *chassis,
                                           uint8_t clear_error);

/**
 * @brief 按 FL→RL→RR→FR 顺序轮询发送舵向目标角度。
 *
 * 运行期绝不阻塞；每次调用最多实际发送 1 帧。
 * @param chassis 已初始化的底盘对象。
 */
void bsp_can_robostride_send_steer_chassis_position(steer_chassis_t *chassis);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CAN_ROBOSTRIDE_H */
