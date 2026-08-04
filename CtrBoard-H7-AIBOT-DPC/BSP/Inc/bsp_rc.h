/**
  ******************************************************************************
  * @file    rc_ctrl.h
  * @brief   遥控器 SBUS 协议解析 (BSP 层)
  *
  * @details SBUS 帧 (25B, 100kbps 8E2) → UART5 DMA+IDLE 收帧 → 16 通道解帧
  *          → 归一化(中位=0) → 写入命名结构体 g_rc。
  ******************************************************************************
  */

#ifndef BSP_RC_CTRL_H
#define BSP_RC_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported defines ----------------------------------------------------------*/

#define BSP_SBUS_FRAME_SIZE     25u
#define BSP_SBUS_CHANNEL_COUNT  16u
#define BSP_SBUS_RX_BUF_SIZE    (BSP_SBUS_FRAME_SIZE * 2u)

#define BSP_SBUS_MID_VALUE      992u  /**< SBUS 通道中位值                      */
#define BSP_SBUS_VAR_VALUE      192u /**< SBUS 旋钮通道中位值                      */

#define BSP_RC_STICK_MAX        800   /**< 摇杆/旋钮最大归一化值 (±800)           */
#define BSP_RC_SW_POLARITY      (-1)  /**< 拨杆极性: 正=不变, 负=反转             */

/** @brief 遥控摇杆方形死区阈值，单位为归一化后的通道值 */
#define BSP_RC_STICK_DEADZONE   150.0f

/** @brief 判断线速度/角速度摇杆输入是否位于方形死区内 */
#define BSP_RC_STICK_IN_DEADZONE(x, y) \
    (((x) >= -BSP_RC_STICK_DEADZONE) && ((x) <= BSP_RC_STICK_DEADZONE) && \
     ((y) >= -BSP_RC_STICK_DEADZONE) && ((y) <= BSP_RC_STICK_DEADZONE))

/** @brief 遥控模拟通道一阶低通时间常数 (ms)，值越大输出越平滑但延迟越大 */
#define BSP_RC_LPF_TAU_MS       200.0f

/** @brief 遥控模拟通道一阶低通滤波系数，alpha = Ts / (Ts + tau) */
#define BSP_RC_SAMPLE_TIME_MS      10.0f
#define BSP_RC_LPF_ALPHA        ((float)BSP_RC_SAMPLE_TIME_MS / \
                            ((float)BSP_RC_SAMPLE_TIME_MS + BSP_RC_LPF_TAU_MS))
                            
#define BSP_RC_TIMEOUT_MS        200u  /**< 掉线超时: 200ms 无帧即判定掉线         */


/* Exported types ------------------------------------------------------------*/

/** @brief 拨杆/摇杆归一化值 (中位=0) */
enum {
    eRC_POS_DOWN = -800,  /**< 下档 / 左下     */
    eRC_POS_MID  = 0,     /**< 中档 / 回中     */
    eRC_POS_UP   = 800    /**< 上档 / 右上     */
};

/** @brief 拨杆数组索引 */
enum {
    eRC_SW_A = 0,         /**< SWA 索引       */
    eRC_SW_B = 1,         /**< SWB 索引       */
    eRC_SW_C = 2,         /**< SWC 索引       */
    eRC_SW_D = 3          /**< SWD 索引       */
};

/** @brief 拨杆状态 (当前 + 上一次, 用于边沿检测) */
typedef struct {
    int16_t curr;         /**< 当前帧状态 (eRC_POS_*)                  */
    int16_t prev;         /**< 上一帧状态                             */
} RC_SwitchState_t;

/**
  * @brief  遥控通道结构体 — 命名访问, 中位=0, 范围约 ±800
  */
typedef struct {
    int16_t  ch_ry;     /**< CH1  右摇杆 Y (左右)  右+ 左-                  */
    int16_t  ch_rx;     /**< CH2  右摇杆 X (上下)  上+ 下-                  */
    int16_t  ch_lx;     /**< CH3  左摇杆 X (上下)  上+ 下-                  */
    int16_t  ch_ly;     /**< CH4  左摇杆 Y (左右)  右+ 左-                  */
    int16_t          sw_val[4];   /**< CH5~8 拨杆 SWA/B/C/D 模拟值          */
    RC_SwitchState_t sw_st[4];    /**< SWA/B/C/D 拨杆状态 (prev→curr)       */
    int16_t  vra;    /**< CH9  旋钮 VRA  -800 ~ +800               */
    int16_t  vrb;       /**< CH10 旋钮 VRB -800 ~ +800 连续                 */
    int16_t  _pad[5];   /**< CH12~CH16 保留 (SBUS 固定 16ch)                */
    uint8_t  lost_flag;  /**< 掉线标志: 0=正常 1=超时 (>200ms 无帧)           */
    uint32_t now_tick;   /**< 当前帧时刻 (ms)                                  */
    uint32_t last_tick;  /**< 上一帧时刻 (ms), 用于掉线判断                     */
    uint32_t sample_now_tick;  /**< 当前采样判断时刻 (ms)                    */
    uint32_t last_sample_tick; /**< 上一次有效采样时刻 (ms)                  */
    uint32_t sample_delta_tick; /**< 当前采样间隔 (ms)                       */
    uint32_t frame_count;    /**< 有效帧计数                                    */
    uint32_t error_count;    /**< 无效帧计数                                    */

} RC_Channels_t;

/**
  * @brief  遥控模拟通道滤波输出
  *
  * @details 仅保存摇杆和旋钮等连续模拟量的一阶低通输出。
  *          拨杆、掉线标志、计数器等离散/安全状态仍使用 RC_Channels_t。
  */
typedef struct {
    float ch_ry;   /**< CH1 右摇杆 Y 滤波值 */
    float ch_rx;   /**< CH2 右摇杆 X 滤波值 */
    float ch_lx;   /**< CH3 左摇杆 X 滤波值 */
    float ch_ly;   /**< CH4 左摇杆 Y 滤波值 */

} RC_Filter_t;

/**
  * @brief  底盘控制指令 (由 app_diff_chassis_motor_ctrl_rc_map_to_chassis 填充, 见 App/Src/app_chassis_motor_ctrl.c)
  */
typedef struct {
    float   fLinearVel;     /**< 目标线速度 (m/s)                       */
    float   fAngularVel;    /**< 目标角速度 (rad/s)                      */
    int16_t i16LeftRpm;     /**< 左轮目标转速 (rpm)                      */
    int16_t i16RightRpm;    /**< 右轮目标转速 (rpm)                      */
} RC_ChassisCmd_t;

/* Exported variables --------------------------------------------------------*/

extern volatile uint8_t  g_rc_dma_buffer[BSP_SBUS_RX_BUF_SIZE];
extern RC_Channels_t     g_rc;           /**< 命名通道值, 中位=0              */
extern RC_Filter_t       g_rc_filter;    /**< 滤波后的模拟通道值              */
extern RC_ChassisCmd_t   g_rc_chassis;   /**< 映射后的底盘指令                 */
extern int16_t           g_rc_raw[BSP_SBUS_CHANNEL_COUNT]; /**< 原始值 (调试用)     */
/* Exported functions --------------------------------------------------------*/

void bsp_rc_init(RC_Channels_t *channels);
void bsp_rc_on_frame_received(uint16_t frame_len);

#ifdef __cplusplus
}
#endif

#endif /* BSP_RC_CTRL_H */

