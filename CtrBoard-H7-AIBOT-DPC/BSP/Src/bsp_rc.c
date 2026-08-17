/**
  ******************************************************************************
  * @file    rc_ctrl.c
  * @brief   遥控器 SBUS 协议解析实现 (寄存器级, 不依赖 HAL IDLE 回调)
  ******************************************************************************
  */

#include "bsp_rc.h"
#include "low_pass_filter.h"
#include "app_config.h"
#include "usart.h"
#include "stm32h7xx_hal.h"

/* Exported variables --------------------------------------------------------*/

volatile uint8_t g_rc_dma_buffer[BSP_SBUS_RX_BUF_SIZE];                /*DMA接收缓冲区*/
int16_t           g_rc_raw[BSP_SBUS_CHANNEL_COUNT];          /* 原始值 (调试)*/

RC_Channels_t     g_rc = {0};                                  /* 命名通道   */

/** @brief 滤波后的遥控模拟通道，作为一阶低通滤波器的输出状态 */
RC_Filter_t       g_rc_filter = {0};                            /* 滤波底盘指令   */
RC_ChassisCmd_t   g_rc_chassis = {0};                           /* 底盘指令   */

/* Private functions ---------------------------------------------------------*/

static uint8_t bsp_sbus_parse_frame(const uint8_t *data_buffer, uint16_t *channels)
{
    if (data_buffer[0] != 0x0F || data_buffer[24] != 0x00) {
        return 0u;
    }

    channels[0]  = ((uint16_t)data_buffer[1]       | (uint16_t)(data_buffer[2]  << 8)) & 0x07FFu;
    channels[1]  = ((uint16_t)(data_buffer[2] >> 3) | (uint16_t)(data_buffer[3]  << 5)) & 0x07FFu;
    channels[2]  = ((uint16_t)(data_buffer[3] >> 6) | (uint16_t)(data_buffer[4]  << 2)
             | (uint16_t)(data_buffer[5] << 10)) & 0x07FFu;
    channels[3]  = ((uint16_t)(data_buffer[5] >> 1) | (uint16_t)(data_buffer[6]  << 7)) & 0x07FFu;
    channels[4]  = ((uint16_t)(data_buffer[6] >> 4) | (uint16_t)(data_buffer[7]  << 4)) & 0x07FFu;
    channels[5]  = ((uint16_t)(data_buffer[7] >> 7) | (uint16_t)(data_buffer[8]  << 1)
             | (uint16_t)(data_buffer[9] << 9)) & 0x07FFu;
    channels[6]  = ((uint16_t)(data_buffer[9] >> 2) | (uint16_t)(data_buffer[10] << 6)) & 0x07FFu;
    channels[7]  = ((uint16_t)(data_buffer[10] >> 5) | (uint16_t)(data_buffer[11] << 3)) & 0x07FFu;

    channels[8]  = ((uint16_t)data_buffer[12]      | (uint16_t)(data_buffer[13] << 8)) & 0x07FFu;
    channels[9]  = ((uint16_t)(data_buffer[13] >> 3) | (uint16_t)(data_buffer[14] << 5)) & 0x07FFu;
    channels[10] = ((uint16_t)(data_buffer[14] >> 6) | (uint16_t)(data_buffer[15] << 2)
             | (uint16_t)(data_buffer[16] << 10)) & 0x07FFu;
    channels[11] = ((uint16_t)(data_buffer[16] >> 1) | (uint16_t)(data_buffer[17] << 7)) & 0x07FFu;
    channels[12] = ((uint16_t)(data_buffer[17] >> 4) | (uint16_t)(data_buffer[18] << 4)) & 0x07FFu;
    channels[13] = ((uint16_t)(data_buffer[18] >> 7) | (uint16_t)(data_buffer[19] << 1)
             | (uint16_t)(data_buffer[20] << 9)) & 0x07FFu;
    channels[14] = ((uint16_t)(data_buffer[20] >> 2) | (uint16_t)(data_buffer[21] << 6)) & 0x07FFu;
    channels[15] = ((uint16_t)(data_buffer[21] >> 5) | (uint16_t)(data_buffer[22] << 3)) & 0x07FFu;

    return 1u;
}


/**
  * @brief  根据原始遥控采样值更新滤波输出
  * @param  rc_channels     原始遥控采样通道
  * @param  filter 滤波输出通道，同时保存上一次输出状态
  *
  * @details 仅对连续模拟通道做一阶低通滤波。
  *          filter 各字段在调用前为 y[k-1]，调用后更新为 y[k]。
  */
static void bsp_rc_filter_update(const RC_Channels_t *rc_channels, RC_Filter_t *filter)
{
    filter->ch_ry = low_pass_filter_update_float((float)rc_channels->ch_ry, filter->ch_ry, BSP_RC_LPF_ALPHA);
    filter->ch_rx = low_pass_filter_update_float((float)rc_channels->ch_rx, filter->ch_rx, BSP_RC_LPF_ALPHA);
    filter->ch_lx = low_pass_filter_update_float((float)rc_channels->ch_lx, filter->ch_lx, BSP_RC_LPF_ALPHA);
    filter->ch_ly = low_pass_filter_update_float((float)rc_channels->ch_ly, filter->ch_ly, BSP_RC_LPF_ALPHA);

}

/* Exported functions --------------------------------------------------------*/

void bsp_rc_on_frame_received(uint16_t frame_len)
{
    if (frame_len >= BSP_SBUS_FRAME_SIZE) {
        uint16_t raw_channels[BSP_SBUS_CHANNEL_COUNT];
        if (bsp_sbus_parse_frame((const uint8_t *)g_rc_dma_buffer, raw_channels)) {
            uint8_t channel_idx;

            /* 1. 记录最后一帧时刻，用于main.c进行check-lost */
            g_rc.last_tick = HAL_GetTick();

            /* 2. 保存原始值 */
            for (channel_idx = 0u; channel_idx < BSP_SBUS_CHANNEL_COUNT; channel_idx++) {
                g_rc_raw[channel_idx] = (int16_t)raw_channels[channel_idx];
            }

            /* 3. 每帧更新通道, delta = 10ms (防 DMA HT+IDLE 双触发覆盖) */
            {
                g_rc.sample_now_tick = HAL_GetTick();
                
                if (g_rc.sample_now_tick != g_rc.last_sample_tick) 
                {
                    g_rc.sample_delta_tick = g_rc.sample_now_tick - g_rc.last_sample_tick;
                    g_rc.last_sample_tick = g_rc.sample_now_tick;
                }

                {
                    /* 逐字段赋值 + 中位归零 + 摇杆通道极性统一 */
                    g_rc.ch_ry =
                        (int16_t)(((int16_t)raw_channels[0] - (int16_t)BSP_SBUS_MID_VALUE)
                                  * BSP_RC_CH_RY_POLARITY);
                    g_rc.ch_rx =
                        (int16_t)(((int16_t)raw_channels[1] - (int16_t)BSP_SBUS_MID_VALUE)
                                  * BSP_RC_CH_RX_POLARITY);
                    g_rc.ch_lx =
                        (int16_t)(((int16_t)raw_channels[2] - (int16_t)BSP_SBUS_MID_VALUE)
                                  * BSP_RC_CH_LX_POLARITY);
                    g_rc.ch_ly =
                        (int16_t)(((int16_t)raw_channels[3] - (int16_t)BSP_SBUS_MID_VALUE)
                                  * BSP_RC_CH_LY_POLARITY);

                    g_rc.vra   =  (int16_t)(raw_channels[8] - BSP_SBUS_VAR_VALUE);
                    g_rc.vrb   =  (int16_t)(raw_channels[9] - BSP_SBUS_VAR_VALUE);

                    /* 4. 拨杆模拟值赋值 */
                    g_rc.sw_val[0] = (int16_t)(raw_channels[4] - BSP_SBUS_MID_VALUE) * BSP_RC_SW_POLARITY;
                    g_rc.sw_val[1] = (int16_t)(raw_channels[5] - BSP_SBUS_MID_VALUE) * BSP_RC_SW_POLARITY;
                    g_rc.sw_val[2] = (int16_t)(raw_channels[6] - BSP_SBUS_MID_VALUE) * BSP_RC_SW_POLARITY;
                    g_rc.sw_val[3] = (int16_t)(raw_channels[7] - BSP_SBUS_MID_VALUE) * BSP_RC_SW_POLARITY;

                    /* 5. 拨杆状态追踪 (prev→curr, 可判沿) */
                    {
                        uint8_t switch_idx;
                        for (switch_idx = 0u; switch_idx < 4u; switch_idx++) {
                            //g_rc.sw_st[switch_idx].prev = g_rc.sw_st[switch_idx].curr;
                            if (g_rc.sw_val[switch_idx] >= 400) {
                                g_rc.sw_st[switch_idx].curr = eRC_POS_UP;
                            } else if (g_rc.sw_val[switch_idx] <= -400) {
                                g_rc.sw_st[switch_idx].curr = eRC_POS_DOWN;
                            } else {
                                g_rc.sw_st[switch_idx].curr = eRC_POS_MID;
                            }
                        }
                    }

                    bsp_rc_filter_update(&g_rc, &g_rc_filter); //一阶低通滤波 3ms 收敛
                }
            }

            if (g_rc.frame_count > 0xFFFFFF00u) { g_rc.frame_count = 0u; }
            g_rc.frame_count++;
        } else {
            if (g_rc.error_count > 0xFFFFFF00u) { g_rc.error_count = 0u; }
            g_rc.error_count++;
        }
    }
    /* 中止当前 DMA，再从 buf[0] 重启，确保每次帧对齐在 buf 开头 */
    (void)HAL_UART_AbortReceive(&huart5);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart5, (uint8_t *)g_rc_dma_buffer, BSP_SBUS_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
}

void bsp_rc_init(RC_Channels_t *channels)
{
    /* SBUS 信号反相 */
    SET_BIT(huart5.Instance->CR2, (1UL << 19U));

    /* 启动 DMA+IDLE 接收 */
    __HAL_UART_CLEAR_IDLEFLAG(&huart5);
    __HAL_UART_CLEAR_OREFLAG(&huart5);
    __HAL_UART_CLEAR_FEFLAG(&huart5);
    __HAL_UART_CLEAR_NEFLAG(&huart5);


    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, (uint8_t *)g_rc_dma_buffer, BSP_SBUS_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);

    channels->lost_flag   = 1u;  /* 默认离线 */
    channels->sample_now_tick = 0u;
    channels->last_sample_tick = 0u;
    channels->sample_delta_tick = 0u;
    channels->frame_count = 0u;
    channels->error_count = 0u;
}

