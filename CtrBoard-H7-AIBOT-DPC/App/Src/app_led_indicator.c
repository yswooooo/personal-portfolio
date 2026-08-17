/**
  ******************************************************************************
  * @file    app_led_indicator.c
  * @brief   系统状态 LED 指示
  *
  * @details 红: 急停生效  黄: 待命  绿: 运行  紫闪: 离线
  *          离线闪烁: 0.5s 内连闪 N 次, 组间隔 1s, 周期 1.5s
  *          非阻塞, 由主循环 app_led_indicator_update() 驱动。
  ******************************************************************************
  */

#include "app_led_indicator.h"
#include "app_chassis_motor_ctrl.h"
#include "bsp_ws4810.h"
#include "bsp_rc.h"
#include "gpio.h"
#include "stm32h7xx_hal.h"

/* 离线紫灯闪烁参数 — 一站修改 */
#define APP_LED_FLASH_WINDOW_MS  500u    /**< 闪灯窗口: 0.5s 内完成 N 次 */
#define APP_LED_FLASH_GAP_MS     1000u   /**< 组间隔: 1s 灭            */
#define APP_LED_FLASH_CYCLE_MS   (APP_LED_FLASH_WINDOW_MS + APP_LED_FLASH_GAP_MS)  /**< 完整周期 */
#define APP_LED_FLASH_DUTY_DIV   3u      /**< 占空比 1/N: 亮=period/N, 灭=(N-1)*period/N */

/** @copydoc app_led_indicator_init */
void app_led_indicator_init(void)
{
    bsp_ws4810_set_rgb(255, 0, 0);  /* 初始红灯 */
}

/** @copydoc app_led_indicator_update */
void app_led_indicator_update(void)
{
    extern volatile uint8_t g_emergency_stop_flag;
    extern RC_Channels_t     g_rc;

    uint8_t offline_count = 0u;
    uint32_t elapsed;
    uint32_t flash_period;
    uint32_t flash_on_ms;

    /* ---- 计算当前离线电机数 ---- */
    if (g_vofa_speed.m1_offline_confirmed > 0.5f) {
        offline_count++;
    }
    if (g_vofa_speed.m2_offline_confirmed > 0.5f) {
        offline_count++;
    }

    /* ---- 优先级1: 紧急停机 → 红灯 ---- */
    if (g_emergency_stop_flag == 1u) {
        bsp_ws4810_set_rgb(255, 0, 0);
        return;
    }

    /* ---- 优先级2: 有电机离线 → 紫灯闪烁 (非阻塞) ---- */
    if (offline_count > 0u) {
        elapsed = HAL_GetTick() % APP_LED_FLASH_CYCLE_MS;

        if (elapsed < APP_LED_FLASH_WINDOW_MS) {
            flash_period = APP_LED_FLASH_WINDOW_MS / (uint32_t)offline_count;
            flash_on_ms  = flash_period / APP_LED_FLASH_DUTY_DIV;
            if ((elapsed % flash_period) < flash_on_ms) {
                bsp_ws4810_set_rgb(255, 0, 255);  /* 紫 */
            } else {
                bsp_ws4810_set_rgb(0, 0, 0);      /* 灭 */
            }
        } else {
            bsp_ws4810_set_rgb(0, 0, 0);          /* 组间隔: 灭 */
        }
        return;
    }

    /* ---- 优先级3: SWA_DOWN → 黄灯待命 ---- */
    if (g_rc.sw_st[eRC_SW_A].curr == eRC_POS_DOWN) {
        bsp_ws4810_set_rgb(255, 255, 0);
        return;
    }

    /* ---- 优先级4: 正常运行 → 绿灯 ---- */
    bsp_ws4810_set_rgb(0, 255, 0);
}
