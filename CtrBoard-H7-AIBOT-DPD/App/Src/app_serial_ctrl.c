#include "app_serial_ctrl.h"
#include "bsp_rc.h"
#include "app_steer_chassis.h"
#include "app_wheel_task.h"
#include "usart.h"
#include "stm32h7xx_hal.h"

#define SERIAL_CMD_TIMEOUT_MS  500U

static serial_command_t s_last_cmd;
static uint8_t          s_has_cmd       = 0U;
static uint8_t          s_cmd_pending   = 0U;
static uint32_t         s_last_cmd_tick = 0U;
static uint8_t          s_tx_busy       = 0U;
static uint8_t          s_tx_buf[SERIAL_FRAME_TOTAL_SIZE_RSP];

extern RC_Channels_t g_rc;

static void on_serial_frame(const serial_command_t *cmd)
{
    s_last_cmd      = *cmd;
    s_cmd_pending   = 1U;
    s_has_cmd       = 1U;
    s_last_cmd_tick = HAL_GetTick();
}

void app_serial_ctrl_init(void)
{
    bsp_serial_protocol_init();
    bsp_serial_protocol_subscribe(on_serial_frame);
    s_has_cmd     = 0U;
    s_cmd_pending = 0U;
}

void app_serial_ctrl_poll(void)
{
    bsp_serial_protocol_poll();

    if (s_cmd_pending) {
        if (!s_tx_busy) {
            s_tx_busy = 1U;
            s_cmd_pending = 0U;

            serial_response_t rsp;
            const steer_chassis_t *chassis = app_steer_chassis_get_state();
            int16_t wheel_speed_rpm[APP_WHEEL_COUNT] = {0};
            app_wheel_task_get_target_speed_rpm(wheel_speed_rpm);

            rsp.hub_speed_fl = (float)wheel_speed_rpm[APP_STEER_MODULE_FL]
                             * 0.104719755f;
            rsp.hub_speed_rl = (float)wheel_speed_rpm[APP_STEER_MODULE_RL]
                             * 0.104719755f;
            rsp.hub_speed_rr = (float)wheel_speed_rpm[APP_STEER_MODULE_RR]
                             * 0.104719755f;
            rsp.hub_speed_fr = (float)wheel_speed_rpm[APP_STEER_MODULE_FR]
                             * 0.104719755f;

            rsp.steer_angle_fl = chassis->modules[APP_STEER_MODULE_FL].feedback.position_raw_deg;
            rsp.steer_angle_rl = chassis->modules[APP_STEER_MODULE_RL].feedback.position_raw_deg;
            rsp.steer_angle_rr = chassis->modules[APP_STEER_MODULE_RR].feedback.position_raw_deg;
            rsp.steer_angle_fr = chassis->modules[APP_STEER_MODULE_FR].feedback.position_raw_deg;

            uint8_t len = bsp_serial_protocol_build_response(&rsp, s_tx_buf);
            HAL_UART_Transmit_DMA(&huart7, s_tx_buf, len);
        }
    }

    if (s_has_cmd && (HAL_GetTick() - s_last_cmd_tick) > SERIAL_CMD_TIMEOUT_MS) {
        s_has_cmd = 0U;
        s_last_cmd.vx_mps   = 0.0f;
        s_last_cmd.vy_mps   = 0.0f;
        s_last_cmd.wz_rad_s = 0.0f;
    }
}

uint8_t app_serial_ctrl_is_active(void)
{
    int16_t swd     = g_rc.sw_st[eRC_SW_D].curr;
    uint8_t rc_lost = g_rc.lost_flag;

    return (swd == eRC_POS_DOWN || rc_lost) ? 1U : 0U;
}

uint8_t app_serial_ctrl_get_command(serial_command_t *cmd)
{
    if (s_has_cmd && cmd != NULL) {
        *cmd = s_last_cmd;
        return 1U;
    }
    return 0U;
}

void app_serial_ctrl_tx_complete(void)
{
    s_tx_busy = 0U;
}
