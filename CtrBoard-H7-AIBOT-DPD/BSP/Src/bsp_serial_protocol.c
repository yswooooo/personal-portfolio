#include "bsp_serial_protocol.h"
#include <string.h>

#define MAX_SUBSCRIBERS  4U
#define RX_WORK_BUF_SIZE 256U

static serial_frame_callback_t s_subscribers[MAX_SUBSCRIBERS];
static uint8_t                 s_subscriber_count = 0U;

static uint8_t           s_rx_work_buf[RX_WORK_BUF_SIZE];
static volatile uint16_t s_rx_work_len = 0U;
static volatile uint8_t  s_rx_pending  = 0U;

uint16_t bsp_serial_protocol_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0x0000U;
    uint16_t i;
    for (i = 0U; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8U;
        uint8_t j;
        for (j = 0U; j < 8U; j++) {
            if (crc & 0x8000U) {
                crc = (crc << 1U) ^ 0x1021U;
            } else {
                crc = crc << 1U;
            }
        }
    }
    return crc;
}

void bsp_serial_protocol_init(void)
{
    s_subscriber_count = 0U;
    s_rx_pending  = 0U;
    s_rx_work_len = 0U;
}

void bsp_serial_protocol_subscribe(serial_frame_callback_t cb)
{
    if (cb != NULL && s_subscriber_count < MAX_SUBSCRIBERS) {
        s_subscribers[s_subscriber_count++] = cb;
    }
}

void bsp_serial_protocol_feed(const uint8_t *data, uint16_t len)
{
    if (len == 0U) return;
    if (len > RX_WORK_BUF_SIZE) len = RX_WORK_BUF_SIZE;
    memcpy(s_rx_work_buf, data, len);
    s_rx_work_len = len;
    s_rx_pending  = 1U;
}

static uint8_t parse_frame(const uint8_t *data, uint16_t len, serial_command_t *cmd)
{
    uint16_t crc_calc, crc_recv;

    if (len < SERIAL_FRAME_TOTAL_SIZE_CMD) return 0U;
    if (data[0] != SERIAL_FRAME_HEADER_H || data[1] != SERIAL_FRAME_HEADER_L) return 0U;
    if (data[2] != SERIAL_FRAME_LENGTH_CMD)  return 0U;
    if (data[3] != SERIAL_FRAME_TYPE_CMD) return 0U;

    crc_calc = bsp_serial_protocol_crc16(&data[2], SERIAL_FRAME_CRC_LEN_CMD);
    crc_recv = (uint16_t)data[SERIAL_FRAME_TOTAL_SIZE_CMD - 2U]
             | ((uint16_t)data[SERIAL_FRAME_TOTAL_SIZE_CMD - 1U] << 8U);
    if (crc_calc != crc_recv) return 0U;

    memcpy(&cmd->vx_mps,     &data[4],  4U);
    memcpy(&cmd->vy_mps,     &data[8],  4U);
    memcpy(&cmd->reserved1,  &data[12], 4U);
    memcpy(&cmd->reserved2,  &data[16], 4U);
    memcpy(&cmd->reserved3,  &data[20], 4U);
    memcpy(&cmd->wz_rad_s,   &data[24], 4U);

    return 1U;
}

uint8_t bsp_serial_protocol_poll(void)
{
    serial_command_t cmd;
    uint8_t i;
    uint8_t parsed = 0U;

    if (!s_rx_pending) return 0U;

    if (parse_frame(s_rx_work_buf, s_rx_work_len, &cmd)) {
        for (i = 0U; i < s_subscriber_count; i++) {
            if (s_subscribers[i] != NULL) {
                s_subscribers[i](&cmd);
            }
        }
        parsed = 1U;
    }

    s_rx_pending  = 0U;
    s_rx_work_len = 0U;
    return parsed;
}

uint8_t bsp_serial_protocol_build_response(const serial_response_t *rsp, uint8_t *buf)
{
    uint16_t crc;

    buf[0] = SERIAL_FRAME_HEADER_H;
    buf[1] = SERIAL_FRAME_HEADER_L;
    buf[2] = SERIAL_FRAME_LENGTH_RSP;
    buf[3] = SERIAL_FRAME_TYPE_RSP;

    memcpy(&buf[4],  &rsp->hub_speed_fl,   4U);
    memcpy(&buf[8],  &rsp->hub_speed_rl,   4U);
    memcpy(&buf[12], &rsp->hub_speed_rr,   4U);
    memcpy(&buf[16], &rsp->hub_speed_fr,   4U);
    memcpy(&buf[20], &rsp->steer_angle_fl, 4U);
    memcpy(&buf[24], &rsp->steer_angle_rl, 4U);
    memcpy(&buf[28], &rsp->steer_angle_rr, 4U);
    memcpy(&buf[32], &rsp->steer_angle_fr, 4U);

    crc = bsp_serial_protocol_crc16(&buf[2], SERIAL_FRAME_CRC_LEN_RSP);
    buf[SERIAL_FRAME_TOTAL_SIZE_RSP - 2U] = (uint8_t)(crc & 0xFFU);
    buf[SERIAL_FRAME_TOTAL_SIZE_RSP - 1U] = (uint8_t)((crc >> 8U) & 0xFFU);

    return SERIAL_FRAME_TOTAL_SIZE_RSP;
}
