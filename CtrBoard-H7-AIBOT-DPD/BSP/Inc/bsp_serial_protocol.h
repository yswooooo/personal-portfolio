#ifndef BSP_SERIAL_PROTOCOL_H
#define BSP_SERIAL_PROTOCOL_H

#include <stdint.h>

#define SERIAL_FRAME_HEADER_H       0x55U
#define SERIAL_FRAME_HEADER_L       0xAAU

#define SERIAL_FRAME_LENGTH_CMD     0x1EU
#define SERIAL_FRAME_TYPE_CMD       0x01U
#define SERIAL_FRAME_DATA_SIZE_CMD  24U
#define SERIAL_FRAME_TOTAL_SIZE_CMD 30U
#define SERIAL_FRAME_CRC_LEN_CMD    26U

#define SERIAL_FRAME_LENGTH_RSP     0x26U
#define SERIAL_FRAME_TYPE_RSP       0x81U
#define SERIAL_FRAME_DATA_SIZE_RSP  32U
#define SERIAL_FRAME_TOTAL_SIZE_RSP 38U
#define SERIAL_FRAME_CRC_LEN_RSP    34U

#define SERIAL_FRAME_TOTAL_SIZE     SERIAL_FRAME_TOTAL_SIZE_RSP

typedef struct {
    float vx_mps;
    float vy_mps;
    float reserved1;
    float reserved2;
    float reserved3;
    float wz_rad_s;
} serial_command_t;

typedef struct {
    float hub_speed_fl;
    float hub_speed_rl;
    float hub_speed_rr;
    float hub_speed_fr;
    float steer_angle_fl;
    float steer_angle_rl;
    float steer_angle_rr;
    float steer_angle_fr;
} serial_response_t;

typedef void (*serial_frame_callback_t)(const serial_command_t *cmd);

void     bsp_serial_protocol_init(void);
void     bsp_serial_protocol_feed(const uint8_t *data, uint16_t len);
uint8_t  bsp_serial_protocol_poll(void);
void     bsp_serial_protocol_subscribe(serial_frame_callback_t cb);
uint8_t  bsp_serial_protocol_build_response(const serial_response_t *rsp, uint8_t *buf);
uint16_t bsp_serial_protocol_crc16(const uint8_t *data, uint16_t len);

#endif
