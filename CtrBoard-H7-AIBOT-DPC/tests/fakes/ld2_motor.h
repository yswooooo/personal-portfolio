#ifndef TEST_LD2_MOTOR_H
#define TEST_LD2_MOTOR_H

#include <stdint.h>
#include "bsp_rs485.h"
#include "modbus_rtu.h"

#define LD2_MOTOR_REG_SPEED_TARGET        0x0309u
#define LD2_MOTOR_REG_RUN_STATUS          0x0B05u
#define LD2_MOTOR_REG_ENCODER_POSITION_H  0x0B1Cu

typedef struct {
    bsp_rs485_handle_t *bus;
    uint8_t slave_id;
    uint32_t timeout_ms;
} ld2_motor_handle_t;

extern ld2_motor_handle_t g_ld2rs_dev_m1;
extern ld2_motor_handle_t g_ld2rs_dev_m2;

#endif
