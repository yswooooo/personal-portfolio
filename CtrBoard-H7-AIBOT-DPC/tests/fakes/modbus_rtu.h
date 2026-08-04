#ifndef TEST_MODBUS_RTU_H
#define TEST_MODBUS_RTU_H

#include <stdint.h>

#define MODBUS_RTU_FC_READ_HOLDING  0x03u
#define MODBUS_RTU_FC_WRITE_SINGLE  0x06u

typedef enum {
    MODBUS_RTU_OK = 0,
    MODBUS_RTU_ERR_PARAM = 1
} modbus_rtu_status_t;

uint16_t modbus_rtu_crc16(const uint8_t *buffer, uint16_t length);

#endif
