#ifndef APP_SERIAL_CTRL_H
#define APP_SERIAL_CTRL_H

#include <stdint.h>
#include "bsp_serial_protocol.h"

void     app_serial_ctrl_init(void);
void     app_serial_ctrl_poll(void);
uint8_t  app_serial_ctrl_is_active(void);
uint8_t  app_serial_ctrl_get_command(serial_command_t *cmd);
void     app_serial_ctrl_tx_complete(void);

#endif
