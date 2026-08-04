#ifndef BSP_VOFA_H
#define BSP_VOFA_H

//....在此替换你的串口函数路径........
#include "usart.h"
//...................................
#include <stdio.h>
#include "stdint.h"
#include <string.h>
#include <stdarg.h>

void bsp_vofa_fire_water(const char *format, ...);
void bsp_vofa_just_float(float *data_buffer, uint8_t data_count);

#endif

