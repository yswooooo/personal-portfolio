#include "bsp_vofa.h"


// 按printf格式写，最后必须加\r\n
void bsp_vofa_fire_water(const char *format, ...)
{
    uint8_t tx_buffer[100];
    uint32_t tx_len;
    va_list va_args;
    va_start(va_args, format);
    tx_len = vsnprintf((char *)tx_buffer, 100, format, va_args);

    //....在此替换你的串口发送函数...........
    //HAL_UART_Transmit_DMA(&huart1, (uint8_t *)tx_buffer, tx_len);
	HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer, tx_len,1000);
    //......................................

    va_end(va_args);
}

// 输入个数和数组地址
void bsp_vofa_just_float(float *data_buffer, uint8_t data_count)
{
    uint8_t frame_buffer[100];
    uint8_t frame_tail[4] = {0, 0, 0x80, 0x7F};
    float data_copy[data_count];

    memcpy(&data_copy, data_buffer, sizeof(float) * data_count);

    memcpy(frame_buffer, (uint8_t *)&data_copy, sizeof(data_copy));
    memcpy(&frame_buffer[data_count * 4], &frame_tail[0], 4);

    //....在此替换你的串口发送函数...........
    //HAL_UART_Transmit(&huart1, frame_buffer, (data_count + 1) * 4,100);
		HAL_UART_Transmit(&huart1, frame_buffer, (data_count + 1) * 4,100);
    //......................................
}

/*...........示例..............
    float f1=0.5,f2=114.514;
    bsp_vofa_fire_water("%f,%f\r\n", f1, f2);

    float f3[3]={88.77,0.66,55.44};
    bsp_vofa_just_float(f3, 3);
*/

