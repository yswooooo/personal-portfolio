#ifndef __FILTER_FIR_H__
#define __FILTER_FIR_H__

#define NUM_TAPS 33  // FIR 滤波器系数的数量
#define BUFFER_SIZE 1000  // 输入信号缓冲区大小

float fir_filter(float input);

#endif
