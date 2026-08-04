#ifndef _dma_H
#define _dma_H

#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"

#define DBUS_MAX_LEN 36
#define DBUS_BUFLEN 18

#define USART2_MAX_LEN 50
#define USART2_BUFLEN  18

#define JUDGE_MAX_LEN 1024

#define PC_MAX_LEN 512

#define USART8_MAX_LEN 50
#define USART8_BUFLEN  18

#define SPI1_BUFLEN    23

extern uint8_t dbus_buf[2][DBUS_MAX_LEN];
extern uint8_t  judge_rxbuf[2][JUDGE_MAX_LEN];
extern uint8_t  pc_rxbuf[2][PC_MAX_LEN];

void USART1_DMA(void);
void USART3_DMA(void);
void USART6_DMA(void);



#endif
