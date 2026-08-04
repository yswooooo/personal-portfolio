#ifndef _usart_H
#define _usart_H

#include "stm32f4xx.h"
#include <stdio.h>

void USART1_DEVICE(void);//PC
void USART3_DEVICE(void);//dbus


void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);
void Usart_SendString( USART_TypeDef * pUSARTx, char *str);
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch);

#endif 

