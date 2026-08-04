#include "st_usb_cdc.h"

/**
 * @file st_usb_cdc.c
 * @author Liao Wenhao
 * @brief USB CDC 驱动
 * @version 1.0
 * @date 2023-04-06
 * 
 * @copyright Copyright (c) 2023
 * 
 * @note 本文件用于初始化USB-FS-Device的gpio和时钟电源
 * 
 */
void USB_OTG_BSP_Init(USB_OTG_CORE_HANDLE *pdev)
{
    /* Configure DM DP Pins */
    GPIO_InitTypeDef GPIO_InitStructure;
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_OTG1_FS);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_OTG1_FS);
		
		RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, ENABLE);


}

void USB_OTG_BSP_EnableInterrupt(USB_OTG_CORE_HANDLE *pdev)
{
	  /* Enable the USB global Interrupt */
    NVIC_InitTypeDef NVIC_InitStructure;
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_InitStructure.NVIC_IRQChannel = OTG_FS_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void USB_OTG_BSP_uDelay(const uint32_t usec)
{
	uint32_t count = 0;
	const uint32_t utime = (168 * usec);		
	do
	{
		if ( ++count > utime )
		{
			return ;
		}
	}
	while (1);
}


void USB_OTG_BSP_mDelay(const uint32_t msec)
{
	USB_OTG_BSP_uDelay(msec * 1000);
}

