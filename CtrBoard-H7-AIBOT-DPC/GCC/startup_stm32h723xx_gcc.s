.syntax unified
.cpu cortex-m7
.fpu fpv5-d16
.thumb

.global g_pfnVectors
.global __Vectors
.global __Vectors_End
.global __Vectors_Size

.word _sidata
.word _sdata
.word _edata
.word _sbss
.word _ebss

.section .text.Reset_Handler
.weak Reset_Handler
.type Reset_Handler, %function
Reset_Handler:
  ldr r0, =_estack
  mov sp, r0

  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
CopyData:
  cmp r0, r1
  bcc CopyDataWord
  b ZeroBss
CopyDataWord:
  ldr r3, [r2], #4
  str r3, [r0], #4
  b CopyData

ZeroBss:
  ldr r0, =_sbss
  ldr r1, =_ebss
  movs r2, #0
ZeroBssLoop:
  cmp r0, r1
  bcc ZeroBssWord
  b CallInit
ZeroBssWord:
  str r2, [r0], #4
  b ZeroBssLoop

CallInit:
  bl SystemInit
  bl __libc_init_array
  bl main

LoopForever:
  b LoopForever

.size Reset_Handler, .-Reset_Handler

.section .text.Default_Handler,"ax",%progbits
Default_Handler:
  b Default_Handler

.macro weak_handler name
  .weak \name
  .thumb_set \name, Default_Handler
.endm

weak_handler NMI_Handler
weak_handler HardFault_Handler
weak_handler MemManage_Handler
weak_handler BusFault_Handler
weak_handler UsageFault_Handler
weak_handler SVC_Handler
weak_handler DebugMon_Handler
weak_handler PendSV_Handler
weak_handler SysTick_Handler
weak_handler WWDG_IRQHandler
weak_handler PVD_AVD_IRQHandler
weak_handler TAMP_STAMP_IRQHandler
weak_handler RTC_WKUP_IRQHandler
weak_handler FLASH_IRQHandler
weak_handler RCC_IRQHandler
weak_handler EXTI0_IRQHandler
weak_handler EXTI1_IRQHandler
weak_handler EXTI2_IRQHandler
weak_handler EXTI3_IRQHandler
weak_handler EXTI4_IRQHandler
weak_handler DMA1_Stream0_IRQHandler
weak_handler DMA1_Stream1_IRQHandler
weak_handler DMA1_Stream2_IRQHandler
weak_handler DMA1_Stream3_IRQHandler
weak_handler DMA1_Stream4_IRQHandler
weak_handler DMA1_Stream5_IRQHandler
weak_handler DMA1_Stream6_IRQHandler
weak_handler ADC_IRQHandler
weak_handler FDCAN1_IT0_IRQHandler
weak_handler FDCAN2_IT0_IRQHandler
weak_handler FDCAN1_IT1_IRQHandler
weak_handler FDCAN2_IT1_IRQHandler
weak_handler EXTI9_5_IRQHandler
weak_handler TIM1_BRK_IRQHandler
weak_handler TIM1_UP_IRQHandler
weak_handler TIM1_TRG_COM_IRQHandler
weak_handler TIM1_CC_IRQHandler
weak_handler TIM2_IRQHandler
weak_handler TIM3_IRQHandler
weak_handler TIM4_IRQHandler
weak_handler I2C1_EV_IRQHandler
weak_handler I2C1_ER_IRQHandler
weak_handler I2C2_EV_IRQHandler
weak_handler I2C2_ER_IRQHandler
weak_handler SPI1_IRQHandler
weak_handler SPI2_IRQHandler
weak_handler USART1_IRQHandler
weak_handler USART2_IRQHandler
weak_handler USART3_IRQHandler
weak_handler EXTI15_10_IRQHandler
weak_handler RTC_Alarm_IRQHandler
weak_handler TIM8_BRK_TIM12_IRQHandler
weak_handler TIM8_UP_TIM13_IRQHandler
weak_handler TIM8_TRG_COM_TIM14_IRQHandler
weak_handler TIM8_CC_IRQHandler
weak_handler DMA1_Stream7_IRQHandler
weak_handler FMC_IRQHandler
weak_handler SDMMC1_IRQHandler
weak_handler TIM5_IRQHandler
weak_handler SPI3_IRQHandler
weak_handler UART4_IRQHandler
weak_handler UART5_IRQHandler
weak_handler TIM6_DAC_IRQHandler
weak_handler TIM7_IRQHandler
weak_handler DMA2_Stream0_IRQHandler
weak_handler DMA2_Stream1_IRQHandler
weak_handler DMA2_Stream2_IRQHandler
weak_handler DMA2_Stream3_IRQHandler
weak_handler DMA2_Stream4_IRQHandler
weak_handler ETH_IRQHandler
weak_handler ETH_WKUP_IRQHandler
weak_handler FDCAN_CAL_IRQHandler
weak_handler DMA2_Stream5_IRQHandler
weak_handler DMA2_Stream6_IRQHandler
weak_handler DMA2_Stream7_IRQHandler
weak_handler USART6_IRQHandler
weak_handler I2C3_EV_IRQHandler
weak_handler I2C3_ER_IRQHandler
weak_handler OTG_HS_EP1_OUT_IRQHandler
weak_handler OTG_HS_EP1_IN_IRQHandler
weak_handler OTG_HS_WKUP_IRQHandler
weak_handler OTG_HS_IRQHandler
weak_handler DCMI_PSSI_IRQHandler
weak_handler RNG_IRQHandler
weak_handler FPU_IRQHandler
weak_handler UART7_IRQHandler
weak_handler UART8_IRQHandler
weak_handler SPI4_IRQHandler
weak_handler SPI5_IRQHandler
weak_handler SPI6_IRQHandler
weak_handler SAI1_IRQHandler
weak_handler LTDC_IRQHandler
weak_handler LTDC_ER_IRQHandler
weak_handler DMA2D_IRQHandler
weak_handler OCTOSPI1_IRQHandler
weak_handler LPTIM1_IRQHandler
weak_handler CEC_IRQHandler
weak_handler I2C4_EV_IRQHandler
weak_handler I2C4_ER_IRQHandler
weak_handler SPDIF_RX_IRQHandler
weak_handler DMAMUX1_OVR_IRQHandler
weak_handler DFSDM1_FLT0_IRQHandler
weak_handler DFSDM1_FLT1_IRQHandler
weak_handler DFSDM1_FLT2_IRQHandler
weak_handler DFSDM1_FLT3_IRQHandler
weak_handler SWPMI1_IRQHandler
weak_handler TIM15_IRQHandler
weak_handler TIM16_IRQHandler
weak_handler TIM17_IRQHandler
weak_handler MDIOS_WKUP_IRQHandler
weak_handler MDIOS_IRQHandler
weak_handler MDMA_IRQHandler
weak_handler SDMMC2_IRQHandler
weak_handler HSEM1_IRQHandler
weak_handler ADC3_IRQHandler
weak_handler DMAMUX2_OVR_IRQHandler
weak_handler BDMA_Channel0_IRQHandler
weak_handler BDMA_Channel1_IRQHandler
weak_handler BDMA_Channel2_IRQHandler
weak_handler BDMA_Channel3_IRQHandler
weak_handler BDMA_Channel4_IRQHandler
weak_handler BDMA_Channel5_IRQHandler
weak_handler BDMA_Channel6_IRQHandler
weak_handler BDMA_Channel7_IRQHandler
weak_handler COMP1_IRQHandler
weak_handler LPTIM2_IRQHandler
weak_handler LPTIM3_IRQHandler
weak_handler LPTIM4_IRQHandler
weak_handler LPTIM5_IRQHandler
weak_handler LPUART1_IRQHandler
weak_handler CRS_IRQHandler
weak_handler ECC_IRQHandler
weak_handler SAI4_IRQHandler
weak_handler DTS_IRQHandler
weak_handler WAKEUP_PIN_IRQHandler
weak_handler OCTOSPI2_IRQHandler
weak_handler FMAC_IRQHandler
weak_handler CORDIC_IRQHandler
weak_handler UART9_IRQHandler
weak_handler USART10_IRQHandler
weak_handler I2C5_EV_IRQHandler
weak_handler I2C5_ER_IRQHandler
weak_handler FDCAN3_IT0_IRQHandler
weak_handler FDCAN3_IT1_IRQHandler
weak_handler TIM23_IRQHandler
weak_handler TIM24_IRQHandler

.section .isr_vector,"a",%progbits
.type g_pfnVectors, %object
g_pfnVectors:
__Vectors:
  .word _estack
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word MemManage_Handler
  .word BusFault_Handler
  .word UsageFault_Handler
  .word 0
  .word 0
  .word 0
  .word 0
  .word SVC_Handler
  .word DebugMon_Handler
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler
  .word WWDG_IRQHandler
  .word PVD_AVD_IRQHandler
  .word TAMP_STAMP_IRQHandler
  .word RTC_WKUP_IRQHandler
  .word FLASH_IRQHandler
  .word RCC_IRQHandler
  .word EXTI0_IRQHandler
  .word EXTI1_IRQHandler
  .word EXTI2_IRQHandler
  .word EXTI3_IRQHandler
  .word EXTI4_IRQHandler
  .word DMA1_Stream0_IRQHandler
  .word DMA1_Stream1_IRQHandler
  .word DMA1_Stream2_IRQHandler
  .word DMA1_Stream3_IRQHandler
  .word DMA1_Stream4_IRQHandler
  .word DMA1_Stream5_IRQHandler
  .word DMA1_Stream6_IRQHandler
  .word ADC_IRQHandler
  .word FDCAN1_IT0_IRQHandler
  .word FDCAN2_IT0_IRQHandler
  .word FDCAN1_IT1_IRQHandler
  .word FDCAN2_IT1_IRQHandler
  .word EXTI9_5_IRQHandler
  .word TIM1_BRK_IRQHandler
  .word TIM1_UP_IRQHandler
  .word TIM1_TRG_COM_IRQHandler
  .word TIM1_CC_IRQHandler
  .word TIM2_IRQHandler
  .word TIM3_IRQHandler
  .word TIM4_IRQHandler
  .word I2C1_EV_IRQHandler
  .word I2C1_ER_IRQHandler
  .word I2C2_EV_IRQHandler
  .word I2C2_ER_IRQHandler
  .word SPI1_IRQHandler
  .word SPI2_IRQHandler
  .word USART1_IRQHandler
  .word USART2_IRQHandler
  .word USART3_IRQHandler
  .word EXTI15_10_IRQHandler
  .word RTC_Alarm_IRQHandler
  .word 0
  .word TIM8_BRK_TIM12_IRQHandler
  .word TIM8_UP_TIM13_IRQHandler
  .word TIM8_TRG_COM_TIM14_IRQHandler
  .word TIM8_CC_IRQHandler
  .word DMA1_Stream7_IRQHandler
  .word FMC_IRQHandler
  .word SDMMC1_IRQHandler
  .word TIM5_IRQHandler
  .word SPI3_IRQHandler
  .word UART4_IRQHandler
  .word UART5_IRQHandler
  .word TIM6_DAC_IRQHandler
  .word TIM7_IRQHandler
  .word DMA2_Stream0_IRQHandler
  .word DMA2_Stream1_IRQHandler
  .word DMA2_Stream2_IRQHandler
  .word DMA2_Stream3_IRQHandler
  .word DMA2_Stream4_IRQHandler
  .word ETH_IRQHandler
  .word ETH_WKUP_IRQHandler
  .word FDCAN_CAL_IRQHandler
  .word 0
  .word 0
  .word 0
  .word 0
  .word DMA2_Stream5_IRQHandler
  .word DMA2_Stream6_IRQHandler
  .word DMA2_Stream7_IRQHandler
  .word USART6_IRQHandler
  .word I2C3_EV_IRQHandler
  .word I2C3_ER_IRQHandler
  .word OTG_HS_EP1_OUT_IRQHandler
  .word OTG_HS_EP1_IN_IRQHandler
  .word OTG_HS_WKUP_IRQHandler
  .word OTG_HS_IRQHandler
  .word DCMI_PSSI_IRQHandler
  .word 0
  .word RNG_IRQHandler
  .word FPU_IRQHandler
  .word UART7_IRQHandler
  .word UART8_IRQHandler
  .word SPI4_IRQHandler
  .word SPI5_IRQHandler
  .word SPI6_IRQHandler
  .word SAI1_IRQHandler
  .word LTDC_IRQHandler
  .word LTDC_ER_IRQHandler
  .word DMA2D_IRQHandler
  .word 0
  .word OCTOSPI1_IRQHandler
  .word LPTIM1_IRQHandler
  .word CEC_IRQHandler
  .word I2C4_EV_IRQHandler
  .word I2C4_ER_IRQHandler
  .word SPDIF_RX_IRQHandler
  .word 0
  .word 0
  .word 0
  .word 0
  .word DMAMUX1_OVR_IRQHandler
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word DFSDM1_FLT0_IRQHandler
  .word DFSDM1_FLT1_IRQHandler
  .word DFSDM1_FLT2_IRQHandler
  .word DFSDM1_FLT3_IRQHandler
  .word 0
  .word SWPMI1_IRQHandler
  .word TIM15_IRQHandler
  .word TIM16_IRQHandler
  .word TIM17_IRQHandler
  .word MDIOS_WKUP_IRQHandler
  .word MDIOS_IRQHandler
  .word 0
  .word MDMA_IRQHandler
  .word 0
  .word SDMMC2_IRQHandler
  .word HSEM1_IRQHandler
  .word 0
  .word ADC3_IRQHandler
  .word DMAMUX2_OVR_IRQHandler
  .word BDMA_Channel0_IRQHandler
  .word BDMA_Channel1_IRQHandler
  .word BDMA_Channel2_IRQHandler
  .word BDMA_Channel3_IRQHandler
  .word BDMA_Channel4_IRQHandler
  .word BDMA_Channel5_IRQHandler
  .word BDMA_Channel6_IRQHandler
  .word BDMA_Channel7_IRQHandler
  .word COMP1_IRQHandler
  .word LPTIM2_IRQHandler
  .word LPTIM3_IRQHandler
  .word LPTIM4_IRQHandler
  .word LPTIM5_IRQHandler
  .word LPUART1_IRQHandler
  .word 0
  .word CRS_IRQHandler
  .word ECC_IRQHandler
  .word SAI4_IRQHandler
  .word DTS_IRQHandler
  .word 0
  .word WAKEUP_PIN_IRQHandler
  .word OCTOSPI2_IRQHandler
  .word 0
  .word 0
  .word FMAC_IRQHandler
  .word CORDIC_IRQHandler
  .word UART9_IRQHandler
  .word USART10_IRQHandler
  .word I2C5_EV_IRQHandler
  .word I2C5_ER_IRQHandler
  .word FDCAN3_IT0_IRQHandler
  .word FDCAN3_IT1_IRQHandler
  .word TIM23_IRQHandler
  .word TIM24_IRQHandler
__Vectors_End:
__Vectors_Size = __Vectors_End - __Vectors
.size g_pfnVectors, .-g_pfnVectors
