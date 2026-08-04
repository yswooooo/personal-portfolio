#include "tim.h"

void TIM8_DEVICE(int per,int psc)//BALL
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
  
  GPIO_PinAFConfig(GPIOI,GPIO_PinSource7,GPIO_AF_TIM8);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOI,&GPIO_InitStructure);
  
  TIM_TimeBaseInitStructure.TIM_Period = per;
  TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM8,&TIM_TimeBaseInitStructure);
  
	TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 2000;
  TIM_OC1Init(TIM8,&TIM_OCInitStructure);
	TIM_OC2Init(TIM8,&TIM_OCInitStructure);
	TIM_OC3Init(TIM8,&TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM8,TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM8,TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM8,TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM8,ENABLE);
	TIM_CtrlPWMOutputs(TIM8,ENABLE);//高级定时器TIM1 TIM8必须使能这一句，否则无法输出PWM
  TIM_Cmd(TIM8,ENABLE);
}

void TIM10_DEVICE(int per,int psc)/*GPYO*/
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);
  
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource6,GPIO_AF_TIM10);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
  
  TIM_TimeBaseInitStructure.TIM_Period = per;
  TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM10,&TIM_TimeBaseInitStructure);
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  
  TIM_OC1Init(TIM10,&TIM_OCInitStructure);

  
  TIM_OC1PreloadConfig(TIM10,TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM10,ENABLE);
  
  TIM_Cmd(TIM10,ENABLE);
}
