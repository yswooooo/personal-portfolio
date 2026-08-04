#ifndef _LADRC_H
#define _LADRC_H

#include "stm32f4xx.h"
/**
 *@Brief  以下为LADRC系统参数
 *@WangShun  2022-07-03  注释
 */
typedef struct LADRC
{
   float v1, v2;        //最速输出值
   float r;             //速度因子
   float h;             //积分步长
   float z1, z2, z3;    //观测器输出
   float w0, wc, b0, u; //观测器带宽 控制器带宽 系统参数 控制器输出
	 int32_t maxout;
} LADRC_NUM;

/**
 *@Brief  以下为LADRC相关函数
 *@WangShun  2022-07-03  注释
 */
void LADRC_Init(LADRC_NUM *LADRC_TYPE1,float h,float r,float wc,float w0,float b0,int32_t max_out);
void LADRC_REST(LADRC_NUM *LADRC_TYPE1);
void LADRC_TD(LADRC_NUM *LADRC_TYPE1, float Expect);
void LADRC_ESO(LADRC_NUM *LADRC_TYPE1, float FeedBack);
void LADRC_LF(LADRC_NUM *LADRC_TYPE1);
float LADRC_Loop(LADRC_NUM *LADRC_TYPE1, float Expect, float RealTimeOut);

extern LADRC_NUM Fric_speed;

//float ladrc_param={0.005, 20, 100, 400, 0.5};
//LADRC_Init(&Fric_speed,ladrc_param[0],ladrc_param[1],ladrc_param[2],ladrc_param[3],ladrc_param[4],int32_t max_out);
//LADRC_Loop(&Fric_speed,set,get);
//LADRC_REST(&Fric_speed);

#endif
