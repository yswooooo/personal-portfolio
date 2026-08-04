#include "judge_tx_data.h"
#include "dma.h"
#include "judge_rx_data.h"
#include "string.h"
#include "shoot_task.h"
#include "remote_ctrl.h"
#include "chassis_task.h"
#include "modeswitch_task.h"
#include "gimbal_task.h"
#include "math.h"
#include "pc_rx_data.h"

extern uint8_t shoot_ready;
extern chassis_t chassis;
judge_txdata_t judge_send_mesg;//裁判系统发送结构体，用户自定义发送的数据打包在里面，到时候可以在debug里查看与修改
static SemaphoreHandle_t judge_txdata_mutex;
fifo_s_t  judge_txdata_fifo;//用来暂时存放数据的邮箱
static uint8_t   judge_txdata_buf[JUDGE_TX_FIFO_BUFLEN];
//extern float cap_store;
void judgement_tx_param_init(void)
{
  /* create the judge_rxdata_mutex mutex  */  
  judge_txdata_mutex = xSemaphoreCreateMutex();
  
  /* judge data fifo init */
  fifo_s_init(&judge_txdata_fifo, judge_txdata_buf, JUDGE_TX_FIFO_BUFLEN, judge_txdata_mutex);
}

#define Hero 1 //步兵用此处要注释掉，英雄用此处为1 不同处在准心 !!!

static void client_graphic_draw_short_line1(void);
static void client_graphic_draw_car_line(void);//车身线和准心线
static void client_graphic_draw_String(char *str,char name,char type,char layer,char color,uint16_t start_x,uint16_t start_y);//动态字符
static void client_grapjic_draw_float(float num1,float num2,char name,char type,char layer,char color,uint16_t start_x,uint16_t start_y);//浮点数

/*以下数据均是需要看实际显示效果修改的，所以在绘图时可以先用它赋值给结构体，便于我们在debug中改变结构体的
	值，等合适的值确定下来，再将这个确定的值以常数赋值给结构体*/
//uint32_t debug_start_angle = 0;
//uint32_t debug_end_angle = 0;
//uint32_t debug_width = 0;
//uint32_t debug_start_x = 800;
//uint32_t debug_start_y = 0;
//uint32_t debug_radius = 0;
//uint32_t debug_end_x = 1400;
//uint32_t debug_end_y = 0;

/**
参数——
num1：浮点数电压，pit用;num1、num2补偿角用 
name: Cap,Pitch; type: ADD,Change; color: 颜色;
start_x、start_y: 起始坐标  
**/
static void client_grapjic_draw_float(float num1,float num2,char name,char type,char layer,char color,uint16_t start_x,uint16_t start_y)
{
	char float_string[20] = {0};
	if(name == Pitch) 
		strcpy(float_string ,"Pit: 00.0");
	if(name == Cap) 
		strcpy(float_string ,"ALL:00.0\n\nCap:00.0");
	if(name == Speed)
		strcpy(float_string ,"Spd:0000\n\nDog:000");
	if(name == Distance)
		strcpy(float_string ,"Dis: 00.0");
	if(name == Compensates)//yaw、pit
		strcpy(float_string ,"Y :00.0\n\n\nP :00.0");
	if(name == Pitch||name == Distance)
	{
		if(num1 < 0) float_string[4] = '-';
		float_string[5] = (uint8_t)fabs(num1)/10+48; //48——'0'ASCII码
		float_string[6] = (uint8_t)fabs(num1)%10+48;
		float_string[7] = '.';
		float_string[8] = (uint8_t)(fmod(fabs(num1)*10.0,10.0)+48);
	}
	else if(name == Compensates)
	{
		if(num1 < 0) float_string[2] = '-';
		float_string[3] = (uint8_t)fabs(num1)/100+48; //48——'0'ASCII码
		float_string[4] = (uint8_t)fabs(num1)/10%10+48;
		float_string[5] = (uint8_t)fabs(num1)%10+48;
		float_string[6] = '.';
		float_string[7] = (uint8_t)(fmod(fabs(num1)*10.0,10.0)+48);	

		if(num2 < 0) float_string[12] = '-';
		float_string[13] = (uint8_t)fabs(num2)/10+48; //48——'0'ASCII码
		float_string[14] = (uint8_t)fabs(num2)%10+48;
		float_string[15] = '.';
		float_string[16] = (uint8_t)(fmod(fabs(num2)*10.0,10.0)+48);		
	}
	 if(name == Cap)
	{
		float_string[4] = (uint8_t)fabs(num1)/10+48; //48——'0'ASCII码
		float_string[5] = (uint8_t)fabs(num1)%10+48;
		float_string[6] = '.';
		float_string[7] = (uint8_t)(fmod(fabs(num1)*10.0,10.0)+48);

		float_string[14] = (uint8_t)fabs(num2)/10+48; //48——'0'ASCII码
		float_string[15] = (uint8_t)fabs(num2)%10+48;
		float_string[16] = '.';
		float_string[17] = (uint8_t)(fmod(fabs(num2)*10.0,10.0)+48);
	}
	if(name == Speed)
	{
		float_string[4] = (uint16_t)fabs(num1)/1000+48; //48——'0'ASCII码
		float_string[5] = (uint16_t)fabs(num1)/100-(uint16_t)fabs(num1)/1000*10+48;
		float_string[6] = (uint16_t)fabs(num1)/10-(uint16_t)fabs(num1)/100*10+48;
		float_string[7] = (uint16_t)fabs(num1)%10+48;
		
		float_string[14] = (uint16_t)fabs(num2)/100+48;
		float_string[15] = (uint16_t)fabs(num2)/10-(uint16_t)fabs(num2)/100*10+48;
		float_string[16] = (uint16_t)fabs(num2)%10+48;
		
	}
	
	memcpy(&judge_send_mesg.ext_client_custom_character.data[0],float_string,sizeof(float_string));
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.graphic_name[0] = name;
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.operate_tpye = type;
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.graphic_tpye = Character;
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.layer = layer;
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.color =  color ;
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.start_angle =  20 ;
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.end_angle =  12 ;
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.width = 3;
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.start_x = start_x;
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.start_y = start_y;
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.radius = 0;
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.end_x = 0;
	judge_send_mesg.ext_client_custom_character.grapic_data_struct.end_y = 0;		
}

/**
** @ 参数  *str, name, type, layer, color, start_x, start_y
** @ str：字符; name: Gimbal,Chassis,Shoot; type: ADD增加,Change; color: 颜色;
**/
static void client_graphic_draw_String(char *str,char name,char type,char layer,char color,uint16_t start_x,uint16_t start_y)
{
	char string[15] = {0};
	strcpy(string,str);
	if(name == Gimbal) 
	{
		memcpy(&judge_send_mesg.ext_client_custom_character_gimbal.data[0], string, sizeof(string));
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.graphic_name[0] = name;
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.operate_tpye = type;
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.graphic_tpye = Character;
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.layer = layer;
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.color =  color ;
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.start_angle =  20 ;
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.end_angle =sizeof(string) ;
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.width = 3;
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.start_x = start_x;
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.start_y = start_y;
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.radius = 0;
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.end_x = 0;
		judge_send_mesg.ext_client_custom_character_gimbal.grapic_data_struct.end_y = 0;	
	}
	else if(name == Chassis)
	{
		memcpy(&judge_send_mesg.ext_client_custom_character_chassis.data[0], string, sizeof(string));
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.graphic_name[0] = name;
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.operate_tpye = type;
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.graphic_tpye = Character;
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.layer = layer;
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.color =  color;
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_angle =  20 ;
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_angle =sizeof(string) ;
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.width = 3;
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_x = start_x;
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.start_y = start_y;
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.radius = 0;
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_x = 0;
		judge_send_mesg.ext_client_custom_character_chassis.grapic_data_struct.end_y = 0;		
	}
	else if(name == Shoot)
	{
		memcpy(&judge_send_mesg.ext_client_custom_character_shoot.data[0], string, sizeof(string));
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.graphic_name[0] = name;
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.operate_tpye = type;
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.graphic_tpye = Character;
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.layer = layer;
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.color =  color;
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.start_angle =  20 ;
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.end_angle =sizeof(string) ;
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.width = 3;
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.start_x = start_x;
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.start_y = start_y;
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.radius = 0;
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.end_x = 0;
		judge_send_mesg.ext_client_custom_character_shoot.grapic_data_struct.end_y = 0;		
	}
	else if(name == Vision)//视觉有效位
	{
		memcpy(&judge_send_mesg.ext_client_custom_character.data[0], string, sizeof(string));
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.graphic_name[0] = name;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.operate_tpye = type;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.graphic_tpye = Character;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.layer = layer;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.color =  color ;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.start_angle =  20;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.end_angle =sizeof(string) ;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.width = 3;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.start_x = start_x;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.start_y = start_y;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.radius = 0;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.end_x = 0;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.end_y = 0;	
		
	}
		else 
	{
		memcpy(&judge_send_mesg.ext_client_custom_character.data[0], string, sizeof(string));
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.graphic_name[0] = name;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.operate_tpye = type;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.graphic_tpye = Character;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.layer = layer;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.color =  color ;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.start_angle =  20;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.end_angle =sizeof(string) ;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.width = 3;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.start_x = start_x;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.start_y = start_y;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.radius = 0;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.end_x = 0;
		judge_send_mesg.ext_client_custom_character.grapic_data_struct.end_y = 0;	
		
	}
}
// uint32_t x_s= 475;
// uint32_t x_e= 600;
// uint32_t y_s= 180;
// uint32_t y_e= 180;

/* Layer 7 */
static void client_graphic_draw_car_line(void)//车身线和5米竖线 
{	//车身线补偿
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[0].graphic_name[0] = line_1c;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[0].operate_tpye = Add;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[0].graphic_tpye = Straight_line;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[0].layer = layer4;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[0].color = Redblue;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[0].start_angle = 0;	
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[0].end_angle = 0;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[0].width = 3;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[0].start_x = 600; 
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[0].start_y = 2;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[0].radius = 0;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[0].end_x = 675;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[0].end_y = 90;
	//车身线横线
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[1].graphic_name[0] = line_2c;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[1].operate_tpye = Add;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[1].graphic_tpye = Straight_line;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[1].layer = layer4;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[1].color = Redblue;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[1].start_angle = 0;	
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[1].end_angle = 0;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[1].width = 3;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[1].start_x = 750;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[1].start_y = 180; 
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[1].radius = 0;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[1].end_x = 1200;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[1].end_y = 180;
	//车身线右斜线（\） 
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[2].graphic_name[0] = line_3c;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[2].operate_tpye = Add;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[2].graphic_tpye = Straight_line;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[2].layer = layer4;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[2].color = Redblue;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[2].start_angle = 0;	
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[2].end_angle = 0;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[2].width = 3;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[2].start_x = 1200; 
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[2].start_y = 180;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[2].radius = 0;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[2].end_x = 1275;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[2].end_y = 90;
	//车身线补偿
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[3].graphic_name[0] = line_4c;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[3].operate_tpye = Add;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[3].graphic_tpye = Straight_line;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[3].layer = layer4;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[3].color = Redblue;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[3].start_angle = 0;	
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[3].end_angle = 0;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[3].width = 3;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[3].start_x = 1275;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[3].start_y = 90;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[3].radius = 0;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[3].end_x = 1350;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[3].end_y = 2;
	//车身线斜线（/）
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[4].graphic_name[0] = line_5c;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[4].operate_tpye = Add;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[4].graphic_tpye = Straight_line;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[4].layer = layer4;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[4].color = Redblue;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[4].start_angle = 0;	
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[4].end_angle = 0;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[4].width = 3;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[4].start_x = 675;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[4].start_y = 90;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[4].radius = 0;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[4].end_x = 750;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[4].end_y = 180;



	//上方第四根，最顶
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[5].graphic_name[0] = line_8s;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[5].operate_tpye = Add;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[5].graphic_tpye = Straight_line;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[5].layer = layer4;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[5].color = Yellow;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[5].start_angle = 0;	
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[5].end_angle = 0;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[5].width = 1;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[5].start_x = 945;//942
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[5].start_y = 660;//367;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[5].radius = 0;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[5].end_x = 977;//980
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[5].end_y =  660;
	//第二上
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[6].graphic_name[0] = line_9s;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[6].operate_tpye = Add;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[6].graphic_tpye = Straight_line;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[6].layer = layer4;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[6].color = Amaranth;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[6].start_angle = 0;	
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[6].end_angle = 0;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[6].width = 2;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[6].start_x = 915;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[6].start_y = 600;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[6].radius = 0;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[6].end_x = 1005;
	judge_send_mesg.ext_client_custom_graphic_seven.interaction_figure[6].end_y = 600;

}

//客户端自定义UI界面 ！！！！！！！！！！！！！！！！！！！！！！！
uint8_t Line_mask = 0;
uint8_t text_mask = 1;
uint8_t Chassis_mask = 0;
uint8_t Gimbal_mask = 0;
uint8_t Shoot_mask = 0;
uint8_t	view_mask = 0;	
uint8_t Idle_mask = 0;//空闲标志
void judgement_client_graphics_draw_pack(uint8_t text_twist)
{
	//自定义UI receiver_ID 只能选当前机器人的对应的客户端,解耦化，以后每台车UI都可以用差不多的结构
	uint8_t current_robot_id = judge_recv_mesg.game_robot_state.robot_id;  //读取当前机器人的id
	uint16_t receiver_ID = 0; //未用
	switch(current_robot_id)
	{
		//red robot
		case STUDENT_RED_HERO_ID:
		{
			receiver_ID = RED_HERO_CLIENT_ID;
		}break;
		case STUDENT_RED_ENGINEER_ID:
		{
			receiver_ID = RED_ENGINEER_CLIENT_ID;
		}break;
		case STUDENT_RED_AERIAL_ID:
		{
			receiver_ID = RED_AERIAL_CLIENT_ID;
		}break;

		case STUDENT_RED_INFANTRY3_ID:
		{
			receiver_ID = RED_INFANTRY3_CLIENT_ID;
		}break;
		case STUDENT_RED_INFANTRY4_ID:
		{
			receiver_ID = RED_INFANTRY4_CLIENT_ID;
		}break;
		case STUDENT_RED_INFANTRY5_ID:
		{
			receiver_ID = RED_INFANTRY5_CLIENT_ID;
		}break;
		
		//blue robot
		case STUDENT_BLUE_HERO_ID:
		{
			receiver_ID = BLUE_HERO_CLIENT_ID;
		}break;
		case STUDENT_BLUE_ENGINEER_ID:
		{
			receiver_ID = BLUE_ENGINEER_CLIENT_ID;
		}break;
		case STUDENT_BLUE_AERIAL_ID:
		{
			receiver_ID = BLUE_AERIAL_CLIENT_ID;
		}break;
		case STUDENT_BLUE_INFANTRY3_ID:
		{
			receiver_ID = BLUE_INFANTRY3_CLIENT_ID;
		}break;	
		case STUDENT_BLUE_INFANTRY4_ID:
		{
			receiver_ID = BLUE_INFANTRY4_CLIENT_ID;
		}break;
		case STUDENT_BLUE_INFANTRY5_ID:
		{
			receiver_ID = BLUE_INFANTRY5_CLIENT_ID;
		}break;
	}
	
	//清除数组
	memset(&judge_send_mesg.ext_client_custom_character.data[0],0,sizeof(judge_send_mesg.ext_client_custom_character.data[0]));
	memset(&judge_send_mesg.ext_client_custom_character_chassis.data[0],0,sizeof(judge_send_mesg.ext_client_custom_character_chassis.data[0]));
	memset(&judge_send_mesg.ext_client_custom_character_shoot.data[0],0,sizeof(judge_send_mesg.ext_client_custom_character_shoot.data[0]));
	memset(&judge_send_mesg.ext_client_custom_character_gimbal.data[0],0,sizeof(judge_send_mesg.ext_client_custom_character_gimbal.data[0]));
	
	static uint8_t i; //i++
	static uint8_t again_flag; //重刷UI

	if(rc.sw2 == RC_DN && rc.sw1 == RC_MI && !again_flag  )//左拨杆打中打下可重复刷新，提高容错率
	{
		Line_mask = 1;
		text_mask = 1;
		again_flag = 1;
	}
	else if(!(rc.sw2 == RC_DN && rc.sw1 == RC_MI))
		again_flag = 0;

#ifdef Hero
	if(rc.sw2 == RC_UP) //启动机器人不画UI防止复活后重复画准心，英雄右拨杆打到上档RC_UP，步兵打中档RC_MI
		Line_mask = 3; //步兵为 =4 !!!
#else
	if(rc.sw2 == RC_MI) //启动机器人不画UI防止复活后重复画准心，英雄打到上档RC_UP，步兵打中档RC_MI
		Line_mask = 4;
#endif	
	
	//画线
	if(Line_mask == 1)//垂直射击准心和3米竖线
	{
		if(i++ <3)
		{
			judge_send_mesg.ext_client_custom_graphic_seven1.data_cmd_id = Client_Draw_Seven_Graph;
			judge_send_mesg.ext_client_custom_graphic_seven1.sender_ID = (uint16_t)current_robot_id;
			judge_send_mesg.ext_client_custom_graphic_seven1.receiver_ID = receiver_ID;
			client_graphic_draw_short_line1 ();//NOTE:画英雄中心准心线
			data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_graphic_seven1,
										sizeof(judge_send_mesg.ext_client_custom_graphic_seven1), DN_REG_ID);
		}
		else
		{
			Line_mask = 2;
			i = 0;
		}
	}
	if(Line_mask == 2)//车身线和5米竖线
	{
		if(i++ < 3)
		{	
			judge_send_mesg.ext_client_custom_graphic_seven.data_cmd_id = Client_Draw_Seven_Graph;
			judge_send_mesg.ext_client_custom_graphic_seven.sender_ID = (uint16_t)current_robot_id;
			judge_send_mesg.ext_client_custom_graphic_seven.receiver_ID = receiver_ID;
			client_graphic_draw_car_line();//NOTE:画车身线和中心准心线
			data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,
							(uint8_t *)&judge_send_mesg.ext_client_custom_graphic_seven,
							sizeof(judge_send_mesg.ext_client_custom_graphic_seven), 
							DN_REG_ID);
		}
		else
		{
			Line_mask = 3;
			i = 0;
		}
	}

//画文字
#ifdef Hero
	if(Line_mask == 3)//线画好了，开始画字符 步兵为 == 4 !!!
#else
	if(Line_mask == 4)
#endif	
	{
		if(text_mask == 1) //Add
		{
			i++;
			if(i >= 1 && i<6) //图层3 底盘6
			{
				judge_send_mesg.ext_client_custom_character_chassis.data_cmd_id = Client_Draw_Character_Graph;
				judge_send_mesg.ext_client_custom_character_chassis.sender_ID = (uint16_t)current_robot_id;
				judge_send_mesg.ext_client_custom_character_chassis.receiver_ID = receiver_ID;
				client_graphic_draw_String("Chassis: NORMAL",Chassis,Add,layer3,Cyan,300,770);//NOTE:画字符串
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_chassis,
												sizeof(judge_send_mesg.ext_client_custom_character_chassis), DN_REG_ID);
			}	
			else if(i >= 6 && i<11)//发射
			{				
				judge_send_mesg.ext_client_custom_character_shoot.data_cmd_id = Client_Draw_Character_Graph;
				judge_send_mesg.ext_client_custom_character_shoot.sender_ID = (uint16_t)current_robot_id;
				judge_send_mesg.ext_client_custom_character_shoot.receiver_ID = receiver_ID;	
				client_graphic_draw_String("FRICTION:OFF",Shoot,Add,layer3,Pink,300,700);
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_shoot,
												sizeof(judge_send_mesg.ext_client_custom_character_shoot), DN_REG_ID);
			}
			else if(i == 11)//Cap
				client_grapjic_draw_float(chassis.cap_store,chassis.CapData[1],Cap,Add,layer2,Yellow,1100,570); //图层2
			else if(i == 12)//轻触开关
			{
				memset(&judge_send_mesg.ext_client_custom_character,0,sizeof(judge_send_mesg.ext_client_custom_character));//清空数组
				client_graphic_draw_String("Shoot_ready:OFF",View,Add,layer2,Pink,1100,467);
			}
			else if(i == 13)//Yaw,Pit
			{
				memset(&judge_send_mesg.ext_client_custom_character,0,sizeof(judge_send_mesg.ext_client_custom_character));//清空数组
				client_grapjic_draw_float(gimbal.sensor.yaw_relative_angle,gimbal.sensor.pit_relative_angle,Compensates,Add,layer2,Yellow,1700,890);			
			}
//			 else if(i == 14)//Speed
//			 {
//			 	memset(&judge_send_mesg.ext_client_custom_character,0,sizeof(judge_send_mesg.ext_client_custom_character));//清空数组
//			 	client_grapjic_draw_float(chassis.vx,chassis.vw,Speed,Add,layer2,Yellow,1500,890);
//			 }
			else
			{
				text_mask = 2;
				i = 0;
			}
			if(i >= 11)
			{
				judge_send_mesg.ext_client_custom_character.data_cmd_id = Client_Draw_Character_Graph;
				judge_send_mesg.ext_client_custom_character.sender_ID = (uint16_t)current_robot_id;
				judge_send_mesg.ext_client_custom_character.receiver_ID = receiver_ID;	
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character,
												sizeof(judge_send_mesg.ext_client_custom_character), DN_REG_ID);				
			}
		}
		if(text_mask == 2)//Change
		{
			judge_send_mesg.ext_client_custom_character_chassis.data_cmd_id = Client_Draw_Character_Graph;
			judge_send_mesg.ext_client_custom_character_chassis.sender_ID = (uint16_t)current_robot_id;
			judge_send_mesg.ext_client_custom_character_chassis.receiver_ID = receiver_ID;

			judge_send_mesg.ext_client_custom_character_gimbal.data_cmd_id = Client_Draw_Character_Graph;
			judge_send_mesg.ext_client_custom_character_gimbal.sender_ID = (uint16_t)current_robot_id;
			judge_send_mesg.ext_client_custom_character_gimbal.receiver_ID = receiver_ID;

			judge_send_mesg.ext_client_custom_character_shoot.data_cmd_id = Client_Draw_Character_Graph;
			judge_send_mesg.ext_client_custom_character_shoot.sender_ID = (uint16_t)current_robot_id;
			judge_send_mesg.ext_client_custom_character_shoot.receiver_ID = receiver_ID;
			
			memset(&judge_send_mesg.ext_client_custom_character,0,sizeof(judge_send_mesg.ext_client_custom_character));//清空数组
			judge_send_mesg.ext_client_custom_character.data_cmd_id = Client_Draw_Character_Graph;
			judge_send_mesg.ext_client_custom_character.sender_ID = (uint16_t)current_robot_id;
			judge_send_mesg.ext_client_custom_character.receiver_ID = receiver_ID;
			
			if(chassis_mode == CHASSIS_DODGE_MODE && Chassis_mask != 3)//底盘模式
			{ 
				i++;
				client_graphic_draw_String("Chassis: DODGE",Chassis,Change,layer3,Pink,300,770);
				if(i>5) Chassis_mask = 3;
				Idle_mask = 0;
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_chassis,
												sizeof(judge_send_mesg.ext_client_custom_character_chassis), DN_REG_ID);
			}
			if(chassis_mode == CHASSIS_NORMAL_MODE && Chassis_mask != 1)
			{
				i++;
				client_graphic_draw_String("Chassis: NORMAL",Chassis,Change,layer3,Green,300,770);
				if(i>5) Chassis_mask = 1;	
				Idle_mask = 0;
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_chassis,
												sizeof(judge_send_mesg.ext_client_custom_character_chassis), DN_REG_ID);				
			}
			if(chassis_mode == CHASSIS_SEPARATE_MODE && Chassis_mask != 2) //分离
			{
				i++;
				client_graphic_draw_String("Chassis: SEPERATE",Chassis,Change,layer3,Redblue,300,770);
				if(i>5) Chassis_mask = 2;
				Idle_mask = 0;
				data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_chassis,
												sizeof(judge_send_mesg.ext_client_custom_character_chassis), DN_REG_ID);				
			}
			
			

			if(shoot.fric_wheel_run == 1 && shoot.shoot_boom_flag == 0 && Shoot_mask != 1)//发射模式
		 {
			i++;
			client_graphic_draw_String("FRICTION:ON",Shoot,Change,layer3,Cyan,300,700);
			if(i>5) Shoot_mask = 1;
			Idle_mask = 0;
			data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_shoot,
							sizeof(judge_send_mesg.ext_client_custom_character_shoot), DN_REG_ID);
		 }
		 if(shoot.fric_wheel_run == 1 && shoot.shoot_boom_flag == 1 && Shoot_mask != 2)
		 {
			i++;
			client_graphic_draw_String("FRICTION:ON BOOM",Shoot,Change,layer3,Amaranth,300,700);
			if(i>5) Shoot_mask = 2;
			Idle_mask = 0;
			data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_shoot,
							sizeof(judge_send_mesg.ext_client_custom_character_shoot), DN_REG_ID);
		 }   
		 if(shoot.fric_wheel_run == 0 && Shoot_mask != 3)
		 {
			i++;
			client_graphic_draw_String("FRICTION:OFF",Shoot,Change,layer3,Pink,300,700);
			if(i>5) Shoot_mask = 3; 
			Idle_mask = 0;
			data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character_shoot,
							sizeof(judge_send_mesg.ext_client_custom_character_shoot), DN_REG_ID);    
		 }

			if(Idle_mask == 1) //空闲时执行
			{
				i = 0;
				if(text_twist == 1) //实时显示电容
				{
					if(chassis.CapData[1]<16.0)
						client_grapjic_draw_float(chassis.cap_store,chassis.CapData[1],Cap,Change,layer2,Pink,1100,570);//NOTE:画浮点数!!!
					else if(chassis.CapData[1]>16.5)
						client_grapjic_draw_float(chassis.cap_store,chassis.CapData[1],Cap,Change,layer2,Yellow,1100,570);//!!!
					data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character,
													sizeof(judge_send_mesg.ext_client_custom_character), DN_REG_ID);					
				}

				if(text_twist == 2) //实时显示Pitch,Yaw
				{
					client_grapjic_draw_float(gimbal.sensor.yaw_relative_angle,gimbal.sensor.pit_relative_angle,Compensates,Change,layer2,Yellow,1700,890);			
					data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character,
													sizeof(judge_send_mesg.ext_client_custom_character), DN_REG_ID);					
				}
				if(text_twist == 3)//实时显示轻触开关
				{
					if(!shoot_ready)
						client_graphic_draw_String("Shoot_ready:OFF",View,Change,layer2,Pink,1100,467);
					else if(shoot_ready)
						client_graphic_draw_String("Shoot_ready:ON",View,Change,layer2,Green,1100,467);
					data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character,
													sizeof(judge_send_mesg.ext_client_custom_character), DN_REG_ID);					
				}
//				 if(text_twist == 4)//实时显示速度
//				 {
//				 	static float num=0;
//				 	if(chassis.vx)num=chassis.vx;
//					else num=chassis.vy;
//				 	client_grapjic_draw_float(num,chassis.vw,Speed,Change,layer2,Yellow,1500,890);
//				 	data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,(uint8_t *)&judge_send_mesg.ext_client_custom_character,
//				 									sizeof(judge_send_mesg.ext_client_custom_character), DN_REG_ID);
//				 }
			}

			Idle_mask = 1;			
		}
		
	}	
}

//Hero 运行3次 
static void client_graphic_draw_short_line1(void)//英雄准星,图层7
{
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[0].graphic_name[0] = line_1s;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[0].operate_tpye = Add;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[0].graphic_tpye = Straight_line;//第二下
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[0].layer = layer7;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[0].color = Yellow;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[0].start_angle = 0;	
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[0].end_angle = 0;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[0].width = 1;    //20240401
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[0].start_x = 895;//起始x
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[0].start_y = 475;//475;//起始y
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[0].radius = 0;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[0].end_x = 1025;//终点x
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[0].end_y = 475;//终点y475
	
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[1].graphic_name[0] = line_2s;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[1].operate_tpye = Add;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[1].graphic_tpye = Straight_line;//第一下
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[1].layer = layer7;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[1].color = Yellow;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[1].start_angle = 0;	
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[1].end_angle = 0;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[1].width = 1;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[1].start_x = 865;//20240401
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[1].start_y = 508;//505
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[1].radius = 0;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[1].end_x = 1055;//20240401
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[1].end_y = 508;//
	
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[2].graphic_name[0] = line_3s;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[2].operate_tpye = Add;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[2].graphic_tpye = Straight_line;//最顶
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[2].layer = layer7;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[2].color = Yellow;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[2].start_angle = 0;	
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[2].end_angle = 0;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[2].width = 1;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[2].start_x = 945;//20240401
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[2].start_y = 690;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[2].radius = 0;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[2].end_x = 977;//20240401
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[2].end_y = 690;
	
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[3].graphic_name[0] = line_4s;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[3].operate_tpye = Add;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[3].graphic_tpye = Straight_line;//竖线
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[3].layer = layer7;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[3].color = Yellow;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[3].start_angle = 0;	
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[3].end_angle = 0;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[3].width = 1;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[3].start_x = 962;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[3].start_y = 805;//中间竖线顶部原560
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[3].radius = 0;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[3].end_x = 962;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[3].end_y = 300;//竖线尾部
	
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[4].graphic_name[0] = line_5s;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[4].operate_tpye = Add;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[4].graphic_tpye = Straight_line;// 中线（水平线）
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[4].layer = layer7;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[4].color = Yellow;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[4].start_angle = 0;	
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[4].end_angle = 0;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[4].width = 1;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[4].start_x = 835;//20240401
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[4].start_y = 540;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[4].radius = 0;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[4].end_x = 1085;//20240401
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[4].end_y = 540;

	//倍镜																																
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[5].graphic_name[0] = line_6s;//第三上
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[5].operate_tpye = Add;		
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[5].graphic_tpye = Straight_line;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[5].layer = layer6;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[5].color = Amaranth;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[5].start_angle = 0;	
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[5].end_angle = 0;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[5].width = 2;
//	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[5].start_x = 932;//940
//	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[5].start_y = 630;//403;
//	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[5].radius = 0;
//	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[5].end_x = 995;//983
//	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[5].end_y = 630;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[5].start_x = 915;//940
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[5].start_y = 630;//403;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[5].radius = 0;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[5].end_x = 1005;//983
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[5].end_y = 630;
	//移动至中心横线
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[6].graphic_name[0] = line_7s;//第一上
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[6].operate_tpye = Add;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[6].graphic_tpye = Straight_line;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[6].layer = layer6;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[6].color = Cyan;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[6].start_angle = 0;	
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[6].end_angle = 0;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[6].width = 2;
//	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[6].start_x = 890;
//	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[6].start_y = 565; 
//	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[6].radius = 0;
//	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[6].end_x = 1030;
//	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[6].end_y = 565;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[6].start_x = 915;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[6].start_y = 565; 
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[6].radius = 0;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[6].end_x = 1005;
	judge_send_mesg.ext_client_custom_graphic_seven1.interaction_figure[6].end_y = 565;
}


//己方机器人通信(未用)
void judgement_client_packet_pack(uint8_t *p_data)
{
	uint8_t current_robot_id = 0;
	uint16_t receiver_ID = 0;
	
	current_robot_id = judge_recv_mesg.game_robot_state.robot_id;  //读取当前机器人的id
	//此处选择的receiver_ID可以自行选择
	switch(current_robot_id)
	{
		//red robot
		case STUDENT_RED_HERO_ID:
		{
			receiver_ID = STUDENT_RED_SENTRY_ID;
		}break;
		case STUDENT_RED_ENGINEER_ID:
		{
			receiver_ID = STUDENT_RED_SENTRY_ID;
		}break;
		case STUDENT_RED_AERIAL_ID:
		{
			receiver_ID = STUDENT_RED_SENTRY_ID;
		}break;
		case STUDENT_RED_SENTRY_ID:
		{
			receiver_ID = STUDENT_RED_SENTRY_ID;
		}break;
		case STUDENT_RED_INFANTRY3_ID:
		{
			receiver_ID = STUDENT_RED_SENTRY_ID;
		}break;
		case STUDENT_RED_INFANTRY4_ID:
		{
			receiver_ID = STUDENT_RED_SENTRY_ID;
		}break;
		case STUDENT_RED_INFANTRY5_ID:
		{
			receiver_ID = STUDENT_RED_SENTRY_ID;
		}break;
		
		//blue robot
		case STUDENT_BLUE_HERO_ID:
		{
			receiver_ID = STUDENT_BLUE_SENTRY_ID;
		}break;
		case STUDENT_BLUE_ENGINEER_ID:
		{
			receiver_ID = STUDENT_BLUE_SENTRY_ID;
		}break;
		case STUDENT_BLUE_AERIAL_ID:
		{
			receiver_ID = STUDENT_BLUE_SENTRY_ID;
		}break;
		case STUDENT_BLUE_SENTRY_ID:
		{
			receiver_ID = STUDENT_BLUE_SENTRY_ID;
		}break;
		
		case STUDENT_BLUE_INFANTRY3_ID:
		{
			receiver_ID = STUDENT_BLUE_SENTRY_ID;
		}break;	
		case STUDENT_BLUE_INFANTRY4_ID:
		{
			receiver_ID = STUDENT_BLUE_SENTRY_ID;
		}break;
		case STUDENT_BLUE_INFANTRY5_ID:
		{
			receiver_ID = STUDENT_BLUE_SENTRY_ID;
		}break;

	}

	judge_send_mesg.ext_student_interactive_data.data_cmd_id = RobotCommunication;
	judge_send_mesg.ext_student_interactive_data.sender_ID = (uint16_t)current_robot_id;
	judge_send_mesg.ext_student_interactive_data.receiver_ID = receiver_ID;
	//将自定义的数据复制到发送结构体中
	memcpy(&judge_send_mesg.ext_student_interactive_data.data[0], p_data,sizeof(judge_send_mesg.ext_student_interactive_data.data));
	//该函数的功能为将需要发送的数据打包，便于下一步通过串口3发送给裁判系统
	data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID, (uint8_t *)&judge_send_mesg.ext_student_interactive_data,
									 STUDENT_DATA_LENGTH, DN_REG_ID);
}

