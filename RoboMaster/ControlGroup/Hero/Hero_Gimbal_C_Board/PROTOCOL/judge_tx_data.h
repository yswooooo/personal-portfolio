#ifndef _judge_tx_data_H
#define _judge_tx_data_H


#include "stm32f4xx.h"
#include "data_packet.h"

#define JUDGE_TX_FIFO_BUFLEN 500

#define STUDENT_DATA_LENGTH   sizeof(robot_interactive_data_t)
#define CLIENT_DATA_LENGTH   	sizeof(client_custom_data_t) 

//学生交互id
typedef enum
{
	//学生交互机器人id
	STUDENT_RED_HERO_ID       = 1,
	STUDENT_RED_ENGINEER_ID	  = 2,
	STUDENT_RED_INFANTRY3_ID  = 3,
	STUDENT_RED_INFANTRY4_ID  = 4,
	STUDENT_RED_INFANTRY5_ID  = 5,
	STUDENT_RED_AERIAL_ID	    = 6,
	STUDENT_RED_SENTRY_ID	    = 7,
	STUDENT_BLUE_HERO_ID	    = 11,
	STUDENT_BLUE_ENGINEER_ID  = 12,
	STUDENT_BLUE_INFANTRY3_ID = 13,
	STUDENT_BLUE_INFANTRY4_ID = 14,
	STUDENT_BLUE_INFANTRY5_ID = 15,
	STUDENT_BLUE_AERIAL_ID    = 16,
	STUDENT_BLUE_SENTRY_ID	  = 17,
	
	//机器人对应客户端id
	RED_HERO_CLIENT_ID	      = 0x0101,
	RED_ENGINEER_CLIENT_ID    = 0x0102,
	RED_INFANTRY3_CLIENT_ID	  = 0x0103,
	RED_INFANTRY4_CLIENT_ID	  = 0x0104,
	RED_INFANTRY5_CLIENT_ID   = 0x0105,
	RED_AERIAL_CLIENT_ID      = 0x0106,
	BLUE_HERO_CLIENT_ID	      = 0x0111,
	BLUE_ENGINEER_CLIENT_ID   = 0x0112,
	BLUE_INFANTRY3_CLIENT_ID  = 0x0113,
	BLUE_INFANTRY4_CLIENT_ID  = 0x0114,
	BLUE_INFANTRY5_CLIENT_ID  = 0x0115,
	BLUE_AERIAL_CLIENT_ID	    = 0x0116,
	
	//自定义数据id
	CLIENT_DATA_ID	          = 0xd180,
	CLIENT_PATTERN_ID					= 0x0100,		//客户端自定义图案
	
}interactive_id_e;

typedef enum
{
	ROUND										= 0x01,			//自定义数据名称
	LINK1										= 0x02,			//3米射击
	LINK2										= 0x03,			//5米射击
	LINK3										= 0x04,			//7米射击
	LINK4										= 0x05,
	LINK5										= 0x06,
	LINK6										= 0x07,
	LINK7										= 0x08,
	LINK8										= 0x09,
	TXT											= 0X10,
}client_graphic_draw_name_e;

typedef enum
{
	NULL_GRAPHICS							= 0x00,			//空
	ADD_GRAPHICS							= 0x01,			//增加图案
	REMOVE_GRAPHICS						= 0x02,			//修改图案
	MODIFY_GRAPHICS						= 0x04,			//删除一个图案
	DELETE_LAYER_GRAPHICS     = 0x05,			//删除一个图层
	DELETA_ALL_GRAOGICS				= 0x06,			//删除所有图案
}client_graphic_draw_operate_tpye_e;

typedef enum
{
	NULL_FORM									= 0x00,			//空
	STAIGHT_LINE							= 0x01,			//直线
	RECTANGLE									= 0x02,			//矩形
	ROUNDNESS									= 0x03,			//圆
	ELLIPSE										= 0x04,			//椭圆
	ARC										    = 0x05,			//弧形
	TEXT											= 0x06,			//文本，ASCII字码
}client_graphic_draw_graphic_tpye_e;


typedef enum
{
	RED_BULE									= 0x00,	//自方颜色
	YELLOW										= 0x01,	//黄色
	GREEN											= 0x02,	//绿色
	ORANGE										= 0x03,	//橙色
	AMARANTH							    = 0x04, //紫红色
	PINK											= 0x05,	//粉色
	CYAN											=	0x06,	//青色
	BLANK											=	0x07,	//黑色
	WHITE											= 0x08, //白色
}client_graphic_draw_color_e;
/* 云台和底盘相对角度
 *
 *
*/
typedef enum
{	//mask 从左到右 从低到高 和 uint8 存储相反
	NAGETIVE_30		=	0x04,		//0000 0100
	NAGETIVE_60		=	0x06,		//0000 0110
	NAGETIVE_90		=	0x07,		//0000 0111
	NAGETIVE_120	=	0x27,		//0010 0111
	NAGETIVE_150	=	0X37,		//0011 0111
	NAGETIVE_180	= 0x3f,		//0011 1111
	
	POSITIVE_180	= 0x3f,		//0011 1111
	POSITIVE_150	= 0X3b,		//0011 1011
	POSITIVE_120	= 0x39,		//0011 1001
	POSITIVE_90		= 0x38,		//0011 1000
	POSITIVE_60		= 0x18,		//0001 1000
	POSITIVE_30		= 0x08		//0000 1000
	
} yaw_relative_angle_2masks;

typedef __packed struct
{
 uint16_t data_cmd_id;
 uint16_t send_ID;
 uint16_t receiver_ID;
	
 uint8_t interactive_data[50];
}ext_TX_student_interactive_header_data_t;

typedef __packed struct
{
 float data1;
 float data2;
 float data3;
uint8_t masks;			//从左到右 低位到高位
} client_custom_data_t;

typedef __packed struct
{
 uint8_t data[50];//113
} robot_interactive_data_t;

typedef __packed struct
	{ 
		uint8_t operate_tpye;  		//图形操作
		uint8_t graphic_tpye; 		//图形类型
		uint8_t graphic_name[5];  //图形名
		uint8_t layer; 						//图层
		uint8_t color;  					//颜色	
		uint8_t width;						//线宽
		uint16_t start_x; 				//起始X轴坐标
		uint16_t start_y; 				//起始Y轴坐标
		uint16_t radius;  				//字体大小或者半径
		uint16_t end_x;  					//终点X轴坐标
		uint16_t end_y; 					//终点Y轴坐标
		int16_t start_angle; 			//起始角度	圆弧的起始角度， 圆弧顺时针绘制，单位为度，范围[-180,180
		int16_t end_angle; 				//终止角度	圆弧的终止角度，单位为度，范围[-180,180]
		uint8_t text_lenght; 			//文本长度
		uint8_t text[30]; 				//文本字符
	} ext_client_graphic_draw_t ;
/** 
  * @brief  the data structure receive from judgement
  */
typedef struct
{
  ext_TX_student_interactive_header_data_t	  student_interactive_header_data; //0x0301
  client_custom_data_t					              client_custom_data;				       //0xD180
  robot_interactive_data_t				            robot_interactive_data;			     //0x0200~ox02FF
	ext_client_graphic_draw_t								    client_graphic_draw;							//0x0100
} judge_txdata_t;

typedef struct
{
	uint8_t		robot_id;
	uint8_t		robot_level;
	float			bullet_num;			//弹仓剩余子弹
}user_judge_t;

extern judge_txdata_t judge_send_mesg;
extern fifo_s_t  judge_txdata_fifo;

void judgement_tx_param_init(void);
void judgement_client_packet_pack(client_custom_data_t *p_data);
void judgement_client_graphics_draw_pack(void);

#endif 

