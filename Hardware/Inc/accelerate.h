#ifndef __ACCELERATE_H
#define __ACCELERATE_H

/******************************************************************************/
#include "stm32l1xx_hal.h"
#include "main.h"

#include "public.h"

#include "usart.h"

/******************************************************************************/
#define ACCELERATE_UART 						(huart1)
#define ACCELERATE_UART_RX_BYTE_MAX				(30)		/* 加速度传感器串口最大接收字节长度 */
#define ACCELERATE_UART_DMA_RX_GL_FLAG			(DMA_FLAG_GL5)
#define ACCELERATE_PROTOCOL_HEAD				(0x55)

#define ACCELERATE_TYPE_DATE_MARK					(1 << 0)	/* 时间输出 */
#define ACCELERATE_TYPE_ACCELERATE_SPEED_MARK		(1 << 1)	/* 加速度输出 */
#define ACCELERATE_TYPE_ANGULAR_SPEED_MARK			(1 << 2)	/* 角速度输出 */
#define ACCELERATE_TYPE_ANGLE_MARK					(1 << 3)	/* 角度输出 */
#define ACCELERATE_TYPE_MAGNETIC_FIELD_MARK			(1 << 4)	/* 磁场输出 */
#define ACCELERATE_TYPE_PORT_STATUS_MARK			(1 << 5)	/* 端口状态数据输出 */
#define ACCELERATE_TYPE_AIR_PRESS_AND_ALTITUDE_MARK	(1 << 6)	/* 气压、高度输出 */
#define ACCELERATE_TYPE_LONGITUDE_AND_LATITUDE_MARK	(1 << 7)	/* 经纬度输出 */
#define ACCELERATE_TYPE_GROUND_SPEED_MARK			(1 << 0)	/* 地速输出 */
#define ACCELERATE_TYPE_FOUR_ELEMENTS_MARK			(1 << 1)	/* 四元素输出 */
#define ACCELERATE_TYPE_LOCATION_MARK				(1 << 2)	/* 卫星定位精度输出 */

#define ACCELERATE_INTEGRAL_TIME				(0.1)		/* 积分时间为100ms */

#define ACCELERATE_DIGITAL_RANGE		(double)(32768)	  /* 数字范围，强转double */
#define ACCELERATE_RANGE_ACCELERATE		(double)(16 * 9.8) /* 加速度范围 */
#define ACCELERATE_RANGE_ANGLE			(double)(180.0)    /* 角度范围 */

/******************************************************************************/
typedef enum
{
	ACCELERATE_TYPE_DATE = 0x50,				/* 时间输出 */
	ACCELERATE_TYPE_ACCELERATE_SPEED,			/* 加速度输出 */
	ACCELERATE_TYPE_ANGULAR_SPEED,				/* 角速度输出 */
	ACCELERATE_TYPE_ANGLE,						/* 角度输出 */
	ACCELERATE_TYPE_MAGNETIC_FIELD,				/* 磁场输出 */
	ACCELERATE_TYPE_PORT_STATUS,				/* 端口状态数据输出 */
	ACCELERATE_TYPE_AIR_PRESS_AND_ALTITUDE,		/* 气压、高度输出 */
	ACCELERATE_TYPE_LONGITUDE_AND_LATITUDE,		/* 经纬度输出 */
	ACCELERATE_TYPE_GROUND_SPEED,				/* 地速输出 */
	ACCELERATE_TYPE_FOUR_ELEMENTS,				/* 四元素输出 */
	ACCELERATE_TYPE_LOCATION,					/* 卫星定位精度输出 */
} ACCELERATE_TypeEnum;

typedef enum
{
	ACCELERATE_ADDR_SAVE,						/* 保存当前配置 */
	ACCELERATE_ADDR_CALSW,						/* 校准 */
	ACCELERATE_ADDR_RSW,						/* 回传数据内容 */
	ACCELERATE_ADDR_RATE,						/* 回传数据速率 */
	ACCELERATE_ADDR_BAUD,						/* 串口波特率 */
	ACCELERATE_ADDR_AXOFFSET,					/* X轴加速度零偏 */
	ACCELERATE_ADDR_AYOFFSET,					/* Y轴加速度零偏 */
	ACCELERATE_ADDR_AZOFFSET,					/* Z轴加速度零偏 */
	ACCELERATE_ADDR_GXOFFSET,					/* X轴角速度零偏 */
	ACCELERATE_ADDR_GYOFFSET,					/* Y轴角速度零偏 */
	ACCELERATE_ADDR_GZOFFSET,					/* Z轴角速度零偏 */
	ACCELERATE_ADDR_HXOFFSET,					/* X轴磁场零偏 */
	ACCELERATE_ADDR_HYOFFSET,					/* Y轴磁场零偏 */
	ACCELERATE_ADDR_HZOFFSET,					/* Z轴磁场零偏 */
	ACCELERATE_ADDR_D0MODE,						/* D0模式 */
	ACCELERATE_ADDR_D1MODE,						/* D1模式 */
	ACCELERATE_ADDR_D2MODE,						/* D2模式 */
	ACCELERATE_ADDR_D3MODE,						/* D3模式 */
	ACCELERATE_ADDR_D0PWMH,						/* D0PWM高电平宽度 */
	ACCELERATE_ADDR_D1PWMH,						/* D1PWM高电平宽度 */
	ACCELERATE_ADDR_D2PWMH,						/* D2PWM高电平宽度 */
	ACCELERATE_ADDR_D3PWMH,						/* D3PWM高电平宽度 */
	ACCELERATE_ADDR_D0PWMT,						/* D0PWM周期 */
	ACCELERATE_ADDR_D1PWMT,						/* D1PWM周期 */
	ACCELERATE_ADDR_D2PWMT,						/* D2PWM周期 */
	ACCELERATE_ADDR_D3PWMT,						/* D3PWM周期 */
	ACCELERATE_ADDR_IICADDR,					/* IIC地址 */
	ACCELERATE_ADDR_LEDOFF,						/* 关闭LED指示灯 */
	ACCELERATE_ADDR_GPSBAUD,					/* GPS连接波特率 */

	/* IIC通讯地址 */
	ACCELERATE_ADDR_YYMM = 0x30,				/* 年、月 */
	ACCELERATE_ADDR_DDHH,						/* 日、时 */
	ACCELERATE_ADDR_MMSS,						/* 分、秒 */
	ACCELERATE_ADDR_MS,							/* 毫秒 */
	ACCELERATE_ADDR_AX,
	ACCELERATE_ADDR_AY,
	ACCELERATE_ADDR_AZ,
	ACCELERATE_ADDR_GX,
	ACCELERATE_ADDR_GY,
	ACCELERATE_ADDR_GZ,
	ACCELERATE_ADDR_HX,
	ACCELERATE_ADDR_HY,
	ACCELERATE_ADDR_HZ,
	ACCELERATE_ADDR_ROLL,
	ACCELERATE_ADDR_PITCH,
	ACCELERATE_ADDR_YAW,
	ACCELERATE_ADDR_TEMP,
	ACCELERATE_ADDR_D0STATUS,
	ACCELERATE_ADDR_D1STATUS,
	ACCELERATE_ADDR_D2STATUS,
	ACCELERATE_ADDR_D3STATUS,
	ACCELERATE_ADDR_PRESSUREL,
	ACCELERATE_ADDR_PRESSUREH,
	ACCELERATE_ADDR_HIGHTL,
	ACCELERATE_ADDR_HIGHTH,
	ACCELERATE_ADDR_LONL,
	ACCELERATE_ADDR_LONH,
	ACCELERATE_ADDR_LATL,
	ACCELERATE_ADDR_LATH,
	ACCELERATE_ADDR_GPS_HEIGHT,
	ACCELERATE_ADDR_GPS_YAW,
	ACCELERATE_ADDR_GPS_VL,
	ACCELERATE_ADDR_GPS_VH,
	ACCELERATE_ADDR_Q0,
	ACCELERATE_ADDR_Q1,
	ACCELERATE_ADDR_Q2,
	ACCELERATE_ADDR_Q3,
}ACCELERATE_AddrEnum;

/******************************************************************************/
#pragma pack(1)
typedef struct
{
	uint8_t head;								/* 包头 */
	uint8_t type;								/* 数据类型 */
	int16_t data1;								/* 数据1 */
	int16_t data2;								/* 数据2 */
	int16_t data3;								/* 数据3 */
	int16_t data4;								/* 数据4 */
	uint8_t sum;								/* 校验和 */
} ACCELERATE_RecvStrcutTypedef;					/* 接收结构体 */

typedef struct
{
	uint8_t head1;								/* 包头1 */
	uint8_t head2;								/* 包头2 */
	uint8_t address;							/* 地址 */
	uint8_t dataL;								/* 数据L */
	uint8_t dataH;								/* 数据H */
} ACCELERATE_SendStrcutTypedef;					/* 发送结构体 */

typedef struct
{
	ACCELERATE_RecvStrcutTypedef buffer;		/* 接收二级缓存 */
	uint8_t size;								/* 接收数据的长度 */
	FunctionalState status;						/* 接收状态 */
} ACCELERATE_RecvTypedef;

#pragma pack()
/******************************************************************************/
void ACCELERATE_Init(void);
void ACCELERATE_Process(void);
void ACCELERATE_UartIdleDeal(void);
void ACCELERATE_SetBackInfo(uint8_t RSWL, uint8_t RSWH);

#endif
