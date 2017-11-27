#ifndef __BLE_H
#define __BLE_H

/******************************************************************************/
#include "stm32l1xx_hal.h"
#include "main.h"

#include "accelerate.h"
#include "process.h"
#include "analog.h"

/******************************************************************************/
#define BLE_UART 						(huart3)
#define BLE_UART_RX_BYTE_MAX			(30)		/* 加速度传感器串口最大接收字节长度 */
#define BLE_UART_DMA_RX_GL_FLAG			(DMA_FLAG_GL3)

#define BLE_PROTOCOL_HEAD				(0xAA)
#define BLE_PROTOCOL_TAIL				(0x55)
/******************************************************************************/
typedef enum
{
	/* 设备整体操作 */
	BLE_CMD_TYPE_SENSOR_VALUE_SET_ZERO = 0x00,			/* 传感器零位校正 */
	BLE_CMD_TYPE_CLEAR_SENSOR_CACHE,					/* 清除传感器缓存 */
	BLE_CMD_TYPE_SWITCH_OFF,							/* 断开蓝牙 */
	BLE_CMD_TYPE_STOP_DETECTED,							/* 停止检测 */
	BLE_CMD_TYPE_DETECTED_STEERING_WHEEL_FORCE,			/* 开启方向盘转向力检测 */
	BLE_CMD_TYPE_DETECTED_STEERING_WHEEL_ANGLE,			/* 开启方向盘转角检测 */
	BLE_CMD_TYPE_DETECTED_BRAKING_DISTANCE,				/* 开启制动距离检测 */
	BLE_CMD_TYPE_DETECTED_PEDAL_FORCE,					/* 开启制动踏板力检测 */
	BLE_CMD_TYPE_DETECTED_HAND_BRAKE_FORCE,				/* 开启手刹制动力检测 */
	BLE_CMD_TYPE_DETECTED_NOISE,						/* 开启喇叭检测 */
	BLE_CMD_TYPE_DETECTED_SIDESLIP_DISTANCE,			/* 开启侧滑量检测 */
	BLE_CMD_TYPE_DETECTED_DOWN_VELOCITY,				/* 开启货叉下降速度检测 */
	BLE_CMD_TYPE_DETECTED_GRADIENT,						/* 坡度检测 */
	BLE_CMD_TYPE_DETECTED_BATTERY_CAPACITY,				/* 开启电池电量检测 */
	
	/* 方向盘转向力 */
	BLE_DATA_TYPE_STEERING_WHEEL_FORCE = 0x10,			/* 实时转向力值 */
	BLE_DATA_TYPE_STEERING_WHEEL_FORCE_MAX,				/* 最大转向力值 */

	/* 方向盘转角 */
	BLE_DATA_TYPE_STEERING_WHEEL_ANGLE = 0x20,			/* 实时转向角度 */
	BLE_DATA_TYPE_STEERING_WHEEL_ANGLE_MAX,				/* 最大转向角度 */

	/* 制动距离 */
	BLE_DATA_TYPE_BRAKING_INITIAL_VELOCITY = 0x30,		/* 初速度值 */
	BLE_DATA_TYPE_BRAKING_ACCELERATE_AVERAGE,			/* 平均减速度值 */
	BLE_DATA_TYPE_BRAKING_DISTANCE,						/* 制动距离 */

	/* 踏板力 */
	BLE_DATA_TYPE_PEDAL_FORCE = 0x40,					/* 实时踏板力值 */
	BLE_DATA_TYPE_PEDAL_FORCE_MAX,						/* 最大踏板力值 */

	/* 手刹力 */
	BLE_DATA_TYPE_HAND_BRAKE_FORCE = 0x50,				/* 实时手刹力值 */
	BLE_DATA_TYPE_HAND_BRAKE_FORCE_MAX,					/* 最大手刹力值 */

	/* 喇叭 */
	BLE_DATA_TYPE_NOISE = 0x60,							/* 实时噪音值 */
	BLE_DATA_TYPE_NOISE_MAX,							/* 最大噪音值 */

	/* 侧滑力 */
	BLE_DATA_TYPE_SIDESLIP_FORCE = 0x70,				/* 实时侧滑力值 */
	BLE_DATA_TYPE_SIDESLIP_DISTANCE_MAX,				/* 最大侧滑量值 */

	/* 下降速度 */
	BLE_DATA_TYPE_DOWN_DISTANCE = 0x80,					/* 货叉离地距离 */
	BLE_DATA_TYPE_DOWN_VELOCITY,						/* 实时货叉下降速度 */
	BLE_DATA_TYPE_DOWN_VELOCITY_MAX,					/* 货叉最大下降速度 */

	/* 坡度检测 */
	BLE_DATA_TYPE_GRADIENT = 0x90,						/* 坡度值 */

	/* 锂电池电量 */
	BLE_DATA_TYPE_BATTERY_CAPACITY = 0XA0,				/* 电池电量 */
} BLE_CmdDataTypeEnum;

typedef enum
{
	BLE_CMD_SUBTYPE_SET_ZORO_STEERING_WHEEL_FORCE = 0x01, /* 转向力置零 */
	BLE_CMD_SUBTYPE_SET_ZORO_STEERING_WHEEL_ANGLE,		  /* 转角置零 */
	BLE_CMD_SUBTYPE_SET_ZORO_BRAKING_DISTANCE,			  /* 制动距离置零 */
	BLE_CMD_SUBTYPE_SET_ZORO_PEDAL_FORCE,				  /* 踏板力置零 */
	BLE_CMD_SUBTYPE_SET_ZORO_HAND_BRAKE_FORCE,			  /* 手刹力置零 */
	BLE_CMD_SUBTYPE_SET_ZORO_NOISE,						  /* 噪声置零 */
	BLE_CMD_SUBTYPE_SET_ZORO_SIDESLIP_FORCE,			  /* 侧滑力置零 */
	BLE_CMD_SUBTYPE_SET_ZORO_DOWN_VELOCITY,				  /* 下降速度置零 */
	BLE_CMD_SUBTYPE_SET_ZERO_GRADIENT,					  /* 坡度置零 */
} BLE_CmdSubtypeEnum;

/******************************************************************************/
#pragma pack(1)

typedef struct
{
	uint8_t head;						/* 帧头 */
	uint8_t cmdType;					/* 数据类型 */
	uint8_t cmdSubType;					/* 数据子类型 */
	uint8_t verify;						/* 校验 */
	uint8_t tail;						/* 帧尾 */
} BLE_RecvStruct;

typedef struct
{
	BLE_RecvStruct buffer;				/* 接收二级缓存 */
	uint8_t size;						/* 接收数据的长度 */
	FunctionalState status;				/* 接收状态 */
} BLE_RecvTypedef;

typedef struct
{
	uint8_t head;						/* 帧头 */
	uint8_t type;						/* 数据类型 */
	uint8_t length;						/* 数据长度 */
	char    data[6];					/* 数据（ASCII） 数据范围-999.9~9999.9*/
	uint8_t verify;						/* 校验和 */
	uint8_t tail;						/* 帧尾 */
} BLE_SendStructTypedef;				/* 蓝牙发送结构体 */

#pragma pack()

/******************************************************************************/
void BLE_Init(void);
void BLE_UartIdleDeal(void);
void BLE_Process(void);
void BLE_SendBytes(BLE_CmdDataTypeEnum type, char* value);

#endif
