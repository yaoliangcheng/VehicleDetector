#ifndef __BLE_H
#define __BLE_H

/******************************************************************************/
#include "stm32l1xx_hal.h"
#include "main.h"

#include "accelerate.h"
#include "process.h"

/******************************************************************************/
#define BLE_UART 						(huart3)
#define BLE_UART_RX_BYTE_MAX			(30)		/* 加速度传感器串口最大接收字节长度 */
#define BLE_UART_DMA_RX_GL_FLAG			(DMA_FLAG_GL3)

/******************************************************************************/
typedef enum
{
	/* 设备整体操作 */
	BLE_CMD_TYPE_CLEAR_SENSOR_CACHE = 0x00,				/* 清除传感器缓存 */
	BLE_CMD_TYPE_SWITCH_OFF,							/* 断开蓝牙 */
	BLE_CMD_TYPE_STOP_DETECTED,							/* 停止检测 */

	/* 方向盘转向力 */
	BLE_CMD_TYPE_STEERING_WHEEL_FORCE_SET_ZERO = 0x10,	/* 传感器零位校正 */
	BLE_CMD_TYPE_DETECTED_STEERING_WHEEL_FORCE,			/* 开启方向盘转向力检测 */
	BLE_DATA_TYPE_STEERING_WHEEL_FORCE = 0x18,			/* 实时转向力值 */
	BLE_DATA_TYPE_STEERING_WHEEL_FORCE_MAX,				/* 最大转向力值 */

	/* 方向盘转角 */
	BLE_CMD_TYPE_STEERING_WHEEL_ANGLE_SET_ZERO = 0x20,	/* 传感器零位校正 */
	BLE_CMD_TYPE_DETECTED_STEERING_WHEEL_ANGLE,			/* 开启方向盘转角检测 */
	BLE_DATA_TYPE_STEERING_WHEEL_ANGLE = 0x28,			/* 实时转向角度 */
	BLE_DATA_TYPE_STEERING_WHEEL_ANGLE_MAX,				/* 最大转向角度 */

	/* 制动距离 */
	BLE_CMD_TYPE_BRAKING_DISTANCE_SET_ZERO = 0x30,		/* 传感器零位校正 */
	BLE_CMD_TYPE_DETECTED_BRAKING_DISTANCE,				/* 开启制动距离检测 */
	BLE_DATA_TYPE_BRAKING_INITIAL_VELOCITY = 0x38,		/* 初速度值 */
	BLE_DATA_TYPE_BRAKING_ACCELERATE_AVERAGE,			/* 平均减速度值 */
	BLE_DATA_TYPE_BRAKING_DISTANCE,						/* 制动距离 */

	/* 踏板力 */
	BLE_CMD_TYPE_PEDAL_FORCE_SET_ZERO = 0x40,			/* 传感器零位校正 */
	BLE_CMD_TYPE_DETECTED_PEDAL_FORCE,					/* 开启制动踏板力检测 */
	BLE_DATA_TYPE_PEDAL_FORCE = 0x48,					/* 实时踏板力值 */
	BLE_DATA_TYPE_PEDAL_FORCE_MAX,						/* 最大踏板力值 */

	/* 手刹力 */
	BLE_CMD_TYPE_HAND_BRAKE_FORCE_SET_ZERO = 0x50,		/* 传感器零位校正 */
	BLE_CMD_TYPE_DETECTED_HAND_BRAKE_FORCE,				/* 开启手刹制动力检测 */
	BLE_DATA_TYPE_HAND_BRAKE_FORCE = 0x58,				/* 实时手刹力值 */
	BLE_DATA_TYPE_HAND_BRAKE_FORCE_MAX,					/* 最大手刹力值 */

	/* 喇叭 */
	BLE_CMD_TYPE_NOISE_SET_ZERO = 0x60,					/* 传感器零位校正 */
	BLE_CMD_TYPE_DETECTED_NOISE,						/* 开启喇叭检测 */
	BLE_DATA_TYPE_NOISE = 0x68,							/* 实时噪音值 */
	BLE_DATA_TYPE_NOISE_MAX,							/* 最大噪音值 */

	/* 侧滑力 */
	BLE_CMD_TYPE_SIDESLIP_DISTANCE_SET_ZERO = 0x70,		/* 传感器零位校正 */
	BLE_CMD_TYPE_DETECTED_SIDESLIP_DISTANCE,			/* 开启侧滑量检测 */
	BLE_DATA_TYPE_SIDESLIP_FORCE = 0x78,				/* 实时侧滑力值 */
	BLE_DATA_TYPE_SIDESLIP_DISTANCE_MAX,				/* 最大侧滑量值 */

	/* 下降速度 */
	BLE_CMD_TYPE_DOWN_VELOCITY_SET_ZERO = 0x80,			/* 传感器零位校正 */
	BLE_CMD_TYPE_DETECTED_DOWN_VELOCITY,				/* 开启货叉下降速度检测 */
	BLE_DATA_TYPE_DOWN_DISTANCE = 0x88,					/* 货叉离地距离 */
	BLE_DATA_TYPE_DOWN_VELOCITY,						/* 实时货叉下降速度 */
	BLE_DATA_TYPE_DOWN_VELOCITY_MAX,					/* 货叉最大下降速度 */
} BLE_CmdDataTypeEnum;



/******************************************************************************/
#pragma pack(1)
typedef struct
{
	uint8_t buffer[BLE_UART_RX_BYTE_MAX];				/* 接收二级缓存 */
//	ACCELERATE_RecvStrcutTypedef buffer;
	uint8_t size;										/* 接收数据的长度 */
	FunctionalState status;								/* 接收状态 */
} BLE_RecvTypedef;



#pragma pack()

/******************************************************************************/
void BLE_Init(void);
void BLE_UartIdleDeal(void);
void BLE_Process(void);

#endif
