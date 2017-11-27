#ifndef __ULTRASONICWAVE_H
#define __ULTRASONICWAVE_H

/******************************************************************************/
#include "stm32l1xx_hal.h"
#include "main.h"
#include "public.h"

/******************************************************************************/
#define ULTRASONICWAVE_HEAD					(0xFF)
#define ULTRASONICWAVE_TIME_PERIOD			(0.1)	/* 超声波获取周期，单位s */

/******************************************************************************/
typedef struct
{
	uint8_t head;						/* 帧头 */
	uint8_t dataH;						/* 数据 */
	uint8_t dataL;
	uint8_t checkSum;					/* 校验和 */
} ULTRASONICWAVE_RecvStrcutTypedef;

typedef struct
{
	ULTRASONICWAVE_RecvStrcutTypedef buffer;
	uint8_t size;										/* 接收数据的长度 */
	FunctionalState status;								/* 接收状态 */
} ULTRASONICWAVE_RecvTypedef;

#endif
