#ifndef __PROCESS_H
#define __PROCESS_H

/******************************************************************************/
#include "stm32l1xx_hal.h"
#include "main.h"

#include "accelerate.h"
#include "noise.h"
#include "pressure.h"

/******************************************************************************/
typedef enum
{
	PROCESS_MODE_INVALID,							/* 无效模式 */
	PROCESS_MODE_DETECTED_STEERING_WHEEL_FORCE,		/* 方向盘转向力检测 */
	PROCESS_MODE_DETECTED_STEERING_WHEEL_ANGLE,		/* 方向盘转角检测 */
	PROCESS_MODE_DETECTED_BRAKING_DISTANCE,			/* 制动距离检测 */
	PROCESS_MODE_DETECTED_PEDAL_FORCE,				/* 制动踏板力检测 */
	PROCESS_MODE_DETECTED_HAND_BRAKE_FORCE,			/* 手刹制动力检测 */
	PROCESS_MODE_DETECTED_NOISE,					/* 喇叭检测 */
	PROCESS_MODE_DETECTED_SIDESLIP_DISTANCE,		/* 侧滑量检测 */
	PROCESS_MODE_DETECTED_DOWN_VELOCITY,			/* 货叉下降速度检测 */
} PROCESS_ModeEnum;

/******************************************************************************/
extern PROCESS_ModeEnum PROCESS_Mode;

/******************************************************************************/
void PROCESS(void);

#endif
