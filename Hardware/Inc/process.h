#ifndef __PROCESS_H
#define __PROCESS_H

/******************************************************************************/
#include "stm32l1xx_hal.h"
#include "main.h"

#include "accelerate.h"
#include "noise.h"
#include "pressure.h"
#include "analog.h"

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
	PROCESS_MODE_DETECTED_BATTERY_CAPACITY,			/* 锂电池电量检测 */
} PROCESS_ModeEnum;

typedef struct
{
	double steeringWheelForce;						/* 方向盘转向力 */
	double steeringWheelAngle;						/* 方向盘转角 */
	double Ax;										/* 加速度 */
	double brakeVelocity;							/* 制动速度 */
	double speed;									/* 制动速度（单位km/h） */
	double brakeVelocityInit;						/* 制动初速度 */
	double brakeDistance;							/* 制动距离 */
	double pedalForce;								/* 踏板力 */
	double handBrakeForce;							/* 手刹力 */
	float  noise;									/* 噪声值 */
	double downAx;									/* 下降加速度 */
	double downVelocity;							/* 下降速度 */
	uint8_t batteryCapacity;						/* 电池电量 */
} ItemValueTypedef;

typedef struct
{
	FunctionalState steeringWheelForce;						/* 方向盘转向力 */
	FunctionalState steeringWheelAngle;						/* 方向盘转角 */
	FunctionalState brakeAx;								/* 加速度 */
	FunctionalState pedalForce;								/* 踏板力 */
	FunctionalState handBrakeForce;							/* 手刹力 */
	FunctionalState noise;									/* 噪声值 */
	FunctionalState downVelocity;							/* 下降速度 */
	FunctionalState batteryCapacity;						/* 电池电量 */
} ItemValueSetZeroEnableTypedef;

typedef struct
{
	double Ax;										/* 加速度零点值 */
	double steeringWheelForce;						/* 方向盘转向力零点值 */
	double steeringWheelAngle;						/* 方向盘转角零点值 */
	double pedalForce;								/* 踏板力零点值 */
	double handBrakeForce;							/* 手刹力零点值 */
	float  noise;									/* 噪声零点值 */
	double downAx;									/* 下降加速度 */
} ItemZeroValueTypedef;

/******************************************************************************/
extern PROCESS_ModeEnum PROCESS_Mode;

/******************************************************************************/
void PROCESS(void);
void ZeroCalibration(void);

#endif
