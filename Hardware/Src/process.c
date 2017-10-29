#include "process.h"

/******************************************************************************/
PROCESS_ModeEnum PROCESS_Mode = PROCESS_MODE_INVALID;

/*******************************************************************************
 *
 */
void PROCESS(void)
{
	switch (PROCESS_Mode)
	{
	/* 方向盘转向力检测 */
	case PROCESS_MODE_DETECTED_STEERING_WHEEL_FORCE:
		/* 获取转向力 */
		PRESSURE_GetSteeringWheelForce();
		break;

	/* 方向盘转角检测 */
	case PROCESS_MODE_DETECTED_STEERING_WHEEL_ANGLE:
		ACCELERATE_Process();
		break;

	/* 制动距离检测 */
	case PROCESS_MODE_DETECTED_BRAKING_DISTANCE:
		ACCELERATE_Process();
		break;

	/* 制动踏板力检测 */
	case PROCESS_MODE_DETECTED_PEDAL_FORCE:
		/* 获取踏板力 */
		PRESSURE_GetPedalForce();
		break;

	/* 手刹制动力检测 */
	case PROCESS_MODE_DETECTED_HAND_BRAKE_FORCE:
		/* 获取手刹力 */
		PRESSURE_GetHandBrakeForce();
		break;

	/* 喇叭检测 */
	case PROCESS_MODE_DETECTED_NOISE:
		/* 噪声数据请求 */
		NOISE_Require();
		break;

	/* 侧滑量检测 */
	case PROCESS_MODE_DETECTED_SIDESLIP_DISTANCE:
		break;

	/* 货叉下降速度检测 */
	case PROCESS_MODE_DETECTED_DOWN_VELOCITY:
		break;

	/* 锂电池电量检测 */
	case PROCESS_MODE_DETECTED_BAT_VOLTAGE:
		ANALOG_Process();
		break;

	default:
		break;
	}
}



















