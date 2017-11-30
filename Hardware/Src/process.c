#include "process.h"
#include "ultrasonicwave.h"
/******************************************************************************/
PROCESS_ModeEnum PROCESS_Mode = PROCESS_MODE_INVALID;
ItemValueTypedef     ItemValue;
ItemZeroValueTypedef ItemZeroValue;
ItemValueSetZeroEnableTypedef ItemValueSetZeroEnable;


/* 制动距离检测 */
double BrakeDistance_speed;
double BrakeDistance_distance;
double BrakeDistance_brakeDistance;

/* 货叉下降速度检测 */
uint16_t DownVelocity_Distance;
uint16_t DownVelocity_DistanceOld;
double   DownVelocity_Speed;
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
		ACCELERATE_Process();
		break;

	/* 货叉下降速度检测 */
	case PROCESS_MODE_DETECTED_DOWN_VELOCITY:
		ULTRASONICWAVE_Process();
		break;

	/* 坡度检测 */
	case PROCESS_MODE_DETECTED_GRADIENT:
		ACCELERATE_Process();
		break;

	/* 锂电池电量检测 */
//	case PROCESS_MODE_DETECTED_BATTERY_CAPACITY:
//		ANALOG_Process();
//		/* 锂电池电量检测，只检测一次 */
//		PROCESS_Mode = PROCESS_MODE_INVALID;
//		break;

	default:
		break;
	}
}

/*******************************************************************************
 * @breif 根据不同模式，选择当前模式下的当前状态为该零点值
 */
void ZeroCalibration(void)
{
	switch (PROCESS_Mode)
	{
	/* 方向盘转向力检测 */
	case PROCESS_MODE_DETECTED_STEERING_WHEEL_FORCE:
		ItemValueSetZeroEnable.steeringWheelForce = ENABLE;
		break;

	/* 方向盘转角检测 */
	case PROCESS_MODE_DETECTED_STEERING_WHEEL_ANGLE:
		ItemValueSetZeroEnable.steeringWheelAngle = ENABLE;
		break;

	/* 制动踏板力检测 */
	case PROCESS_MODE_DETECTED_PEDAL_FORCE:
		ItemValueSetZeroEnable.pedalForce = ENABLE;
		break;

	/* 手刹制动力检测 */
	case PROCESS_MODE_DETECTED_HAND_BRAKE_FORCE:
		ItemValueSetZeroEnable.handBrakeForce = ENABLE;
		break;

	/* 喇叭检测 */
	case PROCESS_MODE_DETECTED_NOISE:
		ItemValueSetZeroEnable.noise = ENABLE;
		break;

	case PROCESS_MODE_DETECTED_BRAKING_DISTANCE:
		ItemValueSetZeroEnable.brakeAx = ENABLE;
		break;

	case PROCESS_MODE_DETECTED_SIDESLIP_DISTANCE:
		ItemValueSetZeroEnable.sideSlip = ENABLE;
		break;

	case PROCESS_MODE_DETECTED_DOWN_VELOCITY:
		ItemValueSetZeroEnable.downVelocity = ENABLE;
		break;


	case PROCESS_MODE_DETECTED_GRADIENT:
		ItemValueSetZeroEnable.gradient = ENABLE;
		break;

	default:
		break;
	}
}
