#include "process.h"
#include "ultrasonicwave.h"
#include "encode.h"
#include "pressure.h"
#include "oled.h"
#include "ble.h"

/******************************************************************************/
PROCESS_ModeEnum PROCESS_Mode = PROCESS_MODE_INVALID;
ItemValueTypedef     ItemValue;
ItemZeroValueTypedef ItemZeroValue;
ItemValueSetZeroEnableTypedef ItemValueSetZeroEnable;


/* 制动距离检测 */
FunctionalState BrakeDistance_SetZeroEnable;
double BrakeDistance_pedalForce;				/* 踏板力 */
double BrakeDistance_pedalForceZeroValue = PEDAL_FORCE_ZERO_VALUE;/* 踏板力零点 */
double BrakeDistance_pedalForceFullValue = PEDAL_FORCE_FULL_VALUE;/* 踏板力满点 */
double BrakeDistance_speed;						/* 制动检测-速度 */
double BrakeDistance_oldSpeed;					/* 制动检测-旧速度 */
double BrakeDistance_initSpeed;					/* 初始速度 */
double BrakeDistance_distance;					/* 制动检测-距离 */

/* 货叉下降速度检测 */
uint16_t DownVelocity_Distance;
uint16_t DownVelocity_DistanceOld;
double   DownVelocity_Speed;



uint16_t Encode_plusCnt = 0;				/* 编码器脉冲数 */
uint16_t Encode_plusCntOld = 0;				/* 编码器旧脉冲数 */
uint16_t Encode_periodCntTotal = 0;			/* 总周期数 */
uint16_t Encode_periodCnt = 0;				/* 周期数 */
FunctionalState Encode_processEnable;		/* 编码器Process使能 */
FunctionalState Encode_initEnable;

extern BLE_SendStructTypedef BLE_SendStruct;

/******************************************************************************/
void SteeringWheel_ForceAndAngleProcess(void);
void PROCESS_PedalForceAndBrakeDistance(void);

/*******************************************************************************
 *
 */
void PROCESS(void)
{
	switch (PROCESS_Mode)
	{
	/* 方向盘转向力检测 */
	case PROCESS_MODE_DETECTED_STEERING_WHEEL_FORCE_AND_ANGLE:
		SteeringWheel_ForceAndAngleProcess();
		break;

	/* 踏板力和制动距离检测 */
	case PROCESS_MODE_DETECTED_PEDAL_FORCE_BRAKING_DISTANCE:
//		ACCELERATE_Process();
//		ENCODE_Process();
		/* 获取踏板力 */
//		PRESSURE_GetPedalForce();
		PROCESS_PedalForceAndBrakeDistance();
		break;

//	/* 制动踏板力检测 */
//	case PROCESS_MODE_DETECTED_PEDAL_FORCE:
//		/* 获取踏板力 */
//		PRESSURE_GetPedalForce();
//		break;

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
	case PROCESS_MODE_DETECTED_BATTERY_CAPACITY:
		ANALOG_Process();
		/* 锂电池电量检测，只检测一次 */
		PROCESS_Mode = PROCESS_MODE_INVALID;
		break;

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
	case PROCESS_MODE_DETECTED_STEERING_WHEEL_FORCE_AND_ANGLE:
		ItemValueSetZeroEnable.steeringWheelForce = ENABLE;
		break;

	/* 制动踏板力检测 */
	case PROCESS_MODE_DETECTED_PEDAL_FORCE_BRAKING_DISTANCE:
		BrakeDistance_SetZeroEnable = ENABLE;
		break;

	/* 手刹制动力检测 */
	case PROCESS_MODE_DETECTED_HAND_BRAKE_FORCE:
		ItemValueSetZeroEnable.handBrakeForce = ENABLE;
		break;

	/* 喇叭检测 */
	case PROCESS_MODE_DETECTED_NOISE:
		ItemValueSetZeroEnable.noise = ENABLE;
		break;

//	case PROCESS_MODE_DETECTED_BRAKING_DISTANCE:
//		ItemValueSetZeroEnable.brakeAx = ENABLE;
//		break;

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

/*******************************************************************************
 *
 */
void SteeringWheel_ForceAndAngleProcess(void)
{
	int32_t data = 0;
	char value[7];

	/* 获取24bitAD值 */
	data = HX711_ReadValue();
	/* 获取的AD值减去中间值 */
	data -= PRESSURE_RANGE_MEDIAN_VALUE;

	/* 转换转向力值 */
//	ItemValue.steeringWheelForce =
//			(data / (double)PRESSURE_Param.steeringWheelForceRange)
//			* PRESSURE_RANGE_STEERING_WHEEL_FORCE * 5.44;
	ItemValue.steeringWheelForce =
			(data / (double)1)
			* PRESSURE_RANGE_STEERING_WHEEL_FORCE * 5.44;

	if (ItemValueSetZeroEnable.steeringWheelForce == ENABLE)
	{
		ItemValueSetZeroEnable.steeringWheelForce = DISABLE;
		ItemZeroValue.steeringWheelForce = ItemValue.steeringWheelForce;
	}

	/* 零点校准 */
	ItemValue.steeringWheelForce -= ItemZeroValue.steeringWheelForce;

	/* 获取方向盘角度 */
	ACCELERATE_Process();

#if DEVICE_OLED_DISPLAY_ENABLE
	sprintf(value, "%6.1f", ItemValue.steeringWheelForce);
	OLED_ShowString(56, 2, value, sizeof(value));
#endif
#if DEVICE_BLE_SEND_ENABLE
	BLE_SendStruct.length = sizeof(ItemValue.steeringWheelForce) +
			sizeof(ItemValue.steeringWheelAngle);
	BLE_SendStruct.pack.SteeringWheel_ForceAndAngle.force = ItemValue.steeringWheelForce;
	BLE_SendStruct.pack.SteeringWheel_ForceAndAngle.angle = ItemValue.steeringWheelAngle;
	BLE_SendBytes(BLE_DATA_TYPE_STEERING_WHELL_FORCE_AND_ANGLE);
#endif

	HAL_Delay(200);
}

/*******************************************************************************
 *
 */
void PROCESS_PedalForceAndBrakeDistance(void)
{
	uint16_t plusCnt = 0;
	uint32_t data = 0;
	uint8_t  value[10];
	uint8_t  size = 0;

	if (Encode_processEnable == ENABLE)
	{
		Encode_processEnable = DISABLE;

		plusCnt = ((ENCODE_PERIOD_PLUS_CNT - Encode_plusCntOld) +
				((Encode_periodCnt - 1) * ENCODE_PERIOD_PLUS_CNT) + Encode_plusCnt) / 2;
		/* 用完后旧的脉冲周期要清空 */
		Encode_periodCnt = 0;
		
		/* 缓存旧值 */
		Encode_plusCntOld = Encode_plusCnt;

		/* 计算速度 */
		BrakeDistance_speed = ((ENCODE_WHEEL_PERIMETER / (double)ENCODE_PERIOD_PLUS_CNT)
								* plusCnt) / GET_VALUE_TIME_PERIOD;

		/* 获取24bitAD值 */
		data = HX711_ReadValue();

		if (BrakeDistance_SetZeroEnable == ENABLE)
		{
			BrakeDistance_SetZeroEnable = DISABLE;
			BrakeDistance_pedalForceZeroValue = data;
		}

		/* 转换踏板力值 */
		data -= BrakeDistance_pedalForceZeroValue;
		BrakeDistance_pedalForce = (data /
				(double)(BrakeDistance_pedalForceFullValue - BrakeDistance_pedalForceZeroValue))
				* PRESSURE_RANGE_PEDAL_FORCE;

		/* 开始制动 */
		if (BrakeDistance_pedalForce > 5)
		{
			if (Encode_initEnable == DISABLE)
			{
				Encode_initEnable = ENABLE;
				LL_TIM_SetCounter(TIM3, 0);
				Encode_plusCntOld = 0;
				Encode_periodCnt = 0;
				Encode_periodCntTotal = 0;
				BrakeDistance_initSpeed = BrakeDistance_speed;
			}
			/* 计算距离 */
			BrakeDistance_distance = ((Encode_periodCntTotal * ENCODE_WHEEL_PERIMETER)
					+ ((ENCODE_WHEEL_PERIMETER / (double)ENCODE_PERIOD_PLUS_CNT)
					* Encode_plusCnt)) / 2;
		}
		else
		{
			Encode_initEnable = DISABLE;
			BrakeDistance_initSpeed = 0;
			BrakeDistance_distance = 0;
		}
#if DEVICE_OLED_DISPLAY_ENABLE
		size = sprintf(value, "%6.1f", BrakeDistance_pedalForce);
		OLED_ShowString(56, 0, value, size);
		size = sprintf(value, "%6.1f", BrakeDistance_initSpeed);
		OLED_ShowString(56, 2, value, size);
		size = sprintf(value, "%6.1f", BrakeDistance_speed);
		OLED_ShowString(56, 4, value, size);
		size = sprintf(value, "%6.1f", BrakeDistance_distance);
		OLED_ShowString(56, 6, value, size);
#endif
#if DEVICE_BLE_SEND_ENABLE
		BLE_SendStruct.length = sizeof(BrakeDistance_pedalForce)
				+ sizeof(BrakeDistance_initSpeed) + sizeof (BrakeDistance_speed)
				+ sizeof(BrakeDistance_distance);
		BLE_SendStruct.pack.PedalForce_BrakeDistance.pedalForce = BrakeDistance_pedalForce;
		BLE_SendStruct.pack.PedalForce_BrakeDistance.initSpeed  = BrakeDistance_initSpeed;
		BLE_SendStruct.pack.PedalForce_BrakeDistance.speed      = BrakeDistance_speed;
		BLE_SendStruct.pack.PedalForce_BrakeDistance.distance   = BrakeDistance_distance;
		BLE_SendBytes(BLE_DATA_TYPE_PEDAL_FORCE_AND_BRAKING_DISTANCE);
#endif
	}
}






