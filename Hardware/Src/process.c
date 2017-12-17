#include "process.h"
#include "ultrasonicwave.h"
#include "encode.h"
#include "pressure.h"
#include "oled.h"
#include "ble.h"

/******************************************************************************/
PROCESS_ModeEnum PROCESS_Mode = PROCESS_MODE_INVALID;
ItemValueTypedef     ItemValue;
ItemValueSetZeroEnableTypedef ItemValueSetZeroEnable;
ItemZeroValueTypedef ItemZeroValue;
/* 制动距离检测 */
FunctionalState BrakeDistance_SetZeroEnable;
double BrakeDistance_pedalForce;				/* 踏板力 */
double BrakeDistance_pedalForceZeroValue = PEDAL_FORCE_ZERO_VALUE;/* 踏板力零点 */
double BrakeDistance_pedalForceFullValue = PEDAL_FORCE_FULL_VALUE;/* 踏板力满点 */
double BrakeDistance_speed;						/* 制动检测-速度 */
double BrakeDistance_oldSpeed;					/* 制动检测-旧速度 */
double BrakeDistance_initSpeed;					/* 初始速度 */
double BrakeDistance_distance;					/* 制动检测-距离 */

uint16_t Encode_plusCnt = 0;					/* 编码器脉冲数 */
uint16_t Encode_plusCntOld = 0;					/* 编码器旧脉冲数 */
uint16_t Encode_periodCntTotal = 0;				/* 总周期数 */
uint16_t Encode_periodCnt = 0;					/* 周期数 */
uint8_t  Encode_errorCnt = 0;					/* 错误计数 */
FunctionalState Encode_errorStatus;				/* 错误状态 */
FunctionalState Encode_processEnable;			/* 编码器Process使能 */
FunctionalState Encode_initEnable;

/* 货叉下降速度检测 */
uint16_t DownVelocity_Distance;
uint16_t DownVelocity_DistanceOld;
double   DownVelocity_Speed;

/* 方向盘 */
FunctionalState SteeringWheel_SetZeroEnable;
double SteeringWheel_Force;
double SteeringWheel_ForceZeroValue = STEERING_WHEEL_FORCE_ZERO_VALUE;
double SteeringWheel_ForceFullValue = STEERING_WHEEL_FORCE_FULL_VALUE;
double SteeringWheel_ForceFullValue_N = STEERING_WHEEL_FORCE_FULL_VALUE_N;
double SteeringWheel_Angle;
double SteeringWheel_AngleZeroValue = 0;

/* 手刹力 */
FunctionalState HandBrake_SetZeroEnable;
double HandBrake_force;											/* 手刹力 */
double HandBrake_forceZeroValue = HAND_BRAKE_FORCE_ZERO_VALUE;	/* 手刹力零点 */
double HandBrake_forceFullValue = HAND_BRAKE_FORCE_FULL_VALUE;	/* 手刹力满点 */

/* 噪声监测 */
float  Noisy_Value;

/* 坡度检测 */
FunctionalState Gradient_SetZeroEnable;
float  Gradient_Value;
float  Gradient_ZeroValue;

/* 电池电量 */
uint8_t BatteryVoltage_Value;

/* 侧滑量检测 */
FunctionalState SideSlip_SetZeroEnable;
double   SideSlip_distance;
uint32_t SideSlip_plusCnt;
double   SideSlip_angle;
double   SideSlip_angleZeroValue;

extern BLE_SendStructTypedef BLE_SendStruct;

/******************************************************************************/
void SteeringWheel_ForceAndAngleProcess(void);
void PROCESS_PedalForceAndBrakeDistance(void);
void Process_SideSlipDistance(void);

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
		PROCESS_PedalForceAndBrakeDistance();
		break;

	/* 手刹制动力检测 */
	case PROCESS_MODE_DETECTED_HAND_BRAKE_FORCE:
		/* 获取手刹力 */
		PRESSURE_GetHandBrakeForce();
		break;

	/* 喇叭检测 */
	case PROCESS_MODE_DETECTED_NOISE:
		NOISE_Process();
		break;

	/* 侧滑量检测 */
	case PROCESS_MODE_DETECTED_SIDESLIP_DISTANCE:
		Process_SideSlipDistance();
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
		SteeringWheel_SetZeroEnable = ENABLE;
		break;

	/* 制动踏板力检测 */
	case PROCESS_MODE_DETECTED_PEDAL_FORCE_BRAKING_DISTANCE:
		BrakeDistance_SetZeroEnable = ENABLE;
		break;

	/* 手刹制动力检测 */
	case PROCESS_MODE_DETECTED_HAND_BRAKE_FORCE:
		HandBrake_SetZeroEnable = ENABLE;
		break;

	/* 侧滑量检测 */
	case PROCESS_MODE_DETECTED_SIDESLIP_DISTANCE:
		SideSlip_SetZeroEnable = ENABLE;
		break;

	/* 坡度检测 */
	case PROCESS_MODE_DETECTED_GRADIENT:
		Gradient_SetZeroEnable = ENABLE;
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
	/* 获取方向盘角度 */
	ACCELERATE_Process();
	if (SteeringWheel_SetZeroEnable == ENABLE)
	{
		SteeringWheel_SetZeroEnable = DISABLE;
		SteeringWheel_ForceZeroValue = data;
		SteeringWheel_AngleZeroValue = SteeringWheel_Angle;
	}

	data -= SteeringWheel_ForceZeroValue;
	/* 转换转向力值 */
	if (data > 0)
	{
		SteeringWheel_Force = (data / (SteeringWheel_ForceFullValue - SteeringWheel_ForceZeroValue))
				* PRESSURE_RANGE_STEERING_WHEEL_FORCE;
	}
	else
	{
		SteeringWheel_Force = (data / (SteeringWheel_ForceZeroValue - SteeringWheel_ForceFullValue_N))
				* PRESSURE_RANGE_STEERING_WHEEL_FORCE;
	}

	/* 角度值 */
	SteeringWheel_Angle -= SteeringWheel_AngleZeroValue;


#if DEVICE_OLED_DISPLAY_ENABLE
	sprintf(value, "%6.1f", SteeringWheel_Force);
	OLED_ShowString(56, 2, value, sizeof(value));
	sprintf(value, "%6.1f", SteeringWheel_Angle);
	OLED_ShowString(56, 4, value, sizeof(value));
#endif
#if DEVICE_BLE_SEND_ENABLE
	BLE_SendStruct.length = sizeof(SteeringWheel_ForceAndAngleTypedef);
	Double2Format(SteeringWheel_Force, BLE_SendStruct.pack.SteeringWheel_ForceAndAngle.force);
	Double2Format(SteeringWheel_Angle, BLE_SendStruct.pack.SteeringWheel_ForceAndAngle.angle);
	BLE_SendBytes(BLE_DATA_TYPE_STEERING_WHELL_FORCE_AND_ANGLE);
#endif

	HAL_Delay(200);
}

uint16_t plusCnt = 0;
/*******************************************************************************
 *
 */
void PROCESS_PedalForceAndBrakeDistance(void)
{
	
	uint32_t data = 0;
	char     value[10];
	uint8_t  size = 0;

	if (Encode_processEnable == ENABLE)
	{
		Encode_processEnable = DISABLE;

		/* 错误状态，直接返回 */
		if (Encode_errorStatus == ENABLE)
			return;

		if (LL_TIM_GetDirection(TIM3) != LL_TIM_COUNTERDIRECTION_UP)
		{
			Encode_errorCnt++;
			if (Encode_errorCnt > 10)
			{
				Encode_errorStatus = ENABLE;
			}
			Encode_periodCnt = 0;
			return;
		}

		/* 有过零脉冲 */
		if (Encode_periodCnt > 0)
		{
			plusCnt = ((ENCODE_PERIOD_PLUS_CNT - Encode_plusCntOld) +
				((Encode_periodCnt - 1) * ENCODE_PERIOD_PLUS_CNT) + Encode_plusCnt) / 2;
			/* 车辆前进，则清空错误计数 */
			Encode_errorCnt = 0;
		}
		else if (Encode_plusCnt > Encode_plusCntOld)
		{
			plusCnt = (Encode_plusCnt - Encode_plusCntOld) / 2;
		}
		else
		{
			/* 连续3s检测到编码器倒退，则认为车辆抖动引起的，停止检测 */
//			Encode_errorCnt++;
//			if (Encode_errorCnt > 6)
//			{
//				Encode_errorCnt = 0;
//				Encode_errorStatus = ENABLE;
//			}
//			return;
		}

		/* 用完后旧的脉冲周期要清空 */
		Encode_periodCnt = 0;
		
		/* 缓存旧值 */
		Encode_plusCntOld = Encode_plusCnt;

		/* 脉冲数大于基数才认为是机动车运行，否则为震动造成的，不计算速度 */
//		if (plusCnt > 100)
		{
			/* 计算速度 */
			BrakeDistance_speed = ((ENCODE_WHEEL_PERIMETER / (double)(ENCODE_PERIOD_PLUS_CNT / 2))
									* plusCnt) / GET_VALUE_TIME_PERIOD;
			/* 速度单位转换 mm/s -> km/h (*0.0036) */
			BrakeDistance_speed = BrakeDistance_speed * 0.0036;
		}

		/* 获取24bitAD值 */
		data = HX711_ReadValue();

		if (BrakeDistance_SetZeroEnable == ENABLE)
		{
			BrakeDistance_SetZeroEnable = DISABLE;
			BrakeDistance_pedalForceZeroValue = data;

			BrakeDistance_pedalForce = 0;					/* 踏板力 */
			BrakeDistance_speed      = 0;					/* 制动检测-速度 */
			BrakeDistance_oldSpeed   = 0;					/* 制动检测-旧速度 */
			BrakeDistance_initSpeed  = 0;					/* 初始速度 */
			BrakeDistance_distance   = 0;					/* 制动检测-距离 */

			Encode_plusCnt = 0;								/* 编码器脉冲数 */
			Encode_plusCntOld = 0;							/* 编码器旧脉冲数 */
			Encode_periodCntTotal = 0;						/* 总周期数 */
			Encode_periodCnt = 0;							/* 周期数 */
		}

		/* 转换踏板力值 */
		data -= BrakeDistance_pedalForceZeroValue;
		BrakeDistance_pedalForce = (data /
				(double)(BrakeDistance_pedalForceFullValue - BrakeDistance_pedalForceZeroValue))
				* PRESSURE_RANGE_PEDAL_FORCE;

		/* 开始制动 */
		if (BrakeDistance_pedalForce > 5)
		{
			/* 开始制动的第一时间，脉冲计算清空重新计数，并获取初速度值 */
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
					+ (ENCODE_WHEEL_PERIMETER / (double)(ENCODE_PERIOD_PLUS_CNT / 2))
					* Encode_plusCnt) / 2;
			BrakeDistance_distance = BrakeDistance_distance * 0.001;
		}
		else
		{
			Encode_initEnable = DISABLE;
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
		BLE_SendStruct.length = sizeof(PedalForce_BrakeDistanceTypedef);
		Double2Format(BrakeDistance_pedalForce,
				BLE_SendStruct.pack.PedalForce_BrakeDistance.pedalForce);
		Double2Format(BrakeDistance_initSpeed,
				BLE_SendStruct.pack.PedalForce_BrakeDistance.initSpeed);
		Double2Format(BrakeDistance_speed,
				BLE_SendStruct.pack.PedalForce_BrakeDistance.speed);
		Double2Format(BrakeDistance_distance,
				BLE_SendStruct.pack.PedalForce_BrakeDistance.distance);
		BLE_SendBytes(BLE_DATA_TYPE_PEDAL_FORCE_AND_BRAKING_DISTANCE);
#endif
	}
}

/*******************************************************************************
 * @brief 侧滑量检测
 */
void Process_SideSlipDistance(void)
{
	uint16_t plusCnt = 0;
	uint32_t data = 0;
	char     value[10];
	uint8_t  size = 0;

	/* 获取位移 */
	plusCnt = LL_TIM_GetCounter(TIM3);

	/* 获取角度 */
	ACCELERATE_Process();
	if (SideSlip_SetZeroEnable == ENABLE)
	{
		SideSlip_SetZeroEnable = DISABLE;
		SideSlip_angleZeroValue = SideSlip_angle;
	}

//	if (plusCnt > 100)
	{
		/* 计算位移 */
		SideSlip_distance = (((SideSlip_plusCnt * ENCODE_WHEEL_PERIMETER)
			+ (ENCODE_WHEEL_PERIMETER / (double)(ENCODE_PERIOD_PLUS_CNT / 2))
			* plusCnt) / 2);
		/* 计算角度 */
		SideSlip_angle -= SideSlip_angleZeroValue;
		/* 计算偏移量 */
		SideSlip_distance = SideSlip_distance * sin(SideSlip_angle);
		SideSlip_distance = SideSlip_distance * 0.001;
	}

#if DEVICE_OLED_DISPLAY_ENABLE
		size = sprintf(value, "%6.1f", SideSlip_distance);
		OLED_ShowString(56, 0, value, size);
#endif
#if DEVICE_BLE_SEND_ENABLE
		BLE_SendStruct.length = 3;
		Double2Format(SideSlip_distance, BLE_SendStruct.pack.doubleData);
		BLE_SendBytes(BLE_DATA_TYPE_SIDESLIP_FORCE);
#endif

	HAL_Delay(500);
}




