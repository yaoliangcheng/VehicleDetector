#include "../Inc/ble.h"
#include "oled.h"
#include "process.h"
#include "tim.h"
/******************************************************************************/
uint8_t BLE_RecvBytes[BLE_UART_RX_BYTE_MAX];
BLE_RecvTypedef BLE_Recv;
uint8_t BLE_sendBuffer[BLE_UART_RX_BYTE_MAX];
BLE_SendStructTypedef BLE_SendStruct;

extern const char ChineseFont_SteeringWheelForce[CHINESE_FONT_SIZE * 6];
extern const char ChineseFont_CurrentValue[CHINESE_FONT_SIZE * 3];
extern const char ChineseFont_ValueMax[CHINESE_FONT_SIZE * 3];
extern const char ChineseFont_ValueMin[CHINESE_FONT_SIZE * 3];
extern const char ChineseFont_SteeringWheelAngle[CHINESE_FONT_SIZE * 5];
extern const char ChineseFont_BrakingDistance[CHINESE_FONT_SIZE * 4];
extern const char ChineseFont_DownVelocityTitle[CHINESE_FONT_SIZE * 8];
extern const char ChineseFont_DownVelocityDistance[CHINESE_FONT_SIZE * 4];
extern const char ChineseFont_DownVelocitySpeed[CHINESE_FONT_SIZE * 4];

/* 制动距离检测 */
extern double BrakeDistance_pedalForce;					/* 踏板力 */
extern double BrakeDistance_speed;						/* 制动检测-速度 */
extern double BrakeDistance_oldSpeed;					/* 制动检测-旧速度 */
extern double BrakeDistance_initSpeed;					/* 初始速度 */
extern double BrakeDistance_distance;					/* 制动检测-距离 */

extern uint16_t Encode_plusCnt;						/* 编码器脉冲数 */
extern uint16_t Encode_plusCntOld;					/* 编码器旧脉冲数 */
extern uint16_t Encode_periodCntTotal;				/* 总周期数 */
extern uint16_t Encode_periodCnt;					/* 周期数 */
extern FunctionalState Encode_errorStatus;				/* 错误状态 */

/* 货叉下降速度检测 */
extern uint16_t DownVelocity_Distance;
extern uint16_t DownVelocity_DistanceOld;
extern double   DownVelocity_Speed;

/* 方向盘 */
extern double SteeringWheel_Force;
extern double SteeringWheel_Angle;

/* 手刹力 */
extern double HandBrake_force;											/* 手刹力 */

/* 噪声监测 */
extern float  Noisy_Value;

/* 坡度检测 */
extern float  Gradient_Value;

/* 电池电量 */
extern uint8_t BatteryVoltage_Value;

/* 侧滑量检测 */
extern double   SideSlip_distance;
extern uint32_t SideSlip_plusCnt;
extern double   SideSlip_angle;


/*******************************************************************************
 * @brief 蓝牙初始化
 */
void BLE_Init(void)
{
	/* 蓝牙发送结构体固定值 */
	BLE_SendStruct.head = BLE_PROTOCOL_HEAD;
	BLE_SendStruct.length = 6;
	BLE_SendStruct.tail = BLE_PROTOCOL_TAIL;
	/* 蓝牙串口DMA接收初始化 */
	UART_DMAIdleConfig(&BLE_UART, BLE_RecvBytes, BLE_UART_RX_BYTE_MAX);
}

/*******************************************************************************
 * @brief 蓝牙串口DMA接收初始化
 */
void BLE_UartIdleDeal(void)
{
	uint32_t tmp_flag = 0, tmp_it_source = 0;

	tmp_flag      = __HAL_UART_GET_FLAG(&BLE_UART, UART_FLAG_IDLE);
	tmp_it_source = __HAL_UART_GET_IT_SOURCE(&BLE_UART, UART_IT_IDLE);
	if((tmp_flag != RESET) && (tmp_it_source != RESET))
	{
		__HAL_DMA_DISABLE(BLE_UART.hdmarx);
		__HAL_DMA_CLEAR_FLAG(BLE_UART.hdmarx, BLE_UART_DMA_RX_GL_FLAG);

		/* Clear Uart IDLE Flag */
		__HAL_UART_CLEAR_IDLEFLAG(&BLE_UART);

		BLE_Recv.size = BLE_UART_RX_BYTE_MAX
						- __HAL_DMA_GET_COUNTER(BLE_UART.hdmarx);

		memcpy(&BLE_Recv.buffer, BLE_RecvBytes, BLE_Recv.size);
		BLE_Recv.status = ENABLE;

		memset(BLE_RecvBytes, 0, BLE_Recv.size);

		BLE_UART.hdmarx->Instance->CNDTR = BLE_UART.RxXferSize;
		__HAL_DMA_ENABLE(BLE_UART.hdmarx);
	}
}

/*******************************************************************************
 * @brief 蓝牙数据透传
 */
void BLE_SendBytes(BLE_CmdDataTypeEnum type)
{
	/* 蓝牙发送结构体固定值 */
	BLE_SendStruct.head = BLE_PROTOCOL_HEAD;
	BLE_SendStruct.tail = BLE_PROTOCOL_TAIL;

	/* 数据类型 */
	BLE_SendStruct.type = type;

	/* 计算校验和,校验长度为数据类型 + 数据长度 + 数据  */
	BLE_SendStruct.verify = CheckSum(&BLE_SendStruct.type, BLE_SendStruct.length + 2);
	memcpy(&BLE_SendStruct.pack.buffer[BLE_SendStruct.length], &BLE_SendStruct.verify, 2);

	/* 串口发送 */
	HAL_UART_Transmit_DMA(&BLE_UART, (uint8_t*)&BLE_SendStruct,
			BLE_SendStruct.length + 5);
}

/*******************************************************************************
 * @brief 蓝牙串口接收到的信息处理
 */
void BLE_Process(void)
{
	if (BLE_Recv.status == ENABLE)
	{
		BLE_Recv.status = DISABLE;

		/* 根据协议判断 */
		if ((BLE_Recv.buffer.head != BLE_PROTOCOL_HEAD)
				|| (BLE_Recv.buffer.tail != BLE_PROTOCOL_TAIL))
			return;
		if (CheckSum(&BLE_Recv.buffer.cmdType, 2) != BLE_Recv.buffer.verify)
			return;
		/* 判断命令类型 */
		switch (BLE_Recv.buffer.cmdType)
		{
		/* 传感器零位校正 */
		case BLE_CMD_TYPE_SENSOR_VALUE_SET_ZERO:
			ZeroCalibration();
			break;

		/* 清除传感器缓存 */
		case BLE_CMD_TYPE_CLEAR_SENSOR_CACHE:
			break;

		/* 断开蓝牙 */
		case BLE_CMD_TYPE_SWITCH_OFF:
			break;

		/* 停止检测 */
		case BLE_CMD_TYPE_STOP_DETECTED:
			PROCESS_Mode = PROCESS_MODE_INVALID;
			/* 取消所有回传信息 */
			ACCELERATE_SetBackInfo(0x00, 0x00);
			break;

		/* 方向盘力和转角检测 */
		case BLE_CMD_TYPE_DETECTED_STEERING_WHEEL_FORCE_AND_ANGLE:
			PROCESS_Mode = PROCESS_MODE_DETECTED_STEERING_WHEEL_FORCE_AND_ANGLE;
			/* 设置角度信息回传 */
			ACCELERATE_SetBackInfo(ACCELERATE_TYPE_ANGLE_MARK, 0x00);
//			OLED_Clear();
			SteeringWheel_Force = 0;
			SteeringWheel_Angle = 0;
			break;

		/* 开启制动距离检测 */
		case BLE_CMD_TYPE_DETECTED_PEDAL_FORCE_AND_BRAKING_DISTANCE:
			PROCESS_Mode = PROCESS_MODE_DETECTED_PEDAL_FORCE_BRAKING_DISTANCE;
			LL_TIM_SetCounter(TIM3, 0);
			LL_TIM_EnableCounter(TIM3);
			/* 开启定时器 */
			LL_TIM_SetAutoReload(TIM7, 499);
			LL_TIM_SetCounter(TIM7, 0);
			LL_TIM_EnableCounter(TIM7);

//			OLED_Clear();
//			OLED_ShowChineseString(0, 0, (char*)ChineseFont_BrakingDistance,
//					sizeof(ChineseFont_BrakingDistance) / CHINESE_FONT_SIZE);
			BrakeDistance_pedalForce = 0;					/* 踏板力 */
			BrakeDistance_speed      = 0;					/* 制动检测-速度 */
			BrakeDistance_oldSpeed   = 0;					/* 制动检测-旧速度 */
			BrakeDistance_initSpeed  = 0;					/* 初始速度 */
			BrakeDistance_distance   = 0;					/* 制动检测-距离 */

			Encode_plusCnt = 0;								/* 编码器脉冲数 */
			Encode_plusCntOld = 0;							/* 编码器旧脉冲数 */
			Encode_periodCntTotal = 0;						/* 总周期数 */
			Encode_periodCnt = 0;							/* 周期数 */
			Encode_errorStatus = DISABLE;
			break;

		/* 开启手刹制动力检测 */
		case BLE_CMD_TYPE_DETECTED_HAND_BRAKE_FORCE:
			PROCESS_Mode = PROCESS_MODE_DETECTED_HAND_BRAKE_FORCE;
			OLED_Clear();
			OLED_ShowString(0, 2, "value = ", 8);
			OLED_ShowString(104, 2, "N", 1);
			HandBrake_force = 0;
			break;

		/* 开启喇叭检测 */
		case BLE_CMD_TYPE_DETECTED_NOISE:
			PROCESS_Mode = PROCESS_MODE_DETECTED_NOISE;
			LL_TIM_SetAutoReload(TIM7, 499);
			/* 开启定时器 */
			LL_TIM_SetCounter(TIM7, 0);
			LL_TIM_EnableCounter(TIM7);
			OLED_Clear();
			OLED_ShowString(0, 2, "value = ", 8);
			OLED_ShowString(104, 2, "dB", 2);
			Noisy_Value = 0;
			break;

		/* 开启侧滑量检测 */
		case BLE_CMD_TYPE_DETECTED_SIDESLIP_DISTANCE:
			/* 打开加速度和角度检测 */
			ACCELERATE_SetBackInfo(ACCELERATE_TYPE_ANGLE_MARK, 0x00);
			PROCESS_Mode = PROCESS_MODE_DETECTED_SIDESLIP_DISTANCE;
			LL_TIM_SetCounter(TIM3, 0);
			LL_TIM_EnableCounter(TIM3);
			SideSlip_plusCnt  = 0;
			SideSlip_angle    = 0;
			SideSlip_distance = 0;
			OLED_Clear();
			break;

		/* 开启货叉下降速度检测 */
		case BLE_CMD_TYPE_DETECTED_DOWN_VELOCITY:
			PROCESS_Mode = PROCESS_MODE_DETECTED_DOWN_VELOCITY;
			LL_TIM_SetAutoReload(TIM7, 999);
			LL_TIM_SetCounter(TIM7, 0);
			LL_TIM_EnableCounter(TIM7);

			OLED_Clear();
			OLED_ShowChineseString(0, 0, (char*)ChineseFont_DownVelocityTitle,
						sizeof(ChineseFont_DownVelocityTitle) / CHINESE_FONT_SIZE);
			OLED_ShowChineseString(0, 2, (char*)ChineseFont_DownVelocityDistance,
						sizeof(ChineseFont_DownVelocityDistance) / CHINESE_FONT_SIZE);
			OLED_ShowChineseString(0, 4, (char*)ChineseFont_DownVelocitySpeed,
						sizeof(ChineseFont_DownVelocitySpeed) / CHINESE_FONT_SIZE);
			DownVelocity_Distance    = 0;
			DownVelocity_DistanceOld = 0;
			DownVelocity_Speed       = 0;
			break;

		/* 坡度检测 */
		case BLE_CMD_TYPE_DETECTED_GRADIENT:
			/* 设置角度信息回传 */
			ACCELERATE_SetBackInfo(ACCELERATE_TYPE_ANGLE_MARK, 0x00);
			PROCESS_Mode = PROCESS_MODE_DETECTED_GRADIENT;
			OLED_Clear();
			Gradient_Value = 0;
			break;

		/* 开启电池电量检测 */
		case BLE_CMD_TYPE_DETECTED_BATTERY_CAPACITY:
			ANALOG_ConvertEnable();
			PROCESS_Mode = PROCESS_MODE_DETECTED_BATTERY_CAPACITY;
			OLED_Clear();
			break;

		default:
			break;
		}
	}
}

