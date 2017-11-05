#include "../Inc/ble.h"
#include "oled.h"
#include "process.h"

/******************************************************************************/
uint8_t BLE_RecvBytes[BLE_UART_RX_BYTE_MAX];
BLE_RecvTypedef BLE_Recv;
uint8_t BLE_sendBuffer[BLE_UART_RX_BYTE_MAX];
BLE_SendStructTypedef BLE_SendStruct;

extern ItemZeroValueTypedef ItemZeroValue;
extern ItemValueTypedef     ItemValue;

extern const char ChineseFont_SteeringWheelForce[CHINESE_FONT_SIZE * 6];
extern const char ChineseFont_CurrentValue[CHINESE_FONT_SIZE * 3];
extern const char ChineseFont_ValueMax[CHINESE_FONT_SIZE * 3];
extern const char ChineseFont_ValueMin[CHINESE_FONT_SIZE * 3];
extern const char ChineseFont_SteeringWheelAngle[CHINESE_FONT_SIZE * 5];
extern const char ChineseFont_BrakingDistance[CHINESE_FONT_SIZE * 4];

/*******************************************************************************
 *
 */
void BLE_Init(void)
{
	BLE_SendStruct.head = BLE_PROTOCOL_HEAD;
	BLE_SendStruct.length = 6;
	BLE_SendStruct.tail = BLE_PROTOCOL_TAIL;
	UART_DMAIdleConfig(&BLE_UART, BLE_RecvBytes, BLE_UART_RX_BYTE_MAX);
}

/*******************************************************************************
 *
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
 *
 */
void BLE_SendBytes(BLE_CmdDataTypeEnum type, char* value)
{
	memcpy(BLE_SendStruct.data, value, sizeof(BLE_SendStruct.data));
	BLE_SendStruct.type = type;
	BLE_SendStruct.verify = CheckSum(&BLE_SendStruct.type, 7);
//	HAL_UART_Transmit_DMA(&BLE_UART, (uint8_t*)&BLE_SendStruct,
//					sizeof(BLE_SendStructTypedef));
	HAL_UART_Transmit(&BLE_UART, (uint8_t*)&BLE_SendStruct,
					sizeof(BLE_SendStructTypedef), 1000);
}

/*******************************************************************************
 * @brief 蓝牙串口接收到的信息处理
 */
void BLE_Process(void)
{
	if (BLE_Recv.status == ENABLE)
	{
		BLE_Recv.status = DISABLE;

//#if DEVICE_TEST_MODE_ENABLE
		/* 如果是测试模式，判断第一个字节 */
//		switch (BLE_Recv.buffer.head)

		/* 根据协议判断 */
		if ((BLE_Recv.buffer.head != BLE_PROTOCOL_HEAD)
				|| (BLE_Recv.buffer.tail != BLE_PROTOCOL_TAIL))
			return;
		if (CheckSum(&BLE_Recv.buffer.cmdType, 2) != BLE_Recv.buffer.verify)
			return;
//#endif
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

		/* 开启方向盘转向力检测 */
		case BLE_CMD_TYPE_DETECTED_STEERING_WHEEL_FORCE:
			PROCESS_Mode = PROCESS_MODE_DETECTED_STEERING_WHEEL_FORCE;
			OLED_Clear();
			OLED_ShowChineseString(0, 0, (char*)ChineseFont_SteeringWheelForce,
					sizeof(ChineseFont_SteeringWheelForce) / CHINESE_FONT_SIZE);
			OLED_ShowString(0, 2, "value = ", 8);
			OLED_ShowString(104, 2, "N", 1);
			break;

		/* 开启方向盘转角检测 */
		case BLE_CMD_TYPE_DETECTED_STEERING_WHEEL_ANGLE:
			PROCESS_Mode = PROCESS_MODE_DETECTED_STEERING_WHEEL_ANGLE;
			/* 设置角度信息回传 */
			ACCELERATE_SetBackInfo(ACCELERATE_TYPE_ANGLE_MARK, 0x00);
			OLED_Clear();
			OLED_ShowChineseString(0, 0, (char*)ChineseFont_SteeringWheelAngle,
					sizeof(ChineseFont_SteeringWheelAngle) / CHINESE_FONT_SIZE);
			OLED_ShowString(0, 2, "value = ", 8);
			OLED_ShowString(104, 2, "°", 1);
			break;

		/* 开启制动距离检测 */
		case BLE_CMD_TYPE_DETECTED_BRAKING_DISTANCE:
			PROCESS_Mode = PROCESS_MODE_DETECTED_BRAKING_DISTANCE;
			/* 设置加速度信息回传 */
			ACCELERATE_SetBackInfo(ACCELERATE_TYPE_ACCELERATE_SPEED_MARK, 0x00);
			OLED_Clear();
			OLED_ShowChineseString(0, 0, (char*)ChineseFont_BrakingDistance,
					sizeof(ChineseFont_BrakingDistance) / CHINESE_FONT_SIZE);
			OLED_ShowString(0, 2, "value = ", 8);
			OLED_ShowString(104, 2, "km/h", 4);
			OLED_ShowString(0, 4, "distance", 8);
			OLED_ShowString(104, 4, "m", 1);
			break;

		/* 开启制动踏板力检测 */
		case BLE_CMD_TYPE_DETECTED_PEDAL_FORCE:
			PROCESS_Mode = PROCESS_MODE_DETECTED_PEDAL_FORCE;
			OLED_Clear();
			OLED_ShowString(0, 2, "value = ", 8);
			OLED_ShowString(104, 2, "N", 1);
			break;

		/* 开启手刹制动力检测 */
		case BLE_CMD_TYPE_DETECTED_HAND_BRAKE_FORCE:
			PROCESS_Mode = PROCESS_MODE_DETECTED_HAND_BRAKE_FORCE;
			OLED_Clear();
			OLED_ShowString(0, 2, "value = ", 8);
			OLED_ShowString(104, 2, "N", 1);
			break;

		/* 开启喇叭检测 */
		case BLE_CMD_TYPE_DETECTED_NOISE:
			PROCESS_Mode = PROCESS_MODE_DETECTED_NOISE;
			OLED_Clear();
			OLED_ShowString(0, 2, "value = ", 8);
			OLED_ShowString(104, 2, "dB", 2);
			break;

		/* 开启侧滑量检测 */
		case BLE_CMD_TYPE_DETECTED_SIDESLIP_DISTANCE:
			PROCESS_Mode = PROCESS_MODE_DETECTED_SIDESLIP_DISTANCE;
			OLED_Clear();
			break;

		/* 开启货叉下降速度检测 */
		case BLE_CMD_TYPE_DETECTED_DOWN_VELOCITY:
			PROCESS_Mode = PROCESS_MODE_DETECTED_DOWN_VELOCITY;
			OLED_Clear();
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

/*******************************************************************************
 * @brief 蓝牙命令子类型处理
 */
void BLE_SonsorSetZeroProcess(uint8_t subType)
{
	switch (subType)
	{
	/* 转向力置零 */
	case BLE_CMD_SUBTYPE_SET_ZORO_STEERING_WHEEL_FORCE:
		break;

	/* 转角置零 */
	case BLE_CMD_SUBTYPE_SET_ZORO_STEERING_WHEEL_ANGLE:
		break;

	/* 制动距离置零 */
	case BLE_CMD_SUBTYPE_SET_ZORO_BRAKING_DISTANCE:
		break;

	/* 踏板力置零 */
	case BLE_CMD_SUBTYPE_SET_ZORO_PEDAL_FORCE:
		break;

	/* 手刹力置零 */
	case BLE_CMD_SUBTYPE_SET_ZORO_HAND_BRAKE_FORCE:
		break;

	/* 噪声置零 */
	case BLE_CMD_SUBTYPE_SET_ZORO_NOISE:
		break;

	 /* 侧滑力置零 */
	case BLE_CMD_SUBTYPE_SET_ZORO_SIDESLIP_FORCE:
		break;

	 /* 下降速度置零 */
	case BLE_CMD_SUBTYPE_SET_ZORO_DOWN_VELOCITY:
		break;

	default:
		break;
	}
}
