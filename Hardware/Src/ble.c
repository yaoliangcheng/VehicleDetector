#include "../Inc/ble.h"

/******************************************************************************/
uint8_t BLE_RecvBytes[BLE_UART_RX_BYTE_MAX];
BLE_RecvTypedef BLE_Recv;
uint8_t BLE_sendBuffer[BLE_UART_RX_BYTE_MAX];

/*******************************************************************************
 *
 */
void BLE_Init(void)
{
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
void BLE_SendBytes(uint8_t size)
{
	HAL_UART_Transmit_DMA(&BLE_UART, BLE_sendBuffer, size);
}

/*******************************************************************************
 * @brief 蓝牙串口接收到的信息处理
 */
void BLE_Process(void)
{
	if (BLE_Recv.status == ENABLE)
	{
		BLE_Recv.status = DISABLE;

#if DEVICE_TEST_MODE_ENABLE
		/* 如果是测试模式，判断第一个字节 */
		switch (BLE_Recv.buffer[0])
#else
		/* 根据协议判断 */
		/* todo */
#endif
		{
		/* 传感器零位校正 */
		case BLE_CMD_TYPE_SENSOR_VALUE_SET_ZERO:
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
			break;

		/* 开启方向盘转角检测 */
		case BLE_CMD_TYPE_DETECTED_STEERING_WHEEL_ANGLE:
			PROCESS_Mode = PROCESS_MODE_DETECTED_STEERING_WHEEL_ANGLE;
			/* 设置角度信息回传 */
			ACCELERATE_SetBackInfo(ACCELERATE_TYPE_ANGLE_MARK, 0x00);
			break;

		/* 开启制动距离检测 */
		case BLE_CMD_TYPE_DETECTED_BRAKING_DISTANCE:
			PROCESS_Mode = PROCESS_MODE_DETECTED_BRAKING_DISTANCE;
			/* 设置加速度信息回传 */
			ACCELERATE_SetBackInfo(ACCELERATE_TYPE_ACCELERATE_SPEED_MARK, 0x00);
			break;

		/* 开启制动踏板力检测 */
		case BLE_CMD_TYPE_DETECTED_PEDAL_FORCE:
			PROCESS_Mode = PROCESS_MODE_DETECTED_PEDAL_FORCE;
			break;

		/* 开启手刹制动力检测 */
		case BLE_CMD_TYPE_DETECTED_HAND_BRAKE_FORCE:
			PROCESS_Mode = PROCESS_MODE_DETECTED_HAND_BRAKE_FORCE;
			break;

		/* 开启喇叭检测 */
		case BLE_CMD_TYPE_DETECTED_NOISE:
			PROCESS_Mode = PROCESS_MODE_DETECTED_NOISE;
			break;

		/* 开启侧滑量检测 */
		case BLE_CMD_TYPE_DETECTED_SIDESLIP_DISTANCE:
			PROCESS_Mode = PROCESS_MODE_DETECTED_SIDESLIP_DISTANCE;
			break;

		/* 开启货叉下降速度检测 */
		case BLE_CMD_TYPE_DETECTED_DOWN_VELOCITY:
			PROCESS_Mode = PROCESS_MODE_DETECTED_DOWN_VELOCITY;
			break;

		/* 开启电池电量检测 */
		case BLE_CMD_TYPE_DETECTED_BAT_VOLTAGE:
			ANALOG_ConvertEnable();
			PROCESS_Mode = PROCESS_MODE_DETECTED_BAT_VOLTAGE;
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
