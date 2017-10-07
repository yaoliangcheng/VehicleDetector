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
 *
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

		/* 方向盘转向力零位校正 */
		case BLE_CMD_TYPE_STEERING_WHEEL_FORCE_SET_ZERO:
			break;

		/* 开启方向盘转向力检测 */
		case BLE_CMD_TYPE_DETECTED_STEERING_WHEEL_FORCE:
			PROCESS_Mode = PROCESS_MODE_DETECTED_STEERING_WHEEL_FORCE;
			break;

		/* 方向盘转角零位校正 */
		case BLE_CMD_TYPE_STEERING_WHEEL_ANGLE_SET_ZERO:
			break;

		/* 开启方向盘转角检测 */
		case BLE_CMD_TYPE_DETECTED_STEERING_WHEEL_ANGLE:
			PROCESS_Mode = PROCESS_MODE_DETECTED_STEERING_WHEEL_ANGLE;
			/* 设置角度信息回传 */
			ACCELERATE_SetBackInfo(ACCELERATE_TYPE_ANGLE_MARK, 0x00);
			break;

		/* 刹车距离零位校正 */
		case BLE_CMD_TYPE_BRAKING_DISTANCE_SET_ZERO:
			break;

		/* 开启制动距离检测 */
		case BLE_CMD_TYPE_DETECTED_BRAKING_DISTANCE:
			PROCESS_Mode = PROCESS_MODE_DETECTED_BRAKING_DISTANCE;
			/* 设置加速度信息回传 */
			ACCELERATE_SetBackInfo(ACCELERATE_TYPE_ACCELERATE_SPEED_MARK, 0x00);
			break;

		/* 踏板力零位校正 */
		case BLE_CMD_TYPE_PEDAL_FORCE_SET_ZERO:
			break;

		/* 开启制动踏板力检测 */
		case BLE_CMD_TYPE_DETECTED_PEDAL_FORCE:
			PROCESS_Mode = PROCESS_MODE_DETECTED_PEDAL_FORCE;
			break;

		/* 手刹力零位校正 */
		case BLE_CMD_TYPE_HAND_BRAKE_FORCE_SET_ZERO:
			break;

		/* 开启手刹制动力检测 */
		case BLE_CMD_TYPE_DETECTED_HAND_BRAKE_FORCE:
			PROCESS_Mode = PROCESS_MODE_DETECTED_HAND_BRAKE_FORCE;
			break;

		/* 噪声零位校正 */
		case BLE_CMD_TYPE_NOISE_SET_ZERO:
			break;

		/* 开启喇叭检测 */
		case BLE_CMD_TYPE_DETECTED_NOISE:
			PROCESS_Mode = PROCESS_MODE_DETECTED_NOISE;
			break;

		/* 侧滑力零位校正 */
		case BLE_CMD_TYPE_SIDESLIP_DISTANCE_SET_ZERO:
					break;

		/* 开启侧滑量检测 */
		case BLE_CMD_TYPE_DETECTED_SIDESLIP_DISTANCE:
			PROCESS_Mode = PROCESS_MODE_DETECTED_SIDESLIP_DISTANCE;
			break;

		/* 下降速度零位校正 */
		case BLE_CMD_TYPE_DOWN_VELOCITY_SET_ZERO:
			break;

		/* 开启货叉下降速度检测 */
		case BLE_CMD_TYPE_DETECTED_DOWN_VELOCITY:
			PROCESS_Mode = PROCESS_MODE_DETECTED_DOWN_VELOCITY;
			break;

		default:
			break;
		}
	}
}
