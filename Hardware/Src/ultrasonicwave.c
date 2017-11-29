#include "ultrasonicwave.h"
#include "oled.h"
#include "noise.h"

/******************************************************************************/
ULTRASONICWAVE_RecvTypedef ULTRASONICWAVE_Recv;

/* 货叉下降速度检测 */
extern uint16_t DownVelocity_Distance;
extern uint16_t DownVelocity_DistanceOld;
extern double   DownVelocity_Speed;
extern uint8_t NOISE_RecvBytes[NOISE_UART_RX_BYTE_MAX];

/*******************************************************************************
 *
 */
void ULTRASONICWAVE_Require(void)
{
	LL_USART_TransmitData8(USART2, 0x55);
}

/*******************************************************************************
 *
 */
void ULTRASONICWAVE_Process(void)
{
	char value[7];
	uint8_t size;

	if (ENABLE == ULTRASONICWAVE_Recv.status)
	{
		ULTRASONICWAVE_Recv.status = DISABLE;

		if (ULTRASONICWAVE_Recv.buffer.head != ULTRASONICWAVE_HEAD)
			return;

		/* 获取离地距离 */
		DownVelocity_Distance = (ULTRASONICWAVE_Recv.buffer.dataH << 8)
				| (ULTRASONICWAVE_Recv.buffer.dataL);
		/* 获取当前速度 */
		DownVelocity_Speed = (DownVelocity_Distance - DownVelocity_DistanceOld)
				/ ULTRASONICWAVE_TIME_PERIOD;
		/* 缓存当前值 */
		DownVelocity_DistanceOld = DownVelocity_Distance;

#if DEVICE_OLED_DISPLAY_ENABLE
		size = sprintf(value, "%d", DownVelocity_Distance);
		OLED_ShowString(72, 2, value, size);
		size = sprintf(value, "%.1f", DownVelocity_Speed);
		OLED_ShowString(72, 4, value, size);
#endif
	}
}

/*******************************************************************************
 * @brief 噪音串口接收处理
 */
//void ULTRASONICWAVE_UartIdleDeal(void)
//{
//	uint32_t tmp_flag = 0, tmp_it_source = 0;

//	tmp_flag      = __HAL_UART_GET_FLAG(&ULTRASONICWAVE_UART, UART_FLAG_IDLE);
//	tmp_it_source = __HAL_UART_GET_IT_SOURCE(&ULTRASONICWAVE_UART, UART_IT_IDLE);
//	if((tmp_flag != RESET) && (tmp_it_source != RESET))
//	{
//		__HAL_DMA_DISABLE(ULTRASONICWAVE_UART.hdmarx);
//		__HAL_DMA_CLEAR_FLAG(ULTRASONICWAVE_UART.hdmarx, ULTRASONICWAVE_UART_DMA_RX_GL_FLAG);

//		/* Clear Uart IDLE Flag */
//		__HAL_UART_CLEAR_IDLEFLAG(&ULTRASONICWAVE_UART);

//		ULTRASONICWAVE_Recv.size = ULTRASONICWAVE_UART_RX_BYTE_MAX
//						- __HAL_DMA_GET_COUNTER(ULTRASONICWAVE_UART.hdmarx);

//		memcpy(&ULTRASONICWAVE_Recv.buffer, NOISE_RecvBytes, ULTRASONICWAVE_Recv.size);
//		ULTRASONICWAVE_Recv.status = ENABLE;

//		memset(NOISE_RecvBytes, 0, ULTRASONICWAVE_Recv.size);

//		ULTRASONICWAVE_UART.hdmarx->Instance->CNDTR = ULTRASONICWAVE_UART.RxXferSize;
//		__HAL_DMA_ENABLE(ULTRASONICWAVE_UART.hdmarx);
//	}
//}

