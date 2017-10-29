#include "noise.h"

/******************************************************************************/
/* 噪声请求指令 */
uint8_t NOISE_RequireCmd[8] = \
		/* 地址  命令字    起始高    起始低    长度高   长度低    CRC16 CRC16*/
		{0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
uint8_t NOISE_RecvBytes[NOISE_UART_RX_BYTE_MAX];
NOISE_RecvTypedef NOISE_Recv;

/*******************************************************************************
 *
 */
void NOISE_Init(void)
{
	UART_DMAIdleConfig(&NOISE_UART, NOISE_RecvBytes, NOISE_UART_RX_BYTE_MAX);
}

/*******************************************************************************
 *
 */
void NOISE_Require(void)
{
	HAL_UART_Transmit_DMA(&NOISE_UART, (uint8_t*)NOISE_RequireCmd,
			sizeof(NOISE_RequireCmd));
	/* 让噪音采集不那么频繁 */
	HAL_Delay(100);
}

/*******************************************************************************
 * @brief 噪声值由dataH和dataL组成一个16位的数值
 * 		  该模块量程 = 30 ~ 120dB
 */
void NOISE_Process(void)
{
	uint16_t noiseValue = 0;
	float    noise = 0.0;

	if (ENABLE == NOISE_Recv.status)
	{
		NOISE_Recv.status = DISABLE;

		if ((NOISE_Recv.buffer.addr == NOISE_MODULE_ADDR)
			&& (NOISE_Recv.buffer.cmdType == NOISE_MODULE_CMD_TYPE)
			&& (NOISE_Recv.buffer.dataLength == NOISE_MODULE_DATA_LENGTH))
		{
			noiseValue = (NOISE_Recv.buffer.dataH << 8) | NOISE_Recv.buffer.dataL;
			noise = (((uint32_t)noiseValue * NOISE_MODULE_RANGE_DB)
						/ (float)65535) + NOISE_MODULE_RANGE_DB_MIN;
#if DEVICE_TEST_MODE_ENABLE
			printf("噪声模块值=%d，噪声=%.1f", noiseValue, noise);
#else

#endif
		}


	}
}

/*******************************************************************************
 *
 */
void NOISE_UartIdleDeal(void)
{
	uint32_t tmp_flag = 0, tmp_it_source = 0;

	tmp_flag      = __HAL_UART_GET_FLAG(&NOISE_UART, UART_FLAG_IDLE);
	tmp_it_source = __HAL_UART_GET_IT_SOURCE(&NOISE_UART, UART_IT_IDLE);
	if((tmp_flag != RESET) && (tmp_it_source != RESET))
	{
		__HAL_DMA_DISABLE(NOISE_UART.hdmarx);
		__HAL_DMA_CLEAR_FLAG(NOISE_UART.hdmarx, NOISE_UART_DMA_RX_GL_FLAG);

		/* Clear Uart IDLE Flag */
		__HAL_UART_CLEAR_IDLEFLAG(&NOISE_UART);

		NOISE_Recv.size = NOISE_UART_RX_BYTE_MAX
						- __HAL_DMA_GET_COUNTER(NOISE_UART.hdmarx);

		memcpy(&NOISE_Recv.buffer, NOISE_RecvBytes, NOISE_Recv.size);
		NOISE_Recv.status = ENABLE;

		memset(NOISE_RecvBytes, 0, NOISE_Recv.size);

		NOISE_UART.hdmarx->Instance->CNDTR = NOISE_UART.RxXferSize;
		__HAL_DMA_ENABLE(NOISE_UART.hdmarx);
	}
}
