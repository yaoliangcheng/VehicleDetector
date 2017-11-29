#include "noise.h"
#include "ble.h"
#include "oled.h"

/******************************************************************************/
/* 噪声请求指令 */
uint8_t NOISE_RequireCmd[8] = \
		/* 地址  命令字    起始高    起始低    长度高   长度低    CRC16 CRC16*/
		{0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
uint8_t NOISE_RecvBytes[NOISE_UART_RX_BYTE_MAX];
NOISE_RecvTypedef NOISE_Recv;
		
extern ItemValueTypedef     ItemValue;
extern ItemZeroValueTypedef ItemZeroValue;
extern ItemValueSetZeroEnableTypedef ItemValueSetZeroEnable;

/*******************************************************************************
 * @brief 请求噪音数据
 */
void NOISE_Require(void)
{
	uint8_t cnt = sizeof(NOISE_RequireCmd);
	uint8_t* pData = NOISE_RequireCmd;

	while (cnt--)
	{
		LL_USART_TransmitData8(USART2, *pData);
		pData++;
	}
}

/*******************************************************************************
 * @brief 噪声值由dataH和dataL组成一个16位的数值
 * 		  该模块量程 = 30 ~ 120dB
 */
void NOISE_Process(void)
{
	uint16_t noiseValue = 0;
	char value[7];

	if (ENABLE == NOISE_Recv.status)
	{
		NOISE_Recv.status = DISABLE;
		/* 校验噪音数据 */
		if ((NOISE_Recv.buffer.addr == NOISE_MODULE_ADDR)
			&& (NOISE_Recv.buffer.cmdType == NOISE_MODULE_CMD_TYPE)
			&& (NOISE_Recv.buffer.dataLength == NOISE_MODULE_DATA_LENGTH))
		{
			noiseValue = (NOISE_Recv.buffer.dataH << 8) | NOISE_Recv.buffer.dataL;
			/* 噪音值 = 数据 * 0.1 */
			ItemValue.noise = ((float)noiseValue * 0.1);
			/* 零点校准使能 */
			if (ItemValueSetZeroEnable.noise == ENABLE)
			{
				ItemValueSetZeroEnable.noise = DISABLE;
				ItemZeroValue.noise = ItemValue.noise;
			}
			/* 零点校准 */
			ItemValue.noise -= ItemZeroValue.noise;

			sprintf(value, "%6.1f", ItemValue.noise);
#if DEVICE_OLED_DISPLAY_ENABLE
			OLED_ShowString(64, 2, value, 6);
#endif
#if DEVICE_BLE_SEND_ENABLE
			BLE_SendBytes(BLE_DATA_TYPE_NOISE, value);
#endif
		}
	}
}

/*******************************************************************************
 * @brief 噪音串口接收处理
 */
void NOISE_UartIdleDeal(void)
{
//	uint32_t tmp_flag = 0, tmp_it_source = 0;

//	tmp_flag      = __HAL_UART_GET_FLAG(&NOISE_UART, UART_FLAG_IDLE);
//	tmp_it_source = __HAL_UART_GET_IT_SOURCE(&NOISE_UART, UART_IT_IDLE);
//	if((tmp_flag != RESET) && (tmp_it_source != RESET))
//	{
//		__HAL_DMA_DISABLE(NOISE_UART.hdmarx);
//		__HAL_DMA_CLEAR_FLAG(NOISE_UART.hdmarx, NOISE_UART_DMA_RX_GL_FLAG);

//		/* Clear Uart IDLE Flag */
//		__HAL_UART_CLEAR_IDLEFLAG(&NOISE_UART);

//		NOISE_Recv.size = NOISE_UART_RX_BYTE_MAX
//						- __HAL_DMA_GET_COUNTER(NOISE_UART.hdmarx);

//		memcpy(&NOISE_Recv.buffer, NOISE_RecvBytes, NOISE_Recv.size);
//		NOISE_Recv.status = ENABLE;

//		memset(NOISE_RecvBytes, 0, NOISE_Recv.size);

//		NOISE_UART.hdmarx->Instance->CNDTR = NOISE_UART.RxXferSize;
//		__HAL_DMA_ENABLE(NOISE_UART.hdmarx);
//	}
}
