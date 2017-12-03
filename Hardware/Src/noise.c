#include "noise.h"
#include "ble.h"
#include "oled.h"
#include "public.h"

/******************************************************************************/
/* 噪声请求指令 */
uint8_t NOISE_RequireCmd[8] = \
		/* 地址  命令字    起始高    起始低    长度高   长度低    CRC16 CRC16*/
		{0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
uint8_t NOISE_RecvBytes[NOISE_UART_RX_BYTE_MAX];
NOISE_RecvTypedef NOISE_Recv;

extern FunctionalState Noisy_SetZeroEnable;
extern float  Noisy_Value;
extern float  Noisy_ZeroValue;

extern BLE_SendStructTypedef BLE_SendStruct;
		
/*******************************************************************************
 * @brief 请求噪音数据
 */
void NOISE_Require(void)
{
	uint8_t cnt = sizeof(NOISE_RequireCmd);
	uint8_t* pData = NOISE_RequireCmd;

	while (cnt--)
	{
		while(!LL_USART_IsActiveFlag_TXE(USART2));
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
			Noisy_Value = ((float)noiseValue * 0.1);
			/* 零点校准使能 */
			if (Noisy_SetZeroEnable == ENABLE)
			{
				Noisy_SetZeroEnable = DISABLE;
				Noisy_ZeroValue = Noisy_Value;
			}
			/* 零点校准 */
			Noisy_Value -= Noisy_ZeroValue;

#if DEVICE_OLED_DISPLAY_ENABLE
			sprintf(value, "%6.1f", Noisy_Value);
			OLED_ShowString(64, 2, value, 6);
#endif
#if DEVICE_BLE_SEND_ENABLE
			BLE_SendStruct.length = 3;
			Double2Format((double)Noisy_Value, BLE_SendStruct.pack.doubleData);
			BLE_SendBytes(BLE_DATA_TYPE_NOISE);
#endif
		}
	}
}


