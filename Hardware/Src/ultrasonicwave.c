#include "ultrasonicwave.h"
#include "oled.h"
#include "noise.h"
#include "ble.h"

/******************************************************************************/
ULTRASONICWAVE_RecvTypedef ULTRASONICWAVE_Recv;

uint8_t  UltrasonicWave_BufferIndex = 0;
uint16_t UltrasonicWave_DataBuffer[10];

/* 货叉下降速度检测 */
extern uint16_t DownVelocity_Distance;
extern uint16_t DownVelocity_DistanceOld;
extern double   DownVelocity_Speed;
extern uint8_t  NOISE_RecvBytes[NOISE_UART_RX_BYTE_MAX];

extern BLE_SendStructTypedef BLE_SendStruct;

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
	uint8_t i, j;
	uint16_t data;

	if (ENABLE == ULTRASONICWAVE_Recv.status)
	{
		ULTRASONICWAVE_Recv.status = DISABLE;

		if (ULTRASONICWAVE_Recv.buffer.head != ULTRASONICWAVE_HEAD)
			return;

		UltrasonicWave_DataBuffer[UltrasonicWave_BufferIndex] =
				(ULTRASONICWAVE_Recv.buffer.dataH << 8) | ULTRASONICWAVE_Recv.buffer.dataL;
		UltrasonicWave_BufferIndex++;

		if (UltrasonicWave_BufferIndex >= 2)
		{
			UltrasonicWave_BufferIndex = 0;

			/* todo 排序 */
			/* 冒泡法排序 */
//			for (i = 0; i < 5; i++)
//			{
//				for (j = i + 1; j < 5; j++)
//				{
//					if (UltrasonicWave_DataBuffer[j] > UltrasonicWave_DataBuffer[i])
//					{
//						data = UltrasonicWave_DataBuffer[i];
//						UltrasonicWave_DataBuffer[i] = UltrasonicWave_DataBuffer[j];
//						UltrasonicWave_DataBuffer[j] = data;
//					}
//				}
//			}
//			/* 取中位值 */
//			DownVelocity_Distance = (UltrasonicWave_DataBuffer[1] + UltrasonicWave_DataBuffer[2]) / 2;

			DownVelocity_Distance = (UltrasonicWave_DataBuffer[0] + UltrasonicWave_DataBuffer[1]) / 2;

			/* 叉车下降，距离减小，对于变大的数据默认为干扰 */
			if (DownVelocity_Distance < DownVelocity_DistanceOld)
			{
				/* 获取当前速度 */
//				DownVelocity_Speed = (DownVelocity_DistanceOld - DownVelocity_Distance)
//						/ ULTRASONICWAVE_TIME_PERIOD;
				DownVelocity_Speed = DownVelocity_DistanceOld - DownVelocity_Distance;
			}
			else
			{
				DownVelocity_Speed = 0;
			}

			/* 缓存当前值 */
			DownVelocity_DistanceOld = DownVelocity_Distance;

#if DEVICE_OLED_DISPLAY_ENABLE
	size = sprintf(value, "%4d", DownVelocity_Distance);
	OLED_ShowString(72, 2, value, size);
	size = sprintf(value, "%5.1f", DownVelocity_Speed);
	OLED_ShowString(72, 4, value, size);
#endif
#if DEVICE_BLE_SEND_ENABLE
	BLE_SendStruct.length = sizeof(DownVelocity_SendBufferTypedef);
	BLE_SendStruct.pack.DownVelocity_SendBuffer.distance = DownVelocity_Distance;
	Double2Format(DownVelocity_Speed,
			BLE_SendStruct.pack.DownVelocity_SendBuffer.speed);
	BLE_SendBytes(BLE_DATA_TYPE_DOWN_VELOCITY);
#endif
		}
	}
}
