#include "analog.h"
#include "ble.h"
#include "oled.h"


/******************************************************************************/
static uint16_t convertValueBuffer[ANALOG_SAMPLE_NUMB];
static FunctionalState convertFinished = DISABLE;

extern ItemValueTypedef     ItemValue;

/******************************************************************************/
static uint16_t GetAverageValue(void);
static uint8_t GetBatVoltage(uint16_t value);

/*******************************************************************************
 * 开启ADC转换
 */
void ANALOG_ConvertEnable(void)
{
	HAL_ADC_Start_DMA(&ANALOG_ADC, (uint32_t*)convertValueBuffer,
								sizeof(convertValueBuffer));
}

/*******************************************************************************
 * 停止ADC转换
 */
void ANALOG_ConvertDisable(void)
{
	HAL_ADC_Stop_DMA(&ANALOG_ADC);
	convertFinished = ENABLE;
}

/*******************************************************************************
 * @brief 模拟量转换进程
 */
void ANALOG_Process(void)
{
	uint16_t data;
	char value[7];
	/* 等待转换结束 */
	if (ENABLE == convertFinished)
	{
		convertFinished = DISABLE;
		/* 获取模拟量的值 */
		data = GetAverageValue();
		/* 获取电池电量值 */
		ItemValue.batteryCapacity = GetBatVoltage(data);

		sprintf(value, "%6d", ItemValue.batteryCapacity);
#if DEVICE_OLED_DISPLAY_ENABLE
		OLED_ShowString(64, 2, value, 6);
#endif
#if DEVICE_BLE_SEND_ENABLE
		BLE_SendBytes(BLE_DATA_TYPE_BATTERY_CAPACITY, value);
#endif

	}
}

/*******************************************************************************
 * @brief 冒泡法排序滤波算法
 * 		算法：采样20个值，将20个值从大到小排序，去中间的10个值取平均
 */
static uint16_t GetAverageValue(void)
{
	uint8_t  j, k;
	uint16_t value;
	uint32_t average;

	/* 冒泡法排序 */
	for (j = RESET; j < ANALOG_SAMPLE_NUMB - 1; j++)
	{
		for (k = j + 1; k < ANALOG_SAMPLE_NUMB; k++)
		{
			if (convertValueBuffer[k] > convertValueBuffer[j])
			{
				value = convertValueBuffer[j];
				convertValueBuffer[j] = convertValueBuffer[k];
				convertValueBuffer[k] = value;
			}
		}
	}

	/* 清空平均值 */
	average = RESET;

	/* 取中间的数值的平均数 */
	for(j = ANALOG_SAMPLE_NUMB / 2 - 5; j < ANALOG_SAMPLE_NUMB / 2 + 5; j++)
	{
		average += convertValueBuffer[j];
	}
	return (uint16_t)(average / 10);
}

/*******************************************************************************
 * @brief 获取锂电池电压值
 * 		  锂电池电压范围为3.2V~4.2V，标称3.7V，充满电为4.2V
 * 		  当电压降到3.4V以下，可能导致3.3V稳压芯片无法工作，而系统掉电
 */
static uint8_t GetBatVoltage(uint16_t value)
{
	uint32_t voltage;
	float percent;

	/* 乘以2，是因为4.2V电阻分压 */
	voltage = (((uint32_t)value * 3300) * 2 / 4096);

	percent = (float)voltage / 4200;

	if (percent > 1)
		percent = 1;

	return (uint8_t)(percent * 100);
}
