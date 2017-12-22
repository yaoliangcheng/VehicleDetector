#include "pressure.h"
#include "ble.h"
#include "oled.h"

/******************************************************************************/
extern BLE_SendStructTypedef BLE_SendStruct;


extern FunctionalState HandBrake_SetZeroEnable;
extern double HandBrake_force;											/* 手刹力 */
extern double HandBrake_forceZeroValue;	/* 手刹力零点 */
extern double HandBrake_forceFullValue;	/* 手刹力满点 */

/******************************************************************************/

/*******************************************************************************
 * @brief 手刹力
 */
void PRESSURE_GetHandBrakeForce(void)
{
	int32_t data = 0;
	char value[7];

	/* 获取24bitAD值 */
	data = HX711_ReadValue();

	if (HandBrake_SetZeroEnable == ENABLE)
	{
		HandBrake_SetZeroEnable = DISABLE;
		HandBrake_forceZeroValue = data;
	}

	data -= HandBrake_forceZeroValue;

	if (data > 0)
	{
		HandBrake_force = (data / (HandBrake_forceFullValue - HandBrake_forceZeroValue))
				* PRESSURE_RANGE_HAND_BRAKE_FORCE;
		/* 在此处添加校正系数 */
		HandBrake_force = HandBrake_force * 1.0;
	}

#if DEVICE_OLED_DISPLAY_ENABLE
	sprintf(value, "%6.1f", HandBrake_force);
	OLED_ShowString(56, 2, value, sizeof(value));
#endif
#if DEVICE_BLE_SEND_ENABLE
	BLE_SendStruct.length = 3;
	Double2Format(HandBrake_force, BLE_SendStruct.pack.doubleData);
	BLE_SendBytes(BLE_DATA_TYPE_HAND_BRAKE_FORCE);
#endif

	HAL_Delay(200);
}

/*******************************************************************************
 * @brief 从HX711上顺序读取24位AD值
 */
static uint32_t ReadValue(void)
{
	uint32_t value = 0;
	uint8_t  i;

	/* 拉低DSCK脚，使能AD转换 */
	PRESSURE_SCK_WRITE(GPIO_PIN_RESET);
	/* 等待DOUT置高，说明转换完成 */
	while (PRESSURE_DT_READ());
	/* 顺序获取24位AD值 */
	for (i = 0; i < 24; i++)
	{
		/* DSCK置高，（发送脉冲） */
		PRESSURE_SCK_WRITE(GPIO_PIN_SET);
		/* 左移一位，等待下降沿获取数据 */
		value = value << 1;
		PRESSURE_SCK_WRITE(GPIO_PIN_RESET);
		/* 下降沿获取数据 */
		if (PRESSURE_DT_READ())
		{
			value++;
		}
	}
	/* 发送第25个脉冲，表示下一次转换为A通道128倍增益 */
	PRESSURE_SCK_WRITE(GPIO_PIN_SET);
	value = value ^ 0x800000;
	PRESSURE_SCK_WRITE(GPIO_PIN_RESET);

	return value;
}

/*******************************************************************************
 * @brief HX711默认采样频率为10Hz或者80Hz
 * 			（RATE = 0 : 输出10Hz		RATE = DVDD : 输出80Hz）
 * 			如果采样频率 = 10Hz，则因为采样频率不够和本身已做过滤波，不需再滤波
 * 			（第一版因为把RATE脚接地，不好割线，所以暂时先把设定采样频率为10Hz）
 *
 * 			模拟量供电电压Vavdd = Vbg * (R1 + R2)/ R2
 * 			注意：Vavdd的值要 < 模块的供电电压 - 100mV（最少）
 *
 */
uint32_t HX711_ReadValue(void)
{
	uint32_t sum = 0;
	
#if (HX711_RATE != HZ711_RATE_10HZ)
	uint8_t  i;
	
	for (i = 0; i < 10; i++)
	{
		sum += ReadValue();
	}

	sum /= 10;
#else
	sum = ReadValue();
#endif

	return sum;
}
