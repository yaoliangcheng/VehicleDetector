#include "pressure.h"
#include "ble.h"
#include "oled.h"

/******************************************************************************/
PRESSURE_ParamTypedef PRESSURE_Param;

extern ItemValueTypedef     ItemValue;
extern ItemZeroValueTypedef ItemZeroValue;
extern ItemValueSetZeroEnableTypedef ItemValueSetZeroEnable;

/******************************************************************************/
static uint32_t HX711_ReadValue(void);

/*******************************************************************************
 * @brief 压力传感器参数初始化
 */
void PRESSURE_Init(void)
{
	PRESSURE_Param.pedalValueMax = 0xFFFFFF;
	PRESSURE_Param.pedalValueMin = 0x7FEEEE;
	PRESSURE_Param.pedalValueRange = PRESSURE_Param.pedalValueMax
			- PRESSURE_Param.pedalValueMin;

	PRESSURE_Param.steeringWheelForceMax = 0xFFFFFF;
	PRESSURE_Param.steeringWheelForceMin = 0x7FEEEE;
	PRESSURE_Param.steeringWheelForceRange = PRESSURE_Param.steeringWheelForceMax
			- PRESSURE_Param.steeringWheelForceMin;

	PRESSURE_Param.handBrakeForceMax = 0xFFFFFF;
	PRESSURE_Param.handBrakeForceMin = 0x7FEEEE;
	PRESSURE_Param.handBrakeForceRange = PRESSURE_Param.handBrakeForceMax
			- PRESSURE_Param.handBrakeForceMin;
}

/*******************************************************************************
 * @brief 获取踏板力值
 * 		  踏板力的线性关系
 * 		  1.最大最小值直接取线性
 * 		  2.线性拟合
 * 		  3.查表
 */
void PRESSURE_GetPedalForce(void)
{
	uint32_t data = 0;
	char value[7];

	/* 获取24bitAD值 */
	data = HX711_ReadValue();
	/* 判断AD值合法 */
	if (data > PRESSURE_Param.pedalValueMin)
	{
		data -= PRESSURE_Param.pedalValueMin;
	}
	else
	{
		data = 0;
	}
	/* 转换踏板力值 */
	ItemValue.pedalForce = (data / (double)PRESSURE_Param.pedalValueRange)
			* PRESSURE_RANGE_PEDAL_FORCE * 1.916;
	/* 零点校准使能 */
	if (ItemValueSetZeroEnable.pedalForce == ENABLE)
	{
		ItemValueSetZeroEnable.pedalForce = DISABLE;
		ItemZeroValue.pedalForce = ItemValue.pedalForce;
	}

	/* 零点校准 */
	ItemValue.pedalForce -= ItemZeroValue.pedalForce;

	sprintf(value, "%6.1f", ItemValue.pedalForce);
#if DEVICE_OLED_DISPLAY_ENABLE
	OLED_ShowString(56, 2, value, sizeof(value));
#endif
#if DEVICE_BLE_SEND_ENABLE
	BLE_SendBytes(BLE_DATA_TYPE_PEDAL_FORCE, value);
#endif

	HAL_Delay(10);
}

/*******************************************************************************
 * @brief 获取转向力
 */
void PRESSURE_GetSteeringWheelForce(void)
{
	uint32_t data = 0;
	char value[7];

	/* 获取24bitAD值 */
	data = HX711_ReadValue();
	/* 判断AD值合法 */
	if (data > PRESSURE_Param.steeringWheelForceMin)
	{
		data -= PRESSURE_Param.steeringWheelForceMin;
	}
	else
	{
		data = 0;
	}
	/* 转换转向力值 */
	ItemValue.steeringWheelForce =
			(data / (double)PRESSURE_Param.steeringWheelForceRange)
			* PRESSURE_RANGE_STEERING_WHEEL_FORCE * 5.44;

	if (ItemValueSetZeroEnable.steeringWheelForce == ENABLE)
	{
		ItemValueSetZeroEnable.steeringWheelForce = DISABLE;
		ItemZeroValue.steeringWheelForce = ItemValue.steeringWheelForce;
	}

	/* 零点校准 */
	ItemValue.steeringWheelForce -= ItemZeroValue.steeringWheelForce;

	sprintf(value, "%6.1f", ItemValue.steeringWheelForce);
#if DEVICE_OLED_DISPLAY_ENABLE
	OLED_ShowString(56, 2, value, sizeof(value));
#endif
#if DEVICE_BLE_SEND_ENABLE
	BLE_SendBytes(BLE_DATA_TYPE_STEERING_WHEEL_FORCE, value);
#endif

	HAL_Delay(10);
}

/*******************************************************************************
 * @brief 获取转向力
 */
void PRESSURE_GetHandBrakeForce(void)
{
	uint32_t data = 0;
	char value[7];

	/* 获取24bitAD值 */
	data = HX711_ReadValue();
	/* 判断AD值合法 */
	if (data > PRESSURE_Param.handBrakeForceMin)
	{
		data -= PRESSURE_Param.handBrakeForceMin;
	}
	else
	{
		data = 0;
	}
	/* 转换踏板力值 */
	ItemValue.handBrakeForce =
			(data / (double)PRESSURE_Param.handBrakeForceRange)
			* PRESSURE_RANGE_HAND_BRAKE_FORCE;

	if (ItemValueSetZeroEnable.handBrakeForce == ENABLE)
	{
		ItemValueSetZeroEnable.handBrakeForce = DISABLE;
		ItemZeroValue.handBrakeForce = ItemValue.handBrakeForce;
	}

	/* 零点校准 */
	ItemValue.handBrakeForce -= ItemZeroValue.handBrakeForce;

	sprintf(value, "%6.1f", ItemValue.handBrakeForce);
#if DEVICE_OLED_DISPLAY_ENABLE
	OLED_ShowString(56, 2, value, sizeof(value));
#endif
#if DEVICE_BLE_SEND_ENABLE
	BLE_SendBytes(BLE_DATA_TYPE_HAND_BRAKE_FORCE, value);
#endif

	HAL_Delay(10);
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
static uint32_t HX711_ReadValue(void)
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
