#include "pressure.h"

/******************************************************************************/
PRESSURE_ParamTypedef PRESSURE_Param;

/******************************************************************************/
static uint32_t HX711_ReadValue(void);

/*******************************************************************************
 *
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
	double pedal = 0.0;
	uint32_t value = 0;

	/* 获取24bitAD值 */
	value = HX711_ReadValue();
	/* 判断AD值合法 */
	if (value > PRESSURE_Param.pedalValueMin)
	{
		value -= PRESSURE_Param.pedalValueMin;
	}
	else
	{
		value = 0;
	}
	/* 转换踏板力值 */
	pedal = (value / (double)PRESSURE_Param.pedalValueRange)
			* PRESSURE_RANGE_PEDAL_FORCE;

#if DEVICE_TEST_MODE_ENABLE
	printf("HX711=%d,%x\r\n", value, value);
	printf("踏板力=%.1fN\r\n", pedal);
#else

#endif
}

/*******************************************************************************
 * @brief 获取转向力
 */
void PRESSURE_GetSteeringWheelForce(void)
{
	double steeringWheelForce = 0.0;
	uint32_t value = 0;

	/* 获取24bitAD值 */
	value = HX711_ReadValue();
	/* 判断AD值合法 */
	if (value > PRESSURE_Param.steeringWheelForceMin)
	{
		value -= PRESSURE_Param.steeringWheelForceMin;
	}
	else
	{
		value = 0;
	}
	/* 转换踏板力值 */
	steeringWheelForce = (value / (double)PRESSURE_Param.steeringWheelForceRange)
			* PRESSURE_RANGE_STEERING_WHEEL_FORCE;

#if DEVICE_TEST_MODE_ENABLE
	printf("HX711=%d,%x\r\n", value, value);
	printf("转向力=%.1fN\r\n", steeringWheelForce);
#else

#endif
}

/*******************************************************************************
 * @brief 获取转向力
 */
void PRESSURE_GetHandBrakeForce(void)
{
	double handBrakeForce = 0.0;
	uint32_t value = 0;

	/* 获取24bitAD值 */
	value = HX711_ReadValue();
	/* 判断AD值合法 */
	if (value > PRESSURE_Param.handBrakeForceMin)
	{
		value -= PRESSURE_Param.handBrakeForceMin;
	}
	else
	{
		value = 0;
	}
	/* 转换踏板力值 */
	handBrakeForce = (value / (double)PRESSURE_Param.handBrakeForceRange)
			* PRESSURE_RANGE_HAND_BRAKE_FORCE;

#if DEVICE_TEST_MODE_ENABLE
	printf("HX711=%d,%x\r\n", value, value);
	printf("手刹力=%.1fN\r\n", handBrakeForce);
#else

#endif
}

/*******************************************************************************
 * @brief 从HX711上顺序读取24位AD值
 */
static uint32_t ReadValue(void)
{
	uint64_t value = 0;
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

	/* 取高16bits数据 */
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
