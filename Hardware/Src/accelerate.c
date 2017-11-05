#include "accelerate.h"
#include "ble.h"
#include "oled.h"

/******************************************************************************/
uint8_t ACCELERATE_RecvBytes[ACCELERATE_UART_RX_BYTE_MAX];
ACCELERATE_RecvTypedef       ACCELERATE_Recv;
ACCELERATE_SendStrcutTypedef ACCELERATE_SendStrcut;

extern ItemValueTypedef     ItemValue;
extern ItemZeroValueTypedef ItemZeroValue;
extern ItemValueSetZeroEnableTypedef ItemValueSetZeroEnable;

/******************************************************************************/
static void AccelerateSpeedProcess(ACCELERATE_RecvStrcutTypedef* buffer);
static void AccelerateAngleProcess(ACCELERATE_RecvStrcutTypedef* buffer);

/*******************************************************************************
 * @brief 加速度模块初始化
 */
void ACCELERATE_Init(void)
{
	UART_DMAIdleConfig(&ACCELERATE_UART, ACCELERATE_RecvBytes, ACCELERATE_UART_RX_BYTE_MAX);
	ACCELERATE_SendStrcut.head1 = 0xFF;
	ACCELERATE_SendStrcut.head2 = 0xAA;
}

/*******************************************************************************
 * @brief 接收加速度传感器返回的信息
 *
 * 			加速度传感器数据处理2种情况：
 * 			1.配置传感器回传信息为100Hz，能获取到最新的数值，但是积分时间10ms无法保障
 * 			2.定时器10ms中断，能获取准确的10ms积分时间，但是数值可能还是上一次返回的值
 * 			待后续测试
 */
void ACCELERATE_Process(void)
{
	if (ACCELERATE_Recv.status == ENABLE)
	{
		ACCELERATE_Recv.status = DISABLE;

		/* 判断帧头 */
		if (ACCELERATE_Recv.buffer.head != ACCELERATE_PROTOCOL_HEAD)
			return;

		/* 校验和 */
//		if (ACCELERATE_Recv.buffer.sum != CheckSum((uint8_t*)&ACCELERATE_Recv.buffer))
//			return;

		/* 判断数据类型 */
		switch (ACCELERATE_Recv.buffer.type)
		{
		/* 加速度输出 */
		case ACCELERATE_TYPE_ACCELERATE_SPEED:
			AccelerateSpeedProcess(&ACCELERATE_Recv.buffer);
			break;

		/* 角度输出 */
		case ACCELERATE_TYPE_ANGLE:
			AccelerateAngleProcess(&ACCELERATE_Recv.buffer);
			break;

		default:
			break;
		}
	}
}

/*******************************************************************************
 * @breif 设置回传内容
 * @param RSWL包含信息：
			ACCELERATE_TYPE_DATE_MARK
			ACCELERATE_TYPE_ACCELERATE_SPEED_MARK
			ACCELERATE_TYPE_ANGULAR_SPEED_MARK
			ACCELERATE_TYPE_ANGLE_MARK
			ACCELERATE_TYPE_MAGNETIC_FIELD_MARK
			ACCELERATE_TYPE_PORT_STATUS_MARK
			ACCELERATE_TYPE_AIR_PRESS_AND_ALTITUDE_MARK
			ACCELERATE_TYPE_LONGITUDE_AND_LATITUDE_MARK
		  RSWH包含信息：
			ACCELERATE_TYPE_GROUND_SPEED_MARK
			ACCELERATE_TYPE_FOUR_ELEMENTS_MARK
			ACCELERATE_TYPE_LOCATION_MARK
 */
void ACCELERATE_SetBackInfo(uint8_t RSWL, uint8_t RSWH)
{
	ACCELERATE_SendStrcut.address = ACCELERATE_ADDR_RSW;
	ACCELERATE_SendStrcut.dataL   = RSWL;
	ACCELERATE_SendStrcut.dataH   = RSWH;
	HAL_UART_Transmit_DMA(&ACCELERATE_UART, (uint8_t*)&ACCELERATE_SendStrcut,
			sizeof(ACCELERATE_SendStrcutTypedef));
}

/*******************************************************************************
 *
 */
void ACCELERATE_UartIdleDeal(void)
{
	uint32_t tmp_flag = 0, tmp_it_source = 0;

	tmp_flag      = __HAL_UART_GET_FLAG(&ACCELERATE_UART, UART_FLAG_IDLE);
	tmp_it_source = __HAL_UART_GET_IT_SOURCE(&ACCELERATE_UART, UART_IT_IDLE);
	if((tmp_flag != RESET) && (tmp_it_source != RESET))
	{
		__HAL_DMA_DISABLE(ACCELERATE_UART.hdmarx);
		__HAL_DMA_CLEAR_FLAG(ACCELERATE_UART.hdmarx, ACCELERATE_UART_DMA_RX_GL_FLAG);

		/* Clear Uart IDLE Flag */
		__HAL_UART_CLEAR_IDLEFLAG(&ACCELERATE_UART);

		ACCELERATE_Recv.size = ACCELERATE_UART_RX_BYTE_MAX
						- __HAL_DMA_GET_COUNTER(ACCELERATE_UART.hdmarx);

		if (ACCELERATE_Recv.size >= sizeof(ACCELERATE_RecvStrcutTypedef))
		{
			memcpy(&ACCELERATE_Recv.buffer, ACCELERATE_RecvBytes, ACCELERATE_Recv.size);
			ACCELERATE_Recv.status = ENABLE;
		}
		memset(ACCELERATE_RecvBytes, 0, ACCELERATE_Recv.size);

		ACCELERATE_UART.hdmarx->Instance->CNDTR = ACCELERATE_UART.RxXferSize;
		__HAL_DMA_ENABLE(ACCELERATE_UART.hdmarx);
	}
}

/*******************************************************************************
 * @brief 根据计算方法获取加速度值
 */
static double GetAccelerateSpeed(int16_t data)
{
	double accelerate = 0.0;

	accelerate = ((data / ACCELERATE_DIGITAL_RANGE) * ACCELERATE_RANGE_ACCELERATE);
	return accelerate;
}

/*******************************************************************************
 * @brief 根据计算方式获取角度值
 */
static float GetAngleValue(int16_t data)
{
	float angle = 0.0;

	angle = (float)((data / ACCELERATE_DIGITAL_RANGE) * ACCELERATE_RANGE_ANGLE);
	return angle;
}

/*******************************************************************************
 * @brief 加速度传感器通过积分方式计算速度和位移
 * 		  速度 = 加速度对时间的积分
 * 		 位移 = 速度对时间的积分
 * 		 制动距离：加速度为负的整个运动过程，一旦出现加速度为正，则制动距离清空
 *
 * 		 测试发现：静止状态下，加速度值有±0.5的零点偏移，所以绝对值<0.5默认为静止，不积分
 */
//double Ax = 0.0;
//double Ay = 0.0;
//double Az = 0.0;
//double Axy = 0.0;
double Speed = 0.0;

static void AccelerateSpeedProcess(ACCELERATE_RecvStrcutTypedef* buffer)
{
//	double Ax = 0.0;
	char value[7];

	/* x轴加速度在data1位置 */
	ItemValue.Ax = GetAccelerateSpeed(buffer->data1);
//	Ay = GetAccelerateSpeed(buffer->data2);
//	Az = GetAccelerateSpeed(buffer->data3);
//	Axy = sqrt(Ax * Ax + Ay * Ay + Az * Az);
//	Axy = sqrt(Ax * Ax + Ay * Ay);

	if (ItemValueSetZeroEnable.brakeAx == ENABLE)
	{
		ItemValueSetZeroEnable.brakeAx = DISABLE;
		ItemZeroValue.Ax = ItemValue.Ax;
		ItemValue.brakeVelocity = 0;
		ItemValue.brakeVelocityInit = 0;
		ItemValue.brakeDistance = 0;
	}

	/* 加速度零点校准 */
	ItemValue.Ax -= ItemZeroValue.Ax;

	if (fabs(ItemValue.Ax) > 0.05)
	{
		ItemValue.brakeVelocity += ItemValue.Ax * ACCELERATE_INTEGRAL_TIME;

		Speed = ItemValue.brakeVelocity * 3.6;

		if (ItemValue.Ax < 0)
		{
			ItemValue.brakeVelocityInit = ItemValue.brakeVelocity;
			ItemValue.brakeDistance += ItemValue.brakeVelocity
					* ACCELERATE_INTEGRAL_TIME;
		}
		else
		{
			ItemValue.brakeVelocityInit = 0;
			ItemValue.brakeDistance     = 0;
		}
	}
	else
	{
		ItemValue.brakeVelocity = 0;
		ItemValue.brakeVelocityInit  = 0;
		ItemValue.brakeDistance = 0;
		Speed = 0;
	}

	/* 显示实时速度 */
	sprintf(value, "%6.1f", Speed);
#if DEVICE_OLED_DISPLAY_ENABLE
	OLED_ShowString(64, 2, value, 6);
#endif
#if DEVICE_BLE_SEND_ENABLE
	BLE_SendBytes(BLE_DATA_TYPE_BRAKING_INITIAL_VELOCITY, value);
#endif

	/* 显示实时速度 */
	sprintf(value, "%6.1f", ItemValue.brakeDistance);
#if DEVICE_OLED_DISPLAY_ENABLE
	OLED_ShowString(64, 4, value, 6);
#endif
#if DEVICE_BLE_SEND_ENABLE
	BLE_SendBytes(BLE_DATA_TYPE_BRAKING_DISTANCE, value);
#endif
}

/*******************************************************************************
 * @brief 方向盘转角检测
 * 		  （方向盘的转动在同一平面内，转动不会引起Roll（x轴滚动角）和Pitch（y轴俯仰角）的变化，则Yaw
 * 		      （航向角）成为计量角度的唯一变量）
 */
static void AccelerateAngleProcess(ACCELERATE_RecvStrcutTypedef* buffer)
{
	char value[7];

	/* 当前的角度值 - 零位值 */
	ItemValue.steeringWheelAngle = GetAngleValue(buffer->data3);

	if (ItemValueSetZeroEnable.steeringWheelAngle == ENABLE)
	{
		ItemValueSetZeroEnable.steeringWheelAngle = DISABLE;
		ItemZeroValue.steeringWheelAngle = ItemValue.steeringWheelAngle;
	}

	ItemValue.steeringWheelAngle -= ItemZeroValue.steeringWheelAngle;

	sprintf(value, "%6.1f", ItemValue.steeringWheelAngle);
#if DEVICE_OLED_DISPLAY_ENABLE
	OLED_ShowString(64, 2, value, 6);
#endif
#if DEVICE_BLE_SEND_ENABLE
	BLE_SendBytes(BLE_DATA_TYPE_STEERING_WHEEL_ANGLE, value);
#endif
}

