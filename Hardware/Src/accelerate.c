#include "accelerate.h"
#include "ble.h"
#include "oled.h"
#include "tim.h"

/******************************************************************************/
uint8_t ACCELERATE_RecvBytes[ACCELERATE_UART_RX_BYTE_MAX];
ACCELERATE_RecvTypedef       ACCELERATE_Recv;
ACCELERATE_SendStrcutTypedef ACCELERATE_SendStrcut;

extern ItemValueTypedef     ItemValue;
extern ItemZeroValueTypedef ItemZeroValue;
extern ItemValueSetZeroEnableTypedef ItemValueSetZeroEnable;

extern uint16_t huaTimeBase;
/******************************************************************************/
static void AccelerateSpeedProcess(ACCELERATE_RecvStrcutTypedef* buffer);
static void AccelerateAngleProcess(ACCELERATE_RecvStrcutTypedef* buffer);
static void AccelerateDownSpeedProcess(ACCELERATE_RecvStrcutTypedef* buffer);

/*******************************************************************************
 * @brief 加速度模块初始化
 */
void ACCELERATE_Init(void)
{
	/* 初始化加速度传感器串口DMA接收 */
	UART_DMAIdleConfig(&ACCELERATE_UART, ACCELERATE_RecvBytes,
			ACCELERATE_UART_RX_BYTE_MAX);
	/* 设定加速度发送数据的固定内容 */
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
			if (PROCESS_Mode == PROCESS_MODE_DETECTED_BRAKING_DISTANCE)
			{
				AccelerateSpeedProcess(&ACCELERATE_Recv.buffer);
			}
			else if (PROCESS_Mode == PROCESS_MODE_DETECTED_DOWN_VELOCITY)
			{
				AccelerateDownSpeedProcess(&ACCELERATE_Recv.buffer);
			}

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
 * @breif 设置加速度传感器回传内容
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
 * @brief 串口DMA接收初始化
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
static void AccelerateSpeedProcess(ACCELERATE_RecvStrcutTypedef* buffer)
{
	char value[7];

	/* x轴加速度在data1位置 */
	ItemValue.Ax = GetAccelerateSpeed(buffer->data1);
	/* 零点校准使能 */
	if (ItemValueSetZeroEnable.brakeAx == ENABLE)
	{
		  ItemValueSetZeroEnable.brakeAx = DISABLE;
		/* 将当前值作为校准值 */
		ItemZeroValue.Ax = ItemValue.Ax;
		/* 校准后清空之前累加的值 */
		ItemValue.brakeVelocity = 0;
		ItemValue.brakeVelocityInit = 0;
		ItemValue.brakeDistance = 0;
	}

	/* 加速度零点校准 */
	ItemValue.Ax -= ItemZeroValue.Ax;

	/* 加速度为正值，速度累加0.1~1的随机数，为负值则减去随机数 */
//	if (ItemValue.Ax > 0)
//	{
//		/* 随机数值为当前定时器值的最后一位值/10 */
//		ItemValue.brakeVelocity += (float)(htim7.Instance->CNT % 10) / 10;
//	}
//	else
//	{
//		ItemValue.brakeVelocity -= (float)(htim7.Instance->CNT % 10) / 10;
//	}    
	
#if 1
	/* 避免零点噪声漂移，将绝对值小于0.5的值认为为静止，不积分 */
	if (fabs(ItemValue.Ax) > 0.05)
	{
		/* 加速度积分获取速度 */
		ItemValue.brakeVelocity += ItemValue.Ax*0.01;// * ACCELERATE_INTEGRAL_TIME;
		/* 前进方向，速度不可能为负值 */
		if (ItemValue.brakeVelocity < 0)
		{
			ItemValue.brakeVelocity = 0;
		}
		/* 速度单位转换成km/h */
		ItemValue.speed = ItemValue.brakeVelocity * 3.6;
		/* 加速度为负值，则机动车刹车，开始对速度积分，算位移 */
		
		if (ItemValue.Ax < 0)
		{
			ItemValue.brakeDistance+=ItemValue.Ax*ItemValue.Ax/20000+ItemValue.brakeVelocity*0.01;
		}
		else	/* 否则则证明在加速过程，不累计位移 */
		{
			ItemValue.brakeVelocityInit = 0;
			//ItemValue.brakeDistance     = 0;   hua
		}
	}
	else	/* 默认为静止模式，速度为0 */
	{
//		ItemValue.brakeVelocity = 0;					hua
//		ItemValue.brakeVelocityInit  = 0;
//		ItemValue.brakeDistance = 0;
//		ItemValue.speed = 0;									hua
	}
#endif

	/* 显示实时速度 */
//	sprintf(value, "%6.1f", Speed);
//#if DEVICE_OLED_DISPLAY_ENABLE
//	OLED_ShowString(64, 2, value, 6);
//#endif
//#if DEVICE_BLE_SEND_ENABLE
//	BLE_SendBytes(BLE_DATA_TYPE_BRAKING_INITIAL_VELOCITY, value);
//#endif

	/* 显示实时位移 */
	sprintf(value, "%6.1f", ItemValue.brakeDistance);
#if DEVICE_OLED_DISPLAY_ENABLE
	OLED_ShowString(64, 4, value, 6);
#endif
#if DEVICE_BLE_SEND_ENABLE
	BLE_SendBytes(BLE_DATA_TYPE_BRAKING_DISTANCE, value);
#endif
}

/*******************************************************************************
 * @brief 下降速度计算
 */
static void AccelerateDownSpeedProcess(ACCELERATE_RecvStrcutTypedef* buffer)
{
	char value[7];
	/* x轴加速度在data1位置 */
	ItemValue.downAx = GetAccelerateSpeed(buffer->data1);
	/* 零点校准使能 */
	if (ItemValueSetZeroEnable.downVelocity == ENABLE)
	{
		ItemValueSetZeroEnable.downVelocity = DISABLE;
		/* 将当前值作为校准值 */
		ItemZeroValue.downAx = ItemValue.downAx;
		/* 校准后清空之前累加的值 */
		ItemValue.downVelocity = 0;
	}

	/* 加速度零点校准 */
	ItemValue.downAx -= ItemZeroValue.downAx;

  if(fabs(ItemValue.downAx)>0.05) {ItemValue.downVelocity+=ItemValue.downAx*0.01;huaTimeBase=0;}
		else huaTimeBase++;
	if(huaTimeBase>=30) {huaTimeBase=0;ItemValue.downVelocity=0;}
	if(ItemValue.downVelocity<0) ItemValue.downVelocity=0;
	/* 显示实时速度 */
	sprintf(value, "%6.1f", ItemValue.downVelocity);
#if DEVICE_OLED_DISPLAY_ENABLE
	OLED_ShowString(64, 2, value, 6);
#endif
#if DEVICE_BLE_SEND_ENABLE
	BLE_SendBytes(BLE_DATA_TYPE_DOWN_VELOCITY, value);
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

	/* 获取当前的角度值 */
	ItemValue.steeringWheelAngle = GetAngleValue(buffer->data3);
	/* 零点校准使能 */
	if (ItemValueSetZeroEnable.steeringWheelAngle == ENABLE)
	{
		ItemValueSetZeroEnable.steeringWheelAngle = DISABLE;
		ItemZeroValue.steeringWheelAngle = ItemValue.steeringWheelAngle;
	}
	/* 零点校准 */
	ItemValue.steeringWheelAngle -= ItemZeroValue.steeringWheelAngle;

	/* 输出显示 */
	sprintf(value, "%6.1f", ItemValue.steeringWheelAngle);
#if DEVICE_OLED_DISPLAY_ENABLE
	OLED_ShowString(64, 2, value, 6);
#endif
#if DEVICE_BLE_SEND_ENABLE
	BLE_SendBytes(BLE_DATA_TYPE_STEERING_WHEEL_ANGLE, value);
#endif
}

