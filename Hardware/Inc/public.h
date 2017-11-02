#ifndef __PUBLIC_H
#define __PUBLIC_H

/******************************************************************************/
#include "stm32l1xx_hal.h"
#include "main.h"

#include <string.h>
#include <stdio.h>
//#include <malloc.h>
#include <math.h>
#include <stdlib.h>

#include "usart.h"

/******************************************************************************/
#define USE_FULL_LL_DRIVER
#define DEBUG_UART					(huart3)

/******************************************************************************/
#define DEVICE_TEST_MODE_ENABLE			(0) /* 1：测试模式 ，2：协议模式*/

#define DEVICE_OLED_DISPLAY_ENABLE		(1)
#define DEVICE_BLE_SEND_ENABLE			(1)

/******************************************************************************/
HAL_StatusTypeDef UART_DMAIdleConfig(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
uint8_t CheckSum(uint8_t* buffer, uint8_t size);

#endif
