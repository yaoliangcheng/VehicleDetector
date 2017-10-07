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
#define DEVICE_TEST_MODE_ENABLE			(1) /* 1：测试模式 ，2：协议模式*/

/******************************************************************************/
HAL_StatusTypeDef UART_DMAIdleConfig(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);


#endif
