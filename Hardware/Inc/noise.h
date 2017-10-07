#ifndef __NOISE_H
#define __NOISE_H

/******************************************************************************/
#include "stm32l1xx_hal.h"
#include "main.h"

#include "public.h"

#include "usart.h"

/******************************************************************************/
#define NOISE_UART 						(huart2)
#define NOISE_UART_RX_BYTE_MAX			(20)
#define NOISE_UART_DMA_RX_GL_FLAG		(DMA_FLAG_GL6)

#define NOISE_MODULE_ADDR				(0x01)		/* 噪声模块地址 */
#define NOISE_MODULE_CMD_TYPE			(0x03)		/* 命令字 */
#define NOISE_MODULE_DATA_LENGTH		(0x02)		/* 数据长度 */
#define NOISE_MODULE_RANGE_DB_MAX		(120)		/* 噪声模块量程最大值120dB */
#define NOISE_MODULE_RANGE_DB_MIN		(30)		/* 噪声模块量程最小值30dB */
#define NOISE_MODULE_RANGE_DB \
						(NOISE_MODULE_RANGE_DB_MAX - NOISE_MODULE_RANGE_DB_MIN)


/******************************************************************************/
#pragma pack(1)
typedef struct
{
	uint8_t addr;						/* 地址位 */
	uint8_t cmdType;					/* 命令字 */
	int16_t dataLength;					/* 数据长度 */
	int16_t dataH;						/* 数据高位 */
	int16_t dataL;						/* 数据低位 */
	int16_t crcL;						/* CRC低位 */
	uint8_t crcH;						/* CRC高位 */
} NOISE_RecvStrcutTypedef;

typedef struct
{
	NOISE_RecvStrcutTypedef buffer;
	uint8_t size;										/* 接收数据的长度 */
	FunctionalState status;								/* 接收状态 */
} NOISE_RecvTypedef;

#pragma pack()

/******************************************************************************/
void NOISE_Init(void);
void NOISE_Require(void);
void NOISE_Process(void);
void NOISE_UartIdleDeal(void);

#endif
