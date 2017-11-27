#ifndef __OLED_H
#define __OLED_H

/******************************************************************************/
#include "stm32l1xx_hal.h"
#include "main.h"

//OLED模式设置
//0:4线串行模式
//1:并行8080模式
#define OLED_MODE 0
#define SIZE 16
#define XLevelL		0x00
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF
#define X_WIDTH 	128
#define Y_WIDTH 	64

#define OLED_COLUMN_MAX 			(128)
#define OLED_ROW_MAX				(64)

/********************** OLED端口定义 *********************************************/
#define OLED_CS_RESET()  HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET)
#define OLED_CS_SET()    HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET)

#define OLED_DC_RESET()  HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET)
#define OLED_DC_SET()    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET)

#define OLED_RST_RESET() HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET)
#define OLED_RST_SET()   HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_SET)

#define OLED_DIN_RESET() HAL_GPIO_WritePin(OLED_DIN_GPIO_Port, OLED_DIN_Pin, GPIO_PIN_RESET)
#define OLED_DIN_SET()   HAL_GPIO_WritePin(OLED_DIN_GPIO_Port, OLED_DIN_Pin, GPIO_PIN_SET)

#define OLED_CLK_RESET() HAL_GPIO_WritePin(OLED_CLK_GPIO_Port, OLED_CLK_Pin, GPIO_PIN_RESET)
#define OLED_CLK_SET()   HAL_GPIO_WritePin(OLED_CLK_GPIO_Port, OLED_CLK_Pin, GPIO_PIN_SET)

#define CHINESE_FONT_SIZE					(32)

/******************************************************************************/
/* OLED发送数值类型：0：命令，1：数据 */
typedef enum
{
	OLED_VALUE_TYPE_CMD,
	OLED_VALUE_TYPE_DATA,
} OLED_ValueTypeEnum;

/******************************************************************************/
void OLED_Init(void);
void OLED_ShowString(uint8_t x, uint8_t y, char* chr, uint8_t size);
void OLED_ShowFloatValue(uint8_t x, uint8_t y, float value);
void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1,uint8_t BMP[]);
void OLED_ShowChineseString(uint8_t x, uint8_t y, char* chinese, uint8_t size);
void OLED_Clear(void);
#endif




