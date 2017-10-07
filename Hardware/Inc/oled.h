#ifndef __OLED_H
#define __OLED_H

/******************************************************************************/
#include "stm32l1xx_hal.h"
#include "main.h"
#include "stdlib.h"
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
//-----------------OLED端口定义----------------
//#define OLED_CS_Clr()  GPIO_ResetBits(GPIOD,GPIO_Pin_3)//CS
//#define OLED_CS_Set()  GPIO_SetBits(GPIOD,GPIO_Pin_3)
//
//#define OLED_RST_Clr() GPIO_ResetBits(GPIOD,GPIO_Pin_4)//RES
//#define OLED_RST_Set() GPIO_SetBits(GPIOD,GPIO_Pin_4)
//
//#define OLED_DC_Clr() GPIO_ResetBits(GPIOD,GPIO_Pin_5)//DC
//#define OLED_DC_Set() GPIO_SetBits(GPIOD,GPIO_Pin_5)
//
//#define OLED_WR_Clr() GPIO_ResetBits(GPIOG,GPIO_Pin_14)
//#define OLED_WR_Set() GPIO_SetBits(GPIOG,GPIO_Pin_14)
//
//#define OLED_RD_Clr() GPIO_ResetBits(GPIOG,GPIO_Pin_13)
//#define OLED_RD_Set() GPIO_SetBits(GPIOG,GPIO_Pin_13)

//#define OLED_CS_Clr()  HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET)
//#define OLED_CS_Set()  GPIO_SetBits(GPIOD,GPIO_Pin_3)
//
//#define OLED_RST_Clr() GPIO_ResetBits(GPIOD,GPIO_Pin_4)//RES
//#define OLED_RST_Set() GPIO_SetBits(GPIOD,GPIO_Pin_4)

#define OLED_DC_Clr() HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET)
#define OLED_DC_Set() HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET)

#define OLED_SCLK_Clr() HAL_GPIO_WritePin(OLED_CLK_GPIO_Port, OLED_CLK_Pin, GPIO_PIN_RESET)
#define OLED_SCLK_Set() HAL_GPIO_WritePin(OLED_CLK_GPIO_Port, OLED_CLK_Pin, GPIO_PIN_SET)

#define OLED_SDIN_Clr() HAL_GPIO_WritePin(OLED_DOUT_GPIO_Port, OLED_DOUT_Pin, GPIO_PIN_RESET)
#define OLED_SDIN_Set() HAL_GPIO_WritePin(OLED_DOUT_GPIO_Port, OLED_DOUT_Pin, GPIO_PIN_SET)


#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据


//OLED控制用函数
void OLED_WR_Byte(uint8_t dat,uint8_t cmd);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);
void OLED_Fill(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t dot);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size);
void OLED_ShowString(uint8_t x,uint8_t y, uint8_t *p);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
#endif


#if 0
#ifndef __OLED_H
#define __OLED_H

/******************************************************************************/
#include "stm32l1xx_hal.h"
#include "main.h"

/******************************************************************************/
#define OLED_SEND_DATA() \
			HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET)
#define OLED_SEND_CMD() \
			HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET)
#define OLED_CLK_SET() \
			HAL_GPIO_WritePin(OLED_CLK_GPIO_Port, OLED_CLK_Pin, GPIO_PIN_SET)
#define OLED_CLK_RESET() \
			HAL_GPIO_WritePin(OLED_CLK_GPIO_Port, OLED_CLK_Pin, GPIO_PIN_RESET)
#define OLED_DOUT_SET() \
			HAL_GPIO_WritePin(OLED_DOUT_GPIO_Port, OLED_DOUT_Pin, GPIO_PIN_SET)
#define OLED_DOUT_RESET() \
			HAL_GPIO_WritePin(OLED_DOUT_GPIO_Port, OLED_DOUT_Pin, GPIO_PIN_RESET)

#define OLED_COLUMN_MAX			(128)	/* OLED横向最大值 */
#define OLED_ROW_MAX			(64)	/* OLED纵向最大值 */

/******************************************************************************/
typedef enum
{
	OLED_BYTE_TYPE_CMD,
	OLED_BYTE_TYPE_DATA,
} OLED_ByteTypeEnum;

typedef enum
{
	OLED_DISP_TYPE_F8x16,
	OLED_DISP_TYPE_F6x8,
} OLED_DispTypeEnum;

/******************************************************************************/
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowFull(void);

#endif

#endif
