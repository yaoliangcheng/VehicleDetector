#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"
//OLED的显存
//存放格式如下.
//[0]0 1 2 3 ... 127
//[1]0 1 2 3 ... 127
//[2]0 1 2 3 ... 127
//[3]0 1 2 3 ... 127
//[4]0 1 2 3 ... 127
//[5]0 1 2 3 ... 127
//[6]0 1 2 3 ... 127
//[7]0 1 2 3 ... 127

/*******************************************************************************
 * @brief 向SSD1106写入一个字节。
 * @param dat:要写入的数据或者命令
 * @param type：数据类型
 */
void OLED_WriteByte(uint8_t dat, OLED_ValueTypeEnum type)
{
	uint8_t i;

	if (OLED_VALUE_TYPE_DATA == type)
		OLED_DC_SET();
	else
		OLED_DC_RESET();

	for (i = 0; i < 8; i++)
	{
		OLED_CLK_RESET();
		if (dat & 0x80)
			OLED_DIN_SET();
		else
			OLED_DIN_RESET();
		OLED_CLK_SET();
		dat <<= 1;
	}
	OLED_DC_SET();
}

/*******************************************************************************
 * @brief 设置OLED显示坐标
 */
void OLED_SetPos(uint8_t x, uint8_t y)
{
	OLED_WriteByte(0xb0 + y, 				 OLED_VALUE_TYPE_CMD);
	OLED_WriteByte(((x & 0xf0) >> 4) | 0x10, OLED_VALUE_TYPE_CMD);
	OLED_WriteByte( (x & 0x0f)       | 0x01, OLED_VALUE_TYPE_CMD);
}
/*******************************************************************************
 * @brief 开启OLED显示
 */
void OLED_DisplayOn(void)
{
	OLED_WriteByte(0X8D, OLED_VALUE_TYPE_CMD);  //SET DCDC命令
	OLED_WriteByte(0X14, OLED_VALUE_TYPE_CMD);  //DCDC ON
	OLED_WriteByte(0XAF, OLED_VALUE_TYPE_CMD);  //DISPLAY ON
}

/*******************************************************************************
 * @brief 关闭OLED显示
 */
void OLED_DisplayOff(void)
{
	OLED_WriteByte(0X8D, OLED_VALUE_TYPE_CMD);  //SET DCDC命令
	OLED_WriteByte(0X10, OLED_VALUE_TYPE_CMD);  //DCDC OFF
	OLED_WriteByte(0XAE, OLED_VALUE_TYPE_CMD);  //DISPLAY OFF
}

/*******************************************************************************
 * @brief 清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!
 */
void OLED_Clear(void)
{
	uint8_t i, n;
	for(i = 0; i < 8; i++)
	{
		OLED_WriteByte(0xb0 + i, OLED_VALUE_TYPE_CMD);    //设置页地址（0~7）
		OLED_WriteByte(0x00,     OLED_VALUE_TYPE_CMD);    //设置显示位置—列低地址
		OLED_WriteByte(0x10,     OLED_VALUE_TYPE_CMD);    //设置显示位置—列高地址
		for(n = 0; n < 128; n++)
			OLED_WriteByte(0, OLED_VALUE_TYPE_DATA);
	}
}

/*******************************************************************************
 * @brief 在指定位置显示一个字符,包括部分字符
 * @param x:0~127
 * @param y:0~63
 *
 */
//mode:0,反白显示;1,正常显示
//size:选择字体 16/12
void OLED_ShowChar(uint8_t x, uint8_t y, char chr)
{
	unsigned char c=0,i=0;
	c = chr - ' ';				//得到偏移后的值
	if(x > (OLED_COLUMN_MAX - 1))
	{
		x = 0;
		y = y + 2;
	}

	if(SIZE == 16)
	{
		OLED_SetPos(x,y);
		for(i = 0; i < 8; i++)
			OLED_WriteByte(F8X16[c * 16 + i],     OLED_VALUE_TYPE_DATA);
		OLED_SetPos(x,y+1);
		for(i = 0; i < 8; i++)
			OLED_WriteByte(F8X16[c * 16 + i + 8], OLED_VALUE_TYPE_DATA);
	}
	else
	{
		OLED_SetPos(x, y + 1);
		for(i = 0; i < 6; i++)
			OLED_WriteByte(F6x8[c][i], OLED_VALUE_TYPE_DATA);
	}
}

/*******************************************************************************
 * @brief m^n函数
 */
uint32_t OLED_Pow(uint8_t m, uint8_t n)
{
	uint32_t result = 1;
	while(n--)
	{
		result *= m;
	}
	return result;
}

/*******************************************************************************
 * @brief 显示2个数字
 * @param x,y :起点坐标
 * @param len :数字的位数
 * @param size:字体大小
 * @param num:数值(0~4294967295);
 *
 */
//mode:模式	0,填充模式;1,叠加模式
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{
	uint8_t t,temp;
	uint8_t enshow = 0;

	for(t = 0; t < len; t++)
	{
		temp = (num / OLED_Pow(10, len - t - 1)) % 10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ');
				continue;
			}else enshow=1;

		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0');
	}
}

/*******************************************************************************
 * @brief 显示一个字符号串
 */
void OLED_ShowString(uint8_t x, uint8_t y, char* chr, uint8_t size)
{
	while (size--)
	{
		OLED_ShowChar(x, y, *chr);
		x += 8;								/* 定位到下一位置 */
		if (x > 120)						/* 需要换行 */
		{
			x = 0;
			y += 2;
		}
		chr++;
	}
}

/*******************************************************************************
 * @brief 显示汉字
 */
void OLED_ShowChinese(uint8_t x, uint8_t y, char* fontLib)
{
	uint8_t t;

	OLED_SetPos(x,y);
    for(t = 0; t < 16; t++)
	{
		OLED_WriteByte(*fontLib, OLED_VALUE_TYPE_DATA);
		fontLib++;
    }
	OLED_SetPos(x, y + 1);
    for(t = 0; t < 16; t++)
	{
		OLED_WriteByte(*fontLib, OLED_VALUE_TYPE_DATA);
		fontLib++;
    }
}

/*******************************************************************************
 *
 *
 */
void OLED_ShowChineseString(uint8_t x, uint8_t y, char* chinese, uint8_t size)
{
	while (size--)
	{
		OLED_ShowChinese(x, y, chinese);
		x += 16;								/* 定位到下一位置 */
		if (x > 112)						/* 需要换行 */
		{
			x = 0;
			y += 2;
		}
		chinese += 32;
	}
}

/*******************************************************************************
 * @brief 在指定位置显示浮点型数值
 */
void OLED_ShowFloatValue(uint8_t x, uint8_t y, float value)
{
	char dataString[6];
	uint8_t size = 0;

	size = sprintf(dataString, "%5.1f", value);
	OLED_ShowString(x, y, dataString, size);
}

/*******************************************************************************
 * @brief 显示显示BMP图片
 * 128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7
 */
void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1,uint8_t BMP[])
{
	unsigned int j=0;
	unsigned char x,y;

	if (y1 % 8 == 0)
		y = y1 / 8;
	else
		y = y1 / 8 + 1;
	for (y = y0; y < y1; y++)
	{
		OLED_SetPos(x0, y);
		for(x = x0; x < x1; x++)
		{
			OLED_WriteByte(BMP[j++], OLED_VALUE_TYPE_DATA);
		}
	}
}

/*******************************************************************************
 *
 */
void OLED_Init(void)
{
	OLED_CS_RESET();
	OLED_RST_SET();
	HAL_Delay(100);
	OLED_RST_RESET();
	HAL_Delay(100);
	OLED_RST_SET();

	OLED_WriteByte(0xAE, OLED_VALUE_TYPE_CMD);//--turn off oled panel
	OLED_WriteByte(0x00, OLED_VALUE_TYPE_CMD);//---set low column address
	OLED_WriteByte(0x10, OLED_VALUE_TYPE_CMD);//---set high column address
	OLED_WriteByte(0x40, OLED_VALUE_TYPE_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	OLED_WriteByte(0x81, OLED_VALUE_TYPE_CMD);//--set contrast control register
	OLED_WriteByte(0xCF, OLED_VALUE_TYPE_CMD); // Set SEG Output Current Brightness
	OLED_WriteByte(0xA1, OLED_VALUE_TYPE_CMD);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
	OLED_WriteByte(0xC8, OLED_VALUE_TYPE_CMD);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
	OLED_WriteByte(0xA6, OLED_VALUE_TYPE_CMD);//--set normal display
	OLED_WriteByte(0xA8, OLED_VALUE_TYPE_CMD);//--set multiplex ratio(1 to 64)
	OLED_WriteByte(0x3f, OLED_VALUE_TYPE_CMD);//--1/64 duty
	OLED_WriteByte(0xD3, OLED_VALUE_TYPE_CMD);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	OLED_WriteByte(0x00, OLED_VALUE_TYPE_CMD);//-not offset
	OLED_WriteByte(0xd5, OLED_VALUE_TYPE_CMD);//--set display clock divide ratio/oscillator frequency
	OLED_WriteByte(0x80, OLED_VALUE_TYPE_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
	OLED_WriteByte(0xD9, OLED_VALUE_TYPE_CMD);//--set pre-charge period
	OLED_WriteByte(0xF1, OLED_VALUE_TYPE_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	OLED_WriteByte(0xDA, OLED_VALUE_TYPE_CMD);//--set com pins hardware configuration
	OLED_WriteByte(0x12, OLED_VALUE_TYPE_CMD);
	OLED_WriteByte(0xDB, OLED_VALUE_TYPE_CMD);//--set vcomh
	OLED_WriteByte(0x40, OLED_VALUE_TYPE_CMD);//Set VCOM Deselect Level
	OLED_WriteByte(0x20, OLED_VALUE_TYPE_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
	OLED_WriteByte(0x02, OLED_VALUE_TYPE_CMD);//
	OLED_WriteByte(0x8D, OLED_VALUE_TYPE_CMD);//--set Charge Pump enable/disable
	OLED_WriteByte(0x14, OLED_VALUE_TYPE_CMD);//--set(0x10) disable
	OLED_WriteByte(0xA4, OLED_VALUE_TYPE_CMD);// Disable Entire Display On (0xa4/0xa5)
	OLED_WriteByte(0xA6, OLED_VALUE_TYPE_CMD);// Disable Inverse Display On (0xa6/a7)
	OLED_WriteByte(0xAF, OLED_VALUE_TYPE_CMD);//--turn on oled panel

	OLED_WriteByte(0xAF, OLED_VALUE_TYPE_CMD); /*display ON*/
	OLED_Clear();
	OLED_SetPos(0,0);
}





























