#include "public.h"

/*******************************************************************************
 *
 */
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&DEBUG_UART, (uint8_t *)&ch, 1, 0xffff);
	return ch;
}

/*******************************************************************************
 *
 */
HAL_StatusTypeDef UART_DMAIdleConfig(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	uint32_t *tmp;

	if((pData == NULL ) || (Size == 0))
	{
	  return HAL_ERROR;
	}

	huart->pRxBuffPtr = pData;
	huart->RxXferSize = Size;

	huart->ErrorCode = HAL_UART_ERROR_NONE;

	/* Enable the DMA channel */
	tmp = (uint32_t*)&pData;
	HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, *(uint32_t*)tmp, Size);

	/* Enable the DMA transfer for the receiver request by setting the DMAR bit
	   in the UART CR3 register */
	SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

	return HAL_OK;
}

/*******************************************************************************
 * @brief 计算校验和
 */
uint8_t CheckSum(uint8_t* buffer, uint8_t size)
{
	uint8_t sum = 0;

	while (size--)
	{
		sum += *buffer;
		buffer++;
	}

	return sum;
}

/*******************************************************************************
 *
 */
void Double2Format(double data, uint8_t* pBuffer)
{
	int16_t temp1 = 0;
	uint8_t temp2 = 0;

	temp1 = (int16_t)data;
	temp2 = (uint8_t)((data - temp1) * 10);

	*pBuffer       = temp1 & 0x00FF;
	*(pBuffer + 1) = (temp1 & 0xFF00) >> 8;
	*(pBuffer + 2) = temp2;
}


















