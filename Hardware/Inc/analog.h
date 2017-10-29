#ifndef __ANALOG_H
#define __ANALOG_H

/******************************************************************************/
#include "stm32l1xx_hal.h"
#include "main.h"

#include "public.h"
#include "adc.h"

/******************************************************************************/
#define ANALOG_ADC					(hadc)
#define ANALOG_SAMPLE_NUMB			(20)

/******************************************************************************/
void ANALOG_ConvertEnable(void);
void ANALOG_ConvertDisable(void);
void ANALOG_Process(void);

#endif
