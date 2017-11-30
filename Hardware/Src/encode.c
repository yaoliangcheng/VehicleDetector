#include "encode.h"

/******************************************************************************/
uint16_t Encode_plusCnt = 0;				/* ±‡¬Î∆˜¬ˆ≥Â ˝ */
uint16_t Encode_plusCntOld = 0;				/* ±‡¬Î∆˜æ…¬ˆ≥Â ˝ */
uint16_t Encode_periodCnt = 0;
FunctionalState Encode_processEnable;		/* ±‡¬Î∆˜Process πƒ‹ */

/*******************************************************************************
 *
 */
void ENCODE_Process(void)
{
	if (Encode_processEnable == ENABLE)
	{
		Encode_processEnable = DISABLE;

		if (Encode_plusCnt > Encode_plusCntOld)
		{
			Encode_plusCnt -= Encode_plusCntOld;
		}
		else
		{
			Encode_plusCnt += ENCODE_PERIOD_PLUS_CNT - Encode_plusCntOld;
		}

		Encode_plusCntOld = Encode_plusCnt;

		/* º∆À„ÀŸ∂» */
		BrakeDistance_Speed = ((ENCODE_WHEEL_PERIMETER / ENCODE_PERIOD_PLUS_CNT)
								* Encode_plusCnt) / 0.1;
		/* º∆À„æ‡¿Î */
		BrakeDistance_Distance = (Encode_periodCnt * ENCODE_WHEEL_PERIMETER)
				+ ((ENCODE_WHEEL_PERIMETER / ENCODE_PERIOD_PLUS_CNT)
				* Encode_plusCnt);


	}



}






