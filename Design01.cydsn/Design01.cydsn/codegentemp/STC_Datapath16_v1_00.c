/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "STC_Datapath16_v1_00.h"

void STC_Datapath16_v1_00_Start()
{
	#if 0 > 0
		STC_Datapath16_v1_00_AUX_CTL_A_REG |= (STC_Datapath16_v1_00_F0_LEVEL_MODE_1 << STC_Datapath16_v1_00_F0_LEVEL_SHIFT);
		STC_Datapath16_v1_00_AUX_CTL_B_REG |= (STC_Datapath16_v1_00_F0_LEVEL_MODE_1 << STC_Datapath16_v1_00_F0_LEVEL_SHIFT);
	#endif
	
	#if 0 > 0
		STC_Datapath16_v1_00_AUX_CTL_A_REG |= (STC_Datapath16_v1_00_F1_LEVEL_MODE_1 << STC_Datapath16_v1_00_F1_LEVEL_SHIFT);
		STC_Datapath16_v1_00_AUX_CTL_B_REG |= (STC_Datapath16_v1_00_F1_LEVEL_MODE_1 << STC_Datapath16_v1_00_F1_LEVEL_SHIFT);
	#endif
	
	#if 0 > 0
		STC_Datapath16_v1_00_AUX_CTL_A_REG |= (STC_Datapath16_v1_00_F0_SINGLE_BUFFER_ENABLE << STC_Datapath16_v1_00_F0_SINGLE_BUFFER_SHIFT);
		STC_Datapath16_v1_00_AUX_CTL_B_REG |= (STC_Datapath16_v1_00_F0_SINGLE_BUFFER_ENABLE << STC_Datapath16_v1_00_F0_SINGLE_BUFFER_SHIFT);
	#endif
	
	#if 0 > 0
		STC_Datapath16_v1_00_AUX_CTL_A_REG |= (STC_Datapath16_v1_00_F1_SINGLE_BUFFER_ENABLE << STC_Datapath16_v1_00_F1_SINGLE_BUFFER_SHIFT);
		STC_Datapath16_v1_00_AUX_CTL_B_REG |= (STC_Datapath16_v1_00_F1_SINGLE_BUFFER_ENABLE << STC_Datapath16_v1_00_F1_SINGLE_BUFFER_SHIFT);
	#endif
	
	return;
}

/* [] END OF FILE */
