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

#include "MultAcc_1.h"

#if 1

	/* DMA Configuration for DMA_IN_A */
	#define MultAcc_1_DMA_IN_A_BYTES_PER_BURST 4
	#define MultAcc_1_DMA_IN_A_REQUEST_PER_BURST 1
	#define MultAcc_1_DMA_IN_A_SRC_BASE (CYDEV_SRAM_BASE)
	#define MultAcc_1_DMA_IN_A_DST_BASE (CYDEV_PERIPH_BASE)

	/* DMA Configuration for DMA_IN_B */
	#define MultAcc_1_DMA_IN_B_BYTES_PER_BURST 4
	#define MultAcc_1_DMA_IN_B_REQUEST_PER_BURST 1
	#define MultAcc_1_DMA_IN_B_SRC_BASE (CYDEV_SRAM_BASE)
	#define MultAcc_1_DMA_IN_B_DST_BASE (CYDEV_PERIPH_BASE)
	
	MultAcc_1_ARRAY MultAcc_1_a;
	MultAcc_1_ARRAY MultAcc_1_b;

	//int32 MultAcc_1_data_a[8];
	//int32 MultAcc_1_data_b[8];

	uint8 MultAcc_1_DMA_IN_A_Chan;
	uint8 MultAcc_1_DMA_IN_A_TD[1];
	
	uint8 MultAcc_1_DMA_IN_B_Chan;
	uint8 MultAcc_1_DMA_IN_B_TD[1];
	
	uint8 MultAcc_1_DMAInitialized = 0;

#endif

void MultAcc_1_Init()
{
	MultAcc_1_MACC_Init();
	#if 1
		MultAcc_1_DMAInit();
	#endif
}

void MultAcc_1_Enable()
{
	MultAcc_1_MACC_Enable();
	#if 1
		MultAcc_1_DMAEnable();
	#endif
}

void MultAcc_1_Start()
{
	MultAcc_1_MACC_Start();
	#if 1
		MultAcc_1_DMAInit();
		MultAcc_1_DMAEnable();
	#endif
}

void MultAcc_1_Stop()
{
	MultAcc_1_MACC_Stop();
	#if 1
		MultAcc_1_DMADisable();
	#endif
}

//void MultAcc_1_Sleep()
//{
//	MultAcc_1_MACC_Sleep();
//}
//
//void MultAcc_1_Wakeup()
//{
//	MultAcc_1_MACC_Wakeup();
//}

#if 1

void MultAcc_1_DMAInit(void)
{
	
	if(MultAcc_1_DMAInitialized == 0)
	{
		MultAcc_1_DMAInitialized = 1;

		MultAcc_1_DMA_IN_A_Chan = MultAcc_1_DMA_IN_A_DmaInitialize(MultAcc_1_DMA_IN_A_BYTES_PER_BURST, MultAcc_1_DMA_IN_A_REQUEST_PER_BURST, 
		    HI16(MultAcc_1_DMA_IN_A_SRC_BASE), HI16(MultAcc_1_DMA_IN_A_DST_BASE));
		MultAcc_1_DMA_IN_A_TD[0] = CyDmaTdAllocate();
		#if 1
			CyDmaTdSetConfiguration(MultAcc_1_DMA_IN_A_TD[0], 8*sizeof(uint32), MultAcc_1_DMA_IN_A_TD[0], TD_SWAP_EN | TD_SWAP_SIZE4 | MultAcc_1_DMA_IN_A__TD_TERMOUT_EN | TD_INC_SRC_ADR);
		#else
			CyDmaTdSetConfiguration(MultAcc_1_DMA_IN_A_TD[0], 8*sizeof(uint32), MultAcc_1_DMA_IN_A_TD[0], MultAcc_1_DMA_IN_A__TD_TERMOUT_EN | TD_INC_SRC_ADR);
		#endif
		CyDmaTdSetAddress(MultAcc_1_DMA_IN_A_TD[0], LO16((uint32)&MultAcc_1_a.dat[0]), LO16((uint32)MultAcc_1_IN_A32_PTR));  // 
		CyDmaChSetInitialTd(MultAcc_1_DMA_IN_A_Chan, MultAcc_1_DMA_IN_A_TD[0]);
		
		CyDmaChRoundRobin(MultAcc_1_DMA_IN_A_Chan, 1);

		MultAcc_1_DMA_IN_B_Chan = MultAcc_1_DMA_IN_B_DmaInitialize(MultAcc_1_DMA_IN_B_BYTES_PER_BURST, MultAcc_1_DMA_IN_B_REQUEST_PER_BURST, 
		    HI16(MultAcc_1_DMA_IN_B_SRC_BASE), HI16(MultAcc_1_DMA_IN_B_DST_BASE));
		MultAcc_1_DMA_IN_B_TD[0] = CyDmaTdAllocate();
		#if 1
			CyDmaTdSetConfiguration(MultAcc_1_DMA_IN_B_TD[0], 8*sizeof(uint32), MultAcc_1_DMA_IN_B_TD[0], TD_SWAP_EN | TD_SWAP_SIZE4 | MultAcc_1_DMA_IN_B__TD_TERMOUT_EN | TD_INC_SRC_ADR);
		#else
			CyDmaTdSetConfiguration(MultAcc_1_DMA_IN_B_TD[0], 8*sizeof(uint32), MultAcc_1_DMA_IN_B_TD[0], MultAcc_1_DMA_IN_B__TD_TERMOUT_EN | TD_INC_SRC_ADR);
		#endif
		CyDmaTdSetAddress(MultAcc_1_DMA_IN_B_TD[0], LO16((uint32)&MultAcc_1_b.dat[0]), LO16((uint32)MultAcc_1_IN_B32_PTR)); // 
		CyDmaChSetInitialTd(MultAcc_1_DMA_IN_B_Chan, MultAcc_1_DMA_IN_B_TD[0]);
		
		CyDmaChRoundRobin(MultAcc_1_DMA_IN_B_Chan, 1);
		
	}

}

void MultAcc_1_DMAEnable(void)
{
	CyDmaChSetRequest(MultAcc_1_DMA_IN_A_Chan, CPU_TERM_CHAIN);
	CyDmaChEnable(MultAcc_1_DMA_IN_A_Chan, 1);
	CyDmaChEnable(MultAcc_1_DMA_IN_A_Chan, 1);

	CyDmaChSetRequest(MultAcc_1_DMA_IN_B_Chan, CPU_TERM_CHAIN);
	CyDmaChEnable(MultAcc_1_DMA_IN_B_Chan, 1);
	CyDmaChEnable(MultAcc_1_DMA_IN_B_Chan, 1);
	
	MultAcc_1_Status_InterruptEnable();
}

void MultAcc_1_DMADisable(void)
{
	CyDmaChSetRequest(MultAcc_1_DMA_IN_A_Chan, CPU_TERM_CHAIN);

	CyDmaChSetRequest(MultAcc_1_DMA_IN_B_Chan, CPU_TERM_CHAIN);
	
	MultAcc_1_Status_InterruptDisable();
}

void MultAcc_1_ProcessBlock(uint8 reset_accum)
{
	MultAcc_1_CheckStatus(MultAcc_1_RESET_STATUS);
	
	if(reset_accum > 0)
	{
		MultAcc_1_Control_Write(MultAcc_1_DMA_GO | MultAcc_1_NRQ_RESET | MultAcc_1_MACC_RESET);
	}
	else
	{
		MultAcc_1_Control_Write(MultAcc_1_DMA_GO | MultAcc_1_NRQ_RESET);
	}
	
	MultAcc_1_Control_Write(0);
}

uint8 MultAcc_1_CheckStatus(uint8 flag_check)
{
	static uint8 status_flags;
	
	status_flags |= (MultAcc_1_Status_Read() & ~MultAcc_1_RESET_STATUS);
	
	if((flag_check & MultAcc_1_RESET_STATUS) > 0)
	{
		status_flags = 0;
		return 0;
	}
	
	if((status_flags & flag_check) > 0)
	{
		status_flags = status_flags & ~flag_check;
		return flag_check;
	}
	
	return 0;
}

#endif

/*******************************************************************************
* Function Name: MultAcc_1_LoadInput
********************************************************************************
*
* Summary:
*  This function loads an input value into the selected input register.
*
* Parameters:
*  channel: Use either MultAcc_1_A (1) or
*           MultAcc_1_B (0) asarguments to the function;
*  sample: The 24-bit, right justified input sample.
*
* Return:
*  None.
*
* Note:
*  The write order is important. When the high byte is loaded, the DFB sets
*  the input ready bit. Pay attention to byte order if coherency or data
*  alignment is changed.
*
*******************************************************************************/
#if !1

void MultAcc_1_LoadInput(int32 A, int32 B) 
{
    uint32 value_A, value_B;

    value_A = (uint32) A;
	value_B = (uint32) B;

    /* Writes the STAGE-"H" reg last as it signals a complete wrote to the DFB.*/

    MultAcc_1_STAGEA_REG  = (uint8) value_A;
    MultAcc_1_STAGEAM_REG = (uint8) (value_A >> 8u);
    MultAcc_1_STAGEAH_REG = (uint8) (value_A >> 16u);

    MultAcc_1_STAGEB_REG  = (uint8) value_B;
    MultAcc_1_STAGEBM_REG = (uint8) (value_B >> 8u);
    MultAcc_1_STAGEBH_REG = (uint8) (value_B >> 16u);

}

#endif

/*******************************************************************************
* Function Name: MultAcc_1_GetOutput
********************************************************************************
*
* Summary:
*  This function gets the value from one of the DFB Output Holding Registers.
*
* Parameters:
*  None
*
* Return:
*  The current output value in the holding register. This is
*  a 24-bit number packed into the least-significant 3 bytes of the output word
*
* Note:
*  Because of the architecture of the DFB, any value read from the Holding A or
*  B registers will be MSB aligned unless shifted otherwise by the Datapath
*  shifter. Pay attention to byte order if coherency or data alignment is
*  changed.
*
*******************************************************************************/
int32 MultAcc_1_GetResult() 
{
    uint8 data0;
    uint8 data1;
    uint8 data2;
    uint8 data3;
    uint32 output;
	
	// due to double buffer acting as a 2 element FIFO, I have to read the 
	// high byte once to clear out the "first" result, then read again
	// to get the most recent result.
	data2 = MultAcc_1_HOLDAH_REG;

    /* Reads the HOLD-"H" reg last as it signals a complete read to the DFB.*/
    data0 = MultAcc_1_HOLDA_REG;
    data1 = MultAcc_1_HOLDAM_REG;
    data2 = MultAcc_1_HOLDAH_REG;
    data3 = ((data2 & 0x80u) == 0x80u) ? 0xFFu : 0x00u;

    output = ((((((uint32) data3 << 8u) | data2) << 8u) | data1) << 8u) | data0;

    return ((int32) output);
}

float MultAcc_1_Q23ToFloat(int32 input)
{
	float output;
	
	// limit the input range to the valid values
	if(input > 8388607)
	{
		input = 8388607;
	}
	if(input < -8388608)
	{
		input = -8388608;
	}
	
	output = ((float) input) / 8388608.0;
	
	return output;
}

int32 MultAcc_1_FloatToQ23(float input)
{
	int32 output;
	
	// limit the input range to the valid values
	if(input > (1.0-0.00000011920928955078))
	{
		input = (1-0.00000011920928955078);
	}
	if(input < -1.0)
	{
		input = -1.0;
	}
	
	output = ((int32) (input * 8388608.0 + 0.5));
	
	return output;
}

/* [] END OF FILE */
