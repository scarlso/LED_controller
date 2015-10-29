/***************************************************************************
* File Name: MultAcc_1_DMA_IN_A_dma.c  
* Version 1.70
*
*  Description:
*   Provides an API for the DMAC component. The API includes functions
*   for the DMA controller, DMA channels and Transfer Descriptors.
*
*
*   Note:
*     This module requires the developer to finish or fill in the auto
*     generated funcions and setup the dma channel and TD's.
*
********************************************************************************
* Copyright 2008-2010, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/
#include <CYLIB.H>
#include <CYDMAC.H>
#include <MultAcc_1_DMA_IN_A_dma.H>



/****************************************************************************
* 
* The following defines are available in Cyfitter.h
* 
* 
* 
* MultAcc_1_DMA_IN_A__DRQ_CTL_REG
* 
* 
* MultAcc_1_DMA_IN_A__DRQ_NUMBER
* 
* Number of TD's used by this channel.
* MultAcc_1_DMA_IN_A__NUMBEROF_TDS
* 
* Priority of this channel.
* MultAcc_1_DMA_IN_A__PRIORITY
* 
* True if MultAcc_1_DMA_IN_A_TERMIN_SEL is used.
* MultAcc_1_DMA_IN_A__TERMIN_EN
* 
* TERMIN interrupt line to signal terminate.
* MultAcc_1_DMA_IN_A__TERMIN_SEL
* 
* 
* True if MultAcc_1_DMA_IN_A_TERMOUT0_SEL is used.
* MultAcc_1_DMA_IN_A__TERMOUT0_EN
* 
* 
* TERMOUT0 interrupt line to signal completion.
* MultAcc_1_DMA_IN_A__TERMOUT0_SEL
* 
* 
* True if MultAcc_1_DMA_IN_A_TERMOUT1_SEL is used.
* MultAcc_1_DMA_IN_A__TERMOUT1_EN
* 
* 
* TERMOUT1 interrupt line to signal completion.
* MultAcc_1_DMA_IN_A__TERMOUT1_SEL
* 
****************************************************************************/


/* Zero based index of MultAcc_1_DMA_IN_A dma channel */
uint8 MultAcc_1_DMA_IN_A_DmaHandle = DMA_INVALID_CHANNEL;

/*********************************************************************
* Function Name: uint8 MultAcc_1_DMA_IN_A_DmaInitalize
**********************************************************************
* Summary:
*   Allocates and initialises a channel of the DMAC to be used by the
*   caller.
*
* Parameters:
*   BurstCount.
*       
*       
*   ReqestPerBurst.
*       
*       
*   UpperSrcAddress.
*       
*       
*   UpperDestAddress.
*       
*
* Return:
*   The channel that can be used by the caller for DMA activity.
*   DMA_INVALID_CHANNEL (0xFF) if there are no channels left. 
*
*
*******************************************************************/
uint8 MultAcc_1_DMA_IN_A_DmaInitialize(uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) 
{

    /* Allocate a DMA channel. */
    MultAcc_1_DMA_IN_A_DmaHandle = (uint8)MultAcc_1_DMA_IN_A__DRQ_NUMBER;

    /* Configure the channel. */
    (void)CyDmaChSetConfiguration(MultAcc_1_DMA_IN_A_DmaHandle,
                                  BurstCount,
                                  ReqestPerBurst,
                                  (uint8)MultAcc_1_DMA_IN_A__TERMOUT0_SEL,
                                  (uint8)MultAcc_1_DMA_IN_A__TERMOUT1_SEL,
                                  (uint8)MultAcc_1_DMA_IN_A__TERMIN_SEL);

    /* Set the extended address for the transfers */
    (void)CyDmaChSetExtendedAddress(MultAcc_1_DMA_IN_A_DmaHandle, UpperSrcAddress, UpperDestAddress);

    /* Set the priority for this channel */
    (void)CyDmaChPriority(MultAcc_1_DMA_IN_A_DmaHandle, (uint8)MultAcc_1_DMA_IN_A__PRIORITY);
    
    return MultAcc_1_DMA_IN_A_DmaHandle;
}

/*********************************************************************
* Function Name: void MultAcc_1_DMA_IN_A_DmaRelease
**********************************************************************
* Summary:
*   Frees the channel associated with MultAcc_1_DMA_IN_A.
*
*
* Parameters:
*   void.
*
*
*
* Return:
*   void.
*
*******************************************************************/
void MultAcc_1_DMA_IN_A_DmaRelease(void) 
{
    /* Disable the channel */
    (void)CyDmaChDisable(MultAcc_1_DMA_IN_A_DmaHandle);
}

