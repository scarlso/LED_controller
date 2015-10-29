/*******************************************************************************
* File Name: MultAcc_1_Status.c  
* Version 1.80
*
* Description:
*  This file contains API to enable firmware to read the value of a Status 
*  Register.
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "MultAcc_1_Status.h"

#if !defined(MultAcc_1_Status_sts_intr_sts_reg__REMOVED) /* Check for removal by optimization */


/*******************************************************************************
* Function Name: MultAcc_1_Status_Read
********************************************************************************
*
* Summary:
*  Reads the current value assigned to the Status Register.
*
* Parameters:
*  None.
*
* Return:
*  The current value in the Status Register.
*
*******************************************************************************/
uint8 MultAcc_1_Status_Read(void) 
{ 
    return MultAcc_1_Status_Status;
}


/*******************************************************************************
* Function Name: MultAcc_1_Status_InterruptEnable
********************************************************************************
*
* Summary:
*  Enables the Status Register interrupt.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void MultAcc_1_Status_InterruptEnable(void) 
{
    uint8 interruptState;
    interruptState = CyEnterCriticalSection();
    MultAcc_1_Status_Status_Aux_Ctrl |= MultAcc_1_Status_STATUS_INTR_ENBL;
    CyExitCriticalSection(interruptState);
}


/*******************************************************************************
* Function Name: MultAcc_1_Status_InterruptDisable
********************************************************************************
*
* Summary:
*  Disables the Status Register interrupt.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void MultAcc_1_Status_InterruptDisable(void) 
{
    uint8 interruptState;
    interruptState = CyEnterCriticalSection();
    MultAcc_1_Status_Status_Aux_Ctrl &= (uint8)(~MultAcc_1_Status_STATUS_INTR_ENBL);
    CyExitCriticalSection(interruptState);
}


/*******************************************************************************
* Function Name: MultAcc_1_Status_WriteMask
********************************************************************************
*
* Summary:
*  Writes the current mask value assigned to the Status Register.
*
* Parameters:
*  mask:  Value to write into the mask register.
*
* Return:
*  None.
*
*******************************************************************************/
void MultAcc_1_Status_WriteMask(uint8 mask) 
{
    #if(MultAcc_1_Status_INPUTS < 8u)
    	mask &= (uint8)((((uint8)1u) << MultAcc_1_Status_INPUTS) - 1u);
	#endif /* End MultAcc_1_Status_INPUTS < 8u */
    MultAcc_1_Status_Status_Mask = mask;
}


/*******************************************************************************
* Function Name: MultAcc_1_Status_ReadMask
********************************************************************************
*
* Summary:
*  Reads the current interrupt mask assigned to the Status Register.
*
* Parameters:
*  None.
*
* Return:
*  The value of the interrupt mask of the Status Register.
*
*******************************************************************************/
uint8 MultAcc_1_Status_ReadMask(void) 
{
    return MultAcc_1_Status_Status_Mask;
}

#endif /* End check for removal by optimization */


/* [] END OF FILE */
