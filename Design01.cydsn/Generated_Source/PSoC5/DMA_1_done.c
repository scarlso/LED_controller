/*******************************************************************************
* File Name: DMA_1_done.c  
* Version 2.10
*
* Description:
*  This file contains API to enable firmware control of a Pins component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "DMA_1_done.h"

/* APIs are not generated for P15[7:6] on PSoC 5 */
#if !(CY_PSOC5A &&\
	 DMA_1_done__PORT == 15 && ((DMA_1_done__MASK & 0xC0) != 0))


/*******************************************************************************
* Function Name: DMA_1_done_Write
********************************************************************************
*
* Summary:
*  Assign a new value to the digital port's data output register.  
*
* Parameters:  
*  prtValue:  The value to be assigned to the Digital Port. 
*
* Return: 
*  None
*  
*******************************************************************************/
void DMA_1_done_Write(uint8 value) 
{
    uint8 staticBits = (DMA_1_done_DR & (uint8)(~DMA_1_done_MASK));
    DMA_1_done_DR = staticBits | ((uint8)(value << DMA_1_done_SHIFT) & DMA_1_done_MASK);
}


/*******************************************************************************
* Function Name: DMA_1_done_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  DMA_1_done_DM_STRONG     Strong Drive 
*  DMA_1_done_DM_OD_HI      Open Drain, Drives High 
*  DMA_1_done_DM_OD_LO      Open Drain, Drives Low 
*  DMA_1_done_DM_RES_UP     Resistive Pull Up 
*  DMA_1_done_DM_RES_DWN    Resistive Pull Down 
*  DMA_1_done_DM_RES_UPDWN  Resistive Pull Up/Down 
*  DMA_1_done_DM_DIG_HIZ    High Impedance Digital 
*  DMA_1_done_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void DMA_1_done_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(DMA_1_done_0, mode);
}


/*******************************************************************************
* Function Name: DMA_1_done_Read
********************************************************************************
*
* Summary:
*  Read the current value on the pins of the Digital Port in right justified 
*  form.
*
* Parameters:  
*  None
*
* Return: 
*  Returns the current value of the Digital Port as a right justified number
*  
* Note:
*  Macro DMA_1_done_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 DMA_1_done_Read(void) 
{
    return (DMA_1_done_PS & DMA_1_done_MASK) >> DMA_1_done_SHIFT;
}


/*******************************************************************************
* Function Name: DMA_1_done_ReadDataReg
********************************************************************************
*
* Summary:
*  Read the current value assigned to a Digital Port's data output register
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value assigned to the Digital Port's data output register
*  
*******************************************************************************/
uint8 DMA_1_done_ReadDataReg(void) 
{
    return (DMA_1_done_DR & DMA_1_done_MASK) >> DMA_1_done_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(DMA_1_done_INTSTAT) 

    /*******************************************************************************
    * Function Name: DMA_1_done_ClearInterrupt
    ********************************************************************************
    * Summary:
    *  Clears any active interrupts attached to port and returns the value of the 
    *  interrupt status register.
    *
    * Parameters:  
    *  None 
    *
    * Return: 
    *  Returns the value of the interrupt status register
    *  
    *******************************************************************************/
    uint8 DMA_1_done_ClearInterrupt(void) 
    {
        return (DMA_1_done_INTSTAT & DMA_1_done_MASK) >> DMA_1_done_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 

#endif /* CY_PSOC5A... */

    
/* [] END OF FILE */
