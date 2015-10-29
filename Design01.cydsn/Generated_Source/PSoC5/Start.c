/*******************************************************************************
* File Name: Start.c  
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
#include "Start.h"

/* APIs are not generated for P15[7:6] on PSoC 5 */
#if !(CY_PSOC5A &&\
	 Start__PORT == 15 && ((Start__MASK & 0xC0) != 0))


/*******************************************************************************
* Function Name: Start_Write
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
void Start_Write(uint8 value) 
{
    uint8 staticBits = (Start_DR & (uint8)(~Start_MASK));
    Start_DR = staticBits | ((uint8)(value << Start_SHIFT) & Start_MASK);
}


/*******************************************************************************
* Function Name: Start_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  Start_DM_STRONG     Strong Drive 
*  Start_DM_OD_HI      Open Drain, Drives High 
*  Start_DM_OD_LO      Open Drain, Drives Low 
*  Start_DM_RES_UP     Resistive Pull Up 
*  Start_DM_RES_DWN    Resistive Pull Down 
*  Start_DM_RES_UPDWN  Resistive Pull Up/Down 
*  Start_DM_DIG_HIZ    High Impedance Digital 
*  Start_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void Start_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(Start_0, mode);
}


/*******************************************************************************
* Function Name: Start_Read
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
*  Macro Start_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Start_Read(void) 
{
    return (Start_PS & Start_MASK) >> Start_SHIFT;
}


/*******************************************************************************
* Function Name: Start_ReadDataReg
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
uint8 Start_ReadDataReg(void) 
{
    return (Start_DR & Start_MASK) >> Start_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Start_INTSTAT) 

    /*******************************************************************************
    * Function Name: Start_ClearInterrupt
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
    uint8 Start_ClearInterrupt(void) 
    {
        return (Start_INTSTAT & Start_MASK) >> Start_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 

#endif /* CY_PSOC5A... */

    
/* [] END OF FILE */
