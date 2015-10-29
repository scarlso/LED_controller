/*******************************************************************************
* File Name: MultAcc_1_MACC_PM.c
* Version 1.30
*
* Description:
*  This file contains the setup, control and status commands to support
*  component operations in low power mode.
*
* Note:
*
********************************************************************************
* Copyright 2011-2013, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "MultAcc_1_MACC.h"

static MultAcc_1_MACC_BACKUP_STRUCT MultAcc_1_MACC_backup = { 0u };


/*******************************************************************************
* Function Name: MultAcc_1_MACC_SaveConfig
********************************************************************************
*
* Summary:
*  Saves the user configuration of the DFB non-retention registers. This routine
*  is called by DFB_Sleep() to save the component configuration before entering
*  sleep.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void MultAcc_1_MACC_SaveConfig(void) 
{
    uint8 count;
    uint8 i = 0u;
	
    MultAcc_1_MACC_backup.cr = MultAcc_1_MACC_CR_REG;
    MultAcc_1_MACC_backup.sr = MultAcc_1_MACC_SR_REG;
    MultAcc_1_MACC_backup.sem = MultAcc_1_MACC_SEM_REG;
	
    /* Put DFB RAM on the bus */
    MultAcc_1_MACC_RAM_DIR_REG = MultAcc_1_MACC_RAM_DIR_BUS;

    /* ACU RAM */
    count = MultAcc_1_MACC_ACU_RAM_SIZE_CUR;
    while (count > 0u)
    {
        MultAcc_1_MACC_backup.acu[i] = CY_GET_REG32(& MultAcc_1_MACC_ACU_RAM_PTR[i]);
        count--;
        i++;
    }
	
    /* Take DFB RAM off the bus */
    MultAcc_1_MACC_RAM_DIR_REG = MultAcc_1_MACC_RAM_DIR_DFB;
}


/*******************************************************************************
* Function Name: MultAcc_1_MACC_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the user configuration of the DFB non-retention registers. This
*  routine is called by DFB_Wakeup() to restore the component configuration when
*  exiting sleep.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void MultAcc_1_MACC_RestoreConfig(void) 
{
    uint8 count;
    uint8 i = 0u;
		
    MultAcc_1_MACC_CR_REG = MultAcc_1_MACC_backup.cr;
    MultAcc_1_MACC_SR_REG = MultAcc_1_MACC_backup.sr;
    MultAcc_1_MACC_SEM_REG = MultAcc_1_MACC_backup.sem;
    
    /* Power on DFB before initializing the RAMs */
    MultAcc_1_MACC_PM_ACT_CFG_REG |= MultAcc_1_MACC_PM_ACT_MSK;
    /* Enables DFB block in Alternate Active (Standby) mode */
    MultAcc_1_MACC_PM_STBY_CFG_REG |= MultAcc_1_MACC_PM_STBY_MSK;
    
    /* Put DFB RAM on the bus */
    MultAcc_1_MACC_RAM_DIR_REG = MultAcc_1_MACC_RAM_DIR_BUS;

    /* ACU RAM */
    count = MultAcc_1_MACC_ACU_RAM_SIZE_CUR;
    while (count > 0u)
    {
        CY_SET_REG32(& MultAcc_1_MACC_ACU_RAM_PTR[i], MultAcc_1_MACC_backup.acu[i]);
        count--;
        i++;
    }

    /* Take DFB RAM off the bus */
    MultAcc_1_MACC_RAM_DIR_REG = MultAcc_1_MACC_RAM_DIR_DFB;
}


/*******************************************************************************
* Function Name: MultAcc_1_MACC_Sleep
********************************************************************************
*
* Summary:
*  This is the preferred routine to prepare the component for sleep. The
*  MultAcc_1_MACC_Sleep() routine saves the current component state. Then it
*  calls the DFB_Stop() function and calls MultAcc_1_MACC_SaveConfig() to save
*  the hardware configuration. Call the MultAcc_1_MACC_Sleep() function before
*  calling the CyPmSleep() or the CyPmHibernate() function. Refer to the PSoC
*  Creator System Reference Guide for more information about power management
*  functions.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void MultAcc_1_MACC_Sleep(void) 
{
    if (0u != (MultAcc_1_MACC_CR_REG & MultAcc_1_MACC_RUN_MASK))
    {
        MultAcc_1_MACC_backup.enableState = 1u;
    }
    else /* The DFB block is disabled */
    {
        MultAcc_1_MACC_backup.enableState = 0u;
    }

    /* Save the configuration */
    MultAcc_1_MACC_SaveConfig();
    
    /* Disables the clock to the entire core of the block */
    MultAcc_1_MACC_CR_REG |= MultAcc_1_MACC_CORECLK_DISABLE;
    
    /* Turn off the run bit */
    MultAcc_1_MACC_Pause();
}


/*******************************************************************************
* Function Name: MultAcc_1_MACC_Wakeup
********************************************************************************
*
* Summary:
*  This is the preferred routine to restore the component to the state when
*  MultAcc_1_MACC_Sleep() was called. The MultAcc_1_MACC_Wakeup() function
*  calls the MultAcc_1_MACC_RestoreConfig() function to restore the
*  configuration. If the component was enabled before the
*  MultAcc_1_MACC_Sleep() function was called, the MultAcc_1_MACC_Wakeup()
*  function will also re-enable the component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Side Effects:
*  Calling the MultAcc_1_MACC_Wakeup() function without first calling the
*  MultAcc_1_MACC_Sleep() or MultAcc_1_MACC_SaveConfig() function may
*  produce unexpected behavior.
*
*******************************************************************************/
void MultAcc_1_MACC_Wakeup(void) 
{
    MultAcc_1_MACC_RestoreConfig();

    /* Enables the clock to the entire core of the block */
    MultAcc_1_MACC_CR_REG &= (uint8) (~MultAcc_1_MACC_CORECLK_DISABLE);

    if (MultAcc_1_MACC_backup.enableState != 0u)
    {
        /* Enables component's operation */
        MultAcc_1_MACC_Resume();
    } /* Left Run Bit in off state if component's block was disabled before */
    else
    {
        /* Takes DFB RAM off the bus */
        MultAcc_1_MACC_RAM_DIR_REG = MultAcc_1_MACC_RAM_DIR_DFB;
        /* Clears any pending interrupts */
        MultAcc_1_MACC_SR_REG = 0xFFu;
    }
}


/* [] END OF FILE */
