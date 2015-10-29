/*******************************************************************************
* File Name: co_PM.c  
* Version 3.0
*
*  Description:
*    This file provides the power management source code to API for the
*    Counter.  
*
*   Note:
*     None
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "co.h"

static co_backupStruct co_backup;


/*******************************************************************************
* Function Name: co_SaveConfig
********************************************************************************
* Summary:
*     Save the current user configuration
*
* Parameters:  
*  void
*
* Return: 
*  void
*
* Global variables:
*  co_backup:  Variables of this global structure are modified to 
*  store the values of non retention configuration registers when Sleep() API is 
*  called.
*
*******************************************************************************/
void co_SaveConfig(void) 
{
    #if (!co_UsingFixedFunction)

        co_backup.CounterUdb = co_ReadCounter();

        #if(!co_ControlRegRemoved)
            co_backup.CounterControlRegister = co_ReadControlRegister();
        #endif /* (!co_ControlRegRemoved) */

    #endif /* (!co_UsingFixedFunction) */
}


/*******************************************************************************
* Function Name: co_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the current user configuration.
*
* Parameters:  
*  void
*
* Return: 
*  void
*
* Global variables:
*  co_backup:  Variables of this global structure are used to 
*  restore the values of non retention registers on wakeup from sleep mode.
*
*******************************************************************************/
void co_RestoreConfig(void) 
{      
    #if (!co_UsingFixedFunction)

       co_WriteCounter(co_backup.CounterUdb);

        #if(!co_ControlRegRemoved)
            co_WriteControlRegister(co_backup.CounterControlRegister);
        #endif /* (!co_ControlRegRemoved) */

    #endif /* (!co_UsingFixedFunction) */
}


/*******************************************************************************
* Function Name: co_Sleep
********************************************************************************
* Summary:
*     Stop and Save the user configuration
*
* Parameters:  
*  void
*
* Return: 
*  void
*
* Global variables:
*  co_backup.enableState:  Is modified depending on the enable 
*  state of the block before entering sleep mode.
*
*******************************************************************************/
void co_Sleep(void) 
{
    #if(!co_ControlRegRemoved)
        /* Save Counter's enable state */
        if(co_CTRL_ENABLE == (co_CONTROL & co_CTRL_ENABLE))
        {
            /* Counter is enabled */
            co_backup.CounterEnableState = 1u;
        }
        else
        {
            /* Counter is disabled */
            co_backup.CounterEnableState = 0u;
        }
    #else
        co_backup.CounterEnableState = 1u;
        if(co_backup.CounterEnableState != 0u)
        {
            co_backup.CounterEnableState = 0u;
        }
    #endif /* (!co_ControlRegRemoved) */
    
    co_Stop();
    co_SaveConfig();
}


/*******************************************************************************
* Function Name: co_Wakeup
********************************************************************************
*
* Summary:
*  Restores and enables the user configuration
*  
* Parameters:  
*  void
*
* Return: 
*  void
*
* Global variables:
*  co_backup.enableState:  Is used to restore the enable state of 
*  block on wakeup from sleep mode.
*
*******************************************************************************/
void co_Wakeup(void) 
{
    co_RestoreConfig();
    #if(!co_ControlRegRemoved)
        if(co_backup.CounterEnableState == 1u)
        {
            /* Enable Counter's operation */
            co_Enable();
        } /* Do nothing if Counter was disabled before */    
    #endif /* (!co_ControlRegRemoved) */
    
}


/* [] END OF FILE */
