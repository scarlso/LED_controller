/*******************************************************************************
* File Name: co.c  
* Version 3.0
*
*  Description:
*     The Counter component consists of a 8, 16, 24 or 32-bit counter with
*     a selectable period between 2 and 2^Width - 1.  
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

uint8 co_initVar = 0u;


/*******************************************************************************
* Function Name: co_Init
********************************************************************************
* Summary:
*     Initialize to the schematic state
* 
* Parameters:  
*  void  
*
* Return: 
*  void
*
*******************************************************************************/
void co_Init(void) 
{
        #if (!co_UsingFixedFunction && !co_ControlRegRemoved)
            uint8 ctrl;
        #endif /* (!co_UsingFixedFunction && !co_ControlRegRemoved) */
        
        #if(!co_UsingFixedFunction) 
            /* Interrupt State Backup for Critical Region*/
            uint8 co_interruptState;
        #endif /* (!co_UsingFixedFunction) */
        
        #if (co_UsingFixedFunction)
            /* Clear all bits but the enable bit (if it's already set for Timer operation */
            co_CONTROL &= co_CTRL_ENABLE;
            
            /* Clear the mode bits for continuous run mode */
            #if (CY_PSOC5A)
                co_CONTROL2 &= ((uint8)(~co_CTRL_MODE_MASK));
            #endif /* (CY_PSOC5A) */
            #if (CY_PSOC3 || CY_PSOC5LP)
                co_CONTROL3 &= ((uint8)(~co_CTRL_MODE_MASK));                
            #endif /* (CY_PSOC3 || CY_PSOC5LP) */
            /* Check if One Shot mode is enabled i.e. RunMode !=0*/
            #if (co_RunModeUsed != 0x0u)
                /* Set 3rd bit of Control register to enable one shot mode */
                co_CONTROL |= co_ONESHOT;
            #endif /* (co_RunModeUsed != 0x0u) */
            
            /* Set the IRQ to use the status register interrupts */
            co_CONTROL2 |= co_CTRL2_IRQ_SEL;
            
            /* Clear and Set SYNCTC and SYNCCMP bits of RT1 register */
            co_RT1 &= ((uint8)(~co_RT1_MASK));
            co_RT1 |= co_SYNC;     
                    
            /*Enable DSI Sync all all inputs of the Timer*/
            co_RT1 &= ((uint8)(~co_SYNCDSI_MASK));
            co_RT1 |= co_SYNCDSI_EN;

        #else
            #if(!co_ControlRegRemoved)
            /* Set the default compare mode defined in the parameter */
            ctrl = co_CONTROL & ((uint8)(~co_CTRL_CMPMODE_MASK));
            co_CONTROL = ctrl | co_DEFAULT_COMPARE_MODE;
            
            /* Set the default capture mode defined in the parameter */
            ctrl = co_CONTROL & ((uint8)(~co_CTRL_CAPMODE_MASK));
            
            #if( 0 != co_CAPTURE_MODE_CONF)
                co_CONTROL = ctrl | co_DEFAULT_CAPTURE_MODE;
            #else
                co_CONTROL = ctrl;
            #endif /* 0 != co_CAPTURE_MODE */ 
            
            #endif /* (!co_ControlRegRemoved) */
        #endif /* (co_UsingFixedFunction) */
        
        /* Clear all data in the FIFO's */
        #if (!co_UsingFixedFunction)
            co_ClearFIFO();
        #endif /* (!co_UsingFixedFunction) */
        
        /* Set Initial values from Configuration */
        co_WritePeriod(co_INIT_PERIOD_VALUE);
        #if (!(co_UsingFixedFunction && (CY_PSOC5A)))
            co_WriteCounter(co_INIT_COUNTER_VALUE);
        #endif /* (!(co_UsingFixedFunction && (CY_PSOC5A))) */
        co_SetInterruptMode(co_INIT_INTERRUPTS_MASK);
        
        #if (!co_UsingFixedFunction)
            /* Read the status register to clear the unwanted interrupts */
            (void)co_ReadStatusRegister();
            /* Set the compare value (only available to non-fixed function implementation */
            co_WriteCompare(co_INIT_COMPARE_VALUE);
            /* Use the interrupt output of the status register for IRQ output */
            
            /* CyEnterCriticalRegion and CyExitCriticalRegion are used to mark following region critical*/
            /* Enter Critical Region*/
            co_interruptState = CyEnterCriticalSection();
            
            co_STATUS_AUX_CTRL |= co_STATUS_ACTL_INT_EN_MASK;
            
            /* Exit Critical Region*/
            CyExitCriticalSection(co_interruptState);
            
        #endif /* (!co_UsingFixedFunction) */
}


/*******************************************************************************
* Function Name: co_Enable
********************************************************************************
* Summary:
*     Enable the Counter
* 
* Parameters:  
*  void  
*
* Return: 
*  void
*
* Side Effects: 
*   If the Enable mode is set to Hardware only then this function has no effect 
*   on the operation of the counter.
*
*******************************************************************************/
void co_Enable(void) 
{
    /* Globally Enable the Fixed Function Block chosen */
    #if (co_UsingFixedFunction)
        co_GLOBAL_ENABLE |= co_BLOCK_EN_MASK;
        co_GLOBAL_STBY_ENABLE |= co_BLOCK_STBY_EN_MASK;
    #endif /* (co_UsingFixedFunction) */  
        
    /* Enable the counter from the control register  */
    /* If Fixed Function then make sure Mode is set correctly */
    /* else make sure reset is clear */
    #if(!co_ControlRegRemoved || co_UsingFixedFunction)
        co_CONTROL |= co_CTRL_ENABLE;                
    #endif /* (!co_ControlRegRemoved || co_UsingFixedFunction) */
    
}


/*******************************************************************************
* Function Name: co_Start
********************************************************************************
* Summary:
*  Enables the counter for operation 
*
* Parameters:  
*  void  
*
* Return: 
*  void
*
* Global variables:
*  co_initVar: Is modified when this function is called for the  
*   first time. Is used to ensure that initialization happens only once.
*
*******************************************************************************/
void co_Start(void) 
{
    if(co_initVar == 0u)
    {
        co_Init();
        
        co_initVar = 1u; /* Clear this bit for Initialization */        
    }
    
    /* Enable the Counter */
    co_Enable();        
}


/*******************************************************************************
* Function Name: co_Stop
********************************************************************************
* Summary:
* Halts the counter, but does not change any modes or disable interrupts.
*
* Parameters:  
*  void  
*
* Return: 
*  void
*
* Side Effects: If the Enable mode is set to Hardware only then this function
*               has no effect on the operation of the counter.
*
*******************************************************************************/
void co_Stop(void) 
{
    /* Disable Counter */
    #if(!co_ControlRegRemoved || co_UsingFixedFunction)
        co_CONTROL &= ((uint8)(~co_CTRL_ENABLE));        
    #endif /* (!co_ControlRegRemoved || co_UsingFixedFunction) */
    
    /* Globally disable the Fixed Function Block chosen */
    #if (co_UsingFixedFunction)
        co_GLOBAL_ENABLE &= ((uint8)(~co_BLOCK_EN_MASK));
        co_GLOBAL_STBY_ENABLE &= ((uint8)(~co_BLOCK_STBY_EN_MASK));
    #endif /* (co_UsingFixedFunction) */
}


/*******************************************************************************
* Function Name: co_SetInterruptMode
********************************************************************************
* Summary:
* Configures which interrupt sources are enabled to generate the final interrupt
*
* Parameters:  
*  InterruptsMask: This parameter is an or'd collection of the status bits
*                   which will be allowed to generate the counters interrupt.   
*
* Return: 
*  void
*
*******************************************************************************/
void co_SetInterruptMode(uint8 interruptsMask) 
{
    co_STATUS_MASK = interruptsMask;
}


/*******************************************************************************
* Function Name: co_ReadStatusRegister
********************************************************************************
* Summary:
*   Reads the status register and returns it's state. This function should use
*       defined types for the bit-field information as the bits in this
*       register may be permuteable.
*
* Parameters:  
*  void
*
* Return: 
*  (uint8) The contents of the status register
*
* Side Effects:
*   Status register bits may be clear on read. 
*
*******************************************************************************/
uint8   co_ReadStatusRegister(void) 
{
    return co_STATUS;
}


#if(!co_ControlRegRemoved)
/*******************************************************************************
* Function Name: co_ReadControlRegister
********************************************************************************
* Summary:
*   Reads the control register and returns it's state. This function should use
*       defined types for the bit-field information as the bits in this
*       register may be permuteable.
*
* Parameters:  
*  void
*
* Return: 
*  (uint8) The contents of the control register
*
*******************************************************************************/
uint8   co_ReadControlRegister(void) 
{
    return co_CONTROL;
}


/*******************************************************************************
* Function Name: co_WriteControlRegister
********************************************************************************
* Summary:
*   Sets the bit-field of the control register.  This function should use
*       defined types for the bit-field information as the bits in this
*       register may be permuteable.
*
* Parameters:  
*  void
*
* Return: 
*  (uint8) The contents of the control register
*
*******************************************************************************/
void    co_WriteControlRegister(uint8 control) 
{
    co_CONTROL = control;
}

#endif  /* (!co_ControlRegRemoved) */


#if (!(co_UsingFixedFunction && (CY_PSOC5A)))
/*******************************************************************************
* Function Name: co_WriteCounter
********************************************************************************
* Summary:
*   This funtion is used to set the counter to a specific value
*
* Parameters:  
*  counter:  New counter value. 
*
* Return: 
*  void 
*
*******************************************************************************/
void co_WriteCounter(uint16 counter) \
                                   
{
    #if(co_UsingFixedFunction)
        /* assert if block is already enabled */
        CYASSERT (0u == (co_GLOBAL_ENABLE & co_BLOCK_EN_MASK));
        /* If block is disabled, enable it and then write the counter */
        co_GLOBAL_ENABLE |= co_BLOCK_EN_MASK;
        CY_SET_REG16(co_COUNTER_LSB_PTR, (uint16)counter);
        co_GLOBAL_ENABLE &= ((uint8)(~co_BLOCK_EN_MASK));
    #else
        CY_SET_REG16(co_COUNTER_LSB_PTR, counter);
    #endif /* (co_UsingFixedFunction) */
}
#endif /* (!(co_UsingFixedFunction && (CY_PSOC5A))) */


/*******************************************************************************
* Function Name: co_ReadCounter
********************************************************************************
* Summary:
* Returns the current value of the counter.  It doesn't matter
* if the counter is enabled or running.
*
* Parameters:  
*  void:  
*
* Return: 
*  (uint16) The present value of the counter.
*
*******************************************************************************/
uint16 co_ReadCounter(void) 
{
    /* Force capture by reading Accumulator */
    /* Must first do a software capture to be able to read the counter */
    /* It is up to the user code to make sure there isn't already captured data in the FIFO */
    #if(co_UsingFixedFunction)
		(void)CY_GET_REG16(co_COUNTER_LSB_PTR);
	#else
		(void)CY_GET_REG8(co_COUNTER_LSB_PTR_8BIT);
	#endif/* (co_UsingFixedFunction) */
    
    /* Read the data from the FIFO (or capture register for Fixed Function)*/
    #if(co_UsingFixedFunction)
        return ((uint16)CY_GET_REG16(co_STATICCOUNT_LSB_PTR));
    #else
        return (CY_GET_REG16(co_STATICCOUNT_LSB_PTR));
    #endif /* (co_UsingFixedFunction) */
}


/*******************************************************************************
* Function Name: co_ReadCapture
********************************************************************************
* Summary:
*   This function returns the last value captured.
*
* Parameters:  
*  void
*
* Return: 
*  (uint16) Present Capture value.
*
*******************************************************************************/
uint16 co_ReadCapture(void) 
{
    #if(co_UsingFixedFunction)
        return ((uint16)CY_GET_REG16(co_STATICCOUNT_LSB_PTR));
    #else
        return (CY_GET_REG16(co_STATICCOUNT_LSB_PTR));
    #endif /* (co_UsingFixedFunction) */
}


/*******************************************************************************
* Function Name: co_WritePeriod
********************************************************************************
* Summary:
* Changes the period of the counter.  The new period 
* will be loaded the next time terminal count is detected.
*
* Parameters:  
*  period: (uint16) A value of 0 will result in
*         the counter remaining at zero.  
*
* Return: 
*  void
*
*******************************************************************************/
void co_WritePeriod(uint16 period) 
{
    #if(co_UsingFixedFunction)
        CY_SET_REG16(co_PERIOD_LSB_PTR,(uint16)period);
    #else
        CY_SET_REG16(co_PERIOD_LSB_PTR, period);
    #endif /* (co_UsingFixedFunction) */
}


/*******************************************************************************
* Function Name: co_ReadPeriod
********************************************************************************
* Summary:
* Reads the current period value without affecting counter operation.
*
* Parameters:  
*  void:  
*
* Return: 
*  (uint16) Present period value.
*
*******************************************************************************/
uint16 co_ReadPeriod(void) 
{
    #if(co_UsingFixedFunction)
        return ((uint16)CY_GET_REG16(co_PERIOD_LSB_PTR));
    #else
        return (CY_GET_REG16(co_PERIOD_LSB_PTR));
    #endif /* (co_UsingFixedFunction) */
}


#if (!co_UsingFixedFunction)
/*******************************************************************************
* Function Name: co_WriteCompare
********************************************************************************
* Summary:
* Changes the compare value.  The compare output will 
* reflect the new value on the next UDB clock.  The compare output will be 
* driven high when the present counter value compares true based on the 
* configured compare mode setting. 
*
* Parameters:  
*  Compare:  New compare value. 
*
* Return: 
*  void
*
*******************************************************************************/
void co_WriteCompare(uint16 compare) \
                                   
{
    #if(co_UsingFixedFunction)
        CY_SET_REG16(co_COMPARE_LSB_PTR, (uint16)compare);
    #else
        CY_SET_REG16(co_COMPARE_LSB_PTR, compare);
    #endif /* (co_UsingFixedFunction) */
}


/*******************************************************************************
* Function Name: co_ReadCompare
********************************************************************************
* Summary:
* Returns the compare value.
*
* Parameters:  
*  void:
*
* Return: 
*  (uint16) Present compare value.
*
*******************************************************************************/
uint16 co_ReadCompare(void) 
{
    return (CY_GET_REG16(co_COMPARE_LSB_PTR));
}


#if (co_COMPARE_MODE_SOFTWARE)
/*******************************************************************************
* Function Name: co_SetCompareMode
********************************************************************************
* Summary:
*  Sets the software controlled Compare Mode.
*
* Parameters:
*  compareMode:  Compare Mode Enumerated Type.
*
* Return:
*  void
*
*******************************************************************************/
void co_SetCompareMode(uint8 compareMode) 
{
    /* Clear the compare mode bits in the control register */
    co_CONTROL &= ((uint8)(~co_CTRL_CMPMODE_MASK));
    
    /* Write the new setting */
    co_CONTROL |= compareMode;
}
#endif  /* (co_COMPARE_MODE_SOFTWARE) */


#if (co_CAPTURE_MODE_SOFTWARE)
/*******************************************************************************
* Function Name: co_SetCaptureMode
********************************************************************************
* Summary:
*  Sets the software controlled Capture Mode.
*
* Parameters:
*  captureMode:  Capture Mode Enumerated Type.
*
* Return:
*  void
*
*******************************************************************************/
void co_SetCaptureMode(uint8 captureMode) 
{
    /* Clear the capture mode bits in the control register */
    co_CONTROL &= ((uint8)(~co_CTRL_CAPMODE_MASK));
    
    /* Write the new setting */
    co_CONTROL |= ((uint8)((uint8)captureMode << co_CTRL_CAPMODE0_SHIFT));
}
#endif  /* (co_CAPTURE_MODE_SOFTWARE) */


/*******************************************************************************
* Function Name: co_ClearFIFO
********************************************************************************
* Summary:
*   This function clears all capture data from the capture FIFO
*
* Parameters:  
*  void:
*
* Return: 
*  None
*
*******************************************************************************/
void co_ClearFIFO(void) 
{

    while(0u != (co_ReadStatusRegister() & co_STATUS_FIFONEMP))
    {
        (void)co_ReadCapture();
    }

}
#endif  /* (!co_UsingFixedFunction) */


/* [] END OF FILE */

