/*******************************************************************************
* File Name: MultAcc_1_Control.h  
* Version 1.70
*
* Description:
*  This file containts Control Register function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_CONTROL_REG_MultAcc_1_Control_H) /* CY_CONTROL_REG_MultAcc_1_Control_H */
#define CY_CONTROL_REG_MultAcc_1_Control_H

#include "cytypes.h"


/***************************************
*         Function Prototypes 
***************************************/

void    MultAcc_1_Control_Write(uint8 control) ;
uint8   MultAcc_1_Control_Read(void) ;


/***************************************
*            Registers        
***************************************/

/* Control Register */
#define MultAcc_1_Control_Control        (* (reg8 *) MultAcc_1_Control_Sync_ctrl_reg__CONTROL_REG )
#define MultAcc_1_Control_Control_PTR    (  (reg8 *) MultAcc_1_Control_Sync_ctrl_reg__CONTROL_REG )

#endif /* End CY_CONTROL_REG_MultAcc_1_Control_H */


/* [] END OF FILE */
