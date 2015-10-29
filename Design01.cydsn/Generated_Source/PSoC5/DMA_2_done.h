/*******************************************************************************
* File Name: DMA_2_done.h  
* Version 2.10
*
* Description:
*  This file containts Control Register function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_DMA_2_done_H) /* Pins DMA_2_done_H */
#define CY_PINS_DMA_2_done_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "DMA_2_done_aliases.h"

/* Check to see if required defines such as CY_PSOC5A are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5A)
    #error Component cy_pins_v2_10 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5A) */

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 DMA_2_done__PORT == 15 && ((DMA_2_done__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

void    DMA_2_done_Write(uint8 value) ;
void    DMA_2_done_SetDriveMode(uint8 mode) ;
uint8   DMA_2_done_ReadDataReg(void) ;
uint8   DMA_2_done_Read(void) ;
uint8   DMA_2_done_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define DMA_2_done_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define DMA_2_done_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define DMA_2_done_DM_RES_UP          PIN_DM_RES_UP
#define DMA_2_done_DM_RES_DWN         PIN_DM_RES_DWN
#define DMA_2_done_DM_OD_LO           PIN_DM_OD_LO
#define DMA_2_done_DM_OD_HI           PIN_DM_OD_HI
#define DMA_2_done_DM_STRONG          PIN_DM_STRONG
#define DMA_2_done_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define DMA_2_done_MASK               DMA_2_done__MASK
#define DMA_2_done_SHIFT              DMA_2_done__SHIFT
#define DMA_2_done_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define DMA_2_done_PS                     (* (reg8 *) DMA_2_done__PS)
/* Data Register */
#define DMA_2_done_DR                     (* (reg8 *) DMA_2_done__DR)
/* Port Number */
#define DMA_2_done_PRT_NUM                (* (reg8 *) DMA_2_done__PRT) 
/* Connect to Analog Globals */                                                  
#define DMA_2_done_AG                     (* (reg8 *) DMA_2_done__AG)                       
/* Analog MUX bux enable */
#define DMA_2_done_AMUX                   (* (reg8 *) DMA_2_done__AMUX) 
/* Bidirectional Enable */                                                        
#define DMA_2_done_BIE                    (* (reg8 *) DMA_2_done__BIE)
/* Bit-mask for Aliased Register Access */
#define DMA_2_done_BIT_MASK               (* (reg8 *) DMA_2_done__BIT_MASK)
/* Bypass Enable */
#define DMA_2_done_BYP                    (* (reg8 *) DMA_2_done__BYP)
/* Port wide control signals */                                                   
#define DMA_2_done_CTL                    (* (reg8 *) DMA_2_done__CTL)
/* Drive Modes */
#define DMA_2_done_DM0                    (* (reg8 *) DMA_2_done__DM0) 
#define DMA_2_done_DM1                    (* (reg8 *) DMA_2_done__DM1)
#define DMA_2_done_DM2                    (* (reg8 *) DMA_2_done__DM2) 
/* Input Buffer Disable Override */
#define DMA_2_done_INP_DIS                (* (reg8 *) DMA_2_done__INP_DIS)
/* LCD Common or Segment Drive */
#define DMA_2_done_LCD_COM_SEG            (* (reg8 *) DMA_2_done__LCD_COM_SEG)
/* Enable Segment LCD */
#define DMA_2_done_LCD_EN                 (* (reg8 *) DMA_2_done__LCD_EN)
/* Slew Rate Control */
#define DMA_2_done_SLW                    (* (reg8 *) DMA_2_done__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define DMA_2_done_PRTDSI__CAPS_SEL       (* (reg8 *) DMA_2_done__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define DMA_2_done_PRTDSI__DBL_SYNC_IN    (* (reg8 *) DMA_2_done__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define DMA_2_done_PRTDSI__OE_SEL0        (* (reg8 *) DMA_2_done__PRTDSI__OE_SEL0) 
#define DMA_2_done_PRTDSI__OE_SEL1        (* (reg8 *) DMA_2_done__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define DMA_2_done_PRTDSI__OUT_SEL0       (* (reg8 *) DMA_2_done__PRTDSI__OUT_SEL0) 
#define DMA_2_done_PRTDSI__OUT_SEL1       (* (reg8 *) DMA_2_done__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define DMA_2_done_PRTDSI__SYNC_OUT       (* (reg8 *) DMA_2_done__PRTDSI__SYNC_OUT) 


#if defined(DMA_2_done__INTSTAT)  /* Interrupt Registers */

    #define DMA_2_done_INTSTAT                (* (reg8 *) DMA_2_done__INTSTAT)
    #define DMA_2_done_SNAP                   (* (reg8 *) DMA_2_done__SNAP)

#endif /* Interrupt Registers */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_DMA_2_done_H */


/* [] END OF FILE */
