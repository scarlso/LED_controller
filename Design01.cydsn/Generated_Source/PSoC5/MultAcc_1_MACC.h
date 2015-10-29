/*******************************************************************************
* File Name: MultAcc_1_MACC.h
* Version 1.30
*
* Description:
*  Contains the function prototypes, constants and register definition
*  of the DFB Component.
*
* Note:
*  None
*
********************************************************************************
* Copyright 2011-2013, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_DFB_MultAcc_1_MACC_H) /* DFB Header File */
#define CY_DFB_MultAcc_1_MACC_H

#include "CyLib.h"

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component DFB_v1_30 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

extern uint8 MultAcc_1_MACC_initVar;


/***************************************
*   Conditional Compilation Parameters
***************************************/

#define MultAcc_1_MACC_OPTIMIZE_ASSEMBLY      (0u)
#define MultAcc_1_MACC_INIT_INTERRUPT_MODE    (4u)
#define MultAcc_1_MACC_INIT_DMA_MODE          (5u)
#define MultAcc_1_MACC_INIT_OUT1_SOURCE       (2u)
#define MultAcc_1_MACC_INIT_OUT2_SOURCE       (0u)

/* Sizes of current DFB program */
#define MultAcc_1_MACC_DA_RAM_SIZE_CUR        (0x01u)
#define MultAcc_1_MACC_DB_RAM_SIZE_CUR        (0x01u)
#define MultAcc_1_MACC_CSA_RAM_SIZE_CUR       (0x16u)
#define MultAcc_1_MACC_CSB_RAM_SIZE_CUR       (0x16u)
#define MultAcc_1_MACC_CFSM_RAM_SIZE_CUR      (0x09u)
#define MultAcc_1_MACC_ACU_RAM_SIZE_CUR       (0x01u)


/***************************************
*     Data Types Definitions
***************************************/

#if (MultAcc_1_MACC_OPTIMIZE_ASSEMBLY)
    extern const uint32 CYCODE MultAcc_1_MACC_cstoreA[MultAcc_1_MACC_CSA_RAM_SIZE_CUR];
    extern const uint32 CYCODE MultAcc_1_MACC_cstoreB[MultAcc_1_MACC_CSB_RAM_SIZE_CUR];
#else
    extern const uint32 CYCODE MultAcc_1_MACC_control[MultAcc_1_MACC_CSA_RAM_SIZE_CUR];
#endif /* MultAcc_1_MACC_OPTIMIZE_ASSEMBLY */

extern const uint32 CYCODE MultAcc_1_MACC_data_a[MultAcc_1_MACC_DA_RAM_SIZE_CUR];
extern const uint32 CYCODE MultAcc_1_MACC_data_b[MultAcc_1_MACC_DB_RAM_SIZE_CUR];
extern const uint32 CYCODE MultAcc_1_MACC_cfsm[MultAcc_1_MACC_CFSM_RAM_SIZE_CUR];
extern const uint32 CYCODE MultAcc_1_MACC_acu[MultAcc_1_MACC_ACU_RAM_SIZE_CUR];

/* Sleep Mode API Support */
typedef struct
{
    uint8  enableState;
    uint8  cr;
    uint8  sr;
    uint8  sem;
    uint32 acu[MultAcc_1_MACC_ACU_RAM_SIZE_CUR];
} MultAcc_1_MACC_BACKUP_STRUCT;


/***************************************
*        Function Prototypes
***************************************/

void   MultAcc_1_MACC_Init(void) ;
void   MultAcc_1_MACC_Enable(void) ;
void   MultAcc_1_MACC_Start(void) ;
void   MultAcc_1_MACC_Stop(void) ;
void   MultAcc_1_MACC_Pause(void) ;
void   MultAcc_1_MACC_Resume(void) ;
void   MultAcc_1_MACC_SetCoherency(uint8 coherencyKeyByte) ;
void   MultAcc_1_MACC_SetDalign(uint8 dalignKeyByte) ;
uint8  MultAcc_1_MACC_LoadDataRAMA(const int32 ptr[], uint32 addr[], uint8 size)
        ;
uint8  MultAcc_1_MACC_LoadDataRAMB(const int32 ptr[], uint32 addr[], uint8 size)
        ;
void   MultAcc_1_MACC_LoadInputValue(uint8 channel, int32 sample)
        ;
int32  MultAcc_1_MACC_GetOutputValue(uint8 channel) ;
void   MultAcc_1_MACC_SetInterruptMode(uint8 events) ;
uint8  MultAcc_1_MACC_GetInterruptSource(void) ;
void   MultAcc_1_MACC_ClearInterrupt(uint8 interruptMask) ;
void   MultAcc_1_MACC_SetDMAMode(uint8 events) ;
void   MultAcc_1_MACC_SetSemaphores(uint8 mask) ;
uint8  MultAcc_1_MACC_GetSemaphores(void) ;
void   MultAcc_1_MACC_ClearSemaphores(uint8 mask) ;
void   MultAcc_1_MACC_SetOutput1Source(uint8 source) ;
void   MultAcc_1_MACC_SetOutput2Source(uint8 source) ;
void   MultAcc_1_MACC_Sleep(void) ;
void   MultAcc_1_MACC_Wakeup(void) ;
void   MultAcc_1_MACC_SaveConfig(void) ;
void   MultAcc_1_MACC_RestoreConfig(void) ;

/* Macros */
#define MultAcc_1_MACC_ClearInterruptSource(event) \
do { \
    MultAcc_1_MACC_SR_REG = (uint8) (((event) & MultAcc_1_MACC_EVENT_MASK) << MultAcc_1_MACC_SR_EVENT_SHIFT); \
} while (0)


/***************************************
*           API Constants
***************************************/

/* Channel Definitions */
#define MultAcc_1_MACC_CHANNEL_A              (1u)
#define MultAcc_1_MACC_CHANNEL_B              (0u)

/* Sizes of DFB memories */
#define MultAcc_1_MACC_DA_RAM_SIZE            (0x80u)
#define MultAcc_1_MACC_DB_RAM_SIZE            (0x80u)
#define MultAcc_1_MACC_CSA_RAM_SIZE           (0x40u)
#define MultAcc_1_MACC_CSB_RAM_SIZE           (0x40u)
#define MultAcc_1_MACC_CFSM_RAM_SIZE          (0x40u)
#define MultAcc_1_MACC_ACU_RAM_SIZE           (0x10u)

#define MultAcc_1_MACC_SUCCESS                (CYRET_SUCCESS)
#define MultAcc_1_MACC_DATA_OUT_OF_RANGE      (0xFEu)
#define MultAcc_1_MACC_ADDRESS_OUT_OF_RANGE   (0xFFu)


/***************************************
*             Registers
***************************************/

/* RAM memory map offsets */
#define MultAcc_1_MACC_DA_RAM_REG             (* (reg32 *) (MultAcc_1_MACC_DFB__DPA_SRAM_DATA))
#define MultAcc_1_MACC_DA_RAM_PTR             (  (reg32 *) (MultAcc_1_MACC_DFB__DPA_SRAM_DATA))
#define MultAcc_1_MACC_DB_RAM_REG             (* (reg32 *) (MultAcc_1_MACC_DFB__DPB_SRAM_DATA))
#define MultAcc_1_MACC_DB_RAM_PTR             (  (reg32 *) (MultAcc_1_MACC_DFB__DPB_SRAM_DATA))
#define MultAcc_1_MACC_CSA_RAM_REG            (* (reg32 *) (MultAcc_1_MACC_DFB__CSA_SRAM_DATA))
#define MultAcc_1_MACC_CSA_RAM_PTR            (  (reg32 *) (MultAcc_1_MACC_DFB__CSA_SRAM_DATA))
#define MultAcc_1_MACC_CSB_RAM_REG            (* (reg32 *) (MultAcc_1_MACC_DFB__CSB_SRAM_DATA))
#define MultAcc_1_MACC_CSB_RAM_PTR            (  (reg32 *) (MultAcc_1_MACC_DFB__CSB_SRAM_DATA))
#define MultAcc_1_MACC_CFSM_RAM_REG           (* (reg32 *) (MultAcc_1_MACC_DFB__FSM_SRAM_DATA))
#define MultAcc_1_MACC_CFSM_RAM_PTR           (  (reg32 *) (MultAcc_1_MACC_DFB__FSM_SRAM_DATA))
#define MultAcc_1_MACC_ACU_RAM_REG            (* (reg32 *) (MultAcc_1_MACC_DFB__ACU_SRAM_DATA))
#define MultAcc_1_MACC_ACU_RAM_PTR            (  (reg32 *) (MultAcc_1_MACC_DFB__ACU_SRAM_DATA))

#define MultAcc_1_MACC_SEM_REG                (* (reg8 *) MultAcc_1_MACC_DFB__SEMA)
#define MultAcc_1_MACC_SEM_PTR                (  (reg8 *) MultAcc_1_MACC_DFB__SEMA)
#define MultAcc_1_MACC_CR_REG                 (* (reg8 *) MultAcc_1_MACC_DFB__CR)
#define MultAcc_1_MACC_CR_PTR                 (  (reg8 *) MultAcc_1_MACC_DFB__CR)
#define MultAcc_1_MACC_SR_REG                 (* (reg8 *) MultAcc_1_MACC_DFB__SR)
#define MultAcc_1_MACC_SR_PTR                 (  (reg8 *) MultAcc_1_MACC_DFB__SR)
#define MultAcc_1_MACC_INT_CTRL_REG           (* (reg8 *) MultAcc_1_MACC_DFB__INT_CTRL)
#define MultAcc_1_MACC_INT_CTRL_PTR           (  (reg8 *) MultAcc_1_MACC_DFB__INT_CTRL)
#define MultAcc_1_MACC_DMA_CTRL_REG           (* (reg8 *) MultAcc_1_MACC_DFB__DMA_CTRL)
#define MultAcc_1_MACC_DMA_CTRL_PTR           (  (reg8 *) MultAcc_1_MACC_DFB__DMA_CTRL)
#define MultAcc_1_MACC_RAM_DIR_REG            (* (reg8 *) MultAcc_1_MACC_DFB__RAM_DIR)
#define MultAcc_1_MACC_RAM_DIR_PTR            (  (reg8 *) MultAcc_1_MACC_DFB__RAM_DIR)

#define MultAcc_1_MACC_DALIGN_REG             (* (reg8 *) MultAcc_1_MACC_DFB__DALIGN)
#define MultAcc_1_MACC_DALIGN_PTR             (  (reg8 *) MultAcc_1_MACC_DFB__DALIGN)
#define MultAcc_1_MACC_DSI_CTRL_REG           (* (reg8 *) MultAcc_1_MACC_DFB__DSI_CTRL)
#define MultAcc_1_MACC_DSI_CTRL_PTR           (  (reg8 *) MultAcc_1_MACC_DFB__DSI_CTRL)
#define MultAcc_1_MACC_HOLDA_REG              (* (reg8 *) MultAcc_1_MACC_DFB__HOLDA)
#define MultAcc_1_MACC_HOLDA_PTR              (  (reg8 *) MultAcc_1_MACC_DFB__HOLDA)
#define MultAcc_1_MACC_HOLDAH_REG             (* (reg8 *) MultAcc_1_MACC_DFB__HOLDAH)
#define MultAcc_1_MACC_HOLDAH_PTR             (  (reg8 *) MultAcc_1_MACC_DFB__HOLDAH)
#define MultAcc_1_MACC_HOLDAM_REG             (* (reg8 *) MultAcc_1_MACC_DFB__HOLDAM)
#define MultAcc_1_MACC_HOLDAM_PTR             (  (reg8 *) MultAcc_1_MACC_DFB__HOLDAM)
#define MultAcc_1_MACC_HOLDB_REG              (* (reg8 *) MultAcc_1_MACC_DFB__HOLDB)
#define MultAcc_1_MACC_HOLDB_PTR              (  (reg8 *) MultAcc_1_MACC_DFB__HOLDB)
#define MultAcc_1_MACC_HOLDBH_REG             (* (reg8 *) MultAcc_1_MACC_DFB__HOLDBH)
#define MultAcc_1_MACC_HOLDBH_PTR             (  (reg8 *) MultAcc_1_MACC_DFB__HOLDBH)
#define MultAcc_1_MACC_HOLDBM_REG             (* (reg8 *) MultAcc_1_MACC_DFB__HOLDBM)
#define MultAcc_1_MACC_HOLDBM_PTR             (  (reg8 *) MultAcc_1_MACC_DFB__HOLDBM)
#define MultAcc_1_MACC_PM_ACT_CFG_REG         (* (reg8 *) MultAcc_1_MACC_DFB__PM_ACT_CFG)
#define MultAcc_1_MACC_PM_ACT_CFG_PTR         (  (reg8 *) MultAcc_1_MACC_DFB__PM_ACT_CFG)
#define MultAcc_1_MACC_PM_STBY_CFG_REG        (* (reg8 *) MultAcc_1_MACC_DFB__PM_STBY_CFG)
#define MultAcc_1_MACC_PM_STBY_CFG_PTR        (  (reg8 *) MultAcc_1_MACC_DFB__PM_STBY_CFG)
#define MultAcc_1_MACC_RAM_EN_REG             (* (reg8 *) MultAcc_1_MACC_DFB__RAM_EN)
#define MultAcc_1_MACC_RAM_EN_PTR             (  (reg8 *) MultAcc_1_MACC_DFB__RAM_EN)
#define MultAcc_1_MACC_STAGEA_REG             (* (reg8 *) MultAcc_1_MACC_DFB__STAGEA)
#define MultAcc_1_MACC_STAGEA_PTR             (  (reg8 *) MultAcc_1_MACC_DFB__STAGEA)
#define MultAcc_1_MACC_STAGEAH_REG            (* (reg8 *) MultAcc_1_MACC_DFB__STAGEAH)
#define MultAcc_1_MACC_STAGEAH_PTR            (  (reg8 *) MultAcc_1_MACC_DFB__STAGEAH)
#define MultAcc_1_MACC_STAGEAM_REG            (* (reg8 *) MultAcc_1_MACC_DFB__STAGEAM)
#define MultAcc_1_MACC_STAGEAM_PTR            (  (reg8 *) MultAcc_1_MACC_DFB__STAGEAM)
#define MultAcc_1_MACC_STAGEB_REG             (* (reg8 *) MultAcc_1_MACC_DFB__STAGEB)
#define MultAcc_1_MACC_STAGEB_PTR             (  (reg8 *) MultAcc_1_MACC_DFB__STAGEB)
#define MultAcc_1_MACC_STAGEBH_REG            (* (reg8 *) MultAcc_1_MACC_DFB__STAGEBH)
#define MultAcc_1_MACC_STAGEBH_PTR            (  (reg8 *) MultAcc_1_MACC_DFB__STAGEBH)
#define MultAcc_1_MACC_STAGEBM_REG            (* (reg8 *) MultAcc_1_MACC_DFB__STAGEBM)
#define MultAcc_1_MACC_STAGEBM_PTR            (  (reg8 *) MultAcc_1_MACC_DFB__STAGEBM)
#define MultAcc_1_MACC_COHER_REG              (* (reg8 *) MultAcc_1_MACC_DFB__COHER)
#define MultAcc_1_MACC_COHER_PTR              (  (reg8 *) MultAcc_1_MACC_DFB__COHER)

#define MultAcc_1_MACC_DA_RAM_LAST_ELEM_PTR   (  (reg32 *) (MultAcc_1_MACC_DFB__DPB_SRAM_DATA - 4u))
#define MultAcc_1_MACC_DB_RAM_LAST_ELEM_PTR   (  (reg32 *) (MultAcc_1_MACC_DFB__CSA_SRAM_DATA - 4u))


/***************************************
*       Register Constants
***************************************/

#define MultAcc_1_MACC_PM_ACT_MSK             (MultAcc_1_MACC_DFB__PM_ACT_MSK)
#define MultAcc_1_MACC_PM_STBY_MSK            (MultAcc_1_MACC_DFB__PM_STBY_MSK)

#define MultAcc_1_MACC_RUN_MASK               (0x01u)
#define MultAcc_1_MACC_CORECLK_DISABLE        (0x04u)
#define MultAcc_1_MACC_RAM_DIR_BUS            (0x3Fu)
#define MultAcc_1_MACC_RAM_DIR_DFB            (0x00u)
#define MultAcc_1_MACC_RAM_DPA_DIR            (0x10u)
#define MultAcc_1_MACC_RAM_DPB_DIR            (0x20u)

/* Mask for bits within DFB Coherency Register */
#define MultAcc_1_MACC_STGA_KEY_SHIFT         (0x00u)
#define MultAcc_1_MACC_STGB_KEY_SHIFT         (0x02u)
#define MultAcc_1_MACC_HOLDA_KEY_SHIFT        (0x04u)
#define MultAcc_1_MACC_HOLDB_KEY_SHIFT        (0x06u)

#define MultAcc_1_MACC_STGA_KEY_LOW           ((uint8) (0x00u << MultAcc_1_MACC_STGA_KEY_SHIFT))
#define MultAcc_1_MACC_STGA_KEY_MID           ((uint8) (0x01u << MultAcc_1_MACC_STGA_KEY_SHIFT))
#define MultAcc_1_MACC_STGA_KEY_HIGH          ((uint8) (0x02u << MultAcc_1_MACC_STGA_KEY_SHIFT))
#define MultAcc_1_MACC_STGB_KEY_LOW           ((uint8) (0x00u << MultAcc_1_MACC_STGB_KEY_SHIFT))
#define MultAcc_1_MACC_STGB_KEY_MID           ((uint8) (0x01u << MultAcc_1_MACC_STGB_KEY_SHIFT))
#define MultAcc_1_MACC_STGB_KEY_HIGH          ((uint8) (0x02u << MultAcc_1_MACC_STGB_KEY_SHIFT))
#define MultAcc_1_MACC_HOLDA_KEY_LOW          ((uint8) (0x00u << MultAcc_1_MACC_HOLDA_KEY_SHIFT))
#define MultAcc_1_MACC_HOLDA_KEY_MID          ((uint8) (0x01u << MultAcc_1_MACC_HOLDA_KEY_SHIFT))
#define MultAcc_1_MACC_HOLDA_KEY_HIGH         ((uint8) (0x02u << MultAcc_1_MACC_HOLDA_KEY_SHIFT))
#define MultAcc_1_MACC_HOLDB_KEY_LOW          ((uint8) (0x00u << MultAcc_1_MACC_HOLDB_KEY_SHIFT))
#define MultAcc_1_MACC_HOLDB_KEY_MID          ((uint8) (0x01u << MultAcc_1_MACC_HOLDB_KEY_SHIFT))
#define MultAcc_1_MACC_HOLDB_KEY_HIGH         ((uint8) (0x02u << MultAcc_1_MACC_HOLDB_KEY_SHIFT))

/* Mask for bits within DFB Data Alignment Register */
#define MultAcc_1_MACC_STGA_DALIGN_SHIFT      (0x00u)
#define MultAcc_1_MACC_STGB_DALIGN_SHIFT      (0x01u)
#define MultAcc_1_MACC_HOLDA_DALIGN_SHIFT     (0x02u)
#define MultAcc_1_MACC_HOLDB_DALIGN_SHIFT     (0x03u)

#define MultAcc_1_MACC_STGA_DALIGN_LOW        ((uint8) (0x00u << MultAcc_1_MACC_STGA_DALIGN_SHIFT))
#define MultAcc_1_MACC_STGA_DALIGN_HIGH       ((uint8) (0x01u << MultAcc_1_MACC_STGA_DALIGN_SHIFT))
#define MultAcc_1_MACC_STGB_DALIGN_LOW        ((uint8) (0x00u << MultAcc_1_MACC_STGB_DALIGN_SHIFT))
#define MultAcc_1_MACC_STGB_DALIGN_HIGH       ((uint8) (0x01u << MultAcc_1_MACC_STGB_DALIGN_SHIFT))
#define MultAcc_1_MACC_HOLDA_DALIGN_LOW       ((uint8) (0x00u << MultAcc_1_MACC_HOLDA_DALIGN_SHIFT))
#define MultAcc_1_MACC_HOLDA_DALIGN_HIGH      ((uint8) (0x01u << MultAcc_1_MACC_HOLDA_DALIGN_SHIFT))
#define MultAcc_1_MACC_HOLDB_DALIGN_LOW       ((uint8) (0x00u << MultAcc_1_MACC_HOLDB_DALIGN_SHIFT))
#define MultAcc_1_MACC_HOLDB_DALIGN_HIGH      ((uint8) (0x01u << MultAcc_1_MACC_HOLDB_DALIGN_SHIFT))

/* Mask for bits within DFB Interrupt Control and Status Registers */
#define MultAcc_1_MACC_SEM_MASK               (0x07u)
#define MultAcc_1_MACC_SEM_ENABLE_SHIFT       (0x04u)
#define MultAcc_1_MACC_EVENT_MASK             (0x1Fu)
#define MultAcc_1_MACC_SR_EVENT_SHIFT         (0x03u)

#define MultAcc_1_MACC_HOLDA_SHIFT            (0x00u)
#define MultAcc_1_MACC_HOLDB_SHIFT            (0x01u)
#define MultAcc_1_MACC_SEMA0_SHIFT            (0x02u)
#define MultAcc_1_MACC_SEMA1_SHIFT            (0x03u)
#define MultAcc_1_MACC_SEMA2_SHIFT            (0x04u)

#define MultAcc_1_MACC_HOLDA                  ((uint8) (0x01u << MultAcc_1_MACC_HOLDA_SHIFT))
#define MultAcc_1_MACC_HOLDB                  ((uint8) (0x01u << MultAcc_1_MACC_HOLDB_SHIFT))
#define MultAcc_1_MACC_SEMA0                  ((uint8) (0x01u << MultAcc_1_MACC_SEMA0_SHIFT))
#define MultAcc_1_MACC_SEMA1                  ((uint8) (0x01u << MultAcc_1_MACC_SEMA1_SHIFT))
#define MultAcc_1_MACC_SEMA2                  ((uint8) (0x01u << MultAcc_1_MACC_SEMA2_SHIFT))

/* Mask for bits within DFB DMAREQ Control Register */
#define MultAcc_1_MACC_DMA_CTRL_MASK          (0x0Fu)
#define MultAcc_1_MACC_DMAREQ1_DISABLED       (0x00u)
#define MultAcc_1_MACC_DMAREQ1_HOLDA          (0x01u)
#define MultAcc_1_MACC_DMAREQ1_SEM0           (0x02u)
#define MultAcc_1_MACC_DMAREQ1_SEM1           (0x03u)
#define MultAcc_1_MACC_DMAREQ2_DISABLED       (0x00u)
#define MultAcc_1_MACC_DMAREQ2_HOLDB          (0x04u)
#define MultAcc_1_MACC_DMAREQ2_SEM0           (0x08u)
#define MultAcc_1_MACC_DMAREQ2_SEM1           (0x0Cu)

/* Mask for bits within DFB Semaphore Register */
#define MultAcc_1_MACC_SEMAPHORE0             (0x01u)
#define MultAcc_1_MACC_SEMAPHORE1             (0x02u)
#define MultAcc_1_MACC_SEMAPHORE2             (0x04u)

/* Mask for bits within Global Control Register */
#define MultAcc_1_MACC_DFB_RUN                (0x00u)
#define MultAcc_1_MACC_SEM0                   (0x01u)
#define MultAcc_1_MACC_SEM1                   (0x02u)
#define MultAcc_1_MACC_DFB_INTR               (0x03u)
#define MultAcc_1_MACC_SEM2                   (0x00u)
#define MultAcc_1_MACC_DPSIGN                 (0x04u)
#define MultAcc_1_MACC_DPTHRESH               (0x08u)
#define MultAcc_1_MACC_DPEQ                   (0x0Cu)
#define MultAcc_1_MACC_DFB_GBL1_OUT_MASK      (0xFCu)
#define MultAcc_1_MACC_DFB_GBL2_OUT_MASK      (0xF3u)

#endif /* End DFB Header File */


/* [] END OF FILE */

