/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#ifndef MultAcc_1_HEADER
#define MultAcc_1_HEADER

#include <project.h>
#include "MultAcc_1_MACC.h"

void MultAcc_1_Init();
void MultAcc_1_Enable();
void MultAcc_1_Start();
void MultAcc_1_Stop();
//void MultAcc_1_Sleep();
//void MultAcc_1_Wakeup();

void MultAcc_1_SetCoherency(uint8 coherencyKeyByte);
void MultAcc_1_SetDalign(uint8 dalignKeyByte);

#if !1
	void MultAcc_1_LoadInput(int32 A, int32 B) ;
#endif

int32 MultAcc_1_GetResult();
float MultAcc_1_Q23ToFloat(int32 input);
int32 MultAcc_1_FloatToQ23(float input);

#if 1
	void MultAcc_1_DMAInit(void);
	void MultAcc_1_DMAEnable(void);
	void MultAcc_1_DMADisable(void);
	void MultAcc_1_ProcessBlock(uint8 reset_accum);
	uint8 MultAcc_1_CheckStatus(uint8 flag_check);
#endif

#if 1
		
	//extern int32 MultAcc_1_data_a[8];
	//extern int32 MultAcc_1_data_b[8];
	
	#define MultAcc_1_DMA_GO 		0x01
	#define MultAcc_1_NRQ_RESET 		0x04
	#define MultAcc_1_MACC_RESET 	0x02
	#define MultAcc_1_ARRAY_SIZE  	8
	
	#define MultAcc_1_BLOCK_COMPLETE		0x01
	#define MultAcc_1_RESULT_READY		0x02
	#define MultAcc_1_NEW_RESULT			0x04
	#define MultAcc_1_NOT_BUSY			0x08
	#define MultAcc_1_OVERFLOW_DETECT	0x10
	#define MultAcc_1_RESET_STATUS		0x80
	
	#define MultAcc_1_RESET			1u
	#define MultAcc_1_CONTINUE		0u
	
	typedef struct
	{
		uint32 dat[MultAcc_1_ARRAY_SIZE];
	} MultAcc_1_ARRAY;
	
	extern MultAcc_1_ARRAY MultAcc_1_a;
	extern MultAcc_1_ARRAY MultAcc_1_b;
	
#endif

//----------------------------------------------------------

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component MultAcc_1 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */


/***************************************
*           API Constants
***************************************/

/* Channel Definitions */
#define MultAcc_1_A              (1u)
#define MultAcc_1_B              (0u)


/***************************************
*             Registers
***************************************/


#define MultAcc_1_SEM_REG                (* (reg8 *) MultAcc_1_MACC_DFB__SEMA)
#define MultAcc_1_SEM_PTR                (  (reg8 *) MultAcc_1_MACC_DFB__SEMA)

#define MultAcc_1_OUT8_PTR             	(  (reg8 *) MultAcc_1_MACC_DFB__HOLDAH)
#define MultAcc_1_OUT16_PTR             	(  (reg8 *) MultAcc_1_MACC_DFB__HOLDAM)
#define MultAcc_1_OUT24_PTR              (  (reg8 *) MultAcc_1_MACC_DFB__HOLDA)
#define MultAcc_1_OUT32_PTR              (  (reg8 *) MultAcc_1_MACC_DFB__HOLDA)

#define MultAcc_1_OUT8_REG             	(* (reg8 *) MultAcc_1_MACC_DFB__HOLDAH)
#define MultAcc_1_OUT16_REG             	(* (reg16 *) MultAcc_1_MACC_DFB__HOLDAM)
#define MultAcc_1_OUT24_REG              (* (reg24 *) MultAcc_1_MACC_DFB__HOLDA)
#define MultAcc_1_OUT32_REG              (* (reg32 *) MultAcc_1_MACC_DFB__HOLDA)

#define MultAcc_1_IN_A8_PTR            	(  (reg8 *) MultAcc_1_MACC_DFB__STAGEAH)
#define MultAcc_1_IN_A16_PTR           	(  (reg8 *) MultAcc_1_MACC_DFB__STAGEAM)
#define MultAcc_1_IN_A24_PTR         	(  (reg8 *) MultAcc_1_MACC_DFB__STAGEA)
#define MultAcc_1_IN_A32_PTR         	(  (reg8 *) MultAcc_1_MACC_DFB__STAGEA)

#define MultAcc_1_IN_A8_REG            	(* (reg8 *) MultAcc_1_MACC_DFB__STAGEAH)
#define MultAcc_1_IN_A16_REG           	(* (reg16 *) MultAcc_1_MACC_DFB__STAGEAM)
#define MultAcc_1_IN_A24_REG         	(* (reg24 *) MultAcc_1_MACC_DFB__STAGEA)
#define MultAcc_1_IN_A32_REG         	(* (reg32 *) MultAcc_1_MACC_DFB__STAGEA)

#define MultAcc_1_IN_B8_PTR            	(  (reg8 *) MultAcc_1_MACC_DFB__STAGEBH)
#define MultAcc_1_IN_B16_PTR            	(  (reg8 *) MultAcc_1_MACC_DFB__STAGEBM)
#define MultAcc_1_IN_B24_PTR           	(  (reg8 *) MultAcc_1_MACC_DFB__STAGEB)
#define MultAcc_1_IN_B32_PTR           	(  (reg8 *) MultAcc_1_MACC_DFB__STAGEB)

#define MultAcc_1_IN_B8_REG            	(* (reg8 *) MultAcc_1_MACC_DFB__STAGEBH)
#define MultAcc_1_IN_B16_REG            	(* (reg16 *) MultAcc_1_MACC_DFB__STAGEBM)
#define MultAcc_1_IN_B24_REG           	(* (reg24 *) MultAcc_1_MACC_DFB__STAGEB)
#define MultAcc_1_IN_B32_REG           	(* (reg24 *) MultAcc_1_MACC_DFB__STAGEB)


#define MultAcc_1_HOLDA_REG              (* (reg8 *) MultAcc_1_MACC_DFB__HOLDA)
#define MultAcc_1_HOLDA_PTR              (  (reg8 *) MultAcc_1_MACC_DFB__HOLDA)
#define MultAcc_1_HOLDAH_REG             (* (reg8 *) MultAcc_1_MACC_DFB__HOLDAH)
#define MultAcc_1_HOLDAH_PTR             (  (reg8 *) MultAcc_1_MACC_DFB__HOLDAH)
#define MultAcc_1_HOLDAM_REG             (* (reg8 *) MultAcc_1_MACC_DFB__HOLDAM)
#define MultAcc_1_HOLDAM_PTR             (  (reg8 *) MultAcc_1_MACC_DFB__HOLDAM)

#define MultAcc_1_STAGEA_REG             (* (reg8 *) MultAcc_1_MACC_DFB__STAGEA)
#define MultAcc_1_STAGEA_PTR             (  (reg8 *) MultAcc_1_MACC_DFB__STAGEA)
#define MultAcc_1_STAGEAH_REG            (* (reg8 *) MultAcc_1_MACC_DFB__STAGEAH)
#define MultAcc_1_STAGEAH_PTR            (  (reg8 *) MultAcc_1_MACC_DFB__STAGEAH)
#define MultAcc_1_STAGEAM_REG            (* (reg8 *) MultAcc_1_MACC_DFB__STAGEAM)
#define MultAcc_1_STAGEAM_PTR            (  (reg8 *) MultAcc_1_MACC_DFB__STAGEAM)
#define MultAcc_1_STAGEB_REG             (* (reg8 *) MultAcc_1_MACC_DFB__STAGEB)
#define MultAcc_1_STAGEB_PTR             (  (reg8 *) MultAcc_1_MACC_DFB__STAGEB)
#define MultAcc_1_STAGEBH_REG            (* (reg8 *) MultAcc_1_MACC_DFB__STAGEBH)
#define MultAcc_1_STAGEBH_PTR            (  (reg8 *) MultAcc_1_MACC_DFB__STAGEBH)
#define MultAcc_1_STAGEBM_REG            (* (reg8 *) MultAcc_1_MACC_DFB__STAGEBM)
#define MultAcc_1_STAGEBM_PTR            (  (reg8 *) MultAcc_1_MACC_DFB__STAGEBM)


/***************************************
*       Register Constants
***************************************/


/* Mask for bits within DFB Interrupt Control and Status Registers */
#define MultAcc_1_SEM_MASK               (0x07u)

/* Mask for bits within DFB Semaphore Register */
#define MultAcc_1_NEWFLAG             	(0x01u)
#define MultAcc_1_OVERFLOWDET            (0x02u)
#define MultAcc_1_DEBUG             		(0x04u)

#endif
//[] END OF FILE
