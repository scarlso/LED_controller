/* ----------------------------------------------------------------------
** Include Files
** ------------------------------------------------------------------- */
#include <project.h>
#include "arm_math.h"
#include "arm_const_structs.h"
#include "math_helper.h"


/* ----------------------------------------------------------------------
 * DMA Definitions
 * ------------------------------------------------------------------- */
/*First buffer*/
uint8 DMA_1_Chan;
uint8 DMA_1_TD[1];

uint8 DMA_2_Chan;
uint8 DMA_2_TD[1];

uint8 ADC2Filter_DMA_Chan;
uint8 ADC2Filter_DMA_TD[2];

uint8 DMA_3_Chan;
uint8 DMA_3_TD[1];

uint8 DMA_4_Chan;
uint8 DMA_4_TD[1];

#define buffer_size 1024
/* Definitions */
#define DMA_1_BYTES_PER_BURST 2
#define DMA_1_REQUEST_PER_BURST 1
#define DMA_1_SRC_BASE (CYDEV_PERIPH_BASE)
#define DMA_1_DST_BASE (CYDEV_SRAM_BASE)
#define buffersize buffer_size*DMA_1_BYTES_PER_BURST
/*Second buffer*/

#define DMA_2_BYTES_PER_BURST 2
#define DMA_2_REQUEST_PER_BURST 0
#define DMA_2_SRC_BASE (CYDEV_SRAM_BASE)
#define DMA_2_DST_BASE (CYDEV_SRAM_BASE)

#define ADC2Filter_DMA_BYTES_PER_BURST 2
#define ADC2Filter_DMA_REQUEST_PER_BURST 1
#define ADC2Filter_DMA_SRC_BASE (CYDEV_PERIPH_BASE)
#define ADC2Filter_DMA_DST_BASE (CYDEV_PERIPH_BASE)

#define DMA_3_BYTES_PER_BURST 2
#define DMA_3_REQUEST_PER_BURST 1
#define DMA_3_SRC_BASE (CYDEV_PERIPH_BASE)
#define DMA_3_DST_BASE (CYDEV_SRAM_BASE)

#define DMA_4_BYTES_PER_BURST 2
#define DMA_4_REQUEST_PER_BURST 0
#define DMA_4_SRC_BASE (CYDEV_SRAM_BASE)
#define DMA_4_DST_BASE (CYDEV_SRAM_BASE)

/*global vairables*/

int16 LPF_samples[buffer_size];
int16 LPF_buffer[buffer_size];
int16 Buffer_samples[buffer_size];
int16 ADC_samples[buffer_size];
q15_t magoutput[buffer_size/2];
static uint32 fftlength=buffer_size/2;

static volatile CYBIT isr_BC_flag = 0;
static volatile CYBIT isr_LPF_buffer_complete_flag = 0;
//MultAcc_1_ARRAY CYCODE accumulatee[windowsize];
/*functions*/
void Configure_DMA();
//uint32 moving_average(uint32 *specflux);

/*ISR routine*/
CY_ISR_PROTO(Buffer_complete);
CY_ISR(Buffer_complete){
    isr_BC_flag=1;
}
CY_ISR_PROTO(LPF_buffer_complete);
CY_ISR(LPF_buffer_complete){
    isr_LPF_buffer_complete_flag=1;
    CyDmaChEnable(DMA_4_Chan, 1);
}

/*main*/
void main(void)
{   
    /*Preliminary parts not important*/  
    LCD_Char_1_Start();
    ADC_DelSig_1_Start();
    ADC_DelSig_1_StartConvert();

    Configure_DMA(); 
    isr_1_StartEx(Buffer_complete);
    isr_2_StartEx(LPF_buffer_complete);
    
    ADC_DelSig_1_SetCoherency(ADC_DelSig_1_COHER_MID);   
    
    Filter_SetDalign(Filter_STAGEA_DALIGN,Filter_ENABLED);
    Filter_SetDalign(Filter_HOLDA_DALIGN,Filter_ENABLED);
    Filter_SetCoherency(Filter_STAGEA_COHER,Filter_KEY_MID);
    Filter_SetCoherency(Filter_HOLDA_COHER,Filter_KEY_MID);
    Filter_SetCoherency(Filter_CHANNEL_A,Filter_KEY_MID);  
    
    Filter_SetDalign(Filter_STAGEB_DALIGN,Filter_ENABLED);
    Filter_SetDalign(Filter_HOLDB_DALIGN,Filter_ENABLED);
    Filter_SetCoherency(Filter_STAGEB_COHER,Filter_KEY_MID);
    Filter_SetCoherency(Filter_HOLDB_COHER,Filter_KEY_MID);
    Filter_SetCoherency(Filter_CHANNEL_B,Filter_KEY_MID);

    CyGlobalIntEnable;

    Filter_Start();

    /*Writes ADC values to ADC_samples array*/
    while(1){
    if (isr_BC_flag==1){        
        arm_cfft_q15(&arm_cfft_sR_q15_len256, Buffer_samples, 0, 1); 
        arm_cmplx_mag_q15(Buffer_samples, magoutput, fftlength); 
        CyDmaChEnable(DMA_2_Chan, 1);
        
        isr_BC_flag=0;
        isr_1_ClearPending();
    }
}
}

void Configure_DMA(){

ADC2Filter_DMA_Chan = ADC2Filter_DMA_DmaInitialize(ADC2Filter_DMA_BYTES_PER_BURST, ADC2Filter_DMA_REQUEST_PER_BURST, 
    HI16(ADC2Filter_DMA_SRC_BASE), HI16(ADC2Filter_DMA_DST_BASE));
ADC2Filter_DMA_TD[0] = CyDmaTdAllocate();
ADC2Filter_DMA_TD[1] = CyDmaTdAllocate();
CyDmaTdSetConfiguration(ADC2Filter_DMA_TD[0], 2, ADC2Filter_DMA_TD[1], TD_AUTO_EXEC_NEXT);
CyDmaTdSetConfiguration(ADC2Filter_DMA_TD[1], 2, ADC2Filter_DMA_TD[0], ADC2Filter_DMA__TD_TERMOUT_EN);
CyDmaTdSetAddress(ADC2Filter_DMA_TD[0], LO16((uint32)ADC_DelSig_1_DEC_SAMP_PTR), LO16((uint32)Filter_STAGEA_PTR));
CyDmaTdSetAddress(ADC2Filter_DMA_TD[1], LO16((uint32)ADC_DelSig_1_DEC_SAMP_PTR), LO16((uint32)Filter_STAGEB_PTR));
CyDmaChSetInitialTd(ADC2Filter_DMA_Chan, ADC2Filter_DMA_TD[0]);
CyDmaChEnable(ADC2Filter_DMA_Chan, 1);


/*Second buffer*/
DMA_2_Chan = DMA_2_DmaInitialize(DMA_2_BYTES_PER_BURST, DMA_2_REQUEST_PER_BURST, 
    HI16(DMA_2_SRC_BASE), HI16(DMA_2_DST_BASE));
DMA_2_TD[0] = CyDmaTdAllocate();
CyDmaTdSetConfiguration(DMA_2_TD[0], buffersize, CY_DMA_DISABLE_TD /* DMA_2_TD[0]*/, DMA_2__TD_TERMOUT_EN | TD_INC_SRC_ADR | TD_INC_DST_ADR);
CyDmaTdSetAddress(DMA_2_TD[0], LO16((uint32)ADC_samples), LO16((uint32)Buffer_samples));
CyDmaChSetInitialTd(DMA_2_Chan, DMA_2_TD[0]);
CyDmaChEnable(DMA_2_Chan, 1);

/*First buffer*/
DMA_1_Chan = DMA_1_DmaInitialize(DMA_1_BYTES_PER_BURST, DMA_1_REQUEST_PER_BURST, 
    HI16(DMA_1_SRC_BASE), HI16(DMA_1_DST_BASE));
DMA_1_TD[0] = CyDmaTdAllocate();
CyDmaTdSetConfiguration(DMA_1_TD[0], buffersize,/* CY_DMA_DISABLE_TD*/ DMA_1_TD[0], DMA_1__TD_TERMOUT_EN | TD_INC_DST_ADR);
CyDmaTdSetAddress(DMA_1_TD[0], LO16((uint32)Filter_HOLDA_PTR), LO16((uint32)ADC_samples));
CyDmaChSetInitialTd(DMA_1_Chan, DMA_1_TD[0]);
CyDmaChEnable(DMA_1_Chan, 1);

DMA_3_Chan = DMA_3_DmaInitialize(DMA_3_BYTES_PER_BURST, DMA_3_REQUEST_PER_BURST, 
    HI16(DMA_3_SRC_BASE), HI16(DMA_3_DST_BASE));
DMA_3_TD[0] = CyDmaTdAllocate();
CyDmaTdSetConfiguration(DMA_3_TD[0], buffersize, DMA_3_TD[0], DMA_3__TD_TERMOUT_EN | TD_INC_DST_ADR);
CyDmaTdSetAddress(DMA_3_TD[0], LO16((uint32)Filter_HOLDB_PTR), LO16((uint32)LPF_samples));
CyDmaChSetInitialTd(DMA_3_Chan, DMA_3_TD[0]);
CyDmaChEnable(DMA_3_Chan, 1);

DMA_4_Chan = DMA_4_DmaInitialize(DMA_4_BYTES_PER_BURST, DMA_4_REQUEST_PER_BURST, 
    HI16(DMA_4_SRC_BASE), HI16(DMA_4_DST_BASE));
DMA_4_TD[0] = CyDmaTdAllocate();
CyDmaTdSetConfiguration(DMA_4_TD[0], buffersize, CY_DMA_DISABLE_TD /*DMA_4_TD[0]*/, DMA_4__TD_TERMOUT_EN | TD_INC_SRC_ADR | TD_INC_DST_ADR);
CyDmaTdSetAddress(DMA_4_TD[0], LO16((uint32)LPF_samples), LO16((uint32)LPF_buffer));
CyDmaChSetInitialTd(DMA_4_Chan, DMA_4_TD[0]);
CyDmaChEnable(DMA_4_Chan, 1);

}