/* ----------------------------------------------------------------------
*	Author: Scott Carlson 	Date:07/14/2015
*	Purpose: This code models an onset detector that uses the Spectral Flux.
*		 The spectral flux is calculated from 4 moving windows and the
*		 Onset is determined by looking for the peak in the middle of
*		 the 3 spectral flux values.
*
*
*
*
*------------------------------------------------------------------------
*/


#include "arm_math.h"
#include "arm_const_structs.h"

#define TEST_LENGTH_SAMPLES 2048

/* -------------------------------------------------------------------
* External Input and Output buffer Declarations for Spectral Flux
* ------------------------------------------------------------------- */
extern q15_t testInput0_q15_10khz[TEST_LENGTH_SAMPLES];
extern q15_t testInput1_q15_10khz[TEST_LENGTH_SAMPLES];
extern q15_t testInput2_q15_10khz[TEST_LENGTH_SAMPLES];
extern q15_t testInput3_q15_10khz[TEST_LENGTH_SAMPLES];
static q15_t testOutput0[TEST_LENGTH_SAMPLES/2];
static q15_t testOutput1[TEST_LENGTH_SAMPLES/2];
static q15_t testOutput2[TEST_LENGTH_SAMPLES/2];
static q15_t testOutput3[TEST_LENGTH_SAMPLES/2];
static q15_t Difference0[TEST_LENGTH_SAMPLES/2];
static q15_t Difference1[TEST_LENGTH_SAMPLES/2];
static q15_t Difference2[TEST_LENGTH_SAMPLES/2];
static q15_t SpectralFlux0;
static q15_t SpectralFlux1;
static q15_t SpectralFlux2;
/* ------------------------------------------------------------------
* Global variables for Spectral Flux
* ------------------------------------------------------------------- */
uint32_t fftSize = 1024;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;
uint8 state;
uint8 nextstate;
/* Reference index at which max energy of bin ocuurs */
uint32_t refIndex = 213, testIndex = 0;

/* ----------------------------------------------------------------------
* Max magnitude FFT Bin test
* ------------------------------------------------------------------- */

int32_t main(void)
{
  arm_status status;
  q15_t maxValue;
  status = ARM_MATH_SUCCESS;

while(1){
  switch(state){
    case 0x01:
        arm_cfft_q15(&arm_cfft_sR_q15_len1024, testInput3_q31_10khz, ifftFlag, doBitReverse);
        arm_cmplx_mag_q15(testInput3_q15_10khz, testOutput3, fftSize);
        if (enable==1){ nextstate=0x02;}
	break;
    case 0x02:
        arm_cfft_q15(&arm_cfft_sR_q15_len1024, testInput2_q31_10khz, ifftFlag, doBitReverse);
        arm_cmplx_mag_q15(testInput2_q15_10khz, testOutput2, fftSize); 
	arm_mat_sub_q15(testOutput3,testOutput2,Difference2);   
	for(i=0; i< TEST_LENGTH_SAMPLES/2; i++)
  	  {
   	     arm_add_q15(&SpectralFlux2, &Difference2[i], &SpectralFlux2, 1);
	  }	
        if (enable==1){ nextstate=0x03;}
	break;
    case 0x03:
        arm_cfft_q15(&arm_cfft_sR_q15_len1024, testInput1_q31_10khz, ifftFlag, doBitReverse);
	arm_cmplx_mag_q15(testInput1_q15_10khz, testOutput1, fftSize); 
	arm_mat_sub_q15(testOutput2,testOutput1,Difference1); 
	for(i=0; i< TEST_LENGTH_SAMPLES/2; i++)
  	  {
   	     arm_add_q15(&SpectralFlux1, &Difference1[i], &SpectralFlux1, 1);
	  }	
        if (enable==1){ nextstate=0x04;}
	break;
    case 0x04:
        SpectralFlux2=SpectralFlux1;
        SpectralFlux1=SpectralFlux0;
        arm_cfft_q15(&arm_cfft_sR_q15_len1024, testInput0_q31_10khz, ifftFlag, doBitReverse);
        arm_cmplx_mag_q15(testInput0_q15_10khz, testOutput0, fftSize);	
	arm_mat_sub_q15(testOutput1,testOutput0,Difference0); 
	testOutput1=testOutput0;
	for(i=0; i< TEST_LENGTH_SAMPLES/2; i++)
  	  {
   	     arm_add_q15(&SpectralFlux0, &Difference0[i], &SpectralFlux0, 1);
	  }
	if ((SpectralFlux[0]>SpectralFlux[1]) && (SpectralFlux[2]>SpectralFlux[1]))
	  {
	     Onset_Write(1);
	     nextstate=0x05;
	  }
	else
	  {
	     nextstate=0x04; 
	  }
    case 0x05:
	Onset_Write(0);
	nextstate=0x04;
	break;
    default: 
	if (enable==1){nextstate=0x01;}
}
state=nextstate;	           
} 	

  
  	

                 
}


