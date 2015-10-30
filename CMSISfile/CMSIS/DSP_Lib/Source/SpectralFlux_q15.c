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


#include <arm_math.h>

#define TEST_LENGTH_SAMPLES 2048

/* -------------------------------------------------------------------
* External Input and Output buffer Declarations for Spectral Flux
* ------------------------------------------------------------------- */
static q15_t testOutput0[TEST_LENGTH_SAMPLES/2];
static q15_t testOutput1[TEST_LENGTH_SAMPLES/2];
static q15_t Difference[TEST_LENGTH_SAMPLES/2];

/* ------------------------------------------------------------------
* Global variables for Spectral Flux
* ------------------------------------------------------------------- */



/* ----------------------------------------------------------------------
* Max magnitude Spectral Flux
* ------------------------------------------------------------------- */

void SpectralFlux(
	q15_t * testInput0,//current  input
	q15_t * testInput1,//previous input
	q15_t * SpectralFlux//outputssss
	)
{
  uint16 i=0;
  arm_status status;
  status = ARM_MATH_SUCCESS;
  arm_matrix_instance_q15 fftmatrix0;
  arm_matrix_instance_q15 fftmatrix1;
  arm_matrix_instance_q15 DiffMatrix;
  uint32_t srcRows, srcColumns; 
  arm_cfft_radix4_instance_q15 arm_cfft_sR_q15_len1024;
  uint16_t fftSize = 1024;
  uint8_t ifftFlag = 0;
  uint8_t doBitReverse = 1;
    
    srcRows = 1;
    srcColumns = TEST_LENGTH_SAMPLES/2;
    
    arm_cfft_radix4_init_q15(&arm_cfft_sR_q15_len1024,fftSize,ifftFlag,doBitReverse);
    
    arm_cfft_q15(&arm_cfft_sR_q15_len1024, testInput0, ifftFlag, doBitReverse);
    arm_cmplx_mag_q15(testInput0, testOutput0, fftSize); 

    arm_cfft_q15(&arm_cfft_sR_q15_len1024, testInput1, ifftFlag, doBitReverse);
	arm_cmplx_mag_q15(testInput1, testOutput1, fftSize);
    
    arm_mat_init_q15( &fftmatrix0, srcRows, srcColumns, testOutput0);
    arm_mat_init_q15( &fftmatrix1, srcRows, srcColumns, testOutput1);
    arm_mat_init_q15( &DiffMatrix, srcRows, srcColumns, testOutput1);
    arm_mat_sub_q15(&fftmatrix1,&fftmatrix0,&DiffMatrix); 
	for(i=0; i< TEST_LENGTH_SAMPLES/2; i++)
  	  {
   	     arm_add_q15(&SpectralFlux, &Difference[i], &SpectralFlux, 1);
	  }	               
}


