#include <arm_math.h>

#define TEST_LENGTH_SAMPLES 2048

void SpectralFlux_q15(
	q15_t * testinput0,//current  input
	q15_t * testinput1,//previous input
	q15_t * SpectralFlux//output
	);