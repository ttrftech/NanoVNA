/* 
 * fft.h is Based on
 * Free FFT and convolution (C)
 * 
 * Copyright (c) 2019 Project Nayuki. (MIT License)
 * https://www.nayuki.io/page/free-small-fft-in-multiple-languages
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 * - The above copyright notice and this permission notice shall be included in
 *   all copies or substantial portions of the Software.
 * - The Software is provided "as is", without warranty of any kind, express or
 *   implied, including but not limited to the warranties of merchantability,
 *   fitness for a particular purpose and noninfringement. In no event shall the
 *   authors or copyright holders be liable for any claim, damages or other
 *   liability, whether in an action of contract, tort or otherwise, arising from,
 *   out of or in connection with the Software or the use or other dealings in the
 *   Software.
 */


#include <math.h>
#include <stdint.h>

#define FFT_USE_SIN_COS_TABLE

static uint16_t reverse_bits(uint16_t x, int n) {
	uint16_t result = 0;
	int i;
	for (i = 0; i < n; i++, x >>= 1)
		result = (result << 1) | (x & 1U);
	return result;
}
#ifdef FFT_USE_SIN_COS_TABLE
static const float sin_table[] = {
	/*
	 * float has about 7.2 digits of precision
		for (uint8_t i = 0; i < FFT_SIZE - (FFT_SIZE / 4); i++) {
			printf("% .8f,%c", sin(2 * M_PI * i / FFT_SIZE), i % 8 == 7 ? '\n' : ' ');
		}
	*/
	 0.00000000,  0.02454123,  0.04906767,  0.07356456,  0.09801714,  0.12241068,  0.14673047,  0.17096189,
	 0.19509032,  0.21910124,  0.24298018,  0.26671276,  0.29028468,  0.31368174,  0.33688985,  0.35989504,
	 0.38268343,  0.40524131,  0.42755509,  0.44961133,  0.47139674,  0.49289819,  0.51410274,  0.53499762,
	 0.55557023,  0.57580819,  0.59569930,  0.61523159,  0.63439328,  0.65317284,  0.67155895,  0.68954054,
	 0.70710678,  0.72424708,  0.74095113,  0.75720885,  0.77301045,  0.78834643,  0.80320753,  0.81758481,
	 0.83146961,  0.84485357,  0.85772861,  0.87008699,  0.88192126,  0.89322430,  0.90398929,  0.91420976,
	 0.92387953,  0.93299280,  0.94154407,  0.94952818,  0.95694034,  0.96377607,  0.97003125,  0.97570213,
	 0.98078528,  0.98527764,  0.98917651,  0.99247953,  0.99518473,  0.99729046,  0.99879546,  0.99969882,
	 1.00000000,  0.99969882,  0.99879546,  0.99729046,  0.99518473,  0.99247953,  0.98917651,  0.98527764,
	 0.98078528,  0.97570213,  0.97003125,  0.96377607,  0.95694034,  0.94952818,  0.94154407,  0.93299280,
	 0.92387953,  0.91420976,  0.90398929,  0.89322430,  0.88192126,  0.87008699,  0.85772861,  0.84485357,
	 0.83146961,  0.81758481,  0.80320753,  0.78834643,  0.77301045,  0.75720885,  0.74095113,  0.72424708,
	 0.70710678,  0.68954054,  0.67155895,  0.65317284,  0.63439328,  0.61523159,  0.59569930,  0.57580819,
	 0.55557023,  0.53499762,  0.51410274,  0.49289819,  0.47139674,  0.44961133,  0.42755509,  0.40524131,
	 0.38268343,  0.35989504,  0.33688985,  0.31368174,  0.29028468,  0.26671276,  0.24298018,  0.21910124,
	 0.19509032,  0.17096189,  0.14673047,  0.12241068,  0.09801714,  0.07356456,  0.04906767,  0.02454123,
	 0.00000000, -0.02454123, -0.04906767, -0.07356456, -0.09801714, -0.12241068, -0.14673047, -0.17096189,
	-0.19509032, -0.21910124, -0.24298018, -0.26671276, -0.29028468, -0.31368174, -0.33688985, -0.35989504,
	-0.38268343, -0.40524131, -0.42755509, -0.44961133, -0.47139674, -0.49289819, -0.51410274, -0.53499762,
	-0.55557023, -0.57580819, -0.59569930, -0.61523159, -0.63439328, -0.65317284, -0.67155895, -0.68954054,
	-0.70710678, -0.72424708, -0.74095113, -0.75720885, -0.77301045, -0.78834643, -0.80320753, -0.81758481,
	-0.83146961, -0.84485357, -0.85772861, -0.87008699, -0.88192126, -0.89322430, -0.90398929, -0.91420976,
	-0.92387953, -0.93299280, -0.94154407, -0.94952818, -0.95694034, -0.96377607, -0.97003125, -0.97570213,
	-0.98078528, -0.98527764, -0.98917651, -0.99247953, -0.99518473, -0.99729046, -0.99879546, -0.99969882,
};
const float* const cos_table = &sin_table[64];
#endif

/***
 * dir = forward: 0, inverse: 1
 * https://www.nayuki.io/res/free-small-fft-in-multiple-languages/fft.c
 */
static void fft256(float array[][2], const uint8_t dir) {
	const uint16_t n = 256;
	const uint8_t levels = 8; // log2(n)

	const uint8_t real =   dir & 1;
	const uint8_t imag = ~real & 1;
	uint16_t i;
	uint16_t size;

	for (i = 0; i < n; i++) {
		uint16_t j = reverse_bits(i, levels);
		if (j > i) {
			float temp = array[i][real];
			array[i][real] = array[j][real];
			array[j][real] = temp;
			temp = array[i][imag];
			array[i][imag] = array[j][imag];
			array[j][imag] = temp;
		}
	}
#ifdef FFT_USE_SIN_COS_TABLE
	// Cooley-Tukey decimation-in-time radix-2 FFT
	for (size = 2; size <= n; size *= 2) {
		uint16_t halfsize = size / 2;
		uint16_t tablestep = n / size;
		for (i = 0; i < n; i += size) {
			uint16_t j, k;
			for (j = i, k = 0; j < i + halfsize; j++, k += tablestep) {
				uint16_t l = j + halfsize;
				float tpre =  array[l][real] * cos_table[k] + array[l][imag] * sin_table[k];
				float tpim = -array[l][real] * sin_table[k] + array[l][imag] * cos_table[k];
				array[l][real] = array[j][real] - tpre;
				array[l][imag] = array[j][imag] - tpim;
				array[j][real] += tpre;
				array[j][imag] += tpim;
			}
		}
//		if (size == n)  // Prevent overflow in 'size *= 2'
//			break;
	}
#else
	// Cooley-Tukey decimation-in-time radix-2 FFT
	for (size = 2; size <= n; size *= 2) {
		uint16_t halfsize = size / 2;
		uint16_t tablestep = n / size;
		for (i = 0; i < n; i += size) {
			uint16_t j, k;
			for (j = i, k = 0; j < i + halfsize; j++, k += tablestep) {
				uint16_t l = j + halfsize;
				float c = cos((2 * VNA_PI / 256) * k);
				float s = sin((2 * VNA_PI / 256) * k);
				float tpre =  array[l][real] * c + array[l][imag] * s;
				float tpim = -array[l][real] * s + array[l][imag] * c;
				array[l][real] = array[j][real] - tpre;
				array[l][imag] = array[j][imag] - tpim;
				array[j][real] += tpre;
				array[j][imag] += tpim;
			}
		}
//		if (size == n)  // Prevent overflow in 'size *= 2'
//			break;
	}
#endif
}

static inline void fft256_forward(float array[][2]) {
	fft256(array, 0);
}

static inline void fft256_inverse(float array[][2]) {
	fft256(array, 1);
}
