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

static uint8_t reverse_bits(uint8_t x, int n) {
	uint8_t result = 0;
	for (int i = 0; i < n; i++, x >>= 1)
		result = (result << 1) | (x & 1U);
	return result;
}

static const float sin_table[] = {
	/*
	 * float has about 7.2 digits of precision
		for (uint8_t i = 0; i < 96; i++) {
			printf("% .8f,%c", sin(2 * M_PI * i / n), i % 8 == 7 ? '\n' : ' ');
		}
	*/
	 0.00000000,  0.04906767,  0.09801714,  0.14673047,  0.19509032,  0.24298018,  0.29028468,  0.33688985,
	 0.38268343,  0.42755509,  0.47139674,  0.51410274,  0.55557023,  0.59569930,  0.63439328,  0.67155895,
	 0.70710678,  0.74095113,  0.77301045,  0.80320753,  0.83146961,  0.85772861,  0.88192126,  0.90398929,
	 0.92387953,  0.94154407,  0.95694034,  0.97003125,  0.98078528,  0.98917651,  0.99518473,  0.99879546,
	 1.00000000,  0.99879546,  0.99518473,  0.98917651,  0.98078528,  0.97003125,  0.95694034,  0.94154407,
	 0.92387953,  0.90398929,  0.88192126,  0.85772861,  0.83146961,  0.80320753,  0.77301045,  0.74095113,
	 0.70710678,  0.67155895,  0.63439328,  0.59569930,  0.55557023,  0.51410274,  0.47139674,  0.42755509,
	 0.38268343,  0.33688985,  0.29028468,  0.24298018,  0.19509032,  0.14673047,  0.09801714,  0.04906767,
	 0.00000000, -0.04906767, -0.09801714, -0.14673047, -0.19509032, -0.24298018, -0.29028468, -0.33688985,
	-0.38268343, -0.42755509, -0.47139674, -0.51410274, -0.55557023, -0.59569930, -0.63439328, -0.67155895,
	-0.70710678, -0.74095113, -0.77301045, -0.80320753, -0.83146961, -0.85772861, -0.88192126, -0.90398929,
	-0.92387953, -0.94154407, -0.95694034, -0.97003125, -0.98078528, -0.98917651, -0.99518473, -0.99879546,
};

/***
 * dir = forward: 0, inverse: 1
 * https://www.nayuki.io/res/free-small-fft-in-multiple-languages/fft.c
 */
static void fft128(float array[][2], const uint8_t dir) {
	const uint8_t n = 128;
	const uint8_t levels = 7; // log2(n)
	const float* const cos_table = &sin_table[32];

	const uint8_t real =   dir & 1;
	const uint8_t imag = ~real & 1;

	for (uint8_t i = 0; i < n; i++) {
		uint8_t j = reverse_bits(i, levels);
		if (j > i) {
			float temp = array[i][real];
			array[i][real] = array[j][real];
			array[j][real] = temp;
			temp = array[i][imag];
			array[i][imag] = array[j][imag];
			array[j][imag] = temp;
		}
	}

	// Cooley-Tukey decimation-in-time radix-2 FFT
	for (uint8_t size = 2; size <= n; size *= 2) {
		uint8_t halfsize = size / 2;
		uint8_t tablestep = n / size;
		for (uint8_t i = 0; i < n; i += size) {
			for (uint8_t j = i, k = 0; j < i + halfsize; j++, k += tablestep) {
				uint8_t l = j + halfsize;
				float tpre =  array[l][real] * cos_table[k] + array[l][imag] * sin_table[k];
				float tpim = -array[l][real] * sin_table[k] + array[l][imag] * cos_table[k] ;
				array[l][real] = array[j][real] - tpre;
				array[l][imag] = array[j][imag] - tpim;
				array[j][real] += tpre;
				array[j][imag] += tpim;
			}
		}
		if (size == n)  // Prevent overflow in 'size *= 2'
			break;
	}
}

static inline void fft128_forward(float array[][2]) {
	fft128(array, 0);
}

static inline void fft128_inverse(float array[][2]) {
	fft128(array, 1);
}
