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

static uint16_t reverse_bits(uint16_t x, int n) {
	uint16_t result = 0;
	for (int i = 0; i < n; i++, x >>= 1)
		result = (result << 1) | (x & 1U);
	return result;
}

/***
 * dir = forward: 0, inverse: 1
 * https://www.nayuki.io/res/free-small-fft-in-multiple-languages/fft.c
 */
static void fft256(float array[][2], const uint8_t dir) {
	const uint16_t n = 256;
	const uint8_t levels = 8; // log2(n)

	const uint8_t real =   dir & 1;
	const uint8_t imag = ~real & 1;

	for (uint16_t i = 0; i < n; i++) {
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

	// Cooley-Tukey decimation-in-time radix-2 FFT
	for (uint16_t size = 2; size <= n; size *= 2) {
		uint16_t halfsize = size / 2;
		uint16_t tablestep = n / size;
		for (uint16_t i = 0; i < n; i += size) {
			for (uint16_t j = i, k = 0; j < i + halfsize; j++, k += tablestep) {
				uint16_t l = j + halfsize;
				float tpre =  array[l][real] * cos(2 * M_PI * k / 256) + array[l][imag] * sin(2 * M_PI * k / 256);
				float tpim = -array[l][real] * sin(2 * M_PI * k / 256) + array[l][imag] * cos(2 * M_PI * k / 256);
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

static inline void fft256_forward(float array[][2]) {
	fft256(array, 0);
}

static inline void fft256_inverse(float array[][2]) {
	fft256(array, 1);
}
