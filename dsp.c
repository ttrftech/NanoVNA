/*
 * Copyright (c) 2014-2015, TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
 * All rights reserved.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * The software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <arm_math.h>
#include "nanovna.h"

#ifdef ENABLED_DUMP
int16_t samp_buf[SAMPLE_LEN];
int16_t ref_buf[SAMPLE_LEN];
#endif

#ifdef USE_VARIABLE_OFFSET
static int16_t sincos_tbl[AUDIO_SAMPLES_COUNT][2];
void generate_DSP_Table(int offset){
  float audio_freq  = AUDIO_ADC_FREQ;
  // N = offset * AUDIO_SAMPLES_COUNT / audio_freq; should be integer
  // AUDIO_SAMPLES_COUNT = N * audio_freq / offset; N - minimum integer value for get integer AUDIO_SAMPLES_COUNT
  // Bandwidth on one step = audio_freq / AUDIO_SAMPLES_COUNT
  float step = 2 * VNA_PI * offset / audio_freq;
  float v = step/2;
  for (int i=0; i<AUDIO_SAMPLES_COUNT; i++){
    sincos_tbl[i][0] = sin(v)*32768.0 + 0.5;
    sincos_tbl[i][1] = cos(v)*32768.0 + 0.5;
    v+=step;
  }
}
#elif FREQUENCY_OFFSET==6000*(AUDIO_ADC_FREQ/AUDIO_SAMPLES_COUNT/1000)
// static Table for 12kHz IF and 96kHz ADC (or 6kHz IF and 48kHz ADC) audio ADC
static const int16_t sincos_tbl[48][2] = {
  { 6393, 32138}, { 27246, 18205}, { 32138,-6393}, { 18205,-27246},
  {-6393,-32138}, {-27246,-18205}, {-32138, 6393}, {-18205, 27246},
  { 6393, 32138}, { 27246, 18205}, { 32138,-6393}, { 18205,-27246},
  {-6393,-32138}, {-27246,-18205}, {-32138, 6393}, {-18205, 27246},
  { 6393, 32138}, { 27246, 18205}, { 32138,-6393}, { 18205,-27246},
  {-6393,-32138}, {-27246,-18205}, {-32138, 6393}, {-18205, 27246},
  { 6393, 32138}, { 27246, 18205}, { 32138,-6393}, { 18205,-27246},
  {-6393,-32138}, {-27246,-18205}, {-32138, 6393}, {-18205, 27246},
  { 6393, 32138}, { 27246, 18205}, { 32138,-6393}, { 18205,-27246},
  {-6393,-32138}, {-27246,-18205}, {-32138, 6393}, {-18205, 27246},
  { 6393, 32138}, { 27246, 18205}, { 32138,-6393}, { 18205,-27246},
  {-6393,-32138}, {-27246,-18205}, {-32138, 6393}, {-18205, 27246}
};
#elif FREQUENCY_OFFSET==5000*(AUDIO_ADC_FREQ/AUDIO_SAMPLES_COUNT/1000)
// static Table for 10kHz IF and 96kHz ADC (or 5kHz IF and 48kHz ADC) audio ADC
static const int16_t sincos_tbl[48][2] = {
  { 10533,  31029 }, { 27246,  18205 }, { 32698,  -2143 }, { 24636, -21605 },
  {  6393, -32138 }, {-14493, -29389 }, {-29389, -14493 }, {-32138,   6393 },
  {-21605,  24636 }, { -2143,  32698 }, { 18205,  27246 }, { 31029,  10533 },
  { 31029, -10533 }, { 18205, -27246 }, { -2143, -32698 }, {-21605, -24636 },
  {-32138,  -6393 }, {-29389,  14493 }, {-14493,  29389 }, {  6393,  32138 },
  { 24636,  21605 }, { 32698,   2143 }, { 27246, -18205 }, { 10533, -31029 },
  {-10533, -31029 }, {-27246, -18205 }, {-32698,   2143 }, {-24636,  21605 },
  { -6393,  32138 }, { 14493,  29389 }, { 29389,  14493 }, { 32138,  -6393 },
  { 21605, -24636 }, { 2143,  -32698 }, {-18205, -27246 }, {-31029, -10533 },
  {-31029,  10533 }, {-18205,  27246 }, {  2143,  32698 }, { 21605,  24636 },
  { 32138,   6393 }, { 29389, -14493 }, { 14493, -29389 }, { -6393, -32138 },
  {-24636, -21605 }, {-32698,  -2143 }, {-27246,  18205 }, {-10533,  31029 }
};
#elif FREQUENCY_OFFSET==4000*(AUDIO_ADC_FREQ/AUDIO_SAMPLES_COUNT/1000)
// static Table for 8kHz IF and 96kHz audio ADC (or 4kHz IF and 48kHz ADC) audio ADC
static const int16_t sincos_tbl[48][2] = {
  {  4277, 32488}, { 19948, 25997}, { 30274, 12540}, { 32488, -4277},
  { 25997,-19948}, { 12540,-30274}, { -4277,-32488}, {-19948,-25997},
  {-30274,-12540}, {-32488,  4277}, {-25997, 19948}, {-12540, 30274},
  {  4277, 32488}, { 19948, 25997}, { 30274, 12540}, { 32488, -4277},
  { 25997,-19948}, { 12540,-30274}, { -4277,-32488}, {-19948,-25997},
  {-30274,-12540}, {-32488,  4277}, {-25997, 19948}, {-12540, 30274},
  {  4277, 32488}, { 19948, 25997}, { 30274, 12540}, { 32488, -4277},
  { 25997,-19948}, { 12540,-30274}, { -4277,-32488}, {-19948,-25997},
  {-30274,-12540}, {-32488,  4277}, {-25997, 19948}, {-12540, 30274},
  {  4277, 32488}, { 19948, 25997}, { 30274, 12540}, { 32488, -4277},
  { 25997,-19948}, { 12540,-30274}, { -4277,-32488}, {-19948,-25997},
  {-30274,-12540}, {-32488,  4277}, {-25997, 19948}, {-12540, 30274}
};
#elif FREQUENCY_OFFSET==3000*(AUDIO_ADC_FREQ/AUDIO_SAMPLES_COUNT/1000)
// static Table for 6kHz IF and 96kHz audio ADC (or 3kHz IF and 48kHz ADC) audio ADC
static const int16_t sincos_tbl[48][2] = {
  {  3212, 32610}, { 15447, 28899}, { 25330, 20788}, { 31357,  9512},
  { 32610, -3212}, { 28899,-15447}, { 20788,-25330}, {  9512,-31357},
  { -3212,-32610}, {-15447,-28899}, {-25330,-20788}, {-31357, -9512},
  {-32610,  3212}, {-28899, 15447}, {-20788, 25330}, { -9512, 31357},
  {  3212, 32610}, { 15447, 28899}, { 25330, 20788}, { 31357,  9512},
  { 32610, -3212}, { 28899,-15447}, { 20788,-25330}, {  9512,-31357},
  { -3212,-32610}, {-15447,-28899}, {-25330,-20788}, {-31357, -9512},
  {-32610,  3212}, {-28899, 15447}, {-20788, 25330}, { -9512, 31357},
  {  3212, 32610}, { 15447, 28899}, { 25330, 20788}, { 31357,  9512},
  { 32610, -3212}, { 28899,-15447}, { 20788,-25330}, {  9512,-31357},
  { -3212,-32610}, {-15447,-28899}, {-25330,-20788}, {-31357, -9512},
  {-32610,  3212}, {-28899, 15447}, {-20788, 25330}, { -9512, 31357}
};
#else
#error "Need check/rebuild sin cos table for DAC"
#endif

#if 1
// Define DSP accumulator value type
typedef float acc_t;
typedef float measure_t;
acc_t acc_samp_s;
acc_t acc_samp_c;
acc_t acc_ref_s;
acc_t acc_ref_c;
void
dsp_process(int16_t *capture, size_t length)
{
  int32_t samp_s = 0;
  int32_t samp_c = 0;
  int32_t ref_s = 0;
  int32_t ref_c = 0;
  uint32_t i = 0;
  do{
    int16_t ref = capture[i+0];
    int16_t smp = capture[i+1];
#ifdef ENABLED_DUMP
    ref_buf[i] = ref;
    samp_buf[i] = smp;
#endif
    int16_t sin = ((int16_t *)sincos_tbl)[i+0];
    int16_t cos = ((int16_t *)sincos_tbl)[i+1];
    samp_s+= (smp * sin)>>4;
    samp_c+= (smp * cos)>>4;
    ref_s += (ref * sin)>>4;
    ref_c += (ref * cos)>>4;
    i+=2;
  }while (i < length);
  acc_samp_s += samp_s;
  acc_samp_c += samp_c;
  acc_ref_s += ref_s;
  acc_ref_c += ref_c;
}

#else
// Define DSP accumulator value type
typedef int64_t acc_t;
typedef float measure_t;
acc_t acc_samp_s;
acc_t acc_samp_c;
acc_t acc_ref_s;
acc_t acc_ref_c;
// Cortex M4 DSP instruction use
#include "dsp.h"
void
dsp_process(int16_t *capture, size_t length)
{
  uint32_t i = 0;
//  int64_t samp_s = 0;
//  int64_t samp_c = 0;
//  int64_t ref_s = 0;
//  int64_t ref_c = 0;

  i=0;
  do{
    int32_t sc = ((int32_t *)sincos_tbl)[i];
    int32_t sr = ((int32_t *)capture)[i];
// int32_t acc DSP functions, but int32 can overflow
//    samp_s = __smlatb(sr, sc, samp_s); // samp_s+= smp * sin
//    samp_c = __smlatt(sr, sc, samp_c); // samp_c+= smp * cos
//    ref_s  = __smlabb(sr, sc, ref_s);  //  ref_s+= ref * sin
//    ref_c  = __smlabt(sr, sc, ref_c);  //  ref_s+= ref * cos
// int64_t acc DSP functions
    acc_samp_s= __smlaltb(acc_samp_s, sr, sc ); // samp_s+= smp * sin
    acc_samp_c= __smlaltt(acc_samp_c, sr, sc ); // samp_c+= smp * cos
    acc_ref_s = __smlalbb( acc_ref_s, sr, sc ); //  ref_s+= ref * sin
    acc_ref_c = __smlalbt( acc_ref_c, sr, sc ); //  ref_s+= ref * cos
    i++;
  } while (i < length/2);

// Accumulate result, for faster calc and prevent overflow reduce size to int32_t
//  acc_samp_s+= (int32_t)(samp_s>>4);
//  acc_samp_c+= (int32_t)(samp_c>>4);
//  acc_ref_s += (int32_t)( ref_s>>4);
//  acc_ref_c += (int32_t)( ref_c>>4);
}
#endif

void
calculate_gamma(float gamma[2])
{
#if 1
  // calculate reflection coeff. by samp divide by ref
#if 0
  measure_t rs = acc_ref_s;
  measure_t rc = acc_ref_c;
  measure_t rr = rs * rs + rc * rc;
  //rr = sqrtf(rr) * 1e8;
  measure_t ss = acc_samp_s;
  measure_t sc = acc_samp_c;
  gamma[0] =  (sc * rc + ss * rs) / rr;
  gamma[1] =  (ss * rc - sc * rs) / rr;
#else
  measure_t rs_rc = (measure_t) acc_ref_s / acc_ref_c;
  measure_t sc_rc = (measure_t)acc_samp_c / acc_ref_c;
  measure_t ss_rc = (measure_t)acc_samp_s / acc_ref_c;
  measure_t rr = rs_rc * rs_rc + 1.0;
  gamma[0] = (sc_rc + ss_rc*rs_rc) / rr;
  gamma[1] = (ss_rc - sc_rc*rs_rc) / rr;
#endif
#elif 0
  gamma[0] =  acc_samp_s;
  gamma[1] =  acc_samp_c;
#else
  gamma[0] =  acc_ref_s;
  gamma[1] =  acc_ref_c;
#endif
}

void
fetch_amplitude(float gamma[2])
{
  gamma[0] =  acc_samp_s * 1e-9;
  gamma[1] =  acc_samp_c * 1e-9;
}

void
fetch_amplitude_ref(float gamma[2])
{
  gamma[0] =  acc_ref_s * 1e-9;
  gamma[1] =  acc_ref_c * 1e-9;
}

void
reset_dsp_accumerator(void)
{
  acc_ref_s = 0;
  acc_ref_c = 0;
  acc_samp_s = 0;
  acc_samp_c = 0;
}
