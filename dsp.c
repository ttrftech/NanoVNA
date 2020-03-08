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

const int16_t sincos_tbl[48][2] = {
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

int64_t acc_samp_s;
int64_t acc_samp_c;
int64_t acc_ref_s;
int64_t acc_ref_c;

void
dsp_process(int16_t *capture, size_t length)
{
  uint32_t i;
#if 1
  acc_samp_s = 0;
  acc_samp_c = 0;
  acc_ref_s = 0;
  acc_ref_c = 0;
  for (i = 0; i < length; i+=2) {
    int32_t ref = capture[i+0];
    int32_t smp = capture[i+1];
#ifdef ENABLED_DUMP
    ref_buf[i] = ref;
    samp_buf[i] = smp;
#endif
    int32_t s = ((int16_t *)sincos_tbl)[i+0];
    int32_t c = ((int16_t *)sincos_tbl)[i+1];
    acc_samp_s += (smp * s);
    acc_samp_c += (smp * c);
    acc_ref_s  += (ref * s);
    acc_ref_c  += (ref * c);
  }
#else
  uint32_t len = length / 2;
  int64_t samp_s = 0;
  int64_t samp_c = 0;
  int64_t ref_s = 0;
  int64_t ref_c = 0;
  //        HI  LO
  int32_t *cos_sin = (int32_t *)sincos_tbl;
  int32_t *ref_smp = (int32_t *)capture;
  for (i = 0; i < len; i++) {
    //
    samp_s = __SMLALBB(*ref_smp, *cos_sin, samp_s); // samp_s+= smp * sin
    samp_c = __SMLALBT(*ref_smp, *cos_sin, samp_c); // samp_c+= smp * cos
    ref_s  = __SMLALTB(*ref_smp, *cos_sin, ref_s);  // ref_s += ref * sin
    ref_c  = __SMLALTT(*ref_smp, *cos_sin, ref_c);  // ref_s += ref * cos
    ref_smp++;
    cos_sin++;
  }
  acc_samp_s = samp_s;
  acc_samp_c = samp_c;
  acc_ref_s  = ref_s;
  acc_ref_c  = ref_c;
#endif
}

void
calculate_gamma(float gamma[2])
{
#if 1
  // calculate reflection coeff. by samp divide by ref
  double rs = acc_ref_s;
  double rc = acc_ref_c;
  double rr = rs * rs + rc * rc;
  //rr = sqrtf(rr) * 1e8;
  double ss = acc_samp_s;
  double sc = acc_samp_c;
  gamma[0] =  (sc * rc + ss * rs) / rr;
  gamma[1] =  (ss * rc - sc * rs) / rr;
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
