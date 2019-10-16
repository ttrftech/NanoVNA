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

float acc_samp_s;
float acc_samp_c;
float acc_ref_s;
float acc_ref_c;

void
dsp_process(int16_t *capture, size_t length)
{
  uint32_t *p = (uint32_t*)capture;
  uint32_t len = length / 2;
  uint32_t i;
  int32_t samp_s = 0;
  int32_t samp_c = 0;
  int32_t ref_s = 0;
  int32_t ref_c = 0;

  // quadrature steps for if=12kHz on fs=48kHz
  for (i = 0; i < len;) {
    uint32_t sr;
    int16_t ref, smp;

    sr = *p++;
    ref = sr & 0xffff;
    smp = (sr>>16) & 0xffff;
#ifdef ENABLED_DUMP
    ref_buf[i] = ref;
    samp_buf[i] = smp;
#endif
    i++;
    samp_s += smp;
    ref_s += ref;

    sr = *p++;
    ref = sr & 0xffff;
    smp = (sr>>16) & 0xffff;
#ifdef ENABLED_DUMP
    ref_buf[i] = ref;
    samp_buf[i] = smp;
#endif
    i++;
    samp_c += smp;
    ref_c += ref;

    sr = *p++;
    ref = sr & 0xffff;
    smp = (sr>>16) & 0xffff;
#ifdef ENABLED_DUMP
    ref_buf[i] = ref;
    samp_buf[i] = smp;
#endif
    i++;
    samp_s -= smp;
    ref_s -= ref;

    sr = *p++;
    ref = sr & 0xffff;
    smp = (sr>>16) & 0xffff;
#ifdef ENABLED_DUMP
    ref_buf[i] = ref;
    samp_buf[i] = smp;
#endif
    i++;
    samp_c -= smp;
    ref_c -= ref;
  }
  acc_samp_s += samp_s;
  acc_samp_c += samp_c;
  acc_ref_s += ref_s;
  acc_ref_c += ref_c;
}

void
calculate_gamma(float gamma[2])
{
#if 1
  // calculate reflection coeff. by samp divide by ref
  float rs = acc_ref_s;
  float rc = acc_ref_c;
  float rr = rs * rs + rc * rc;
  //rr = sqrtf(rr) * 1e8;
  float ss = acc_samp_s;
  float sc = acc_samp_c;
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
