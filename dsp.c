#include <arm_math.h>
#include "nanovna.h"

int16_t ref_state[STATE_LEN];
int16_t ref_buf[SAMPLE_LEN];
//int16_t refq_buf[SAMPLE_LEN];
int16_t refiq_buf[AUDIO_BUFFER_LEN];

int16_t samp_buf[SAMPLE_LEN];


// Bi-Quad IIR Filter state
q15_t bq_state1[4 * 4];
q15_t bq_state2[4 * 4];

q15_t bq_coeffs[] = {
         189, 0,    -72,   189, 26371, -15931,
        1008, 0,  -1952,  1008, 25915, -15917,
        1761, 0,  -2113,  1761, 26887, -16201,
        3075, 0,  -5627,  3075, 25801, -16186,
};

arm_biquad_casd_df1_inst_q15 bq1 = { 3, bq_state1, bq_coeffs, 1};
arm_biquad_casd_df1_inst_q15 bq2 = { 3, bq_state2, bq_coeffs, 1};

const q15_t hilbert31_coeffs[] = {
  20570, 6125, 2918, 1456, 682, 279, 91, 19 
};

static void
hilbert_transform(void)
{ 
  __SIMD32_TYPE *src = __SIMD32_CONST(ref_state);
  //__SIMD32_TYPE *dst = __SIMD32_CONST(refq_buf);
  __SIMD32_TYPE *dst = __SIMD32_CONST(refiq_buf);
  int j;

  for (j = 0; j < SAMPLE_LEN / 2; j++) {
    int i;
    int32_t acc0 = 0;
    int32_t accn0 = 0;
    int32_t acc1 = 0;
    int32_t accn1 = 0;

    for (i = 0; i < 8; i += 2) {
      uint32_t c = *(uint32_t*)&hilbert31_coeffs[i];
#define OFFSET (STATE_LEN / 2 / 2)
      __SIMD32_TYPE a0 = src[OFFSET - i-1];
      __SIMD32_TYPE a1 = src[OFFSET - i-2];
      __SIMD32_TYPE b0 = src[OFFSET + i];
      __SIMD32_TYPE b1 = src[OFFSET + i+1];

      __SIMD32_TYPE a = __PKHTB(a1, a0, 16);
      __SIMD32_TYPE b = __PKHTB(b1, b0, 16);
      acc0 = __SMLAD(c, b, acc0);
      accn0 = __SMLAD(c, a, accn0);
      a = __PKHBT(a0, a1, 16);
      b = __PKHBT(b0, b1, 16);
      acc1 = __SMLAD(c, b, acc1);
      accn1 = __SMLAD(c, a, accn1);
    }
    acc0 -= accn0;
    acc1 -= accn1;
    //*dst++ = __PKHTB(acc0, acc1, 16);
    *dst++ = __PKHTB(acc1<<1, src[OFFSET-1], 16);
    *dst++ = __PKHTB(acc0<<1, src[OFFSET], 0);
    src++;
  }

  dst = __SIMD32_CONST(ref_state);
  for (j = 0; j < STATE_LEN / 2; j++) {
    *dst++ = *src++;
  }
}

void calclate_gamma(float *gamma)
#if 0
{
  __SIMD32_TYPE *r = __SIMD32_CONST(refiq_buf);
  __SIMD32_TYPE *s = __SIMD32_CONST(samp_buf);
  q31_t acc_r = 0;
  q31_t acc_i = 0;
  q31_t acc_ref = 0;
  int i;

  for (i = 0; i < SAMPLE_LEN/2; i++) {
    __SIMD32_TYPE s0 = *s++;
    __SIMD32_TYPE r0 = *r++;
    __SIMD32_TYPE r1 = *r++;
    __SIMD32_TYPE rr = __PKHBT(r1, r0, 16);
    __SIMD32_TYPE ri = __PKHTB(r0, r1, 16);
    acc_r = __SMLAD(rr, s0, acc_r);
    acc_i = __SMLAD(ri, s0, acc_i);
    acc_ref = __SMLAD(r0, r0, acc_ref);
    acc_ref = __SMLAD(r1, r1, acc_ref);
  }
  //acc_ref = sqrt(acc_ref / SAMPLE_LEN) / 65536;
  gamma_real = acc_r / 65536;
  gamma_imag = acc_i / 65536;
}
#else
{
  int16_t *r = refiq_buf;
  int16_t *s = samp_buf;
  int len = SAMPLE_LEN;
  float acc_r = 0;
  float acc_i = 0;
  float acc_ref = 0;
  int i;
  float rn;

  for (i = 0; i < len; i++) {
    int16_t s0 = *s++;
    int16_t rr = *r++;
    int16_t ri = *r++;
    acc_r += (float)(s0 * rr);
    acc_i += (float)(s0 * ri);
    acc_ref += (float)rr*rr + (float)ri*ri;
  }
  rn = sqrtf(acc_ref / len);
  gamma[0] = 16 * acc_r / rn / len;
  gamma[1] = 16 * acc_i / rn / len;
}
#endif

void
dsp_process(int16_t *capture, size_t length)
{
  uint32_t *p = (uint32_t*)capture;
  uint32_t len = length / 2;
  uint32_t i;
  for (i = 0; i < len; i++) {
    uint32_t sr = *p++;
    ref_buf[i] = sr & 0xffff;
    samp_buf[i] = (sr>>16) & 0xffff;
  }

  // apply low pass filter
  //arm_biquad_cascade_df1_q15(&bq1, ref_buf, ref_buf, len);
  //arm_biquad_cascade_df1_q15(&bq2, samp_buf, samp_buf, len);

  hilbert_transform();
}

