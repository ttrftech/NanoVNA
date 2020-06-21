/*
 * Copyright (c) 2014-2015, TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
 * Modified by DiSlord dislordlive@gmail.com
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
#include "hal.h"
#include "nanovna.h"
#include "si5351.h"

// XTAL frequency on si5351
#define XTALFREQ 26000000U
// audio codec frequency clock
#define CLK2_FREQUENCY AUDIO_CLOCK_REF

// Fixed PLL mode multiplier (used in band 1 for frequency 800-10k)
#define PLL_N_1  8
// Fixed PLL mode multiplier (used in band 2 for frequency 10k-100M)
#define PLL_N_2 32

// I2C address on bus (only 0x60 for Si5351A in 10-Pin MSOP)
#define SI5351_I2C_ADDR     0x60

static uint8_t  current_band   = 0;
static uint32_t current_freq   = 0;
static int32_t  current_offset = FREQUENCY_OFFSET;
// Use cache for this reg, not update if not change
static uint8_t  clk_cache[3] = {0, 0, 0};

#if 1
// Minimum value is 2, freq change apply at next dsp measure, and need skip it
#define DELAY_NORMAL          2
// Delay for bands (depend set band 1 more fast (can change before next dsp buffer ready, need wait additional interval)
#define DELAY_BAND_1_2        2
#define DELAY_BAND_3_4        2
// Band changes need set additional delay after reset PLL
#define DELAY_BANDCHANGE_1_2  3
#define DELAY_BANDCHANGE_3_4  4
// Delay after set new PLL values, and send reset (on band 1 unstable if less then 900, on 2000-5000 no amplitude spike on change)
#define DELAY_RESET_PLL_BEFORE    1000
#define DELAY_RESET_PLL_AFTER     3500

#else
// Debug timer set
uint16_t timings[8]={2,2,2,3,4,1000, 3500};
void si5351_set_timing(int i, int v) {timings[i]=v;}
#define DELAY_NORMAL          timings[0]
// Delay for bands (depend set band 1 more fast (can change before next dsp buffer ready, need wait additional interval)
#define DELAY_BAND_1_2        timings[1]
#define DELAY_BAND_3_4        timings[2]
// Band changes need set additional delay after reset PLL
#define DELAY_BANDCHANGE_1_2  timings[3]
#define DELAY_BANDCHANGE_3_4  timings[4]
// Delay after set new PLL values, and send reset (on band 1-2 unstable if less then 900, on 2000-5000 no amplitude spike on change)
#define DELAY_RESET_PLL_BEFORE      timings[5]
#define DELAY_RESET_PLL_AFTER       timings[6]
#endif

uint32_t si5351_get_frequency(void)
{
  return current_freq;
}

void si5351_set_frequency_offset(int32_t offset)
{
  current_offset = offset;
  current_freq = 0; // reset freq, for
}

static void
si5351_bulk_write(const uint8_t *buf, int len)
{
//  i2cAcquireBus(&I2CD1);
  (void)i2cMasterTransmitTimeout(&I2CD1, SI5351_I2C_ADDR, buf, len, NULL, 0, 1000);
//  i2cReleaseBus(&I2CD1);
}

#if 0
static bool si5351_bulk_read(uint8_t reg, uint8_t* buf, int len)
{
  i2cAcquireBus(&I2CD1);
  msg_t mr = i2cMasterTransmitTimeout(&I2CD1, SI5351_I2C_ADDR, &reg, 1, buf, len, 1000);
  i2cReleaseBus(&I2CD1);
  return mr == MSG_OK;
}

static void si5351_wait_pll_lock(void)
{
  uint8_t status;
  int count = 100;
  do{
    status=0xFF;
    si5351_bulk_read(0, &status, 1);
    if ((status & 0x60) == 0) // PLLA and PLLB locked
      return;
  }while (--count);
}
#endif

static inline void
si5351_write(uint8_t reg, uint8_t dat)
{
  uint8_t buf[] = { reg, dat };
  si5351_bulk_write(buf, 2);
}

// register addr, length, data, ...
const uint8_t si5351_configs[] = {
  2, SI5351_REG_3_OUTPUT_ENABLE_CONTROL, 0xff,
  4, SI5351_REG_16_CLK0_CONTROL, SI5351_CLK_POWERDOWN, SI5351_CLK_POWERDOWN, SI5351_CLK_POWERDOWN,
  2, SI5351_REG_183_CRYSTAL_LOAD, SI5351_CRYSTAL_LOAD_8PF,
// All of this init code run late on sweep
#if 0
  // setup PLL (26MHz * 32 = 832MHz, 32/2-2=14)
  9, SI5351_REG_PLL_A, /*P3*/0, 1, /*P1*/0, 14, 0, /*P3/P2*/0, 0, 0,
  9, SI5351_REG_PLL_B, /*P3*/0, 1, /*P1*/0, 14, 0, /*P3/P2*/0, 0, 0,
  // RESET PLL
  2, SI5351_REG_177_PLL_RESET, SI5351_PLL_RESET_A | SI5351_PLL_RESET_B | 0x0C, //
  // setup multisynth (832MHz / 104 = 8MHz, 104/2-2=50)
  9, SI5351_REG_58_MULTISYNTH2, /*P3*/0, 1, /*P1*/0, 50, 0, /*P2|P3*/0, 0, 0,
  2, SI5351_REG_18_CLK2_CONTROL, SI5351_CLK_DRIVE_STRENGTH_2MA | SI5351_CLK_INPUT_MULTISYNTH_N | SI5351_CLK_INTEGER_MODE,
#endif
  2, SI5351_REG_3_OUTPUT_ENABLE_CONTROL, ~(SI5351_CLK0_EN|SI5351_CLK1_EN|SI5351_CLK2_EN),
  0 // sentinel
};

void
si5351_init(void)
{
  const uint8_t *p = si5351_configs;
  while (*p) {
    uint8_t len = *p++;
    si5351_bulk_write(p, len);
    p += len;
  }
}

static const uint8_t disable_output[] = {
  SI5351_REG_16_CLK0_CONTROL,
  SI5351_CLK_POWERDOWN,  // CLK 0
  SI5351_CLK_POWERDOWN,  // CLK 1
  SI5351_CLK_POWERDOWN   // CLK 2
};

/* Get the appropriate starting point for the PLL registers */
static const uint8_t msreg_base[] = {
  SI5351_REG_42_MULTISYNTH0,
  SI5351_REG_50_MULTISYNTH1,
  SI5351_REG_58_MULTISYNTH2,
};

// Reset PLL need then band changes
static void si5351_reset_pll(uint8_t mask)
{
  // Writing a 1<<5 will reset PLLA, 1<<7 reset PLLB, this is a self clearing bits.
  si5351_write(SI5351_REG_177_PLL_RESET, mask | 0x0C);
}

void si5351_disable_output(void)
{
  si5351_write(SI5351_REG_3_OUTPUT_ENABLE_CONTROL, SI5351_CLK0_EN|SI5351_CLK1_EN|SI5351_CLK2_EN);
  si5351_bulk_write(disable_output, sizeof(disable_output));
  current_band = 0;
}

void si5351_enable_output(void)
{
  si5351_write(SI5351_REG_3_OUTPUT_ENABLE_CONTROL, ~(SI5351_CLK0_EN|SI5351_CLK1_EN|SI5351_CLK2_EN));
//si5351_reset_pll(SI5351_PLL_RESET_A | SI5351_PLL_RESET_B);
  current_freq = 0;
  current_band = 0;
}

// Set PLL freq = XTALFREQ * (mult + num/denom)
static void si5351_setupPLL(uint8_t   pllSource,  /* SI5351_REG_PLL_A or SI5351_REG_PLL_B */
                            uint32_t  mult,
                            uint32_t  num,
                            uint32_t  denom)
{
  /* Feedback Multisynth Divider Equation
   * where: a = mult, b = num and c = denom
   * P1 register is an 18-bit value using following formula:
   *    P1[17:0] = 128 * mult + int((128*num)/denom) - 512
   * P2 register is a 20-bit value using the following formula:
   *    P2[19:0] = (128 * num) % denom
   * P3 register is a 20-bit value using the following formula:
   *    P3[19:0] = denom
   */
  /* Set the main PLL config registers */
  mult <<= 7;
  num <<= 7;
  uint32_t P1 = mult - 512;  // Integer mode
  uint32_t P2 = 0;
  uint32_t P3 = 1;
  if (num) {                 // Fractional mode
    P1+= num / denom;
    P2 = num % denom;
    P3 = denom;
  }
  // Pll MSN(A|B) registers Datasheet
  uint8_t reg[9];
  reg[0] = pllSource;                                       // SI5351_REG_PLL_A or SI5351_REG_PLL_B
  reg[1] = (P3 & 0x0FF00) >> 8;                             // MSN_P3[15: 8]
  reg[2] = (P3 & 0x000FF);                                  // MSN_P3[ 7: 0]
  reg[3] = (P1 & 0x30000) >> 16;                            // MSN_P1[17:16]
  reg[4] = (P1 & 0x0FF00) >> 8;                             // MSN_P1[15: 8]
  reg[5] = (P1 & 0x000FF);                                  // MSN_P1[ 7: 0]
  reg[6] = ((P3 & 0xF0000) >> 12) | ((P2 & 0xF0000) >> 16); // MSN_P3[19:16] | MSN_P2[19:16]
  reg[7] = (P2 & 0x0FF00) >> 8;                             // MSN_P2[15: 8]
  reg[8] = (P2 & 0x000FF);                                  // MSN_P2[ 7: 0]
  si5351_bulk_write(reg, 9);
}

// Set Multisynth divider = (div + num/denom) * rdiv
static void
si5351_setupMultisynth(uint8_t   channel,
                       uint32_t  div,    // 4,6,8, 8+ ~ 900
                       uint32_t  num,
                       uint32_t  denom,
                       uint32_t  rdiv,   // SI5351_R_DIV_1~128
                       uint8_t   chctrl) // SI5351_REG_16_CLKX_CONTROL settings
{
  /* Output Multisynth Divider Equations
   * where: a = div, b = num and c = denom
   * P1 register is an 18-bit value using following formula:
   *   P1[17:0] = 128 * a + int((128*b)/c) - 512
   * P2 register is a 20-bit value using the following formula:
   *   P2[19:0] = (128 * b) % c
   * P3 register is a 20-bit value using the following formula:
   *   P3[19:0] = c
   */
  /* Set the main PLL config registers */
  uint32_t P1 = 0;
  uint32_t P2 = 0;
  uint32_t P3 = 1;
  if (div == 4)
    rdiv|= SI5351_DIVBY4;
  else {
    num<<=7;
    div<<=7;
    P1 = div - 512; // Integer mode
    if (num) {       // Fractional mode
      P1+= num / denom;
      P2 = num % denom;
      P3 = denom;
    }
  }
  /* Set the MSx config registers */
  uint8_t reg[9];
  reg[0] = msreg_base[channel];                       // SI5351_REG_42_MULTISYNTH0, SI5351_REG_50_MULTISYNTH1, SI5351_REG_58_MULTISYNTH2
  reg[1] = (P3 & 0x0FF00)>>8;                         // MSx_P3[15: 8]
  reg[2] = (P3 & 0x000FF);                            // MSx_P3[ 7: 0]
  reg[3] = ((P1 & 0x30000)>>16)| rdiv;                // Rx_DIV[2:0] | MSx_DIVBY4[1:0] | MSx_P1[17:16]
  reg[4] = (P1 & 0x0FF00)>> 8;                        // MSx_P1[15: 8]
  reg[5] = (P1 & 0x000FF);                            // MSx_P1[ 7: 0]
  reg[6] = ((P3 & 0xF0000)>>12)|((P2 & 0xF0000)>>16); // MSx_P3[19:16] | MSx_P2[19:16]
  reg[7] = (P2 & 0x0FF00)>>8;                         // MSx_P2[15: 8]
  reg[8] = (P2 & 0x000FF);                            // MSx_P2[ 7: 0]
  si5351_bulk_write(reg, 9);

  /* Configure the clk control and enable the output */
  uint8_t dat = chctrl | SI5351_CLK_INPUT_MULTISYNTH_N;
  if (num == 0)
    dat |= SI5351_CLK_INTEGER_MODE;
  if (clk_cache[channel]!=dat) {
    si5351_write(SI5351_REG_16_CLK0_CONTROL+channel, dat);
    clk_cache[channel]=dat;
  }
}

// Find better approximate values for n/d
#define MAX_DENOMINATOR ((1 << 20) - 1)
static inline void approximate_fraction(uint32_t *n, uint32_t *d)
{
  // cf. https://github.com/python/cpython/blob/master/Lib/fractions.py#L227
  uint32_t denom = *d;
  if (denom > MAX_DENOMINATOR) {
    uint32_t num = *n;
    uint32_t p0 = 0, q0 = 1, p1 = 1, q1 = 0;
    while (denom != 0) {
      uint32_t a = num / denom;
      uint32_t b = num % denom;
      uint32_t q2 = q0 + a*q1;
      if (q2 > MAX_DENOMINATOR)
        break;
      uint32_t p2 = p0 + a*p1;
      p0 = p1; q0 = q1; p1 = p2; q1 = q2;
      num = denom; denom = b;
    }
    *n = p1;
    *d = q1;
  }
}

// Setup Multisynth divider for get correct output freq if fixed PLL = pllfreq
static void
si5351_set_frequency_fixedpll(uint8_t channel, uint64_t pllfreq, uint32_t freq, uint32_t rdiv, uint8_t chctrl)
{
  uint32_t denom = freq;
  uint32_t div = pllfreq / denom; // range: 8 ~ 1800
  uint32_t num = pllfreq % denom;
  approximate_fraction(&num, &denom);
  si5351_setupMultisynth(channel, div, num, denom, rdiv, chctrl);
}

// Setup PLL freq if Multisynth divider fixed = div (need get output =  freq/mul)
static void
si5351_setupPLL_freq(uint32_t pllSource, uint32_t freq, uint32_t div, uint32_t mul)
{
  uint32_t denom = XTALFREQ * mul;
  uint64_t pllfreq = (uint64_t)freq * div;
  uint32_t multi = pllfreq / denom;
  uint32_t num   = pllfreq % denom;
  approximate_fraction(&num, &denom);
  si5351_setupPLL(pllSource, multi, num, denom);
}

#if 0
static void
si5351_set_frequency_fixeddiv(uint8_t channel, uint32_t pll, uint32_t freq, uint32_t div,
                              uint8_t chctrl, uint32_t mul)
{
  si5351_setupPLL_freq(pll, freq, div, mul);
  si5351_setupMultisynth(channel, div, 0, 1, SI5351_R_DIV_1, chctrl);
}

void
si5351_set_frequency(int channel, uint32_t freq, uint8_t drive_strength)
{
  if (freq <= 100000000) {
    si5351_setupPLL(SI5351_PLL_B, 32, 0, 1);
    si5351_set_frequency_fixedpll(channel, SI5351_PLL_B, PLLFREQ, freq, SI5351_R_DIV_1, drive_strength, 1);
  } else if (freq < 150000000) {
    si5351_set_frequency_fixeddiv(channel, SI5351_PLL_B, freq, 6, drive_strength, 1);
  } else {
    si5351_set_frequency_fixeddiv(channel, SI5351_PLL_B, freq, 4, drive_strength, 1);
  }
}
#endif

/*
 * Frequency generation divide on band
 *  Band 1
 * 800~10kHz       fixed PLL = XTALFREQ * PLL_N_1, fractional divider
 *  Band 2
 * 10kHz~100MHz    fixed PLL = XTALFREQ * PLL_N_2, fractional divider
 *  Band 3
 * 100~130MHz fractional PLL = 800-1040MHz, fixed divider 'fdiv = 8'
 *  Band 4
 * 130~170MHz fractional PLL = 780-1080MHz, fixed divider 'fdiv = 6'
 *  Band 5
 * 680~300MHz fractional PLL = 680-1200MHz, fixed divider 'fdiv = 4'
 *
 * For FREQ_HARMONICS = 300MHz - band range is:
 *  +-----------------------------------------------------------------------------------------------------------------------------------------------+
 *  |    Band 2     |   Band 3     |    Band 4    |    Band 5    |   Band 3   |   Band 4   |                       Band 5                           |
 *  +-----------------------------------------------------------------------------------------------------------------------------------------------+
 *  |           Direct mode  x1 :  x1                            |          x3 : x5                     |   x5-x7     |    x7-x9     |    x9-x11    |
 *  +-----------------------------------------------------------------------------------------------------------------------------------------------+
 *  |10kHz - 100MHz | 100 - 130MHz | 130 - 170MHz | 170 - 300MHz | 300-390MHz | 390-510MHz | 510-900MHz | 900-1500MHz | 1500-2100MHz | 2100-2700MHz |
 *  +-----------------------------------------------------------------------------------------------------------------------------------------------+
 *  |             f = 50kHz-300MHz                               | f=100-130  | f=130-170  | f=170-300  | f=180-300   | f=214-300    | f=233-300    |
 *  |            of = 50kHz-300MHz                               |of= 60- 78  |of= 78-102  |of=102-180  |of=128-215   |of=166-234    |of=190-246    |
 *  +-----------------------------------------------------------------------------------------------------------------------------------------------+
 */
static inline uint8_t
si5351_get_band(uint32_t freq)
{
// not correct use like this, freq multiplied before if freq < 4MHz
//  if (freq <   10000U) return 1;
  if (freq < 100000000U) return 2;
  if (freq < 130000000U) return 3;
  if (freq < 170000000U) return 4;
  return 5;
}

#define MAX_HARMONIC 5
uint32_t
si5351_get_harmonic_lvl(uint32_t f){
  uint32_t h = config.harmonic_freq_threshold;
  if (f < h) return 0;
  f-=h;
  h<<=1;
  uint32_t lvl = 1 + f/h;
  return lvl < MAX_HARMONIC ? lvl : (MAX_HARMONIC-1);
}

static const uint8_t h_mult[][2] ={
  {1, 1}, // f < threshold (           f <  300MHz)
  {3, 5}, // f < threshold ( 300MHz <= f <  900MHz)
  {5, 7}, // f < threshold ( 900MHz <= f < 1500MHz)
  {7, 9}, // f < threshold (1500MHz <= f < 2100MHz)
  {9,11}  // f < threshold (2100MHz <= f < 2700MHz)
};

/*
 * Maximum supported frequency = FREQ_HARMONICS * 9U
 * configure output as follows:
 * CLK0: frequency + offset
 * CLK1: frequency
 * CLK2: fixed 8MHz
 */
int
si5351_set_frequency(uint32_t freq, uint8_t drive_strength)
{
  uint8_t band;
  if (freq == current_freq)
    return DELAY_NORMAL;

  int delay;
  uint32_t ofreq = freq + current_offset;

  uint32_t rdiv = SI5351_R_DIV_1;
  uint32_t fdiv, pll_n;
  // Fix possible incorrect input
  drive_strength&=SI5351_CLK_DRIVE_STRENGTH_MASK;

  // Harmonic mode prepare
#if 1
  uint32_t harmonic = si5351_get_harmonic_lvl(freq);
  uint32_t mul  = h_mult[harmonic][0];
  uint32_t omul = h_mult[harmonic][1];
#else
  uint32_t mul = 1, omul = 1;
  if (freq >= config.harmonic_freq_threshold * 7U) {
     mul =  9;
    omul = 11;
  } else if (freq >= config.harmonic_freq_threshold * 5U) {
     mul = 7;
    omul = 9;
  } else if (freq >= config.harmonic_freq_threshold * 3U) {
     mul = 5;
    omul = 7;
  } else if (freq >= config.harmonic_freq_threshold) {
     mul = 3;
    omul = 5;
  }
#endif
  // Select optimal band for prepared freq
  if (freq <  10000U) {
     rdiv = SI5351_R_DIV_128;
     freq<<= 7;
    ofreq<<= 7;
    band = 1;
  } else if (freq <= 500000U) {
    rdiv = SI5351_R_DIV_64;
     freq<<= 6;
    ofreq<<= 6;
    band = 2;
  } else if (freq <= 4000000U) {
    rdiv = SI5351_R_DIV_8;
     freq<<= 3;
    ofreq<<= 3;
    band = 2;
  }
  else
    band = si5351_get_band(freq / mul);
  if (current_band != band) {
    si5351_reset_pll(SI5351_PLL_RESET_A | SI5351_PLL_RESET_B);
    // Possibly not need add delay now
    chThdSleepMicroseconds(DELAY_RESET_PLL_BEFORE);
  }
  static const uint8_t band_setting[] = {1, PLL_N_1, PLL_N_2, 8, 6, 4};
  switch (band) {
    case 1: // 800Hz to 10kHz   PLLN =  8
    case 2: // 10kHz to 100MHz  PLLN = 32
      pll_n = band_setting[band];
      // Setup CH0 and CH1 constant PLLA freq at band change, and set CH2 freq = CLK2_FREQUENCY
      if (current_band != band) {
        si5351_setupPLL(SI5351_REG_PLL_A,   pll_n, 0, 1);
        si5351_setupPLL(SI5351_REG_PLL_B, PLL_N_2, 0, 1);
        si5351_set_frequency_fixedpll(
            2, XTALFREQ * PLL_N_2, CLK2_FREQUENCY, SI5351_R_DIV_1,
            SI5351_CLK_DRIVE_STRENGTH_2MA | SI5351_CLK_PLL_SELECT_B);
        delay = DELAY_BANDCHANGE_1_2;
      } else {
        delay = DELAY_BAND_1_2;
      }
      // Calculate and set CH0 and CH1 divider
      si5351_set_frequency_fixedpll(0, (uint64_t)omul * XTALFREQ * pll_n, ofreq, rdiv,
                                    drive_strength | SI5351_CLK_PLL_SELECT_A);
      si5351_set_frequency_fixedpll(1, (uint64_t)mul * XTALFREQ * pll_n, freq, rdiv,
                                    drive_strength | SI5351_CLK_PLL_SELECT_A);
      break;
    case 3:  // fdiv = 8, f 100-130   PLL 800-1040
    case 4:  // fdiv = 6, f 130-170   PLL 780-1050
    case 5:  // fdiv = 4, f 170-300   PLL 680-1200
      fdiv = band_setting[band];

      // Setup CH0 and CH1 constant fdiv divider at change
      if (current_band != band) {
        si5351_setupMultisynth(0, fdiv, 0, 1, SI5351_R_DIV_1,
                               drive_strength | SI5351_CLK_PLL_SELECT_A);
        si5351_setupMultisynth(1, fdiv, 0, 1, SI5351_R_DIV_1,
                               drive_strength | SI5351_CLK_PLL_SELECT_B);
        delay= DELAY_BANDCHANGE_3_4;
      } else {
       delay= DELAY_BAND_3_4;
      }
      // Calculate and set CH0 and CH1 PLL freq
      si5351_setupPLL_freq(SI5351_REG_PLL_A, ofreq, fdiv,
                           omul);  // set PLLA freq = (ofreq/omul)*fdiv
      si5351_setupPLL_freq(SI5351_REG_PLL_B, freq, fdiv,
                           mul);  // set PLLB freq = ( freq/ mul)*fdiv
      // Calculate CH2 freq = CLK2_FREQUENCY, depend from calculated before CH1 PLLB = (freq/mul)*fdiv
      si5351_set_frequency_fixedpll(
          2, (uint64_t)freq * fdiv, CLK2_FREQUENCY * mul, SI5351_R_DIV_1,
          SI5351_CLK_DRIVE_STRENGTH_2MA | SI5351_CLK_PLL_SELECT_B);
      break;
  }

  if (current_band != band) {
    // Possibly not need add delay now
    chThdSleepMicroseconds(DELAY_RESET_PLL_AFTER);
    si5351_reset_pll(SI5351_PLL_RESET_A|SI5351_PLL_RESET_B);
    current_band = band;
  }
  current_freq = freq;
  return delay;
}
