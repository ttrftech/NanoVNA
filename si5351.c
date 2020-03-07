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

// Enable cache for SI5351 CLKX_CONTROL register, little speedup exchange
#define USE_CLK_CONTROL_CACHE        TRUE

// XTAL frequency on si5351
#define XTALFREQ 26000000U
// MCLK (processor clock if set, audio codec) frequency clock
#define CLK2_FREQUENCY 8000000U

// Fixed PLL mode multiplier (used in band 1)
#define PLL_N 32

// I2C address on bus (only 0x60 for Si5351A in 10-Pin MSOP)
#define SI5351_I2C_ADDR   	0x60

static uint8_t  current_band = 0;
static uint32_t current_freq = 0;

static void
si5351_bulk_write(const uint8_t *buf, int len)
{
  i2cAcquireBus(&I2CD1);
  (void)i2cMasterTransmitTimeout(&I2CD1, SI5351_I2C_ADDR, buf, len, NULL, 0, 1000);
  i2cReleaseBus(&I2CD1);
}
#if 0
static void si5351_bulk_read(uint8_t reg, uint8_t* buf, int len)
{
  int addr = SI5351_I2C_ADDR>>1;
  i2cAcquireBus(&I2CD1);
  msg_t mr = i2cMasterTransmitTimeout(&I2CD1, addr, &reg, 1, buf, len, 1000);
  i2cReleaseBus(&I2CD1);
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
static const uint8_t clkctrl[] = {
  SI5351_REG_16_CLK0_CONTROL,
  SI5351_REG_17_CLK1_CONTROL,
  SI5351_REG_18_CLK2_CONTROL
};

// Reset PLL need then band changes
static void si5351_reset_pll(uint8_t mask)
{
  // Writing a 1<<5 will reset PLLA, 1<<7 reset PLLB, this is a self clearing bits.
  // !!! Need delay before reset PLL for apply PLL freq changes before
  chThdSleepMicroseconds(400);
  si5351_write(SI5351_REG_177_PLL_RESET, mask | 0x0C);
}

void si5351_disable_output(void)
{
  si5351_write(SI5351_REG_3_OUTPUT_ENABLE_CONTROL, 0xFF);
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
   * 	P1[17:0] = 128 * mult + int((128*num)/denom) - 512
   * P2 register is a 20-bit value using the following formula:
   * 	P2[19:0] = (128 * num) % denom
   * P3 register is a 20-bit value using the following formula:
   * 	P3[19:0] = denom
   */
  /* Set the main PLL config registers */
  mult<<=7;
  num<<=7;
  uint32_t P1 = mult - 512; // Integer mode
  uint32_t P2 = 0;
  uint32_t P3 = 1;
  if (num){                 // Fractional mode
    P1+= num / denom;
    P2 = num % denom;
    P3 = denom;
  }
  // Pll MSN(A|B) registers Datasheet
  uint8_t reg[9];
  reg[0]= pllSource;                                // SI5351_REG_PLL_A or SI5351_REG_PLL_B
  reg[1]=( P3 & 0x0FF00)>> 8;                       // MSN_P3[15: 8]
  reg[2]=( P3 & 0x000FF);                           // MSN_P3[ 7: 0]
  reg[3]=( P1 & 0x30000)>>16;                       // MSN_P1[17:16]
  reg[4]=( P1 & 0x0FF00)>> 8;                       // MSN_P1[15: 8]
  reg[5]=( P1 & 0x000FF);                           // MSN_P1[ 7: 0]
  reg[6]=((P3 & 0xF0000)>>12)|((P2 & 0xF0000)>>16); // MSN_P3[19:16] | MSN_P2[19:16]
  reg[7]=( P2 & 0x0FF00)>> 8;                       // MSN_P2[15: 8]
  reg[8]=( P2 & 0x000FF);                           // MSN_P2[ 7: 0]
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
    if (num){       // Fractional mode
      P1+= num / denom;
      P2 = num % denom;
      P3 = denom;
    }
  }
  /* Set the MSx config registers */
  uint8_t reg[9];
  reg[0]= msreg_base[channel];                      // SI5351_REG_42_MULTISYNTH0, SI5351_REG_50_MULTISYNTH1, SI5351_REG_58_MULTISYNTH2
  reg[1]=( P3 & 0x0FF00)>>8;                        // MSx_P3[15: 8]
  reg[2]=( P3 & 0x000FF);                           // MSx_P3[ 7: 0]
  reg[3]=((P1 & 0x30000)>>16)| rdiv;                // Rx_DIV[2:0] | MSx_DIVBY4[1:0] | MSx_P1[17:16]
  reg[4]=( P1 & 0x0FF00)>> 8;                       // MSx_P1[15: 8]
  reg[5]=( P1 & 0x000FF);                           // MSx_P1[ 7: 0]
  reg[6]=((P3 & 0xF0000)>>12)|((P2 & 0xF0000)>>16); // MSx_P3[19:16] | MSx_P2[19:16]
  reg[7]=( P2 & 0x0FF00)>>8;                        // MSx_P2[15: 8]
  reg[8]=( P2 & 0x000FF);                           // MSx_P2[ 7: 0]
  si5351_bulk_write(reg, 9);

  /* Configure the clk control and enable the output */
  uint8_t dat = chctrl | SI5351_CLK_INPUT_MULTISYNTH_N;
  if (num == 0)
    dat |= SI5351_CLK_INTEGER_MODE;

#if USE_CLK_CONTROL_CACHE == TRUE
  // Use cache for this reg, not update if not change
  static uint8_t clk_cache[3];
  if (clk_cache[channel]!=dat){
    si5351_write(clkctrl[channel], dat);
    clk_cache[channel]=dat;
  }
#else
  si5351_write(clkctrl[channel], dat);
#endif
}

// Find better approximate values for n/d
#define MAX_DENOMINATOR ((1 << 20) - 1)
static inline void fractionalSolve(uint32_t *n, uint32_t *d){
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
  fractionalSolve(&num, &denom);
  si5351_setupMultisynth(channel, div, num, denom, rdiv, chctrl);
}

// Setup PLL freq if Multisynth divider fixed = div (need get output =  freq/mul)
static void
si5351_setupPLL_freq(uint32_t pllSource, uint32_t freq, uint32_t div, uint32_t mul){
  uint32_t denom = XTALFREQ * mul;
  uint64_t pllfreq = (uint64_t)freq * div;
  uint32_t multi = pllfreq / denom;
  uint32_t num   = pllfreq % denom;
  fractionalSolve(&num, &denom);
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
si5351_set_frequency(int channel, uint32_t freq, uint8_t drive_strength){
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
 * Frequency generation divide on 3 band
 *  Band 1
 *   1~100MHz      fixed PLL = XTALFREQ * PLL_N, fractional divider
 *  Band 2
 * 100~150MHz fractional PLL = 600- 900MHz, fixed divider 'fdiv = 6'
 *  Band 3
 * 150~300MHz fractional PLL = 600-1200MHz, fixed divider 'fdiv = 4'
 *
 * For FREQ_HARMONICS = 300MHz - band range is:
 *  +-----------------------------------------------------------------------------------------------------------------------+
 *  |     Band 1      |   Band 2     |    Band 3    |   Band 2     |                       Band 3                           |
 *  +-----------------------------------------------------------------------------------------------------------------------+
 *  |    x1 :  x1     |    x1 : x1   |    x1 : x1   |    x3 : x5   |   x3 : x5  |    x5-x7    |    x7-x9     |    x9-x11    |
 *  | 50kHz - 100MHz  | 100 - 150MHz | 150 - 300MHz |  300-450MHz  | 450-900MHz | 900-1500MHz | 1500-2100MHz | 2100-2700MHz |
 *  +-----------------------------------------------------------------------------------------------------------------------+
 *  |              f = 50kHz-100MHz                 | f=100-150    | f=150-300  | f=150-300   | f=214-300    | f=233-300    |
 *  |             of = 50kHz-100MHz                 |of= 60- 90    |of= 90-180  |of=128-215   |of=166-234    |of=190-246    |
 *  +-----------------------------------------------------------------------------------------------------------------------+
 */
static inline uint8_t si5351_getBand(uint32_t freq){
  if (freq < 100000000U) return 1;
  if (freq < 150000000U) return 2;
  return 3;
}

// Minimum value is 2, freq change apply at next dsp measure, and need skip it
#define DELAY_NORMAL 2
// Additional delay for band 1 (remove unstable generation at begin)
#define DELAY_BAND_1	 1
// Band changes need additional delay after reset PLL
#define DELAY_BANDCHANGE_1 3
#define DELAY_BANDCHANGE_2 3

/*
 * Maximum supported frequency = FREQ_HARMONICS * 9U
 * configure output as follows:
 * CLK0: frequency + offset
 * CLK1: frequency
 * CLK2: fixed 8MHz
 */
int
si5351_set_frequency_with_offset(uint32_t freq, int offset, uint8_t drive_strength){
  uint8_t band;
  int delay = DELAY_NORMAL;
  if (freq == current_freq)
    return delay;
  uint32_t ofreq = freq + offset;
  uint32_t mul = 1, omul = 1;
  uint32_t rdiv = SI5351_R_DIV_1;
  uint32_t fdiv;
  current_freq = freq;
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
  else if (freq <= 500000U) {
    rdiv = SI5351_R_DIV_64;
     freq<<= 6;
    ofreq<<= 6;
  } else if (freq <= 4000000U) {
    rdiv = SI5351_R_DIV_8;
     freq<<= 3;
    ofreq<<= 3;
  }

  band = si5351_getBand(freq/mul);
  switch (band) {
  case 1:
    // Setup CH0 and CH1 constant PLLA freq at band change, and set CH2 freq = CLK2_FREQUENCY
    if (current_band != 1){
      si5351_setupPLL(SI5351_REG_PLL_A, PLL_N, 0, 1);
      si5351_set_frequency_fixedpll(2, XTALFREQ * PLL_N, CLK2_FREQUENCY, SI5351_R_DIV_1, SI5351_CLK_DRIVE_STRENGTH_2MA|SI5351_CLK_PLL_SELECT_A);
      delay+=DELAY_BANDCHANGE_1;
    }
    // Calculate and set CH0 and CH1 divider
    si5351_set_frequency_fixedpll(0,  (uint64_t)omul * XTALFREQ * PLL_N, ofreq, rdiv, drive_strength|SI5351_CLK_PLL_SELECT_A);
    si5351_set_frequency_fixedpll(1,  (uint64_t) mul * XTALFREQ * PLL_N,  freq, rdiv, drive_strength|SI5351_CLK_PLL_SELECT_A);
    delay+=DELAY_BAND_1;
    break;
  case 2:// fdiv = 6
  case 3:// fdiv = 4;
    fdiv = (band == 2) ? 6 : 4;
    // Setup CH0 and CH1 constant fdiv divider at change
    if (current_band != band){
   	  si5351_setupMultisynth(0, fdiv, 0, 1, SI5351_R_DIV_1, drive_strength|SI5351_CLK_PLL_SELECT_A);
      si5351_setupMultisynth(1, fdiv, 0, 1, SI5351_R_DIV_1, drive_strength|SI5351_CLK_PLL_SELECT_B);
      delay+=DELAY_BANDCHANGE_2;
    }
    // Calculate and set CH0 and CH1 PLL freq
    si5351_setupPLL_freq(SI5351_REG_PLL_A, ofreq, fdiv, omul);// set PLLA freq = (ofreq/omul)*fdiv
    si5351_setupPLL_freq(SI5351_REG_PLL_B,  freq, fdiv,  mul);// set PLLB freq = ( freq/ mul)*fdiv
    // Calculate CH2 freq = CLK2_FREQUENCY, depend from calculated before CH1 PLLB = (freq/mul)*fdiv
    si5351_set_frequency_fixedpll(2, (uint64_t)freq*fdiv, CLK2_FREQUENCY*mul, SI5351_R_DIV_1, SI5351_CLK_DRIVE_STRENGTH_2MA|SI5351_CLK_PLL_SELECT_B);
    break;
  }

  if (current_band != band) {
    si5351_reset_pll(SI5351_PLL_RESET_A|SI5351_PLL_RESET_B);
    current_band = band;
  }
  return delay;
}
