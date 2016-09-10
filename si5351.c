#include "hal.h"
#include "si5351.h"

#define SI5351_I2C_ADDR   	(0x60<<1)

extern int I2CWrite(int addr, char d0, char d1);

static void
si5351_write(uint8_t reg, uint8_t dat)
{
  int addr = SI5351_I2C_ADDR>>1;
  uint8_t buf[] = { reg, dat };
  i2cAcquireBus(&I2CD1);
  (void)i2cMasterTransmitTimeout(&I2CD1, addr, buf, 2, NULL, 0, 1000);
  i2cReleaseBus(&I2CD1);
}

static void
si5351_bulk_write(const uint8_t *buf, int len)
{
  int addr = SI5351_I2C_ADDR>>1;
  i2cAcquireBus(&I2CD1);
  (void)i2cMasterTransmitTimeout(&I2CD1, addr, buf, len, NULL, 0, 1000);
  i2cReleaseBus(&I2CD1);
}

// register addr, length, data, ...
const uint8_t si5351_configs[] = {
  2, SI5351_REG_3_OUTPUT_ENABLE_CONTROL, 0xff,
  4, SI5351_REG_16_CLK0_CONTROL, SI5351_CLK_POWERDOWN, SI5351_CLK_POWERDOWN, SI5351_CLK_POWERDOWN,
  2, SI5351_REG_183_CRYSTAL_LOAD, SI5351_CRYSTAL_LOAD_8PF,
  // setup PLL (26MHz * 32 = 832MHz, 32/2-2=14)
  9, SI5351_REG_26_PLL_A, /*P3*/0, 1, /*P1*/0, 14, 0, /*P3/P2*/0, 0, 0,
  // RESET PLL
  2, SI5351_REG_177_PLL_RESET, SI5351_PLL_RESET_A | SI5351_PLL_RESET_B,
  // setup multisynth (832MHz / 104 = 8MHz, 104/2-2=50)
  9, SI5351_REG_58_MULTISYNTH2, /*P3*/0, 1, /*P1*/0, 50, 0, /*P2|P3*/0, 0, 0,
  2, SI5351_REG_18_CLK2_CONTROL, SI5351_CLK_DRIVE_STRENGTH_2MA | SI5351_CLK_INPUT_MULTISYNTH_N | SI5351_CLK_INTEGER_MODE,
  2, SI5351_REG_3_OUTPUT_ENABLE_CONTROL, 0,
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

void si5351_disable_output(void)
{
  si5351_write(SI5351_REG_3_OUTPUT_ENABLE_CONTROL, 0xff);
  si5351_write(SI5351_REG_16_CLK0_CONTROL, 0x80);
  si5351_write(SI5351_REG_17_CLK1_CONTROL, 0x80);
  si5351_write(SI5351_REG_18_CLK2_CONTROL, 0x80);
}

void si5351_enable_output(void)
{
  si5351_write(SI5351_REG_3_OUTPUT_ENABLE_CONTROL, 0x00);
}

void si5351_reset_pll(void)
{
  //si5351_write(SI5351_REG_177_PLL_RESET, SI5351_PLL_RESET_A | SI5351_PLL_RESET_B);
  si5351_write(SI5351_REG_177_PLL_RESET, 0xAC);
}

void si5351_setupPLL(uint8_t pll, /* SI5351_PLL_A or SI5351_PLL_B */
                     uint8_t     mult,
                     uint32_t    num,
                     uint32_t    denom)
{
  /* Get the appropriate starting point for the PLL registers */
  const uint8_t pllreg_base[] = {
    SI5351_REG_26_PLL_A,
    SI5351_REG_34_PLL_B
  };
  uint8_t baseaddr = pllreg_base[pll];

  uint32_t P1;
  uint32_t P2;
  uint32_t P3;

  /* Feedback Multisynth Divider Equation
   * where: a = mult, b = num and c = denom
   * P1 register is an 18-bit value using following formula:
   * 	P1[17:0] = 128 * mult + floor(128*(num/denom)) - 512
   * P2 register is a 20-bit value using the following formula:
   * 	P2[19:0] = 128 * num - denom * floor(128*(num/denom))
   * P3 register is a 20-bit value using the following formula:
   * 	P3[19:0] = denom
   */

  /* Set the main PLL config registers */
  if (num == 0)
  {
    /* Integer mode */
    P1 = 128 * mult - 512;
    P2 = 0;
    P3 = 1;
  }
  else
  {
    /* Fractional mode */
    //P1 = (uint32_t)(128 * mult + floor(128 * ((float)num/(float)denom)) - 512);
    P1 = 128 * mult + ((128 * num) / denom) - 512;
    //P2 = (uint32_t)(128 * num - denom * floor(128 * ((float)num/(float)denom)));
    P2 = 128 * num - denom * ((128 * num) / denom);
    P3 = denom;
  }

  /* The datasheet is a nightmare of typos and inconsistencies here! */
  si5351_write(baseaddr,   (P3 & 0x0000FF00) >> 8);
  si5351_write(baseaddr+1, (P3 & 0x000000FF));
  si5351_write(baseaddr+2, (P1 & 0x00030000) >> 16);
  si5351_write(baseaddr+3, (P1 & 0x0000FF00) >> 8);
  si5351_write(baseaddr+4, (P1 & 0x000000FF));
  si5351_write(baseaddr+5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16) );
  si5351_write(baseaddr+6, (P2 & 0x0000FF00) >> 8);
  si5351_write(baseaddr+7, (P2 & 0x000000FF));
}

void 
si5351_setupMultisynth(uint8_t     output,
                       uint8_t	   pllSource,
                       uint32_t    div, // 4,6,8, 8+ ~ 900
                       uint32_t    num,
                       uint32_t    denom,
                       uint8_t     drive_strength)
{
  /* Get the appropriate starting point for the PLL registers */
  const uint8_t msreg_base[] = {
    SI5351_REG_42_MULTISYNTH0,
    SI5351_REG_50_MULTISYNTH1,
    SI5351_REG_58_MULTISYNTH2,
  };
  uint8_t baseaddr = msreg_base[output];
  const uint8_t clkctrl[] = {
    SI5351_REG_16_CLK0_CONTROL,
    SI5351_REG_17_CLK1_CONTROL,
    SI5351_REG_18_CLK2_CONTROL
  };
  uint8_t dat;

  uint32_t P1;
  uint32_t P2;
  uint32_t P3;
  uint32_t div4 = 0;

  /* Output Multisynth Divider Equations
   * where: a = div, b = num and c = denom
   * P1 register is an 18-bit value using following formula:
   * 	P1[17:0] = 128 * a + floor(128*(b/c)) - 512
   * P2 register is a 20-bit value using the following formula:
   * 	P2[19:0] = 128 * b - c * floor(128*(b/c))
   * P3 register is a 20-bit value using the following formula:
   * 	P3[19:0] = c
   */
  /* Set the main PLL config registers */
  if (div == 4) {
    div4 = SI5351_DIVBY4;
    P1 = P2 = 0;
    P3 = 1;
  } else if (num == 0) {
    /* Integer mode */
    P1 = 128 * div - 512;
    P2 = 0;
    P3 = 1;
  } else {
    /* Fractional mode */
    P1 = 128 * div + ((128 * num) / denom) - 512;
    P2 = 128 * num - denom * ((128 * num) / denom);
    P3 = denom;
  }

  /* Set the MSx config registers */
  si5351_write(baseaddr,   (P3 & 0x0000FF00) >> 8);
  si5351_write(baseaddr+1, (P3 & 0x000000FF));
  si5351_write(baseaddr+2, ((P1 & 0x00030000) >> 16) | div4);
  si5351_write(baseaddr+3, (P1 & 0x0000FF00) >> 8);
  si5351_write(baseaddr+4, (P1 & 0x000000FF));
  si5351_write(baseaddr+5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
  si5351_write(baseaddr+6, (P2 & 0x0000FF00) >> 8);
  si5351_write(baseaddr+7, (P2 & 0x000000FF));

  /* Configure the clk control and enable the output */
  dat = drive_strength | SI5351_CLK_INPUT_MULTISYNTH_N;
  if (pllSource == SI5351_PLL_B)
    dat |= SI5351_CLK_PLL_SELECT_B;
  if (num == 0)
    dat |= SI5351_CLK_INTEGER_MODE;
  si5351_write(clkctrl[output], dat);
}

static uint32_t
gcd(uint32_t x, uint32_t y)
{
  uint32_t z;
  while (y != 0) {
    z = x % y;
    x = y;
    y = z;
  }
  return x;
}

#define XTALFREQ 26000000L
#define PLL_N 32
#define PLLFREQ (XTALFREQ * PLL_N)

void
si5351_set_frequency_fixedpll(int channel, int pll, int pllfreq, int freq,
                              uint8_t drive_strength)
{
    int32_t div = pllfreq / freq; // range: 8 ~ 1800
    int32_t num = pllfreq - freq * div;
    int32_t denom = freq;
    //int32_t k = freq / (1<<20) + 1;
    int32_t k = gcd(num, denom);
    num /= k;
    denom /= k;
    while (denom >= (1<<20)) {
      num >>= 1;
      denom >>= 1;
    }
    si5351_setupMultisynth(channel, pll, div, num, denom, drive_strength);
}

void
si5351_set_frequency_fixeddiv(int channel, int pll, int freq, int div,
                              uint8_t     drive_strength)
{
    int32_t pllfreq = freq * div;
    int32_t multi = pllfreq / XTALFREQ;
    int32_t num = pllfreq - multi * XTALFREQ;
#if 0
    //int32_t denom = 1000000;
    int32_t denom = 520000;
    int32_t k = XTALFREQ / denom;
    num /= k;
#else
    int32_t denom = XTALFREQ;
    int32_t k = gcd(num, denom);
    num /= k;
    denom /= k;
    while (denom >= (1<<20)) {
      num >>= 1;
      denom >>= 1;
    }
#endif
    si5351_setupPLL(pll, multi, num, denom);
    si5351_setupMultisynth(channel, pll, div, 0, 1, drive_strength);
}

/* 
 * 1~100MHz fixed PLL 900MHz, fractional divider
 * 100~150MHz fractional PLL 600-900MHz, fixed divider 6
 * 150~200MHz fractional PLL 600-900MHz, fixed divider 4
 */
void
si5351_set_frequency(int channel, int freq, uint8_t drive_strength)
{
  if (freq <= 100000000) {
    si5351_set_frequency_fixedpll(channel, SI5351_PLL_B, PLLFREQ, freq, drive_strength);
  } else if (freq < 150000000) {
    si5351_set_frequency_fixeddiv(channel, SI5351_PLL_B, freq, 6, drive_strength);
  } else {
    si5351_set_frequency_fixeddiv(channel, SI5351_PLL_B, freq, 4, drive_strength);
  }
}

/*
 * configure output as follows:
 * CLK0: frequency + offset
 * CLK1: frequency
 * CLK2: fixed 8MHz
 */
#define CLK2_FREQUENCY 8000000L
void
si5351_set_frequency_with_offset(int freq, int offset, uint8_t drive_strength)
{
  si5351_disable_output();
  if (freq <= 100000000) {
    // fractional divider mode. only PLL A is used.
    si5351_setupPLL(SI5351_PLL_A, 32, 0, 1);
    si5351_set_frequency_fixedpll(0, SI5351_PLL_A, PLLFREQ, freq + offset,
                                  SI5351_CLK_DRIVE_STRENGTH_2MA);
    si5351_set_frequency_fixedpll(1, SI5351_PLL_A, PLLFREQ, freq, drive_strength);
    si5351_set_frequency_fixedpll(2, SI5351_PLL_A, PLLFREQ, CLK2_FREQUENCY,
                                  SI5351_CLK_DRIVE_STRENGTH_2MA);
  } else if (freq < 150000000) {
    // div by 6 mode. both PLL A and B are dedicated for CLK0, CLK1
    si5351_set_frequency_fixeddiv(0, SI5351_PLL_A, freq + offset, 6,
                                  SI5351_CLK_DRIVE_STRENGTH_2MA);
    si5351_set_frequency_fixeddiv(1, SI5351_PLL_B, freq, 6, drive_strength);
    si5351_set_frequency_fixedpll(2, SI5351_PLL_B, freq * 6, CLK2_FREQUENCY,
                                  SI5351_CLK_DRIVE_STRENGTH_2MA);
  } else {
    // div by 4 mode. both PLL A and B are dedicated for CLK0, CLK1
    si5351_set_frequency_fixeddiv(0, SI5351_PLL_A, freq + offset, 4,
                                  SI5351_CLK_DRIVE_STRENGTH_2MA);
    si5351_set_frequency_fixeddiv(1, SI5351_PLL_B, freq, 4, drive_strength);
    si5351_set_frequency_fixedpll(2, SI5351_PLL_B, freq * 4, CLK2_FREQUENCY,
                                  SI5351_CLK_DRIVE_STRENGTH_2MA);
  }
  si5351_reset_pll();
  si5351_enable_output();
}
