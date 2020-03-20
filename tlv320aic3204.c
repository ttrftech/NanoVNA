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
#include "hal.h"
#include "nanovna.h"

#define REFCLK_8000KHZ
#define AIC3204_ADDR 0x18

#define wait_ms(ms)     chThdSleepMilliseconds(ms)

static const uint8_t conf_data[] = {
// reg, data,
// PLL clock config
  0x00, 0x00, /* Initialize to Page 0 */
  0x01, 0x01, /* Initialize the device through software reset */
  0x04, 0x43, /* PLL Clock High, MCLK, PLL */
#ifdef REFCLK_8000KHZ
  /* 8.000MHz*10.7520 = 86.016MHz, 86.016MHz/(2*7*128) = 48kHz */
  0x05, 0x91, /* Power up PLL, P=1,R=1 */
  0x06, 0x0a, /* J=10 */
  0x07, 29,   /* D=7520 = (29<<8) + 96 */
  0x08, 96,
#endif
// Clock config, default fs=48kHz
  0x0b, 0x82, /* Power up the NDAC divider with value 2 */
  0x0c, 0x87, /* Power up the MDAC divider with value 7 */
  0x0d, 0x00, /* Program the OSR of DAC to 128 */
  0x0e, 0x80,
  0x3c, 0x08, /* Set the DAC Mode to PRB_P8 */
  //0x3c, 25, /* Set the DAC Mode to PRB_P25 */
  0x1b, 0x0c, /* Set the BCLK,WCLK as output */
  0x1e, 0x80 + 28, /* Enable the BCLKN divider with value 28 */
  0x25, 0xee, /* DAC power up */

  0x12, 0x82, /* Power up the NADC divider with value 2 */
  0x13, 0x87, /* Power up the MADC divider with value 7 */
  0x14, 0x80, /* Program the OSR of ADC to 128 */
  0x3d, 0x01, /* Select ADC PRB_R1 */
// Data routing
  0x00, 0x01, /* Select Page 1 */
  0x01, 0x08, /* Disable Internal Crude AVdd in presence of external AVdd supply or before powering up internal AVdd LDO*/
  0x02, 0x01, /* Enable Master Analog Power Control */
  0x7b, 0x01, /* Set the REF charging time to 40ms */
  0x14, 0x25, /* HP soft stepping settings for optimal pop performance at power up Rpop used is 6k with N = 6 and soft step = 20usec. This should work with 47uF coupling capacitor. Can try N=5,6 or 7 time constants as well. Trade-off delay vs “pop” sound. */
  0x0a, 0x33, /* Set the Input Common Mode to 0.9V and Output Common Mode for Headphone to 1.65V */

  0x3d, 0x00, /* Select ADC PTM_R4 */
  0x47, 0x32, /* Set MicPGA startup delay to 3.1ms */
  0x7b, 0x01, /* Set the REF charging time to 40ms */
  0x34, 0x10, /* Route IN2L to LEFT_P with 10K */
  0x36, 0x10, /* Route IN2R to LEFT_N with 10K */
//0x37, 0x04, /* Route IN3R to RIGHT_P with 10K */
//0x39, 0x04, /* Route IN3L to RIGHT_N with 10K */
//0x3b, 0x00, /* Unmute Left MICPGA, Gain selection of 32dB to make channel gain 0dB */
//0x3c, 0x00, /* Unmute Right MICPGA, Gain selection of 32dB to make channel gain 0dB */
};

static const uint8_t conf_data_unmute[] = {
// reg, data,
  0x00, 0x00, /* Select Page 0 */
  0x51, 0xc0, /* Power up Left and Right ADC Channels */
  0x52, 0x00, /* Unmute Left and Right ADC Digital Volume Control */
};

static const uint8_t conf_data_ch3_select[] = {
// reg, data,
  0x00, 0x01, /* Select Page 1 */
  0x37, 0x04, /* Route IN3R to RIGHT_P with input impedance of 10K */
  0x39, 0x04, /* Route IN3L to RIGHT_N with input impedance of 10K */
};

static const uint8_t conf_data_ch1_select[] = {
// reg, data,
  0x00, 0x01, /* Select Page 1 */
  0x37, 0x40, /* Route IN1R to RIGHT_P with input impedance of 10K */
  0x39, 0x10, /* Route IN1L to RIGHT_N with input impedance of 10K */
};

static inline void
tlv320aic3204_bulk_write(const uint8_t *buf, int len)
{
  (void)i2cMasterTransmitTimeout(&I2CD1, AIC3204_ADDR, buf, len, NULL, 0, 1000);
}

#if 0
static int
tlv320aic3204_read(uint8_t d0)
{
  int addr = AIC3204_ADDR;
  uint8_t buf[] = { d0 };
  i2cAcquireBus(&I2CD1);
  i2cMasterTransmitTimeout(&I2CD1, addr, buf, 1, buf, 1, 1000);
  i2cReleaseBus(&I2CD1);
  return buf[0];
}
#endif

static void
tlv320aic3204_config(const uint8_t *data, int len)
{
  i2cAcquireBus(&I2CD1);
  for (; len--; data += 2)
    tlv320aic3204_bulk_write(data, 2);
  i2cReleaseBus(&I2CD1);
}

void tlv320aic3204_init(void)
{
  tlv320aic3204_config(conf_data, sizeof(conf_data)/2);
  wait_ms(40);
  tlv320aic3204_config(conf_data_unmute, sizeof(conf_data_unmute)/2);
}

void tlv320aic3204_select(int channel)
{
  tlv320aic3204_config(channel ? conf_data_ch1_select : conf_data_ch3_select, sizeof(conf_data_ch3_select)/2);
}

void tlv320aic3204_set_gain(int lgain, int rgain)
{
  uint8_t data[] = {
    0x00, 0x01, /* Select Page 1 */
    0x3b, lgain, /* Unmute Left MICPGA, set gain */
    0x3c, rgain, /* Unmute Right MICPGA, set gain */
  };
  tlv320aic3204_config(data, sizeof(data)/2);
}
