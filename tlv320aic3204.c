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

void tlv320aic3204_config_adc_filter(void);

static void I2CWrite(int addr, uint8_t d0, uint8_t d1)
{
    uint8_t buf[] = { d0, d1 };
    i2cAcquireBus(&I2CD1);
    (void)i2cMasterTransmitTimeout(&I2CD1, addr, buf, 2, NULL, 0, 1000);
    i2cReleaseBus(&I2CD1);
}

void tlv320aic3204_init(void)
{
    I2CWrite(AIC3204_ADDR, 0x00, 0x00); /* Initialize to Page 0 */
    I2CWrite(AIC3204_ADDR, 0x01, 0x01); /* Initialize the device through software reset */
    I2CWrite(AIC3204_ADDR, 0x04, 0x43); /* PLL Clock High, MCLK, PLL */
#ifdef REFCLK_8000KHZ
    /* 8.000MHz*10.7520 = 86.016MHz, 86.016MHz/(2*7*128) = 48kHz */
    I2CWrite(AIC3204_ADDR, 0x05, 0x91); /* Power up PLL, P=1,R=1 */
    I2CWrite(AIC3204_ADDR, 0x06, 0x0a); /* J=10 */
    I2CWrite(AIC3204_ADDR, 0x07, 29);    /* D=7520 = (29<<8) + 96 */
    I2CWrite(AIC3204_ADDR, 0x08, 96);
#endif
#ifdef REFCLK_12000KHZ
    /* 12.000MHz*7.1680 = 86.016MHz, 86.016MHz/(2*7*128) = 48kHz */
    I2CWrite(AIC3204_ADDR, 0x05, 0x91); /* Power up PLL, P=1,R=1 */
    I2CWrite(AIC3204_ADDR, 0x06, 0x07); /* J=7 */
    I2CWrite(AIC3204_ADDR, 0x07, 6);    /* D=1680 = (6<<8) + 144 */
    I2CWrite(AIC3204_ADDR, 0x08, 144);
#endif
#ifdef REFCLK_19200KHZ
    /* 19.200MHz*4.48 = 86.016MHz, 86.016MHz/(2*7*128) = 48kHz */
    I2CWrite(AIC3204_ADDR, 0x05, 0x91); /* Power up PLL, P=1,R=1 */
    I2CWrite(AIC3204_ADDR, 0x06, 0x04); /* J=4 */
    I2CWrite(AIC3204_ADDR, 0x07, 18);    /* D=4800 = (18<<8) + 192 */
    I2CWrite(AIC3204_ADDR, 0x08, 192);
#endif
    I2CWrite(AIC3204_ADDR, 0x0b, 0x82); /* Power up the NDAC divider with value 2 */
    I2CWrite(AIC3204_ADDR, 0x0c, 0x87); /* Power up the MDAC divider with value 7 */
    I2CWrite(AIC3204_ADDR, 0x0d, 0x00); /* Program the OSR of DAC to 128 */
    I2CWrite(AIC3204_ADDR, 0x0e, 0x80);
    I2CWrite(AIC3204_ADDR, 0x3c, 0x08); /* Set the DAC Mode to PRB_P8 */
    I2CWrite(AIC3204_ADDR, 0x1b, 0x0c); /* Set the BCLK,WCLK as output */    
    I2CWrite(AIC3204_ADDR, 0x1e, 0x80 + 28); /* Enable the BCLKN divider with value 28 */
    I2CWrite(AIC3204_ADDR, 0x25, 0xee); /* DAC power up */
    I2CWrite(AIC3204_ADDR, 0x00, 0x01); /* Select Page 1 */
    I2CWrite(AIC3204_ADDR, 0x01, 0x08); /* Disable Internal Crude AVdd in presence of external AVdd supply or before powering up internal AVdd LDO*/
    I2CWrite(AIC3204_ADDR, 0x02, 0x01); /* Enable Master Analog Power Control */
    I2CWrite(AIC3204_ADDR, 0x7b, 0x01); /* Set the REF charging time to 40ms */
//  I2CWrite(AIC3204_ADDR, 0x0a, 0x00); /* Set the Input Common Mode to 0.9V and Output Common Mode for Headphone to Input Common Mode */
    I2CWrite(AIC3204_ADDR, 0x0a, 0x33); /* Set the Input Common Mode to 0.9V and Output Common Mode for Headphone to 1.65V */
    
    I2CWrite(AIC3204_ADDR, 0x00, 0x00); /* Select Page 0 */
    I2CWrite(AIC3204_ADDR, 0x12, 0x87); /* Power up the NADC divider with value 7 */
    I2CWrite(AIC3204_ADDR, 0x13, 0x82); /* Power up the MADC divider with value 2 */
    I2CWrite(AIC3204_ADDR, 0x14, 0x80); /* Program the OSR of ADC to 128 */
    I2CWrite(AIC3204_ADDR, 0x3d, 0x01); /* Select ADC PRB_R1 */
#if 0
  tlv320aic3204_adc_filter_enable(TRUE);
  I2CWrite(AIC3204_ADDR, 0x00, 0x08); // Select page 8, Disable Adaptive Filtering for ADC
  I2CWrite(AIC3204_ADDR, 0x01, 0x00);
  tlv320aic3204_config_adc_filter();
#endif
    I2CWrite(AIC3204_ADDR, 0x00, 0x01); /* Select Page 1 */
    I2CWrite(AIC3204_ADDR, 0x3d, 0x00); /* Select ADC PTM_R4 */
    I2CWrite(AIC3204_ADDR, 0x47, 0x32); /* Set MicPGA startup delay to 3.1ms */
    I2CWrite(AIC3204_ADDR, 0x7b, 0x01); /* Set the REF charging time to 40ms */
    I2CWrite(AIC3204_ADDR, 0x34, 0x10); /* Route IN2L to LEFT_P with 10K input impedance */
    I2CWrite(AIC3204_ADDR, 0x36, 0x10); /* Route IN2R to LEFT_N with 10K input impedance */
    I2CWrite(AIC3204_ADDR, 0x37, 0x04); /* Route IN3R to RIGHT_P with input impedance of 10K */
    I2CWrite(AIC3204_ADDR, 0x39, 0x04); /* Route IN3L to RIGHT_N with impedance of 10K */
    I2CWrite(AIC3204_ADDR, 0x3b, 0); /* Unmute Left MICPGA, Gain selection of 32dB to make channel gain 0dB */
    I2CWrite(AIC3204_ADDR, 0x3c, 0); /* Unmute Right MICPGA, Gain selection of 32dB to make channel gain 0dB */

    wait_ms(40);
    I2CWrite(AIC3204_ADDR, 0x00, 0x00); /* Select Page 0 */
    I2CWrite(AIC3204_ADDR, 0x51, 0xc0); /* Power up Left and Right ADC Channels */
    I2CWrite(AIC3204_ADDR, 0x52, 0x00); /* Unmute Left and Right ADC Digital Volume Control */    

    tlv320aic3204_config_adc_filter();
    tlv320aic3204_adc_filter_enable(TRUE);
}

void tlv320aic3204_select_in3(void)
{
    I2CWrite(AIC3204_ADDR, 0x00, 0x01); /* Select Page 1 */
    I2CWrite(AIC3204_ADDR, 0x37, 0x04); /* Route IN3R to RIGHT_P with input impedance of 10K */
    I2CWrite(AIC3204_ADDR, 0x39, 0x04); /* Route IN3L to RIGHT_N with input impedance of 10K */
    I2CWrite(AIC3204_ADDR, 0x00, 0x00); /* Select Page 0 */
}

void tlv320aic3204_select_in1(void)
{
    I2CWrite(AIC3204_ADDR, 0x00, 0x01); /* Select Page 1 */
    I2CWrite(AIC3204_ADDR, 0x37, 0x40); /* Route IN1R to RIGHT_P with input impedance of 10K */
    I2CWrite(AIC3204_ADDR, 0x39, 0x10); /* Route IN1L to RIGHT_N with input impedance of 10K */
    I2CWrite(AIC3204_ADDR, 0x00, 0x00); /* Select Page 0 */
}

void tlv320aic3204_adc_filter_enable(int enable)
{
  if (enable)
    I2CWrite(AIC3204_ADDR, 0x3d, 0x02); /* Select ADC PRB_R2 */
  else
    I2CWrite(AIC3204_ADDR, 0x3d, 0x01); /* Select ADC PRB_R1 */
}

#if 0
/* bb, aa = signal.ellip(5, 0.1, 100, (4800.0/24000, 5200.0/24000), 'bandpass') */
const uint8_t adc_filter_config[] = {
  /* len, page, reg, data.... */

  /* left channel C7 - C31 */
  92, 8, 36, 
  /* Pg8 Reg36-127 */
  0x0b, 0xb3, 0xea, 0x00,
  0xf5, 0xeb, 0x1c, 0x00,
  0x0b, 0xb3, 0xea, 0x00,
  0x63, 0x04, 0xf8, 0x00,
  0x82, 0xf3, 0x20, 0x00,
  0x0b, 0xb3, 0xea, 0x00,
  0xf8, 0xac, 0x58, 0x00,
  0x0b, 0xb3, 0xea, 0x00, 
  0x64, 0x26, 0x9e, 0x00,
  0x83, 0x9c, 0x9a, 0x00,
  0x0b, 0xb3, 0xea, 0x00,
  0xf5, 0x92, 0x43, 0x00,
  0x0b, 0xb3, 0xea, 0x00,
  0x65, 0xcc, 0x37, 0x00,
  0x82, 0xd1, 0x6e, 0x00,
  0x0b, 0xb3, 0xea, 0x00,
  0xf7, 0xd5, 0x05, 0x00,
  0x0b, 0xb3, 0xea, 0x00,
  0x67, 0x48, 0x63, 0x00,
  0x81, 0x0a, 0xab, 0x00,
  0x0b, 0xb3, 0xea, 0x00,
  0x00, 0x00, 0x00, 0x00,
  0xf4, 0x4c, 0x16, 0x00,
  8, 9, 8,
  /* Pg9 Reg 8-15 */
  0x62, 0xdd, 0xc7, 0x00,
  0x81, 0x1e, 0xf9, 0x00,

  /* right channel C39 - C63 */
  84, 9, 44, 
  /* Pg9 Reg 44-127 */
  0x0b, 0xb3, 0xea, 0x00,
  0xf5, 0xeb, 0x1c, 0x00,
  0x0b, 0xb3, 0xea, 0x00,
  0x63, 0x04, 0xf8, 0x00,
  0x82, 0xf3, 0x20, 0x00,
  0x0b, 0xb3, 0xea, 0x00,
  0xf8, 0xac, 0x58, 0x00,
  0x0b, 0xb3, 0xea, 0x00, 
  0x64, 0x26, 0x9e, 0x00,
  0x83, 0x9c, 0x9a, 0x00,
  0x0b, 0xb3, 0xea, 0x00,
  0xf5, 0x92, 0x43, 0x00,
  0x0b, 0xb3, 0xea, 0x00,
  0x65, 0xcc, 0x37, 0x00,
  0x82, 0xd1, 0x6e, 0x00,
  0x0b, 0xb3, 0xea, 0x00,
  0xf7, 0xd5, 0x05, 0x00,
  0x0b, 0xb3, 0xea, 0x00,
  0x67, 0x48, 0x63, 0x00,
  0x81, 0x0a, 0xab, 0x00,
  0x0b, 0xb3, 0xea, 0x00,

  16, 10, 8,
  /* Pg10 Reg 8-23 */
  0x00, 0x00, 0x00, 0x00,
  0xf4, 0x4c, 0x16, 0x00,
  0x62, 0xdd, 0xc7, 0x00,
  0x81, 0x1e, 0xf9, 0x00,
  0 /* sentinel */
};
#elif 0
/* bb, aa = signal.ellip(2, 0.1, 100, (4500.0/24000, 5500.0/24000), 'bandpass') */
const uint8_t adc_filter_config[] = {
  /* len, page, reg, data.... */
  /* left channel C7 - C31 */
  40, 8, 36, 
  /* Pg8 Reg36-127 */
  0x02, 0x65, 0xce, 0x00,
  0x02, 0x65, 0x1b, 0x00,
  0x02, 0x65, 0xce, 0x00,
  0x65, 0x27, 0x96, 0x00,
  0x90, 0x4b, 0xd5, 0x00,
  0x52, 0x46, 0xbb, 0x00,
  0xad, 0xb9, 0x96, 0x00,
  0x52, 0x46, 0xbb, 0x00,
  0x56, 0x5f, 0xd2, 0x00,
  0x94, 0x52, 0x41, 0x00,
  /* right channel C39 - C63 */
  40, 9, 44, 
  /* Pg9 Reg 44-127 */
  0x02, 0x65, 0xce, 0x00,
  0x02, 0x65, 0x1b, 0x00,
  0x02, 0x65, 0xce, 0x00,
  0x65, 0x27, 0x96, 0x00,
  0x90, 0x4b, 0xd5, 0x00,
  0x52, 0x46, 0xbb, 0x00,
  0xad, 0xb9, 0x96, 0x00,
  0x52, 0x46, 0xbb, 0x00,
  0x56, 0x5f, 0xd2, 0x00,
  0x94, 0x52, 0x41, 0x00,
  0 /* sentinel */
};
#else
/* bb, aa = signal.bessel(2, (4500.0/24000, 5500.0/24000), 'bandpass') */
const uint8_t adc_filter_config[] = {
  /* len, page, reg, data.... */
  /* left channel C7 - C31 */
  40, 8, 36, 
  /* Pg8 Reg36-127 */
  0x02, 0x9b, 0xed, 0x00,
  0x02, 0x9b, 0xed, 0x00,
  0x02, 0x9b, 0xed, 0x00,
  0x5d, 0x91, 0x0f, 0x00,
  0x8e, 0x4b, 0x9a, 0x00,
  0x18, 0x22, 0x1d, 0x00,
  0xe7, 0xdd, 0xe3, 0x00,
  0x18, 0x22, 0x1d, 0x00,
  0x62, 0xd9, 0x9b, 0x00,
  0x8d, 0x2c, 0xda, 0x00,
    
  /* right channel C39 - C63 */
  40, 9, 44, 
  /* Pg9 Reg 44-127 */
  0x02, 0x9b, 0xed, 0x00,
  0x02, 0x9b, 0xed, 0x00,
  0x02, 0x9b, 0xed, 0x00,
  0x5d, 0x91, 0x0f, 0x00,
  0x8e, 0x4b, 0x9a, 0x00,
  0x18, 0x22, 0x1d, 0x00,
  0xe7, 0xdd, 0xe3, 0x00,
  0x18, 0x22, 0x1d, 0x00,
  0x62, 0xd9, 0x9b, 0x00,
  0x8d, 0x2c, 0xda, 0x00,
  0 /* sentinel */
};
#endif


void tlv320aic3204_config_adc_filter(void)
{
  const uint8_t *p = adc_filter_config;

  while (*p != 0) {
    uint8_t len = *p++;
    uint8_t page = *p++;
    uint8_t reg = *p++;
    I2CWrite(AIC3204_ADDR, 0x00, page);
    while (len-- > 0)
      I2CWrite(AIC3204_ADDR, reg++, *p++);
  }
  I2CWrite(AIC3204_ADDR, 0x00, 0x00); /* page 0 */
}


void tlv320aic3204_set_gain(int lgain, int rgain)
{
    if (lgain < 0)
        lgain = 0;
    if (lgain > 95)
        lgain = 95;
    if (rgain < 0)
        rgain = 0;
    if (rgain > 95)
        rgain = 95;

    I2CWrite(AIC3204_ADDR, 0x00, 0x01); /* Select Page 1 */
    I2CWrite(AIC3204_ADDR, 0x3b, lgain); /* Unmute Left MICPGA, set gain */
    I2CWrite(AIC3204_ADDR, 0x3c, rgain); /* Unmute Right MICPGA, set gain */
    I2CWrite(AIC3204_ADDR, 0x00, 0x00); /* Select Page 0 */
}

void tlv320aic3204_set_digital_gain(int gain)
{
    if (gain < -24)
        gain = -24;
    if (gain > 40)
        gain = 40;

    I2CWrite(AIC3204_ADDR, 0x00, 0x00); /* Select Page 0 */
    I2CWrite(AIC3204_ADDR, 0x53, gain & 0x7f); /* Left ADC Channel Volume */
    I2CWrite(AIC3204_ADDR, 0x54, gain & 0x7f); /* Right ADC Channel Volume */
}

void tlv320aic3204_set_volume(int gain)
{
    if (gain > 29)
        gain = 29;
    else if (gain < -6) 
        gain = 0x40;
    else
        gain &= 0x3f;

    I2CWrite(AIC3204_ADDR, 0x00, 0x01); /* Select Page 1 */
    I2CWrite(AIC3204_ADDR, 0x10, gain); /* Unmute Left MICPGA, set gain */
    I2CWrite(AIC3204_ADDR, 0x11, gain); /* Unmute Right MICPGA, set gain */
    I2CWrite(AIC3204_ADDR, 0x00, 0x00); /* Select Page 0 */
}

void tlv320aic3204_agc_config(tlv320aic3204_agc_config_t *conf)
{
    int ctrl = 0;
    if (conf == NULL) {
      ctrl = 0;
    } else {
      ctrl = 0x80
        | ((conf->target_level & 0x7) << 4) 
        | (conf->gain_hysteresis & 0x3);
    }
    I2CWrite(AIC3204_ADDR, 0x00, 0x00); /* Select Page 0 */
    I2CWrite(AIC3204_ADDR, 0x56, ctrl); /* Left AGC Control Register */
    I2CWrite(AIC3204_ADDR, 0x5e, ctrl); /* Right AGC Control Register */
    if (ctrl == 0)
      return;

    ctrl = ((conf->attack & 0x1f) << 3) | (conf->attack_scale & 0x7);
    I2CWrite(AIC3204_ADDR, 0x59, ctrl); /* Left AGC Attack Time */
    I2CWrite(AIC3204_ADDR, 0x61, ctrl); /* Right AGC Attack Time */

    ctrl = ((conf->decay & 0x1f) << 3) | (conf->decay_scale & 0x7);
    I2CWrite(AIC3204_ADDR, 0x5a, ctrl); /* Left AGC Decay Time */
    I2CWrite(AIC3204_ADDR, 0x62, ctrl); /* Right AGC Decay Time */
}
