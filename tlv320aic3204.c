#include "hal.h"
#include "nanovna.h"


#define REFCLK_8000KHZ
#define AIC3204_ADDR 0x18

#define wait_ms(ms)     chThdSleepMilliseconds(ms)


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
