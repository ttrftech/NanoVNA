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
#include "ch.h"
#include "hal.h"
#include "nanovna.h"


#define ADC_TR(low, high)               (((uint32_t)(high) << 16U) |        \
                                         (uint32_t)(low))
#define ADC_SMPR_SMP_1P5        0U  /**< @brief 14 cycles conversion time   */
#define ADC_SMPR_SMP_239P5      7U  /**< @brief 252 cycles conversion time. */ 
#define ADC_CFGR1_RES_12BIT             (0U << 3U)

void adc_init(void)
{
  rccEnableADC1(FALSE);

  /* Ensure flag states */
  ADC1->IER = 0;

  /* Calibration procedure.*/
  ADC->CCR = 0;
  if (ADC1->CR & ADC_CR_ADEN) {
      ADC1->CR |= ~ADC_CR_ADDIS; /* Disable ADC */
  }
  while (ADC1->CR & ADC_CR_ADEN)
    ;
  ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;
  ADC1->CR |= ADC_CR_ADCAL;
  while (ADC1->CR & ADC_CR_ADCAL)
    ;

  if (ADC1->ISR & ADC_ISR_ADRDY) {
      ADC1->ISR |= ADC_ISR_ADRDY; /* clear ADRDY */
  }
  /* Enable ADC */
  ADC1->CR |= ADC_CR_ADEN;
  while (!(ADC1->ISR & ADC_ISR_ADRDY))
    ;
}

uint16_t adc_single_read(ADC_TypeDef *adc, uint32_t chsel)
{
  /* ADC setup */
  adc->ISR    = adc->ISR;
  adc->IER    = 0;
  adc->TR     = ADC_TR(0, 0);
  adc->SMPR   = ADC_SMPR_SMP_239P5;
  adc->CFGR1  = ADC_CFGR1_RES_12BIT;
  adc->CHSELR = chsel;

  /* ADC conversion start.*/
  adc->CR |= ADC_CR_ADSTART;

  while (adc->CR & ADC_CR_ADSTART)
    ;

  return adc->DR;
}

int16_t adc_vbat_read(ADC_TypeDef *adc)
{
#define ADC_FULL_SCALE 3300
#define VBAT_DIODE_VF 500
#define VREFINT_CAL (*((uint16_t*)0x1FFFF7BA))
	float vbat = 0;
	float vrefint = 0;

	ADC->CCR |= ADC_CCR_VREFEN | ADC_CCR_VBATEN;
	// VREFINT == ADC_IN17
	vrefint = adc_single_read(adc, ADC_CHSELR_CHSEL17);
	// VBAT == ADC_IN18
	// VBATEN enables resiter devider circuit. It consume vbat power.
	vbat = adc_single_read(adc, ADC_CHSELR_CHSEL18);
	ADC->CCR &= ~(ADC_CCR_VREFEN | ADC_CCR_VBATEN);

	uint32_t vbat_raw = (uint32_t)(ADC_FULL_SCALE * VREFINT_CAL * vbat * 2 / (vrefint * ((1<<12)-1)));
	if (vbat_raw < 100) {
		// maybe D2 is not installed
		return -1;
	}
	
	return vbat_raw + VBAT_DIODE_VF;

}

void adc_start_analog_watchdogd(ADC_TypeDef *adc, uint32_t chsel)
{
  uint32_t cfgr1;

  cfgr1 = ADC_CFGR1_RES_12BIT | ADC_CFGR1_AWDEN
    | ADC_CFGR1_EXTEN_0 // rising edge of external trigger
    | ADC_CFGR1_EXTSEL_0 | ADC_CFGR1_EXTSEL_1; // TRG3  , /* CFGR1 */

  /* ADC setup, if it is defined a callback for the analog watch dog then it
     is enabled.*/
  adc->ISR    = adc->ISR;
  adc->IER    = ADC_IER_AWDIE;
  adc->TR     = ADC_TR(0, TOUCH_THRESHOLD);
  adc->SMPR   = ADC_SMPR_SMP_1P5;
  adc->CHSELR = chsel;

  /* ADC configuration and start.*/
  adc->CFGR1  = cfgr1;

  /* ADC conversion start.*/
  adc->CR |= ADC_CR_ADSTART;
}

void adc_stop(ADC_TypeDef *adc)
{
  if (adc->CR & ADC_CR_ADEN) {
    if (adc->CR & ADC_CR_ADSTART) {
      adc->CR |= ADC_CR_ADSTP;
      while (adc->CR & ADC_CR_ADSTP)
        ;
    }

    /*    adc->CR |= ADC_CR_ADDIS;
    while (adc->CR & ADC_CR_ADDIS)
    ;*/
  }
}

void adc_interrupt(ADC_TypeDef *adc)
{
  uint32_t isr = adc->ISR;
  adc->ISR = isr;

  if (isr & ADC_ISR_OVR) {
    /* ADC overflow condition, this could happen only if the DMA is unable
       to read data fast enough.*/
    
  }
  if (isr & ADC_ISR_AWD) {
    /* Analog watchdog error.*/
    handle_touch_interrupt();
  }
}

OSAL_IRQ_HANDLER(STM32_ADC1_HANDLER)
{
  OSAL_IRQ_PROLOGUE();

  adc_interrupt(ADC1);

  OSAL_IRQ_EPILOGUE();
}
