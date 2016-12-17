#include "ch.h"
#include "hal.h"
#include "nanovna.h"


#define ADC_TR(low, high)               (((uint32_t)(high) << 16U) |        \
                                         (uint32_t)(low))
#define ADC_SMPR_SMP_1P5        0U  /**< @brief 14 cycles conversion time   */
#define ADC_CFGR1_RES_12BIT             (0U << 3U)

void adc_init(void)
{
  rccEnableADC1(FALSE);

  /* Calibration procedure.*/
  ADC->CCR = 0;
  ADC1->CR |= ADC_CR_ADCAL;
  while (ADC1->CR & ADC_CR_ADCAL)
    ;

  ADC1->CR = ADC_CR_ADEN;
  while (!(ADC1->ISR & ADC_ISR_ADRDY))
    ;
}

uint16_t adc_single_read(ADC_TypeDef *adc, uint32_t chsel)
{
  /* ADC setup */
  adc->ISR    = adc->ISR;
  adc->IER    = 0;
  adc->TR     = ADC_TR(0, 0);
  adc->SMPR   = ADC_SMPR_SMP_1P5;
  adc->CFGR1  = ADC_CFGR1_RES_12BIT;
  adc->CHSELR = chsel;

  /* ADC conversion start.*/
  adc->CR |= ADC_CR_ADSTART;

  while (adc->CR & ADC_CR_ADSTART)
    ;

  return adc->DR;
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
  adc->TR     = ADC_TR(0, 2000);
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
    extern int awd_count;
    awd_count++;
  }
}

OSAL_IRQ_HANDLER(STM32_ADC1_HANDLER)
{
  OSAL_IRQ_PROLOGUE();

  adc_interrupt(ADC1);

  OSAL_IRQ_EPILOGUE();
}
