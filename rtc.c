/*
 * Copyright (c) 2019-2020, Dmitry (DiSlord) dislordlive@gmail.com
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

#ifdef __USE_RTC__

// Compact STM32 RTC time library
#if HAL_USE_RTC == TRUE
#error "Error VNA use self RTC lib, define HAL_USE_RTC = FALSE in halconf.h"
#endif

// Get RTC time as binary structure in 0x00HHMMSS
uint32_t rtc_get_tr_bin(void){
  uint32_t tr = RTC->TR;
  uint32_t v = (tr&0x0F0F0F) + ((tr&0x707070)>>1) + ((tr&0x707070)>>3);
  return v;
}

// Get RTC time as binary structure in 0x00YYMMDD
uint32_t rtc_get_dr_bin(void){
  uint32_t dr = RTC->DR;
  uint32_t v = (dr&0x000F0F0F) + ((dr&0x00F01030)>>1) + ((dr&0x00F01030)>>3);
  return v;// | ((dr&0xE000)<<15); // day of week at end
}

uint32_t rtc_get_FAT(void) {
  uint32_t fattime;
  uint32_t tr = rtc_get_tr_bin();
  uint32_t dr = rtc_get_dr_bin();
  fattime  = ((tr>> 0)&0xFF) >>  1U; // Seconds / 2
  fattime |= ((tr>> 8)&0xFF) <<  5U; // Minutes
  fattime |= ((tr>>16)&0xFF) << 11U; // Hour
  fattime |= ((dr>> 0)&0xFF) << 16U; // Day
  fattime |= ((dr>> 8)&0xFF) << 21U; // Month
  fattime |= (((dr>>16)&0xFF) + RTC_START_YEAR - 1980) << 25U; // Local year begin from 2000, fat from 1980
  return fattime;
}

void rtc_set_time(uint32_t dr, uint32_t tr) {
  // Beginning of configuration procedure.
  RTC->ISR |= RTC_ISR_INIT;
  while ((RTC->ISR & RTC_ISR_INITF) == 0)
    ;
  // Writing the registers.
  RTC->TR = tr;
  RTC->DR = dr;
  RTC->ISR &= ~RTC_ISR_INIT;
}

#define RTC_PRER(a, s)              ((((a) - 1) << 16) | ((s) - 1))

// Initiate RTC clock, LSE or LSI generators initiate by ChibiOS !!!
void rtc_init(void){
  // Disable write protection.
  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  // If calendar has not been initialized yet then proceed with the initial setup.
  if (!(RTC->ISR & RTC_ISR_INITS)) {
    // Beginning of configuration procedure.
    RTC->ISR |= RTC_ISR_INIT;
    while ((RTC->ISR & RTC_ISR_INITF) == 0)
      ;
    RTC->CR   = 0;
    RTC->ISR  = RTC_ISR_INIT;     // Clearing all but RTC_ISR_INIT.
    RTC->PRER = RTC_PRER(STM32_RTC_PRESA_VALUE, STM32_RTC_PRESS_VALUE);
    RTC->PRER = RTC_PRER(STM32_RTC_PRESA_VALUE, STM32_RTC_PRESS_VALUE);
    RTC->ISR &= ~RTC_ISR_INIT;
  }
  else
    RTC->ISR &= ~RTC_ISR_RSF;
#if 0
  // ChibiOS init BDCR by self!!
  // For add auto select RTC source need rewrite it
  // see hal_lld_backup_domain_init() in hal_lld.c for every CPU
  // Default RTC clock is LSE, but it possible not launch if no quartz installed
  uint32_t rtc_drv  = STM32_RTCSEL_LSI;
  uint32_t rtc_prer = RTC_PRER(40, 1000);

  // If LSE off try launch it
  if ((RCC->BDCR & RCC_BDCR_LSEON) == 0){
    // Try start LSE
    RCC->BDCR |= STM32_LSEDRV | RCC_BDCR_LSEON;
    uint32_t count = 65535;
    do{
      if (RCC->BDCR & RCC_BDCR_LSERDY) break;
    }while (--count);// Waits until LSE is stable. or count == 0
  }
  // Check, if LSE ready, then prepare it data
  if (RCC->BDCR & RCC_BDCR_LSERDY){
    rtc_drv = STM32_RTCSEL_LSE;
    rtc_prer = RTC_PRER(32, 1024);
  } else{
    // Try start LSI
    RCC->CSR |= RCC_CSR_LSION;
    while ((RCC->CSR & RCC_CSR_LSIRDY) == 0)
      ;
  }

  PWR->CR |= PWR_CR_DBP;
  // If the backup domain hasn't been initialized yet then proceed with initialization or source different
  if ((RCC->BDCR & RCC_BDCR_RTCEN) == 0 || (RCC->BDCR & STM32_RTCSEL_MASK)!=rtc_drv) {
    // Backup domain reset.
    RCC->BDCR = RCC_BDCR_BDRST;
    RCC->BDCR = 0;
    // Selects clock source.
    RCC->BDCR |= rtc_drv;
    // Disable write protection.
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
    // Beginning of configuration procedure.
    RTC->ISR |= RTC_ISR_INIT;
    while ((RTC->ISR & RTC_ISR_INITF) == 0)
      ;
    // Prescaler value loaded in registers.
    RTC->CR   = 0;
    RTC->ISR  = RTC_ISR_INIT;     // Clearing all but RTC_ISR_INIT.
    RTC->PRER = rtc_prer;
    RTC->PRER = rtc_prer;
    // Finalizing of configuration procedure.
    RTC->ISR &= ~RTC_ISR_INIT;
    RCC->BDCR |= RCC_BDCR_RTCEN;  // RTC clock enabled.
  }
#endif
}
#endif
