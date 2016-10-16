#include "ch.h"
#include "hal.h"
#include "nanovna.h"
#include <string.h>

static int flash_wait_for_last_operation(void)
{
  while (FLASH->SR == FLASH_SR_BSY) {
    //WWDG->CR = WWDG_CR_T;
  }
  return FLASH->SR;
}

static void flash_erase_page0(uint32_t page_address)
{
	flash_wait_for_last_operation();
	FLASH->CR |= FLASH_CR_PER;
	FLASH->AR = page_address;
	FLASH->CR |= FLASH_CR_STRT;
	flash_wait_for_last_operation();
	FLASH->CR &= ~FLASH_CR_PER;
}

int flash_erase_page(uint32_t page_address)
{
  chSysLock();
  flash_erase_page0(page_address);
  chSysUnlock();
  return 0;
}

void flash_program_half_word(uint32_t address, uint16_t data)
{
	flash_wait_for_last_operation();
	FLASH->CR |= FLASH_CR_PG;
    *(__IO uint16_t*)address = data;
	flash_wait_for_last_operation();
	FLASH->CR &= ~FLASH_CR_PG;
}

void flash_unlock(void) {
  // unlock sequence
  FLASH->KEYR = 0x45670123;
  FLASH->KEYR = 0xCDEF89AB;
}


config_t cal_saved1 __attribute__ ((section(".calsave")));

static uint32_t
checksum(void *start, size_t len)
{
  uint32_t *p = (uint32_t*)start;
  uint32_t *tail = (uint32_t*)(start + len);
  uint32_t value = 0;
  while (p < tail)
    value ^= *p++;
  return value;
}

int
caldata_save(void)
{
  uint16_t *src = (uint16_t*)&current_config;
  uint16_t *dst = (uint16_t*)&cal_saved1;
  int count = sizeof current_config / sizeof(uint16_t);

  current_config.magic = CONFIG_MAGIC;
  current_config.checksum = 0;
  current_config.checksum = checksum(&current_config, sizeof current_config);

  flash_unlock();
  flash_erase_page(0x801e800);
  flash_erase_page(0x801f000);
  flash_erase_page(0x801f800);
  flash_erase_page((uint32_t)dst);
  while(count-- > 0) {
    flash_program_half_word((uint32_t)dst, *src++);
    dst++;
  }
  return 0;
}

int
caldata_recall(void)
{
  void *src = &cal_saved1;
  void *dst = &current_config;

  if (cal_saved1.magic != CONFIG_MAGIC)
    return -1;
  if (checksum(&cal_saved1, sizeof cal_saved1) != 0)
    return -1;

  memcpy(dst, src, sizeof current_config);
  return 0;
}
