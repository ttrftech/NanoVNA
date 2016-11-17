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

void flash_unlock(void)
{
  // unlock sequence
  FLASH->KEYR = 0x45670123;
  FLASH->KEYR = 0xCDEF89AB;
}


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

#define SAVEAREA_MAX 5

const uint32_t saveareas[] =
  { 0x08018800, 0x0801a000, 0x0801b800, 0x0801d000, 0x0801e8000 };

#define FLASH_PAGESIZE 0x800

int16_t lastsaveid = 0;


int
caldata_save(int id)
{
  uint16_t *src = (uint16_t*)&current_config;
  uint16_t *dst;
  int count = sizeof(config_t) / sizeof(uint16_t);

  if (id < 0 || id >= SAVEAREA_MAX)
    return -1;
  dst = (uint16_t*)saveareas[id];

  current_config.magic = CONFIG_MAGIC;
  current_config.checksum = 0;
  current_config.checksum = checksum(&current_config, sizeof current_config);

  flash_unlock();

  /* erase flash pages */
  void *p = dst;
  void *tail = p + sizeof(config_t);
  while (p < tail) {
    flash_erase_page((uint32_t)p);
    p += FLASH_PAGESIZE;
  }

  /* write to flahs */
  while(count-- > 0) {
    flash_program_half_word((uint32_t)dst, *src++);
    dst++;
  }

  /* after saving data, make active configuration points to flash */
  active = (config_t*)saveareas[id];
  lastsaveid = id;

  return 0;
}

int
caldata_recall(int id)
{
  config_t *src;
  void *dst = &current_config;

  if (id < 0 || id >= SAVEAREA_MAX)
    return -1;
  src = (config_t*)saveareas[id];

  if (src->magic != CONFIG_MAGIC)
    return -1;
  if (checksum(src, sizeof(config_t)) != 0)
    return -1;

  /* active configuration points to save data on flash memory */
  active = src;
  lastsaveid = id;

  /* duplicated saved data onto sram to be able to modify marker/trace */
  memcpy(dst, src, sizeof(config_t));

  return 0;
}
