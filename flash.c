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

#define rotate(x) ((x<<1) | (x&(1<<31)?1:0))

static uint32_t
checksum(const void *start, size_t len)
{
  uint32_t *p = (uint32_t*)start;
  uint32_t *tail = (uint32_t*)(start + len);
  uint32_t value = 0;
  while (p < tail)
    value = rotate(value) | *p++;
  return value;
}


#define FLASH_PAGESIZE 0x800

const uint32_t save_config_area = 0x08018000;

int
config_save(void)
{
  uint16_t *src = (uint16_t*)&config;
  uint16_t *dst = (uint16_t*)save_config_area;
  int count = sizeof(config_t) / sizeof(uint16_t);

  config.magic = CONFIG_MAGIC;
  config.checksum = checksum(&config, sizeof config - sizeof config.checksum);

  flash_unlock();

  /* erase flash pages */
  flash_erase_page((uint32_t)dst);

  /* write to flahs */
  while(count-- > 0) {
    flash_program_half_word((uint32_t)dst, *src++);
    dst++;
  }

  return 0;
}

int
config_recall(void)
{
  const config_t *src = (const config_t*)save_config_area;
  void *dst = &config;

  if (src->magic != CONFIG_MAGIC)
    return -1;
  if (checksum(src, sizeof *src - sizeof src->checksum) != src->checksum)
    return -1;

  /* duplicated saved data onto sram to be able to modify marker/trace */
  memcpy(dst, src, sizeof(config_t));
  return 0;
}

#define SAVEAREA_MAX 5

const uint32_t saveareas[] =
  { 0x08018800, 0x0801a000, 0x0801b800, 0x0801d000, 0x0801e800 };

int16_t lastsaveid = 0;


int
caldata_save(int id)
{
  uint16_t *src = (uint16_t*)&current_props;
  uint16_t *dst;
  int count = sizeof(properties_t) / sizeof(uint16_t);

  if (id < 0 || id >= SAVEAREA_MAX)
    return -1;
  dst = (uint16_t*)saveareas[id];

  current_props.magic = CONFIG_MAGIC;
  current_props.checksum = 0;
  current_props.checksum = checksum(&current_props, sizeof current_props);

  flash_unlock();

  /* erase flash pages */
  void *p = dst;
  void *tail = p + sizeof(properties_t);
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
  active_props = (properties_t*)saveareas[id];
  lastsaveid = id;

  return 0;
}

int
caldata_recall(int id)
{
  properties_t *src;
  void *dst = &current_props;

  if (id < 0 || id >= SAVEAREA_MAX)
    return -1;

  // point to saved area on the flash memory
  src = (properties_t*)saveareas[id];

  if (src->magic != CONFIG_MAGIC)
    return -1;
  if (checksum(src, sizeof *src - sizeof src->checksum) != src->checksum)
    return -1;

  /* active configuration points to save data on flash memory */
  active_props = src;
  lastsaveid = id;

  /* duplicated saved data onto sram to be able to modify marker/trace */
  memcpy(dst, src, sizeof(properties_t));

  return 0;
}

const properties_t *
caldata_ref(int id)
{
  const properties_t *src;
  if (id < 0 || id >= SAVEAREA_MAX)
    return NULL;
  src = (const properties_t*)saveareas[id];

  if (src->magic != CONFIG_MAGIC)
    return NULL;
  if (checksum(src, sizeof *src - sizeof src->checksum) != src->checksum)
    return NULL;
  return src;
}

const uint32_t save_config_prop_area_size = 0x8000;

void
clear_all_config_prop_data(void)
{
  flash_unlock();

  /* erase flash pages */
  void *p = (void*)save_config_area;
  void *tail = p + save_config_prop_area_size;
  while (p < tail) {
    flash_erase_page((uint32_t)p);
    p += FLASH_PAGESIZE;
  }
}

