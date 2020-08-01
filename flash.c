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

uint16_t lastsaveid = 0;
static uint32_t checksum_ok = 0;

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

static void flash_erase_page(uint32_t page_address)
{
  chSysLock();
  flash_erase_page0(page_address);
  chSysUnlock();
}

static inline void flash_unlock(void)
{
  // unlock sequence
  FLASH->KEYR = 0x45670123;
  FLASH->KEYR = 0xCDEF89AB;
}

static void flash_program_half_word_buffer(uint16_t* dst, uint16_t *data, uint16_t size)
{
  uint32_t i;
  flash_unlock();
  // erase flash pages for buffer (aligned to FLASH_PAGESIZE)
  for (i = 0; i < size; i+=FLASH_PAGESIZE)
    flash_erase_page((uint32_t)dst + i);
  // Save buffer
  __IO uint16_t* p = dst;
  for (i = 0; i < size/sizeof(uint16_t); i++){
    flash_wait_for_last_operation();
    FLASH->CR |= FLASH_CR_PG;
    p[i] = data[i];
    flash_wait_for_last_operation();
    FLASH->CR &= ~FLASH_CR_PG;
  }
}

static uint32_t
checksum(const void *start, size_t len)
{
  uint32_t *p = (uint32_t*)start;
  uint32_t value = 0;
  // align by sizeof(uint32_t)
  len = (len + sizeof(uint32_t)-1)/sizeof(uint32_t);
  while (len-- > 0)
    value = __ROR(value, 31) + *p++;
  return value;
}

int
config_save(void)
{
  // Apply magic word and calculate checksum
  config.magic = CONFIG_MAGIC;
  config.checksum = checksum(&config, sizeof config - sizeof config.checksum);

  // write to flash
  flash_program_half_word_buffer((uint16_t*)SAVE_CONFIG_ADDR, (uint16_t*)&config, sizeof(config_t));
  return 0;
}

int
config_recall(void)
{
  const config_t *src = (const config_t*)SAVE_CONFIG_ADDR;

  if (src->magic != CONFIG_MAGIC || checksum(src, sizeof *src - sizeof src->checksum) != src->checksum)
    return -1;
  // duplicated saved data onto sram to be able to modify marker/trace
  memcpy(&config, src, sizeof(config_t));
  return 0;
}

int
caldata_save(uint32_t id)
{
  if (id >= SAVEAREA_MAX)
    return -1;

  // Apply magic word and calculate checksum
  current_props.magic = CONFIG_MAGIC;
  current_props.checksum = checksum(&current_props, sizeof current_props - sizeof current_props.checksum);

  // write to flash
  uint16_t *dst = (uint16_t*)(SAVE_PROP_CONFIG_ADDR + id * SAVE_PROP_CONFIG_SIZE);
  flash_program_half_word_buffer(dst, (uint16_t*)&current_props, sizeof(properties_t));

  // after saving data, make active configuration points to flash
  active_props = (properties_t*)(SAVE_PROP_CONFIG_ADDR + id * SAVE_PROP_CONFIG_SIZE);
  lastsaveid = id;

  return 0;
}

int
caldata_recall(uint32_t id)
{
  if (id >= SAVEAREA_MAX)
    return -1;
  // point to saved area on the flash memory
  properties_t *src = (properties_t*)(SAVE_PROP_CONFIG_ADDR + id * SAVE_PROP_CONFIG_SIZE);

  if (src->magic != CONFIG_MAGIC || checksum(src, sizeof *src - sizeof src->checksum) != src->checksum)
    goto load_default;

  // active configuration points to save data on flash memory
  active_props = src;
  lastsaveid = id;

  // duplicated saved data onto sram to be able to modify marker/trace
  memcpy(&current_props, src, sizeof(properties_t));
  return 0;
load_default:
  load_default_properties();
  return lastsaveid;
}

// Used in interpolate
const properties_t *
caldata_reference(void)
{
  if (lastsaveid >= SAVEAREA_MAX)
    return NULL;
  const properties_t *src;

  src = (const properties_t*)(SAVE_PROP_CONFIG_ADDR + lastsaveid * SAVE_PROP_CONFIG_SIZE);
  // Check crc cache mask (made it only 1 time)
  if (checksum_ok&(1<<lastsaveid))
    return src;
  if (src->magic != CONFIG_MAGIC || checksum(src, sizeof *src - sizeof src->checksum) != src->checksum)
    return NULL;
  checksum_ok|=1<<lastsaveid;
  return src;
}

void
clear_all_config_prop_data(void)
{
  uint32_t i;
  flash_unlock();

  // erase flash pages
  for (i = 0; i < SAVE_FULL_AREA_SIZE; i+=FLASH_PAGESIZE)
    flash_erase_page(SAVE_CONFIG_ADDR + i);
}

