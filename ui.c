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
#include <stdlib.h>

struct {
    enum { CHANNEL, FREQ, VOLUME, MOD, AGC, RFGAIN, DGAIN, MODE_MAX } mode;
	int digit; /* 0~5 */
} uistat;

#define NO_EVENT					0
#define EVT_BUTTON_SINGLE_CLICK		0x01
#define EVT_BUTTON_DOUBLE_CLICK		0x02
#define EVT_BUTTON_DOWN_LONG		0x04
#define EVT_UP					0x10
#define EVT_DOWN				0x20
#define EVT_REPEAT				0x40

#define BUTTON_DOWN_LONG_TICKS		10000
#define BUTTON_DOUBLE_TICKS			5000
#define BUTTON_DEBOUNCE_TICKS		10

/* lever switch assignment */
#define BIT_UP1 	3
#define BIT_PUSH	2
#define BIT_DOWN1	1

#define READ_PORT() palReadPort(GPIOA)
#define BUTTON_MASK 0b1111

static uint16_t last_button = 0b0000;
static uint32_t last_button_down_ticks;
static uint8_t inhibit_until_release = FALSE;
static uint8_t last_button_event;
uint8_t operation_requested = FALSE;

int ui_status = FALSE;
int selection = 1;



static int btn_check(void)
{
    int cur_button = READ_PORT() & BUTTON_MASK;
	int changed = last_button ^ cur_button;
	int status = 0;
    uint32_t ticks = chVTGetSystemTime();
	if (changed & (1<<BIT_PUSH)) {
		if (ticks >= last_button_down_ticks + BUTTON_DEBOUNCE_TICKS) {
            if (cur_button & (1<<BIT_PUSH)) {
				// button pushed
              status |= EVT_BUTTON_SINGLE_CLICK;
              last_button_down_ticks = ticks;
			} else {
              // button released
              if (inhibit_until_release) {
                status = 0;
                inhibit_until_release = FALSE;
              }
			}
		}
	} else {
		// button unchanged
		if (cur_button & (1<<BIT_PUSH)
			&& ticks >= last_button_down_ticks + BUTTON_DOWN_LONG_TICKS) {
			status |= EVT_BUTTON_DOWN_LONG;
            inhibit_until_release = TRUE;
		}
	}

	if (cur_button & (1<<BIT_UP1)) {
		if (ticks >= last_button_down_ticks + BUTTON_DEBOUNCE_TICKS) {
          if (changed & (1<<BIT_UP1)) {
            status |= EVT_UP;
            last_button_down_ticks = ticks;
          }
          if (ticks >= last_button_down_ticks + BUTTON_DOWN_LONG_TICKS) {
            status |= EVT_UP;
          }
        }
    }
	if (cur_button & (1<<BIT_DOWN1)) {
		if (ticks >= last_button_down_ticks + BUTTON_DEBOUNCE_TICKS) {
          if (changed & (1<<BIT_DOWN1)) {
            status |= EVT_DOWN;
            last_button_down_ticks = ticks;
          }
          if (ticks >= last_button_down_ticks + BUTTON_DOWN_LONG_TICKS) {
            status |= EVT_DOWN;
          }
        }
    }
    last_button = cur_button;

	return status;
}

static int btn_wait_release(void)
{
  while (TRUE) {
    int cur_button = READ_PORT() & BUTTON_MASK;
    int changed = last_button ^ cur_button;
    uint32_t ticks = chVTGetSystemTime();
    int status = 0;
    if (changed) {
      // finished
      last_button = cur_button;
      return 0;
    }

    if (ticks - last_button_down_ticks >= BUTTON_DOWN_LONG_TICKS) {
      if (cur_button & (1<<BIT_DOWN1)) {
        status |= EVT_DOWN | EVT_REPEAT;
      }
      if (cur_button & (1<<BIT_UP1)) {
        status |= EVT_UP | EVT_REPEAT;
      }
      return status;
    }
  }
}

void
ui_digit(void)
{
    int count = 0;
    while (TRUE) {
        int status = btn_check();
        if (status & EVT_BUTTON_SINGLE_CLICK)
            break;
        if (status & EVT_UP && uistat.digit < 7)
            uistat.digit++;
        if (status & EVT_DOWN && uistat.digit > 0)
            uistat.digit--;
        if (count++ % 4 < 2) {
          //i2clcd_cmd(0x0e); // enable show-cursor flag
          //i2clcd_pos(7 - uistat.digit, 1);
        } else {
          //i2clcd_cmd(0x0c); // disable show-cursor flag
        }
        chThdSleepMilliseconds(100);
    }
}

const char *menu_items[] = {
  "OPEN", "SHORT", "LOAD", "ISOLN", "THRU", NULL, "DONE"
};

void
draw_buttons(void)
{
  int i = 0;
  for (i = 0; i < 7; i++) {
    int y = 32*i;
    uint16_t bg = 0xffff;
    if (i == selection)
      bg = 0x7777;
    if (menu_items[i] != NULL) {
      ili9341_fill(320-60, y, 60, 30, bg);
      ili9341_drawstring_5x7(menu_items[i], 320-54, y+12, 0x0000, bg);
    }
  }
}

void
erase_buttons(void)
{
  uint16_t bg = 0;
  ili9341_fill(320-60, 0, 60, 32*7, bg);
}

void
ui_show(void)
{
  area_width = WIDTH - (64-14-4);
  area_height = HEIGHT;
  draw_buttons();
}

void
ui_hide(void)
{
  area_width = WIDTH;
  area_height = HEIGHT;
  erase_buttons();
  force_draw_cells();
}

void
ui_process(void)
{
  if (!operation_requested)
    return;
  
  int status = btn_check();
  int n;
  if (status != 0) {
    if (status & EVT_BUTTON_SINGLE_CLICK) {
      //markers[active_marker].index = 30;
      ui_status = !ui_status;
      if (ui_status) 
        ui_show();
      else
        ui_hide();
    } else {
      if (ui_status) {
        if (status & EVT_UP) {
          selection++;
          draw_buttons();
        }
        if (status & EVT_DOWN) {
          selection--;
          draw_buttons();
        }
      } else {
        do {
          if (active_marker >= 0 && markers[active_marker].enabled) {
            if ((status & EVT_UP) && markers[active_marker].index > 0) {
              markers[active_marker].index--;
              redraw_marker(active_marker, FALSE);
            }
            if ((status & EVT_DOWN) && markers[active_marker].index < 100) {
              markers[active_marker].index++;
              redraw_marker(active_marker, FALSE);
            }
          }
          status = btn_wait_release();
        } while (status != 0);
        redraw_marker(active_marker, TRUE);
      }
    }
  }
  operation_requested = FALSE;
}

/* Triggered when the button is pressed or released. The LED4 is set to ON.*/
static void extcb1(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;
  operation_requested = TRUE;

#if 0
  if (channel == 1)
    ui_status = TRUE;
  else if (channel == 2)
    ui_status = !ui_status;
  else if (channel == 3)
    ui_status = FALSE;
  if (ui_status)
    ui_show();
  else
    ui_hide();
#endif
}

static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, extcb1},
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, extcb1},
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, extcb1},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL}
  }
};

void
ui_init()
{
  /*
   * Activates the EXT driver 1.
   */
  extStart(&EXTD1, &extcfg);
}
