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
  int digit; /* 0~5 */
  int current_trace; /* 0..3 */
} uistat;


#define NO_EVENT					0
#define EVT_BUTTON_SINGLE_CLICK		0x01
#define EVT_BUTTON_DOUBLE_CLICK		0x02
#define EVT_BUTTON_DOWN_LONG		0x04
#define EVT_UP					0x10
#define EVT_DOWN				0x20
#define EVT_REPEAT				0x40

#define BUTTON_DOWN_LONG_TICKS		5000  /* 1sec */
#define BUTTON_DOUBLE_TICKS			5000   /* 500ms */
#define BUTTON_REPEAT_TICKS			1000   /* 100ms */
#define BUTTON_DEBOUNCE_TICKS		100

/* lever switch assignment */
#define BIT_UP1 	3
#define BIT_PUSH	2
#define BIT_DOWN1	1

#define READ_PORT() palReadPort(GPIOA)
#define BUTTON_MASK 0b1111

static uint16_t last_button = 0b0000;
static uint32_t last_button_down_ticks;
static uint32_t last_button_repeat_ticks;

enum { OP_NONE = 0, OP_LEVER, OP_TOUCH };
uint8_t operation_requested = OP_NONE;

enum {
  UI_NORMAL, UI_MENU, UI_KEYPAD
};

enum {
  KM_START, KM_STOP, KM_CENTER, KM_SPAN, KM_CW, KM_SCALE
};

uint8_t ui_mode = UI_NORMAL;
uint8_t keypad_mode;
uint8_t selection = 0;

void ui_mode_normal(void);
void ui_mode_menu(void);
void ui_mode_keypad(int _keypad_mode);
void draw_menu(void);
void erase_menu_buttons(void);

typedef struct {
  uint8_t type;
  char *label;
  const void *reference;
} menuitem_t;

static void menu_push_submenu(const menuitem_t *submenu);


static int btn_check(void)
{
    int cur_button = READ_PORT() & BUTTON_MASK;
	int changed = last_button ^ cur_button;
	int status = 0;
    uint32_t ticks = chVTGetSystemTime();
	if (changed & (1<<BIT_PUSH)) {
		if (cur_button & (1<<BIT_PUSH)
            && ticks - last_button_down_ticks >= BUTTON_DEBOUNCE_TICKS) {
          // button pushed
          status |= EVT_BUTTON_SINGLE_CLICK;
		}
        last_button_down_ticks = ticks;
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

    if (ticks - last_button_down_ticks >= BUTTON_DOWN_LONG_TICKS
        && ticks - last_button_repeat_ticks >= BUTTON_REPEAT_TICKS) {
      if (cur_button & (1<<BIT_DOWN1)) {
        status |= EVT_DOWN | EVT_REPEAT;
      }
      if (cur_button & (1<<BIT_UP1)) {
        status |= EVT_UP | EVT_REPEAT;
      }
      last_button_repeat_ticks = ticks;
      return status;
    }
  }
}

int
touch_measure_y(void)
{
  // open Y line
  palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_PULLDOWN );
  palSetPadMode(GPIOA, 7, PAL_MODE_INPUT_PULLDOWN );
  // drive low to high on X line
  palSetPadMode(GPIOB, 0, PAL_MODE_OUTPUT_PUSHPULL );
  palClearPad(GPIOB, 0);
  palSetPadMode(GPIOA, 6, PAL_MODE_OUTPUT_PUSHPULL );
  palSetPad(GPIOA, 6);

  chThdSleepMilliseconds(1);

  return adc_single_read(ADC1, ADC_CHSELR_CHSEL7);
}

int
touch_measure_x(void)
{
  // open X line
  palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_PULLDOWN );
  palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_PULLDOWN );
  // drive low to high on Y line
  palSetPadMode(GPIOB, 1, PAL_MODE_OUTPUT_PUSHPULL );
  palSetPad(GPIOB, 1);
  palSetPadMode(GPIOA, 7, PAL_MODE_OUTPUT_PUSHPULL );
  palClearPad(GPIOA, 7);

  chThdSleepMilliseconds(1);

  return adc_single_read(ADC1, ADC_CHSELR_CHSEL6);
}

void
touch_prepare_sense(void)
{
  // open Y line
  palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_PULLDOWN );
  palSetPadMode(GPIOA, 7, PAL_MODE_INPUT_PULLDOWN );
  // force high X line
  palSetPadMode(GPIOB, 0, PAL_MODE_OUTPUT_PUSHPULL );
  palSetPad(GPIOB, 0);
  palSetPadMode(GPIOA, 6, PAL_MODE_OUTPUT_PUSHPULL );
  palSetPad(GPIOA, 6);
}

void
touch_start_watchdog(void)
{
  touch_prepare_sense();
  adc_start_analog_watchdogd(ADC1, ADC_CHSELR_CHSEL7);
}

int
touch_status(void)
{
  touch_prepare_sense();
  return adc_single_read(ADC1, ADC_CHSELR_CHSEL7) > TOUCH_THRESHOLD;
}

int8_t last_touch_status = FALSE;
int16_t last_touch_x;
int16_t last_touch_y;
//int16_t touch_cal[4] = { 1000, 1000, 10*16, 12*16 };
int16_t touch_cal[4] = { 620, 600, 130, 180 };
#define EVT_TOUCH_NONE 0
#define EVT_TOUCH_DOWN 1
#define EVT_TOUCH_PRESSED 2
#define EVT_TOUCH_RELEASED 3

int touch_check(void)
{
  int stat = touch_status();
  if (stat) {
    last_touch_x = touch_measure_x();
    last_touch_y = touch_measure_y();
  }

  if (stat != last_touch_status) {
    last_touch_status = stat;
    if (stat) {
      return EVT_TOUCH_PRESSED;
    } else {
      return EVT_TOUCH_RELEASED;
    }
  } else {
    if (stat) 
      return EVT_TOUCH_DOWN;
    else
      return EVT_TOUCH_NONE;
  }
}

void touch_wait_release(void)
{
  int status;
  /* wait touch release */
  do {
    status = touch_check();
  } while(status != EVT_TOUCH_RELEASED);
}

void
touch_cal_exec(void)
{
  int status;
  int x1, x2, y1, y2;
  
  adc_stop(ADC1);

  do {
    status = touch_check();
  } while(status != EVT_TOUCH_PRESSED);
  x1 = last_touch_x;
  y1 = last_touch_y;

  do {
    status = touch_check();
  } while(status != EVT_TOUCH_PRESSED);
  x2 = last_touch_x;
  y2 = last_touch_y;

  touch_cal[0] = x1;
  touch_cal[1] = y1;
  touch_cal[2] = (x2 - x1) * 16 / 320;
  touch_cal[3] = (y2 - y1) * 16 / 240;
}

void
touch_position(int *x, int *y)
{
  *x = (last_touch_x - touch_cal[0]) * 16 / touch_cal[2];
  *y = (last_touch_y - touch_cal[1]) * 16 / touch_cal[3];
}




// type of menu item 
enum {
  MT_NONE,
  MT_BLANK,
  MT_SUBMENU,
  MT_CALLBACK,
  MT_CANCEL,
  MT_CLOSE
};

typedef void (*menuaction_cb_t)(int item);


static void menu_move_back(void);


static void
menu_calop_cb(int item)
{
  switch (item) {
  case 0: // OPEN
    cal_collect(CAL_OPEN);
    break;
  case 1: // SHORT
    cal_collect(CAL_SHORT);
    break;
  case 2: // LOAD
    cal_collect(CAL_LOAD);
    break;
  case 3: // ISOLN
    cal_collect(CAL_ISOLN);
    break;
  case 4: // THRU
    cal_collect(CAL_THRU);
    break;
  }
  selection++;
  draw_cal_status();
  draw_menu();
}

static void
menu_caldone_cb(int item)
{
  (void)item;
  cal_done();
  draw_cal_status();
  menu_move_back();
}

static void
menu_cal2_cb(int item)
{
  switch (item) {
  case 0: // RESET
    cal_status = 0;
    break;
  case 1: // OFF
    cal_status &= ~CALSTAT_APPLY;
    break;
  case 2: // ON
    cal_status |= CALSTAT_APPLY;
    break;
  }
  draw_cal_status();
  menu_move_back();
}

static void
menu_recall_cb(int item)
{
  if (item < 0 || item >= 5)
    return;
  if (caldata_recall(item) == 0) {
    ui_mode_normal();
    update_grid();
    draw_cal_status();
  }
}

static void
menu_save_cb(int item)
{
  if (item < 0 || item >= 5)
    return;
  if (caldata_save(item) == 0) {
    ui_mode_normal();
    draw_cal_status();
  }
}

static void
menu_trace_cb(int item)
{
  if (item < 0 || item >= 4)
    return;
  uistat.current_trace = item;
  menu_move_back();
}

static void
menu_format_cb(int item)
{
  set_trace_type(uistat.current_trace, item);
  ui_mode_normal();
}

static void
menu_format2_cb(int item)
{
  menu_format_cb(item + 5);
}

static void
menu_channel_cb(int item)
{
  if (item < 0 || item >= 2)
    return;
  set_trace_channel(uistat.current_trace, item);
  menu_move_back();
  ui_mode_normal();
}

static void 
choose_active_marker(void)
{
  int i;
  for (i = 0; i < 4; i++)
    if (markers[i].enabled) {
      active_marker = i;
      return;
    }
  active_marker = -1;
}

void ui_process_keypad(void);

static void
menu_scale_cb(int item)
{
  (void)item;
  ui_mode_keypad(KM_SCALE);
  ui_process_keypad();
}

static void
menu_stimulus_cb(int item)
{
  switch (item) {
  case 0:
  case 1:
  case 2:
  case 3:
  case 4:
    ui_mode_keypad(item);
    ui_process_keypad();
    break;
  }
}

static void
menu_marker_cb(int item)
{
  if (item < 0 || item >= 4)
    return;

  if (active_marker == item) {
    markers[active_marker].enabled = FALSE;
    choose_active_marker();
  } else {
    active_marker = item;
    markers[active_marker].enabled = TRUE;
  }
  if (active_marker >= 0)
    redraw_marker(active_marker, TRUE);
  ui_mode_normal();
}

const menuitem_t menu_calop[] = {
  { MT_CALLBACK, "OPEN", menu_calop_cb },
  { MT_CALLBACK, "SHORT", menu_calop_cb },
  { MT_CALLBACK, "LOAD", menu_calop_cb },
  { MT_CALLBACK, "ISOLN", menu_calop_cb },
  { MT_CALLBACK, "THRU", menu_calop_cb },
  { MT_CALLBACK, "DONE", menu_caldone_cb },
  { MT_CANCEL, "BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_cal[] = {
  { MT_CALLBACK, "RESET", menu_cal2_cb },
  { MT_CALLBACK, "OFF", menu_cal2_cb },
  { MT_CALLBACK, "ON", menu_cal2_cb },
  { MT_CANCEL, "BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

static void
menu_cal_cb(int item)
{
  (void)item;
  if (cal_status != 0) {
    menu_push_submenu(menu_cal);
  } else {
    menu_push_submenu(menu_calop);
  }
}


const menuitem_t menu_trace[] = {
  { MT_CALLBACK, "0", menu_trace_cb },
  { MT_CALLBACK, "1", menu_trace_cb },
  { MT_CALLBACK, "2", menu_trace_cb },
  { MT_CALLBACK, "3", menu_trace_cb },
  { MT_CANCEL, "BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_format2[] = {
  { MT_CALLBACK, "LINEAR", menu_format2_cb },
  { MT_CALLBACK, "SWR", menu_format2_cb },
  { MT_CANCEL, "BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_format[] = {
  { MT_CALLBACK, "LOGMAG", menu_format_cb },
  { MT_CALLBACK, "PHASE", menu_format_cb },
  { MT_CALLBACK, "SMITH", menu_format_cb },
  { MT_CALLBACK, "ADMIT", menu_format_cb },
  { MT_CALLBACK, "POLAR", menu_format_cb },
  { MT_SUBMENU, "NEXT", menu_format2 },  
  //{ MT_CALLBACK, "LINEAR", menu_format_cb },
  //{ MT_CALLBACK, "SWR", menu_format_cb },
  { MT_CANCEL, "BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_channel[] = {
  { MT_CALLBACK, "CH0", menu_channel_cb },
  { MT_CALLBACK, "CH1", menu_channel_cb },
  { MT_CANCEL, "BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_display[] = {
  { MT_SUBMENU, "TRACE", menu_trace },
  { MT_SUBMENU, "FORMAT", menu_format },
  { MT_CALLBACK, "SCALE", menu_scale_cb },
  { MT_SUBMENU, "CHANNEL", menu_channel },
  { MT_CANCEL, "BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_stimulus[] = {
  { MT_CALLBACK, "START", menu_stimulus_cb },
  { MT_CALLBACK, "STOP", menu_stimulus_cb },
  { MT_CALLBACK, "CENTER", menu_stimulus_cb },
  { MT_CALLBACK, "SPAN", menu_stimulus_cb },
  { MT_CALLBACK, "CW", menu_stimulus_cb },
  { MT_CANCEL, "BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_marker[] = {
  { MT_CALLBACK, "1", menu_marker_cb },
  { MT_CALLBACK, "2", menu_marker_cb },
  { MT_CALLBACK, "3", menu_marker_cb },
  { MT_CALLBACK, "4", menu_marker_cb },
  { MT_CANCEL, "BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_recall[] = {
  { MT_CALLBACK, "0", menu_recall_cb },
  { MT_CALLBACK, "1", menu_recall_cb },
  { MT_CALLBACK, "2", menu_recall_cb },
  { MT_CALLBACK, "3", menu_recall_cb },
  { MT_CALLBACK, "4", menu_recall_cb },
  { MT_CANCEL, "BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_save[] = {
  { MT_CALLBACK, "0", menu_save_cb },
  { MT_CALLBACK, "1", menu_save_cb },
  { MT_CALLBACK, "2", menu_save_cb },
  { MT_CALLBACK, "3", menu_save_cb },
  { MT_CALLBACK, "4", menu_save_cb },
  { MT_CANCEL, "BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_top[] = {
  { MT_SUBMENU, "DISPLAY", menu_display },
  { MT_SUBMENU, "MARKER", menu_marker },
  { MT_SUBMENU, "STIMULUS", menu_stimulus },
  { MT_CALLBACK, "CAL", menu_cal_cb },
  { MT_SUBMENU, "RECALL", menu_recall },
  { MT_SUBMENU, "SAVE", menu_save },
  { MT_CLOSE, "CLOSE", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

#define MENU_STACK_DEPTH_MAX 4
uint8_t menu_current_level = 0;
const menuitem_t *menu_stack[4] = {
  menu_top, NULL, NULL, NULL
};

static void
ensure_selection(void)
{
  const menuitem_t *menu = menu_stack[menu_current_level];
  int i;
  for (i = 0; menu[i].type != MT_NONE; i++)
    ;
  if (selection >= i)
    selection = i-1;
}

static void menu_move_back(void)
{
  if (menu_current_level == 0)
    return;
  menu_current_level--;
  ensure_selection();
  erase_menu_buttons();
  draw_menu();
}

static void menu_push_submenu(const menuitem_t *submenu)
{
  if (menu_current_level < MENU_STACK_DEPTH_MAX-1)
    menu_current_level++;
  menu_stack[menu_current_level] = submenu;
  ensure_selection();
  erase_menu_buttons();
  draw_menu();
}

static void menu_move_top(void)
{
  if (menu_current_level == 0)
    return;
  menu_current_level = 0;
  ensure_selection();
  erase_menu_buttons();
  draw_menu();
}

void menu_invoke(int item)
{
  const menuitem_t *menu = menu_stack[menu_current_level];
  menu = &menu[item];

  switch (menu->type) {
  case MT_NONE:
  case MT_BLANK:
  case MT_CLOSE:
    ui_mode_normal();
    break;

  case MT_CANCEL:
    menu_move_back();
    break;

  case MT_CALLBACK: {
    menuaction_cb_t cb = (menuaction_cb_t)menu->reference;
    if (cb == NULL)
      return;
    (*cb)(item);
    break;
  }

  case MT_SUBMENU:
    menu_push_submenu((const menuitem_t*)menu->reference);
    break;
  }
}

#define KP_X(x) (48*(x) + 2 + (320-64-192))
#define KP_Y(y) (48*(y) + 2)

#define KP_PERIOD 10
#define KP_MINUS 11
#define KP_X1 12
#define KP_K 13
#define KP_M 14
#define KP_G 15
#define KP_BS 16
#define KP_INF 17
#define KP_DB 18

const struct {
  uint16_t x, y;
  uint8_t c;
} keypads[] = {
  { KP_X(1), KP_Y(3), KP_PERIOD },
  { KP_X(0), KP_Y(3), 0 },
  { KP_X(0), KP_Y(2), 1 },
  { KP_X(1), KP_Y(2), 2 },
  { KP_X(2), KP_Y(2), 3 },
  { KP_X(0), KP_Y(1), 4 },
  { KP_X(1), KP_Y(1), 5 },
  { KP_X(2), KP_Y(1), 6 },
  { KP_X(0), KP_Y(0), 7 },
  { KP_X(1), KP_Y(0), 8 },
  { KP_X(2), KP_Y(0), 9 },
  { KP_X(3), KP_Y(0), KP_G },
  { KP_X(3), KP_Y(1), KP_M },
  { KP_X(3), KP_Y(2), KP_K },
  { KP_X(3), KP_Y(3), KP_X1 },
  { KP_X(2), KP_Y(3), KP_BS },
  { 0, 0, 0 }
};

void
draw_keypad(void)
{
  int i = 0;
  while (keypads[i].x) {
    uint16_t bg = 0xffff;
    if (i == selection)
      bg = 0x7777;
    ili9341_fill(keypads[i].x, keypads[i].y, 44, 44, bg);
    ili9341_drawfont(keypads[i].c, &NF20x24, keypads[i].x+12, keypads[i].y+10, 0x0000, bg);
    i++;
  }
}

const char *keypad_mode_label[] = {
  "START", "STOP", "CENTER", "SPAN", "SCALE"
};

void
draw_numeric_input(const char *buf)
{
  int i = 0;
  ili9341_fill(0, 208, 320, 32, 0xffff);
  ili9341_drawstring_5x7(keypad_mode_label[keypad_mode], 10, 220, 0x0000, 0xffff);
  while (buf[i] && i < 10) {
    int c;
    if (buf[i] == '.')
      c = KP_PERIOD;
    else if (buf[i] == '-')
      c = KP_MINUS;
    else if (buf[i] >= '0' && buf[i] <= '9')
      c = buf[i] - '0';
    else {
      i++;
      continue;
    }
    ili9341_drawfont(c, &NF20x24, i * 20 + 64, 208+4, 0x0000, 0xffff);
    i++;
  }
}


void
draw_menu_buttons(const menuitem_t *menu)
{
  int i = 0;
  for (i = 0; i < 7; i++) {
    if (menu[i].type == MT_NONE)
      break;
    if (menu[i].type == MT_BLANK) 
      continue;
    int y = 32*i;
    uint16_t bg = 0xffff;
    // focus only in MENU mode but not in KEYPAD mode
    if (ui_mode == UI_MENU && i == selection)
      bg = 0x7777;
    ili9341_fill(320-60, y, 60, 30, bg);
    ili9341_drawstring_5x7(menu[i].label, 320-54, y+12, 0x0000, bg);
  }
}

void
menu_select_touch(int i)
{
  selection = i;
  draw_menu();
  touch_wait_release();
  menu_invoke(selection);
}

void
menu_apply_touch(void)
{
  int touch_x, touch_y;
  const menuitem_t *menu = menu_stack[menu_current_level];
  int i;

  touch_position(&touch_x, &touch_y);
  for (i = 0; i < 7; i++) {
    if (menu[i].type == MT_NONE)
      break;
    if (menu[i].type == MT_BLANK) 
      continue;
    int y = 32*i;
    if (y < touch_y && touch_y < y+30
        && 320-60 < touch_x && touch_x < 320) {
      menu_select_touch(i);
      return;
    }
  }

  touch_wait_release();
  ui_mode_normal();
}

void
draw_menu(void)
{
  draw_menu_buttons(menu_stack[menu_current_level]);
}

void
erase_menu_buttons(void)
{
  uint16_t bg = 0;
  ili9341_fill(320-60, 0, 60, 32*7, bg);
}

void
ui_mode_menu(void)
{
  if (ui_mode == UI_MENU) 
    return;

  ui_mode = UI_MENU;
  area_width = WIDTH - (64-14-4);
  area_height = HEIGHT;
  ensure_selection();
  draw_menu();
}

void
ui_mode_keypad(int _keypad_mode)
{
  if (ui_mode == UI_KEYPAD) 
    return;

  ui_mode = UI_KEYPAD;
  area_width = WIDTH - (64-14-4);
  area_height = HEIGHT;
  draw_menu();
  draw_keypad();
  keypad_mode = _keypad_mode;
  draw_numeric_input("");
}

void
ui_mode_normal(void)
{
  if (ui_mode == UI_NORMAL) 
    return;

  ui_mode = UI_NORMAL;
  area_width = WIDTH;
  area_height = HEIGHT;
  erase_menu_buttons();
  force_draw_cells();
}

void
ui_process_normal(void)
{
  int status = btn_check();
  if (status != 0) {
    if (status & EVT_BUTTON_SINGLE_CLICK) {
      ui_mode_menu();
    } else {
      do {
        if (active_marker >= 0 && markers[active_marker].enabled) {
          if ((status & EVT_DOWN) && markers[active_marker].index > 0) {
            markers[active_marker].index--;
            redraw_marker(active_marker, FALSE);
          }
          if ((status & EVT_UP) && markers[active_marker].index < 100) {
            markers[active_marker].index++;
            redraw_marker(active_marker, FALSE);
          }
        }
        status = btn_wait_release();
      } while (status != 0);
      if (active_marker >= 0)
        redraw_marker(active_marker, TRUE);
    }
  }
}

void
ui_process_menu(void)
{
  int status = btn_check();
  if (status != 0) {
    if (status & EVT_BUTTON_SINGLE_CLICK) {
      menu_invoke(selection);
    } else {
      do {
        if (status & EVT_UP
            && menu_stack[menu_current_level][selection+1].type != MT_NONE) {
          selection++;
          draw_menu();
        }
        if (status & EVT_DOWN
            && selection > 0) {
          selection--;
          draw_menu();
        }
        status = btn_wait_release();
      } while (status != 0);
    }
  }
}

#define NUMINPUT_LEN 10

#define KP_CONTINUE 0
#define KP_DONE 1
#define KP_CANCEL 2

char kp_buf[11];
int8_t kp_index = 0;

int
keypad_click(int selection) 
{
  int c = keypads[selection].c;
  if (c >= KP_X1 && c <= KP_G) {
    int n = c - KP_X1;
    float scale = 1;
    while (n-- > 0)
      scale *= 1000;
    /* numeric input done */
    float value = my_atof(kp_buf) * scale;
    switch (keypad_mode) {
    case KM_START:
      set_sweep_frequency(ST_START, value);
      break;
    case KM_STOP:
      set_sweep_frequency(ST_STOP, value);
      break;
    case KM_CENTER:
      set_sweep_frequency(ST_CENTER, value);
      break;
    case KM_SPAN:
      set_sweep_frequency(ST_SPAN, value);
      break;
    case KM_CW:
      set_sweep_frequency(ST_CW, value);
      break;
    case KM_SCALE:
      set_trace_scale(uistat.current_trace, value);
      break;
    }

    return KP_DONE;
  } else if (c <= 9 && kp_index < NUMINPUT_LEN)
    kp_buf[kp_index++] = '0' + c;
  else if (c == KP_PERIOD && kp_index < NUMINPUT_LEN) {
    // check period in former input
    int j;
    for (j = 0; j < kp_index && kp_buf[j] != '.'; j++)
      ;
    // append period if there are no period
    if (kp_index == j)
      kp_buf[kp_index++] = '.';
  } else if (c == KP_BS) {
    if (kp_index == 0) {
      return KP_CANCEL;
    }
    --kp_index;
  }
  kp_buf[kp_index] = '\0';
  draw_numeric_input(kp_buf);
  return KP_CONTINUE;
}

int
keypad_apply_touch(void)
{
  int touch_x, touch_y;
  int i = 0;

  touch_position(&touch_x, &touch_y);

  while (keypads[i].x) {
    if (keypads[i].x < touch_x && touch_x < keypads[i].x+44
        && keypads[i].y < touch_y && touch_y < keypads[i].y+44) {
      selection = i;
      draw_keypad();
      touch_wait_release();
      return TRUE;
    }
    i++;
  }
  return FALSE;
}

void
ui_process_keypad(void)
{
  int status;
  adc_stop(ADC1);

  kp_index = 0;
  while (TRUE) {
    status = btn_check();
    if (status & (EVT_UP|EVT_DOWN)) {
      int s = status;
      do {
        if (s & EVT_UP) {
          selection--;
          selection %= 16;
          draw_keypad();
        }
        if (s & EVT_DOWN) {
          selection++;
          selection %= 16;
          draw_keypad();
        }
        s = btn_wait_release();
      } while (s != 0);
    }

    if (status == EVT_BUTTON_SINGLE_CLICK) {
      if (keypad_click(selection))
        /* exit loop on done or cancel */
        break; 
    }

    status = touch_check();
    if (status == EVT_TOUCH_PRESSED) {
      if (keypad_apply_touch()
          && keypad_click(selection))
        /* exit loop on done or cancel */
        break; 
    }
  }

  ui_mode_normal();
  redraw();
  force_set_markmap();
  draw_cell_all();
}

void
ui_process_lever(void)
{
  switch (ui_mode) {
  case UI_NORMAL:
    ui_process_normal();
    break;    
  case UI_MENU:
    ui_process_menu();
    break;    
  case UI_KEYPAD:
    ui_process_keypad();
    break;    
  }
}

void ui_process_touch(void)
{
  extern int awd_count;
  awd_count++;
  adc_stop(ADC1);

  int status = touch_check();
  if (status == EVT_TOUCH_PRESSED) {
    switch (ui_mode) {
    case UI_NORMAL:
      touch_wait_release();
      ui_mode_menu();
      break;

    case UI_MENU:
      menu_apply_touch();
      break;
    }
  }
  touch_start_watchdog();
}

void
ui_process(void)
{
  switch (operation_requested) {
  case OP_LEVER:
    ui_process_lever();
    break;
  case OP_TOUCH:
    ui_process_touch();
    break;
  }
  operation_requested = OP_NONE;
}

/* Triggered when the button is pressed or released. The LED4 is set to ON.*/
static void extcb1(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;
  operation_requested = OP_LEVER;
  //cur_button = READ_PORT() & BUTTON_MASK;
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


int awd_count;
int touch_x, touch_y;

static const GPTConfig gpt3cfg = {
  1000,    /* 1kHz timer clock.*/
  NULL,   /* Timer callback.*/
  0x0020, /* CR2:MMS=02 to output TRGO */
  0
};

void
test_touch(int *x, int *y)
{
  adc_stop(ADC1);

  *x = touch_measure_x();
  *y = touch_measure_y();

  touch_start_watchdog();
}

void
handle_touch_interrupt(void)
{
  operation_requested = OP_TOUCH;
}

void
ui_init()
{
  adc_init();
  
  /*
   * Activates the EXT driver 1.
   */
  extStart(&EXTD1, &extcfg);

#if 1
  gptStart(&GPTD3, &gpt3cfg);
  gptPolledDelay(&GPTD3, 10); /* Small delay.*/

  gptStartContinuous(&GPTD3, 10);
#endif

  touch_start_watchdog();
}
