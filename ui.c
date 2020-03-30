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
#include "chprintf.h"
#include "nanovna.h"
//#include <stdlib.h>
#include <string.h>

uistat_t uistat = {
 digit: 6,
 current_trace: 0,
 lever_mode: LM_MARKER,
 marker_delta: FALSE,
 marker_tracking : FALSE,
};

#define NO_EVENT                    0
#define EVT_BUTTON_SINGLE_CLICK     0x01
#define EVT_BUTTON_DOUBLE_CLICK     0x02
#define EVT_BUTTON_DOWN_LONG        0x04
#define EVT_UP                  0x10
#define EVT_DOWN                0x20
#define EVT_REPEAT              0x40

#define BUTTON_DOWN_LONG_TICKS      5000   /* 1sec */
#define BUTTON_DOUBLE_TICKS         2500   /* 500ms */
#define BUTTON_REPEAT_TICKS         500    /* 100ms */
#define BUTTON_DEBOUNCE_TICKS       400    /* 80ms */

/* lever switch assignment */
#define BIT_UP1     3
#define BIT_PUSH    2
#define BIT_DOWN1   1

#define READ_PORT() palReadPort(GPIOA)
#define BUTTON_MASK 0b1111

static uint16_t last_button = 0b0000;
static systime_t last_button_down_ticks;
static systime_t last_button_repeat_ticks;

volatile uint8_t operation_requested = OP_NONE;

int8_t previous_marker = -1;

enum {
  UI_NORMAL, UI_MENU, UI_NUMERIC, UI_KEYPAD
};

enum {
  KM_START, KM_STOP, KM_CENTER, KM_SPAN, KM_CW, KM_SCALE, KM_REFPOS, KM_EDELAY, KM_VELOCITY_FACTOR, KM_SCALEDELAY
};

#define NUMINPUT_LEN 10

static uint8_t ui_mode = UI_NORMAL;
static uint8_t keypad_mode;
static uint8_t keypads_last_index;
static char    kp_buf[NUMINPUT_LEN+1];
static int8_t  kp_index = 0;
static uint8_t menu_current_level = 0;
static int8_t  selection = 0;

// Set structure align as WORD (save flash memory)
#pragma pack(push, 2)
typedef struct {
  uint8_t type;
  uint8_t data;
  char *label;
  const void *reference;
} menuitem_t;
#pragma pack(pop)

// Touch screen
#define EVT_TOUCH_NONE     0
#define EVT_TOUCH_DOWN     1
#define EVT_TOUCH_PRESSED  2
#define EVT_TOUCH_RELEASED 3

static int8_t last_touch_status = EVT_TOUCH_NONE;
static int16_t last_touch_x;
static int16_t last_touch_y;

//int16_t touch_cal[4] = { 1000, 1000, 10*16, 12*16 };
//int16_t touch_cal[4] = { 620, 600, 130, 180 };

//int awd_count;
//int touch_x, touch_y;

#define KP_CONTINUE 0
#define KP_DONE 1
#define KP_CANCEL 2

static void ui_mode_normal(void);
static void ui_mode_menu(void);
static void ui_mode_numeric(int _keypad_mode);
static void ui_mode_keypad(int _keypad_mode);
static void draw_menu(void);
static void leave_ui_mode(void);
static void erase_menu_buttons(void);
static void ui_process_keypad(void);
static void ui_process_numeric(void);

static void menu_move_back(void);
static void menu_push_submenu(const menuitem_t *submenu);

static int btn_check(void)
{
  systime_t ticks;
  // Debounce input
  while(TRUE){
    ticks = chVTGetSystemTimeX();
    if(ticks - last_button_down_ticks > BUTTON_DEBOUNCE_TICKS)
      break;
    chThdSleepMilliseconds(10);
  }
  int status = 0;
  uint16_t cur_button = READ_PORT() & BUTTON_MASK;
  // Detect only changed and pressed buttons
  uint16_t button_set = (last_button ^ cur_button) & cur_button;
  last_button_down_ticks = ticks;
  last_button = cur_button;

  if (button_set & (1<<BIT_PUSH))
    status |= EVT_BUTTON_SINGLE_CLICK;
  if (button_set & (1<<BIT_UP1))
    status |= EVT_UP;
  if (button_set & (1<<BIT_DOWN1))
    status |= EVT_DOWN;
  return status;
}

static int btn_wait_release(void)
{
  while (TRUE) {
    systime_t ticks = chVTGetSystemTimeX();
    systime_t dt = ticks - last_button_down_ticks;
    // Debounce input
    if (dt < BUTTON_DEBOUNCE_TICKS){
      chThdSleepMilliseconds(10);
      continue;
    }
    uint16_t cur_button = READ_PORT() & BUTTON_MASK;
    uint16_t changed = last_button ^ cur_button;
    if (dt >= BUTTON_DOWN_LONG_TICKS && (cur_button & (1<<BIT_PUSH)))
      return EVT_BUTTON_DOWN_LONG;
    else if (changed & (1<<BIT_PUSH)) // release
      return EVT_BUTTON_SINGLE_CLICK;

    if (changed) {
      // finished
      last_button = cur_button;
      last_button_down_ticks = ticks;
      return 0;
    }

    if (dt > BUTTON_DOWN_LONG_TICKS &&
        ticks > last_button_repeat_ticks) {
      int status = 0;
      if (cur_button & (1<<BIT_DOWN1))
        status |= EVT_DOWN | EVT_REPEAT;
      if (cur_button & (1<<BIT_UP1))
        status |= EVT_UP | EVT_REPEAT;
      last_button_repeat_ticks = ticks + BUTTON_REPEAT_TICKS;
      return status;
    }
  }
}

static int
touch_measure_y(void)
{
  int v;
  // open Y line
  palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_PULLDOWN );
  palSetPadMode(GPIOA, 7, PAL_MODE_INPUT_PULLDOWN );
  // drive low to high on X line
  palSetPadMode(GPIOB, 0, PAL_MODE_OUTPUT_PUSHPULL );
  palClearPad(GPIOB, 0);
  palSetPadMode(GPIOA, 6, PAL_MODE_OUTPUT_PUSHPULL );
  palSetPad(GPIOA, 6);

  chThdSleepMilliseconds(2);
  v = adc_single_read(ADC_CHSELR_CHSEL7);
  //chThdSleepMilliseconds(2);
  //v += adc_single_read(ADC1, ADC_CHSELR_CHSEL7);
  return v;
}

static int
touch_measure_x(void)
{
  int v;
  // open X line
  palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_PULLDOWN );
  palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_PULLDOWN );
  // drive low to high on Y line
  palSetPadMode(GPIOB, 1, PAL_MODE_OUTPUT_PUSHPULL );
  palSetPad(GPIOB, 1);
  palSetPadMode(GPIOA, 7, PAL_MODE_OUTPUT_PUSHPULL );
  palClearPad(GPIOA, 7);

  chThdSleepMilliseconds(2);
  v = adc_single_read(ADC_CHSELR_CHSEL6);
  //chThdSleepMilliseconds(2);
  //v += adc_single_read(ADC1, ADC_CHSELR_CHSEL6);
  return v;
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
  adc_start_analog_watchdogd(ADC_CHSELR_CHSEL7);
}

static int
touch_status(void)
{
  touch_prepare_sense();
  return adc_single_read(ADC_CHSELR_CHSEL7) > TOUCH_THRESHOLD;
}

static int
touch_check(void)
{
  int stat = touch_status();
  if (stat) {
    chThdSleepMilliseconds(10);
    int x = touch_measure_x();
    int y = touch_measure_y();
    if (touch_status()) {
      last_touch_x = x;
      last_touch_y = y;
    }
    touch_prepare_sense();
  }

  if (stat != last_touch_status) {
    last_touch_status = stat;
    return stat ? EVT_TOUCH_PRESSED : EVT_TOUCH_RELEASED;
  }
  return stat ? EVT_TOUCH_DOWN : EVT_TOUCH_NONE;
}

static inline void
touch_wait_release(void)
{
  while (touch_check() != EVT_TOUCH_RELEASED)
    ;
}

static inline void
touch_wait_pressed(void)
{
  while (touch_check() != EVT_TOUCH_PRESSED)
    ;
}

void
touch_cal_exec(void)
{
  int x1, x2, y1, y2;

  adc_stop();
  ili9341_set_foreground(DEFAULT_FG_COLOR);
  ili9341_set_background(DEFAULT_BG_COLOR);
  ili9341_clear_screen();
  ili9341_line(0, 0, 0, 32);
  ili9341_line(0, 0, 32, 0);
  ili9341_drawstring("TOUCH UPPER LEFT", 10, 10);

  touch_wait_release();
  x1 = last_touch_x;
  y1 = last_touch_y;

  ili9341_clear_screen();
  ili9341_line(LCD_WIDTH-1, LCD_HEIGHT-1, LCD_WIDTH-1, LCD_HEIGHT-32);
  ili9341_line(LCD_WIDTH-1, LCD_HEIGHT-1, LCD_WIDTH-32, LCD_HEIGHT-1);
  ili9341_drawstring("TOUCH LOWER RIGHT", LCD_HEIGHT-FONT_GET_HEIGHT-10, LCD_WIDTH-17*(FONT_WIDTH)-10);

  touch_wait_release();
  x2 = last_touch_x;
  y2 = last_touch_y;

  config.touch_cal[0] = x1;
  config.touch_cal[1] = y1;
  config.touch_cal[2] = (x2 - x1) * 16 / LCD_WIDTH;
  config.touch_cal[3] = (y2 - y1) * 16 / LCD_HEIGHT;

  //redraw_all();
  touch_start_watchdog();
}

void
touch_draw_test(void)
{
  int x0, y0;
  int x1, y1;
  
  adc_stop();

  ili9341_set_foreground(DEFAULT_FG_COLOR);
  ili9341_set_background(DEFAULT_BG_COLOR);
  ili9341_clear_screen();
  ili9341_drawstring("TOUCH TEST: DRAG PANEL", OFFSETX, 233);

  touch_wait_pressed();
  touch_position(&x0, &y0);

  do {
    touch_position(&x1, &y1);
    ili9341_line(x0, y0, x1, y1);
    x0 = x1;
    y0 = y1;
    chThdSleepMilliseconds(50);
  } while (touch_check() != EVT_TOUCH_RELEASED);

  touch_start_watchdog();
}


void
touch_position(int *x, int *y)
{
  *x = (last_touch_x - config.touch_cal[0]) * 16 / config.touch_cal[2];
  *y = (last_touch_y - config.touch_cal[1]) * 16 / config.touch_cal[3];
}

void
show_version(void)
{
  int x = 5, y = 5, i = 0;
  adc_stop();
  ili9341_set_foreground(DEFAULT_FG_COLOR);
  ili9341_set_background(DEFAULT_BG_COLOR);

  ili9341_clear_screen();
  uint16_t shift = 0b0000010000111110;
  ili9341_drawstring_size(info_about[i++], x , y, 4);
  while (info_about[i]) {
    do {shift>>=1; y+=5;} while (shift&1);
    ili9341_drawstring(info_about[i++], x, y+=5);
  }
  while (true) {
    if (touch_check() == EVT_TOUCH_PRESSED)
      break;
    if (btn_check() & EVT_BUTTON_SINGLE_CLICK)
      break;
  }

  touch_start_watchdog();
}

void
enter_dfu(void)
{
  adc_stop();

  int x = 5, y = 5;
  ili9341_set_foreground(DEFAULT_FG_COLOR);
  ili9341_set_background(DEFAULT_BG_COLOR);
  // leave a last message 
  ili9341_clear_screen();
  ili9341_drawstring("DFU: Device Firmware Update Mode", x, y += 10);
  ili9341_drawstring("To exit DFU mode, please reset device yourself.", x, y += 10);

  // see __early_init in ./NANOVNA_STM32_F072/board.c
  *((unsigned long *)BOOT_FROM_SYTEM_MEMORY_MAGIC_ADDRESS) = BOOT_FROM_SYTEM_MEMORY_MAGIC;
  NVIC_SystemReset();
}

static void
select_lever_mode(int mode)
{
  if (uistat.lever_mode != mode) {
    uistat.lever_mode = mode;
    redraw_request |= REDRAW_FREQUENCY | REDRAW_MARKER;
  }
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

typedef void (*menuaction_cb_t)(int item, uint8_t data);

static void
menu_calop_cb(int item, uint8_t data)
{
  cal_collect(data);
  selection = item+1;
  draw_cal_status();
  draw_menu();
}

static void
menu_caldone_cb(int item, uint8_t data)
{
  extern const menuitem_t menu_save[];
  //extern const menuitem_t menu_cal[];
  (void)item;
  (void)data;
  cal_done();
  draw_cal_status();
  menu_move_back();
  menu_push_submenu(menu_save);
}

static void
menu_cal2_cb(int item, uint8_t data)
{
  (void)data;
  switch (item) {
  case 2: // RESET
    cal_status = 0;
    break;
  case 3: // CORRECTION
    // toggle applying correction
    cal_status ^= CALSTAT_APPLY;
    break;
  }
  draw_menu();
  draw_cal_status();
  //menu_move_back();
}

static void
menu_recall_cb(int item, uint8_t data)
{
  (void)item;
  caldata_recall(data);
  menu_move_back();
  ui_mode_normal();
  update_grid();
  draw_cal_status();
}

static void
menu_config_cb(int item, uint8_t data)
{
  (void)data;
  switch (item) {
  case 0:
      touch_cal_exec();
      break;
  case 1:
      touch_draw_test();
      break;
  case 3:
      show_version();
      break;
  }
  redraw_frame();
  request_to_redraw_grid();
  draw_menu();
}

static void
menu_config_save_cb(int item, uint8_t data)
{
  (void)item;
  (void)data;
  config_save();
  menu_move_back();
  ui_mode_normal();
}

static void
menu_dfu_cb(int item, uint8_t data)
{
  (void)item;
  (void)data;
  enter_dfu();
}

static void
menu_save_cb(int item, uint8_t data)
{
  (void)item;
  if (caldata_save(data) == 0) {
    menu_move_back();
    ui_mode_normal();
    draw_cal_status();
  }
}

static void 
choose_active_trace(void)
{
  int i;
  if (trace[uistat.current_trace].enabled)
    // do nothing
    return;
  for (i = 0; i < TRACES_MAX; i++)
    if (trace[i].enabled) {
      uistat.current_trace = i;
      return;
    }
}

static void
menu_trace_cb(int item, uint8_t data)
{
  (void)item;
  if (trace[data].enabled) {
    if (data == uistat.current_trace) {
      // disable if active trace is selected
      trace[data].enabled = FALSE;
      choose_active_trace();
    } else {
      // make active selected trace
      uistat.current_trace = data;
    }
  } else {
    trace[data].enabled = TRUE;
    uistat.current_trace = data;
  }
  request_to_redraw_grid();
  draw_menu();
}

static void
menu_format_cb(int item, uint8_t data)
{
  (void)item;
  set_trace_type(uistat.current_trace, data);
  request_to_redraw_grid();
  ui_mode_normal();
  //redraw_all();
}

static void
menu_channel_cb(int item, uint8_t data)
{
  (void)item;
  set_trace_channel(uistat.current_trace, data);
  menu_move_back();
  ui_mode_normal();
}

static void
menu_transform_window_cb(int item, uint8_t data)
{
  (void)item;
  // TODO
  domain_mode = (domain_mode & ~TD_WINDOW) | data;
  ui_mode_normal();
}

static void
menu_transform_cb(int item, uint8_t data)
{
  (void)item;
  (void)data;
  domain_mode ^= DOMAIN_TIME;
  select_lever_mode(LM_MARKER);
  draw_frequencies();
  ui_mode_normal();
}

static void
menu_velocity_cb(int item, uint8_t data)
{
  (void)item;
  (void)data;
  if (btn_wait_release() & EVT_BUTTON_DOWN_LONG) {
      ui_mode_numeric(KM_VELOCITY_FACTOR);
      ui_process_numeric();
  } else {
      ui_mode_keypad(KM_VELOCITY_FACTOR);
      ui_process_keypad();
  }
}

static void
menu_transform_filter_cb(int item, uint8_t data)
{
  (void)item;
  domain_mode = (domain_mode & ~TD_FUNC) | data;
  ui_mode_normal();
}

static void
menu_bandwidth_cb(int item, uint8_t data)
{
  (void)item;
  bandwidth = data;
  draw_menu();
}

static void 
choose_active_marker(void)
{
  int i;
  for (i = 0; i < MARKERS_MAX; i++)
    if (markers[i].enabled) {
      active_marker = i;
      return;
    }
  active_marker = -1;
}

static void
menu_scale_cb(int item, uint8_t data)
{
  (void)item;
  if (data == KM_SCALE && trace[uistat.current_trace].type == TRC_DELAY) {
    data = KM_SCALEDELAY;
  }
  if (btn_wait_release() & EVT_BUTTON_DOWN_LONG) {
    ui_mode_numeric(data);
    ui_process_numeric();
  } else {
    ui_mode_keypad(data);
    ui_process_keypad();
  }
}

static void
menu_stimulus_cb(int item, uint8_t data)
{
  (void)data;
  switch (item) {
  case 0: /* START */
  case 1: /* STOP */
  case 2: /* CENTER */
  case 3: /* SPAN */
  case 4: /* CW */
    if (btn_wait_release() & EVT_BUTTON_DOWN_LONG) {
      ui_mode_numeric(item);
      ui_process_numeric();
    } else {
      ui_mode_keypad(item);
      ui_process_keypad();
    }
    break;
  case 5: /* PAUSE */
    toggle_sweep();
    //menu_move_back();
    //ui_mode_normal();
    draw_menu();
    break;
  }
}

static uint32_t
get_marker_frequency(int marker)
{
  if (marker < 0 || marker >= MARKERS_MAX)
    return 0;
  if (!markers[marker].enabled)
    return 0;
  return frequencies[markers[marker].index];
}

static void
menu_marker_op_cb(int item, uint8_t data)
{
  uint32_t freq = get_marker_frequency(active_marker);
  if (freq == 0)
    return; // no active marker

  switch (item) {
  case 0: /* MARKER->START */
  case 1: /* MARKER->STOP */
  case 2: /* MARKER->CENTER */
    set_sweep_frequency(data, freq);
    break;
  case 3: /* MARKERS->SPAN */
    {
      if (previous_marker == -1 || active_marker == previous_marker) {
        // if only 1 marker is active, keep center freq and make span the marker comes to the edge
        uint32_t center = get_sweep_frequency(ST_CENTER);
        uint32_t span = center > freq ? center - freq : freq - center;
        set_sweep_frequency(ST_SPAN, span * 2);
      } else {
        // if 2 or more marker active, set start and stop freq to each marker
        uint32_t freq2 = get_marker_frequency(previous_marker);
        if (freq2 == 0)
          return;
        if (freq > freq2) {
          freq2 = freq;
          freq = get_marker_frequency(previous_marker);
        }
        set_sweep_frequency(ST_START, freq);
        set_sweep_frequency(ST_STOP, freq2);
      }
    }
    break;
  case 4: /* MARKERS->EDELAY */
    { 
      if (uistat.current_trace == -1)
        break;
      float (*array)[2] = measured[trace[uistat.current_trace].channel];
      float v = groupdelay_from_array(markers[active_marker].index, array);
      set_electrical_delay(electrical_delay + (v / 1e-12));
    }
    break;
  }
  ui_mode_normal();
  draw_cal_status();
  //redraw_all();
}

static void
menu_marker_search_cb(int item, uint8_t data)
{
  (void)data;
  int i = -1;
  if (active_marker == -1)
    return;

  switch (item) {
  case 0: /* maximum */
  case 1: /* minimum */
    set_marker_search(item);
    i = marker_search();
    break;
  case 2: /* search Left */
    i = marker_search_left(markers[active_marker].index);
    break;
  case 3: /* search right */
    i = marker_search_right(markers[active_marker].index);
    break;
  case 4: /* tracking */
    uistat.marker_tracking = !uistat.marker_tracking;
    break;
  }
  if (i != -1)
    markers[active_marker].index = i;
  draw_menu();
  redraw_marker(active_marker);
  select_lever_mode(LM_SEARCH);
}

static void
menu_marker_smith_cb(int item, uint8_t data)
{
  (void)item;
  marker_smith_format = data;
  redraw_marker(active_marker);
  draw_menu();
}

static void
active_marker_select(int item)
{
  if (item == -1) {
    active_marker = previous_marker;
    previous_marker = -1;
    if (active_marker == -1) {
      choose_active_marker();
    }
  } else {
    if (previous_marker != active_marker)
      previous_marker = active_marker;
    active_marker = item;
  }
}

static void
menu_marker_sel_cb(int item, uint8_t data)
{
  (void)data;
  int t;
  if (item >= 0 && item < MARKERS_MAX) {
    if (markers[item].enabled) {
      if (item == active_marker) {
        // disable if active trace is selected
        markers[item].enabled = FALSE;
        active_marker_select(-1);
      } else {
        active_marker_select(item);
      }
    } else {
      markers[item].enabled = TRUE;
      active_marker_select(item);
    }
  } else if (item == 4) { /* all off */
      for (t = 0; t < MARKERS_MAX; t++)
        markers[t].enabled = FALSE;
      previous_marker = -1;
      active_marker = -1;      
  } else if (item == 5) { /* marker delta */
    uistat.marker_delta = !uistat.marker_delta;
  }
  redraw_marker(active_marker);
  draw_menu();
}

static const menuitem_t menu_calop[] = {
  { MT_CALLBACK, CAL_OPEN,  "OPEN",  menu_calop_cb },
  { MT_CALLBACK, CAL_SHORT, "SHORT", menu_calop_cb },
  { MT_CALLBACK, CAL_LOAD,  "LOAD",  menu_calop_cb },
  { MT_CALLBACK, CAL_ISOLN, "ISOLN", menu_calop_cb },
  { MT_CALLBACK, CAL_THRU,  "THRU",  menu_calop_cb },
  { MT_CALLBACK, 0,         "DONE",  menu_caldone_cb },
  { MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

const menuitem_t menu_save[] = {
  { MT_CALLBACK, 0, "SAVE 0", menu_save_cb },
  { MT_CALLBACK, 1, "SAVE 1", menu_save_cb },
  { MT_CALLBACK, 2, "SAVE 2", menu_save_cb },
  { MT_CALLBACK, 3, "SAVE 3", menu_save_cb },
  { MT_CALLBACK, 4, "SAVE 4", menu_save_cb },
  { MT_CANCEL, 0, S_LARROW" BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t menu_cal[] = {
  { MT_SUBMENU,  0, "CALIBRATE", menu_calop },
  { MT_SUBMENU,  0, "SAVE", menu_save },
  { MT_CALLBACK, 0, "RESET", menu_cal2_cb },
  { MT_CALLBACK, 0, "CORRECTION", menu_cal2_cb },
  { MT_CANCEL, 0, S_LARROW" BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t menu_trace[] = {
  { MT_CALLBACK, 0, "TRACE 0", menu_trace_cb },
  { MT_CALLBACK, 1, "TRACE 1", menu_trace_cb },
  { MT_CALLBACK, 2, "TRACE 2", menu_trace_cb },
  { MT_CALLBACK, 3, "TRACE 3", menu_trace_cb },
  { MT_CANCEL, 0, S_LARROW" BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t menu_format2[] = {
  { MT_CALLBACK, TRC_POLAR, "POLAR", menu_format_cb },
  { MT_CALLBACK, TRC_LINEAR, "LINEAR", menu_format_cb },
  { MT_CALLBACK, TRC_REAL, "REAL", menu_format_cb },
  { MT_CALLBACK, TRC_IMAG, "IMAG", menu_format_cb },
  { MT_CALLBACK, TRC_R, "RESISTANCE", menu_format_cb },
  { MT_CALLBACK, TRC_X, "REACTANCE", menu_format_cb },
  { MT_CANCEL, 0, S_LARROW" BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t menu_format[] = {
  { MT_CALLBACK, TRC_LOGMAG, "LOGMAG", menu_format_cb },
  { MT_CALLBACK, TRC_PHASE, "PHASE", menu_format_cb },
  { MT_CALLBACK, TRC_DELAY, "DELAY", menu_format_cb },
  { MT_CALLBACK, TRC_SMITH, "SMITH", menu_format_cb },
  { MT_CALLBACK, TRC_SWR, "SWR", menu_format_cb },
  { MT_SUBMENU, 0, S_RARROW" MORE", menu_format2 },
  //{ MT_CALLBACK, TRC_LINEAR, "LINEAR", menu_format_cb },
  //{ MT_CALLBACK, TRC_SWR, "SWR", menu_format_cb },
  { MT_CANCEL, 0, S_LARROW" BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t menu_scale[] = {
  { MT_CALLBACK, KM_SCALE, "SCALE/DIV", menu_scale_cb },
  { MT_CALLBACK, KM_REFPOS, "\2REFERENCE\0POSITION", menu_scale_cb },
  { MT_CALLBACK, KM_EDELAY, "\2ELECTRICAL\0DELAY", menu_scale_cb },
  { MT_CANCEL, 0, S_LARROW" BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t menu_channel[] = {
  { MT_CALLBACK, 0, "\2CH0\0REFLECT", menu_channel_cb },
  { MT_CALLBACK, 1, "\2CH1\0THROUGH", menu_channel_cb },
  { MT_CANCEL, 0, S_LARROW" BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t menu_transform_window[] = {
  { MT_CALLBACK, TD_WINDOW_MINIMUM, "MINIMUM", menu_transform_window_cb },
  { MT_CALLBACK, TD_WINDOW_NORMAL,   "NORMAL", menu_transform_window_cb },
  { MT_CALLBACK, TD_WINDOW_MAXIMUM, "MAXIMUM", menu_transform_window_cb },
  { MT_CANCEL, 0, S_LARROW" BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t menu_transform[] = {
  { MT_CALLBACK, 0, "\2TRANSFORM\0ON", menu_transform_cb },
  { MT_CALLBACK, TD_FUNC_LOWPASS_IMPULSE, "\2LOW PASS\0IMPULSE", menu_transform_filter_cb },
  { MT_CALLBACK, TD_FUNC_LOWPASS_STEP, "\2LOW PASS\0STEP", menu_transform_filter_cb },
  { MT_CALLBACK, TD_FUNC_BANDPASS, "BANDPASS", menu_transform_filter_cb },
  { MT_SUBMENU, 0, "WINDOW", menu_transform_window },
  { MT_CALLBACK, 0, "\2VELOCITY\0FACTOR", menu_velocity_cb },
  { MT_CANCEL, 0, S_LARROW" BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t menu_bandwidth[] = {
  { MT_CALLBACK, BANDWIDTH_1000, "1 kHz", menu_bandwidth_cb },
  { MT_CALLBACK, BANDWIDTH_300, "300 Hz", menu_bandwidth_cb },
  { MT_CALLBACK, BANDWIDTH_100, "100 Hz", menu_bandwidth_cb },
  { MT_CALLBACK, BANDWIDTH_30,   "30 Hz", menu_bandwidth_cb },
  { MT_CALLBACK, BANDWIDTH_10,   "10 Hz", menu_bandwidth_cb },
  { MT_CANCEL, 255, S_LARROW" BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t menu_display[] = {
  { MT_SUBMENU, 0, "TRACE", menu_trace },
  { MT_SUBMENU, 0, "FORMAT", menu_format },
  { MT_SUBMENU, 0, "SCALE", menu_scale },
  { MT_SUBMENU, 0, "CHANNEL", menu_channel },
  { MT_SUBMENU, 0, "TRANSFORM", menu_transform },
  { MT_SUBMENU, 0, "BANDWIDTH", menu_bandwidth },
  { MT_CANCEL, 0, S_LARROW" BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t menu_stimulus[] = {
  { MT_CALLBACK, 0, "START", menu_stimulus_cb },
  { MT_CALLBACK, 0, "STOP", menu_stimulus_cb },
  { MT_CALLBACK, 0, "CENTER", menu_stimulus_cb },
  { MT_CALLBACK, 0, "SPAN", menu_stimulus_cb },
  { MT_CALLBACK, 0, "CW FREQ", menu_stimulus_cb },
  { MT_CALLBACK, 0, "\2PAUSE\0SWEEP", menu_stimulus_cb },
  { MT_CANCEL, 0, S_LARROW" BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t menu_marker_sel[] = {
  { MT_CALLBACK, 1, "MARKER 1", menu_marker_sel_cb },
  { MT_CALLBACK, 2, "MARKER 2", menu_marker_sel_cb },
  { MT_CALLBACK, 3, "MARKER 3", menu_marker_sel_cb },
  { MT_CALLBACK, 4, "MARKER 4", menu_marker_sel_cb },
  { MT_CALLBACK, 0, "ALL OFF", menu_marker_sel_cb },
  { MT_CALLBACK, 0, "DELTA", menu_marker_sel_cb },
  { MT_CANCEL, 0, S_LARROW" BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t menu_marker_ops[] = {
  { MT_CALLBACK, ST_START, S_RARROW"START", menu_marker_op_cb },
  { MT_CALLBACK, ST_STOP, S_RARROW"STOP", menu_marker_op_cb },
  { MT_CALLBACK, ST_CENTER, S_RARROW"CENTER", menu_marker_op_cb },
  { MT_CALLBACK, ST_SPAN, S_RARROW"SPAN", menu_marker_op_cb },
  { MT_CALLBACK, 0, S_RARROW"EDELAY", menu_marker_op_cb },
  { MT_CANCEL, 0, S_LARROW" BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t menu_marker_search[] = {
  //{ MT_CALLBACK, "OFF", menu_marker_search_cb },
  { MT_CALLBACK, 0, "MAXIMUM", menu_marker_search_cb },
  { MT_CALLBACK, 0, "MINIMUM", menu_marker_search_cb },
  { MT_CALLBACK, 0, "\2SEARCH\0" S_LARROW" LEFT", menu_marker_search_cb },
  { MT_CALLBACK, 0, "\2SEARCH\0" S_RARROW" RIGHT", menu_marker_search_cb },
  { MT_CALLBACK, 0, "TRACKING", menu_marker_search_cb },
  { MT_CANCEL, 0, S_LARROW" BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t menu_marker_smith[] = {
  { MT_CALLBACK, MS_LIN, "LIN", menu_marker_smith_cb },
  { MT_CALLBACK, MS_LOG, "LOG", menu_marker_smith_cb },
  { MT_CALLBACK, MS_REIM,"Re+Im", menu_marker_smith_cb },
  { MT_CALLBACK, MS_RX,  "R+Xj", menu_marker_smith_cb },
  { MT_CALLBACK, MS_RLC, "R+L/C", menu_marker_smith_cb },
  { MT_CANCEL, 0, S_LARROW" BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t menu_marker[] = {
  { MT_SUBMENU, 0, "\2SELECT\0MARKER", menu_marker_sel },
  { MT_SUBMENU, 0, "SEARCH", menu_marker_search },
  { MT_SUBMENU, 0, "OPERATIONS", menu_marker_ops },
  { MT_SUBMENU, 0, "\2SMITH\0VALUE", menu_marker_smith },
  { MT_CANCEL, 0, S_LARROW" BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t menu_recall[] = {
  { MT_CALLBACK, 0, "RECALL 0", menu_recall_cb },
  { MT_CALLBACK, 1, "RECALL 1", menu_recall_cb },
  { MT_CALLBACK, 2, "RECALL 2", menu_recall_cb },
  { MT_CALLBACK, 3, "RECALL 3", menu_recall_cb },
  { MT_CALLBACK, 4, "RECALL 4", menu_recall_cb },
  { MT_CANCEL, 0, S_LARROW" BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t menu_dfu[] = {
  { MT_CALLBACK, 0, "\2RESET AND\0ENTER DFU", menu_dfu_cb },
  { MT_CANCEL, 0, S_LARROW"CANCEL", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t menu_config[] = {
  { MT_CALLBACK, 0, "TOUCH CAL", menu_config_cb },
  { MT_CALLBACK, 0, "TOUCH TEST", menu_config_cb },
  { MT_CALLBACK, 0, "SAVE", menu_config_save_cb },
  { MT_CALLBACK, 0, "VERSION", menu_config_cb },
  { MT_SUBMENU, 0, S_RARROW"DFU", menu_dfu },
  { MT_CANCEL, 0, S_LARROW" BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t menu_top[] = {
  { MT_SUBMENU, 0, "DISPLAY", menu_display },
  { MT_SUBMENU, 0, "MARKER", menu_marker },
  { MT_SUBMENU, 0, "STIMULUS", menu_stimulus },
  { MT_SUBMENU, 0, "CAL", menu_cal },
  { MT_SUBMENU, 0, "RECALL", menu_recall },
  { MT_SUBMENU, 0, "CONFIG", menu_config },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

#define MENU_STACK_DEPTH_MAX 4
const menuitem_t *menu_stack[MENU_STACK_DEPTH_MAX] = {
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

static void
menu_move_back(void)
{
  if (menu_current_level == 0)
    return;
  menu_current_level--;
  ensure_selection();
  erase_menu_buttons();
  draw_menu();
}

static void
menu_push_submenu(const menuitem_t *submenu)
{
  if (menu_current_level < MENU_STACK_DEPTH_MAX-1)
    menu_current_level++;
  menu_stack[menu_current_level] = submenu;
  ensure_selection();
  erase_menu_buttons();
  draw_menu();
}

/*
static void
menu_move_top(void)
{
  if (menu_current_level == 0)
    return;
  menu_current_level = 0;
  ensure_selection();
  erase_menu_buttons();
  draw_menu();
}
*/

static void
menu_invoke(int item)
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
    (*cb)(item, menu->data);
    break;
  }

  case MT_SUBMENU:
    menu_push_submenu((const menuitem_t*)menu->reference);
    break;
  }
}

// Maximum menu buttons count
#define MENU_BUTTON_MAX     7
// Menu buttons size
#define MENU_BUTTON_WIDTH  60
#define MENU_BUTTON_HEIGHT 30
// Height of numerical input field (at bottom)
#define NUM_INPUT_HEIGHT   30

#define KP_WIDTH     48
#define KP_HEIGHT    48
// Key x, y position (0 - 15) on screen
#define KP_GET_X(posx) ((posx)*KP_WIDTH + (LCD_WIDTH-64-KP_WIDTH*4))
#define KP_GET_Y(posy) ((posy)*KP_HEIGHT + 12 )

// Key names
#define KP_0          0
#define KP_1          1
#define KP_2          2
#define KP_3          3
#define KP_4          4
#define KP_5          5
#define KP_6          6
#define KP_7          7
#define KP_8          8
#define KP_9          9
#define KP_PERIOD    10
#define KP_MINUS     11
#define KP_X1        12
#define KP_K         13
#define KP_M         14
#define KP_G         15
#define KP_BS        16
#define KP_INF       17
#define KP_DB        18
#define KP_PLUSMINUS 19
#define KP_KEYPAD    20
#define KP_N         21
#define KP_P         22

typedef struct {
  uint8_t x:4;
  uint8_t y:4;
  int8_t  c;
} keypads_t;

static const keypads_t *keypads;

static const keypads_t keypads_freq[] = {
  { 1, 3, KP_PERIOD },
  { 0, 3, KP_0 },
  { 0, 2, KP_1 },
  { 1, 2, KP_2 },
  { 2, 2, KP_3 },
  { 0, 1, KP_4 },
  { 1, 1, KP_5 },
  { 2, 1, KP_6 },
  { 0, 0, KP_7 },
  { 1, 0, KP_8 },
  { 2, 0, KP_9 },
  { 3, 0, KP_G },
  { 3, 1, KP_M },
  { 3, 2, KP_K },
  { 3, 3, KP_X1 },
  { 2, 3, KP_BS },
  { 0, 0, -1 }
};

static const keypads_t keypads_scale[] = {
  { 1, 3, KP_PERIOD },
  { 0, 3, KP_0 },
  { 0, 2, KP_1 },
  { 1, 2, KP_2 },
  { 2, 2, KP_3 },
  { 0, 1, KP_4 },
  { 1, 1, KP_5 },
  { 2, 1, KP_6 },
  { 0, 0, KP_7 },
  { 1, 0, KP_8 },
  { 2, 0, KP_9 },
  { 3, 3, KP_X1 },
  { 2, 3, KP_BS },
  { 0, 0, -1 }
};

static const keypads_t keypads_time[] = {
  { 1, 3, KP_PERIOD },
  { 0, 3, KP_0 },
  { 0, 2, KP_1 },
  { 1, 2, KP_2 },
  { 2, 2, KP_3 },
  { 0, 1, KP_4 },
  { 1, 1, KP_5 },
  { 2, 1, KP_6 },
  { 0, 0, KP_7 },
  { 1, 0, KP_8 },
  { 2, 0, KP_9 },
  { 3, 1, KP_N },
  { 3, 2, KP_P },
  { 3, 3, KP_MINUS },
  { 2, 3, KP_BS },
  { 0, 0, -1 }
};

static const keypads_t * const keypads_mode_tbl[] = {
  keypads_freq, // start
  keypads_freq, // stop
  keypads_freq, // center
  keypads_freq, // span
  keypads_freq, // cw freq
  keypads_scale, // scale
  keypads_scale, // refpos
  keypads_time, // electrical delay
  keypads_scale, // velocity factor
  keypads_time // scale of delay
};

static const char * const keypad_mode_label[] = {
  "START", "STOP", "CENTER", "SPAN", "CW FREQ", "SCALE", "REFPOS", "EDELAY", "VELOCITY%", "DELAY"
};

static void
draw_keypad(void)
{
  int i = 0;
  while (keypads[i].c >= 0) {
    uint16_t bg = config.menu_normal_color;
    if (i == selection)
      bg = config.menu_active_color;
    ili9341_set_foreground(DEFAULT_MENU_TEXT_COLOR);
    ili9341_set_background(bg);
    int x = KP_GET_X(keypads[i].x);
    int y = KP_GET_Y(keypads[i].y);
    ili9341_fill(x+2, y+2, KP_WIDTH-4, KP_HEIGHT-4, bg);
    ili9341_drawfont(keypads[i].c,
                     x + (KP_WIDTH - NUM_FONT_GET_WIDTH) / 2,
                     y + (KP_HEIGHT - NUM_FONT_GET_HEIGHT) / 2);
    i++;
  }
}

static void
draw_numeric_area_frame(void)
{
  ili9341_fill(0, LCD_HEIGHT-NUM_INPUT_HEIGHT, LCD_WIDTH, NUM_INPUT_HEIGHT, config.menu_normal_color);
  ili9341_set_foreground(DEFAULT_MENU_TEXT_COLOR);
  ili9341_set_background(config.menu_normal_color);
  ili9341_drawstring(keypad_mode_label[keypad_mode], 10, LCD_HEIGHT-(FONT_GET_HEIGHT+NUM_INPUT_HEIGHT)/2);
  //ili9341_drawfont(KP_KEYPAD, 300, 216);
}

static void
draw_numeric_input(const char *buf)
{
  int i;
  int x;
  int focused = FALSE;
  uint16_t xsim = 0b0010010000000000;

  for (i = 0, x = 64; i < 10 && buf[i]; i++, xsim<<=1) {
    uint16_t fg = DEFAULT_MENU_TEXT_COLOR;
    uint16_t bg = config.menu_normal_color;
    int c = buf[i];
    if (c == '.')
      c = KP_PERIOD;
    else if (c == '-')
      c = KP_MINUS;
    else// if (c >= '0' && c <= '9')
      c = c - '0';

    if (ui_mode == UI_NUMERIC && uistat.digit == 8-i) {
      fg = DEFAULT_SPEC_INPUT_COLOR;
      focused = TRUE;
//      if (uistat.digit_mode)
//        bg = DEFAULT_MENU_COLOR;
    }
    ili9341_set_foreground(fg);
    ili9341_set_background(bg);
    if (c >= 0) // c is number
      ili9341_drawfont(c, x, LCD_HEIGHT-NUM_INPUT_HEIGHT+4);
    else if (focused) // c not number, but focused
      ili9341_drawfont(0, x, LCD_HEIGHT-NUM_INPUT_HEIGHT+4);
    else // erase
      ili9341_fill(x, LCD_HEIGHT-NUM_INPUT_HEIGHT+4, NUM_FONT_GET_HEIGHT, NUM_FONT_GET_WIDTH+2+8, bg);

    x += xsim&0x8000 ? NUM_FONT_GET_WIDTH+2+8 : NUM_FONT_GET_WIDTH+2;
  }
  // erase last
  ili9341_fill(x, LCD_HEIGHT-NUM_INPUT_HEIGHT+4, NUM_FONT_GET_WIDTH+2+8, NUM_FONT_GET_WIDTH+2+8, config.menu_normal_color);
}

static int
menu_is_multiline(const char *label, const char **l1, const char **l2)
{
  if (label[0] != '\2')
    return FALSE;

  *l1 = &label[1];
  *l2 = &label[1] + strlen(&label[1]) + 1;
  return TRUE;
}

static void
menu_item_modify_attribute(const menuitem_t *menu, int item,
                           uint16_t *fg, uint16_t *bg)
{
  if (menu == menu_trace && item < TRACES_MAX) {
    if (trace[item].enabled)
      *bg = config.trace_color[item];
  } else if (menu == menu_marker_sel) {
    if (item < 4) {
      if (markers[item].enabled) {
        *bg = DEFAULT_MENU_TEXT_COLOR;
        *fg = config.menu_normal_color;
      }
    } else if (item == 5) {
      if (uistat.marker_delta) {
        *bg = DEFAULT_MENU_TEXT_COLOR;
        *fg = config.menu_normal_color;
      }
    }
  } else if (menu == menu_marker_search) {
    if (item == 4 && uistat.marker_tracking) {
      *bg = DEFAULT_MENU_TEXT_COLOR;
      *fg = config.menu_normal_color;
    }
  } else if (menu == menu_marker_smith) {

    if (marker_smith_format == item) {
      *bg = DEFAULT_MENU_TEXT_COLOR;
      *fg = config.menu_normal_color;
    }
  } else if (menu == menu_calop) {
    if ((item == 0 && (cal_status & CALSTAT_OPEN))
        || (item == 1 && (cal_status & CALSTAT_SHORT))
        || (item == 2 && (cal_status & CALSTAT_LOAD))
        || (item == 3 && (cal_status & CALSTAT_ISOLN))
        || (item == 4 && (cal_status & CALSTAT_THRU))) {
      domain_mode = (domain_mode & ~DOMAIN_MODE) | DOMAIN_FREQ;
      *bg = DEFAULT_MENU_TEXT_COLOR;
      *fg = config.menu_normal_color;
    }
  } else if (menu == menu_stimulus) {
    if (item == 5 /* PAUSE */ && !(sweep_mode&SWEEP_ENABLE)) {
      *bg = DEFAULT_MENU_TEXT_COLOR;
      *fg = config.menu_normal_color;
    }
  } else if (menu == menu_cal) {
    if (item == 3 /* CORRECTION */ && (cal_status & CALSTAT_APPLY)) {
      *bg = DEFAULT_MENU_TEXT_COLOR;
      *fg = config.menu_normal_color;
    }
  } else if (menu == menu_bandwidth) {
    if (menu_bandwidth[item].data == bandwidth) {
       *bg = 0x0000;
       *fg = 0xffff;
     }
  } else if (menu == menu_transform) {
      if ((item == 0 && (domain_mode & DOMAIN_MODE) == DOMAIN_TIME)
       || (item == 1 && (domain_mode & TD_FUNC) == TD_FUNC_LOWPASS_IMPULSE)
       || (item == 2 && (domain_mode & TD_FUNC) == TD_FUNC_LOWPASS_STEP)
       || (item == 3 && (domain_mode & TD_FUNC) == TD_FUNC_BANDPASS)
       ) {
        *bg = DEFAULT_MENU_TEXT_COLOR;
        *fg = config.menu_normal_color;
      }
  } else if (menu == menu_transform_window) {
      if ((item == 0 && (domain_mode & TD_WINDOW) == TD_WINDOW_MINIMUM)
       || (item == 1 && (domain_mode & TD_WINDOW) == TD_WINDOW_NORMAL)
       || (item == 2 && (domain_mode & TD_WINDOW) == TD_WINDOW_MAXIMUM)
       ) {
        *bg = DEFAULT_MENU_TEXT_COLOR;
        *fg = config.menu_normal_color;
      }
  }
}

static void
draw_menu_buttons(const menuitem_t *menu)
{
  int i = 0;
  for (i = 0; i < MENU_BUTTON_MAX; i++) {
    const char *l1, *l2;
    if (menu[i].type == MT_NONE)
      break;
    if (menu[i].type == MT_BLANK)
      continue;
    int y = MENU_BUTTON_HEIGHT*i;
    uint16_t bg = config.menu_normal_color;
    uint16_t fg = DEFAULT_MENU_TEXT_COLOR;
    // focus only in MENU mode but not in KEYPAD mode
    if (ui_mode == UI_MENU && i == selection)
      bg = config.menu_active_color;
    ili9341_fill(LCD_WIDTH-MENU_BUTTON_WIDTH, y, MENU_BUTTON_WIDTH, MENU_BUTTON_HEIGHT-2, bg);
    
    menu_item_modify_attribute(menu, i, &fg, &bg);
    ili9341_set_foreground(fg);
    ili9341_set_background(bg);
    if (menu_is_multiline(menu[i].label, &l1, &l2)) {
      ili9341_fill(LCD_WIDTH-MENU_BUTTON_WIDTH+3, y+5, MENU_BUTTON_WIDTH-6, 2+FONT_GET_HEIGHT+1+FONT_GET_HEIGHT+2, bg);
      ili9341_drawstring(l1, LCD_WIDTH-MENU_BUTTON_WIDTH+5, y+7);
      ili9341_drawstring(l2, LCD_WIDTH-MENU_BUTTON_WIDTH+5, y+7+FONT_GET_HEIGHT+1);
    } else {
      ili9341_fill(LCD_WIDTH-MENU_BUTTON_WIDTH+3, y+8, MENU_BUTTON_WIDTH-6, 2+FONT_GET_HEIGHT+2, bg);
      ili9341_drawstring(menu[i].label, LCD_WIDTH-MENU_BUTTON_WIDTH+5, y+10);
    }
  }
}

static void
menu_select_touch(int i)
{
  selection = i;
  draw_menu();
  touch_wait_release();
  selection = -1;
  menu_invoke(i);
}

static void
menu_apply_touch(void)
{
  int touch_x, touch_y;
  const menuitem_t *menu = menu_stack[menu_current_level];
  int i;

  touch_position(&touch_x, &touch_y);
  for (i = 0; i < MENU_BUTTON_MAX; i++) {
    if (menu[i].type == MT_NONE)
      break;
    if (menu[i].type == MT_BLANK)
      continue;
    int y = MENU_BUTTON_HEIGHT*i;
    if (y < touch_y && touch_y < y+MENU_BUTTON_HEIGHT && LCD_WIDTH-MENU_BUTTON_WIDTH < touch_x) {
      menu_select_touch(i);
      return;
    }
  }

  touch_wait_release();
  ui_mode_normal();
}

static void
draw_menu(void)
{
  draw_menu_buttons(menu_stack[menu_current_level]);
}

static void
erase_menu_buttons(void)
{
  ili9341_fill(LCD_WIDTH-MENU_BUTTON_WIDTH, 0, MENU_BUTTON_WIDTH, MENU_BUTTON_HEIGHT*MENU_BUTTON_MAX, DEFAULT_BG_COLOR);
}

static void
erase_numeric_input(void)
{
  ili9341_fill(0, LCD_HEIGHT-NUM_INPUT_HEIGHT, LCD_WIDTH, NUM_INPUT_HEIGHT, DEFAULT_BG_COLOR);
}

static void
leave_ui_mode()
{
  if (ui_mode == UI_MENU) {
    request_to_draw_cells_behind_menu();
    erase_menu_buttons();
  } else if (ui_mode == UI_NUMERIC) {
    request_to_draw_cells_behind_numeric_input();
    erase_numeric_input();
    draw_frequencies();
  }
}

static void
fetch_numeric_target(void)
{
  switch (keypad_mode) {
  case KM_START:
    uistat.value = get_sweep_frequency(ST_START);
    break;
  case KM_STOP:
    uistat.value = get_sweep_frequency(ST_STOP);
    break;
  case KM_CENTER:
    uistat.value = get_sweep_frequency(ST_CENTER);
    break;
  case KM_SPAN:
    uistat.value = get_sweep_frequency(ST_SPAN);
    break;
  case KM_CW:
    uistat.value = get_sweep_frequency(ST_CW);
    break;
  case KM_SCALE:
    uistat.value = get_trace_scale(uistat.current_trace) * 1000;
    break;
  case KM_REFPOS:
    uistat.value = get_trace_refpos(uistat.current_trace) * 1000;
    break;
  case KM_EDELAY:
    uistat.value = get_electrical_delay();
    break;
  case KM_VELOCITY_FACTOR:
    uistat.value = velocity_factor * 100;
    break;
  case KM_SCALEDELAY:
    uistat.value = get_trace_scale(uistat.current_trace) * 1e12;
    break;
  }

  {
    uint32_t x = uistat.value;
    int n = 0;
    for (; x >= 10 && n < 9; n++)
      x /= 10;
    uistat.digit = n;
  }
//  uistat.previous_value = uistat.value;
}

static void
set_numeric_value(void)
{
  switch (keypad_mode) {
  case KM_START:
    set_sweep_frequency(ST_START, uistat.value);
    break;
  case KM_STOP:
    set_sweep_frequency(ST_STOP, uistat.value);
    break;
  case KM_CENTER:
    set_sweep_frequency(ST_CENTER, uistat.value);
    break;
  case KM_SPAN:
    set_sweep_frequency(ST_SPAN, uistat.value);
    break;
  case KM_CW:
    set_sweep_frequency(ST_CW, uistat.value);
    break;
  case KM_SCALE:
    set_trace_scale(uistat.current_trace, uistat.value / 1000.0);
    break;
  case KM_REFPOS:
    set_trace_refpos(uistat.current_trace, uistat.value / 1000.0);
    break;
  case KM_EDELAY:
    set_electrical_delay(uistat.value);
    break;
  case KM_VELOCITY_FACTOR:
    velocity_factor = uistat.value/100.0;
    break;
  }
}

static void
draw_numeric_area(void)
{
  char buf[10];
  plot_printf(buf, sizeof buf, "%9d", uistat.value);
  draw_numeric_input(buf);
}

static void
ui_mode_menu(void)
{
  if (ui_mode == UI_MENU)
    return;

  ui_mode = UI_MENU;
  /* narrowen plotting area */
  area_width  = AREA_WIDTH_NORMAL - MENU_BUTTON_WIDTH;
  area_height = AREA_HEIGHT_NORMAL;
  ensure_selection();
  draw_menu();
}

static void
ui_mode_numeric(int _keypad_mode)
{
  if (ui_mode == UI_NUMERIC)
    return;

  leave_ui_mode();

  // keypads array
  keypad_mode = _keypad_mode;
  ui_mode = UI_NUMERIC;
  area_width = AREA_WIDTH_NORMAL;
  area_height = LCD_HEIGHT-NUM_INPUT_HEIGHT;//AREA_HEIGHT_NORMAL - 32;

  draw_numeric_area_frame();
  fetch_numeric_target();
  draw_numeric_area();
}

static void
ui_mode_keypad(int _keypad_mode)
{
  if (ui_mode == UI_KEYPAD)
    return;

  // keypads array
  keypad_mode = _keypad_mode;
  keypads = keypads_mode_tbl[_keypad_mode];
  int i;
  for (i = 0; keypads[i+1].c >= 0; i++)
    ;
  keypads_last_index = i;

  ui_mode = UI_KEYPAD;
  area_width = AREA_WIDTH_NORMAL - MENU_BUTTON_WIDTH;
  area_height = HEIGHT - 32;
  draw_menu();
  draw_keypad();
  draw_numeric_area_frame();
  draw_numeric_input("");
}

static void
ui_mode_normal(void)
{
  if (ui_mode == UI_NORMAL)
    return;

  area_width  = AREA_WIDTH_NORMAL;
  area_height = AREA_HEIGHT_NORMAL;
  leave_ui_mode();
  ui_mode = UI_NORMAL;
}

static void
lever_move_marker(int status)
{
  do {
    if (active_marker >= 0 && markers[active_marker].enabled) {
      if ((status & EVT_DOWN) && markers[active_marker].index > 0) {
        markers[active_marker].index--;
      }
      if ((status & EVT_UP) && markers[active_marker].index < sweep_points-1) {
        markers[active_marker].index++;
      }
      markers[active_marker].frequency = frequencies[markers[active_marker].index];
      redraw_marker(active_marker);
    }
    status = btn_wait_release();
  } while (status != 0);
}

static void
lever_search_marker(int status)
{
  int i = -1;
  if (active_marker >= 0) {
    if (status & EVT_DOWN)
      i = marker_search_left(markers[active_marker].index);
    else if (status & EVT_UP)
      i = marker_search_right(markers[active_marker].index);
    if (i != -1){
      markers[active_marker].index = i;
      redraw_marker(active_marker);
    }
  }
}

// ex. 10942 -> 10000
//      6791 ->  5000
//       341 ->   200
static uint32_t
step_round(uint32_t v)
{
  // decade step
  uint32_t x = 1;
  for (x = 1; x*10 < v; x*= 10)
    ;

  // 1-2-5 step
  if (x * 2 > v)
    return x;
  else if (x * 5 > v)
    return x * 2;
  else
    return x * 5;
}

static void
lever_zoom_span(int status)
{
  uint32_t span = get_sweep_frequency(ST_SPAN);
  if (status & EVT_UP) {
    span = step_round(span - 1);
  } else if (status & EVT_DOWN) {
    span = step_round(span + 1);
    span = step_round(span * 3);
  }
  set_sweep_frequency(ST_SPAN, span);
}

static void
lever_move(int status, int mode)
{
  uint32_t center = get_sweep_frequency(mode);
  uint32_t span = get_sweep_frequency(ST_SPAN);
  span = step_round(span / 3);
  if (status & EVT_UP) {
    set_sweep_frequency(mode, center + span);
  } else if (status & EVT_DOWN) {
    set_sweep_frequency(mode, center - span);
  }
}

#define STEPRATIO 0.2

static void
lever_edelay(int status)
{
  float value = get_electrical_delay();
  float ratio = STEPRATIO;
  if (value < 0)
    ratio = -ratio;
  if (status & EVT_UP) {
    value = (1 - ratio) * value;
  } else if (status & EVT_DOWN) {
    value = (1 + ratio) * value;
  }
  set_electrical_delay(value);
}

static void
ui_process_normal(void)
{
  int status = btn_check();
  if (status != 0) {
    if (status & EVT_BUTTON_SINGLE_CLICK) {
      ui_mode_menu();
    } else {
    switch (uistat.lever_mode) {
      case LM_MARKER: lever_move_marker(status);   break;
      case LM_SEARCH: lever_search_marker(status); break;
      case LM_CENTER:
        lever_move(status, FREQ_IS_STARTSTOP() ? ST_START : ST_CENTER);
        break;
      case LM_SPAN:
        if (FREQ_IS_STARTSTOP())
          lever_move(status, ST_STOP);
        else
        lever_zoom_span(status);
        break;
      case LM_EDELAY:
        lever_edelay(status);
        break;
      }
    }
  }
}

static void
ui_process_menu(void)
{
  int status = btn_check();
  if (status != 0) {
    if (status & EVT_BUTTON_SINGLE_CLICK) {
      menu_invoke(selection);
    } else {
      do {
        if (status & EVT_UP) {
          // close menu if next item is sentinel
          if (menu_stack[menu_current_level][selection+1].type == MT_NONE)
            goto menuclose;
          selection++;
        }
        if (status & EVT_DOWN) {
          if (selection == 0)
            goto menuclose;
          selection--;
        }
        draw_menu();
        status = btn_wait_release();
      } while (status != 0);
    }
  }
  return;

menuclose:
  ui_mode_normal();
}

static int
keypad_click(int key)
{
  int c = keypads[key].c;
  if ((c >= KP_X1 && c <= KP_G) || c == KP_N || c == KP_P) {
    int32_t scale = 1;
    if (c >= KP_X1 && c <= KP_G) {
      int n = c - KP_X1;
      while (n-- > 0)
        scale *= 1000;
    } else if (c == KP_N) {
      scale *= 1000;
    }
    /* numeric input done */
    double value = my_atof(kp_buf) * scale;
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
    case KM_REFPOS:
      set_trace_refpos(uistat.current_trace, value);
      break;
    case KM_EDELAY:
      set_electrical_delay(value); // pico seconds
      break;
    case KM_VELOCITY_FACTOR:
      velocity_factor = value / 100.0;
      break;
    case KM_SCALEDELAY:
      set_trace_scale(uistat.current_trace, value * 1e-12); // pico second
      break;
    }

    return KP_DONE;
  } else if (c <= 9 && kp_index < NUMINPUT_LEN) {
    kp_buf[kp_index++] = '0' + c;
  } else if (c == KP_PERIOD && kp_index < NUMINPUT_LEN) {
    // check period in former input
    int j;
    for (j = 0; j < kp_index && kp_buf[j] != '.'; j++)
      ;
    // append period if there are no period
    if (kp_index == j)
      kp_buf[kp_index++] = '.';
  } else if (c == KP_MINUS) {
    if (kp_index == 0)
      kp_buf[kp_index++] = '-';
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

static int
keypad_apply_touch(void)
{
  int touch_x, touch_y;
  int i = 0;

  touch_position(&touch_x, &touch_y);

  while (keypads[i].c >= 0) {
    int x = KP_GET_X(keypads[i].x);
    int y = KP_GET_Y(keypads[i].y);
    if (x < touch_x && touch_x < x+KP_WIDTH && y < touch_y && touch_y < y+KP_HEIGHT) {
      // draw focus
      selection = i;
      draw_keypad();
      touch_wait_release();
      // erase focus
      selection = -1;
      draw_keypad();
      return i;
    }
    i++;
  }
  return -1;
}

static void
numeric_apply_touch(void)
{
  int touch_x, touch_y;
  touch_position(&touch_x, &touch_y);

  if (touch_x < 64) {
    ui_mode_normal();
    return;
  }
  if (touch_x > 64+9*20+8+8) {
    ui_mode_keypad(keypad_mode);
    ui_process_keypad();
    return;
  }

  if (touch_y > LCD_HEIGHT-40) {
    int n = 9 - (touch_x - 64) / 20;
    uistat.digit = n;
    uistat.digit_mode = TRUE;
  } else {
    int step, n;
    if (touch_y < 100) {
      step = 1;
    } else {
      step = -1;
    }

    for (n = uistat.digit; n > 0; n--)
      step *= 10;
    uistat.value += step;
  }
  draw_numeric_area();

  touch_wait_release();
  uistat.digit_mode = FALSE;
  draw_numeric_area();

  return;
}

static void
ui_process_numeric(void)
{
  int status = btn_check();

  if (status != 0) {
    if (status == EVT_BUTTON_SINGLE_CLICK) {
      status = btn_wait_release();
      if (uistat.digit_mode) {
        if (status & (EVT_BUTTON_SINGLE_CLICK | EVT_BUTTON_DOWN_LONG)) {
          uistat.digit_mode = FALSE;
          draw_numeric_area();
        }
      } else {
        if (status & EVT_BUTTON_DOWN_LONG) {
          uistat.digit_mode = TRUE;
          draw_numeric_area();
        } else if (status & EVT_BUTTON_SINGLE_CLICK) {
          set_numeric_value();
          ui_mode_normal();
        }
      }
    } else {
      do {
        if (uistat.digit_mode) {
          if (status & EVT_DOWN) {
            if (uistat.digit < 8)
              uistat.digit++;
            else
              goto exit;
          }
          if (status & EVT_UP) {
            if (uistat.digit > 0)
              uistat.digit--;
            else
              goto exit;
          }
        } else {
          int32_t step = 1;
          int n;
          for (n = uistat.digit; n > 0; n--)
            step *= 10;
          if (status & EVT_DOWN)
            uistat.value += step;
          if (status & EVT_UP)
            uistat.value -= step;
        }
        draw_numeric_area();
        status = btn_wait_release();
      } while (status != 0);
    }
  }

  return;

 exit:
  // cancel operation
  ui_mode_normal();
}

static void
ui_process_keypad(void)
{
  int status;
  adc_stop();

  kp_index = 0;
  while (TRUE) {
    status = btn_check();
    if (status & (EVT_UP|EVT_DOWN)) {
      int s = status;
      do {
        if (s & EVT_UP)
          if (--selection < 0)
            selection = keypads_last_index;
        if (s & EVT_DOWN)
          if (++selection > keypads_last_index)
            selection = 0;
        draw_keypad();
        s = btn_wait_release();
      } while (s != 0);
    }

    if (status == EVT_BUTTON_SINGLE_CLICK) {
      if (keypad_click(selection))
        /* exit loop on done or cancel */
        break;
    }

    if (touch_check() == EVT_TOUCH_PRESSED) {
      int key = keypad_apply_touch();
      if (key >= 0 && keypad_click(key))
        /* exit loop on done or cancel */
        break;
    }
  }

  redraw_frame();
  request_to_redraw_grid();
  ui_mode_normal();
  //redraw_all();
  touch_start_watchdog();
}

static void
ui_process_lever(void)
{
  switch (ui_mode) {
  case UI_NORMAL:
    ui_process_normal();
    break;
  case UI_MENU:
    ui_process_menu();
    break;
  case UI_NUMERIC:
    ui_process_numeric();
    break;
  case UI_KEYPAD:
    ui_process_keypad();
    break;
  }
}

static void
drag_marker(int t, int m)
{
  /* wait touch release */
  do {
    int touch_x, touch_y;
    int index;
    touch_position(&touch_x, &touch_y);
    touch_x -= OFFSETX;
    touch_y -= OFFSETY;
    index = search_nearest_index(touch_x, touch_y, t);
    if (index >= 0) {
      markers[m].index = index;
      markers[m].frequency = frequencies[index];
      redraw_marker(m);
    }
  } while (touch_check()!= EVT_TOUCH_RELEASED);
}

static int
touch_pickup_marker(void)
{
  int touch_x, touch_y;
  int m, t;
  touch_position(&touch_x, &touch_y);
  touch_x -= OFFSETX;
  touch_y -= OFFSETY;

  for (m = 0; m < MARKERS_MAX; m++) {
    if (!markers[m].enabled)
      continue;

    for (t = 0; t < TRACES_MAX; t++) {
      int x, y;
      if (!trace[t].enabled)
        continue;

      marker_position(m, t, &x, &y);
      x -= touch_x;
      y -= touch_y;
      if ((x * x + y * y) < 20 * 20) {
        if (active_marker != m) {
          previous_marker = active_marker;
          active_marker = m;
          redraw_marker(active_marker);
        }
        // select trace
        uistat.current_trace = t;
        select_lever_mode(LM_MARKER);

        // drag marker until release
        drag_marker(t, m);
        return TRUE;
      }
    }
  }

  return FALSE;
}

static int
touch_lever_mode_select(void)
{
  int touch_x, touch_y;
  touch_position(&touch_x, &touch_y);
  if (touch_y > HEIGHT) {
    select_lever_mode(touch_x < FREQUENCIES_XPOS2 ? LM_CENTER : LM_SPAN);
    return TRUE;
  }
  if (touch_y < 25) {
    if (touch_x < FREQUENCIES_XPOS2 && get_electrical_delay() != 0.0) {
      select_lever_mode(LM_EDELAY);
    } else {
      select_lever_mode(LM_MARKER);
    }
    return TRUE;
  }
  return FALSE;
}

static
void ui_process_touch(void)
{
//  awd_count++;
  adc_stop();

  int status = touch_check();
  if (status == EVT_TOUCH_PRESSED || status == EVT_TOUCH_DOWN) {
    switch (ui_mode) {
    case UI_NORMAL:
      // Try drag marker
      if (touch_pickup_marker())
        break;
      // Try select lever mode (top and bottom screen)
      if (touch_lever_mode_select()) {
        touch_wait_release();
        break;
      }
      // switch menu mode after release
      touch_wait_release();
      selection = -1; // hide keyboard mode selection
      ui_mode_menu();
      break;
    case UI_MENU:
      menu_apply_touch();
      break;

    case UI_NUMERIC:
      numeric_apply_touch();
      break;
    }
  }
  touch_start_watchdog();
}

void
ui_process(void)
{
  if (operation_requested&OP_LEVER)
    ui_process_lever();
  if (operation_requested&OP_TOUCH)
    ui_process_touch();
  operation_requested = OP_NONE;
}

/* Triggered when the button is pressed or released. The LED4 is set to ON.*/
static void extcb1(EXTDriver *extp, expchannel_t channel)
{
  (void)extp;
  (void)channel;
  operation_requested|=OP_LEVER;
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

static const GPTConfig gpt3cfg = {
  1000,    /* 1kHz timer clock.*/
  NULL,   /* Timer callback.*/
  0x0020, /* CR2:MMS=02 to output TRGO */
  0
};

#if 0
static void
test_touch(int *x, int *y)
{
  adc_stop(ADC1);

  *x = touch_measure_x();
  *y = touch_measure_y();

  touch_start_watchdog();
}
#endif

void
handle_touch_interrupt(void)
{
  operation_requested|= OP_TOUCH;
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
