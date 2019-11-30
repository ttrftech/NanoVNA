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
#include "chprintf.h"
#include <stdlib.h>
#include <string.h>


uistat_t uistat = {
 digit: 6,
 current_trace: 0,
 lever_mode: LM_MARKER,
 marker_delta: FALSE,
 marker_smith_format: MS_RLC
};


#define NO_EVENT          0
#define EVT_BUTTON_SINGLE_CLICK    0x01
#define EVT_BUTTON_DOUBLE_CLICK    0x02
#define EVT_BUTTON_DOWN_LONG    0x04
#define EVT_UP          0x10
#define EVT_DOWN        0x20
#define EVT_REPEAT        0x40

#define BUTTON_DOWN_LONG_TICKS    5000  /* 1sec */
#define BUTTON_DOUBLE_TICKS      5000   /* 500ms */
#define BUTTON_REPEAT_TICKS      1000   /* 100ms */
#define BUTTON_DEBOUNCE_TICKS    200

/* lever switch assignment */
#define BIT_UP1   3
#define BIT_PUSH  2
#define BIT_DOWN1  1

#define READ_PORT() palReadPort(GPIOA)
#define BUTTON_MASK 0b1111

static uint16_t last_button = 0b0000;
static uint32_t last_button_down_ticks;
static uint32_t last_button_repeat_ticks;
static int8_t inhibit_until_release = FALSE;

uint8_t operation_requested = OP_NONE;

int8_t previous_marker = -1;

enum {
  UI_NORMAL, UI_MENU, UI_NUMERIC, UI_KEYPAD
};

enum {
  KM_START, KM_STOP, KM_CENTER, KM_SPAN, KM_CW, KM_SCALE, KM_REFPOS, KM_EDELAY, KM_VELOCITY_FACTOR, KM_SCALEDELAY
};

uint8_t ui_mode = UI_NORMAL;
uint8_t keypad_mode;
int8_t selection = 0;

typedef struct {
  uint8_t type;
  char *label;
  const void *reference;
} menuitem_t;

int8_t last_touch_status = FALSE;
int16_t last_touch_x;
int16_t last_touch_y;
//int16_t touch_cal[4] = { 1000, 1000, 10*16, 12*16 };
//int16_t touch_cal[4] = { 620, 600, 130, 180 };
#define EVT_TOUCH_NONE 0
#define EVT_TOUCH_DOWN 1
#define EVT_TOUCH_PRESSED 2
#define EVT_TOUCH_RELEASED 3

int awd_count;
//int touch_x, touch_y;

#define NUMINPUT_LEN 10

#define KP_CONTINUE 0
#define KP_DONE 1
#define KP_CANCEL 2

char kp_buf[11];
int8_t kp_index = 0;


void ui_mode_normal(void);
void ui_mode_menu(void);
void ui_mode_numeric(int _keypad_mode);
void ui_mode_keypad(int _keypad_mode);
void draw_menu(void);
void leave_ui_mode(void);
void erase_menu_buttons(void);
void ui_process_keypad(void);
static void ui_process_numeric(void);

static void menu_push_submenu(const menuitem_t *submenu);



static int btn_check(void)
{
  int cur_button = READ_PORT() & BUTTON_MASK;
  int changed = last_button ^ cur_button;
  int status = 0;
    uint32_t ticks = chVTGetSystemTime();
  if (changed & (1<<BIT_PUSH)) {
      if (ticks - last_button_down_ticks >= BUTTON_DEBOUNCE_TICKS) {
        if (cur_button & (1<<BIT_PUSH)) {
          // button released
          status |= EVT_BUTTON_SINGLE_CLICK;
          if (inhibit_until_release) {
            status = 0;
            inhibit_until_release = FALSE;
          }
        }
        last_button_down_ticks = ticks;
      }
  }

    if (changed & (1<<BIT_UP1)) {
      if ((cur_button & (1<<BIT_UP1))
          && (ticks >= last_button_down_ticks + BUTTON_DEBOUNCE_TICKS)) {
        status |= EVT_UP;
      }
      last_button_down_ticks = ticks;
    }
    if (changed & (1<<BIT_DOWN1)) {
      if ((cur_button & (1<<BIT_DOWN1))
          && (ticks >= last_button_down_ticks + BUTTON_DEBOUNCE_TICKS)) {
        status |= EVT_DOWN;
      }
      last_button_down_ticks = ticks;
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

    if (!inhibit_until_release) {
      if ((cur_button & (1<<BIT_PUSH))
          && ticks - last_button_down_ticks >= BUTTON_DOWN_LONG_TICKS) {
        inhibit_until_release = TRUE;
        return EVT_BUTTON_DOWN_LONG;
      }
      if ((changed & (1<<BIT_PUSH))
          && ticks - last_button_down_ticks < BUTTON_DOWN_LONG_TICKS) {
        return EVT_BUTTON_SINGLE_CLICK;
      }
    }

    if (changed) {
      // finished
      last_button = cur_button;
      last_button_down_ticks = ticks;
      inhibit_until_release = FALSE;
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
  v = adc_single_read(ADC1, ADC_CHSELR_CHSEL7);
  //chThdSleepMilliseconds(2);
  //v += adc_single_read(ADC1, ADC_CHSELR_CHSEL7);
  return v;
}

int
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
  v = adc_single_read(ADC1, ADC_CHSELR_CHSEL6);
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
  adc_start_analog_watchdogd(ADC1, ADC_CHSELR_CHSEL7);
}

int
touch_status(void)
{
  touch_prepare_sense();
  return adc_single_read(ADC1, ADC_CHSELR_CHSEL7) > TOUCH_THRESHOLD;
}

int touch_check(void)
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

extern void ili9341_line(int, int, int, int, int);

void
touch_cal_exec(void)
{
  int status;
  int x1, x2, y1, y2;
  
  adc_stop(ADC1);

  ili9341_fill(0, 0, 320, 240, 0);
  ili9341_line(0, 0, 0, 32, 0xffff);
  ili9341_line(0, 0, 32, 0, 0xffff);
  ili9341_drawstring_8x8_var("TOUCH UPPER LEFT", 10, 10, 0xffff, 0x0000);

  do {
    status = touch_check();
  } while(status != EVT_TOUCH_RELEASED);
  x1 = last_touch_x;
  y1 = last_touch_y;

  ili9341_fill(0, 0, 320, 240, 0);
  ili9341_line(320-1, 240-1, 320-1, 240-32, 0xffff);
  ili9341_line(320-1, 240-1, 320-32, 240-1, 0xffff);
  ili9341_drawstring_8x8_var("TOUCH LOWER RIGHT", 190, 220, 0xffff, 0x0000);

  do {
    status = touch_check();
  } while(status != EVT_TOUCH_RELEASED);
  x2 = last_touch_x;
  y2 = last_touch_y;

  config.touch_cal[0] = x1;
  config.touch_cal[1] = y1;
  config.touch_cal[2] = (x2 - x1) * 16 / 320;
  config.touch_cal[3] = (y2 - y1) * 16 / 240;

  //redraw_all();
  touch_start_watchdog();
}



void
touch_position(int *x, int *y)
{
  *x = (last_touch_x - config.touch_cal[0]) * 16 / config.touch_cal[2];
  *y = (last_touch_y - config.touch_cal[1]) * 16 / config.touch_cal[3];
}



void
touch_draw_test(void)
{
  int status;
  int x0, y0;
  int x1, y1;
  
  adc_stop(ADC1);

  ili9341_fill(0, 0, 320, 240, 0);
  ili9341_drawstring_8x8_var("TOUCH TEST: DRAG PANEL", OFFSETX, 232, 0xffff, 0x0000);

  do {
    status = touch_check();
  } while(status != EVT_TOUCH_PRESSED);
  touch_position(&x0, &y0);

  do {
    status = touch_check();
    touch_position(&x1, &y1);
    ili9341_line(x0, y0, x1, y1, 0xffff);
    x0 = x1;
    y0 = y1;
    chThdSleepMilliseconds(50);
  } while(status != EVT_TOUCH_RELEASED);

  touch_start_watchdog();
}



void
show_version(void)
{
  int x = 5, y = 5;
  
  adc_stop(ADC1);
  ili9341_fill(0, 0, 320, 240, 0);


  ili9341_drawstring_size(USER_CALL, x, y, 0xf800, 0x0000, 3);
  ili9341_drawstring_size(BOARD_NAME, x, y += 25, 0xffff, 0x0000, 4);
  y += 25;

  ili9341_drawstring_8x8_var("2016-2019 Copyright @edy555", x, y += 10, 0xffff, 0x0000);
  ili9341_drawstring_8x8_var("Variant with lager fonts by DL9CAT. =^..^=", x, y += 10, 0xffff, 0x0000);
  ili9341_drawstring_8x8_var("Licensed under GPL.", x, y += 10, 0xffff, 0x0000);
  ili9341_drawstring_8x8_var("  see: https://github.com/reald/NanoVNA", x, y += 10, 0xffff, 0x0000);
  ili9341_drawstring_8x8_var("Version: " VERSION, x, y += 10, 0xffff, 0x0000);
  ili9341_drawstring_8x8_var("Build Time: " __DATE__ " - " __TIME__, x, y += 10, 0xffff, 0x0000);
  y += 5;
  ili9341_drawstring_8x8_var("Kernel: " CH_KERNEL_VERSION, x, y += 10, 0xffff, 0x0000);
  ili9341_drawstring_8x8_var("Compiler: " PORT_COMPILER_NAME, x, y += 10, 0xffff, 0x0000);
  ili9341_drawstring_8x8_var("Architecture: " PORT_ARCHITECTURE_NAME " Core Variant: " PORT_CORE_VARIANT_NAME, x, y += 10, 0xffff, 0x0000);
  ili9341_drawstring_8x8_var("Port Info: " PORT_INFO, x, y += 10, 0xffff, 0x0000);
  y += 5;
  ili9341_drawstring_8x8_var("Platform: ", x, y += 10, 0xffff, 0x0000);
  ili9341_drawstring_8x8_var(PLATFORM_NAME, x, y += 10, 0xffff, 0x0000);

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
  adc_stop(ADC1);

  int x = 110, y = 20;

  // leave a last message 
  ili9341_fill(0, 0, 320, 240, 0);
  ili9341_drawstring_size("DFU", x, y, 0xffff, 0x0000, 4);

  x = 5;
  y += 50;
  ili9341_drawstring_8x8_var("Device Firmware Update Mode", x, y += 10, 0xffff, 0x0000);
  ili9341_drawstring_8x8_var("To exit DFU mode, please reset device yourself.", x, y += 10, 0xffff, 0x0000);

  // see __early_init in ./NANOVNA_STM32_F072/board.c
  *((unsigned long *)BOOT_FROM_SYTEM_MEMORY_MAGIC_ADDRESS) = BOOT_FROM_SYTEM_MEMORY_MAGIC;
  NVIC_SystemReset();
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
  selection = item+1;
  draw_cal_status();
  draw_menu();
}

static void
menu_caldone_cb(int item)
{
  extern const menuitem_t menu_save[];
  //extern const menuitem_t menu_cal[];
  (void)item;
  cal_done();
  draw_cal_status();
  menu_move_back();
  menu_push_submenu(menu_save);
}

static void
menu_cal2_cb(int item)
{
  switch (item) {
  case 2: // RESET
    cal_status = 0;
    break;
  case 3: // CORRECTION
    // toggle applying correction
    if (cal_status)
      cal_status ^= CALSTAT_APPLY;
    draw_menu();
    break;
  }
  draw_cal_status();
  //menu_move_back();
}

static void
menu_recall_cb(int item)
{
  if (item < 0 || item >= 5)
    return;
  if (caldata_recall(item) == 0) {
    menu_move_back();
    ui_mode_normal();
    update_grid();
    draw_cal_status();
  }
}

static void
menu_config_cb(int item)
{
  switch (item) {
  case 0:
      touch_cal_exec();
      redraw_frame();
      request_to_redraw_grid();
      draw_menu();
      break;
  case 1:
      touch_draw_test();
      redraw_frame();
      request_to_redraw_grid();
      draw_menu();
      break;
  case 2:
      config_save();
      menu_move_back();
      ui_mode_normal();
      break;
  case 3:
      show_version();
      redraw_frame();
      request_to_redraw_grid();
      draw_menu();
      break;
  case 4:
      if ( biginfo_enabled == FALSE )
      {
        biginfo_enabled = TRUE;
      }
      else
      {
        biginfo_enabled = FALSE;
      }
      config.biginfo_enabled = biginfo_enabled;
      menu_move_back();
      ui_mode_normal();
      break;
  }
}

static void
menu_dfu_cb(int item)
{
  switch (item) {
  case 0:
      enter_dfu();
  }
}

static void
menu_save_cb(int item)
{
  if (item < 0 || item >= 5)
    return;
  if (caldata_save(item) == 0) {
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
  for (i = 0; i < 4; i++)
    if (trace[i].enabled) {
      uistat.current_trace = i;
      return;
    }
}

static void
menu_trace_cb(int item)
{
  if (item < 0 || item >= 4)
    return;
  if (trace[item].enabled) {
    if (item == uistat.current_trace) {
      // disable if active trace is selected
      trace[item].enabled = FALSE;
      choose_active_trace();
    } else {
      // make active selected trace
      uistat.current_trace = item;
    }
  } else {
    trace[item].enabled = TRUE;
    uistat.current_trace = item;
  }
  request_to_redraw_grid();
  draw_menu();
}

static void
menu_format_cb(int item)
{
  switch (item) {
  case 0:
    set_trace_type(uistat.current_trace, TRC_LOGMAG);
    break;
  case 1:
    set_trace_type(uistat.current_trace, TRC_PHASE);
    break;
  case 2:
    set_trace_type(uistat.current_trace, TRC_DELAY);
    break;
  case 3:
    set_trace_type(uistat.current_trace, TRC_SMITH);
    break;
  case 4:
    set_trace_type(uistat.current_trace, TRC_SWR);
    break;
  }

  request_to_redraw_grid();
  ui_mode_normal();
  //redraw_all();
}

static void
menu_format2_cb(int item)
{
  switch (item) {
  case 0:
    set_trace_type(uistat.current_trace, TRC_POLAR);
    break;
  case 1:
    set_trace_type(uistat.current_trace, TRC_LINEAR);
    break;
  case 2:
    set_trace_type(uistat.current_trace, TRC_REAL);
    break;
  case 3:
    set_trace_type(uistat.current_trace, TRC_IMAG);
    break;
  case 4:
    set_trace_type(uistat.current_trace, TRC_R);
    break;
  case 5:
    set_trace_type(uistat.current_trace, TRC_X);
    break;
  }

  request_to_redraw_grid();
  ui_mode_normal();
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
menu_transform_window_cb(int item)
{
  // TODO
  switch (item) {
    case 0:
      domain_mode = (domain_mode & ~TD_WINDOW) | TD_WINDOW_MINIMUM;
      ui_mode_normal();
      break;
    case 1:
      domain_mode = (domain_mode & ~TD_WINDOW) | TD_WINDOW_NORMAL;
      ui_mode_normal();
      break;
    case 2:
      domain_mode = (domain_mode & ~TD_WINDOW) | TD_WINDOW_MAXIMUM;
      ui_mode_normal();
      break;
  }
}

static void
menu_transform_cb(int item)
{
  int status;
  switch (item) {
    case 0:
      if ((domain_mode & DOMAIN_MODE) == DOMAIN_TIME) {
          domain_mode = (domain_mode & ~DOMAIN_MODE) | DOMAIN_FREQ;
      } else {
          domain_mode = (domain_mode & ~DOMAIN_MODE) | DOMAIN_TIME;
      }
      uistat.lever_mode = LM_MARKER;
      draw_frequencies();
      ui_mode_normal();
      break;
    case 1:
      domain_mode = (domain_mode & ~TD_FUNC) | TD_FUNC_LOWPASS_IMPULSE;
      ui_mode_normal();
      break;
    case 2:
      domain_mode = (domain_mode & ~TD_FUNC) | TD_FUNC_LOWPASS_STEP;
      ui_mode_normal();
      break;
    case 3:
      domain_mode = (domain_mode & ~TD_FUNC) | TD_FUNC_BANDPASS;
      ui_mode_normal();
      break;
    case 5:
      status = btn_wait_release();
      if (status & EVT_BUTTON_DOWN_LONG) {
        ui_mode_numeric(KM_VELOCITY_FACTOR);
        ui_process_numeric();
      } else {
        ui_mode_keypad(KM_VELOCITY_FACTOR);
        ui_process_keypad();
      }
      break;
  }
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

static void
menu_scale_cb(int item)
{
  int status;
  int km = KM_SCALE + item;
  if (km == KM_SCALE && trace[uistat.current_trace].type == TRC_DELAY) {
    km = KM_SCALEDELAY;
  }
  status = btn_wait_release();
  if (status & EVT_BUTTON_DOWN_LONG) {
    ui_mode_numeric(km);
    ui_process_numeric();
  } else {
    ui_mode_keypad(km);
    ui_process_keypad();
  }
}

static void
menu_stimulus_cb(int item)
{
  int status;
  switch (item) {
  case 0: /* START */
  case 1: /* STOP */
  case 2: /* CENTER */
  case 3: /* SPAN */
  case 4: /* CW */
    uistat.lever_mode = item == 3 ? LM_SPAN : LM_CENTER;
    status = btn_wait_release();
    if (status & EVT_BUTTON_DOWN_LONG) {
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


static int32_t
get_marker_frequency(int marker)
{
  if (marker < 0 || marker >= 4)
    return -1;
  if (!markers[marker].enabled)
    return -1;
  return frequencies[markers[marker].index];
}

static void
menu_marker_op_cb(int item)
{
  int32_t freq = get_marker_frequency(active_marker);
  if (freq < 0)
    return; // no active marker

  switch (item) {
  case 0: /* MARKER->START */
    set_sweep_frequency(ST_START, freq);
    break;
  case 1: /* MARKER->STOP */
    set_sweep_frequency(ST_STOP, freq);
    break;
  case 2: /* MARKER->CENTER */
    set_sweep_frequency(ST_CENTER, freq);
    break;
  case 3: /* MARKERS->SPAN */
    {
      if (previous_marker == -1 || active_marker == previous_marker) {
        // if only 1 marker is active, keep center freq and make span the marker comes to the edge  
        int32_t center = get_sweep_frequency(ST_CENTER);
        int32_t span = center - freq;
        if (span < 0) span = -span;
        set_sweep_frequency(ST_SPAN, span * 2);
      } else {
        // if 2 or more marker active, set start and stop freq to each marker
        int32_t freq2 = get_marker_frequency(previous_marker);
        if (freq2 < 0)
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
menu_marker_search_cb(int item)
{
  int i;
  if (active_marker == -1)
    return;

  switch (item) {
  case 0: /* maximum */
  case 1: /* minimum */
    i = marker_search(item);
    if (i != -1)
      markers[active_marker].index = i;
    draw_menu();
    break;
  case 2: /* search Left */
    i = marker_search_left(markers[active_marker].index);
    if (i != -1)
      markers[active_marker].index = i;
    draw_menu();
    break;
  case 3: /* search right */
    i = marker_search_right(markers[active_marker].index);
    if (i != -1)
      markers[active_marker].index = i;
    draw_menu();
    break;
  }
  redraw_marker(active_marker, TRUE);
  uistat.lever_mode = LM_SEARCH;
}

static void
menu_marker_smith_cb(int item)
{
  uistat.marker_smith_format = item;
  redraw_marker(active_marker, TRUE);
  draw_menu();
}

void 
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
menu_marker_sel_cb(int item)
{
  if (item >= 0 && item < 4) {
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
      markers[0].enabled = FALSE;
      markers[1].enabled = FALSE;
      markers[2].enabled = FALSE;
      markers[3].enabled = FALSE;
      previous_marker = -1;
      active_marker = -1;      
  } else if (item == 5) { /* marker delta */
    uistat.marker_delta = !uistat.marker_delta;
  }
  redraw_marker(active_marker, TRUE);
  draw_menu();
  uistat.lever_mode = LM_MARKER;
}

const menuitem_t menu_calop[] = {
  { MT_CALLBACK, "OPEN", menu_calop_cb },
  { MT_CALLBACK, "SHORT", menu_calop_cb },
  { MT_CALLBACK, "LOAD", menu_calop_cb },
  { MT_CALLBACK, "ISOLN", menu_calop_cb },
  { MT_CALLBACK, "THRU", menu_calop_cb },
  { MT_CALLBACK, "DONE", menu_caldone_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_save[] = {
  { MT_CALLBACK, "SAVE 0", menu_save_cb },
  { MT_CALLBACK, "SAVE 1", menu_save_cb },
  { MT_CALLBACK, "SAVE 2", menu_save_cb },
  { MT_CALLBACK, "SAVE 3", menu_save_cb },
  { MT_CALLBACK, "SAVE 4", menu_save_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_cal[] = {
  { MT_SUBMENU, S_RARROW" CALIBR", menu_calop },
  { MT_SUBMENU, "SAVE", menu_save },
  { MT_CALLBACK, "RESET", menu_cal2_cb },
  { MT_CALLBACK, "Correct.", menu_cal2_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_trace[] = {
  { MT_CALLBACK, "TRACE 0", menu_trace_cb },
  { MT_CALLBACK, "TRACE 1", menu_trace_cb },
  { MT_CALLBACK, "TRACE 2", menu_trace_cb },
  { MT_CALLBACK, "TRACE 3", menu_trace_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_format2[] = {
  { MT_CALLBACK, "POLAR", menu_format2_cb },
  { MT_CALLBACK, "LINEAR", menu_format2_cb },
  { MT_CALLBACK, "REAL", menu_format2_cb },
  { MT_CALLBACK, "IMAG", menu_format2_cb },
  { MT_CALLBACK, "RESIST.", menu_format2_cb },
  { MT_CALLBACK, "REACT.", menu_format2_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_format[] = {
  { MT_CALLBACK, "LOGMAG", menu_format_cb },
  { MT_CALLBACK, "PHASE", menu_format_cb },
  { MT_CALLBACK, "DELAY", menu_format_cb },
  { MT_CALLBACK, "SMITH", menu_format_cb },
  { MT_CALLBACK, "SWR", menu_format_cb },
  { MT_SUBMENU, S_RARROW" MORE", menu_format2 },  
  //{ MT_CALLBACK, "LINEAR", menu_format_cb },
  //{ MT_CALLBACK, "SWR", menu_format_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_scale[] = {
  { MT_CALLBACK, "Scle/Div", menu_scale_cb },
  { MT_CALLBACK, "\2Rfrnce\0Position", menu_scale_cb },
  { MT_CALLBACK, "\2Electr.\0Delay", menu_scale_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};


const menuitem_t menu_channel[] = {
  { MT_CALLBACK, "\2CH 0\0REFLECT", menu_channel_cb },
  { MT_CALLBACK, "\2CH 1\0THROUGH", menu_channel_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_transform_window[] = {
  { MT_CALLBACK, "MINIMUM", menu_transform_window_cb },
  { MT_CALLBACK, "NORMAL", menu_transform_window_cb },
  { MT_CALLBACK, "MAXIMUM", menu_transform_window_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_transform[] = {
  { MT_CALLBACK, "\2TRANSFO\0ON", menu_transform_cb },
  { MT_CALLBACK, "\2LOWPASS\0IMPULSE", menu_transform_cb },
  { MT_CALLBACK, "\2LOWPASS\0STEP", menu_transform_cb },
  { MT_CALLBACK, "BANDP.", menu_transform_cb },
  { MT_SUBMENU, "WINDOW", menu_transform_window },
  { MT_CALLBACK, "\2Velocity\0FACTOR", menu_transform_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_display[] = {
  { MT_SUBMENU, "TRACE", menu_trace },
  { MT_SUBMENU, "FORMAT", menu_format },
  { MT_SUBMENU, "SCALE", menu_scale },
  { MT_SUBMENU, "CHANNEL", menu_channel },
  { MT_SUBMENU, "TRNSFRM", menu_transform },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_stimulus[] = {
  { MT_CALLBACK, "START", menu_stimulus_cb },
  { MT_CALLBACK, "STOP", menu_stimulus_cb },
  { MT_CALLBACK, "CENTER", menu_stimulus_cb },
  { MT_CALLBACK, "SPAN", menu_stimulus_cb },
  { MT_CALLBACK, "CW FREQ", menu_stimulus_cb },
  { MT_CALLBACK, "\2PAUSE\0SWEEP", menu_stimulus_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_marker_sel[] = {
  { MT_CALLBACK, "MARKER1", menu_marker_sel_cb },
  { MT_CALLBACK, "MARKER2", menu_marker_sel_cb },
  { MT_CALLBACK, "MARKER3", menu_marker_sel_cb },
  { MT_CALLBACK, "MARKER4", menu_marker_sel_cb },
  { MT_CALLBACK, "ALL OFF", menu_marker_sel_cb },
  { MT_CALLBACK, "DELTA", menu_marker_sel_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_marker_ops[] = {
  { MT_CALLBACK, S_RARROW"START", menu_marker_op_cb },
  { MT_CALLBACK, S_RARROW"STOP", menu_marker_op_cb },
  { MT_CALLBACK, S_RARROW"CENTER", menu_marker_op_cb },
  { MT_CALLBACK, S_RARROW"SPAN", menu_marker_op_cb },
  { MT_CALLBACK, S_RARROW"EDELAY", menu_marker_op_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_marker_search[] = {
  //{ MT_CALLBACK, "OFF", menu_marker_search_cb },
  { MT_CALLBACK, "MAXIMUM", menu_marker_search_cb },
  { MT_CALLBACK, "MINIMUM", menu_marker_search_cb },
  { MT_CALLBACK, "\2SEARCH\0" S_LARROW" LEFT", menu_marker_search_cb },
  { MT_CALLBACK, "\2SEARCH\0" S_RARROW" RIGHT", menu_marker_search_cb },
  //{ MT_CALLBACK, "TRACKING", menu_marker_search_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_marker_smith[] = {
  { MT_CALLBACK, "LIN", menu_marker_smith_cb },
  { MT_CALLBACK, "LOG", menu_marker_smith_cb },
  { MT_CALLBACK, "Re+Im", menu_marker_smith_cb },
  { MT_CALLBACK, "R+Xj", menu_marker_smith_cb },
  { MT_CALLBACK, "R+L/C", menu_marker_smith_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_marker[] = {
  { MT_SUBMENU, "\2SELECT\0MARKER", menu_marker_sel },
  { MT_SUBMENU, "SEARCH", menu_marker_search },
  { MT_SUBMENU, "OPERATIONS", menu_marker_ops },
  { MT_SUBMENU, "\2SMITH\0VALUE", menu_marker_smith },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_recall[] = {
  { MT_CALLBACK, "RECALL0", menu_recall_cb },
  { MT_CALLBACK, "RECALL1", menu_recall_cb },
  { MT_CALLBACK, "RECALL2", menu_recall_cb },
  { MT_CALLBACK, "RECALL3", menu_recall_cb },
  { MT_CALLBACK, "RECALL4", menu_recall_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_dfu[] = {
  { MT_CALLBACK, "\2RESET+\0Ent DFU", menu_dfu_cb },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_config[] = {
  { MT_CALLBACK, "\2TOUCH\0CAL", menu_config_cb },
  { MT_CALLBACK, "\2TOUCH\0TEST", menu_config_cb },
  { MT_CALLBACK, "SAVE", menu_config_cb },
  { MT_CALLBACK, "VERSION", menu_config_cb },
  { MT_CALLBACK, "INFO CH0", menu_config_cb },
  { MT_SUBMENU, " " S_RARROW " DFU", menu_dfu },
  { MT_CANCEL, S_LARROW" BACK", NULL },
  { MT_NONE, NULL, NULL } // sentinel
};

const menuitem_t menu_top[] = {
  { MT_SUBMENU, "DISPLAY", menu_display },
  { MT_SUBMENU, "MARKER", menu_marker },
  { MT_SUBMENU, "STIMULUS", menu_stimulus },
  { MT_SUBMENU, "CAL", menu_cal },
  { MT_SUBMENU, "RECALL", menu_recall },
  { MT_SUBMENU, "CONFIG", menu_config },
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

/*
static void menu_move_top(void)
{
  if (menu_current_level == 0)
    return;
  menu_current_level = 0;
  ensure_selection();
  erase_menu_buttons();
  draw_menu();
}
*/

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
#define KP_PLUSMINUS 19
#define KP_KEYPAD 20
#define KP_N 21
#define KP_P 22

typedef struct {
  uint16_t x, y;
  int8_t c;
} keypads_t;

const keypads_t *keypads;
uint8_t keypads_last_index;

const keypads_t keypads_freq[] = {
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
  { 0, 0, -1 }
};

const keypads_t keypads_scale[] = {
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
  { KP_X(3), KP_Y(3), KP_X1 },
  { KP_X(2), KP_Y(3), KP_BS },
  { 0, 0, -1 }
};

const keypads_t keypads_time[] = {
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
  { KP_X(3), KP_Y(1), KP_N },
  { KP_X(3), KP_Y(2), KP_P },
  { KP_X(3), KP_Y(3), KP_MINUS },
  { KP_X(2), KP_Y(3), KP_BS },
  { 0, 0, -1 }
};

const keypads_t * const keypads_mode_tbl[] = {
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

const char * const keypad_mode_label[] = {
  "START", "STOP", "CENTER", "SPAN", "CW FREQ", "SCALE", "REFPOS", "EDELAY", "VELOCITY%", "DELAY"
};

void
draw_keypad(void)
{
  int i = 0;
  while (keypads[i].x) {
    uint16_t bg = config.menu_normal_color;
    if (i == selection)
      bg = config.menu_active_color;
    ili9341_fill(keypads[i].x, keypads[i].y, 44, 44, bg);
    ili9341_drawfont(keypads[i].c, &NF20x22, keypads[i].x+12, keypads[i].y+10, 0x0000, bg);
    i++;
  }
}

void
draw_numeric_area_frame(void)
{
  ili9341_fill(0, 208, 320, 32, 0xffff);
  ili9341_drawstring_8x8_var(keypad_mode_label[keypad_mode], 10, 220, 0x0000, 0xffff);
  ili9341_drawfont(KP_KEYPAD, &NF20x22, 300, 216, 0x0000, 0xffff);
}

void
draw_numeric_input(const char *buf)
{
  int i = 0;
  int x = 64;
  int focused = FALSE;
  const uint16_t xsim[] = { 0, 0, 8, 0, 0, 8, 0, 0, 0, 0 };
  for (i = 0; i < 10 && buf[i]; i++) {
    uint16_t fg = 0x0000;
    uint16_t bg = 0xffff;
    int c = buf[i];
    if (c == '.')
      c = KP_PERIOD;
    else if (c == '-')
      c = KP_MINUS;
    else if (c >= '0' && c <= '9')
      c = c - '0';
    else
      c = -1;

    if (uistat.digit == 8-i) {
      fg = RGB565(128,255,128);
      focused = TRUE;
      if (uistat.digit_mode)
        bg = 0x0000;
    }

    if (c >= 0)
      ili9341_drawfont(c, &NF20x22, x, 208+4, fg, bg);
    else if (focused)
      ili9341_drawfont(0, &NF20x22, x, 208+4, fg, bg);
    else
      ili9341_fill(x, 208+4, 20, 24, bg);
      
    x += 20;
    if (xsim[i] > 0) {
      //ili9341_fill(x, 208+4, xsim[i], 20, bg);
      x += xsim[i];
    }
  }
  if (i < 10) {
      ili9341_fill(x, 208+4, 20*(10-i), 24, 0xffff);
  }
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
  if (menu == menu_trace && item < 4) {
    if (trace[item].enabled)
      *bg = config.trace_color[item];
  } else if (menu == menu_marker_sel) {
    if (item < 4) {
      if (markers[item].enabled) {
        *bg = 0x0000;
        *fg = 0xffff;
      }
    } else if (item == 5) {
      if (uistat.marker_delta) {
        *bg = 0x0000;
        *fg = 0xffff;
      }
    }
  } else if (menu == menu_marker_smith) {
    if (uistat.marker_smith_format == item) {
      *bg = 0x0000;
      *fg = 0xffff;
    }
  } else if (menu == menu_calop) {
    if ((item == 0 && (cal_status & CALSTAT_OPEN))
        || (item == 1 && (cal_status & CALSTAT_SHORT))
        || (item == 2 && (cal_status & CALSTAT_LOAD))
        || (item == 3 && (cal_status & CALSTAT_ISOLN))
        || (item == 4 && (cal_status & CALSTAT_THRU))) {
      domain_mode = (domain_mode & ~DOMAIN_MODE) | DOMAIN_FREQ;
      *bg = 0x0000;
      *fg = 0xffff;
    }
  } else if (menu == menu_stimulus) {
    if (item == 5 /* PAUSE */ && !sweep_enabled) {
      *bg = 0x0000;
      *fg = 0xffff;
    }
  } else if (menu == menu_cal) {
    if (item == 3 /* CORRECTION */ && (cal_status & CALSTAT_APPLY)) {
      *bg = 0x0000;
      *fg = 0xffff;
    }
  } else if (menu == menu_transform) {
      if ((item == 0 && (domain_mode & DOMAIN_MODE) == DOMAIN_TIME)
       || (item == 1 && (domain_mode & TD_FUNC) == TD_FUNC_LOWPASS_IMPULSE)
       || (item == 2 && (domain_mode & TD_FUNC) == TD_FUNC_LOWPASS_STEP)
       || (item == 3 && (domain_mode & TD_FUNC) == TD_FUNC_BANDPASS)
       ) {
        *bg = 0x0000;
        *fg = 0xffff;
      }
  } else if (menu == menu_transform_window) {
      if ((item == 0 && (domain_mode & TD_WINDOW) == TD_WINDOW_MINIMUM)
       || (item == 1 && (domain_mode & TD_WINDOW) == TD_WINDOW_NORMAL)
       || (item == 2 && (domain_mode & TD_WINDOW) == TD_WINDOW_MAXIMUM)
       ) {
        *bg = 0x0000;
        *fg = 0xffff;
      }
  }
}

void
draw_menu_buttons(const menuitem_t *menu)
{
  int i = 0;
  for (i = 0; i < 7; i++) 
  {
    const char *l1, *l2;
    if (menu[i].type == MT_NONE)
      break;
    if (menu[i].type == MT_BLANK) 
      continue;

    int y = 32*i;

    uint16_t bg = config.menu_normal_color;
    uint16_t fg = 0x0000;

    // focus only in MENU mode but not in KEYPAD mode
    if (ui_mode == UI_MENU && i == selection)
      bg = config.menu_active_color;

    ili9341_fill(320-60, y, 60, 30, bg);
    
    menu_item_modify_attribute(menu, i, &fg, &bg);

    if (menu_is_multiline(menu[i].label, &l1, &l2)) 
    {
      ili9341_drawstring_8x8_var(l1, 320-57, y+7, fg, bg);
      ili9341_drawstring_8x8_var(l2, 320-57, y+16, fg, bg);
    } 
    else 
    {
      ili9341_drawstring_8x8_var(menu[i].label, 320-57, y+12, fg, bg);
    }
  }
}



void
menu_select_touch(int i)
{
  selection = i;
  draw_menu();
  touch_wait_release();
  selection = -1;
  menu_invoke(i);
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
    if (y-2 < touch_y && touch_y < y+30+2
        && 320-60 < touch_x) {
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
erase_numeric_input(void)
{
  uint16_t bg = 0;
  ili9341_fill(0, 240-32, 320, 32, bg);
}

void
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

void
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
    uistat.value = velocity_factor;
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
  uistat.previous_value = uistat.value;
}

void set_numeric_value(void)
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
    velocity_factor = uistat.value;
    break;
  }
}

void
draw_numeric_area(void)
{
  char buf[10];
  chsnprintf(buf, sizeof buf, "%9d", uistat.value);
  draw_numeric_input(buf);
}


void
ui_mode_menu(void)
{
  if (ui_mode == UI_MENU) 
    return;

  ui_mode = UI_MENU;
  /* narrowen plotting area */
  area_width = AREA_WIDTH_NORMAL - (64-8);
  area_height = HEIGHT;
  ensure_selection();
  draw_menu();
}

void
ui_mode_numeric(int _keypad_mode)
{
  if (ui_mode == UI_NUMERIC) 
    return;

  leave_ui_mode();
  
  // keypads array
  keypad_mode = _keypad_mode;
  ui_mode = UI_NUMERIC;
  area_width = AREA_WIDTH_NORMAL;
  area_height = 240-32;//HEIGHT - 32;

  draw_numeric_area_frame();
  fetch_numeric_target();
  draw_numeric_area();
}

void
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
  area_width = AREA_WIDTH_NORMAL - (64-8);
  area_height = HEIGHT - 32;
  draw_menu();
  draw_keypad();
  draw_numeric_area_frame();
  draw_numeric_input("");
}

void
ui_mode_normal(void)
{
  if (ui_mode == UI_NORMAL) 
    return;

  area_width = AREA_WIDTH_NORMAL;
  area_height = HEIGHT;
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
        markers[active_marker].frequency = frequencies[markers[active_marker].index];
        redraw_marker(active_marker, FALSE);
      }
      if ((status & EVT_UP) && markers[active_marker].index < 100) {
        markers[active_marker].index++;
        markers[active_marker].frequency = frequencies[markers[active_marker].index];
        redraw_marker(active_marker, FALSE);
      }
    }
    status = btn_wait_release();
  } while (status != 0);
  if (active_marker >= 0)
    redraw_marker(active_marker, TRUE);
}

static void
lever_search_marker(int status)
{
  if (active_marker >= 0) {
    if (status & EVT_DOWN) {
      int i = marker_search_left(markers[active_marker].index);
      if (i != -1)
        markers[active_marker].index = i;
    } else if (status & EVT_UP) {
      int i = marker_search_right(markers[active_marker].index);
      if (i != -1)
        markers[active_marker].index = i;
    }
    redraw_marker(active_marker, TRUE);
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
  for (x = 1; x*10 < v; x *= 10)
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
    set_sweep_frequency(ST_SPAN, span);
  } else if (status & EVT_DOWN) {
    span = step_round(span + 1);
    span = step_round(span * 3);
    set_sweep_frequency(ST_SPAN, span);
  }
}

static void
lever_move_center(int status)
{
  uint32_t center = get_sweep_frequency(ST_CENTER);
  uint32_t span = get_sweep_frequency(ST_SPAN);
  span = step_round(span / 3);
  if (status & EVT_UP) {
    set_sweep_frequency(ST_CENTER, center + span);
  } else if (status & EVT_DOWN) {
    set_sweep_frequency(ST_CENTER, center - span);
  }
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
      case LM_CENTER: lever_move_center(status);   break;
      case LM_SPAN:   lever_zoom_span(status);     break;      
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
          draw_menu();
        }
        if (status & EVT_DOWN) {
          if (selection == 0)
            goto menuclose;
          selection--;
          draw_menu();
        }
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
    float scale = 1;
    if (c >= KP_X1 && c <= KP_G) {
      int n = c - KP_X1;
      while (n-- > 0)
        scale *= 1000;
    } else if (c == KP_N) {
      scale *= 1000;
    }
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
    case KM_REFPOS:
      set_trace_refpos(uistat.current_trace, value);
      break;
    case KM_EDELAY:
      set_electrical_delay(value); // pico seconds
      break;
    case KM_VELOCITY_FACTOR:
      velocity_factor = value;
      break;
    case KM_SCALEDELAY:
      set_trace_scale(uistat.current_trace, value * 1e-12); // pico second
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

  while (keypads[i].x) {
    if (keypads[i].x-2 < touch_x && touch_x < keypads[i].x+44+2
        && keypads[i].y-2 < touch_y && touch_y < keypads[i].y+44+2) {
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
  if (touch_y > 48 * 4) {
    // exit keypad mode
    return -2;
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

  if (touch_y > 240-40) {
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
            if (uistat.digit < 8) {
              uistat.digit++;
              draw_numeric_area();
            } else {
              goto exit;
            }
          }
          if (status & EVT_UP) {
            if (uistat.digit > 0) {
              uistat.digit--;
              draw_numeric_area();
            } else {
              goto exit;
            }
          }
        } else {
          int32_t step = 1;
          int n;
          for (n = uistat.digit; n > 0; n--)
            step *= 10;
          if (status & EVT_DOWN) {
            uistat.value += step;
            draw_numeric_area();
          }
          if (status & EVT_UP) {
            uistat.value -= step;
            draw_numeric_area();
          }
        }
        status = btn_wait_release();
      } while (status != 0);
    }
  }

  return;

 exit:
  // cancel operation
  ui_mode_normal();
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
          if (selection < 0)
            selection = keypads_last_index;
          draw_keypad();
        }
        if (s & EVT_DOWN) {
          selection++;
          if (keypads[selection].c < 0) {
            // reaches to tail
            selection = 0;
          }
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
      int key = keypad_apply_touch();
      if (key >= 0 && keypad_click(key))
        /* exit loop on done or cancel */
        break;
      else if (key == -2) {
        //xxx;
        return;
      }
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
  int status;
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
      redraw_marker(m, TRUE);
    }

    status = touch_check();
  } while(status != EVT_TOUCH_RELEASED);
}

static int 
sq_distance(int x0, int y0)
{
  return x0*x0 + y0*y0;
}

static int
touch_pickup_marker(void)
{
  int touch_x, touch_y;
  int m, t;
  touch_position(&touch_x, &touch_y);
  touch_x -= OFFSETX;
  touch_y -= OFFSETY;

  for (m = 0; m < 4; m++) {
    if (!markers[m].enabled)
      continue;

    for (t = 0; t < 4; t++) {
      int x, y;
      if (!trace[t].enabled)
        continue;

      marker_position(m, t, &x, &y);

      if (sq_distance(x - touch_x, y - touch_y) < 400) {
        if (active_marker != m) {
          previous_marker = active_marker;
          active_marker = m;
          redraw_marker(active_marker, TRUE);
        }
        // select trace
        uistat.current_trace = t;
        
        // drag marker until release
        drag_marker(t, m);
        return TRUE;
      }
    }
  }

  return FALSE;
}


static
void ui_process_touch(void)
{
  awd_count++;
  adc_stop(ADC1);

  int status = touch_check();
  if (status == EVT_TOUCH_PRESSED || status == EVT_TOUCH_DOWN) {
    switch (ui_mode) {
    case UI_NORMAL:

      if (touch_pickup_marker()) {
        break;
      }
      
      touch_wait_release();

      // switch menu mode
      selection = -1;
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
