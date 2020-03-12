/*
 * Copyright (c) 2016-2017, TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
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
#include "usbcfg.h"
#include "si5351.h"
#include "nanovna.h"
#include "fft.h"

#include <chprintf.h>
//#include <stdlib.h>
#include <string.h>
//#include <ctype.h>
#include <math.h>

/*
 *  Shell settings
 */
// If need run shell as thread (use more amount of memory fore stack), after enable this need reduce spi_buffer size, by default shell run in main thread
//#define VNA_SHELL_THREAD

static BaseSequentialStream *shell_stream = (BaseSequentialStream *)&SDU1;

// Shell new line
#define VNA_SHELL_NEWLINE_STR    "\r\n"
// Shell command promt
#define VNA_SHELL_PROMPT_STR     "ch> "
// Shell max arguments
#define VNA_SHELL_MAX_ARGUMENTS   4
// Shell max command line size
#define VNA_SHELL_MAX_LENGTH     48
// Shell command functions prototypes
typedef                                         void (*vna_shellcmd_t)(int argc, char *argv[]);
#define VNA_SHELL_FUNCTION(command_name) static void      command_name(int argc, char *argv[])

// Shell command line buffer
static char shell_line[VNA_SHELL_MAX_LENGTH];

//#define ENABLED_DUMP
//#define ENABLE_THREADS_COMMAND
//#define ENABLE_TIME_COMMAND
#define ENABLE_VBAT_OFFSET_COMMAND
#define ENABLE_INFO_COMMAND

static void apply_error_term_at(int i);
static void apply_edelay_at(int i);
static void cal_interpolate(int s);
void update_frequencies(void);
void set_frequencies(uint32_t start, uint32_t stop, uint16_t points);

static bool sweep(bool break_on_operation);
static void transform_domain(void);

static MUTEX_DECL(mutex);

#define DRIVE_STRENGTH_AUTO (-1)
#define FREQ_HARMONICS (config.harmonic_freq_threshold)
#define IS_HARMONIC_MODE(f) ((f) > FREQ_HARMONICS)
// Obsolete, always use interpolate
#define  cal_auto_interpolate  TRUE

static int32_t frequency_offset = 5000;
static uint32_t frequency = 10000000;
static int8_t drive_strength = DRIVE_STRENGTH_AUTO;
int8_t sweep_mode = SWEEP_ENABLE;
int8_t sweep_log = 0;
volatile uint8_t redraw_request = 0; // contains REDRAW_XXX flags

// Version text, displayed in Config->Version menu, also send by info command
const char *info_about[]={
  BOARD_NAME,
  "2016-2020 Copyright @edy555",
  "Licensed under GPL. See: https://github.com/ttrftech/NanoVNA",
  "Version: " VERSION,
  "Build Time: " __DATE__ " - " __TIME__,
  "Kernel: " CH_KERNEL_VERSION,
  "Compiler: " PORT_COMPILER_NAME,
  "Architecture: " PORT_ARCHITECTURE_NAME " Core Variant: " PORT_CORE_VARIANT_NAME,
  "Port Info: " PORT_INFO,
  "Platform: " PLATFORM_NAME,
  0 // sentinel
};

static THD_WORKING_AREA(waThread1, 640);
static THD_FUNCTION(Thread1, arg)
{
  (void)arg;
  chRegSetThreadName("sweep");

  while (1) {
    bool completed = false;
    if (sweep_mode&(SWEEP_ENABLE|SWEEP_ONCE)) {
      chMtxLock(&mutex);
      completed = sweep(true);
      sweep_mode&=~SWEEP_ONCE;
      chMtxUnlock(&mutex);
    } else {
      __WFI();
    }

    chMtxLock(&mutex);
    ui_process();

    if (sweep_mode&SWEEP_ENABLE) {
      // calculate trace coordinates and plot only if scan completed
      if (completed) {
        if ((domain_mode & DOMAIN_MODE) == DOMAIN_TIME)
          transform_domain();
        plot_into_index(measured);
        redraw_request |= REDRAW_CELLS|REDRAW_BATTERY;

        if (uistat.marker_tracking) {
          int i = marker_search();
          if (i != -1 && active_marker != -1) {
            markers[active_marker].index = i;
            redraw_request |= REDRAW_MARKER;
          }
        }
      }
    }
    // plot trace and other indications as raster
    draw_all(completed); // flush markmap only if scan completed to prevent remaining traces
    chMtxUnlock(&mutex);
  }
}

static inline void
pause_sweep(void)
{
  sweep_mode&=~SWEEP_ENABLE;
}

static inline void
resume_sweep(void)
{
  sweep_mode|=SWEEP_ENABLE;
}

void toggle_logLin(void) {
  sweep_log = !sweep_log;
}
void
toggle_sweep(void)
{
  sweep_mode^=SWEEP_ENABLE;
}

static float
bessel0(float x) {
  const float eps = 0.0001;

  float ret = 0;
  float term = 1;
  float m = 0;

  while (term  > eps * ret) {
    ret += term;
    ++m;
    term *= (x*x) / (4*m*m);
  }
  return ret;
}

static float
kaiser_window(float k, float n, float beta) {
  if (beta == 0.0) return 1.0;
  float r = (2 * k) / (n - 1) - 1;
  return bessel0(beta * sqrt(1 - r * r)) / bessel0(beta);
}

static void
transform_domain(void)
{
  // use spi_buffer as temporary buffer
  // and calculate ifft for time domain
  float* tmp = (float*)spi_buffer;

  uint8_t window_size = POINTS_COUNT, offset = 0;
  uint8_t is_lowpass = FALSE;
  switch (domain_mode & TD_FUNC) {
      case TD_FUNC_BANDPASS:
          offset = 0;
          window_size = POINTS_COUNT;
          break;
      case TD_FUNC_LOWPASS_IMPULSE:
      case TD_FUNC_LOWPASS_STEP:
          is_lowpass = TRUE;
          offset = POINTS_COUNT;
          window_size = POINTS_COUNT*2;
          break;
  }

  float beta = 0.0;
  switch (domain_mode & TD_WINDOW) {
      case TD_WINDOW_MINIMUM:
          beta = 0.0; // this is rectangular
          break;
      case TD_WINDOW_NORMAL:
          beta = 6.0;
          break;
      case TD_WINDOW_MAXIMUM:
          beta = 13;
          break;
  }


  for (int ch = 0; ch < 2; ch++) {
      memcpy(tmp, measured[ch], sizeof(measured[0]));
      for (int i = 0; i < POINTS_COUNT; i++) {
          float w = kaiser_window(i+offset, window_size, beta);
          tmp[i*2+0] *= w;
          tmp[i*2+1] *= w;
      }
      for (int i = POINTS_COUNT; i < FFT_SIZE; i++) {
          tmp[i*2+0] = 0.0;
          tmp[i*2+1] = 0.0;
      }
      if (is_lowpass) {
          for (int i = 1; i < POINTS_COUNT; i++) {
              tmp[(FFT_SIZE-i)*2+0] =  tmp[i*2+0];
              tmp[(FFT_SIZE-i)*2+1] = -tmp[i*2+1];
          }
      }

      fft256_inverse((float(*)[2])tmp);
      memcpy(measured[ch], tmp, sizeof(measured[0]));
      for (int i = 0; i < POINTS_COUNT; i++) {
          measured[ch][i][0] /= (float)FFT_SIZE;
          if (is_lowpass) {
              measured[ch][i][1] = 0.0;
          } else {
              measured[ch][i][1] /= (float)FFT_SIZE;
          }
      }
      if ( (domain_mode & TD_FUNC) == TD_FUNC_LOWPASS_STEP ) {
          for (int i = 1; i < POINTS_COUNT; i++) {
              measured[ch][i][0] += measured[ch][i-1][0];
          }
      }
  }
}

// Shell commands output
static int shell_printf(const char *fmt, ...) {
  va_list ap;
  int formatted_bytes;
  va_start(ap, fmt);
  formatted_bytes = chvprintf(shell_stream, fmt, ap);
  va_end(ap);
  return formatted_bytes;
}

VNA_SHELL_FUNCTION(cmd_pause)
{
  (void)argc;
  (void)argv;
  pause_sweep();
}

VNA_SHELL_FUNCTION(cmd_resume)
{
  (void)argc;
  (void)argv;

  // restore frequencies array and cal
  update_frequencies();
  if (cal_auto_interpolate && (cal_status & CALSTAT_APPLY))
    cal_interpolate(lastsaveid);

  resume_sweep();
}

VNA_SHELL_FUNCTION(cmd_reset)
{
  (void)argc;
  (void)argv;

  if (argc == 1) {
    if (strcmp(argv[0], "dfu") == 0) {
      shell_printf("Performing reset to DFU mode\r\n");
      enter_dfu();
      return;
    }
  }
  shell_printf("Performing reset\r\n");

  rccEnableWWDG(FALSE);
  WWDG->CFR = 0x60;
  WWDG->CR = 0xff;

  /* wait forever */
  while (1)
    ;
}

const int8_t gain_table[] = {
  0,  // 0 ~ 300MHz
  40, // 300 ~ 600MHz
  50, // 600 ~ 900MHz
  75, // 900 ~ 1200MHz
  85, // 1200 ~ 1500MHz
  95, // 1500MHz ~
  95, // 1800MHz ~
  95, // 2100MHz ~
  95  // 2400MHz ~
};

#define DELAY_GAIN_CHANGE 2

static int
adjust_gain(uint32_t newfreq)
{
  int new_order = newfreq / FREQ_HARMONICS;
  int old_order = frequency / FREQ_HARMONICS;
  if (new_order != old_order) {
    tlv320aic3204_set_gain(gain_table[new_order], gain_table[new_order]);
    return DELAY_GAIN_CHANGE;
  }
  return 0;
}

int set_frequency(uint32_t freq)
{
  int delay = adjust_gain(freq);
  int8_t ds = drive_strength;
  if (ds == DRIVE_STRENGTH_AUTO) {
    ds = freq > FREQ_HARMONICS ? SI5351_CLK_DRIVE_STRENGTH_8MA : SI5351_CLK_DRIVE_STRENGTH_2MA;
  }
  delay += si5351_set_frequency_with_offset(freq, frequency_offset, ds);

  frequency = freq;
  return delay;
}

// Use macro, std isdigit more big
#define _isdigit(c) (c >= '0' && c <= '9')
// Rewrite universal standart str to value functions to more compact
//
// Convert string to int32
static int32_t my_atoi(const char *p){
  int32_t value = 0;
  uint32_t c;
  bool neg = false;

  if (*p == '-') {neg = true; p++;}
  if (*p == '+') p++;
  while ((c = *p++ - '0') < 10)
    value = value * 10 + c;
  return neg ? -value : value;
}

// Convert string to uint32
uint32_t my_atoui(const char *p){
  uint32_t value = 0;
  uint32_t c;
  if (*p == '+') p++;
  while ((c = *p++ - '0') < 10)
    value = value * 10 + c;
  return value;
}

double
my_atof(const char *p)
{
  int neg = FALSE;
  if (*p == '-')
    neg = TRUE;
  if (*p == '-' || *p == '+')
    p++;
  double x = my_atoi(p);
  while (_isdigit((int)*p))
    p++;
  if (*p == '.') {
    double d = 1.0f;
    p++;
    while (_isdigit((int)*p)) {
      d /= 10;
      x += d * (*p - '0');
      p++;
    }
  }
  if (*p == 'e' || *p == 'E') {
    p++;
    int exp = my_atoi(p);
    while (exp > 0) {
      x *= 10;
      exp--;
    }
    while (exp < 0) {
      x /= 10;
      exp++;
    }
  }
  if (neg)
    x = -x;
  return x;
}

//
// Function used for search substring v in list
// Example need search parameter "center" in "start|stop|center|span|cw" getStringIndex return 2
// If not found return -1
// Used for easy parse command arguments
static int getStringIndex(char *v, const char *list){
  int i = 0;
  while(1){
    char *p = v;
    while (1){
      char c = *list; if (c == '|') c = 0;
      if (c == *p++){
        // Found, return index
        if (c == 0) return i;
        list++;    // Compare next symbol
        continue;
      }
      break;  // Not equal, break
    }
    // Set new substring ptr
    while (1){
      // End of string, not found
      if (*list   ==  0 ) return -1;
      if (*list++ == '|') break;
    }
    i++;
  }
  return -1;
}

VNA_SHELL_FUNCTION(cmd_offset)
{
    if (argc != 1) {
        shell_printf("usage: offset {frequency offset(Hz)}\r\n");
        return;
    }
    frequency_offset = my_atoui(argv[0]);
    set_frequency(frequency);
}

VNA_SHELL_FUNCTION(cmd_freq)
{
    if (argc != 1) {
        goto usage;
    }
    uint32_t freq = my_atoui(argv[0]);

    pause_sweep();
    set_frequency(freq);
    return;
usage:
    shell_printf("usage: freq {frequency(Hz)}\r\n");
}

VNA_SHELL_FUNCTION(cmd_power)
{
  if (argc != 1) {
      shell_printf("usage: power {0-3|-1}\r\n");
      return;
  }
  drive_strength = my_atoi(argv[0]);
  set_frequency(frequency);
}

#ifdef ENABLE_TIME_COMMAND
#if HAL_USE_RTC == FALSE
#error "Error cmd_time require define HAL_USE_RTC = TRUE in halconf.h"
#endif
VNA_SHELL_FUNCTION(cmd_time)
{
  RTCDateTime timespec;
  (void)argc;
  (void)argv;
  rtcGetTime(&RTCD1, &timespec);
  shell_printf("%d/%d/%d %d\r\n", timespec.year+1980, timespec.month, timespec.day, timespec.millisecond);
}
#endif

VNA_SHELL_FUNCTION(cmd_dac)
{
  int value;
  if (argc != 1) {
    shell_printf("usage: dac {value(0-4095)}\r\n"\
                 "current value: %d\r\n", config.dac_value);
    return;
  }
  value = my_atoi(argv[0]);
  config.dac_value = value;
  dacPutChannelX(&DACD2, 0, value);
}

VNA_SHELL_FUNCTION(cmd_threshold)
{
  uint32_t value;
  if (argc != 1) {
    shell_printf("usage: threshold {frequency in harmonic mode}\r\n"\
                 "current: %d\r\n", config.harmonic_freq_threshold);
    return;
  }
  value = my_atoui(argv[0]);
  config.harmonic_freq_threshold = value;
}

VNA_SHELL_FUNCTION(cmd_saveconfig)
{
  (void)argc;
  (void)argv;
  config_save();
  shell_printf("Config saved.\r\n");
}

VNA_SHELL_FUNCTION(cmd_clearconfig)
{
  if (argc != 1) {
    shell_printf("usage: clearconfig {protection key}\r\n");
    return;
  }

  if (strcmp(argv[0], "1234") != 0) {
    shell_printf("Key unmatched.\r\n");
    return;
  }

  clear_all_config_prop_data();
  shell_printf("Config and all cal data cleared.\r\n"\
               "Do reset manually to take effect. Then do touch cal and save.\r\n");
}

static struct {
  int16_t rms[2];
  int16_t ave[2];
  int callback_count;

#if 0
  int32_t last_counter_value;
  int32_t interval_cycles;
  int32_t busy_cycles;
#endif
} stat;

int16_t rx_buffer[AUDIO_BUFFER_LEN * 2];

#ifdef ENABLED_DUMP
int16_t dump_buffer[AUDIO_BUFFER_LEN];
int16_t dump_selection = 0;
#endif

volatile int16_t wait_count = 0;

float measured[2][POINTS_COUNT][2];

#ifdef ENABLED_DUMP
static void
duplicate_buffer_to_dump(int16_t *p)
{
  if (dump_selection == 1)
    p = samp_buf;
  else if (dump_selection == 2)
    p = ref_buf;
  memcpy(dump_buffer, p, sizeof dump_buffer);
}
#endif

void i2s_end_callback(I2SDriver *i2sp, size_t offset, size_t n)
{
#if PORT_SUPPORTS_RT
  int32_t cnt_s = port_rt_get_counter_value();
  int32_t cnt_e;
#endif
  int16_t *p = &rx_buffer[offset];
  (void)i2sp;
  (void)n;

  if (wait_count > 0) {
    if (wait_count == 1) 
      dsp_process(p, n);
#ifdef ENABLED_DUMP
      duplicate_buffer_to_dump(p);
#endif
    --wait_count;
  }

#if PORT_SUPPORTS_RT
  cnt_e = port_rt_get_counter_value();
  stat.interval_cycles = cnt_s - stat.last_counter_value;
  stat.busy_cycles = cnt_e - cnt_s;
  stat.last_counter_value = cnt_s;
#endif
  stat.callback_count++;
}

static const I2SConfig i2sconfig = {
  NULL, // TX Buffer
  rx_buffer, // RX Buffer
  AUDIO_BUFFER_LEN * 2,
  NULL, // tx callback
  i2s_end_callback, // rx callback
  0, // i2scfgr
  2 // i2spr
};

VNA_SHELL_FUNCTION(cmd_data)
{
  int i;
  int sel = 0;
  float (*array)[2];
  if (argc == 1)
    sel = my_atoi(argv[0]);

  if (sel == 0 || sel == 1)
    array = measured[sel];
  else if (sel >= 2 && sel < 7)
    array = cal_data[sel-2];
  else
    goto usage;
  for (i = 0; i < sweep_points; i++) {
    if (frequencies[i] != 0)
      shell_printf("%f %f\r\n", array[i][0], array[i][1]);
  }
  return;
usage:
  shell_printf("usage: data [array]\r\n");
}

#ifdef ENABLED_DUMP
VNA_SHELL_FUNCTION(cmd_dump)
{
  int i, j;
  int len;

  if (argc == 1)
    dump_selection = my_atoi(argv[0]);

  wait_dsp(3);

  len = AUDIO_BUFFER_LEN;
  if (dump_selection == 1 || dump_selection == 2)
    len /= 2;
  for (i = 0; i < len; ) {
    for (j = 0; j < 16; j++, i++) {
      shell_printf("%04x ", 0xffff & (int)dump_buffer[i]);
    }
    shell_printf("\r\n");
  }
}
#endif

VNA_SHELL_FUNCTION(cmd_capture)
{
// read pixel count at one time (PART*2 bytes required for read buffer)
  (void)argc;
  (void)argv;
  int i, y;
#if SPI_BUFFER_SIZE < (3*320 + 1)
#error "Low size of spi_buffer for cmd_capture"
#endif
  // read 2 row pixel time (read buffer limit by 2/3 + 1 from spi_buffer size)
  for (y=0; y < 240; y+=2){
    // use uint16_t spi_buffer[2048] (defined in ili9341) for read buffer
    uint8_t *buf = (uint8_t *)spi_buffer;
    ili9341_read_memory(0, y, 320, 2, 2*320, spi_buffer);
    for (i = 0; i < 4*320; i++) {
      streamPut(shell_stream, *buf++);
    }
  }
}

#if 0
VNA_SHELL_FUNCTION(cmd_gamma)
{
  float gamma[2];
  (void)argc;
  (void)argv;
  
  pause_sweep();
  chMtxLock(&mutex);
  wait_dsp(4);  
  calculate_gamma(gamma);
  chMtxUnlock(&mutex);

  shell_printf("%d %d\r\n", gamma[0], gamma[1]);
}
#endif

static void (*sample_func)(float *gamma) = calculate_gamma;

VNA_SHELL_FUNCTION(cmd_sample)
{
  if (argc!=1) goto usage;
  //                                         0    1   2
  static const char cmd_sample_list[] = "gamma|ampl|ref";
  switch (getStringIndex(argv[0], cmd_sample_list)){
    case 0:sample_func = calculate_gamma; return;
    case 1:sample_func = fetch_amplitude; return;
    case 2:sample_func = fetch_amplitude_ref; return;
    default:break;
  }
usage:
  shell_printf("usage: sample {%s}\r\n", cmd_sample_list);
}

config_t config = {
  .magic =             CONFIG_MAGIC,
  .dac_value =         1922,
  .grid_color =        DEFAULT_GRID_COLOR,
  .menu_normal_color = DEFAULT_MENU_COLOR,
  .menu_active_color = DEFAULT_MENU_ACTIVE_COLOR,
  .trace_color =       { DEFAULT_TRACE_1_COLOR, DEFAULT_TRACE_2_COLOR, DEFAULT_TRACE_3_COLOR, DEFAULT_TRACE_4_COLOR },
//  .touch_cal =         { 693, 605, 124, 171 },  // 2.4 inch LCD panel
  .touch_cal =         { 338, 522, 153, 192 },  // 2.8 inch LCD panel
  .harmonic_freq_threshold = 300000000,
  .vbat_offset = 500
};

properties_t current_props;
properties_t *active_props = &current_props;

// NanoVNA Default settings
static const trace_t def_trace[TRACES_MAX] = {//enable, type, channel, reserved, scale, refpos
  { 1, TRC_LOGMAG, 0, 0, 10.0, NGRIDY-1 },
  { 1, TRC_LOGMAG, 1, 0, 10.0, NGRIDY-1 },
  { 1, TRC_SMITH,  0, 0, 1.0, 0 },
  { 1, TRC_PHASE,  1, 0, 90.0, NGRIDY/2 }
};

static const marker_t def_markers[MARKERS_MAX] = {
  { 1, 30, 0 }, { 0, 40, 0 }, { 0, 60, 0 }, { 0, 80, 0 }
};

// Load propeties default settings
void loadDefaultProps(void){
//Magic add on caldata_save
//current_props.magic = CONFIG_MAGIC;
  current_props._frequency0   =     50000;    // start =  50kHz
  current_props._frequency1   = 900000000;    // end   = 900MHz
  current_props._sweep_points = POINTS_COUNT;
  current_props._cal_status   = 0;
//This data not loaded by default
//current_props._frequencies[POINTS_COUNT];
//current_props._cal_data[5][POINTS_COUNT][2];
//=============================================
  current_props._electrical_delay = 0.0;
  memcpy(current_props._trace, def_trace, sizeof(def_trace));
  memcpy(current_props._markers, def_markers, sizeof(def_markers));
  current_props._velocity_factor =  0.7;
  current_props._active_marker   = 0;
  current_props._domain_mode     = 0;
  current_props._marker_smith_format = MS_RLC;
//Checksum add on caldata_save
//current_props.checksum = 0;
}

void
ensure_edit_config(void)
{
  if (active_props == &current_props)
    return;

  //memcpy(&current_props, active_props, sizeof(config_t));
  active_props = &current_props;
  // move to uncal state
  cal_status = 0;
}

#define DSP_START(delay) wait_count = delay;
#define DSP_WAIT_READY   while (wait_count) __WFI();

#define DELAY_CHANNEL_CHANGE 2

// main loop for measurement
bool sweep(bool break_on_operation)
{
  int i, delay;
  // blink LED while scanning
  palClearPad(GPIOC, GPIOC_LED);
  // Power stabilization after LED off, also align timings
  // Also touch made some
  DSP_START(1); DSP_WAIT_READY;
  for (i = 0; i < sweep_points; i++) {         // 5300
    delay = set_frequency(frequencies[i]);     // 700
    tlv320aic3204_select(0);                   // 60 CH0:REFLECT
    DSP_START(delay);                          // 1900
    //================================================
    // Place some code thats need execute while delay
    //================================================
    DSP_WAIT_READY;
    // calculate reflection coefficient
    (*sample_func)(measured[0][i]);            // 60

    tlv320aic3204_select(1);                   // 60 CH1:TRANSMISSION
    DSP_START(DELAY_CHANNEL_CHANGE);           // 1700
    //================================================
    // Place some code thats need execute while delay
    //================================================
    DSP_WAIT_READY;
    // calculate transmission coefficient
    (*sample_func)(measured[1][i]);            // 60
                                               // ======== 170 ===========
    if (cal_status & CALSTAT_APPLY)
      apply_error_term_at(i);

    if (electrical_delay != 0)
      apply_edelay_at(i);

    // back to toplevel to handle ui operation
    if (operation_requested && break_on_operation)
      return false;
  }
  // blink LED while scanning
  palSetPad(GPIOC, GPIOC_LED);
  return true;
}

VNA_SHELL_FUNCTION(cmd_scan)
{
  uint32_t start, stop;
  int16_t points = sweep_points;

  if (argc != 2 && argc != 3 && argc != 4) {
    shell_printf("usage: scan {start(Hz)} {stop(Hz)} [points [log|lin]]\r\n");
    return;
  }

  start = my_atoui(argv[0]);
  stop = my_atoui(argv[1]);
  if (start == 0 || stop == 0 || start > stop) {
      shell_printf("frequency range is invalid\r\n");
      return;
  }
  if ((argc == 3) || (argc == 4)) {
    points = my_atoi(argv[2]);
    if (points <= 0 || points > sweep_points) {
      shell_printf("sweep points exceeds range\r\n");
      return;
    }
  }
  sweep_log = 0;
  if (argc == 4) {
    if (strcmp("log",argv[3]) == 0)
      sweep_log = 1;
    else {
      shell_printf("arg 4 can only be 'log'\r\n");
      return;
    }
  }
  pause_sweep();
  chMtxLock(&mutex);
  set_frequencies(start, stop, points);
  if (cal_auto_interpolate && (cal_status & CALSTAT_APPLY))
    cal_interpolate(lastsaveid);

  sweep_mode|= SWEEP_ONCE;
  chMtxUnlock(&mutex);

  // wait finishing sweep
  while (sweep_mode&SWEEP_ONCE)
    chThdSleepMilliseconds(10);
}

static void
update_marker_index(void)
{
  int m;
  int i;
  for (m = 0; m < MARKERS_MAX; m++) {
    if (!markers[m].enabled)
      continue;
    uint32_t f = markers[m].frequency;
    uint32_t fstart = get_sweep_frequency(ST_START);
    uint32_t fstop  = get_sweep_frequency(ST_STOP);
    if (f < fstart) {
      markers[m].index = 0;
      markers[m].frequency = fstart;
    } else if (f >= fstop) {
      markers[m].index = sweep_points-1;
      markers[m].frequency = fstop;
    } else {
      for (i = 0; i < sweep_points-1; i++) {
        if (frequencies[i] <= f && f < frequencies[i+1]) {
          markers[m].index = f < (frequencies[i]/2 + frequencies[i+1]/2) ? i : i+1;
          break;
        }
      }      
    }
  }
}

void
set_lin_frequencies(uint32_t start, uint32_t stop, uint16_t points)
{
  uint32_t i;
  uint32_t step = (points - 1);
  uint32_t span = stop - start;
  uint32_t delta = span / step;
  uint32_t error = span % step;
  uint32_t f = start, df = step>>1;
  for (i = 0; i <= step; i++, f+=delta) {
    frequencies[i] = f;
    df+=error;
    if (df >=step) {
      f++;
      df-=step;
    }
  }
  // disable at out of sweep range
  for (; i < sweep_points; i++)
    frequencies[i] = 0;
}

void
set_log_frequencies(uint32_t start, uint32_t stop, uint16_t points)
{
  float logStart = logf((float)start);
  float logStop  = logf((float)stop);
  float inc = (logStop-logStart)/((float)(points));
  uint32_t i;
  for (i = 0; i <= points; i++) {
    uint32_t f = (uint32_t)(exp((double)(logStart+inc*i)));
    frequencies[i] = f;
  }
  frequencies[0] = start;
  frequencies[points] = stop;
  // disable at out of sweep range
  for (; i < sweep_points; i++)
    frequencies[i] = 0;
}

void
set_frequencies(uint32_t start, uint32_t stop, uint16_t points) {
  if (sweep_log)
    set_log_frequencies(start, stop, points);
  else
    set_lin_frequencies(start, stop, points);
}

void
update_frequencies(void)
{
  uint32_t start, stop;
  if (frequency0 < frequency1) {
    start = frequency0;
    stop = frequency1;
  } else {
    start = frequency1;
    stop = frequency0;
  }

  set_frequencies(start, stop, sweep_points);
//  operation_requested|= OP_FREQCHANGE;
  
  update_marker_index();
  
  // set grid layout
  update_grid();
}

static void
freq_mode_startstop(void)
{
  if (frequency0 > frequency1) {
    ensure_edit_config();
    uint32_t f = frequency1;
    frequency1 = frequency0;
    frequency0 = f;
  }
}

static void
freq_mode_centerspan(void)
{
  if (frequency0 <= frequency1) {
    ensure_edit_config();
    uint32_t f = frequency1;
    frequency1 = frequency0;
    frequency0 = f;
  }
}

void
set_sweep_frequency(int type, uint32_t freq)
{
  int cal_applied = cal_status & CALSTAT_APPLY;

  // Check frequency for out of bounds (minimum SPAN can be any value)
  if (type!=ST_SPAN && freq < START_MIN)
    freq = START_MIN;
  if (freq > STOP_MAX)
    freq = STOP_MAX;

  switch (type) {
  case ST_START:
    freq_mode_startstop();
    if (frequency0 != freq) {
      ensure_edit_config();
      frequency0 = freq;
      // if start > stop then make start = stop
      if (frequency1 < freq)
        frequency1 = freq;
    }
    break;
  case ST_STOP:
    freq_mode_startstop();
    if (frequency1 != freq) {
      ensure_edit_config();
      frequency1 = freq;
      // if start > stop then make start = stop
      if (frequency0 > freq)
        frequency0 = freq;
    }
    break;
  case ST_CENTER:
    freq_mode_centerspan();
    uint32_t center = FREQ_CENTER();
    if (center != freq) {
      uint32_t span = FREQ_SPAN();
      ensure_edit_config();
      if (freq < START_MIN + span/2) {
        span = (freq - START_MIN) * 2;
      }
      if (freq > STOP_MAX - span/2) {
        span = (STOP_MAX - freq) * 2;
      }
      frequency0 = freq + span/2;
      frequency1 = freq - span/2;
    }
    break;
  case ST_SPAN:
    freq_mode_centerspan();
    if (frequency0 - frequency1 != freq) {
      ensure_edit_config();
      uint32_t center = frequency0/2 + frequency1/2;
      if (center < START_MIN + freq/2) {
        center = START_MIN + freq/2;
      }
      if (center > STOP_MAX - freq/2) {
        center = STOP_MAX - freq/2;
      }
      frequency1 = center - freq/2;
      frequency0 = center + freq/2;
    }
    break;
  case ST_CW:
    freq_mode_centerspan();
    if (frequency0 != freq || frequency1 != freq) {
      ensure_edit_config();
      frequency0 = freq;
      frequency1 = freq;
    }
    break;
  }
  update_frequencies();
  if (cal_auto_interpolate && cal_applied)
    cal_interpolate(lastsaveid);
}

uint32_t
get_sweep_frequency(int type)
{
  if (frequency0 <= frequency1) {
    switch (type) {
    case ST_START: return frequency0;
    case ST_STOP: return frequency1;
    case ST_CENTER: return frequency0/2 + frequency1/2;
    case ST_SPAN: return frequency1 - frequency0;
    case ST_CW: return frequency0/2 + frequency1/2;
    }
  } else {
    switch (type) {
    case ST_START: return frequency1;
    case ST_STOP: return frequency0;
    case ST_CENTER: return frequency0/2 + frequency1/2;
    case ST_SPAN: return frequency0 - frequency1;
    case ST_CW: return frequency0/2 + frequency1/2;
    }
  }
  return 0;
}

VNA_SHELL_FUNCTION(cmd_sweep)
{
  if (argc == 0) {
    shell_printf("%d %d %d\r\n", frequency0, frequency1, sweep_points);
    return;
  } else if (argc > 3) {
    goto usage;
  }
  uint32_t value0 = 0;
  uint32_t value1 = 0;
  if (argc >=1) value0 = my_atoui(argv[0]);
  if (argc >=2) value1 = my_atoui(argv[1]);
#if MAX_FREQ_TYPE!=5
#error "Sweep mode possibly changed, check cmd_sweep function"
#endif
  // Parse sweep {start|stop|center|span|cw} {freq(Hz)}
  // get enum ST_START, ST_STOP, ST_CENTER, ST_SPAN, ST_CW
  static const char sweep_cmd[] = "start|stop|center|span|cw";
  if (argc == 2 && value0 == 0) {
    int type = getStringIndex(argv[0], sweep_cmd);
    if (type == -1)
      goto usage;
    set_sweep_frequency(type, value1);
    return;
  }
  //  Parse sweep {start(Hz)} [stop(Hz)]
  if (value0)
    set_sweep_frequency(ST_START, value0);
  if (value1)
    set_sweep_frequency(ST_STOP, value1);
  return;
usage:
  shell_printf("usage: sweep {start(Hz)} [stop(Hz)]\r\n"\
               "\tsweep {%s} {freq(Hz)}\r\n", sweep_cmd);
}


static void
eterm_set(int term, float re, float im)
{
  int i;
  for (i = 0; i < sweep_points; i++) {
    cal_data[term][i][0] = re;
    cal_data[term][i][1] = im;
  }
}

static void
eterm_copy(int dst, int src)
{
  memcpy(cal_data[dst], cal_data[src], sizeof cal_data[dst]);
}

#if 0
const struct open_model {
  float c0;
  float c1;
  float c2;
  float c3;
} open_model = { 50, 0, -300, 27 };
#endif

#if 0
static void
adjust_ed(void)
{
  int i;
  for (i = 0; i < sweep_points; i++) {
    // z=1/(jwc*z0) = 1/(2*pi*f*c*z0)  Note: normalized with Z0
    // s11ao = (z-1)/(z+1) = (1-1/z)/(1+1/z) = (1-jwcz0)/(1+jwcz0)
    // prepare 1/s11ao to avoid dividing complex
    float c = 1000e-15;
    float z0 = 50;
    //float z = 2 * M_PI * frequencies[i] * c * z0;
    float z = 0.02;
    cal_data[ETERM_ED][i][0] += z;
  }
}
#endif

static void
eterm_calc_es(void)
{
  int i;
  for (i = 0; i < sweep_points; i++) {
    // z=1/(jwc*z0) = 1/(2*pi*f*c*z0)  Note: normalized with Z0
    // s11ao = (z-1)/(z+1) = (1-1/z)/(1+1/z) = (1-jwcz0)/(1+jwcz0)
    // prepare 1/s11ao for effeiciency
    float c = 50e-15;
    //float c = 1.707e-12;
    float z0 = 50;
    float z = 2 * M_PI * frequencies[i] * c * z0;
    float sq = 1 + z*z;
    float s11aor = (1 - z*z) / sq;
    float s11aoi = 2*z / sq;

    // S11mo’= S11mo - Ed
    // S11ms’= S11ms - Ed
    float s11or = cal_data[CAL_OPEN][i][0] - cal_data[ETERM_ED][i][0];
    float s11oi = cal_data[CAL_OPEN][i][1] - cal_data[ETERM_ED][i][1];
    float s11sr = cal_data[CAL_SHORT][i][0] - cal_data[ETERM_ED][i][0];
    float s11si = cal_data[CAL_SHORT][i][1] - cal_data[ETERM_ED][i][1];
    // Es = (S11mo'/s11ao + S11ms’)/(S11mo' - S11ms’)
    float numr = s11sr + s11or * s11aor - s11oi * s11aoi;
    float numi = s11si + s11oi * s11aor + s11or * s11aoi;
    float denomr = s11or - s11sr;
    float denomi = s11oi - s11si;
    sq = denomr*denomr+denomi*denomi;
    cal_data[ETERM_ES][i][0] = (numr*denomr + numi*denomi)/sq;
    cal_data[ETERM_ES][i][1] = (numi*denomr - numr*denomi)/sq;
  }
  cal_status &= ~CALSTAT_OPEN;
  cal_status |= CALSTAT_ES;
}

static void
eterm_calc_er(int sign)
{
  int i;
  for (i = 0; i < sweep_points; i++) {
    // Er = sign*(1-sign*Es)S11ms'
    float s11sr = cal_data[CAL_SHORT][i][0] - cal_data[ETERM_ED][i][0];
    float s11si = cal_data[CAL_SHORT][i][1] - cal_data[ETERM_ED][i][1];
    float esr = cal_data[ETERM_ES][i][0];
    float esi = cal_data[ETERM_ES][i][1];
    if (sign > 0) {
      esr = -esr;
      esi = -esi;
    }
    esr = 1 + esr;
    float err = esr * s11sr - esi * s11si;
    float eri = esr * s11si + esi * s11sr;
    if (sign < 0) {
      err = -err;
      eri = -eri;
    }
    cal_data[ETERM_ER][i][0] = err;
    cal_data[ETERM_ER][i][1] = eri;
  }
  cal_status &= ~CALSTAT_SHORT;
  cal_status |= CALSTAT_ER;
}

// CAUTION: Et is inversed for efficiency
static void
eterm_calc_et(void)
{
  int i;
  for (i = 0; i < sweep_points; i++) {
    // Et = 1/(S21mt - Ex)
    float etr = cal_data[CAL_THRU][i][0] - cal_data[CAL_ISOLN][i][0];
    float eti = cal_data[CAL_THRU][i][1] - cal_data[CAL_ISOLN][i][1];
    float sq = etr*etr + eti*eti;
    float invr = etr / sq;
    float invi = -eti / sq;
    cal_data[ETERM_ET][i][0] = invr;
    cal_data[ETERM_ET][i][1] = invi;
  }
  cal_status &= ~CALSTAT_THRU;
  cal_status |= CALSTAT_ET;
}

#if 0
void apply_error_term(void)
{
  int i;
  for (i = 0; i < sweep_points; i++) {
    // S11m' = S11m - Ed
    // S11a = S11m' / (Er + Es S11m')
    float s11mr = measured[0][i][0] - cal_data[ETERM_ED][i][0];
    float s11mi = measured[0][i][1] - cal_data[ETERM_ED][i][1];
    float err = cal_data[ETERM_ER][i][0] + s11mr * cal_data[ETERM_ES][i][0] - s11mi * cal_data[ETERM_ES][i][1];
    float eri = cal_data[ETERM_ER][i][1] + s11mr * cal_data[ETERM_ES][i][1] + s11mi * cal_data[ETERM_ES][i][0];
    float sq = err*err + eri*eri;
    float s11ar = (s11mr * err + s11mi * eri) / sq;
    float s11ai = (s11mi * err - s11mr * eri) / sq;
    measured[0][i][0] = s11ar;
    measured[0][i][1] = s11ai;

    // CAUTION: Et is inversed for efficiency
    // S21m' = S21m - Ex
    // S21a = S21m' (1-EsS11a)Et
    float s21mr = measured[1][i][0] - cal_data[ETERM_EX][i][0];
    float s21mi = measured[1][i][1] - cal_data[ETERM_EX][i][1];
    float esr = 1 - (cal_data[ETERM_ES][i][0] * s11ar - cal_data[ETERM_ES][i][1] * s11ai);
    float esi = - (cal_data[ETERM_ES][i][1] * s11ar + cal_data[ETERM_ES][i][0] * s11ai);
    float etr = esr * cal_data[ETERM_ET][i][0] - esi * cal_data[ETERM_ET][i][1];
    float eti = esr * cal_data[ETERM_ET][i][1] + esi * cal_data[ETERM_ET][i][0];
    float s21ar = s21mr * etr - s21mi * eti;
    float s21ai = s21mi * etr + s21mr * eti;
    measured[1][i][0] = s21ar;
    measured[1][i][1] = s21ai;
  }
}
#endif

static void apply_error_term_at(int i)
{
    // S11m' = S11m - Ed
    // S11a = S11m' / (Er + Es S11m')
    float s11mr = measured[0][i][0] - cal_data[ETERM_ED][i][0];
    float s11mi = measured[0][i][1] - cal_data[ETERM_ED][i][1];
    float err = cal_data[ETERM_ER][i][0] + s11mr * cal_data[ETERM_ES][i][0] - s11mi * cal_data[ETERM_ES][i][1];
    float eri = cal_data[ETERM_ER][i][1] + s11mr * cal_data[ETERM_ES][i][1] + s11mi * cal_data[ETERM_ES][i][0];
    float sq = err*err + eri*eri;
    float s11ar = (s11mr * err + s11mi * eri) / sq;
    float s11ai = (s11mi * err - s11mr * eri) / sq;
    measured[0][i][0] = s11ar;
    measured[0][i][1] = s11ai;

    // CAUTION: Et is inversed for efficiency
    // S21m' = S21m - Ex
    // S21a = S21m' (1-EsS11a)Et
    float s21mr = measured[1][i][0] - cal_data[ETERM_EX][i][0];
    float s21mi = measured[1][i][1] - cal_data[ETERM_EX][i][1];
    float esr = 1 - (cal_data[ETERM_ES][i][0] * s11ar - cal_data[ETERM_ES][i][1] * s11ai);
    float esi = - (cal_data[ETERM_ES][i][1] * s11ar + cal_data[ETERM_ES][i][0] * s11ai);
    float etr = esr * cal_data[ETERM_ET][i][0] - esi * cal_data[ETERM_ET][i][1];
    float eti = esr * cal_data[ETERM_ET][i][1] + esi * cal_data[ETERM_ET][i][0];
    float s21ar = s21mr * etr - s21mi * eti;
    float s21ai = s21mi * etr + s21mr * eti;
    measured[1][i][0] = s21ar;
    measured[1][i][1] = s21ai;
}

static void apply_edelay_at(int i)
{
  float w = 2 * M_PI * electrical_delay * frequencies[i] * 1E-12;
  float s = sin(w);
  float c = cos(w);
  float real = measured[0][i][0];
  float imag = measured[0][i][1];
  measured[0][i][0] = real * c - imag * s;
  measured[0][i][1] = imag * c + real * s;
  real = measured[1][i][0];
  imag = measured[1][i][1];
  measured[1][i][0] = real * c - imag * s;
  measured[1][i][1] = imag * c + real * s;
}

void
cal_collect(int type)
{
  ensure_edit_config();

  switch (type) {
  case CAL_LOAD:
    cal_status |= CALSTAT_LOAD;
    memcpy(cal_data[CAL_LOAD], measured[0], sizeof measured[0]);
    break;

  case CAL_OPEN:
    cal_status |= CALSTAT_OPEN;
    cal_status &= ~(CALSTAT_ES|CALSTAT_APPLY);
    memcpy(cal_data[CAL_OPEN], measured[0], sizeof measured[0]);
    break;

  case CAL_SHORT:
    cal_status |= CALSTAT_SHORT;
    cal_status &= ~(CALSTAT_ER|CALSTAT_APPLY);
    memcpy(cal_data[CAL_SHORT], measured[0], sizeof measured[0]);
    break;

  case CAL_THRU:
    cal_status |= CALSTAT_THRU;
    memcpy(cal_data[CAL_THRU], measured[1], sizeof measured[0]);
    break;

  case CAL_ISOLN:
    cal_status |= CALSTAT_ISOLN;
    memcpy(cal_data[CAL_ISOLN], measured[1], sizeof measured[0]);
    break;
  }
  redraw_request |= REDRAW_CAL_STATUS;
}

void
cal_done(void)
{
  ensure_edit_config();
  if (!(cal_status & CALSTAT_LOAD))
    eterm_set(ETERM_ED, 0.0, 0.0);
  //adjust_ed();
  if ((cal_status & CALSTAT_SHORT) && (cal_status & CALSTAT_OPEN)) {
    eterm_calc_es();
    eterm_calc_er(-1);
  } else if (cal_status & CALSTAT_OPEN) {
    eterm_copy(CAL_SHORT, CAL_OPEN);
    eterm_set(ETERM_ES, 0.0, 0.0);
    eterm_calc_er(1);
  } else if (cal_status & CALSTAT_SHORT) {
    eterm_set(ETERM_ES, 0.0, 0.0);
    cal_status &= ~CALSTAT_SHORT;
    eterm_calc_er(-1);
  } else {
    eterm_set(ETERM_ER, 1.0, 0.0);
    eterm_set(ETERM_ES, 0.0, 0.0);
  }
    
  if (!(cal_status & CALSTAT_ISOLN))
    eterm_set(ETERM_EX, 0.0, 0.0);
  if (cal_status & CALSTAT_THRU) {
    eterm_calc_et();
  } else {
    eterm_set(ETERM_ET, 1.0, 0.0);
  }

  cal_status |= CALSTAT_APPLY;
  redraw_request |= REDRAW_CAL_STATUS;
}

static void
cal_interpolate(int s)
{
  const properties_t *src = caldata_ref(s);
  int i, j;
  int eterm;
  if (src == NULL)
    return;

  ensure_edit_config();

  // lower than start freq of src range
  for (i = 0; i < sweep_points; i++) {
    if (frequencies[i] >= src->_frequencies[0])
      break;

    // fill cal_data at head of src range
    for (eterm = 0; eterm < 5; eterm++) {
      cal_data[eterm][i][0] = src->_cal_data[eterm][0][0];
      cal_data[eterm][i][1] = src->_cal_data[eterm][0][1];
    }
  }

  j = 0;
  for (; i < sweep_points; i++) {
    uint32_t f = frequencies[i];

    for (; j < sweep_points-1; j++) {
      if (src->_frequencies[j] <= f && f < src->_frequencies[j+1]) {
        // found f between freqs at j and j+1
        float k1 = (float)(f - src->_frequencies[j])
                        / (src->_frequencies[j+1] - src->_frequencies[j]);
        
        // avoid glitch between freqs in different harmonics mode
        if (IS_HARMONIC_MODE(src->_frequencies[j]) != IS_HARMONIC_MODE(src->_frequencies[j+1])) {
          // assume f[j] < f[j+1]
          k1 = IS_HARMONIC_MODE(f) ? 1.0 : 0.0;
        }

        float k0 = 1.0 - k1;
        for (eterm = 0; eterm < 5; eterm++) {
          cal_data[eterm][i][0] = src->_cal_data[eterm][j][0] * k0 + src->_cal_data[eterm][j+1][0] * k1;
          cal_data[eterm][i][1] = src->_cal_data[eterm][j][1] * k0 + src->_cal_data[eterm][j+1][1] * k1;
        }
        break;
      }
    }
    if (j == sweep_points-1)
      break;
  }
  
  // upper than end freq of src range
  for (; i < sweep_points; i++) {
    // fill cal_data at tail of src
    for (eterm = 0; eterm < 5; eterm++) {
      cal_data[eterm][i][0] = src->_cal_data[eterm][sweep_points-1][0];
      cal_data[eterm][i][1] = src->_cal_data[eterm][sweep_points-1][1];
    }
  }
    
  cal_status |= src->_cal_status | CALSTAT_APPLY | CALSTAT_INTERPOLATED;
  redraw_request |= REDRAW_CAL_STATUS;
}

VNA_SHELL_FUNCTION(cmd_cal)
{
  static const char *items[] = { "load", "open", "short", "thru", "isoln", "Es", "Er", "Et", "cal'ed" };

  if (argc == 0) {
    int i;
    for (i = 0; i < 9; i++) {
      if (cal_status & (1<<i))
        shell_printf("%s ", items[i]);
    }
    shell_printf("\r\n");
    return;
  }
  redraw_request|=REDRAW_CAL_STATUS;
  //                                     0    1     2    3     4    5  6   7     8    9 10
  static const char cmd_cal_list[] = "load|open|short|thru|isoln|done|on|off|reset|data|in";
  switch (getStringIndex(argv[0], cmd_cal_list)){
    case  0:cal_collect(CAL_LOAD ); return;
    case  1:cal_collect(CAL_OPEN ); return;
    case  2:cal_collect(CAL_SHORT); return;
    case  3:cal_collect(CAL_THRU ); return;
    case  4:cal_collect(CAL_ISOLN); return;
    case  5:cal_done(); return;
    case  6:cal_status|= CALSTAT_APPLY;return;
    case  7:cal_status&=~CALSTAT_APPLY;return;
    case  8:cal_status = 0;            return;
    case  9:
      shell_printf("%f %f\r\n", cal_data[CAL_LOAD ][0][0], cal_data[CAL_LOAD ][0][1]);
      shell_printf("%f %f\r\n", cal_data[CAL_OPEN ][0][0], cal_data[CAL_OPEN ][0][1]);
      shell_printf("%f %f\r\n", cal_data[CAL_SHORT][0][0], cal_data[CAL_SHORT][0][1]);
      shell_printf("%f %f\r\n", cal_data[CAL_THRU ][0][0], cal_data[CAL_THRU ][0][1]);
      shell_printf("%f %f\r\n", cal_data[CAL_ISOLN][0][0], cal_data[CAL_ISOLN][0][1]);
      return;
    case 10:
      cal_interpolate((argc > 1) ? my_atoi(argv[1]) : 0);
      return;
    default:break;
  }
  shell_printf("usage: cal [%s]\r\n", cmd_cal_list);
}

VNA_SHELL_FUNCTION(cmd_save)
{
  if (argc != 1)
    goto usage;

  int id = my_atoi(argv[0]);
  if (id < 0 || id >= SAVEAREA_MAX)
    goto usage;
  caldata_save(id);
  redraw_request |= REDRAW_CAL_STATUS;
  return;

 usage:
  shell_printf("save {id}\r\n");
}

VNA_SHELL_FUNCTION(cmd_recall)
{
  if (argc != 1)
    goto usage;

  int id = my_atoi(argv[0]);
  if (id < 0 || id >= SAVEAREA_MAX)
    goto usage;
  // Check for success
  if (caldata_recall(id) == -1)
    shell_printf("Err, default load\r\n");
  update_frequencies();
  redraw_request |= REDRAW_CAL_STATUS;
  return;
 usage:
  shell_printf("recall {id}\r\n");
}

static const struct {
  const char *name;
  uint16_t refpos;
  float scale_unit;
} trace_info[] = {
  { "LOGMAG", NGRIDY-1,  10.0 },
  { "PHASE",  NGRIDY/2,  90.0 },
  { "DELAY",  NGRIDY/2,  1e-9 },
  { "SMITH",         0,  1.00 },
  { "POLAR",         0,  1.00 },
  { "LINEAR",        0,  0.125},
  { "SWR",           0,  0.25 },
  { "REAL",   NGRIDY/2,  0.25 },
  { "IMAG",   NGRIDY/2,  0.25 },
  { "R",      NGRIDY/2, 100.0 },
  { "X",      NGRIDY/2, 100.0 }
};

static const char * const trc_channel_name[] = {
  "CH0", "CH1"
};

const char *get_trace_typename(int t)
{
  return trace_info[trace[t].type].name;
}

void set_trace_type(int t, int type)
{
  int enabled = type != TRC_OFF;
  int force = FALSE;

  if (trace[t].enabled != enabled) {
    trace[t].enabled = enabled;
    force = TRUE;
  }
  if (trace[t].type != type) {
    trace[t].type = type;
    // Set default trace refpos
    trace[t].refpos = trace_info[type].refpos;
    // Set default trace scale
    trace[t].scale  = trace_info[type].scale_unit;
    force = TRUE;
  }    
  if (force) {
    plot_into_index(measured);
    force_set_markmap();
  }
}

void set_trace_channel(int t, int channel)
{
  if (trace[t].channel != channel) {
    trace[t].channel = channel;
    force_set_markmap();
  }
}

void set_trace_scale(int t, float scale)
{
  if (trace[t].scale != scale) {
    trace[t].scale = scale;
    force_set_markmap();
  }
}

float get_trace_scale(int t)
{
  return trace[t].scale;
}

void set_trace_refpos(int t, float refpos)
{
  if (trace[t].refpos != refpos) {
    trace[t].refpos = refpos;
    force_set_markmap();
  }
}

float get_trace_refpos(int t)
{
  return trace[t].refpos;
}

VNA_SHELL_FUNCTION(cmd_trace)
{
  int t;
  if (argc == 0) {
    for (t = 0; t < TRACES_MAX; t++) {
      if (trace[t].enabled) {
        const char *type = get_trace_typename(t);
        const char *channel = trc_channel_name[trace[t].channel];
        float scale = get_trace_scale(t);
        float refpos = get_trace_refpos(t);
        shell_printf("%d %s %s %f %f\r\n", t, type, channel, scale, refpos);
      }
    }
    return;
  } 

  if (strcmp(argv[0], "all") == 0 &&
      argc > 1 && strcmp(argv[1], "off") == 0) {
  for (t = 0; t < TRACES_MAX; t++)
      set_trace_type(t, TRC_OFF);
    goto exit;
  }

  t = my_atoi(argv[0]);
  if (t < 0 || t >= TRACES_MAX)
    goto usage;
  if (argc == 1) {
    const char *type = get_trace_typename(t);
    const char *channel = trc_channel_name[trace[t].channel];
    shell_printf("%d %s %s\r\n", t, type, channel);
    return;
  }
#if MAX_TRACE_TYPE!=12
#error "Trace type enum possibly changed, check cmd_trace function"
#endif
  // enum TRC_LOGMAG, TRC_PHASE, TRC_DELAY, TRC_SMITH, TRC_POLAR, TRC_LINEAR, TRC_SWR, TRC_REAL, TRC_IMAG, TRC_R, TRC_X, TRC_OFF
  static const char cmd_type_list[] = "logmag|phase|delay|smith|polar|linear|swr|real|imag|r|x|off";
  int type = getStringIndex(argv[1], cmd_type_list);
  if (type >= 0){
    set_trace_type(t, type);
    goto check_ch_num;
  }
  //                                            0      1
  static const char cmd_scale_ref_list[] = "scale|refpos";
  if (argc >= 3){
    switch (getStringIndex(argv[1], cmd_scale_ref_list)){
      case 0:
        //trace[t].scale = my_atof(argv[2]);
        set_trace_scale(t, my_atof(argv[2]));
        goto exit;
      case 1:
        //trace[t].refpos = my_atof(argv[2]);
        set_trace_refpos(t, my_atof(argv[2]));
        goto exit;
      default:
        goto usage;
    }
  }
check_ch_num:
  if (argc > 2) {
    int src = my_atoi(argv[2]);
    if (src != 0 && src != 1)
      goto usage;
    trace[t].channel = src;
  }
exit:
  return;
usage:
  shell_printf("trace {0|1|2|3|all} [%s] [src]\r\n"\
               "trace {0|1|2|3} {%s} {value}\r\n", cmd_type_list, cmd_scale_ref_list);
}


void set_electrical_delay(float picoseconds)
{
  if (electrical_delay != picoseconds) {
    electrical_delay = picoseconds;
    force_set_markmap();
  }
  redraw_request |= REDRAW_MARKER;
}

float get_electrical_delay(void)
{
  return electrical_delay;
}

VNA_SHELL_FUNCTION(cmd_edelay)
{
  if (argc == 0) {
    shell_printf("%f\r\n", electrical_delay);
    return;
  }
  if (argc > 0) {
    set_electrical_delay(my_atof(argv[0]));
  }
}


VNA_SHELL_FUNCTION(cmd_marker)
{
  int t;
  if (argc == 0) {
    for (t = 0; t < MARKERS_MAX; t++) {
      if (markers[t].enabled) {
        shell_printf("%d %d %d\r\n", t+1, markers[t].index, markers[t].frequency);
      }
    }
    return;
  }
  redraw_request |= REDRAW_MARKER;
  if (strcmp(argv[0], "off") == 0) {
    active_marker = -1;
    for (t = 0; t < MARKERS_MAX; t++)
      markers[t].enabled = FALSE;
    return;
  }
  t = my_atoi(argv[0])-1;
  if (t < 0 || t >= MARKERS_MAX)
    goto usage;
  if (argc == 1) {
    shell_printf("%d %d %d\r\n", t+1, markers[t].index, frequency);
    active_marker = t;
    // select active marker
    markers[t].enabled = TRUE;
    return;
  }
  static const char cmd_marker_list[] = "on|off";
  switch (getStringIndex(argv[1], cmd_marker_list)){
    case 0: markers[t].enabled = TRUE; active_marker = t; return;
    case 1: markers[t].enabled =FALSE; if (active_marker == t) active_marker = -1; return;
    default:
      // select active marker and move to index
      markers[t].enabled = TRUE;
      int index = my_atoi(argv[1]);
      markers[t].index = index;
      markers[t].frequency = frequencies[index];
      active_marker = t;
      return;
  }
 usage:
  shell_printf("marker [n] [%s|{index}]\r\n", cmd_marker_list);
}

VNA_SHELL_FUNCTION(cmd_touchcal)
{
  (void)argc;
  (void)argv;
  //extern int16_t touch_cal[4];
  int i;

  shell_printf("first touch upper left, then lower right...");
  touch_cal_exec();
  shell_printf("done\r\n");

  shell_printf("touch cal params: ");
  for (i = 0; i < 4; i++) {
    shell_printf("%d ", config.touch_cal[i]);
  }
  shell_printf("\r\n");
}

VNA_SHELL_FUNCTION(cmd_touchtest)
{
  (void)argc;
  (void)argv;
  do {
    touch_draw_test();
  } while(argc);
}

VNA_SHELL_FUNCTION(cmd_frequencies)
{
  int i;
  (void)argc;
  (void)argv;
  for (i = 0; i < sweep_points; i++) {
    if (frequencies[i] != 0)
      shell_printf("%d\r\n", frequencies[i]);
  }
}

static void
set_domain_mode(int mode) // accept DOMAIN_FREQ or DOMAIN_TIME
{
  if (mode != (domain_mode & DOMAIN_MODE)) {
    domain_mode = (domain_mode & ~DOMAIN_MODE) | (mode & DOMAIN_MODE);
    redraw_request |= REDRAW_FREQUENCY;
    uistat.lever_mode = LM_MARKER;
  }
}

static void
set_timedomain_func(int func) // accept TD_FUNC_LOWPASS_IMPULSE, TD_FUNC_LOWPASS_STEP or TD_FUNC_BANDPASS
{
  domain_mode = (domain_mode & ~TD_FUNC) | (func & TD_FUNC);
}

static void
set_timedomain_window(int func) // accept TD_WINDOW_MINIMUM/TD_WINDOW_NORMAL/TD_WINDOW_MAXIMUM
{
  domain_mode = (domain_mode & ~TD_WINDOW) | (func & TD_WINDOW);
}

VNA_SHELL_FUNCTION(cmd_transform)
{
  int i;
  if (argc == 0) {
    goto usage;
  }
  //                                         0   1       2    3        4       5      6       7
  static const char cmd_transform_list[] = "on|off|impulse|step|bandpass|minimum|normal|maximum";
  for (i = 0; i < argc; i++) {
    switch (getStringIndex(argv[i], cmd_transform_list)){
      case 0:set_domain_mode(DOMAIN_TIME);return;
      case 1:set_domain_mode(DOMAIN_FREQ);return;
      case 2:set_timedomain_func(TD_FUNC_LOWPASS_IMPULSE);return;
      case 3:set_timedomain_func(TD_FUNC_LOWPASS_STEP);return;
      case 4:set_timedomain_func(TD_FUNC_BANDPASS);return;
      case 5:set_timedomain_window(TD_WINDOW_MINIMUM);return;
      case 6:set_timedomain_window(TD_WINDOW_NORMAL);return;
      case 7:set_timedomain_window(TD_WINDOW_MAXIMUM);return;
      default: goto usage;
    }
  }
  return;
usage:
  shell_printf("usage: transform {%s} [...]\r\n", cmd_transform_list);
}

VNA_SHELL_FUNCTION(cmd_test)
{
  (void)argc;
  (void)argv;

#if 0
  int i;
  for (i = 0; i < 100; i++) {
    palClearPad(GPIOC, GPIOC_LED);
    set_frequency(10000000);
    palSetPad(GPIOC, GPIOC_LED);
    chThdSleepMilliseconds(50);

    palClearPad(GPIOC, GPIOC_LED);
    set_frequency(90000000);
    palSetPad(GPIOC, GPIOC_LED);
    chThdSleepMilliseconds(50);
  }
#endif

#if 0
  int i;
  int mode = 0;
  if (argc >= 1)
    mode = my_atoi(argv[0]);

  for (i = 0; i < 20; i++) {
    palClearPad(GPIOC, GPIOC_LED);
    ili9341_test(mode);
    palSetPad(GPIOC, GPIOC_LED);
    chThdSleepMilliseconds(50);
  }
#endif

#if 0
  //extern adcsample_t adc_samples[2];
  //shell_printf("adc: %d %d\r\n", adc_samples[0], adc_samples[1]);
  int i;
  int x, y;
  for (i = 0; i < 50; i++) {
    test_touch(&x, &y);
    shell_printf("adc: %d %d\r\n", x, y);
    chThdSleepMilliseconds(200);
  }
  //extern int touch_x, touch_y;
  //shell_printf("adc: %d %d\r\n", touch_x, touch_y);
#endif

  while (argc > 1) {
    int x, y;
    touch_position(&x, &y);
    shell_printf("touch: %d %d\r\n", x, y);
    chThdSleepMilliseconds(200);
  }
}

VNA_SHELL_FUNCTION(cmd_gain)
{
  int rvalue;
  int lvalue = 0;
  if (argc != 1 && argc != 2) {
    shell_printf("usage: gain {lgain(0-95)} [rgain(0-95)]\r\n");
    return;
  }
  rvalue = my_atoi(argv[0]);
  if (argc == 2) 
    lvalue = my_atoi(argv[1]);
  tlv320aic3204_set_gain(lvalue, rvalue);
}

VNA_SHELL_FUNCTION(cmd_port)
{
  int port;
  if (argc != 1) {
    shell_printf("usage: port {0:TX 1:RX}\r\n");
    return;
  }
  port = my_atoi(argv[0]);
  tlv320aic3204_select(port);
}

VNA_SHELL_FUNCTION(cmd_stat)
{
  int16_t *p = &rx_buffer[0];
  int32_t acc0, acc1;
  int32_t ave0, ave1;
  int32_t count = AUDIO_BUFFER_LEN;
  int i;
  (void)argc;
  (void)argv;
  acc0 = acc1 = 0;
  for (i = 0; i < AUDIO_BUFFER_LEN*2; i += 2) {
    acc0 += p[i];
    acc1 += p[i+1];
  }
  ave0 = acc0 / count;
  ave1 = acc1 / count;
  acc0 = acc1 = 0;
  for (i = 0; i < AUDIO_BUFFER_LEN*2; i += 2) {
    acc0 += (p[i] - ave0)*(p[i] - ave0);
    acc1 += (p[i+1] - ave1)*(p[i+1] - ave1);
  }
  stat.rms[0] = sqrtf(acc0 / count);
  stat.rms[1] = sqrtf(acc1 / count);
  stat.ave[0] = ave0;
  stat.ave[1] = ave1;

  shell_printf("average: %d %d\r\n", stat.ave[0], stat.ave[1]);
  shell_printf("rms: %d %d\r\n", stat.rms[0], stat.rms[1]);
  shell_printf("callback count: %d\r\n", stat.callback_count);
  //shell_printf("interval cycle: %d\r\n", stat.interval_cycles);
  //shell_printf("busy cycle: %d\r\n", stat.busy_cycles);
  //shell_printf("load: %d\r\n", stat.busy_cycles * 100 / stat.interval_cycles);
//  extern int awd_count;
//  shell_printf("awd: %d\r\n", awd_count);
}


#ifndef VERSION
#define VERSION "unknown"
#endif

const char NANOVNA_VERSION[] = VERSION;

VNA_SHELL_FUNCTION(cmd_version)
{
  (void)argc;
  (void)argv;
  shell_printf("%s\r\n", NANOVNA_VERSION);
}

VNA_SHELL_FUNCTION(cmd_vbat)
{
  (void)argc;
  (void)argv;
  shell_printf("%d mV\r\n", adc_vbat_read());
}

#ifdef ENABLE_VBAT_OFFSET_COMMAND
VNA_SHELL_FUNCTION(cmd_vbat_offset)
{
  if (argc != 1) {
    shell_printf("%d\r\n", config.vbat_offset);
    return;
  }
  config.vbat_offset = (int16_t)my_atoi(argv[0]);
}
#endif

#ifdef ENABLE_INFO_COMMAND
VNA_SHELL_FUNCTION(cmd_info)
{
  (void)argc;
  (void)argv;
  int i=0;
  while (info_about[i])
    shell_printf("%s\r\n", info_about[i++]);
}
#endif

#ifdef ENABLE_THREADS_COMMAND
#if CH_CFG_USE_REGISTRY == FALSE
#error "Threads Requite enabled CH_CFG_USE_REGISTRY in chconf.h"
#endif
VNA_SHELL_FUNCTION(cmd_threads) {
  static const char *states[] = {CH_STATE_NAMES};
  thread_t *tp;
  (void)argc;
  (void)argv;
  shell_printf("stklimit|   stack|stk free|    addr|refs|prio|    state|        name"VNA_SHELL_NEWLINE_STR);
  tp = chRegFirstThread();
  do {
    uint32_t max_stack_use = 0U;
#if (CH_DBG_ENABLE_STACK_CHECK == TRUE) || (CH_CFG_USE_DYNAMIC == TRUE)
    uint32_t stklimit = (uint32_t)tp->wabase;
#if CH_DBG_FILL_THREADS == TRUE
    uint8_t *p = (uint8_t *)tp->wabase; while(p[max_stack_use]==CH_DBG_STACK_FILL_VALUE) max_stack_use++;
#endif
#else
    uint32_t stklimit = 0U;
#endif
    shell_printf("%08x|%08x|%08x|%08x|%4u|%4u|%9s|%12s"VNA_SHELL_NEWLINE_STR,
             stklimit, (uint32_t)tp->ctx.sp, max_stack_use, (uint32_t)tp,
             (uint32_t)tp->refs - 1, (uint32_t)tp->prio, states[tp->state],
             tp->name == NULL ? "" : tp->name);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
}
#endif

//=============================================================================
VNA_SHELL_FUNCTION(cmd_help);

#pragma pack(push, 2)
typedef struct {
  const char           *sc_name;
  vna_shellcmd_t    sc_function;
  uint16_t flags;
} VNAShellCommand;
#pragma pack(pop)

// Some commands can executed only if process thread not in main cycle
#define CMD_WAIT_MUTEX  1
static const VNAShellCommand commands[] =
{
    {"version"     , cmd_version     , 0},
    {"reset"       , cmd_reset       , 0},
    {"freq"        , cmd_freq        , CMD_WAIT_MUTEX},
    {"offset"      , cmd_offset      , 0},
#ifdef ENABLE_TIME_COMMAND
    {"time"        , cmd_time        , 0},
#endif
    {"dac"         , cmd_dac         , 0},
    {"saveconfig"  , cmd_saveconfig  , 0},
    {"clearconfig" , cmd_clearconfig , 0},
    {"data"        , cmd_data        , CMD_WAIT_MUTEX},
#ifdef ENABLED_DUMP
    {"dump"        , cmd_dump        , 0},
#endif
    {"frequencies" , cmd_frequencies , 0},
    {"port"        , cmd_port        , 0},
    {"stat"        , cmd_stat        , 0},
    {"gain"        , cmd_gain        , 0},
    {"power"       , cmd_power       , 0},
    {"sample"      , cmd_sample      , 0},
//  {"gamma"       , cmd_gamma       , 0},
    {"scan"        , cmd_scan        , 0}, // Wait mutex hardcoded in cmd, need wait one sweep manually
    {"sweep"       , cmd_sweep       , 0},
    {"test"        , cmd_test        , 0},
    {"touchcal"    , cmd_touchcal    , CMD_WAIT_MUTEX},
    {"touchtest"   , cmd_touchtest   , CMD_WAIT_MUTEX},
    {"pause"       , cmd_pause       , 0},
    {"resume"      , cmd_resume      , 0},
    {"cal"         , cmd_cal         , CMD_WAIT_MUTEX},
    {"save"        , cmd_save        , 0},
    {"recall"      , cmd_recall      , CMD_WAIT_MUTEX},
    {"trace"       , cmd_trace       , 0},
    {"marker"      , cmd_marker      , 0},
    {"edelay"      , cmd_edelay      , 0},
    {"capture"     , cmd_capture     , CMD_WAIT_MUTEX},
    {"vbat"        , cmd_vbat        , 0},
#ifdef ENABLE_VBAT_OFFSET_COMMAND
    {"vbat_offset" , cmd_vbat_offset , 0},
#endif
    {"transform"   , cmd_transform   , 0},
    {"threshold"   , cmd_threshold   , 0},
    {"help"        , cmd_help        , 0},
#ifdef ENABLE_INFO_COMMAND
    {"info"        , cmd_info        , 0},
#endif
#ifdef ENABLE_THREADS_COMMAND
    {"threads"     , cmd_threads     , 0},
#endif
    {NULL          , NULL            , 0}
};

VNA_SHELL_FUNCTION(cmd_help)
{
  (void)argc;
  (void)argv;
  const VNAShellCommand *scp = commands;
  shell_printf("Commands:");
  while (scp->sc_name != NULL) {
    shell_printf(" %s", scp->sc_name);
    scp++;
  }
  shell_printf(VNA_SHELL_NEWLINE_STR);
  return;
}

/*
 * VNA shell functions
 */

//
// Read command line from shell_stream
//
static int VNAShell_readLine(char *line, int max_size){
  // Read line from input stream
  uint8_t c;
  char *ptr = line;
  while (1){
    // Return 0 only if stream not active
    if (streamRead(shell_stream, &c, 1) == 0)
      return 0;
    // Backspace or Delete
    if (c == 8 || c == 0x7f) {
      if (ptr != line) {
        static const char backspace[] = {0x08,0x20,0x08,0x00};
        shell_printf(backspace);
        ptr--;
      }
      continue;
    }
    // New line (Enter)
    if (c == '\r') {
      shell_printf(VNA_SHELL_NEWLINE_STR);
      *ptr = 0;
      return 1;
    }
    // Others (skip)
    if (c < 0x20)
      continue;
    // Store
    if (ptr < line + max_size - 1) {
      streamPut(shell_stream, c); // Echo
      *ptr++ = (char)c;
    }
  }
  return 0;
}

// Macros for convert define value to string
#define STR1(x)  #x
#define define_to_STR(x)  STR1(x)
//
// Parse and run command line
//
static void VNAShell_executeLine(char *line){
  // Parse and execute line
  char *args[VNA_SHELL_MAX_ARGUMENTS + 1];
  int n = 0;
  char *lp = line, *ep;
  while (*lp!=0){
    // Skipping white space and tabs at string begin.
    while (*lp==' ' || *lp=='\t') lp++;
    // If an argument starts with a double quote then its delimiter is another quote, else delimiter is white space.
    ep = (*lp == '"') ? strpbrk(++lp, "\"") : strpbrk(  lp, " \t");
    // Store in args string
    args[n++]=lp;
    // Stop, end of input string
    if ((lp = ep) == NULL)
      break;
    // Argument limits check
    if (n > VNA_SHELL_MAX_ARGUMENTS) {
      shell_printf("too many arguments, max "define_to_STR(VNA_SHELL_MAX_ARGUMENTS)""VNA_SHELL_NEWLINE_STR);
      return;
    }
    // Set zero at the end of string and continue check
    *lp++ = 0;
  }
  if (n == 0)
    return;
  // Execute line
  const VNAShellCommand *scp;
  for (scp = commands; scp->sc_name!=NULL;scp++) {
    if (strcmp(scp->sc_name, args[0]) == 0) {
      if (scp->flags&CMD_WAIT_MUTEX) {
        chMtxLock(&mutex);
        scp->sc_function(n-1, &args[1]);
        chMtxUnlock(&mutex);
      }
      else
        scp->sc_function(n-1, &args[1]);
      return;
    }
  }
  shell_printf("%s?"VNA_SHELL_NEWLINE_STR, args[0]);
}

#ifdef VNA_SHELL_THREAD
static THD_WORKING_AREA(waThread2, /* cmd_* max stack size + alpha */442);
THD_FUNCTION(myshellThread, p) {
  (void)p;
  chRegSetThreadName("shell");
  shell_printf(VNA_SHELL_NEWLINE_STR"NanoVNA Shell"VNA_SHELL_NEWLINE_STR);
  while (true) {
    shell_printf(VNA_SHELL_PROMPT_STR);
    if (VNAShell_readLine(shell_line, VNA_SHELL_MAX_LENGTH))
      VNAShell_executeLine(shell_line);
    else // Putting a delay in order to avoid an endless loop trying to read an unavailable stream.
      osalThreadSleepMilliseconds(100);
  }
}
#endif

// I2C clock bus setting: depend from STM32_I2C1SW in mcuconf.h
static const I2CConfig i2ccfg = {
  .timingr =     // TIMINGR register initialization. (use I2C timing configuration tool for STM32F3xx and STM32F0xx microcontrollers (AN4235))
#if STM32_I2C1SW == STM32_I2C1SW_HSI
  // STM32_I2C1SW == STM32_I2C1SW_HSI     (HSI=8MHz)
  // 400kHz @ HSI 8MHz (Use 26.4.10 I2C_TIMINGR register configuration examples from STM32 RM0091 Reference manual)
  STM32_TIMINGR_PRESC(0U)  |
  STM32_TIMINGR_SCLDEL(3U) | STM32_TIMINGR_SDADEL(1U) |
  STM32_TIMINGR_SCLH(3U)   | STM32_TIMINGR_SCLL(9U),
  // Old values voodoo magic 400kHz @ HSI 8MHz
  //0x00300506,
#elif  STM32_I2C1SW == STM32_I2C1SW_SYSCLK
  // STM32_I2C1SW == STM32_I2C1SW_SYSCLK  (SYSCLK = 48MHz)
  // 400kHz @ SYSCLK 48MHz (Use 26.4.10 I2C_TIMINGR register configuration examples from STM32 RM0091 Reference manual)
  STM32_TIMINGR_PRESC(5U)  |
  STM32_TIMINGR_SCLDEL(3U) | STM32_TIMINGR_SDADEL(3U) |
  STM32_TIMINGR_SCLH(3U)   | STM32_TIMINGR_SCLL(9U),
#else
#error "Need Define STM32_I2C1SW and set correct TIMINGR settings"
#endif
  .cr1 = 0,     // CR1 register initialization.
  .cr2 = 0      // CR2 register initialization.
};

static DACConfig dac1cfg1 = {
  //init:         2047U,
  init:         1922U,
  datamode:     DAC_DHRM_12BIT_RIGHT
};


// Main thread stack size defined in makefile USE_PROCESS_STACKSIZE = 0x200
// Profile stack usage (enable threads command by def ENABLE_THREADS_COMMAND) show:
// Stack maximum usage = 472 bytes (need test more and run all commands), free stack = 40 bytes
//
int main(void)
{
  halInit();
  chSysInit();

  chMtxObjectInit(&mutex);

  //palSetPadMode(GPIOB, 8, PAL_MODE_ALTERNATE(1) | PAL_STM32_OTYPE_OPENDRAIN);
  //palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(1) | PAL_STM32_OTYPE_OPENDRAIN);
  i2cStart(&I2CD1, &i2ccfg);
  si5351_init();

  // MCO on PA8
  //palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(0));
/*
 * Initializes a serial-over-USB CDC driver.
 */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);
/*
 * Activates the USB driver and then the USB bus pull-up on D+.
 * Note, a delay is inserted in order to not have to disconnect the cable
 * after a reset.
 */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(100);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

/*
 * SPI LCD Initialize
 */
  ili9341_init();

/* restore config */
  config_recall();
/* restore frequencies and calibration 0 slot properties from flash memory */
  caldata_recall(0);

  dac1cfg1.init = config.dac_value;
/*
 * Starting DAC1 driver, setting up the output pin as analog as suggested
 * by the Reference Manual.
 */
  dacStart(&DACD2, &dac1cfg1);

/* initial frequencies */
  update_frequencies();

/*
 * I2S Initialize
 */
  tlv320aic3204_init();
  i2sInit();
  i2sObjectInit(&I2SD2);
  i2sStart(&I2SD2, &i2sconfig);
  i2sStartExchange(&I2SD2);

  ui_init();
  //Initialize graph plotting
  plot_init();
  redraw_frame();
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO-1, Thread1, NULL);

  while (1) {
    if (SDU1.config->usbp->state == USB_ACTIVE) {
#ifdef VNA_SHELL_THREAD
#if CH_CFG_USE_WAITEXIT == FALSE
#error "VNA_SHELL_THREAD use chThdWait, need enable CH_CFG_USE_WAITEXIT in chconf.h"
#endif
      thread_t *shelltp = chThdCreateStatic(waThread2, sizeof(waThread2),
                                            NORMALPRIO + 1,
                                            myshellThread, NULL);
      chThdWait(shelltp);
#else
      shell_printf(VNA_SHELL_NEWLINE_STR"NanoVNA Shell"VNA_SHELL_NEWLINE_STR);
      do {
        shell_printf(VNA_SHELL_PROMPT_STR);
        if (VNAShell_readLine(shell_line, VNA_SHELL_MAX_LENGTH))
          VNAShell_executeLine(shell_line);
        else
          chThdSleepMilliseconds(200);
      } while (SDU1.config->usbp->state == USB_ACTIVE);
#endif
    }
    chThdSleepMilliseconds(1000);
  }
}

/* The prototype shows it is a naked function - in effect this is just an
assembly function. */
void HardFault_Handler( void );

void hard_fault_handler_c(uint32_t *sp) __attribute__( ( naked ) );;

void HardFault_Handler(void)
{
  uint32_t* sp;
  //__asm volatile ("mrs %0, msp \n\t": "=r" (sp) );
  __asm volatile ("mrs %0, psp \n\t": "=r" (sp) );
  hard_fault_handler_c(sp);
}

void hard_fault_handler_c(uint32_t* sp)
{
  (void)sp;
  while (true) {}
}
