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
#include <shell.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#define ENABLED_DUMP
//#define __SCANRAW_CMD__


static void apply_error_term_at(int i);
static void apply_edelay_at(int i);
static void cal_interpolate(int s);
void update_frequencies(void);
void set_frequencies(uint32_t start, uint32_t stop, int16_t points);

bool sweep(bool break_on_operation);

static MUTEX_DECL(mutex);

#define DRIVE_STRENGTH_AUTO (-1)
#define FREQ_HARMONICS (config.harmonic_freq_threshold)
#define IS_HARMONIC_MODE(f) ((f) > FREQ_HARMONICS)

int32_t frequency_offset = 5000;
uint32_t frequency = 10000000;
int8_t drive_strength = DRIVE_STRENGTH_AUTO;
int8_t sweep_enabled = TRUE;
int8_t sweep_once = FALSE;
int8_t cal_auto_interpolate = TRUE;
uint16_t redraw_request = 0; // contains REDRAW_XXX flags
int16_t vbat = 0;


static THD_WORKING_AREA(waThread1, 640);
static THD_FUNCTION(Thread1, arg)
{
    (void)arg;
    chRegSetThreadName("sweep");

    while (1) {
      
      bool completed = false;
      if (sweep_enabled || sweep_once) {
        chMtxLock(&mutex);

        // disable led and wait for voltage stabilization
        palClearPad(GPIOC, GPIOC_LED);
        chThdSleepMilliseconds(10);

        completed = sweep(true);
        sweep_once = FALSE;
        chMtxUnlock(&mutex);
      } else {
        __WFI();
      }

      chMtxLock(&mutex);

      // enable led
      palSetPad(GPIOC, GPIOC_LED);

      ui_process();

      if (sweep_enabled) {
        if (vbat != -1) {
          adc_stop(ADC1);
          vbat = adc_vbat_read(ADC1);
          touch_start_watchdog();
          draw_battery_status();
        }

        /* calculate trace coordinates and plot only if scan completed */
        if (completed) {
          plot_into_index(measured);
          redraw_request |= REDRAW_CELLS;
        }
      }

      /* plot trace and other indications as raster */
      draw_all(completed); // flush markmap only if scan completed to prevent remaining traces
      chMtxUnlock(&mutex);
    }
}

void
pause_sweep(void)
{
  sweep_enabled = FALSE;
}

void
resume_sweep(void)
{
  sweep_enabled = TRUE;
}

void
toggle_sweep(void)
{
  sweep_enabled = !sweep_enabled;
}

float bessel0(float x) {
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

float kaiser_window(float k, float n, float beta) {
	if (beta == 0.0) return 1.0;
	float r = (2 * k) / (n - 1) - 1;
	return bessel0(beta * sqrt(1 - r * r)) / bessel0(beta);
}

static
void
transform_domain(void)
{
  if ((domain_mode & DOMAIN_MODE) != DOMAIN_TIME) return; // nothing to do for freq domain
  // use spi_buffer as temporary buffer
  // and calculate ifft for time domain
  float* tmp = (float*)spi_buffer;

  uint8_t window_size = 101, offset = 0;
  uint8_t is_lowpass = FALSE;
  switch (domain_mode & TD_FUNC) {
      case TD_FUNC_BANDPASS:
          offset = 0;
          window_size = 101;
          break;
      case TD_FUNC_LOWPASS_IMPULSE:
      case TD_FUNC_LOWPASS_STEP:
          is_lowpass = TRUE;
          offset = 101;
          window_size = 202;
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
      for (int i = 0; i < 101; i++) {
          float w = kaiser_window(i+offset, window_size, beta);
          tmp[i*2+0] *= w;
          tmp[i*2+1] *= w;
      }
      for (int i = 101; i < FFT_SIZE; i++) {
          tmp[i*2+0] = 0.0;
          tmp[i*2+1] = 0.0;
      }
      if (is_lowpass) {
          for (int i = 1; i < 101; i++) {
              tmp[(FFT_SIZE-i)*2+0] =  tmp[i*2+0];
              tmp[(FFT_SIZE-i)*2+1] = -tmp[i*2+1];
          }
      }

      fft256_inverse((float(*)[2])tmp);
      memcpy(measured[ch], tmp, sizeof(measured[0]));
      for (int i = 0; i < 101; i++) {
          measured[ch][i][0] /= (float)FFT_SIZE;
          if (is_lowpass) {
              measured[ch][i][1] = 0.0;
          } else {
              measured[ch][i][1] /= (float)FFT_SIZE;
          }
      }
      if ( (domain_mode & TD_FUNC) == TD_FUNC_LOWPASS_STEP ) {
          for (int i = 1; i < 101; i++) {
              measured[ch][i][0] += measured[ch][i-1][0];
          }
      }
  }
}

static void cmd_pause(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)chp;
    (void)argc;
    (void)argv;
    pause_sweep();
}

static void cmd_resume(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)chp;
    (void)argc;
    (void)argv;

    // restore frequencies array and cal
    update_frequencies();
    if (cal_auto_interpolate && (cal_status & CALSTAT_APPLY))
      cal_interpolate(lastsaveid);

    resume_sweep();
}

static void cmd_reset(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    if (argc == 1) {
        if (strcmp(argv[0], "dfu") == 0) {
            chprintf(chp, "Performing reset to DFU mode\r\n");
            enter_dfu();
            return;
        }
    }

    chprintf(chp, "Performing reset\r\n");

    rccEnableWWDG(FALSE);

    WWDG->CFR = 0x60;
    WWDG->CR = 0xff;

    /* wait forever */
    while (1)
      ;
}

// {gainLeft, gainRight}
const int8_t gain_table[][2] = {
    {  0,  0 },     // 1st: 0 ~ 300MHz
    { 43, 40 },     // 2nd: 300 ~ 600MHz
    { 53, 50 },     // 3rd: 600 ~ 900MHz
    { 75, 72 },     // 4th: 900 ~ 1200MHz
    { 83, 80 },     // 5th: 1200 ~ 1400MHz
    { 93, 90 },     // 6th: 1400MHz ~
};

static int adjust_gain(int newfreq)
{
  int delay = 0;
  int new_order = newfreq / FREQ_HARMONICS;
  int old_order = frequency / FREQ_HARMONICS;
  if (new_order != old_order) {
    tlv320aic3204_set_gain(gain_table[new_order][0], gain_table[new_order][1]);
    delay += 10;
  }
  return delay;
}

int set_frequency(uint32_t freq)
{
    int delay = 0;
    if (frequency == freq)
      return delay;

    delay += adjust_gain(freq);

    int8_t ds = drive_strength;
    if (ds == DRIVE_STRENGTH_AUTO) {
      ds = freq > FREQ_HARMONICS ? SI5351_CLK_DRIVE_STRENGTH_8MA : SI5351_CLK_DRIVE_STRENGTH_2MA;
    }
    delay += si5351_set_frequency_with_offset(freq, frequency_offset, ds);

    frequency = freq;
    return delay;
}

static void cmd_offset(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc != 1) {
        chprintf(chp, "usage: offset {frequency offset(Hz)}\r\n");
        return;
    }
    frequency_offset = atoi(argv[0]);
    set_frequency(frequency);
}

static void cmd_freq(BaseSequentialStream *chp, int argc, char *argv[])
{
    int freq;
    if (argc != 1) {
        chprintf(chp, "usage: freq {frequency(Hz)}\r\n");
        return;
    }
    pause_sweep();
    chMtxLock(&mutex);
    freq = atoi(argv[0]);
    set_frequency(freq);
    chMtxUnlock(&mutex);
}

static void cmd_power(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc != 1) {
        chprintf(chp, "usage: power {0-3|-1}\r\n");
        return;
    }
    drive_strength = atoi(argv[0]);
    set_frequency(frequency);
}

static void cmd_time(BaseSequentialStream *chp, int argc, char *argv[])
{
    RTCDateTime timespec;
    (void)argc;
    (void)argv;
    rtcGetTime(&RTCD1, &timespec);
    chprintf(chp, "%d/%d/%d %d\r\n", timespec.year+1980, timespec.month, timespec.day, timespec.millisecond);
}


static void cmd_dac(BaseSequentialStream *chp, int argc, char *argv[])
{
    int value;
    if (argc != 1) {
        chprintf(chp, "usage: dac {value(0-4095)}\r\n");
        chprintf(chp, "current value: %d\r\n", config.dac_value);
        return;
    }
    value = atoi(argv[0]);
    config.dac_value = value;
    dacPutChannelX(&DACD2, 0, value);
}

static void cmd_threshold(BaseSequentialStream *chp, int argc, char *argv[])
{
    int value;
    if (argc != 1) {
        chprintf(chp, "usage: threshold {frequency in harmonic mode}\r\n");
        chprintf(chp, "current: %d\r\n", config.harmonic_freq_threshold);
        return;
    }
    value = atoi(argv[0]);
    config.harmonic_freq_threshold = value;
}

static void cmd_saveconfig(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argc;
  (void)argv;
  config_save();
  chprintf(chp, "Config saved.\r\n");
}

static void cmd_clearconfig(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc != 1) {
    chprintf(chp, "usage: clearconfig {protection key}\r\n");
    return;
  }

  if (strcmp(argv[0], "1234") != 0) {
    chprintf(chp, "Key unmatched.\r\n");
    return;
  }

  clear_all_config_prop_data();
  chprintf(chp, "Config and all cal data cleared.\r\n");
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

float measured[2][101][2];

static void
wait_dsp(int count)
{
  wait_count = count;
  //reset_dsp_accumerator();
  while (wait_count)
    __WFI();
}

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
  .tx_buffer    = NULL,                 // TX Buffer
  .rx_buffer    = rx_buffer,            // RX Buffer
  .size         = AUDIO_BUFFER_LEN * 2,
  .tx_end_cb    = NULL,                 // tx callback
  .rx_end_cb    = i2s_end_callback,     // rx callback
  .i2scfgr      = 0,                    // i2scfgr
  .i2spr        = 2                     // i2spr
};

static void cmd_data(BaseSequentialStream *chp, int argc, char *argv[])
{
  int i;
  int sel = 0;

  if (argc == 1)
    sel = atoi(argv[0]);
  if (sel == 0 || sel == 1) {
    chMtxLock(&mutex);
    for (i = 0; i < sweep_points; i++) {
      if (frequencies[i] != 0)
        chprintf(chp, "%f %f\r\n", measured[sel][i][0], measured[sel][i][1]);
    }
    chMtxUnlock(&mutex);
  } else if (sel >= 2 && sel < 7) {
    chMtxLock(&mutex);
    for (i = 0; i < sweep_points; i++) {
      if (frequencies[i] != 0)
        chprintf(chp, "%f %f\r\n", cal_data[sel-2][i][0], cal_data[sel-2][i][1]);
    }
    chMtxUnlock(&mutex);
  } else {
    chprintf(chp, "usage: data [array]\r\n");
  }
}

#ifdef ENABLED_DUMP
static void cmd_dump(BaseSequentialStream *chp, int argc, char *argv[])
{
  int i, j;
  int len;

  if (argc == 1)
    dump_selection = atoi(argv[0]);

  wait_dsp(3);

  len = AUDIO_BUFFER_LEN;
  if (dump_selection == 1 || dump_selection == 2)
    len /= 2;
  for (i = 0; i < len; ) {
    for (j = 0; j < 16; j++, i++) {
      chprintf(chp, "%04x ", 0xffff & (int)dump_buffer[i]);
    }
    chprintf(chp, "\r\n");
  }
}
#endif

static void cmd_capture(BaseSequentialStream *chp, int argc, char *argv[])
{
// read pixel count at one time (PART*2 bytes required for read buffer)
#define PART 320
    (void)argc;
    (void)argv;

    chMtxLock(&mutex);

    // use uint16_t spi_buffer[1024] (defined in ili9341) for read buffer
    uint16_t *buf = &spi_buffer[0];
    int len = 320 * 240;
    int i;
    ili9341_read_memory(0, 0, 320, 240, PART, buf);
    for (i = 0; i < PART; i++) {
        streamPut(chp, buf[i] >> 8);
        streamPut(chp, buf[i] & 0xff);
    }

    len -= PART;
    while (len > 0) {
        ili9341_read_memory_continue(PART, buf);
        for (i = 0; i < PART; i++) {
            streamPut(chp, buf[i] >> 8);
            streamPut(chp, buf[i] & 0xff);
        }
        len -= PART;
    }
    //*/

    chMtxUnlock(&mutex);
}

#if 0
static void cmd_gamma(BaseSequentialStream *chp, int argc, char *argv[])
{
  float gamma[2];
  (void)argc;
  (void)argv;
  
  pause_sweep();
  chMtxLock(&mutex);
  wait_dsp(4);  
  calculate_gamma(gamma);
  chMtxUnlock(&mutex);

  chprintf(chp, "%d %d\r\n", gamma[0], gamma[1]);
}
#endif

static void (*sample_func)(float *gamma) = calculate_gamma;

static void cmd_sample(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc == 1) {
    if (strcmp(argv[0], "ref") == 0) {
      sample_func = fetch_amplitude_ref;
      return;
    } else if (strcmp(argv[0], "ampl") == 0) {
      sample_func = fetch_amplitude;
      return;
    } else if (strcmp(argv[0], "gamma") == 0) {
      sample_func = calculate_gamma;
      return;
    }
  }
  chprintf(chp, "usage: sample {gamma|ampl|ref}\r\n");
}


#if 0
int32_t frequency0 = 1000000;
int32_t frequency1 = 300000000;
int16_t sweep_points = 101;

uint32_t frequencies[101];
uint16_t cal_status;
float cal_data[5][101][2];
#endif

config_t config = {
  .magic =             CONFIG_MAGIC,
  .dac_value =         1922,
  .grid_color =        0x1084,
  .menu_normal_color = 0xffff,
  .menu_active_color = 0x7777,
  .trace_color =       { RGB565(0,255,255), RGB565(255,0,40), RGB565(0,0,255), RGB565(50,255,0) },
  .touch_cal =         { 693, 605, 124, 171 },  //{ 620, 600, 160, 190 },
  .default_loadcal =   0,
  .harmonic_freq_threshold = 300000000,
  .checksum =          0
};

properties_t current_props = {
  .magic =      CONFIG_MAGIC,
  ._frequency0 =       50000, // start = 50kHz
  ._frequency1 =   900000000, // end = 900MHz
  ._sweep_points =       101,
  ._cal_status =           0,
  //._frequencies =         {},
  //._cal_data =            {},
  ._electrical_delay =     0,
  ._trace = /*[4] */
  {/*enable, type, channel, polar, scale, refpos*/
    { 1, TRC_LOGMAG, 0, 0, 1.0, 7.0 },
    { 1, TRC_LOGMAG, 1, 0, 1.0, 7.0 },
    { 1, TRC_SMITH,  0, 1, 1.0, 0.0 },
    { 1, TRC_PHASE,  1, 0, 1.0, 4.0 }
  },
  ._markers = /*[4] */ {
    { 1, 30, 0 }, { 0, 40, 0 }, { 0, 60, 0 }, { 0, 80, 0 }
  },
  ._active_marker =        0,
  ._domain_mode =          0,
  ._velocity_factor =     70,
  .checksum =              0
};
properties_t *active_props = &current_props;

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

// main loop for measurement
bool sweep(bool break_on_operation)
{
  int i;

  for (i = 0; i < sweep_points; i++) {
    int delay = set_frequency(frequencies[i]);
    delay = delay < 3 ? 3 : delay;
    delay = delay > 8 ? 8 : delay;
    
    tlv320aic3204_select_in3(); // CH0:REFLECT
    wait_dsp(delay);

    /* calculate reflection coeficient */
    (*sample_func)(measured[0][i]);

    tlv320aic3204_select_in1(); // CH1:TRANSMISSION
    wait_dsp(delay);

    /* calculate transmission coeficient */
    (*sample_func)(measured[1][i]);

    if (cal_status & CALSTAT_APPLY)
      apply_error_term_at(i);

    if (electrical_delay != 0)
      apply_edelay_at(i);

    // back to toplevel to handle ui operation
    if (operation_requested && break_on_operation)
      return false;
  }

  transform_domain();
  return true;
}

#ifdef __SCANRAW_CMD__
#include <stdio.h>

static void measure_gamma_avg(int channel, uint32_t freq, uint32_t avg_count, float* gamma) {
    int delay = set_frequency(freq);
    delay = delay < 3 ? 3 : delay;
    delay = delay > 8 ? 8 : delay;
    
    if (channel == 0)
        tlv320aic3204_select_in3(); // CH0:REFLECT
    else
        tlv320aic3204_select_in1(); // CH1:TRANSMISSION
    wait_dsp(delay);
    
    gamma[0] = 0.0;
    gamma[1] = 0.0;
    float gamma_acc[2] = { 0, 0 };
    for (int j = 0; j < avg_count; j++) {
            
        wait_dsp(1);
        /* calculate reflection/transmission coeficient */
        (*sample_func)(gamma);

        if (avg_count == 1) break;
        gamma_acc[0] += gamma[0];
        gamma_acc[1] += gamma[1];
    }
    if (avg_count > 1) {
        gamma[0] = gamma_acc[0] / avg_count;
        gamma[1] = gamma_acc[1] / avg_count;
    }
}

static void cmd_scanraw(BaseSequentialStream *chp, int argc, char *argv[])
{
    int32_t chan, freq, step, count, avg_count;
    if (argc != 4 && argc != 5) {
        chprintf(chp, "usage: scanraw {channel(0|1)} {start(Hz)} {stEp(Hz)} {count} [average]\r\n");
        return;
    }
    chan = atoi(argv[0]);
    freq = atoi(argv[1]);
    step = atoi(argv[2]);
    count = atoi(argv[3]);
    avg_count = 1;
    if (argc == 5)
        avg_count = atoi(argv[4]);
    if (chan < 0 || chan > 1) {
        chprintf(chp, "invalid channel\r\n");
        return;
    }
    if (freq < 0 || step == 0 || (freq+step*count) < 0) {
        chprintf(chp, "frequency range is invalid\r\n");
        return;
    }
    if (avg_count < 1 || avg_count > 1000) {
        chprintf(chp, "average out of range [1..1000]\r\n");
        return;
    }

    chMtxLock(&mutex);
    
    // disable led and wait for voltage stabilization
    palClearPad(GPIOC, GPIOC_LED);
    chThdSleepMilliseconds(10);

    for (int i = 0; i < count; i++, freq += step) {

        float gamma[2];
        measure_gamma_avg(chan, freq, avg_count, gamma);

        // print floating point losslessly: float="%.9g", double="%.17g"
        //
        // [chprintf doesn't support proper float formatting]
        //chprintf(chp, "%d\t%.9g\t%.9g\r\n", 
        //    freq, gamma[0], gamma[1]);
        
        char tmpbuf[20];
        int leng;
        leng = snprintf(tmpbuf, sizeof(tmpbuf), "%.9g", gamma[0]);
        for (int j=0; j < leng; j++) {
            streamPut(chp, (uint8_t)tmpbuf[j]); 
        }
        streamPut(chp, (uint8_t)'\t'); 
        leng = snprintf(tmpbuf, sizeof(tmpbuf), "%.9g", gamma[1]);
        for (int j=0; j < leng; j++) {
            streamPut(chp, (uint8_t)tmpbuf[j]); 
        }
        streamPut(chp, (uint8_t)'\r'); 
        streamPut(chp, (uint8_t)'\n'); 
    }
    chMtxUnlock(&mutex);
}
#endif //__SCANRAW_CMD__

static void cmd_scan(BaseSequentialStream *chp, int argc, char *argv[])
{
  int32_t start, stop;
  int16_t points = sweep_points;

  if (argc != 2 && argc != 3) {
    chprintf(chp, "usage: sweep {start(Hz)} {stop(Hz)} [points]\r\n");
    return;
  }

  start = atoi(argv[0]);
  stop = atoi(argv[1]);
  if (start == 0 || stop == 0 || start > stop) {
      chprintf(chp, "frequency range is invalid\r\n");
      return;
  }
  if (argc == 3) {
    points = atoi(argv[2]);
    if (points <= 0 || points > sweep_points) {
      chprintf(chp, "sweep points exceeds range\r\n");
      return;
    }
  }

  pause_sweep();
  chMtxLock(&mutex);
  set_frequencies(start, stop, points);
  if (cal_auto_interpolate && (cal_status & CALSTAT_APPLY))
    cal_interpolate(lastsaveid);

  sweep_once = TRUE;
  chMtxUnlock(&mutex);

  // wait finishing sweep
  while (sweep_once)
    chThdSleepMilliseconds(10);
}

static void
update_marker_index(void)
{
  int m;
  int i;
  for (m = 0; m < 4; m++) {
    if (!markers[m].enabled)
      continue;
    uint32_t f = markers[m].frequency;
    if (f < frequencies[0]) {
      markers[m].index = 0;
      markers[m].frequency = frequencies[0];
    } else if (f >= frequencies[sweep_points-1]) {
      markers[m].index = sweep_points-1;
      markers[m].frequency = frequencies[sweep_points-1];
    } else {
      for (i = 0; i < sweep_points-1; i++) {
        if (frequencies[i] <= f && f < frequencies[i+1]) {
          uint32_t mid = (frequencies[i] + frequencies[i+1])/2;
          if (f < mid) {
            markers[m].index = i;
          } else {
            markers[m].index = i + 1;
          }
          break;
        }
      }      
    }
  }
}

void
set_frequencies(uint32_t start, uint32_t stop, int16_t points)
{
  int i;
  float span = stop - start;
  for (i = 0; i < points; i++) {
    float offset = i * span / (float)(points - 1);
    frequencies[i] = start + (uint32_t)offset;
  }
  // disable at out of sweep range
  for (; i < sweep_points; i++)
    frequencies[i] = 0;
}

void
update_frequencies(void)
{
  uint32_t start, stop;
  if (frequency1 > 0) {
    start = frequency0;
    stop = frequency1;
  } else {
    int32_t center = frequency0;
    int32_t span = -frequency1;
    start = center - span/2;
    stop = center + span/2;
  }

  set_frequencies(start, stop, sweep_points);
  operation_requested = OP_FREQCHANGE;
  
  update_marker_index();
  
  // set grid layout
  update_grid();
}

void
freq_mode_startstop(void)
{
  if (frequency1 <= 0) {
    int center = frequency0;
    int span = -frequency1;
    ensure_edit_config();
    frequency0 = center - span/2;
    frequency1 = center + span/2;
  }
}

void
freq_mode_centerspan(void)
{
  if (frequency1 > 0) {
    int start = frequency0;
    int stop = frequency1;
    ensure_edit_config();
    frequency0 = (start + stop)/2; // center
    frequency1 = -(stop - start); // span
  }
}


#define START_MIN 10000
//#define STOP_MAX 900000000
#define STOP_MAX 1500000000

void
set_sweep_frequency(int type, int32_t freq)
{
  int cal_applied = cal_status & CALSTAT_APPLY;
  switch (type) {
  case ST_START:
    freq_mode_startstop();
    if (freq < START_MIN)
      freq = START_MIN;
    if (freq > STOP_MAX)
      freq = STOP_MAX;
    if (frequency0 != freq) {
      ensure_edit_config();
      frequency0 = freq;
      // if start > stop then make start = stop
      if (frequency1 < freq)
        frequency1 = freq;
      update_frequencies();
    }
    break;
  case ST_STOP:
    freq_mode_startstop();
    if (freq > STOP_MAX)
      freq = STOP_MAX;
    if (freq < START_MIN)
      freq = START_MIN;
    if (frequency1 != freq) {
      ensure_edit_config();
      frequency1 = freq;
      // if start > stop then make start = stop
      if (frequency0 > freq)
        frequency0 = freq;
      update_frequencies();
    }
    break;
  case ST_CENTER:
    ensure_edit_config();
    freq_mode_centerspan();
    if (frequency0 != freq) {
      ensure_edit_config();
      frequency0 = freq;
      int center = frequency0;
      int span = -frequency1;
      if (center-span/2 < START_MIN) {
        span = (center - START_MIN) * 2;
        frequency1 = -span;
      }
      if (center+span/2 > STOP_MAX) {
        span = (STOP_MAX - center) * 2;
        frequency1 = -span;
      }
      update_frequencies();
    }
    break;
  case ST_SPAN:
    freq_mode_centerspan();
    if (frequency1 != -freq) {
      ensure_edit_config();
      frequency1 = -freq;
      int center = frequency0;
      int span = -frequency1;
      if (center-span/2 < START_MIN) {
        center = START_MIN + span/2;
        frequency0 = center;
      }
      if (center+span/2 > STOP_MAX) {
        center = STOP_MAX - span/2;
        frequency0 = center;
      }
      update_frequencies();
    }
    break;
  case ST_CW:
    freq_mode_centerspan();
    if (frequency0 != freq || frequency1 != 0) {
      ensure_edit_config();
      frequency0 = freq;
      frequency1 = 0;
      update_frequencies();
    }
    break;
  }

  if (cal_auto_interpolate && cal_applied)
    cal_interpolate(lastsaveid);
}

uint32_t
get_sweep_frequency(int type)
{
  if (frequency1 >= 0) {
    switch (type) {
    case ST_START: return frequency0;
    case ST_STOP: return frequency1;
    case ST_CENTER: return (frequency0 + frequency1)/2;
    case ST_SPAN: return frequency1 - frequency0;
    case ST_CW: return (frequency0 + frequency1)/2;
    }
  } else {
    switch (type) {
    case ST_START: return frequency0 + frequency1/2;
    case ST_STOP: return frequency0 - frequency1/2;
    case ST_CENTER: return frequency0;
    case ST_SPAN: return -frequency1;
    case ST_CW: return frequency0;
    }
  }
  return 0;
}


static void cmd_sweep(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc == 0) {
    chprintf(chp, "%d %d %d\r\n", frequency0, frequency1, sweep_points);
    return;
  } else if (argc > 3) {
    goto usage;
  }
  if (argc >= 2) {
    if (strcmp(argv[0], "start") == 0) {
      int32_t value = atoi(argv[1]);
      set_sweep_frequency(ST_START, value);
      return;
    } else if (strcmp(argv[0], "stop") == 0) {
      int32_t value = atoi(argv[1]);
      set_sweep_frequency(ST_STOP, value);
      return;
    } else if (strcmp(argv[0], "center") == 0) {
      int32_t value = atoi(argv[1]);
      set_sweep_frequency(ST_CENTER, value);
      return;
    } else if (strcmp(argv[0], "span") == 0) {
      int32_t value = atoi(argv[1]);
      set_sweep_frequency(ST_SPAN, value);
      return;
    } else if (strcmp(argv[0], "cw") == 0) {
      int32_t value = atoi(argv[1]);
      set_sweep_frequency(ST_CW, value);
      return;
    }
  }

  if (argc >= 1) {
    int32_t value = atoi(argv[0]);
    if (value == 0)
      goto usage;
    set_sweep_frequency(ST_START, value);
  }
  if (argc >= 2) {
    int32_t value = atoi(argv[1]);
    set_sweep_frequency(ST_STOP, value);
  }
  return;
usage:
  chprintf(chp, "usage: sweep {start(Hz)} [stop(Hz)]\r\n");
  chprintf(chp, "\tsweep {start|stop|center|span|cw} {freq(Hz)}\r\n");
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


const struct open_model {
  float c0;
  float c1;
  float c2;
  float c3;
} open_model = { 50, 0, -300, 27 };

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
    //float z = 6.2832 * frequencies[i] * c * z0;
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
    float z = 6.2832 * frequencies[i] * c * z0;
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

void apply_error_term_at(int i)
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
  chMtxLock(&mutex);

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
  chMtxUnlock(&mutex);
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

void
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
}

static void cmd_cal(BaseSequentialStream *chp, int argc, char *argv[])
{
  const char *items[] = { "load", "open", "short", "thru", "isoln", "Es", "Er", "Et", "cal'ed" };

  if (argc == 0) {
    int i;
    for (i = 0; i < 9; i++) {
      if (cal_status & (1<<i))
        chprintf(chp, "%s ", items[i]);
    }
    chprintf(chp, "\r\n");
    return;
  }

  char *cmd = argv[0];
  if (strcmp(cmd, "load") == 0) {
    cal_collect(CAL_LOAD);
  } else if (strcmp(cmd, "open") == 0) {
    cal_collect(CAL_OPEN);
  } else if (strcmp(cmd, "short") == 0) {
    cal_collect(CAL_SHORT);
  } else if (strcmp(cmd, "thru") == 0) {
    cal_collect(CAL_THRU);
  } else if (strcmp(cmd, "isoln") == 0) {
    cal_collect(CAL_ISOLN);
  } else if (strcmp(cmd, "done") == 0) {
    cal_done();
    return;
  } else if (strcmp(cmd, "on") == 0) {
    cal_status |= CALSTAT_APPLY;
    redraw_request |= REDRAW_CAL_STATUS;
    return;
  } else if (strcmp(cmd, "off") == 0) {
    cal_status &= ~CALSTAT_APPLY;
    redraw_request |= REDRAW_CAL_STATUS;
    return;
  } else if (strcmp(cmd, "reset") == 0) {
    cal_status = 0;
    redraw_request |= REDRAW_CAL_STATUS;
    return;
  } else if (strcmp(cmd, "data") == 0) {
    chprintf(chp, "%f %f\r\n", cal_data[CAL_LOAD][0][0], cal_data[CAL_LOAD][0][1]);
    chprintf(chp, "%f %f\r\n", cal_data[CAL_OPEN][0][0], cal_data[CAL_OPEN][0][1]);
    chprintf(chp, "%f %f\r\n", cal_data[CAL_SHORT][0][0], cal_data[CAL_SHORT][0][1]);
    chprintf(chp, "%f %f\r\n", cal_data[CAL_THRU][0][0], cal_data[CAL_THRU][0][1]);
    chprintf(chp, "%f %f\r\n", cal_data[CAL_ISOLN][0][0], cal_data[CAL_ISOLN][0][1]);
    return;
  } else if (strcmp(cmd, "in") == 0) {
    int s = 0;
    if (argc > 1)
      s = atoi(argv[1]);
    cal_interpolate(s);
    redraw_request |= REDRAW_CAL_STATUS;
    return;
  } else {
    chprintf(chp, "usage: cal [load|open|short|thru|isoln|done|reset|on|off|in]\r\n");
    return;
  }
}

static void cmd_save(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)chp;

  if (argc != 1)
    goto usage;

  int id = atoi(argv[0]);
  if (id < 0 || id >= SAVEAREA_MAX)
    goto usage;
  caldata_save(id);
  redraw_request |= REDRAW_CAL_STATUS;
  return;

 usage:
  chprintf(chp, "save {id}\r\n");
}

static void cmd_recall(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)chp;
  if (argc != 1)
    goto usage;

  int id = atoi(argv[0]);
  if (id < 0 || id >= SAVEAREA_MAX)
    goto usage;

  pause_sweep();
  chMtxLock(&mutex);
  if (caldata_recall(id) == 0) {
    // success
    update_frequencies();
    redraw_request |= REDRAW_CAL_STATUS;
  }
  chMtxUnlock(&mutex);
  resume_sweep();
  return;

 usage:
  chprintf(chp, "recall {id}\r\n");
}

const struct {
  const char *name;
  uint16_t refpos;
  float scale_unit;
} trace_info[] = {
  { "LOGMAG", 7, 10 },
  { "PHASE",  4, 90 },
  { "DELAY",  4,  1e-9 },
  { "SMITH",  0,  1 },
  { "POLAR",  0,  1 },
  { "LINEAR", 0,  0.125 },
  { "SWR",    0,  1 },
  { "REAL",   4,  0.25 },
  { "IMAG",   4,  0.25 },
  { "R",      0, 100 },
  { "X",      4, 100 }
};

const char * const trc_channel_name[] = {
  "CH0", "CH1"
};

const char *
get_trace_typename(int t)
{
  return trace_info[trace[t].type].name;
}

void set_trace_type(int t, int type)
{
  int polar = type == TRC_SMITH || type == TRC_POLAR;
  int enabled = type != TRC_OFF;
  int force = FALSE;

  if (trace[t].polar != polar) {
    trace[t].polar = polar;
    force = TRUE;
  }
  if (trace[t].enabled != enabled) {
    trace[t].enabled = enabled;
    force = TRUE;
  }
  if (trace[t].type != type) {
    trace[t].type = type;
    trace[t].refpos = trace_info[type].refpos;
    if (polar)
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
  scale /= trace_info[trace[t].type].scale_unit;
  if (trace[t].scale != scale) {
    trace[t].scale = scale;
    force_set_markmap();
  }
}

float get_trace_scale(int t)
{
  return trace[t].scale * trace_info[trace[t].type].scale_unit;
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

float
my_atof(const char *p)
{
  int neg = FALSE;
  if (*p == '-')
    neg = TRUE;
  if (*p == '-' || *p == '+')
    p++;
  float x = atoi(p);
  while (isdigit((int)*p))
    p++;
  if (*p == '.') {
    float d = 1.0f;
    p++;
    while (isdigit((int)*p)) {
      d /= 10;
      x += d * (*p - '0');
      p++;
    }
  }
  if (*p == 'e' || *p == 'E') {
    p++;
    int exp = atoi(p);
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

static void cmd_trace(BaseSequentialStream *chp, int argc, char *argv[])
{
  int t;
  if (argc == 0) {
    for (t = 0; t < 4; t++) {
      if (trace[t].enabled) {
        const char *type = trace_info[trace[t].type].name;
        const char *channel = trc_channel_name[trace[t].channel];
        float scale = get_trace_scale(t);
        float refpos = get_trace_refpos(t);
        chprintf(chp, "%d %s %s %f %f\r\n", t, type, channel, scale, refpos);
      }
    }
    return;
  } 

  if (strcmp(argv[0], "all") == 0 &&
      argc > 1 && strcmp(argv[1], "off") == 0) {
    set_trace_type(0, TRC_OFF);
    set_trace_type(1, TRC_OFF);
    set_trace_type(2, TRC_OFF);
    set_trace_type(3, TRC_OFF);
    goto exit;
  }

  t = atoi(argv[0]);
  if (t < 0 || t >= 4)
    goto usage;
  if (argc == 1) {
    const char *type = get_trace_typename(t);
    const char *channel = trc_channel_name[trace[t].channel];
    chprintf(chp, "%d %s %s\r\n", t, type, channel);
    return;
  }
  if (argc > 1) {
    if (strcmp(argv[1], "logmag") == 0) {
      set_trace_type(t, TRC_LOGMAG);
    } else if (strcmp(argv[1], "phase") == 0) {
      set_trace_type(t, TRC_PHASE);
    } else if (strcmp(argv[1], "polar") == 0) {
      set_trace_type(t, TRC_POLAR);
    } else if (strcmp(argv[1], "smith") == 0) {
      set_trace_type(t, TRC_SMITH);
    } else if (strcmp(argv[1], "delay") == 0) {
      set_trace_type(t, TRC_DELAY);
    } else if (strcmp(argv[1], "linear") == 0) {
      set_trace_type(t, TRC_LINEAR);
    } else if (strcmp(argv[1], "swr") == 0) {
      set_trace_type(t, TRC_SWR);
    } else if (strcmp(argv[1], "real") == 0) {
      set_trace_type(t, TRC_REAL);
    } else if (strcmp(argv[1], "imag") == 0) {
      set_trace_type(t, TRC_IMAG);
    } else if (strcmp(argv[1], "r") == 0) {
      set_trace_type(t, TRC_R);
    } else if (strcmp(argv[1], "x") == 0) {
      set_trace_type(t, TRC_X);
    } else if (strcmp(argv[1], "linear") == 0) {
      set_trace_type(t, TRC_LINEAR);
    } else if (strcmp(argv[1], "off") == 0) {
      set_trace_type(t, TRC_OFF);
    } else if (strcmp(argv[1], "scale") == 0 && argc >= 3) {
      //trace[t].scale = my_atof(argv[2]);
      set_trace_scale(t, my_atof(argv[2]));
      goto exit;
    } else if (strcmp(argv[1], "refpos") == 0 && argc >= 3) {
      //trace[t].refpos = my_atof(argv[2]);
      set_trace_refpos(t, my_atof(argv[2]));
      goto exit;
    } else {
      goto usage;
    }
  }
  if (argc > 2) {
    int src = atoi(argv[2]);
    if (src != 0 && src != 1)
      goto usage;
    trace[t].channel = src;
  }  
 exit:
  return;
 usage:
  chprintf(chp, "trace {0|1|2|3|all} [logmag|phase|smith|linear|delay|swr|real|imag|r|x|off] [src]\r\n");
  chprintf(chp, "trace {0|1|2|3} {scale|refpos} {value}\r\n");
}


void set_electrical_delay(float picoseconds)
{
  if (electrical_delay != picoseconds) {
    electrical_delay = picoseconds;
    force_set_markmap();
  }
}

float get_electrical_delay(void)
{
  return electrical_delay;
}

static void cmd_edelay(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc == 0) {
    chprintf(chp, "%f\r\n", electrical_delay);
    return;
  }
  if (argc > 0) {
    set_electrical_delay(my_atof(argv[0]));
  }
}


static void cmd_marker(BaseSequentialStream *chp, int argc, char *argv[])
{
  int t;
  if (argc == 0) {
    for (t = 0; t < 4; t++) {
      if (markers[t].enabled) {
        chprintf(chp, "%d %d %d\r\n", t+1, markers[t].index, markers[t].frequency);
      }
    }
    return;
  } 
  if (strcmp(argv[0], "off") == 0) {
    active_marker = -1;
    for (t = 0; t < 4; t++)
      markers[t].enabled = FALSE;
    redraw_request |= REDRAW_MARKER;
    return;
  }

  t = atoi(argv[0])-1;
  if (t < 0 || t >= 4)
    goto usage;
  if (argc == 1) {
    chprintf(chp, "%d %d %d\r\n", t+1, markers[t].index, frequency);
    active_marker = t;
    // select active marker
    markers[t].enabled = TRUE;
    redraw_request |= REDRAW_MARKER;
    return;
  }
  if (argc > 1) {
    if (strcmp(argv[1], "off") == 0) {
      markers[t].enabled = FALSE;
      if (active_marker == t)
        active_marker = -1;
      redraw_request |= REDRAW_MARKER;
    } else if (strcmp(argv[1], "on") == 0) {
      markers[t].enabled = TRUE;
      active_marker = t;
      redraw_request |= REDRAW_MARKER;
    } else {
      // select active marker and move to index
      markers[t].enabled = TRUE;
      int index = atoi(argv[1]);
      markers[t].index = index;
      markers[t].frequency = frequencies[index];
      active_marker = t;
      redraw_request |= REDRAW_MARKER;
    }
  }
  return;
 usage:
  chprintf(chp, "marker [n] [off|{index}]\r\n");
}

static void cmd_touchcal(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argc;
  (void)argv;
  //extern int16_t touch_cal[4];
  int i;

  chMtxLock(&mutex);
  chprintf(chp, "first touch upper left, then lower right...");
  touch_cal_exec();
  chprintf(chp, "done\r\n");

  chprintf(chp, "touch cal params: ");
  for (i = 0; i < 4; i++) {
    chprintf(chp, "%d ", config.touch_cal[i]);
  }
  chprintf(chp, "\r\n");
  chMtxUnlock(&mutex);
}

static void cmd_touchtest(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)chp;
  (void)argc;
  (void)argv;
  chMtxLock(&mutex);
  do {
    touch_draw_test();
  } while(argc);
  chMtxUnlock(&mutex);
  
}

static void cmd_frequencies(BaseSequentialStream *chp, int argc, char *argv[])
{
  int i;
  (void)chp;
  (void)argc;
  (void)argv;
  for (i = 0; i < sweep_points; i++) {
    if (frequencies[i] != 0)
      chprintf(chp, "%d\r\n", frequencies[i]);
  }
}

static void
set_domain_mode(int mode) // accept DOMAIN_FREQ or DOMAIN_TIME
{
  if (mode != (domain_mode & DOMAIN_MODE)) {
    domain_mode = (domain_mode & ~DOMAIN_MODE) | (mode & DOMAIN_MODE);
    redraw_request |= REDRAW_FREQUENCY;
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

static void cmd_transform(BaseSequentialStream *chp, int argc, char *argv[])
{
  int i;
  if (argc == 0) {
    goto usage;
  }

  for (i = 0; i < argc; i++) {
    char *cmd = argv[i];
    if (strcmp(cmd, "on") == 0) {
      set_domain_mode(DOMAIN_TIME);
    } else if (strcmp(cmd, "off") == 0) {
      set_domain_mode(DOMAIN_FREQ);
    } else if (strcmp(cmd, "impulse") == 0) {
      set_timedomain_func(TD_FUNC_LOWPASS_IMPULSE);
    } else if (strcmp(cmd, "step") == 0) {
      set_timedomain_func(TD_FUNC_LOWPASS_STEP);
    } else if (strcmp(cmd, "bandpass") == 0) {
      set_timedomain_func(TD_FUNC_BANDPASS);
    } else if (strcmp(cmd, "minimum") == 0) {
      set_timedomain_window(TD_WINDOW_MINIMUM);
    } else if (strcmp(cmd, "normal") == 0) {
      set_timedomain_window(TD_WINDOW_NORMAL);
    } else if (strcmp(cmd, "maximum") == 0) {
      set_timedomain_window(TD_WINDOW_MAXIMUM);
    } else {
      goto usage;
    }
  }
  return;

usage:
  chprintf(chp, "usage: transform {on|off|impulse|step|bandpass|minimum|normal|maximum} [...]\r\n");
}

static void cmd_test(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)chp;
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
    mode = atoi(argv[0]);

  for (i = 0; i < 20; i++) {
    palClearPad(GPIOC, GPIOC_LED);
    ili9341_test(mode);
    palSetPad(GPIOC, GPIOC_LED);
    chThdSleepMilliseconds(50);
  }
#endif

#if 0
  //extern adcsample_t adc_samples[2];
  //chprintf(chp, "adc: %d %d\r\n", adc_samples[0], adc_samples[1]);
  int i;
  int x, y;
  for (i = 0; i < 50; i++) {
    test_touch(&x, &y);
    chprintf(chp, "adc: %d %d\r\n", x, y);
    chThdSleepMilliseconds(200);
  }
  //extern int touch_x, touch_y;
  //chprintf(chp, "adc: %d %d\r\n", touch_x, touch_y);
#endif

  while (argc > 1) {
    int x, y;
    touch_position(&x, &y);
    chprintf(chp, "touch: %d %d\r\n", x, y);
    chThdSleepMilliseconds(200);
  }
}

static void cmd_gain(BaseSequentialStream *chp, int argc, char *argv[])
{
  int rvalue;
  int lvalue = 0;
  if (argc != 1 && argc != 2) {
    chprintf(chp, "usage: gain {lgain(0-95)} [rgain(0-95)]\r\n");
    return;
  }
  rvalue = atoi(argv[0]);
  if (argc == 2) 
    lvalue = atoi(argv[1]);
  tlv320aic3204_set_gain(lvalue, rvalue);
}

static void cmd_port(BaseSequentialStream *chp, int argc, char *argv[])
{
  int port;
  if (argc != 1) {
    chprintf(chp, "usage: port {0:TX 1:RX}\r\n");
    return;
  }
  port = atoi(argv[0]);
  if (port)
    tlv320aic3204_select_in1();
  else
    tlv320aic3204_select_in3(); // default
}

static void cmd_stat(BaseSequentialStream *chp, int argc, char *argv[])
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

  chprintf(chp, "average: %d %d\r\n", stat.ave[0], stat.ave[1]);
  chprintf(chp, "rms: %d %d\r\n", stat.rms[0], stat.rms[1]);
  chprintf(chp, "callback count: %d\r\n", stat.callback_count);
  //chprintf(chp, "interval cycle: %d\r\n", stat.interval_cycles);
  //chprintf(chp, "busy cycle: %d\r\n", stat.busy_cycles);
  //chprintf(chp, "load: %d\r\n", stat.busy_cycles * 100 / stat.interval_cycles);
  extern int awd_count;
  chprintf(chp, "awd: %d\r\n", awd_count);
}


#ifndef VERSION
#define VERSION "unknown"
#endif

const char NANOVNA_VERSION[] = VERSION;

static void cmd_version(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argc;
  (void)argv;
  chprintf(chp, "%s\r\n", NANOVNA_VERSION);
}

static void cmd_vbat(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argc;
  (void)argv;
  chprintf(chp, "%d mV\r\n", vbat);
}

#ifdef __SCANRAW_CMD__
static THD_WORKING_AREA(waThread2, /* cmd_* max stack size + alpha */502 + 16);
#else
static THD_WORKING_AREA(waThread2, /* cmd_* max stack size + alpha */442);
#endif // __SCANRAW_CMD__

static const ShellCommand commands[] =
{
    { "version", cmd_version },
    { "reset", cmd_reset },
    { "freq", cmd_freq },
    { "offset", cmd_offset },
    { "time", cmd_time },
    { "dac", cmd_dac },
    { "saveconfig", cmd_saveconfig },
    { "clearconfig", cmd_clearconfig },
    { "data", cmd_data },
#ifdef ENABLED_DUMP
    { "dump", cmd_dump },
#endif
    { "frequencies", cmd_frequencies },
    { "port", cmd_port },
    { "stat", cmd_stat },
    { "gain", cmd_gain },
    { "power", cmd_power },
    { "sample", cmd_sample },
    //{ "gamma", cmd_gamma },
    { "scan", cmd_scan },
#ifdef __SCANRAW_CMD__
    { "scanraw", cmd_scanraw },
#endif // __SCANRAW_CMD__
    { "sweep", cmd_sweep },
    { "test", cmd_test },
    { "touchcal", cmd_touchcal },
    { "touchtest", cmd_touchtest },
    { "pause", cmd_pause },
    { "resume", cmd_resume },
    { "cal", cmd_cal },
    { "save", cmd_save },
    { "recall", cmd_recall },
    { "trace", cmd_trace },
    { "marker", cmd_marker },
    { "edelay", cmd_edelay },
    { "capture", cmd_capture },
    { "vbat", cmd_vbat },
    { "transform", cmd_transform },
    { "threshold", cmd_threshold },
    { NULL, NULL }
};

static const ShellConfig shell_cfg1 =
{
    .sc_channel  = (BaseSequentialStream *)&SDU1,
    .sc_commands = commands
};

static const I2CConfig i2ccfg = {
  .timingr  = 0x00300506, //voodoo magic 400kHz @ HSI 8MHz
  .cr1      = 0,
  .cr2      = 0
};

static DACConfig dac1cfg1 = {
  //.init =         2047U,
  .init =         1922U,
  .datamode =     DAC_DHRM_12BIT_RIGHT
};

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

  /*
   * Initialize graph plotting
   */
  plot_init();

  /* restore config */
  config_recall();

  dac1cfg1.init = config.dac_value;
  /*
   * Starting DAC1 driver, setting up the output pin as analog as suggested
   * by the Reference Manual.
   */
  dacStart(&DACD2, &dac1cfg1);

  /* initial frequencies */
  update_frequencies();

  /* restore frequencies and calibration properties from flash memory */
  if (config.default_loadcal >= 0)
    caldata_recall(config.default_loadcal);

  redraw_frame();

  /*
   * I2S Initialize
   */
  tlv320aic3204_init();
  i2sInit();
  i2sObjectInit(&I2SD2);
  i2sStart(&I2SD2, &i2sconfig);
  i2sStartExchange(&I2SD2);

  ui_init();

  /*
   * Shell manager initialization.
   */
    shellInit();

    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

    while (1) {
      if (SDU1.config->usbp->state == USB_ACTIVE) {
        thread_t *shelltp = chThdCreateStatic(waThread2, sizeof(waThread2), 
                                              NORMALPRIO + 1,
                                              shellThread, (void *)&shell_cfg1);
        chThdWait(shelltp);               /* Waiting termination.             */
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
