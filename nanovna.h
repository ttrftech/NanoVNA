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

/*
 * main.c
 */
extern float measured[2][101][2];

#define CAL_LOAD 0
#define CAL_OPEN 1
#define CAL_SHORT 2
#define CAL_THRU 3
#define CAL_ISOLN 4

#define CALSTAT_LOAD (1<<0)
#define CALSTAT_OPEN (1<<1)
#define CALSTAT_SHORT (1<<2)
#define CALSTAT_THRU (1<<3)
#define CALSTAT_ISOLN (1<<4)
#define CALSTAT_ES (1<<5)
#define CALSTAT_ER (1<<6)
#define CALSTAT_ET (1<<7)
#define CALSTAT_ED CALSTAT_LOAD
#define CALSTAT_EX CALSTAT_ISOLN
#define CALSTAT_APPLY (1<<8)
#define CALSTAT_INTERPOLATED (1<<9)

#define ETERM_ED 0 /* error term directivity */
#define ETERM_ES 1 /* error term source match */
#define ETERM_ER 2 /* error term refrection tracking */
#define ETERM_ET 3 /* error term transmission tracking */
#define ETERM_EX 4 /* error term isolation */

void cal_collect(int type);
void cal_done(void);

enum {
  ST_START, ST_STOP, ST_CENTER, ST_SPAN, ST_CW
};

void set_sweep_frequency(int type, float frequency);
uint32_t get_sweep_frequency(int type);

float my_atof(const char *p);

void toggle_sweep(void);

extern int8_t sweep_enabled;

/*
 * ui.c
 */
extern void ui_init(void);
extern void ui_process(void);

/*
 * dsp.c
 */
// 5ms @ 48kHz
#define AUDIO_BUFFER_LEN 96

extern int16_t rx_buffer[];

#define STATE_LEN 32
#define SAMPLE_LEN 48

extern int16_t ref_buf[];
extern int16_t samp_buf[];

void dsp_process(int16_t *src, size_t len);
void reset_dsp_accumerator(void);
void calculate_gamma(float *gamma);
void fetch_amplitude(float *gamma);
void fetch_amplitude_ref(float *gamma);

int si5351_set_frequency_with_offset(int freq, int offset, uint8_t drive_strength);


/*
 * tlv320aic3204.c
 */
typedef struct {
  int target_level;
  int gain_hysteresis;
  int attack;
  int attack_scale;
  int decay;
  int decay_scale;
} tlv320aic3204_agc_config_t;

extern void tlv320aic3204_init(void);
extern void tlv320aic3204_set_gain(int lgain, int rgain);
extern void tlv320aic3204_set_digital_gain(int gain);
extern void tlv320aic3204_set_volume(int gain);
extern void tlv320aic3204_agc_config(tlv320aic3204_agc_config_t *conf);
extern void tlv320aic3204_select_in1(void);
extern void tlv320aic3204_select_in3(void);
extern void tlv320aic3204_adc_filter_enable(int enable);


/*
 * plot.c
 */
#define OFFSETX 15
#define OFFSETY 0
#define WIDTH 291
#define HEIGHT 233

#define CELLOFFSETX 5
#define AREA_WIDTH_NORMAL (WIDTH + CELLOFFSETX*2)

extern int area_width;
extern int area_height;

#define GRIDY 29

// font

extern const uint16_t x5x7_bits [];
extern const uint32_t numfont20x24[][24];

#define S_PI    "\034"
#define S_MICRO "\035"
#define S_OHM   "\036"
#define S_DEGREE "\037"
#define S_LARROW "\032"
#define S_RARROW "\033"

// trace 

#define TRACES_MAX 4

enum {
  TRC_LOGMAG, TRC_PHASE, TRC_DELAY, TRC_SMITH, TRC_POLAR, TRC_LINEAR, TRC_SWR, TRC_REAL, TRC_IMAG, TRC_R, TRC_X, TRC_OFF
};

// LOGMAG: SCALE, REFPOS, REFVAL
// PHASE: SCALE, REFPOS, REFVAL
// DELAY: SCALE, REFPOS, REFVAL
// SMITH: SCALE, <REFPOS>, <REFVAL>
// LINMAG: SCALE, REFPOS, REFVAL
// SWR: SCALE, REFPOS, REFVAL

// Electrical Delay
// Phase

typedef struct {
  uint8_t enabled;
  uint8_t type;
  uint8_t channel;
  uint8_t polar;
  float scale;
  float refpos;
} trace_t;

typedef struct {
  int32_t magic;
  uint16_t dac_value;
  uint16_t grid_color;
  uint16_t menu_normal_color;
  uint16_t menu_active_color;
  uint16_t trace_color[TRACES_MAX];
  int16_t touch_cal[4];
  int8_t default_loadcal;
  int32_t checksum;
} config_t;

extern config_t config;

//extern trace_t trace[TRACES_MAX];

void set_trace_type(int t, int type);
void set_trace_channel(int t, int channel);
void set_trace_scale(int t, float scale);
void set_trace_refpos(int t, float refpos);
float get_trace_scale(int t);
float get_trace_refpos(int t);
const char *get_trace_typename(int t);
void draw_battery_status(void);

void set_electrical_delay(float picoseconds);
float get_electrical_delay(void);

// marker

typedef struct {
  int8_t enabled;
  int16_t index;
  uint32_t frequency;
} marker_t;

//extern marker_t markers[4];
//extern int active_marker;

void plot_init(void);
void update_grid(void);
void request_to_redraw_grid(void);
void redraw_frame(void);
//void redraw_all(void);
void request_to_draw_cells_behind_menu(void);
void request_to_draw_cells_behind_numeric_input(void);
void redraw_marker(int marker, int update_info);
void trace_get_info(int t, char *buf, int len);
void plot_into_index(float measured[2][101][2]);
void force_set_markmap(void);
void draw_all_cells(void);

void draw_cal_status(void);

void markmap_all_markers(void);

void marker_position(int m, int t, int *x, int *y);
int search_nearest_index(int x, int y, int t);

extern int8_t redraw_requested;
extern int16_t vbat;

/*
 * ili9341.c
 */
#define RGB565(b,r,g)     ( (((b)<<8)&0xfc00) | (((r)<<2)&0x03e0) | (((g)>>3)&0x001f) )

typedef struct {
	uint16_t width;
	uint16_t height;
	uint16_t scaley;
	uint16_t slide;
	const uint32_t *bitmap;
} font_t;

extern const font_t NF20x24;

extern uint16_t spi_buffer[1024];

void ili9341_init(void);
void ili9341_test(int mode);
void ili9341_bulk(int x, int y, int w, int h);
void ili9341_fill(int x, int y, int w, int h, int color);
void ili9341_drawchar_5x7(uint8_t ch, int x, int y, uint16_t fg, uint16_t bg);
void ili9341_drawstring_5x7(const char *str, int x, int y, uint16_t fg, uint16_t bg);
void ili9341_drawchar_size(uint8_t ch, int x, int y, uint16_t fg, uint16_t bg, uint8_t size);
void ili9341_drawstring_size(const char *str, int x, int y, uint16_t fg, uint16_t bg, uint8_t size);
void ili9341_drawfont(uint8_t ch, const font_t *font, int x, int y, uint16_t fg, uint16_t bg);
void ili9341_read_memory(int x, int y, int w, int h, int len, uint16_t* out);
void ili9341_read_memory_continue(int len, uint16_t* out);


/*
 * flash.c
 */
#define SAVEAREA_MAX 5

typedef struct {
  int32_t magic;
  int32_t _frequency0; // start or center
  int32_t _frequency1; // stop or span
  int16_t _sweep_points;
  uint16_t _cal_status;

  uint32_t _frequencies[101];
  float _cal_data[5][101][2];
  float _electrical_delay; // picoseconds
  
  trace_t _trace[TRACES_MAX];
  marker_t _markers[4];
  int _active_marker;

  int32_t checksum;
} properties_t;

#define CONFIG_MAGIC 0x434f4e45 /* 'CONF' */

extern int16_t lastsaveid;
extern properties_t *active_props;
extern properties_t current_props;

extern int8_t previous_marker;

#define frequency0 current_props._frequency0
#define frequency1 current_props._frequency1
#define sweep_points current_props._sweep_points
#define cal_status current_props._cal_status
#define frequencies current_props._frequencies
#define cal_data active_props->_cal_data
#define electrical_delay current_props._electrical_delay

#define trace current_props._trace
#define markers current_props._markers
#define active_marker current_props._active_marker

int caldata_save(int id);
int caldata_recall(int id);
const properties_t *caldata_ref(int id);

int config_save(void);
int config_recall(void);

void clear_all_config_prop_data(void);

/*
 * ui.c
 */

typedef struct {
  int8_t digit; /* 0~5 */
  int8_t digit_mode;
  int8_t current_trace; /* 0..3 */
  uint32_t value; // for editing at numeric input area
  uint32_t previous_value;
} uistat_t;

extern uistat_t uistat;
  
void ui_init(void);
void ui_show(void);
void ui_hide(void);

extern uint8_t operation_requested;

void handle_touch_interrupt(void);

#define TOUCH_THRESHOLD 2000

void touch_cal_exec(void);
void touch_draw_test(void);
void enter_dfu(void);

/*
 * adc.c
 */

void adc_init(void);
uint16_t adc_single_read(ADC_TypeDef *adc, uint32_t chsel);
void adc_start_analog_watchdogd(ADC_TypeDef *adc, uint32_t chsel);
void adc_stop(ADC_TypeDef *adc);
void adc_interrupt(ADC_TypeDef *adc);
int16_t adc_vbat_read(ADC_TypeDef *adc);

/*
 * misclinous
 */
#define PULSE do { palClearPad(GPIOC, GPIOC_LED); palSetPad(GPIOC, GPIOC_LED);} while(0)

// convert vbat [mV] to battery indicator
static inline uint8_t vbat2bati(int16_t vbat)
{
	if (vbat < 3200) return 0;
	if (vbat < 3450) return 25;
	if (vbat < 3700) return 50;
	if (vbat < 4100) return 75;
	return 100;
}

/*EOF*/
