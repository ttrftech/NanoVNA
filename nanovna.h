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

// Need enable HAL_USE_SPI in halconf.h
#define __USE_DISPLAY_DMA__

/*
 * main.c
 */

#define POINTS_COUNT 101
extern float measured[2][POINTS_COUNT][2];

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

#define DOMAIN_MODE (1<<0)
#define DOMAIN_FREQ (0<<0)
#define DOMAIN_TIME (1<<0)
#define TD_FUNC (0b11<<1)
#define TD_FUNC_BANDPASS (0b00<<1)
#define TD_FUNC_LOWPASS_IMPULSE (0b01<<1)
#define TD_FUNC_LOWPASS_STEP    (0b10<<1)
#define TD_WINDOW (0b11<<3)
#define TD_WINDOW_NORMAL (0b00<<3)
#define TD_WINDOW_MINIMUM (0b01<<3)
#define TD_WINDOW_MAXIMUM (0b10<<3)

#define FFT_SIZE 256

void cal_collect(int type);
void cal_done(void);

enum stimulus {
  ST_START, ST_STOP, ST_CENTER, ST_SPAN, ST_CW
};

void set_sweep_frequency(int type, uint32_t frequency);
uint32_t get_sweep_frequency(int type);

double my_atof(const char *p);

void toggle_sweep(void);

extern int8_t sweep_enabled;

/*
 * ui.c
 */
extern void ui_init(void);
extern void ui_process(void);

enum opreq { OP_NONE = 0, OP_LEVER, OP_TOUCH, OP_FREQCHANGE };
extern uint8_t operation_requested;

/*
 * dsp.c
 */
// 5ms @ 48kHz
#define AUDIO_BUFFER_LEN 96

extern int16_t rx_buffer[];

#define STATE_LEN 32
#define SAMPLE_LEN 48

#ifdef ENABLED_DUMP
extern int16_t ref_buf[];
extern int16_t samp_buf[];
#endif

void dsp_process(int16_t *src, size_t len);
void reset_dsp_accumerator(void);
void calculate_gamma(float *gamma);
void fetch_amplitude(float *gamma);
void fetch_amplitude_ref(float *gamma);

int si5351_set_frequency_with_offset(uint32_t freq, int offset, uint8_t drive_strength);


/*
 * tlv320aic3204.c
 */

extern void tlv320aic3204_init(void);
extern void tlv320aic3204_set_gain(int lgain, int rgain);
extern void tlv320aic3204_select(int channel);

/*
 * plot.c
 */

// Offset of plot area
#define OFFSETX 10
#define OFFSETY  0

// WIDTH better be n*(POINTS_COUNT-1)
#define WIDTH  300
// HEIGHT = 8*GRIDY
#define HEIGHT 232

//#define NGRIDY 10
#define NGRIDY 8

#define FREQUENCIES_XPOS1 OFFSETX
#define FREQUENCIES_XPOS2 200
#define FREQUENCIES_YPOS  (HEIGHT+1)

// GRIDX calculated depends from frequency span
//#define GRIDY 29
#define GRIDY (HEIGHT / NGRIDY)

//
#define CELLOFFSETX 5
#define AREA_WIDTH_NORMAL  (CELLOFFSETX + WIDTH  + 1 + 4)
#define AREA_HEIGHT_NORMAL (              HEIGHT + 1)

// Smith/polar chart
#define P_CENTER_X (CELLOFFSETX + WIDTH/2)
#define P_CENTER_Y (HEIGHT/2)
#define P_RADIUS   (HEIGHT/2)

extern int16_t area_width;
extern int16_t area_height;

// font
extern const uint8_t x5x7_bits [];
#define FONT_GET_DATA(ch)	(&x5x7_bits[ch*7])
#define FONT_GET_WIDTH(ch)	(8-(x5x7_bits[ch*7]&7))
#define FONT_GET_HEIGHT		7

extern const uint16_t numfont16x22[];
#define NUM_FONT_GET_DATA(ch)	(&numfont16x22[ch*22])
#define NUM_FONT_GET_WIDTH  	16
#define NUM_FONT_GET_HEIGHT		22

#define S_DELTA "\004"
#define S_DEGREE "\037"
#define S_SARROW "\030"
#define S_INFINITY "\031"
#define S_LARROW "\032"
#define S_RARROW "\033"
#define S_PI    "\034"
#define S_MICRO "\035"
#define S_OHM   "\036"
// trace 

#define TRACES_MAX 4

enum trace_type {
  TRC_LOGMAG, TRC_PHASE, TRC_DELAY, TRC_SMITH, TRC_POLAR, TRC_LINEAR, TRC_SWR, TRC_REAL, TRC_IMAG, TRC_R, TRC_X, TRC_OFF
};
// Mask for define rectangular plot
#define RECTANGULAR_GRID_MASK ((1<<TRC_LOGMAG)|(1<<TRC_PHASE)|(1<<TRC_DELAY)|(1<<TRC_LINEAR)|(1<<TRC_SWR)|(1<<TRC_REAL)|(1<<TRC_IMAG)|(1<<TRC_R)|(1<<TRC_X))

// LOGMAG: SCALE, REFPOS, REFVAL
// PHASE: SCALE, REFPOS, REFVAL
// DELAY: SCALE, REFPOS, REFVAL
// SMITH: SCALE, <REFPOS>, <REFVAL>
// LINMAG: SCALE, REFPOS, REFVAL
// SWR: SCALE, REFPOS, REFVAL

// Electrical Delay
// Phase

typedef struct trace {
  uint8_t enabled;
  uint8_t type;
  uint8_t channel;
  uint8_t reserved;
  float scale;
  float refpos;
} trace_t;

typedef struct config {
  int32_t magic;
  uint16_t dac_value;
  uint16_t grid_color;
  uint16_t menu_normal_color;
  uint16_t menu_active_color;
  uint16_t trace_color[TRACES_MAX];
  int16_t touch_cal[4];
  int8_t default_loadcal;
  uint32_t harmonic_freq_threshold;

  uint8_t _reserved[24];
  uint32_t checksum;
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
float groupdelay_from_array(int i, float array[POINTS_COUNT][2]);

// marker

#define MARKERS_MAX 4

typedef struct marker {
  int8_t enabled;
  int16_t index;
  uint32_t frequency;
} marker_t;

extern int8_t previous_marker;
extern int8_t marker_tracking;

void plot_init(void);
void update_grid(void);
void request_to_redraw_grid(void);
void redraw_frame(void);
//void redraw_all(void);
void request_to_draw_cells_behind_menu(void);
void request_to_draw_cells_behind_numeric_input(void);
void redraw_marker(int marker, int update_info);
void plot_into_index(float measured[2][POINTS_COUNT][2]);
void force_set_markmap(void);
void draw_frequencies(void);
void draw_all(bool flush);

void draw_cal_status(void);

//void markmap_all_markers(void);

void marker_position(int m, int t, int *x, int *y);
int search_nearest_index(int x, int y, int t);
void set_marker_search(int mode);
int marker_search(void);
int marker_search_left(int from);
int marker_search_right(int from);

extern uint16_t redraw_request;

#define REDRAW_CELLS      (1<<0)
#define REDRAW_FREQUENCY  (1<<1)
#define REDRAW_CAL_STATUS (1<<2)
#define REDRAW_MARKER     (1<<3)

extern int16_t vbat;

/*
 * ili9341.c
 */
// SPI bus revert byte order
//gggBBBbb RRRrrGGG
#define RGB565(r,g,b)  ( (((g)&0x1c)<<11) | (((b)&0xf8)<<5) | ((r)&0xf8) | (((g)&0xe0)>>5) )
#define RGBHEX(hex) ( (((hex)&0x001c00)<<3) | (((hex)&0x0000f8)<<5) | (((hex)&0xf80000)>>16) | (((hex)&0x00e000)>>13) )

#define DEFAULT_FG_COLOR			RGB565(255,255,255)
#define DEFAULT_BG_COLOR			RGB565(  0,  0,  0)
#define DEFAULT_GRID_COLOR			RGB565(128,128,128)
#define DEFAULT_MENU_COLOR			RGB565(255,255,255)
#define DEFAULT_MENU_TEXT_COLOR		RGB565(  0,  0,  0)
#define DEFAULT_MENU_ACTIVE_COLOR	RGB565(180,255,180)
#define DEFAULT_TRACE_1_COLOR		RGB565(255,255,  0)
#define DEFAULT_TRACE_2_COLOR		RGB565(  0,255,255)
#define DEFAULT_TRACE_3_COLOR		RGB565(  0,255,  0)
#define DEFAULT_TRACE_4_COLOR		RGB565(255,  0,255)
#define DEFAULT_NORMAL_BAT_COLOR	RGB565( 31,227,  0)
#define DEFAULT_LOW_BAT_COLOR		RGB565(255,  0,  0)
#define	DEFAULT_SPEC_INPUT_COLOR	RGB565(128,255,128);

extern uint16_t foreground_color;
extern uint16_t background_color;

extern uint16_t spi_buffer[2048];

void ili9341_init(void);
//void ili9341_setRotation(uint8_t r);
void ili9341_test(int mode);
void ili9341_bulk(int x, int y, int w, int h);
void ili9341_fill(int x, int y, int w, int h, int color);
void setForegroundColor(uint16_t fg);
void setBackgroundColor(uint16_t fg);
void blit8BitWidthBitmap(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *bitmap);
void blit16BitWidthBitmap(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint16_t *bitmap);
void ili9341_drawchar(uint8_t ch, int x, int y);
void ili9341_drawstring(const char *str, int x, int y);
void ili9341_drawstringV(const char *str, int x, int y);
int  ili9341_drawchar_size(uint8_t ch, int x, int y, uint8_t size);
void ili9341_drawstring_size(const char *str, int x, int y, uint8_t size);
void ili9341_drawfont(uint8_t ch, int x, int y);
void ili9341_read_memory(int x, int y, int w, int h, int len, uint16_t* out);
void ili9341_line(int x0, int y0, int x1, int y1);
void show_version(void);
void show_logo(void);

/*
 * flash.c
 */
#define SAVEAREA_MAX 5

typedef struct properties {
  uint32_t magic;
  uint32_t _frequency0;
  uint32_t _frequency1;
  uint16_t _sweep_points;
  uint16_t _cal_status;

  uint32_t _frequencies[POINTS_COUNT];
  float _cal_data[5][POINTS_COUNT][2];
  float _electrical_delay; // picoseconds
  
  trace_t _trace[TRACES_MAX];
  marker_t _markers[MARKERS_MAX];

  float _velocity_factor; // %
  int8_t _active_marker;
  uint8_t _domain_mode; /* 0bxxxxxffm : where ff: TD_FUNC m: DOMAIN_MODE */
  uint8_t _marker_smith_format;
  uint8_t _reserved[50];
  uint32_t checksum;
} properties_t;

//sizeof(properties_t) == 0x1200

#define CONFIG_MAGIC 0x434f4e45 /* 'CONF' */

extern int16_t lastsaveid;
extern properties_t *active_props;
extern properties_t current_props;

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
#define domain_mode current_props._domain_mode
#define velocity_factor current_props._velocity_factor
#define marker_smith_format current_props._marker_smith_format

#define FREQ_IS_STARTSTOP() (frequency0 < frequency1)
#define FREQ_IS_CENTERSPAN() (frequency0 > frequency1)
#define FREQ_IS_CW() (frequency0 == frequency1)
#define FREQ_START() (frequency0)
#define FREQ_STOP() (frequency1)
#define FREQ_CENTER() (frequency0/2 + frequency1/2)
#define FREQ_SPAN() (frequency0 - frequency1)

int caldata_save(int id);
int caldata_recall(int id);
const properties_t *caldata_ref(int id);

int config_save(void);
int config_recall(void);

void clear_all_config_prop_data(void);

/*
 * ui.c
 */

// lever_mode
enum lever_mode {
  LM_MARKER, LM_SEARCH, LM_CENTER, LM_SPAN, LM_EDELAY
};

// marker smith value format
enum marker_smithvalue {
  MS_LIN, MS_LOG, MS_REIM, MS_RX, MS_RLC
};

typedef struct uistat {
  int8_t digit; /* 0~5 */
  int8_t digit_mode;
  int8_t current_trace; /* 0..3 */
  uint32_t value; // for editing at numeric input area
  uint32_t previous_value;
  uint8_t lever_mode;
  bool marker_delta;
} uistat_t;

extern uistat_t uistat;
  
void ui_init(void);
void ui_show(void);
void ui_hide(void);

void touch_start_watchdog(void);
void touch_position(int *x, int *y);
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

/*EOF*/
