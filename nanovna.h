
#include "ch.h"

/*
 * tlv320aic3204.c
 */
extern void I2CWrite(int addr, uint8_t d0, uint8_t d1);

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
extern int16_t tx_buffer[];

#define STATE_LEN 32
#define SAMPLE_LEN 48

extern int16_t ref_state[];
extern int16_t ref_buf[];
extern int16_t samp_buf[];

//extern int16_t refq_buf[];
extern int16_t refiq_buf[];

void dsp_process(int16_t *src, size_t len);
void calculate_gamma(float *gamma);

int si5351_set_frequency_with_offset(int freq, int offset, uint8_t drive_strength);

#define RGB565(b,r,g)     ( (((b)<<8)&0xfc00) | (((r)<<2)&0x03e0) | (((g)>>3)&0x001f) )

/*
 * ili9341.c
 */
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
void ili9341_drawfont(uint8_t ch, const font_t *font, int x, int y, uint16_t fg, uint16_t bg);


/*
 * plot.c
 */
#define OFFSETX 15
#define OFFSETY 0
#define WIDTH 291
#define HEIGHT 233

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

// trace 

#define TRACES_MAX 4

enum {
  TRC_LOGMAG, TRC_PHASE, TRC_SMITH, TRC_ADMIT, TRC_POLAR, TRC_LINEAR, TRC_SWR, TRC_OFF
};

extern const char *trc_type_name[];

// LOGMAG: SCALE, REFPOS, REFVAL
// PHASE: SCALE, REFPOS, REFVAL
// DELAY: SCALE, REFPOS, REFVAL
// SMITH: SCALE, <REFPOS>, <REFVAL>
// LINMAG: SCALE, REFPOS, REFVAL
// SWR: SCALE, REFPOS, REFVAL

// Electrical Delay
// Phase

typedef struct {
  int enabled;
  int type;
  int channel;
  float scale;
  //float ref;
  uint16_t color;
  uint8_t polar;
} trace_t;

//extern trace_t trace[TRACES_MAX];

void set_trace_type(int t, int type);
void set_trace_channel(int t, int channel);
void set_trace_scale(int t, float scale);

// marker

typedef struct {
  int enabled;
  //uint32_t frequency;
  int index;
} marker_t;

//extern marker_t markers[4];
//extern int active_marker;

void plot_init(void);
void update_grid(void);
void redraw(void);
void force_draw_cells(void);
void redraw_marker(int marker, int update_info);
void trace_get_info(int t, char *buf, int len);
void plot_into_index(float measured[2][101][2]);
void draw_cell_all(void);
void force_set_markmap(void);

void draw_cal_status(void);

void markmap_all_markers(void);


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

void set_sweep_frequency(int type, int frequency);

float my_atof(const char *p);

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

  trace_t _trace[TRACES_MAX];
  marker_t _markers[4];
  int _active_marker;

  int32_t checksum;
} config_t;

#define CONFIG_MAGIC 0x436f4e45 /* 'CoNF' */

extern int16_t lastsaveid;
extern config_t *active;
extern config_t current_config;

#define frequency0 current_config._frequency0
#define frequency1 current_config._frequency1
#define sweep_points current_config._sweep_points
#define cal_status current_config._cal_status
#define frequencies current_config._frequencies
#define cal_data active->_cal_data

#define trace current_config._trace
#define markers current_config._markers
#define active_marker current_config._active_marker

int caldata_save(int id);
int caldata_recall(int id);

/*
 * ui.c
 */
void ui_init(void);
void ui_show(void);
void ui_hide(void);

extern uint8_t operation_requested;


/*
 * adc.c
 */

void adc_init(void);
uint16_t adc_single_read(ADC_TypeDef *adc, uint32_t chsel);
void adc_start_analog_watchdogd(ADC_TypeDef *adc, uint32_t chsel);
void adc_stop(ADC_TypeDef *adc);
void adc_interrupt(ADC_TypeDef *adc);

/*
 * misclinous
 */
#define PULSE do { palClearPad(GPIOC, GPIOC_LED); palSetPad(GPIOC, GPIOC_LED);} while(0)

/*EOF*/
