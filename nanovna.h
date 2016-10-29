
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

extern void ui_init(void);
extern void ui_process(void);

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
void calclate_gamma(float *gamma);

int si5351_set_frequency_with_offset(int freq, int offset, uint8_t drive_strength);

#define RGB565(b,r,g)     ( (((b)<<8)&0xfc00) | (((r)<<2)&0x03e0) | (((g)>>3)&0x001f) )

extern uint16_t spi_buffer[1024];

void ili9341_init(void);
void ili9341_test(int mode);
void ili9341_bulk(int x, int y, int w, int h);
void ili9341_fill(int x, int y, int w, int h, int color);
void ili9341_drawchar_5x7(uint8_t ch, int x, int y, uint16_t fg, uint16_t bg);
void ili9341_drawstring_5x7(char *str, int x, int y, uint16_t fg, uint16_t bg);

void plot_init(void);


void set_sweep(int32_t start, int stop);
void redraw(void);

#define TRACES_MAX 4

enum {
  TRC_LOGMAG, TRC_PHASE, TRC_SMITH, TRC_ADMIT, TRC_POLAR, TRC_LINEAR, TRC_SWR
};

extern const char *trc_type_name[];

typedef struct {
  int enabled;
  int type;
  int channel;
  float scale;
  uint16_t color;
  uint8_t polar;
} trace_t;

extern trace_t trace[TRACES_MAX];
extern float measured[2][101][2];

void trace_get_info(int t, char *buf, int len);

typedef struct {
  int enabled;
  //uint32_t frequency;
  int index;
} marker_t;

extern marker_t markers[4];
extern int active_marker;

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


void plot_into_index(float measured[2][101][2]);
void draw_cell_all(void);

extern const uint16_t x5x7_bits [];
extern const uint32_t numfont20x24[][24];

#define CHAR_OHM  '\0x1e'

int caldata_save(void);
int caldata_recall(void);

typedef struct {
  int32_t magic;
  int32_t _freq_start;
  int32_t _freq_stop;
  int16_t _sweep_points;
  uint16_t _cal_status;

  uint32_t _frequencies[101];
  float _cal_data[5][101][2];
  int32_t checksum;
} config_t;

#define CONFIG_MAGIC 0x436f4e45 /* 'CoNF' */

extern config_t *active;
extern config_t current_config;

#define freq_start active->_freq_start
#define freq_stop active->_freq_stop
#define sweep_points active->_sweep_points
#define cal_status active->_cal_status
#define frequencies active->_frequencies
#define cal_data active->_cal_data
