
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

void ili9341_init(void);
void ili9341_test(int mode);

void set_sweep(int32_t start, int stop);
#if 0
void sweep_plot(int32_t freq, int first, float port0[2], float port1[2]);
void sweep_tail(void);
#endif
void redraw(void);
void polar_plot(float measured[101][4]);

extern uint16_t cal_status;
extern float cal_data[5][101][2];

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
#define ETERM_ER 1 /* error term refrection tracking */
#define ETERM_ES 2 /* error term source match */
#define ETERM_ET 3 /* error term transmission tracking */
#define ETERM_EX 4 /* error term isolation */


void plot_into_index(float measured[2][101][2]);
void draw_cell_all(void);

extern const uint16_t x5x7_bits [];
extern const uint32_t numfont20x24[][24];

