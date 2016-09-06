
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
#define AUDIO_BUFFER_LEN 480

extern int16_t rx_buffer[];
extern int16_t tx_buffer[];

extern int16_t buffer_i[];
extern int16_t buffer_q[];

void dsp_process(int16_t *src, int16_t *dst, size_t len);
void set_agc_mode(int agcmode);

void si5351_set_frequency_with_offset(int freq, int offset);
