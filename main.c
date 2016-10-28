#include "ch.h"
#include "hal.h"
#include "usbcfg.h"
#include "si5351.h"
#include "nanovna.h"

#include <chprintf.h>
#include <shell.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

RTCDateTime timespec;

static void apply_error_term(void);


static const I2CConfig i2ccfg = {
  0x00300506, //voodoo magic 400kHz @ HSI 8MHz
  //0x00902025, //voodoo magic
  //0x00420F13,  // 100kHz @ 72MHz
  0,
  0
};

void I2CWrite(int addr, uint8_t d0, uint8_t d1)
{
    uint8_t buf[] = { d0, d1 };
    i2cAcquireBus(&I2CD1);
    (void)i2cMasterTransmitTimeout(&I2CD1, addr, buf, 2, NULL, 0, 1000);
    i2cReleaseBus(&I2CD1);
}

int I2CRead(int addr, uint8_t d0)
{
    uint8_t buf[] = { d0 };
    i2cAcquireBus(&I2CD1);
    i2cMasterTransmitTimeout(&I2CD1, addr, buf, 1, buf, 1, 1000);
    i2cReleaseBus(&I2CD1);
    return buf[0];
}

void scan_lcd(void);

static MUTEX_DECL(mutex);



static THD_WORKING_AREA(waThread1, 384);
static THD_FUNCTION(Thread1, arg)
{
    (void)arg;

    chRegSetThreadName("blink");

    palSetPadMode(GPIOC, 13, PAL_MODE_OUTPUT_PUSHPULL);
    while (1)
    {
#if 0
      systime_t time = 500;
      if (serusbcfg.usbp->state != USB_ACTIVE)
        palClearPad(GPIOC, 13);
      chThdSleepMilliseconds(time);
      palSetPad(GPIOC, 13);
      chThdSleepMilliseconds(time);
#else
      chMtxLock(&mutex);
      scan_lcd();
      chMtxUnlock(&mutex);
#endif
    }
}

void
pause_sweep(void)
{
  chMtxLock(&mutex);
}

void
resume_sweep(void)
{
  chMtxUnlockAll();
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
    resume_sweep();
}

static void cmd_reset(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    chprintf(chp, "Performing reset\r\n");

    rccEnableWWDG(FALSE);

    WWDG->CFR = 0x60;
    WWDG->CR = 0xff;

    while (1)
	;
}


int32_t frequency_offset = 5000;
int32_t frequency = 10000000;
uint8_t drive_strength = SI5351_CLK_DRIVE_STRENGTH_2MA;

int set_frequency(int freq)
{
#if 0
    si5351_set_frequency(0, freq + frequency_offset);
    si5351_set_frequency(1, freq);
    frequency = freq;
#else
    int delay;
    delay = si5351_set_frequency_with_offset(freq, frequency_offset, drive_strength);
    frequency = freq;
    return delay;
#endif
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
    pause_sweep();
    if (argc != 1) {
        chprintf(chp, "usage: freq {frequency(Hz)}\r\n");
        return;
    }
    freq = atoi(argv[0]);
    set_frequency(freq);
}

static void cmd_power(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc != 1) {
        chprintf(chp, "usage: power {0-3}\r\n");
        return;
    }
    drive_strength = atoi(argv[0]);
    set_frequency(frequency);
}



static void cmd_time(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    rtcGetTime(&RTCD1, &timespec);
    chprintf(chp, "%d/%d/%d %d\r\n", timespec.year+1980, timespec.month, timespec.day, timespec.millisecond);
}


static const DACConfig dac1cfg1 = {
  //init:         2047U,
  init:         1922U,
  datamode:     DAC_DHRM_12BIT_RIGHT
};

static void cmd_dac(BaseSequentialStream *chp, int argc, char *argv[])
{
    int value;
    if (argc != 1) {
        chprintf(chp, "usage: dac {value(0-4095)}\r\n");
        return;
    }
    value = atoi(argv[0]);
    dacPutChannelX(&DACD2, 0, value);
}




static struct {
  int16_t rms[2];
  int16_t ave[2];
  int callback_count;

  int32_t last_counter_value;
  int32_t interval_cycles;
  int32_t busy_cycles;
} stat;

int16_t rx_buffer[AUDIO_BUFFER_LEN * 2];

int16_t dump_buffer[AUDIO_BUFFER_LEN];
volatile int16_t wait_count = 0;
int16_t dump_selection = 0;

int16_t dsp_disabled = FALSE;
float measured[2][101][2];



void i2s_end_callback(I2SDriver *i2sp, size_t offset, size_t n)
{
#if PORT_SUPPORTS_RT
  int32_t cnt_s = port_rt_get_counter_value();
  int32_t cnt_e;
#endif
  int16_t *p = &rx_buffer[offset];
  (void)i2sp;
  (void)n;
  //palClearPad(GPIOC, GPIOC_LED);

  if (!dsp_disabled)
    dsp_process(p, n);

  if (wait_count > 0) {
    if (dump_selection == 1)
      p = samp_buf;
    else if (dump_selection == 2)
      p = ref_buf;
    else if (dump_selection == 3)
      p = refiq_buf;
    if (wait_count == 1)
      memcpy(dump_buffer, p, sizeof dump_buffer);
    --wait_count;
  }

#if PORT_SUPPORTS_RT
  cnt_e = port_rt_get_counter_value();
  stat.interval_cycles = cnt_s - stat.last_counter_value;
  stat.busy_cycles = cnt_e - cnt_s;
  stat.last_counter_value = cnt_s;
#endif
  stat.callback_count++;
  //palSetPad(GPIOC, GPIOC_LED);
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

static void cmd_data(BaseSequentialStream *chp, int argc, char *argv[])
{
  int i;
  int len;
  int sel = 0;

  if (argc == 1)
    sel = atoi(argv[0]);
  if (sel == 0 || sel == 1) {
    pause_sweep();
    for (i = 0; i < 101; i++) {
      chprintf(chp, "%f %f\r\n", measured[sel][i][0], measured[sel][i][1]);
    }
  } else if (sel >= 2 && sel < 7) {
    pause_sweep();
    for (i = 0; i < 101; i++) {
      chprintf(chp, "%f %f\r\n", cal_data[sel-2][i][0], cal_data[sel-2][i][1]);
    }
  } else {
    chprintf(chp, "usage: data [array]\r\n");
  }
}

static void cmd_dump(BaseSequentialStream *chp, int argc, char *argv[])
{
  int i, j;
  int len;

  pause_sweep();
  if (argc == 1)
    dump_selection = atoi(argv[0]);

  wait_count = 3;
  //palClearPad(GPIOC, GPIOC_LED);
  while (wait_count)
    ;
  len = AUDIO_BUFFER_LEN;
  if (dump_selection == 1 || dump_selection == 2)
    len /= 2;
  for (i = 0; i < len; ) {
    for (j = 0; j < 16; j++, i++) {
      chprintf(chp, "%04x ", 0xffff & (int)dump_buffer[i]);
    }
    chprintf(chp, "\r\n");
  }
  //palSetPad(GPIOC, GPIOC_LED);
}

static void cmd_gamma(BaseSequentialStream *chp, int argc, char *argv[])
{
  float gamma[2];
  (void)argc;
  (void)argv;
  
  pause_sweep();
  wait_count = 4;
  while (wait_count)
    ;
  dsp_disabled = TRUE;
  calclate_gamma(gamma);
  dsp_disabled = FALSE;

  chprintf(chp, "%d %d\r\n", gamma[0], gamma[1]);
}

#if 0
int32_t freq_start = 1000000;
int32_t freq_stop = 300000000;
int16_t sweep_points = 101;

uint32_t frequencies[101];
uint16_t cal_status;
float cal_data[5][101][2];
#endif

config_t current_config = {
  /* magic */   CONFIG_MAGIC,
  /* freq_start */   1000000,
  /* freq_stop */  300000000,
  /* sweep_points */     101,
  /* cal_status */         0,
  /* frequencies */       {},
  /* cal_data */          {},
  /* checksum */           0
};
config_t *active = &current_config;



static void cmd_scan(BaseSequentialStream *chp, int argc, char *argv[])
{
  float gamma[2];
  int i;
  int32_t freq, step;
  int delay;
  (void)argc;
  (void)argv;

  pause_sweep();
  freq = freq_start;
  step = (freq_stop - freq_start) / (sweep_points-1);
  delay = set_frequency(freq);
  delay += 2;
  for (i = 0; i < sweep_points; i++) {
    freq = freq + step;
    wait_count = delay + 1;
    while (wait_count)
      ;
    //dsp_disabled = TRUE;
    __disable_irq();
    delay = set_frequency(freq);
    palClearPad(GPIOC, GPIOC_LED);
    calclate_gamma(gamma);
    palSetPad(GPIOC, GPIOC_LED);
    //dsp_disabled = FALSE;
    __enable_irq();
    chprintf(chp, "%d %d\r\n", gamma[0], gamma[1]);
  }
}

void scan_lcd(void)
{
  int i;
  int delay;
  //int first = TRUE;

  delay = set_frequency(frequencies[0]);
  delay += 2;
  for (i = 0; i < sweep_points; i++) {
    wait_count = delay + 2;
    tlv320aic3204_select_in3();
    while (wait_count)
      ;
    palClearPad(GPIOC, GPIOC_LED);
    __disable_irq();
    calclate_gamma(measured[0][i]);
    __enable_irq();

    tlv320aic3204_select_in1();
    wait_count = 2 + 2;
    while (wait_count)
      ;
    __disable_irq();
    calclate_gamma(measured[1][i]);
    __enable_irq();

    delay = set_frequency(frequencies[(i+1)%sweep_points]);
#if 0
    sweep_plot(frequencies[i], first, measured[0][i], measured[1][i]);
    first = FALSE;
#endif
    palSetPad(GPIOC, GPIOC_LED);
  }
#if 0
  for (i = 0; i < sweep_points; i++) {
    sweep_plot(frequencies[i], first, measured[0][i], measured[1][i]);
    first = FALSE;
  }  
#endif
#if 0
  sweep_tail();
  polar_plot(measured);
#endif
  if (cal_status & CALSTAT_APPLY)
    apply_error_term();
  plot_into_index(measured);
  draw_cell_all();
}

static void cmd_scan_lcd(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)chp;
  (void)argc;
  (void)argv;
  pause_sweep();
  scan_lcd();
}


void
set_frequencies(void)
{
  int i;
  int32_t span = (freq_stop - freq_start)/100;
  for (i = 0; i < sweep_points; i++)
    frequencies[i] = freq_start + span * i / (sweep_points - 1) * 100;
}

static void cmd_sweep(BaseSequentialStream *chp, int argc, char *argv[])
{
  if (argc == 0) {
    chprintf(chp, "%d %d %d\r\n", freq_start, freq_stop, sweep_points);
    return;
  } else if (argc > 3) {
    chprintf(chp, "usage: sweep {start(Hz)} [stop] [points]\r\n");
    return;
  }
  if (argc >= 1) {
    int32_t x = atoi(argv[0]);
    if (x < 300000) {
      chprintf(chp, "bad parameter\r\n");
      return;
    }
    freq_start = x;
  }
  if (argc >= 2) {
    int32_t x = atoi(argv[1]);
    if (x < 300000 || x <= freq_start) {
      chprintf(chp, "bad parameter\r\n");
      return;
    }
    freq_stop = x;
  }
  if (argc >= 3) {
    int32_t x = atoi(argv[2]);
    if (x < 1 || x > 1601) {
      chprintf(chp, "bad parameter\r\n");
      return;
    }
    sweep_points = x;
  }

  set_frequencies();
  set_sweep(freq_start, freq_stop);
}


static void
eterm_set(int term, float re, float im)
{
  int i;
  for (i = 0; i < 101; i++) {
    cal_data[term][i][0] = re;
    cal_data[term][i][1] = im;
  }
}

static void
eterm_copy(int dst, int src)
{
  memcpy(cal_data[dst], cal_data[src], sizeof cal_data[dst]);
}


struct open_model {
  float c0;
  float c1;
  float c2;
  float c3;
} open_model = { 50, 0, -300, 27 };

#if 1
static void
adjust_ed(void)
{
  int i;
  for (i = 0; i < 101; i++) {
    // z=1/(jwc*z0) = 1/(2*pi*f*c*z0)  Note: normalized with Z0
    // s11ao = (z-1)/(z+1) = (1-1/z)/(1+1/z) = (1-jwcz0)/(1+jwcz0)
    // prepare 1/s11ao for effeiciency
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
  for (i = 0; i < 101; i++) {
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
  for (i = 0; i < 101; i++) {
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
    cal_data[ETERM_ES][i][1] = 0;
  }
  cal_status &= ~CALSTAT_SHORT;
  cal_status |= CALSTAT_ER;
}

// CAUTION: Et is inversed for efficiency
static void
eterm_calc_et(void)
{
  int i;
  for (i = 0; i < 101; i++) {
    // Et = 1/(S21mt - Ex)(1 - Es)
    float esr = 1 - cal_data[ETERM_ES][i][0];
    float esi = -cal_data[ETERM_ES][i][1];
    float s21mr = cal_data[CAL_THRU][i][0] - cal_data[CAL_ISOLN][i][0];
    float s21mi = cal_data[CAL_THRU][i][1] - cal_data[CAL_ISOLN][i][1];
    float etr = esr * s21mr - esi * s21mi;
    float eti = esr * s21mi + esi * s21mr;
    float sq = etr*etr + eti*eti;
    float invr = etr / sq;
    float invi = -eti / sq;
    cal_data[ETERM_ET][i][0] = invr;
    cal_data[ETERM_ET][i][1] = invi;
  }
  cal_status &= ~CALSTAT_THRU;
  cal_status |= CALSTAT_ET;
}

void apply_error_term(void)
{
  int i;
  for (i = 0; i < 101; i++) {
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
    cal_status |= CALSTAT_LOAD;
    chMtxLock(&mutex);
    memcpy(cal_data[CAL_LOAD], measured[0], sizeof measured[0]);
    chMtxUnlock(&mutex);
  } else if (strcmp(cmd, "open") == 0) {
    cal_status |= CALSTAT_OPEN;
    cal_status &= ~(CALSTAT_ES|CALSTAT_APPLY);
    chMtxLock(&mutex);
    memcpy(cal_data[CAL_OPEN], measured[0], sizeof measured[0]);
    chMtxUnlock(&mutex);
  } else if (strcmp(cmd, "short") == 0) {
    cal_status |= CALSTAT_SHORT;
    cal_status &= ~(CALSTAT_ER|CALSTAT_APPLY);
    chMtxLock(&mutex);
    memcpy(cal_data[CAL_SHORT], measured[0], sizeof measured[0]);
    chMtxUnlock(&mutex);
  } else if (strcmp(cmd, "thru") == 0) {
    cal_status |= CALSTAT_THRU;
    chMtxLock(&mutex);
    memcpy(cal_data[CAL_THRU], measured[1], sizeof measured[0]);
    chMtxUnlock(&mutex);
  } else if (strcmp(cmd, "isoln") == 0) {
    cal_status |= CALSTAT_ISOLN;
    chMtxLock(&mutex);
    memcpy(cal_data[CAL_ISOLN], measured[1], sizeof measured[0]);
    chMtxUnlock(&mutex);
  } else if (strcmp(cmd, "done") == 0) {
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
    return;
  } else if (strcmp(cmd, "on") == 0) {
    cal_status |= CALSTAT_APPLY;
    return;
  } else if (strcmp(cmd, "off") == 0) {
    cal_status &= ~CALSTAT_APPLY;
    return;
  } else if (strcmp(cmd, "reset") == 0) {
    cal_status = 0;
    return;
  } else if (strcmp(cmd, "data") == 0) {
    chprintf(chp, "%f %f\r\n", cal_data[CAL_LOAD][0][0], cal_data[CAL_LOAD][0][1]);
    chprintf(chp, "%f %f\r\n", cal_data[CAL_OPEN][0][0], cal_data[CAL_OPEN][0][1]);
    chprintf(chp, "%f %f\r\n", cal_data[CAL_SHORT][0][0], cal_data[CAL_SHORT][0][1]);
    chprintf(chp, "%f %f\r\n", cal_data[CAL_THRU][0][0], cal_data[CAL_THRU][0][1]);
    chprintf(chp, "%f %f\r\n", cal_data[CAL_ISOLN][0][0], cal_data[CAL_ISOLN][0][1]);
    return;
  } else {
    chprintf(chp, "usage: cal [load|open|short|thru|isoln|done|reset|on|off]\r\n");
    return;
  }
}

static void cmd_save(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)chp;
  (void)argc;
  (void)argv;
  caldata_save();
}

static void cmd_recall(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)chp;
  (void)argc;
  (void)argv;
  caldata_recall();
}


const char *trc_type_name[] = {
  "LOGMAG", "PHASE", "SMITH", "ADMIT", "POLAR", "LINEAR", "SWR"
};
const char *trc_channel_name[] = {
  "S11", "S21"
};

static void cmd_trace(BaseSequentialStream *chp, int argc, char *argv[])
{
  int t;
  if (argc == 0) {
    for (t = 0; t < 4; t++) {
      if (trace[t].enabled) {
        const char *type = trc_type_name[trace[t].type];
        const char *channel = trc_channel_name[trace[t].channel];
        chprintf(chp, "%d %s %s\r\n", t, type, channel);
      }
    }
    return;
  } 
  t = atoi(argv[0]);
  if (t < 0 || t >= 4)
    goto usage;
  if (argc == 1) {
    const char *type = trc_type_name[trace[t].type];
    const char *channel = trc_channel_name[trace[t].channel];
    chprintf(chp, "%d %s %s\r\n", t, type, channel);
    return;
  }
  if (argc > 1) {
    if (strcmp(argv[1], "logmag") == 0) {
      trace[t].type = TRC_LOGMAG;
      trace[t].polar = FALSE;
      trace[t].enabled = TRUE;
    } else if (strcmp(argv[1], "phase") == 0) {
      trace[t].type = TRC_PHASE;
      trace[t].polar = FALSE;
      trace[t].enabled = TRUE;
    } else if (strcmp(argv[1], "polar") == 0) {
      trace[t].type = TRC_POLAR;
      trace[t].polar = TRUE;
      trace[t].enabled = TRUE;
    } else if (strcmp(argv[1], "smith") == 0) {
      trace[t].type = TRC_SMITH;
      trace[t].polar = TRUE;
      trace[t].enabled = TRUE;
    } else if (strcmp(argv[1], "admit") == 0) {
      trace[t].type = TRC_ADMIT;
      trace[t].polar = TRUE;
      trace[t].enabled = TRUE;
    } else if (strcmp(argv[1], "linear") == 0) {
      trace[t].type = TRC_LINEAR;
      trace[t].polar = FALSE;
      trace[t].enabled = TRUE;
    } else if (strcmp(argv[1], "swr") == 0) {
      trace[t].type = TRC_SWR;
      trace[t].polar = FALSE;
      trace[t].enabled = TRUE;
    } else if (strcmp(argv[1], "off") == 0) {
      trace[t].enabled = FALSE;
    } else if (strcmp(argv[1], "scale") == 0 && argc >= 3) {
      trace[t].scale = atoi(argv[2]);
      goto exit;
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
  chprintf(chp, "trace [n] [logmag|phase|smith|swr] [src]\r\n");
}

static void cmd_marker(BaseSequentialStream *chp, int argc, char *argv[])
{
  int t;
  if (argc == 0) {
    for (t = 0; t < 4; t++) {
      if (markers[t].enabled) {
        chprintf(chp, "%d %d\r\n", t+1, markers[t].index);
      }
    }
    return;
  } 
  t = atoi(argv[0])-1;
  if (t < 0 || t >= 4)
    goto usage;
  if (argc == 1) {
    chprintf(chp, "%d %d\r\n", t+1, markers[t].index);
    return;
  }
  if (argc > 1) {
    if (strcmp(argv[1], "off") == 0) {
      markers[t].enabled = FALSE;
      if (active_marker == t)
        active_marker = -1;
    } else if (strcmp(argv[1], "on") == 0) {
      markers[t].enabled = TRUE;
      active_marker = t;
    } else {
      markers[t].enabled = TRUE;
      int index = atoi(argv[1]);
      markers[t].index = index;
      active_marker = t;
    }
  }
  return;
 usage:
  chprintf(chp, "marker [n] [off|{index}]\r\n");
}


static void cmd_test(BaseSequentialStream *chp, int argc, char *argv[])
{
  int i;
  (void)chp;
  (void)argc;
  (void)argv;

  pause_sweep();
#if 0
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

#if 1
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
  chprintf(chp, "interval cycle: %d\r\n", stat.interval_cycles);
  chprintf(chp, "busy cycle: %d\r\n", stat.busy_cycles);
  chprintf(chp, "load: %d\r\n", stat.busy_cycles * 100 / stat.interval_cycles);
}





#define SHELL_WA_SIZE THD_WORKING_AREA_SIZE(454)

static const ShellCommand commands[] =
{
    { "reset", cmd_reset },
    { "freq", cmd_freq },
    { "offset", cmd_offset },
    { "time", cmd_time },
    { "dac", cmd_dac },
    { "data", cmd_data },
    { "dump", cmd_dump },
    { "port", cmd_port },
    { "stat", cmd_stat },
    { "gain", cmd_gain },
    { "power", cmd_power },
    { "gamma", cmd_gamma },
    { "scan", cmd_scan },
    { "sweep", cmd_sweep },
    { "test", cmd_test },
    { "plot", cmd_scan_lcd },
    { "pause", cmd_pause },
    { "resume", cmd_resume },
    { "cal", cmd_cal },
    { "save", cmd_save },
    { "recall", cmd_recall },
    { "trace", cmd_trace },
    { "marker", cmd_marker },
    { NULL, NULL }
};

static const ShellConfig shell_cfg1 =
{
    (BaseSequentialStream *)&SDU1,
    commands
};

int main(void)
{
    halInit();
    chSysInit();

    chMtxObjectInit(&mutex);

    /*
     * Starting DAC1 driver, setting up the output pin as analog as suggested
     * by the Reference Manual.
     */
    //palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);
    //palSetPadMode(GPIOA, 5, PAL_MODE_OUTPUT_PUSHPULL);
    //palSetPadMode(GPIOA, 5, PAL_MODE_INPUT);
    dacStart(&DACD2, &dac1cfg1);

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

  /* restore config and calibration data from flash memory */
  caldata_recall();

  set_sweep(freq_start, freq_stop);
  redraw();

  /*
   */
  set_frequencies();

  /*
   * I2S Initialize
   */
  tlv320aic3204_init();
  i2sInit();
  i2sObjectInit(&I2SD2);
  i2sStart(&I2SD2, &i2sconfig);
  i2sStartExchange(&I2SD2);

  /*
   * Shell manager initialization.
   */
    shellInit();

    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

    //set_frequency(10000000);

    while (1)
    {
    if (SDU1.config->usbp->state == USB_ACTIVE) {
      //palSetPad(GPIOC, GPIOC_LED);
      thread_t *shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                              "shell", NORMALPRIO + 1,
                                              shellThread, (void *)&shell_cfg1);
      chThdWait(shelltp);               /* Waiting termination.             */
      //palClearPad(GPIOC, GPIOC_LED);
    }
	chThdSleepMilliseconds(1000);
    }
}
