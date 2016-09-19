#include "ch.h"
#include "hal.h"
#include "usbcfg.h"
#include "si5351.h"
#include "nanovna.h"

#include <chprintf.h>
#include <shell.h>
#include <stdlib.h>
#include <string.h>

RTCDateTime timespec;


static const I2CConfig i2ccfg = {
  0x00902025, //voodoo magic
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

static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg)
{
    (void)arg;

    chRegSetThreadName("blink");

    palSetPadMode(GPIOC, 13, PAL_MODE_OUTPUT_PUSHPULL);
    while (1)
    {
      systime_t time = 500;
      if (serusbcfg.usbp->state != USB_ACTIVE)
        palClearPad(GPIOC, 13);
      chThdSleepMilliseconds(time);
      palSetPad(GPIOC, 13);
      chThdSleepMilliseconds(time);
    }
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

void set_frequency(int freq)
{
    frequency = freq;
#if 0
    si5351_set_frequency(0, freq + frequency_offset);
    si5351_set_frequency(1, freq);
#else
    si5351_set_frequency_with_offset(freq, frequency_offset, drive_strength);
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
#if 1
    dacPutChannelX(&DACD2, 0, value);
#else
    if (value & 1)
      palSetPad(GPIOA, 5);
    else
      palClearPad(GPIOA, 5);
#endif
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
int32_t gamma_real;
int32_t gamma_imag;



void i2s_end_callback(I2SDriver *i2sp, size_t offset, size_t n)
{
#if PORT_SUPPORTS_RT
  int32_t cnt_s = port_rt_get_counter_value();
  int32_t cnt_e;
#endif
  int16_t *p = &rx_buffer[offset];
  (void)i2sp;
  (void)n;
  palClearPad(GPIOC, GPIOC_LED);

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

#if 0
  if (request_calcgamma > 0) {
    if (request_calcgamma == 1)
      calclate_gamma();
    --request_calcgamma;
  }
#endif

#if PORT_SUPPORTS_RT
  cnt_e = port_rt_get_counter_value();
  stat.interval_cycles = cnt_s - stat.last_counter_value;
  stat.busy_cycles = cnt_e - cnt_s;
  stat.last_counter_value = cnt_s;
#endif
  stat.callback_count++;
  palSetPad(GPIOC, GPIOC_LED);
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
  int i, j;
  int len;

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
  (void)argc;
  (void)argv;

  wait_count = 3;
  while (wait_count)
    ;
  dsp_disabled = TRUE;
  calclate_gamma();
  dsp_disabled = FALSE;

  chprintf(chp, "%d %d\r\n", gamma_real, gamma_imag);
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
  stat.rms[0] = sqrt(acc0 / count);
  stat.rms[1] = sqrt(acc1 / count);
  stat.ave[0] = ave0;
  stat.ave[1] = ave1;

  chprintf(chp, "average: %d %d\r\n", stat.ave[0], stat.ave[1]);
  chprintf(chp, "rms: %d %d\r\n", stat.rms[0], stat.rms[1]);
  chprintf(chp, "callback count: %d\r\n", stat.callback_count);
  chprintf(chp, "interval cycle: %d\r\n", stat.interval_cycles);
  chprintf(chp, "busy cycle: %d\r\n", stat.busy_cycles);
  chprintf(chp, "load: %d\r\n", stat.busy_cycles * 100 / stat.interval_cycles);
}





#define SHELL_WA_SIZE THD_WORKING_AREA_SIZE(2048)

static const ShellCommand commands[] =
{
    { "reset", cmd_reset },
    { "freq", cmd_freq },
    { "offset", cmd_offset },
    { "time", cmd_time },
    { "dac", cmd_dac },
    { "data", cmd_data },
    { "port", cmd_port },
    { "stat", cmd_stat },
    { "gain", cmd_gain },
    { "power", cmd_power },
    { "gamma", cmd_gamma },
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

    set_frequency(10000000);

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
