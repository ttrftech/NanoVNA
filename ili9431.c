#include <math.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "nanovna.h"

#define RESET_ASSERT	palClearPad(GPIOA, 15)
#define RESET_NEGATE	palSetPad(GPIOA, 15)
#define CS_LOW			palClearPad(GPIOB, 6)
#define CS_HIGH			palSetPad(GPIOB, 6)
#define DC_CMD			palClearPad(GPIOB, 7)
#define DC_DATA			palSetPad(GPIOB, 7)

#define RGB565(b,r,g)     ( (((b)<<8)&0xfc00) | (((r)<<2)&0x03e0) | (((g)>>3)&0x001f) )

static inline void force_set_markmap(void);

uint16_t spi_buffer[1024];

void draw_frequencies(void);

static const SPIConfig spicfg = {
  NULL,
  GPIOB,
  6,
  0,
  SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0
};

void
ssp_wait(void)
{
  while (SPI1->SR & SPI_SR_BSY)
    ;
}

void
ssp_wait_slot(void)
{
  while ((SPI1->SR & 0x1800) == 0x1800)
    ;
}

void
ssp_senddata(uint8_t x)
{
  *(uint8_t*)(&SPI1->DR) = x;
  while (SPI1->SR & SPI_SR_BSY)
    ;
}

void
ssp_senddata16(uint16_t x)
{
  ssp_wait_slot();
  SPI1->DR = x;
  //while (SPI1->SR & SPI_SR_BSY)
  //  ;
}

void
ssp_databit8(void)
{
  SPI1->CR2 = (SPI1->CR2 & 0xf0ff) | 0x0700;
//LPC_SSP1->CR0 = (LPC_SSP1->CR0 & 0xf0) | SSP_DATABIT_8;
}

void
ssp_databit16(void)
{
  SPI1->CR2 = (SPI1->CR2 & 0xf0ff) | 0x0f00;
  //LPC_SSP1->CR0 = (LPC_SSP1->CR0 & 0xf0) | SSP_DATABIT_16;
}


const stm32_dma_stream_t  *dmatx;
uint32_t txdmamode;

static void spi_lld_serve_tx_interrupt(SPIDriver *spip, uint32_t flags) {
  (void)spip;
  (void)flags;
}

void
spi_init(void)
{
  rccEnableSPI1(FALSE);

  dmatx     = STM32_DMA_STREAM(STM32_SPI_SPI1_TX_DMA_STREAM);
  txdmamode = STM32_DMA_CR_CHSEL(SPI1_TX_DMA_CHANNEL) |
    STM32_DMA_CR_PL(STM32_SPI_SPI1_DMA_PRIORITY) |
    STM32_DMA_CR_DIR_M2P |
    STM32_DMA_CR_DMEIE |
    STM32_DMA_CR_TEIE |
    STM32_DMA_CR_PSIZE_HWORD |
    STM32_DMA_CR_MSIZE_HWORD;
  dmaStreamAllocate(dmatx,
                    STM32_SPI_SPI1_IRQ_PRIORITY,
                    (stm32_dmaisr_t)spi_lld_serve_tx_interrupt,
                    NULL);
  dmaStreamSetPeripheral(dmatx, &SPI1->DR);

  //spiStart(&SPID1, &spicfg);       /* Setup transfer parameters.       */
  SPI1->CR1 = 0;
  SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BIDIOE | SPI_CR1_SSM | SPI_CR1_SSI;// | SPI_CR1_BR_1;
  SPI1->CR2 = 0x0700 | SPI_CR2_TXDMAEN;
  SPI1->CR1 |= SPI_CR1_SPE;  
}

#if 0
void
spi_test(void)
{
    spiSelect(&SPID1);                  /* Slave Select assertion.          */
    spiSend(&SPID1, 512, txbuf);        /* Atomic transfer operations.      */
    spiUnselect(&SPID1);                /* Slave Select de-assertion.       */
}
#endif

void
send_command(uint8_t cmd, int len, const uint8_t *data)
{
	CS_LOW;
	DC_CMD;
    ssp_databit8();
	ssp_senddata(cmd);
	DC_DATA;
	while (len-- > 0) {
      ssp_senddata(*data++);
	}
	//CS_HIGH;
}

void
send_command16(uint8_t cmd, int data)
{
	CS_LOW;
	DC_CMD;
    ssp_databit8();
	ssp_senddata(cmd);
	DC_DATA;
    ssp_databit16();
	ssp_senddata16(data);
	CS_HIGH;
}

const uint8_t ili9341_init_seq[] = {
		// cmd, len, data...,
		// Power control B
		0xCF, 3, 0x00, 0x83, 0x30,
		// Power on sequence control
		0xED, 4, 0x64, 0x03, 0x12, 0x81,
		//0xED, 4, 0x55, 0x01, 0x23, 0x01,
		// Driver timing control A
		0xE8, 3, 0x85, 0x01, 0x79,
		//0xE8, 3, 0x84, 0x11, 0x7a,
		// Power control A
		0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
		// Pump ratio control
		0xF7, 1, 0x20,
		// Driver timing control B
		0xEA, 2, 0x00, 0x00,
		// POWER_CONTROL_1
		0xC0, 1, 0x26,
		// POWER_CONTROL_2
		0xC1, 1, 0x11,
		// VCOM_CONTROL_1
		0xC5, 2, 0x35, 0x3E,
		// VCOM_CONTROL_2
		0xC7, 1, 0xBE,
		// MEMORY_ACCESS_CONTROL
		//0x36, 1, 0x48, // portlait
		0x36, 1, 0x28, // landscape
		// COLMOD_PIXEL_FORMAT_SET : 16 bit pixel
		0x3A, 1, 0x55,
		// Frame Rate
		0xB1, 2, 0x00, 0x1B,
		// Gamma Function Disable
		0xF2, 1, 0x08,
		// gamma set for curve 01/2/04/08
		0x26, 1, 0x01,
		// positive gamma correction
		0xE0, 15, 0x1F,  0x1A,  0x18,  0x0A,  0x0F,  0x06,  0x45,  0x87,  0x32,  0x0A,  0x07,  0x02,  0x07, 0x05,  0x00,
		// negativ gamma correction
		0xE1, 15, 0x00,  0x25,  0x27,  0x05,  0x10,  0x09,  0x3A,  0x78,  0x4D,  0x05,  0x18,  0x0D,  0x38, 0x3A,  0x1F,

		// Column Address Set
	    0x2A, 4, 0x00, 0x00, 0x01, 0x3f, // width 320
	    // Page Address Set
	    0x2B, 4, 0x00, 0x00, 0x00, 0xef, // height 240

		// entry mode
		0xB7, 1, 0x06,
		// display function control
		0xB6, 4, 0x0A, 0x82, 0x27, 0x00,
		// sleep out
		0x11, 0,
		0 // sentinel
};

void
ili9341_init(void)
{
  //spiAcquireBus(&SPID1);              /* Acquire ownership of the bus.    */
  spi_init();

  DC_DATA;
  RESET_ASSERT;
  chThdSleepMilliseconds(10);
  RESET_NEGATE;

  send_command(0x01, 0, NULL); // SW reset
  chThdSleepMilliseconds(5);
  send_command(0x28, 0, NULL); // display off

  const uint8_t *p;
  for (p = ili9341_init_seq; *p; ) {
    send_command(p[0], p[1], &p[2]);
    p += 2 + p[1];
    chThdSleepMilliseconds(5);
  }

  chThdSleepMilliseconds(100);
  send_command(0x29, 0, NULL); // display on
  //spiReleaseBus(&SPID1);              /* Ownership release.               */

  force_set_markmap();
}

void ili9341_pixel(int x, int y, int color)
{
	uint8_t xx[4] = { x >> 8, x, (x+1) >> 8, (x+1) };
	uint8_t yy[4] = { y >> 8, y, (y+1) >> 8, (y+1) };
	uint8_t cc[2] = { color >> 8, color };
	send_command(0x2A, 4, xx);
    send_command(0x2B, 4, yy);
    send_command(0x2C, 2, cc);
    //send_command16(0x2C, color);
}



void ili9341_fill(int x, int y, int w, int h, int color)
{
	uint8_t xx[4] = { x >> 8, x, (x+w-1) >> 8, (x+w-1) };
	uint8_t yy[4] = { y >> 8, y, (y+h-1) >> 8, (y+h-1) };
    int len = w * h;
	send_command(0x2A, 4, xx);
    send_command(0x2B, 4, yy);
    send_command(0x2C, 0, NULL);
    while (len-- > 0) 
      ssp_senddata16(color);
}

#if 0
void ili9341_bulk(int x, int y, int w, int h)
{
	uint8_t xx[4] = { x >> 8, x, (x+w-1) >> 8, (x+w-1) };
	uint8_t yy[4] = { y >> 8, y, (y+h-1) >> 8, (y+h-1) };
	uint16_t *buf = spi_buffer;
    int len = w * h;
	send_command(0x2A, 4, xx);
	send_command(0x2B, 4, yy);
	send_command(0x2C, 0, NULL);
    while (len-- > 0) 
      ssp_senddata16(*buf++);
}
#else
void ili9341_bulk(int x, int y, int w, int h)
{
	uint8_t xx[4] = { x >> 8, x, (x+w-1) >> 8, (x+w-1) };
	uint8_t yy[4] = { y >> 8, y, (y+h-1) >> 8, (y+h-1) };
    int len = w * h;

	send_command(0x2A, 4, xx);
	send_command(0x2B, 4, yy);
	send_command(0x2C, 0, NULL);

    dmaStreamSetMemory0(dmatx, spi_buffer);
    dmaStreamSetTransactionSize(dmatx, len);
    dmaStreamSetMode(dmatx, txdmamode | STM32_DMA_CR_MINC);
    dmaStreamEnable(dmatx);
    dmaWaitCompletion(dmatx);
}
#endif

#define SWAP(x,y) do { int z=x; x = y; y = z; } while(0)

void
ili9341_line(int x0, int y0, int x1, int y1, uint16_t fg)
{
  if (x0 > x1) {
    SWAP(x0, x1);
    SWAP(y0, y1);
  }

  while (x0 <= x1) {
    int dx = x1 - x0 + 1;
    int dy = y1 - y0;
    if (dy >= 0) {
      dy++;
      if (dy > dx) {
        dy /= dx; dx = 1;
      } else {
        dx /= dy; dy = 1;
      }
    } else {
      dy--;
      if (-dy > dx) {
        dy /= dx; dx = 1;
      } else {
        dx /= -dy; dy = -1;
      }
    }
    if (dy > 0)
      ili9341_fill(x0, y0, dx, dy, fg);
    else
      ili9341_fill(x0, y0+dy, dx, -dy, fg);
    x0 += dx;
    y0 += dy;
  }
}


typedef struct {
	uint16_t width;
	uint16_t height;
	uint16_t scaley;
	uint16_t slide;
	const uint32_t *bitmap;
} font_t;

const font_t NF20x24 = { 20, 24, 1, 24, (const uint32_t *)numfont20x24 };
//const font_t NF32x24 = { 32, 24, 1, 24, (const uint32_t *)numfont32x24 };
//const font_t NF32x48 = { 32, 48, 2, 24, (const uint32_t *)numfont32x24 };

void
ili9341_drawfont(uint8_t ch, const font_t *font, int x, int y, uint16_t fg, uint16_t bg)
{
	int ex = x + font->width-1;
	int ey = y + font->height-1;
	uint8_t xx[4] = { x >> 8, x, ex >> 8, ex };
	uint8_t yy[4] = { y >> 8, y, ey >> 8, ey };
	uint16_t *buf = spi_buffer;
	uint32_t bits;
	const uint32_t *bitmap = &font->bitmap[font->slide * ch];
    int len;
	int c, r, j;

	for (c = 0; c < font->slide; c++) {
		for (j = 0; j < font->scaley; j++) {
			bits = bitmap[c];
			for (r = 0; r < font->width; r++) {
				*buf++ = (0x80000000UL & bits) ? fg : bg;
				bits <<= 1;
			}
		}
	}
	send_command(0x2A, 4, xx);
	send_command(0x2B, 4, yy);
	send_command(0x2C, 0, NULL);
    len = buf - spi_buffer;
    buf = spi_buffer;
    while (len-- > 0) 
      ssp_senddata16(*buf++);
}


const uint16_t colormap[] = {
  RGB565(255,0,0), RGB565(0,255,0), RGB565(0,0,255),
  RGB565(255,255,0), RGB565(0,255,255), RGB565(255,0,255)
};

void
ili9341_test(int mode)
{
  int x, y;
  int i;
  switch (mode) {
  default:
#if 1
    ili9341_fill(0, 0, 320, 240, 0);
    for (y = 0; y < 240; y++) {
      ili9341_fill(0, y, 320, 1, RGB565(y, (y + 120) % 256, 240-y));
    }
    break;
  case 1:
    ili9341_fill(0, 0, 320, 240, 0);
    for (y = 0; y < 240; y++) {
      for (x = 0; x < 320; x++) {
        ili9341_pixel(x, y, (y<<8)|x);
      }
    }
    break;
  case 2:
    //send_command16(0x55, 0xff00);
    ili9341_pixel(64, 64, 0xaa55);
    break;
#endif
#if 1
  case 3:
	for (i = 0; i < 10; i++)
      ili9341_drawfont(i, &NF20x24, i*20, 120, colormap[i%6], 0x0000);
    break;
#endif
#if 0
  case 4:
    draw_grid(10, 8, 29, 29, 15, 0, 0xffff, 0);
    break;
#endif
  case 4:
    ili9341_line(0, 0, 15, 100, 0xffff);
    ili9341_line(0, 0, 100, 100, 0xffff);
    ili9341_line(0, 15, 100, 0, 0xffff);
    ili9341_line(0, 100, 100, 0, 0xffff);
    break;
  }
}



int prev_x;

int32_t fstart = 0;
int32_t fstop = 300000000;
int32_t fspan = 300000000;
int32_t fgrid = 50000000;
int grid_offset;
int grid_width;

#define OFFSETX 15
#define OFFSETY 0
#define WIDTH 291
#define HEIGHT 233

//#define GRID_COLOR 0x0863
uint16_t grid_color = 0x1084;

void set_sweep(int32_t start, int stop)
{
  int32_t gdigit = 100000000;
  int32_t grid;
  fstart = start;
  fstop = stop;
  fspan = stop - start;

  while (gdigit > 100) {
    grid = 5 * gdigit;
    if (fspan / grid >= 4)
      break;
    grid = 2 * gdigit;
    if (fspan / grid >= 4)
      break;
    grid = gdigit;
    if (fspan / grid >= 4)
      break;
    gdigit /= 10;
  }
  fgrid = grid;

  grid_offset = (WIDTH-1) * ((fstart % fgrid) / 100) / (fspan / 100);
  grid_width = (WIDTH-1) * (fgrid / 100) / (fspan / 1000);

  force_set_markmap();
  draw_frequencies();
}

int
circle_inout(int x, int y, int r)
{
  int d = x*x + y*y - r*r;
  if (d <= -r)
    return 1;
  if (d > r)
    return -1;
  return 0;
}

int
smith_grid(int x, int y)
{
  int d = circle_inout(x-146, y-116, 116);
  int c = grid_color;
  if (d < 0)
    return 0;
  else if (d == 0)
    return c;
  x -= 146+116;
  y -= 116;
  
  if (circle_inout(x, y+58, 58) == 0)
    return c;
  if (circle_inout(x, y-58, 58) == 0)
    return c;
  d = circle_inout(x+29, y, 29);
  if (d > 0) return 0;
  if (d == 0) return c;
  if (circle_inout(x, y+116, 116) == 0)
    return c;
  if (circle_inout(x, y-116, 116) == 0)
    return c;
  d = circle_inout(x+58, y, 58);
  if (d > 0) return 0;
  if (d == 0) return c;
  if (circle_inout(x, y+232, 232) == 0)
    return c;
  if (circle_inout(x, y-232, 232) == 0)
    return c;
  if (circle_inout(x+87, y, 87) == 0)
    return c;
  return 0;
}

int
rectangular_grid(int x, int y)
{
#define FREQ(x) (((x) * (fspan / 1000) / (WIDTH-1)) * 1000 + fstart)
  int c = grid_color;
  //int32_t n = FREQ(x-1) / fgrid;
  //int32_t m = FREQ(x) / fgrid;
  //if ((m - n) > 0)
  //if (((x * 6) % (WIDTH-1)) < 6)
  //if (((x - grid_offset) % grid_width) == 0)
  if ((((x + grid_offset) * 10) % grid_width) < 10)
    return c;
  if (x == 0 || x == (WIDTH-1))
    return c;
  if ((y % 29) == 0)
    return c;
  return 0;
}

int
set_strut_grid(int x)
{
  uint16_t *buf = spi_buffer;
  int y;

  for (y = 0; y < HEIGHT; y++) {
    int c = rectangular_grid(x, y);
    c |= smith_grid(x, y);
    *buf++ = c;
  }
  return y;
}

void
draw_on_strut(int v0, int d, int color)
{
  int v;
  int v1 = v0 + d;
  if (v0 < 0) v0 = 0;
  if (v1 < 0) v1 = 0;
  if (v0 >= HEIGHT) v0 = HEIGHT-1;
  if (v1 >= HEIGHT) v1 = HEIGHT-1;
  if (v0 == v1) {
    v = v0; d = 2;
  } else if (v0 < v1) {
    v = v0; d = v1 - v0 + 1;
  } else {
    v = v1; d = v0 - v1 + 1;
  }
  while (d-- > 0)
    spi_buffer[v++] |= color;
}

#define TRACES_MAX 4

struct {
  int enabled;
  float value;
  float prev_value;
  float d;
  uint16_t color;
  uint8_t polar;
} trace[TRACES_MAX] = {
  { 1, 0, 0, 0, RGB565(0,255,255), 0 },
  { 1, 0, 0, 0, RGB565(255,0,40), 0 },
  { 1, 0, 0, 0, RGB565(0,0,255), 1 },
  { 0, 0, 0, 0, RGB565(0,255,0), 1 }
};

uint32_t trace_index[TRACES_MAX][101];

float logmag(float *v)
{
  return 11 - log10f(v[0]*v[0] + v[1]*v[1]);
}

float phase(float *v)
{
  return 4 + 2 * atan2f(v[1], v[0]) / M_PI;
}

void sweep_plot(int32_t freq, int first, float *measured)
{
  int curr_x = ((float)WIDTH * (freq - fstart) / fspan);
  int i;
  if (trace[0].enabled)
    trace[0].value = logmag(&measured[0]) * 29;
  if (trace[1].enabled)
    trace[1].value = logmag(&measured[2]) * 29;
  if (trace[2].enabled)
    trace[2].value = phase(&measured[0]) * 29;
  if (trace[3].enabled)
    trace[3].value = phase(&measured[2]) * 29;

  if (first) {
    prev_x = 0;
    while (prev_x < curr_x) {
      int len = set_strut_grid(prev_x);
      ili9341_bulk(OFFSETX + prev_x, OFFSETY, 1, len);
      prev_x++;
    }
  } else {
    int w = curr_x - prev_x;
    for (i = 0; i < TRACES_MAX; i++)
      if (trace[i].enabled)
        trace[i].d = (trace[i].value - trace[i].prev_value) / w;

    while (prev_x < curr_x) {
      int len = set_strut_grid(prev_x);
      for (i = 0; i < TRACES_MAX; i++)
        if (trace[i].enabled) {
          draw_on_strut(trace[i].prev_value, trace[i].d, trace[i].color);
          trace[i].prev_value += trace[i].d;
        }
      ili9341_bulk(OFFSETX + prev_x, OFFSETY, 1, len);
      prev_x++;
    }
  }
  for (i = 0; i < TRACES_MAX; i++)
    if (trace[i].enabled)
      trace[i].prev_value = trace[i].value;
}

void sweep_tail()
{
  while (prev_x < WIDTH) {
    int len = set_strut_grid(prev_x);
    ili9341_bulk(OFFSETX + prev_x, OFFSETY, 1, len);
    prev_x++;
  }
}

void
cartesian_scale(float re, float im, int *xp, int *yp)
{
  float scale = 4e-3;
  int x = WIDTH / 2 - re * scale;
  int y = HEIGHT / 2 + im * scale;
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  if (x > WIDTH) x = WIDTH;
  if (y > HEIGHT) y = HEIGHT;
  *xp = x;
  *yp = y;
}

void polar_plot(float measured[101][4])
{
  int x0, y0;
  int i;
  cartesian_scale(measured[0][1], measured[0][0], &x0, &y0);
  for (i = 1; i < 101; i++) {
    int x1, y1;
    cartesian_scale(measured[i][1], measured[i][0], &x1, &y1);
    ili9341_line(x0, y0, x1, y1, trace[2].color);
    x0 = x1;
    y0 = y1;
  }
}


#define INDEX(x, y, n) \
  ((((x)&0x03e0UL)<<22) | (((y)&0x03e0UL)<<17) | (((n)&0x0fffUL)<<10)  \
 | (((x)&0x1fUL)<<5) | ((y)&0x1fUL))
#define CELL_X(i) (int)((((i)>>5)&0x1f) | (((i)>>22)&0x03e0))
#define CELL_Y(i) (int)(((i)&0x1f) | (((i)>>17)&0x03e0))
#define CELL_N(i) (int)(((i)>>10)&0xfff)

#define CELL_X0(i) (int)(((i)>>22)&0x03e0)
#define CELL_Y0(i) (int)(((i)>>17)&0x03e0)

#define CELL_P(i, x, y) (((((x)&0x03e0UL)<<22) | (((y)&0x03e0UL)<<17)) == ((i)&0xffc00000UL))

#define CELLWIDTH 32
#define CELLHEIGHT 32


inline void swap(uint32_t *a, uint32_t *b)
{
  uint32_t t=*a; *a=*b; *b=t;
}

void insertionsort(uint32_t *arr, int start, int end)
{
  int i;
  for (i = start + 1; i < end; i++) {
    uint32_t val = arr[i];
    int j = i - 1;
    while (j >= start && val > arr[j]) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = val;
  }
}

void quicksort(uint32_t *arr, int beg, int end)
{
  if (end - beg <= 1) 
    return;
  else if (end - beg < 10) {
    insertionsort(arr, beg, end);
  } else {
    int l = beg;
    int r = end-1;
    uint32_t piv = arr[(beg + end) / 2];
    while (l < r) {
      while (arr[l] < piv)
        l++;
      while (arr[r] > piv)
        r--;
      if (l < r)
        swap(&arr[l], &arr[r]);
    }

    quicksort(arr, beg, l);
    quicksort(arr, r, end);
  }
}

#if 0
uint32_t polar_index[101];

void polar_plot2index(float measured[101][4])
{
  int i;
  for (i = 0; i < 101; i++) {
    int x1, y1;
    cartesian_scale(measured[i][1], measured[i][0], &x1, &y1);
    polar_index[i] = INDEX(x1, y1, i);
  }
  quicksort(polar_index, 0, 101);
}
#endif


uint16_t markmap[2][8];
uint16_t current_mappage = 0;

static inline void
mark_map(int x, int y)
{
  if (y >= 0 && y < 8 && x >= 0 && x < 16)
    markmap[current_mappage][y] |= 1<<x;
}

static inline int
is_mapmarked(int x, int y)
{
  uint16_t bit = 1<<x;
  return (markmap[0][y] & bit) || (markmap[1][y] & bit);
}

static void
swap_markmap(void)
{
  current_mappage = 1 - current_mappage;
}

static inline void
clear_markmap(void)
{
  memset(markmap[current_mappage], 0, sizeof markmap[current_mappage]);
}

static inline void
force_set_markmap(void)
{
  memset(markmap[current_mappage], 0xff, sizeof markmap[current_mappage]);
}

void plot_into_index(float measured[101][2][2])
{
  int i, t;
  for (i = 0; i < 101; i++) {
    int x = i * (WIDTH-1) / (101-1);
    for (t = 0; t < TRACES_MAX; t++) {
      int n = t % 2;
      if (!trace[t].enabled)
        continue;
      if (trace[t].polar) {
        int x1, y1;
        cartesian_scale(measured[i][n][1], measured[i][n][0], &x1, &y1);
        trace_index[t][i] = INDEX(x1, y1, i);
        //mark_map(x1>>5, y1>>5);
      } else {
        int y1 = logmag(measured[i][n]) * 29;
        trace_index[t][i] = INDEX(x, y1, i);
        //mark_map(x>>5, y1>>5);
      }
    }
  }
#if 0
  for (t = 0; t < TRACES_MAX; t++)
    if (trace[t].enabled && trace[t].polar)
      quicksort(trace_index[t], 0, 101);
#endif

  /* mark cells between each neighber points */
  for (t = 0; t < TRACES_MAX; t++) {
    int x0 = CELL_X(trace_index[t][0]);
    int y0 = CELL_Y(trace_index[t][0]);
    int m0 = x0 >> 5;
    int n0 = y0 >> 5;
    mark_map(m0, n0);
    for (i = 1; i < 101; i++) {
      int x1 = CELL_X(trace_index[t][i]);
      int y1 = CELL_Y(trace_index[t][i]);
      int m1 = x1 >> 5;
      int n1 = y1 >> 5;
      while (m0 != m1 || n0 != n1) {
        if (m0 == m1) {
          if (n0 < n1) n0++; else n0--;
        } else if (n0 == n1) {
          if (m0 < m1) m0++; else m0--;
        } else {
          int x = (m0 < m1) ? (m0 + 1)<<5 : m0<<5;
          int y = (n0 < n1) ? (n0 + 1)<<5 : n0<<5;
          int sgn = (n0 < n1) ? 1 : -1;
          if (sgn*(y-y0)*(x1-x0) < sgn*(x-x0)*(y1-y0)) {
            if (m0 < m1) m0++;
            else m0--;
          } else {
            if (n0 < n1) n0++;
            else n0--;
          }
        }
        mark_map(m0, n0);
      }
      x0 = x1;
      y0 = y1;
      m0 = m1;
      n0 = n1;
    }
  }
}

void
line_in_cell(int w, int h, int x0, int y0, int x1, int y1, int c)
{
  if (x0 > x1) {
    SWAP(x0, x1);
    SWAP(y0, y1);
  }

  while (x0 <= x1) {
    int dx = x1 - x0 + 1;
    int dy = y1 - y0;
    if (dy >= 0) {
      dy++;
      if (dy > dx) {
        dy /= dx; dx = 1;
      } else {
        dx /= dy; dy = 1;
      }
    } else {
      dy--;
      if (-dy > dx) {
        dy /= dx; dx = 1;
      } else {
        dx /= -dy; dy = -1;
      }
    }

    if (dx == 1) {
      if (dy > 0) {
        while (dy-- > 0) {
          if (y0 >= 0 && y0 < h && x0 >= 0 && x0 < w)
            spi_buffer[y0*w+x0] = c;
          y0++;
        }
      } else {
        while (dy++ < 0) {
          if (y0 >= 0 && y0 < h && x0 >= 0 && x0 < w)
            spi_buffer[y0*w+x0] = c;
          y0--;
        }
      }
      x0++;
    } else {
      while (dx-- > 0) {
        if (y0 >= 0 && y0 < h && x0 >= 0 && x0 < w)
          spi_buffer[y0*w+x0] = c;
        x0++;
      }
      y0 += dy;
    }
  }
}

int
search_index(int x, int y, uint32_t index[101], int *i0, int *i1)
{
  int i, j;
  int head = 0;
  int tail = 101;
  x &= 0x03e0;
  y &= 0x03e0;
  while (head < tail) {
    i = (head + tail) / 2;
    if (x < CELL_X0(index[i]))
      tail = i+1;
    else if (x > CELL_X0(index[i]))
      head = i;
    else if (y < CELL_Y0(index[i]))
      tail = i+1;
    else if (y > CELL_Y0(index[i]))
      head = i;
    else
      break;
  }

  if (x != CELL_X0(index[i]) || y != CELL_Y0(index[i]))
    return FALSE;
    
  j = i;
  while (j > 0 && x == CELL_X0(index[j-1]) && y == CELL_Y0(index[j-1]))
    j--;
  *i0 = j;
  j = i;
  while (j < 100 && x == CELL_X0(index[j+1]) && y == CELL_Y0(index[j+1]))
    j++;
  *i1 = j;
  return TRUE;
}

int
search_index_x(int x, uint32_t index[101], int *i0, int *i1)
{
  int i, j;
  int head = 0;
  int tail = 101;
  x &= 0x03e0;
  while (head < tail) {
    i = (head + tail) / 2;
    if (x < CELL_X0(index[i]))
      tail = i+1;
    else if (x > CELL_X0(index[i]))
      head = i;
    else
      break;
  }

  if (x != CELL_X0(index[i]))
    return FALSE;

  j = i;
  while (j > 0 && x == CELL_X0(index[j-1]))
    j--;
  *i0 = j;
  j = i;
  while (j < 100 && x == CELL_X0(index[j+1]))
    j++;
  *i1 = j;
  return TRUE;
}

void
draw_marker(int w, int h, int x, int y, int c, int ch)
{
  int i, j;
  for (j = 10; j >= 0; j--) {
    int j0 = j / 2;
    for (i = -j0; i <= j0; i++) {
      int x0 = x + i;
      int y0 = y - j;
      int cc = c;
      if (j <= 9 && j > 2 && i >= -1 && i <= 3) {
        uint16_t bits = x5x7_bits[(ch * 7) + (9-j)];
        if (bits & (0x8000>>(i+1)))
          cc = 0;
      }
      if (y0 >= 0 && y0 < h && x0 >= 0 && x0 < w)
        spi_buffer[y0*w+x0] = cc;
    }
  }
}

void
draw_cell(int m, int n)
{
  int x0 = m * CELLWIDTH;
  int y0 = n * CELLHEIGHT;
  int w = CELLWIDTH;
  int h = CELLHEIGHT;
  int x, y;
  int i0, i1;
  int i;
  int t;
  if (x0 + w > WIDTH)
    w = WIDTH - x0;
  if (y0 + h > HEIGHT)
    h = HEIGHT - y0;

  /* draw grid */
  for (y = 0; y < h; y++) {
    for (x = 0; x < w; x++) {
      uint16_t c = rectangular_grid(x+x0, y+y0);
      c |= smith_grid(x+x0, y+y0);
      spi_buffer[y * w + x] = c;
    }
  }

#if 1
  /* draw rectanglar plot */
  if (search_index_x(x0, trace_index[0], &i0, &i1)) {
    for (t = 0; t < TRACES_MAX; t++) {
      if (!trace[t].enabled || trace[t].polar)
        continue;
      if (i0 > 0)
        i0--;
      for (i = i0; i < i1; i++) {
        int x1 = CELL_X(trace_index[t][i]);
        int x2 = CELL_X(trace_index[t][i+1]);
        int y1 = CELL_Y(trace_index[t][i]);
        int y2 = CELL_Y(trace_index[t][i+1]);
        int c = trace[t].color;
        line_in_cell(w, h, x1 - x0, y1 - y0, x2 - x0, y2 - y0, c);
      }
    }
  }
#endif
#if 1
  /* draw polar plot */
  for (t = 0; t < TRACES_MAX; t++) {
    int c = trace[t].color;
    if (!trace[t].enabled || !trace[t].polar)
      continue;
    for (i = 1; i < 101; i++) {
      //uint32_t index = trace_index[t][i];
      //uint32_t pindex = trace_index[t][i-1];
      //if (!CELL_P(index, x0, y0) && !CELL_P(pindex, x0, y0))
      //  continue;
      int x1 = CELL_X(trace_index[t][i-1]);
      int x2 = CELL_X(trace_index[t][i]);
      int y1 = CELL_Y(trace_index[t][i-1]);
      int y2 = CELL_Y(trace_index[t][i]);
      line_in_cell(w, h, x1 - x0, y1 - y0, x2 - x0, y2 - y0, c);
    }
  }
#endif
#if 0
  /* draw polar plot */
  for (t = 0; t < TRACES_MAX; t++) {
    int prev = -100;
    int c = trace[t].color;
    if (!trace[t].enabled || !trace[t].polar)
      continue;
    if (search_index(x0, y0, trace_index[t], &i0, &i1)) {
      for (i = i0; i < i1; i++) {
        uint32_t index = trace_index[t][i];
        uint32_t pindex;
        int n = i;
        if (!CELL_P(index, x0, y0))
          continue;
        n = CELL_N(index);
        if (n - prev == 1) {
          pindex = trace_index[t][prev];
          line_in_cell(w, h, CELL_X(pindex) - x0, CELL_Y(pindex) - y0, CELL_X(index) - x0, CELL_Y(index) - y0, c);
          }
          prev = n;
      }
    }
  }
#endif
  if (m == 0 && n == 0) {
    draw_marker(w, h, 8, 12, trace[0].color, '1');
    draw_marker(w, h, 18, 20, trace[1].color, '2');
    draw_marker(w, h, 4, 30, trace[2].color, '3');
  }

  ili9341_bulk(OFFSETX + x0, OFFSETY + y0, w, h);
}

void
draw_cell_all(void)
{
  int m, n;
  for (m = 0; m < (WIDTH+CELLWIDTH-1) / CELLWIDTH; m++)
    for (n = 0; n < (HEIGHT+CELLHEIGHT-1) / CELLHEIGHT; n++)
      if (is_mapmarked(m, n))
        draw_cell(m, n);
  swap_markmap();
  clear_markmap();
}

void
ili9341_drawchar_5x7(uint8_t ch, int x, int y, uint16_t fg, uint16_t bg)
{
  uint16_t *buf = spi_buffer;
  uint16_t bits;
  int c, r;
  for(c = 0; c < 7; c++) {
    bits = x5x7_bits[(ch * 7) + c];
    for (r = 0; r < 5; r++) {
      *buf++ = (0x8000 & bits) ? fg : bg;
      bits <<= 1;
    }
  }
  ili9341_bulk(x, y, 5, 7);
}

void
ili9341_drawstring_5x7(char *str, int x, int y, uint16_t fg, uint16_t bg)
{
  while (*str) {
    ili9341_drawchar_5x7(*str, x, y, fg, bg);
    x += 5;
    str++;
  }
}


void
draw_frequencies(void)
{
  char buf[24];
  chsnprintf(buf, 24, "START%3d.%03d %03d MHz",
             (int)(fstart / 1000000),
             (int)((fstart / 1000) % 1000),
             (int)(fstart % 1000));
  ili9341_drawstring_5x7(buf, OFFSETX, 233, 0xffff, 0x0000);
  chsnprintf(buf, 24, "STOP %3d.%03d %03d MHz",
             (int)(fstop / 1000000),
             (int)((fstop / 1000) % 1000),
             (int)(fstop % 1000));
  ili9341_drawstring_5x7(buf, 205, 233, 0xffff, 0x0000);
}

void
redraw(void)
{
  ili9341_fill(0, 0, 320, 240, 0);
  draw_frequencies();
}
