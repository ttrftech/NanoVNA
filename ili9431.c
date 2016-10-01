#include <math.h>
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

#define RGB565(r,g,b)     ( (((r)<<8)&0xf800) | (((g)<<3)&0x07e0) | (((b)>>3)&0x001f) )

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

void
spi_init(void)
{
  rccEnableSPI1(FALSE);

  //spiStart(&SPID1, &spicfg);       /* Setup transfer parameters.       */
  SPI1->CR1 = 0;
  SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BIDIOE | SPI_CR1_SSM | SPI_CR1_SSI;// | SPI_CR1_BR_1;
  SPI1->CR2 = 0x0700;
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


void
draw_grid(int n, int m, int w, int h, int x, int y, uint16_t fg, uint16_t bg)
{
  int ww = w*n+1;
  int hh = h*m+1;
  int xx = x;
  int yy = y;
  int i;

  ili9341_fill(x, y, ww, hh, bg);

  for (i = 0; i <= n; i++) {
    ili9341_fill(xx, y, 1, hh, fg);
    xx += w;
  }
  for (i = 0; i <= m; i++) {
    ili9341_fill(x, yy, ww, 1, fg);
    yy += h;
  }
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
  case 4:
    draw_grid(10, 8, 29, 29, 15, 0, 0xffff, 0);
    break;
  }
}



int prev_x;

int32_t fstart = 0;
int32_t fstop = 300000000;
int32_t fspan = 300000000;
int32_t fgrid = 50000000;

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
  int32_t n = FREQ(x-1) / fgrid;
  int32_t m = FREQ(x) / fgrid;
  if ((m - n) > 0)
  //if (((x * 6) % (WIDTH-1)) < 6)
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
    spi_buffer[v++] = color;
}

struct {
  float value;
  float prev_value;
  float d;
  uint16_t color;
} trace[2] = {
  { 0, 0, 0, RGB565(0,255,255) },
  { 0, 0, 0, RGB565(255,0,40) }
};

float logmag(float *v)
{
  return 11 - log10f(v[0]*v[0] + v[1]*v[1]);
}

void sweep_plot(int32_t freq, int first)
{
  int curr_x = ((float)WIDTH * (freq - fstart) / fspan);
  //float value = 11 - log10f(measured[0]*measured[0] + measured[1]*measured[1]);
  //value *= 29;
  trace[0].value = logmag(&measured[0]) * 29;
  trace[1].value = logmag(&measured[2]) * 29;

  if (first) {
    prev_x = 0;
    while (prev_x < curr_x) {
      int len = set_strut_grid(prev_x);
      ili9341_bulk(OFFSETX + prev_x, OFFSETY, 1, len);
      prev_x++;
    }
  } else {
    int w = curr_x - prev_x;
    trace[0].d = (trace[0].value - trace[0].prev_value) / w;
    trace[1].d = (trace[1].value - trace[1].prev_value) / w;

    while (prev_x < curr_x) {
      int len = set_strut_grid(prev_x);
      draw_on_strut(trace[0].prev_value, trace[0].d, trace[0].color);
      trace[0].prev_value += trace[0].d;
      draw_on_strut(trace[1].prev_value, trace[1].d, trace[1].color);
      trace[1].prev_value += trace[1].d;
      ili9341_bulk(OFFSETX + prev_x, OFFSETY, 1, len);
      prev_x++;
    }
  }
  trace[0].prev_value = trace[0].value;
  trace[1].prev_value = trace[1].value;
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
