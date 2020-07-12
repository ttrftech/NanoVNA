/*
 * Copyright (c) 2014-2015, TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
 * updated by DiSlord dislordlive@gmail.com
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
#include "hal.h"
#include "nanovna.h"

#include "spi.h"
// Allow enable DMA for read display data
//#define __USE_DISPLAY_DMA_RX__

// Pin macros for LCD
#define LCD_CS_LOW        palClearPad(GPIOB, GPIOB_LCD_CS)
#define LCD_CS_HIGH       palSetPad(GPIOB, GPIOB_LCD_CS)
#define LCD_RESET_ASSERT  palClearPad(GPIOA, GPIOA_LCD_RESET)
#define LCD_RESET_NEGATE  palSetPad(GPIOA, GPIOA_LCD_RESET)
#define LCD_DC_CMD        palClearPad(GPIOB, GPIOB_LCD_CD)
#define LCD_DC_DATA       palSetPad(GPIOB, GPIOB_LCD_CD)

#define LCD_SPI           SPI1
// Set SPI bus speed for LCD
#define LCD_SPI_SPEED    SPI_BR_DIV2

uint16_t spi_buffer[SPI_BUFFER_SIZE];
// Default foreground & background colors
uint16_t foreground_color = 0;
uint16_t background_color = 0;

// Display width and height definition
#define ILI9341_WIDTH     320
#define ILI9341_HEIGHT    240

// Display commands list
#define ILI9341_NOP                        0x00
#define ILI9341_SOFTWARE_RESET             0x01
#define ILI9341_READ_IDENTIFICATION        0x04
#define ILI9341_READ_STATUS                0x09
#define ILI9341_READ_POWER_MODE            0x0A
#define ILI9341_READ_MADCTL                0x0B
#define ILI9341_READ_PIXEL_FORMAT          0x0C
#define ILI9341_READ_IMAGE_FORMAT          0x0D
#define ILI9341_READ_SIGNAL_MODE           0x0E
#define ILI9341_READ_SELF_DIAGNOSTIC       0x0F
#define ILI9341_SLEEP_IN                   0x10
#define ILI9341_SLEEP_OUT                  0x11
#define ILI9341_PARTIAL_MODE_ON            0x12
#define ILI9341_NORMAL_DISPLAY_MODE_ON     0x13
#define ILI9341_INVERSION_OFF              0x20
#define ILI9341_INVERSION_ON               0x21
#define ILI9341_GAMMA_SET                  0x26
#define ILI9341_DISPLAY_OFF                0x28
#define ILI9341_DISPLAY_ON                 0x29
#define ILI9341_COLUMN_ADDRESS_SET         0x2A
#define ILI9341_PAGE_ADDRESS_SET           0x2B
#define ILI9341_MEMORY_WRITE               0x2C
#define ILI9341_COLOR_SET                  0x2D
#define ILI9341_MEMORY_READ                0x2E
#define ILI9341_PARTIAL_AREA               0x30
#define ILI9341_VERTICAL_SCROLLING_DEF     0x33
#define ILI9341_TEARING_LINE_OFF           0x34
#define ILI9341_TEARING_LINE_ON            0x35
#define ILI9341_MEMORY_ACCESS_CONTROL      0x36
#define ILI9341_VERTICAL_SCROLLING         0x37
#define ILI9341_IDLE_MODE_OFF              0x38
#define ILI9341_IDLE_MODE_ON               0x39
#define ILI9341_PIXEL_FORMAT_SET           0x3A
#define ILI9341_WRITE_MEMORY_CONTINUE      0x3C
#define ILI9341_READ_MEMORY_CONTINUE       0x3E
#define ILI9341_SET_TEAR_SCANLINE          0x44
#define ILI9341_GET_SCANLINE               0x45
#define ILI9341_WRITE_BRIGHTNESS           0x51
#define ILI9341_READ_BRIGHTNESS            0x52
#define ILI9341_WRITE_CTRL_DISPLAY         0x53
#define ILI9341_READ_CTRL_DISPLAY          0x54
#define ILI9341_WRITE_CA_BRIGHTNESS        0x55
#define ILI9341_READ_CA_BRIGHTNESS         0x56
#define ILI9341_WRITE_CA_MIN_BRIGHTNESS    0x5E
#define ILI9341_READ_CA_MIN_BRIGHTNESS     0x5F
#define ILI9341_READ_ID1                   0xDA
#define ILI9341_READ_ID2                   0xDB
#define ILI9341_READ_ID3                   0xDC
#define ILI9341_RGB_INTERFACE_CONTROL      0xB0
#define ILI9341_FRAME_RATE_CONTROL_1       0xB1
#define ILI9341_FRAME_RATE_CONTROL_2       0xB2
#define ILI9341_FRAME_RATE_CONTROL_3       0xB3
#define ILI9341_DISPLAY_INVERSION_CONTROL  0xB4
#define ILI9341_BLANKING_PORCH_CONTROL     0xB5
#define ILI9341_DISPLAY_FUNCTION_CONTROL   0xB6
#define ILI9341_ENTRY_MODE_SET             0xB7
#define ILI9341_BACKLIGHT_CONTROL_1        0xB8
#define ILI9341_BACKLIGHT_CONTROL_2        0xB9
#define ILI9341_BACKLIGHT_CONTROL_3        0xBA
#define ILI9341_BACKLIGHT_CONTROL_4        0xBB
#define ILI9341_BACKLIGHT_CONTROL_5        0xBC
#define ILI9341_BACKLIGHT_CONTROL_7        0xBE
#define ILI9341_BACKLIGHT_CONTROL_8        0xBF
#define ILI9341_POWER_CONTROL_1            0xC0
#define ILI9341_POWER_CONTROL_2            0xC1
#define ILI9341_VCOM_CONTROL_1             0xC5
#define ILI9341_VCOM_CONTROL_2             0xC7
#define ILI9341_POWERA                     0xCB
#define ILI9341_POWERB                     0xCF
#define ILI9341_NV_MEMORY_WRITE            0xD0
#define ILI9341_NV_PROTECTION_KEY          0xD1
#define ILI9341_NV_STATUS_READ             0xD2
#define ILI9341_READ_ID4                   0xD3
#define ILI9341_POSITIVE_GAMMA_CORRECTION  0xE0
#define ILI9341_NEGATIVE_GAMMA_CORRECTION  0xE1
#define ILI9341_DIGITAL_GAMMA_CONTROL_1    0xE2
#define ILI9341_DIGITAL_GAMMA_CONTROL_2    0xE3
#define ILI9341_DTCA                       0xE8
#define ILI9341_DTCB                       0xEA
#define ILI9341_POWER_SEQ                  0xED
#define ILI9341_3GAMMA_EN                  0xF2
#define ILI9341_INTERFACE_CONTROL          0xF6
#define ILI9341_PUMP_RATIO_CONTROL         0xF7

//
// ILI9341_MEMORY_ACCESS_CONTROL registers
//
#define ILI9341_MADCTL_MY  0x80
#define ILI9341_MADCTL_MX  0x40
#define ILI9341_MADCTL_MV  0x20
#define ILI9341_MADCTL_ML  0x10
#define ILI9341_MADCTL_BGR 0x08
#define ILI9341_MADCTL_MH  0x04
#define ILI9341_MADCTL_RGB 0x00

#define DISPLAY_ROTATION_270   (ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR)
#define DISPLAY_ROTATION_90    (ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR)
#define DISPLAY_ROTATION_0     (ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR)
#define DISPLAY_ROTATION_180   (ILI9341_MADCTL_MX | ILI9341_MADCTL_MY  \
                              | ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR)
//*****************************************************
// SPI DMA settings and data
//*****************************************************
#ifdef __USE_DISPLAY_DMA__
static const stm32_dma_stream_t *dmatx =
    STM32_DMA_STREAM(STM32_SPI_SPI1_TX_DMA_STREAM);
static const uint32_t txdmamode =
    STM32_DMA_CR_CHSEL(SPI1_TX_DMA_CHANNEL)         // Select SPI1 Tx DMA
    | STM32_DMA_CR_PL(STM32_SPI_SPI1_DMA_PRIORITY)  // Set priority
    | STM32_DMA_CR_DIR_M2P;                         // Memory to Spi

// Not handle interrupt
#if 0
static void spi_lld_serve_tx_interrupt(SPIDriver *spip, uint32_t flags)
{
  (void)spip;
  (void)flags;
}
#endif

#ifdef __USE_DISPLAY_DMA_RX__
static const stm32_dma_stream_t  *dmarx = STM32_DMA_STREAM(STM32_SPI_SPI1_RX_DMA_STREAM);
static const uint32_t rxdmamode =
    STM32_DMA_CR_CHSEL(SPI1_RX_DMA_CHANNEL)         // Select SPI1 Rx DMA
    | STM32_DMA_CR_PL(STM32_SPI_SPI1_DMA_PRIORITY)  // Set priority
    | STM32_DMA_CR_DIR_P2M;                         // SPI to Memory

// Not handle interrupt
#if 0
static void spi_lld_serve_rx_interrupt(SPIDriver *spip, uint32_t flags)
{
  (void)spip;
  (void)flags;
}
#endif
#endif

// Send prepared DMA data, and wait completion
static void dmaStreamFlush(uint32_t len)
{
  while (len) {
    // DMA data transfer limited by 65535
    uint16_t tx_size = len > 65535 ? 65535 : len;
    dmaStreamSetTransactionSize(dmatx, tx_size);
    dmaStreamEnable(dmatx);
    len -= tx_size;
    dmaWaitCompletion(dmatx);
  }
}
#endif

// SPI transmit byte to SPI
static void spi_TxByte(uint8_t data) {
  while (SPI_TX_IS_NOT_EMPTY(LCD_SPI));
  SPI_WRITE_8BIT(LCD_SPI, data);
}

// Transmit word to SPI bus (if SPI in 8 bit mode LSB send first!!!!!)
static inline void spi_TxWord(uint16_t data) {
  while (SPI_TX_IS_NOT_EMPTY(LCD_SPI));
  SPI_WRITE_16BIT(LCD_SPI, data);
}

// Transmit byte to SPI bus  (len should be > 0)
static void  spi_TxBuffer(uint8_t *buffer, uint16_t len) {
  do {
    while (SPI_TX_IS_NOT_EMPTY(LCD_SPI));
    SPI_WRITE_8BIT(LCD_SPI, *buffer++);
  }while(--len);
}
// Receive byte from SPI bus
static uint8_t spi_RxByte(void) {
  // Start RX clock (by sending data)
  SPI_WRITE_8BIT(LCD_SPI, 0xFF);
  while (SPI_RX_IS_EMPTY(LCD_SPI) || SPI_IS_BUSY(LCD_SPI));
  return SPI_READ_8BIT(LCD_SPI);
}

// Receive byte from SPI bus (len should be > 0)
static void spi_RxBuffer(uint8_t *buffer, uint16_t len) {
  do{
    SPI_WRITE_8BIT(LCD_SPI, 0xFF);
    while (SPI_RX_IS_EMPTY(LCD_SPI));
    *buffer++ = SPI_READ_8BIT(LCD_SPI);
  }while(--len);
}

static void spi_DropRx(void){
  // Drop Rx buffer after tx
  while (SPI_RX_IS_NOT_EMPTY(LCD_SPI))
    (void)SPI_READ_8BIT(LCD_SPI);
}

#ifdef __USE_DISPLAY_DMA__
// SPI receive byte buffer use DMA
static void spi_DMATxBuffer(uint8_t *buffer, uint16_t len) {
  dmaStreamSetMemory0(dmatx, buffer);
  dmaStreamSetMode(dmatx, txdmamode | STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE | STM32_DMA_CR_MINC);
  dmaStreamFlush(len);
}
#ifdef __USE_DISPLAY_DMA_RX__
// SPI transmit byte buffer use DMA
static void spi_DMARxBuffer(uint8_t *buffer, uint16_t len) {
  uint8_t dummy_tx = 0xFF;
  // Init Rx DMA buffer, size, mode (spi and mem data size is 8 bit)
  dmaStreamSetMemory0(dmarx, buffer);
  dmaStreamSetTransactionSize(dmarx, len);
  dmaStreamSetMode(dmarx, rxdmamode | STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE | STM32_DMA_CR_MINC);
  // Init dummy Tx DMA (for rx clock), size, mode (spi and mem data size is 8 bit)
  dmaStreamSetMemory0(dmatx, &dummy_tx);
  dmaStreamSetTransactionSize(dmatx, len);
  dmaStreamSetMode(dmatx, txdmamode | STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE);
  // Start DMA exchange
  dmaStreamEnable(dmarx);
  dmaStreamEnable(dmatx);
  // Wait DMA completion
  dmaWaitCompletion(dmatx);
  dmaWaitCompletion(dmarx);
  // Skip SPI rx buffer
//  spi_DropRx();
}
#endif
#endif

static void spi_init(void)
{
  rccEnableSPI1(FALSE);
  LCD_SPI->CR1 = 0;
  LCD_SPI->CR1 = SPI_CR1_MSTR      // SPI is MASTER
               | SPI_CR1_SSM       // Software slave management (The external NSS pin is free for other application uses)
               | SPI_CR1_SSI       // Internal slave select (This bit has an effect only when the SSM bit is set. Allow use NSS pin as I/O)
               | LCD_SPI_SPEED;    // Baud rate control

  LCD_SPI->CR2 = SPI_CR2_8BIT      // SPI data size, set to 8 bit
               | SPI_CR2_FRXTH;    // SPI_SR_RXNE generated every 8 bit data
//             | SPI_CR2_SSOE;     //

#ifdef __USE_DISPLAY_DMA__
  // Tx DMA init
  dmaStreamAllocate(dmatx, STM32_SPI_SPI1_IRQ_PRIORITY, NULL, NULL);
  dmaStreamSetPeripheral(dmatx, &LCD_SPI->DR);
  LCD_SPI->CR2|= SPI_CR2_TXDMAEN;    // Tx DMA enable
#ifdef __USE_DISPLAY_DMA_RX__
  // Rx DMA init
  dmaStreamAllocate(dmarx, STM32_SPI_SPI1_IRQ_PRIORITY, NULL, NULL);
  dmaStreamSetPeripheral(dmarx, &LCD_SPI->DR);
  // Enable DMA on SPI
  LCD_SPI->CR2|= SPI_CR2_RXDMAEN;   // Rx DMA enable
#endif
#endif
  LCD_SPI->CR1|= SPI_CR1_SPE;       //SPI enable
}

// Disable inline for this function
static void send_command(uint8_t cmd, uint8_t len, const uint8_t *data)
{
// Uncomment on low speed SPI (possible get here before previous tx complete)
//  while (SPI_IN_TX_RX);
  LCD_CS_LOW;
  LCD_DC_CMD;
  SPI_WRITE_8BIT(LCD_SPI, cmd);
  // Need wait transfer complete and set data bit
  while (SPI_IN_TX_RX(LCD_SPI))
    ;
  // Send command data (if need)
  LCD_DC_DATA;
  while (len-- > 0) {
    while (SPI_TX_IS_NOT_EMPTY(LCD_SPI))
      ;
    SPI_WRITE_8BIT(LCD_SPI, *data++);
  }
  //LCD_CS_HIGH;
}

static const uint8_t ili9341_init_seq[] = {
  // cmd, len, data...,
  // SW reset
  ILI9341_SOFTWARE_RESET, 0,
  // display off
  ILI9341_DISPLAY_OFF, 0,
  // Power control B
  ILI9341_POWERB, 3, 0x00, 0xC1, 0x30,
  // Power on sequence control
  ILI9341_POWER_SEQ, 4, 0x64, 0x03, 0x12, 0x81,
  // Driver timing control A
  ILI9341_DTCA, 3, 0x85, 0x00, 0x78,
  // Power control A
  ILI9341_POWERA, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
  // Pump ratio control
  ILI9341_PUMP_RATIO_CONTROL, 1, 0x20,
  // Driver timing control B
  ILI9341_DTCB, 2, 0x00, 0x00,
  // POWER_CONTROL_1
  ILI9341_POWER_CONTROL_1, 1, 0x23,
  // POWER_CONTROL_2
  ILI9341_POWER_CONTROL_2, 1, 0x10,
  // VCOM_CONTROL_1
  ILI9341_VCOM_CONTROL_1, 2, 0x3e, 0x28,
  // VCOM_CONTROL_2
  ILI9341_VCOM_CONTROL_2, 1, 0xBE,
  // MEMORY_ACCESS_CONTROL
  //ILI9341_MEMORY_ACCESS_CONTROL, 1, 0x48, // portlait
  ILI9341_MEMORY_ACCESS_CONTROL, 1, DISPLAY_ROTATION_0, // landscape
  // COLMOD_PIXEL_FORMAT_SET : 16 bit pixel
  ILI9341_PIXEL_FORMAT_SET, 1, 0x55,
  // Frame Rate
  ILI9341_FRAME_RATE_CONTROL_1, 2, 0x00, 0x18,
  // Gamma Function Disable
  ILI9341_3GAMMA_EN, 1, 0x00,
  // gamma set for curve 01/2/04/08
  ILI9341_GAMMA_SET, 1, 0x01,
  // positive gamma correction
  ILI9341_POSITIVE_GAMMA_CORRECTION, 15, 0x0F,  0x31,  0x2B,  0x0C,  0x0E,  0x08,  0x4E,  0xF1,  0x37,  0x07,  0x10,  0x03,  0x0E, 0x09,  0x00,
  // negativ gamma correction
  ILI9341_NEGATIVE_GAMMA_CORRECTION, 15, 0x00,  0x0E,  0x14,  0x03,  0x11,  0x07,  0x31,  0xC1,  0x48,  0x08,  0x0F,  0x0C,  0x31, 0x36,  0x0F,
  // Column Address Set
//  ILI9341_COLUMN_ADDRESS_SET, 4, 0x00, 0x00, 0x01, 0x3f, // width 320
  // Page Address Set
//  ILI9341_PAGE_ADDRESS_SET, 4, 0x00, 0x00, 0x00, 0xef,   // height 240
  // entry mode
  ILI9341_ENTRY_MODE_SET, 1, 0x06,
  // display function control
  ILI9341_DISPLAY_FUNCTION_CONTROL, 3, 0x08, 0x82, 0x27,
  // Interface Control (set WEMODE=0)
  ILI9341_INTERFACE_CONTROL, 3, 0x00, 0x00, 0x00,
  // sleep out
  ILI9341_SLEEP_OUT, 0,
  // display on
  ILI9341_DISPLAY_ON, 0,
  0 // sentinel
};

void ili9341_init(void)
{
  spi_init();
  LCD_DC_DATA;
  LCD_RESET_ASSERT;
  chThdSleepMilliseconds(10);
  LCD_RESET_NEGATE;
  const uint8_t *p;
  for (p = ili9341_init_seq; *p; ) {
    send_command(p[0], p[1], &p[2]);
    p += 2 + p[1];
    chThdSleepMilliseconds(5);
  }
}

void ili9341_bulk_8bit(int x, int y, int w, int h, uint16_t *palette)
{
//uint8_t xx[4] = { x >> 8, x, (x+w-1) >> 8, (x+w-1) };
//uint8_t yy[4] = { y >> 8, y, (y+h-1) >> 8, (y+h-1) };
  uint32_t xx = __REV16(x | ((x + w - 1) << 16));
  uint32_t yy = __REV16(y | ((y + h - 1) << 16));
  send_command(ILI9341_COLUMN_ADDRESS_SET, 4, (uint8_t *)&xx);
  send_command(ILI9341_PAGE_ADDRESS_SET, 4, (uint8_t *)&yy);
  send_command(ILI9341_MEMORY_WRITE, 0, NULL);

  uint8_t *buf = (uint8_t *)spi_buffer;
  int32_t len = w * h;
  while (len-- > 0)
    spi_TxWord(palette[*buf++]);
}

#ifndef __USE_DISPLAY_DMA__
void ili9341_fill(int x, int y, int w, int h, uint16_t color)
{
//uint8_t xx[4] = { x >> 8, x, (x+w-1) >> 8, (x+w-1) };
//uint8_t yy[4] = { y >> 8, y, (y+h-1) >> 8, (y+h-1) };
  uint32_t xx = __REV16(x | ((x + w - 1) << 16));
  uint32_t yy = __REV16(y | ((y + h - 1) << 16));
  send_command(ILI9341_COLUMN_ADDRESS_SET, 4, (uint8_t*)&xx);
  send_command(ILI9341_PAGE_ADDRESS_SET, 4, (uint8_t*)&yy);
  send_command(ILI9341_MEMORY_WRITE, 0, NULL);
  int32_t len = w * h;
  while (len-- > 0)
    spi_TxWord(color);
}

void ili9341_bulk(int x, int y, int w, int h)
{
//uint8_t xx[4] = { x >> 8, x, (x+w-1) >> 8, (x+w-1) };
//uint8_t yy[4] = { y >> 8, y, (y+h-1) >> 8, (y+h-1) };
  uint16_t *buf = spi_buffer;
  uint32_t xx = __REV16(x | ((x + w - 1) << 16));
  uint32_t yy = __REV16(y | ((y + h - 1) << 16));
  send_command(ILI9341_COLUMN_ADDRESS_SET, 4, (uint8_t *)&xx);
  send_command(ILI9341_PAGE_ADDRESS_SET, 4, (uint8_t*)&yy);
  send_command(ILI9341_MEMORY_WRITE, 0, NULL);
  int32_t len = w * h;
  while (len-- > 0)
    spi_TxWord(*buf++);
}
#else
//
// Use DMA for send data
//
// Fill region by some color
void ili9341_fill(int x, int y, int w, int h, uint16_t color)
{
//uint8_t xx[4] = { x >> 8, x, (x+w-1) >> 8, (x+w-1) };
//uint8_t yy[4] = { y >> 8, y, (y+h-1) >> 8, (y+h-1) };
  uint32_t xx = __REV16(x | ((x + w - 1) << 16));
  uint32_t yy = __REV16(y | ((y + h - 1) << 16));
  send_command(ILI9341_COLUMN_ADDRESS_SET, 4, (uint8_t *)&xx);
  send_command(ILI9341_PAGE_ADDRESS_SET, 4, (uint8_t *)&yy);
  send_command(ILI9341_MEMORY_WRITE, 0, NULL);

  dmaStreamSetMemory0(dmatx, &color);
  dmaStreamSetMode(dmatx, txdmamode | STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD);
  dmaStreamFlush(w * h);
}

// Copy spi_buffer to region
void ili9341_bulk(int x, int y, int w, int h)
{
  uint32_t xx = __REV16(x | ((x + w - 1) << 16));
  uint32_t yy = __REV16(y | ((y + h - 1) << 16));
  send_command(ILI9341_COLUMN_ADDRESS_SET, 4, (uint8_t *)&xx);
  send_command(ILI9341_PAGE_ADDRESS_SET, 4, (uint8_t *)&yy);
  send_command(ILI9341_MEMORY_WRITE, 0, NULL);

  // Init Tx DMA mem->spi, set size, mode (spi and mem data size is 16 bit)
  dmaStreamSetMemory0(dmatx, spi_buffer);
  dmaStreamSetMode(dmatx, txdmamode | STM32_DMA_CR_PSIZE_HWORD |
                              STM32_DMA_CR_MSIZE_HWORD | STM32_DMA_CR_MINC);
  dmaStreamFlush(w * h);
}
#endif

#ifndef __USE_DISPLAY_DMA_RX__

void ili9341_read_memory(int x, int y, int w, int h, int len, uint16_t *out)
{
//uint8_t xx[4] = { x >> 8, x, (x+w-1) >> 8, (x+w-1) };
//uint8_t yy[4] = { y >> 8, y, (y+h-1) >> 8, (y+h-1) };
  uint32_t xx = __REV16(x | ((x + w - 1) << 16));
  uint32_t yy = __REV16(y | ((y + h - 1) << 16));
  send_command(ILI9341_COLUMN_ADDRESS_SET, 4, (uint8_t *)&xx);
  send_command(ILI9341_PAGE_ADDRESS_SET, 4, (uint8_t*)&yy);
  send_command(ILI9341_MEMORY_READ, 0, NULL);
  // Skip data from rx buffer
  spi_DropRx();
  // require 8bit dummy clock
  spi_RxByte();
  while (len-- > 0) {
    uint8_t r, g, b;
    // read data is always 18bit
    r = spi_RxByte();
    g = spi_RxByte();
    b = spi_RxByte();
    *out++ = RGB565(r, g, b);
  }
  LCD_CS_HIGH;
}

#else
// Copy screen data to buffer
// Warning!!! buffer size must be greater then 3*len + 1 bytes
void ili9341_read_memory(int x, int y, int w, int h, int len, uint16_t *out)
{
  uint16_t dummy_tx = 0;
  uint8_t *rgbbuf = (uint8_t *)out;
  uint16_t data_size = len * 3 + 1;
  uint32_t xx = __REV16(x | ((x + w - 1) << 16));
  uint32_t yy = __REV16(y | ((y + h - 1) << 16));
  send_command(ILI9341_COLUMN_ADDRESS_SET, 4, (uint8_t *)&xx);
  send_command(ILI9341_PAGE_ADDRESS_SET, 4, (uint8_t *)&yy);
  send_command(ILI9341_MEMORY_READ, 0, NULL);
  // Skip data from rx buffer
  spi_DropRx();
  // Init Rx DMA buffer, size, mode (spi and mem data size is 8 bit)
  dmaStreamSetMemory0(dmarx, rgbbuf);
  dmaStreamSetTransactionSize(dmarx, data_size);
  dmaStreamSetMode(dmarx, rxdmamode | STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE |
                              STM32_DMA_CR_MINC);
  // Init dummy Tx DMA (for rx clock), size, mode (spi and mem data size is 8 bit)
  dmaStreamSetMemory0(dmatx, &dummy_tx);
  dmaStreamSetTransactionSize(dmatx, data_size);
  dmaStreamSetMode(dmatx, txdmamode | STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE);

  // Start DMA exchange
  dmaStreamEnable(dmarx);
  dmaStreamEnable(dmatx);
  // Wait DMA completion
  dmaWaitCompletion(dmatx);
  dmaWaitCompletion(dmarx);
  LCD_CS_HIGH;

  // Parce recived data
  // Skip dummy 8-bit read
  rgbbuf++;
  while (len-- > 0) {
    uint8_t r, g, b;
    // read data is always 18bit
    r = rgbbuf[0];
    g = rgbbuf[1];
    b = rgbbuf[2];
    *out++ = RGB565(r, g, b);
    rgbbuf += 3;
  }
}
#endif

void ili9341_clear_screen(void) 
{
  ili9341_fill(0, 0, ILI9341_WIDTH, ILI9341_HEIGHT, background_color);
}

void ili9341_set_foreground(uint16_t fg)
{
  foreground_color = fg;
}

void ili9341_set_background(uint16_t bg)
{
  background_color = bg; 
}

void ili9341_set_rotation(uint8_t r)
{
  //  static const uint8_t rotation_const[]={DISPLAY_ROTATION_0, DISPLAY_ROTATION_90,
  //  DISPLAY_ROTATION_180, DISPLAY_ROTATION_270};
  send_command(ILI9341_MEMORY_ACCESS_CONTROL, 1, &r);
}

void blit8BitWidthBitmap(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                         const uint8_t *bitmap)
{
  uint16_t *buf = spi_buffer;
  for (uint16_t c = 0; c < height; c++) {
    uint8_t bits = *bitmap++;
    for (uint16_t r = 0; r < width; r++) {
      *buf++ = (0x80 & bits) ? foreground_color : background_color;
      bits <<= 1;
    }
  }
  ili9341_bulk(x, y, width, height);
}

static void blit16BitWidthBitmap(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                                 const uint16_t *bitmap)
{
  uint16_t *buf = spi_buffer;
  for (uint16_t c = 0; c < height; c++) {
    uint16_t bits = *bitmap++;
    for (uint16_t r = 0; r < width; r++) {
      *buf++ = (0x8000 & bits) ? foreground_color : background_color;
      bits <<= 1;
    }
  }
  ili9341_bulk(x, y, width, height);
}

void ili9341_drawchar(uint8_t ch, int x, int y)
{
  blit8BitWidthBitmap(x, y, FONT_GET_WIDTH(ch), FONT_GET_HEIGHT, FONT_GET_DATA(ch));
}

void ili9341_drawstring(const char *str, int x, int y)
{
  int x_pos = x;
  while (*str) {
    uint8_t ch = *str++;
    if (ch == '\n') {x = x_pos; y+=FONT_STR_HEIGHT; continue;}
    const uint8_t *char_buf = FONT_GET_DATA(ch);
    uint16_t w = FONT_GET_WIDTH(ch);
    blit8BitWidthBitmap(x, y, w, FONT_GET_HEIGHT, char_buf);
    x += w;
  }
}

void ili9341_drawstringV(const char *str, int x, int y)
{
  ili9341_set_rotation(DISPLAY_ROTATION_270);
  ili9341_drawstring(str, ILI9341_HEIGHT-y, x);
  ili9341_set_rotation(DISPLAY_ROTATION_0);
}

int ili9341_drawchar_size(uint8_t ch, int x, int y, uint8_t size)
{
  uint16_t *buf = spi_buffer;
  const uint8_t *char_buf = FONT_GET_DATA(ch);
  uint16_t w = FONT_GET_WIDTH(ch);
  for (int c = 0; c < FONT_GET_HEIGHT; c++, char_buf++) {
    for (int i = 0; i < size; i++) {
      uint8_t bits = *char_buf;
      for (int r = 0; r < w; r++, bits <<= 1)
        for (int j = 0; j < size; j++)
          *buf++ = (0x80 & bits) ? foreground_color : background_color;
    }
  }
  ili9341_bulk(x, y, w * size, FONT_GET_HEIGHT * size);
  return w*size;
}

void ili9341_drawfont(uint8_t ch, int x, int y)
{
  blit16BitWidthBitmap(x, y, NUM_FONT_GET_WIDTH, NUM_FONT_GET_HEIGHT,
                       NUM_FONT_GET_DATA(ch));
}

void ili9341_drawstring_size(const char *str, int x, int y, uint8_t size)
{
  while (*str)
    x += ili9341_drawchar_size(*str++, x, y, size);
}
#if 0
static void ili9341_pixel(int x, int y, uint16_t color)
{
  uint32_t xx = __REV16(x|((x)<<16));
  uint32_t yy = __REV16(y|((y)<<16));
  send_command(ILI9341_COLUMN_ADDRESS_SET, 4, (uint8_t*)&xx);
  send_command(ILI9341_PAGE_ADDRESS_SET, 4, (uint8_t*)&yy);
  send_command(ILI9341_MEMORY_WRITE, 2, &color);
}
#endif

#define SWAP(x, y) { int z = x; x = y; y = z; }

void ili9341_line(int x0, int y0, int x1, int y1)
{
#if 0
  // modifed Bresenham's line algorithm, see https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
  int dx = x1 - x0, sx = 1; if (dx < 0) {dx = -dx; sx = -1;}
  int dy = y1 - y0, sy = 1; if (dy < 0) {dy = -dy; sy = -1;}
  int err = (dx > dy ? dx : -dy) / 2;
  while (1) {
    ili9341_pixel(x0, y0, DEFAULT_FG_COLOR);
    if (x0 == x1 && y0 == y1)
      break;
    int e2 = err;
    if (e2 > -dx) { err -= dy; x0 += sx; }
    if (e2 <  dy) { err += dx; y0 += sy; }
  }
#endif

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
        dx /= -dy;dy = -1;
      }
    }
    if (dy > 0)
      ili9341_fill(x0, y0, dx, dy, foreground_color);
    else
      ili9341_fill(x0, y0+dy, dx, -dy, foreground_color);
    x0 += dx;
    y0 += dy;
  }
}

#if 0
static const uint16_t colormap[] = {
  RGBHEX(0x00ff00), RGBHEX(0x0000ff), RGBHEX(0xff0000),
  RGBHEX(0x00ffff), RGBHEX(0xff00ff), RGBHEX(0xffff00)
};

void ili9341_test(int mode)
{
  int x, y;
  int i;
  switch (mode) {
    default:
#if 1
    ili9341_fill(0, 0, LCD_WIDTH, LCD_HEIGHT, 0);
    for (y = 0; y < LCD_HEIGHT; y++) {
      ili9341_fill(0, y, LCD_WIDTH, 1, RGB(LCD_HEIGHT-y, y, (y + 120) % 256));
    }
    break;
    case 1:
      ili9341_fill(0, 0, LCD_WIDTH, LCD_HEIGHT, 0);
      for (y = 0; y < LCD_HEIGHT; y++) {
        for (x = 0; x < LCD_WIDTH; x++) {
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
        ili9341_drawfont(i, i*20, 120);
    break;
#endif
#if 0
    case 4:
      draw_grid(10, 8, 29, 29, 15, 0, 0xffff, 0);
    break;
#endif
    case 4:
      ili9341_line(0, 0, 15, 100);
      ili9341_line(0, 0, 100, 100);
      ili9341_line(0, 15, 100, 0);
      ili9341_line(0, 100, 100, 0);
    break;
  }
}
#endif

#ifdef __USE_SD_CARD__
//*****************************************************
//* SD functions and definitions
//*****************************************************
// Definitions for MMC/SDC command
#define CMD0     (0x40+0)     // GO_IDLE_STATE
#define CMD1     (0x40+1)     // SEND_OP_COND
#define CMD8     (0x40+8)     // SEND_IF_COND
#define CMD9     (0x40+9)     // SEND_CSD
#define CMD10    (0x40+10)    // SEND_CID
#define CMD12    (0x40+12)    // STOP_TRANSMISSION
#define CMD13    (0x40+13)    // SEND_STATUS
#define CMD16    (0x40+16)    // SET_BLOCKLEN
#define CMD17    (0x40+17)    // READ_SINGLE_BLOCK
#define CMD18    (0x40+18)    // READ_MULTIPLE_BLOCK
#define CMD23    (0x40+23)    // SET_BLOCK_COUNT
#define CMD24    (0x40+24)    // WRITE_BLOCK
#define CMD25    (0x40+25)    // WRITE_MULTIPLE_BLOCK
#define CMD55    (0x40+55)    // APP_CMD
#define CMD58    (0x40+58)    // READ_OCR
#define CMD59    (0x40+59)    // CRC_ON_OFF
// Then send after CMD55 (APP_CMD) interpret as ACMD
#define ACMD41   (0x40+41)    // SEND_OP_COND (ACMD)

// MMC card type flags (MMC_GET_TYPE)
#define CT_MMC      0x01      // MMC v3
#define CT_SD1      0x02      // SDv1
#define CT_SD2      0x04      // SDv2
#define CT_SDC      0x06      // SD
#define CT_BLOCK    0x08      // Block addressing

// 7.3.2 Responses
// 7.3.2.1 Format R1 (1 byte)
// This response token is sent by the card after every command with the exception of SEND_STATUS commands.
#define SD_R1_IDLE                                 ((uint8_t)0x01) // The card is in idle state
#define SD_R1_ERASE_RESET                          ((uint8_t)0x02) // erase reset
#define SD_R1_ILLEGAL_CMD                          ((uint8_t)0x04) // Illegal command
#define SD_R1_CRC_ERROR                            ((uint8_t)0x08) // The CRC check of the last command failed
#define SD_R1_ERR_ERASE_CLR                        ((uint8_t)0x10) // error in the sequence of erase commands
#define SD_R1_ADDR_ERROR                           ((uint8_t)0x20) // Incorrect address specified
#define SD_R1_PARAM_ERROR                          ((uint8_t)0x40) // Parameter error
#define SD_R1_NOT_R1                               ((uint8_t)0x80) // Not R1 register
// 7.3.2.2 Format R1b (R1 + Busy)
// The busy signal token can be any number of bytes. A zero value indicates card is busy.
// A non-zero value indicates the card is ready for the next command.
// 7.3.2.3 Format R2  (2 byte)
// This response token is two bytes long and sent as a response to the SEND_STATUS command.
// 1 byte - some as R1
// 2 byte -

// 7.3.2.4 Format R3 (R1 + OCR, 5 bytes)
// This response token is sent by the card when a READ_OCR command is received.
// 1 byte - some as R1
// 2-5 byte - OCR
// On Send byte order in SendCommand send MSB first!!
// Received byte order MSB last!!
#define _OCR(dword) (((dword&0x000000FF)<<24)|((dword&0x0000FF00)<<8)|((dword&0x00FF0000)>>8)|((dword&0xFF000000)>>24))
#define SD_OCR_LOW_VOLTAGE                         ((uint32_t)0x00000080) // Reserved for Low Voltage Range
#define SD_OCR_27_VOLTAGE                          ((uint32_t)0x00008000) // VDD Voltage Window 2.7-2.8V
#define SD_OCR_28_VOLTAGE                          ((uint32_t)0x00010000) // VDD Voltage Window 2.8-2.9V
#define SD_OCR_29_VOLTAGE                          ((uint32_t)0x00020000) // VDD Voltage Window 2.9-3.0V
#define SD_OCR_30_VOLTAGE                          ((uint32_t)0x00040000) // VDD Voltage Window 3.0-3.1V
#define SD_OCR_31_VOLTAGE                          ((uint32_t)0x00080000) // VDD Voltage Window 3.1-3.2V
#define SD_OCR_32_VOLTAGE                          ((uint32_t)0x00100000) // VDD Voltage Window 3.2-3.3V
#define SD_OCR_33_VOLTAGE                          ((uint32_t)0x00200000) // VDD Voltage Window 3.3-3.4V
#define SD_OCR_34_VOLTAGE                          ((uint32_t)0x00400000) // VDD Voltage Window 3.4-3.8V
#define SD_OCR_35_VOLTAGE                          ((uint32_t)0x00800000) // VDD Voltage Window 3.5-3.6V
#define SD_OCR_18_VOLTAGE                          ((uint32_t)0x01000000) // VDD Voltage switch to 1.8V (UHS-I only)
#define SD_OCR_CAPACITY                            ((uint32_t)0x40000000) // Card Capacity Status (CCS)
#define SD_OCR_BUSY                                ((uint32_t)0x80000000) // Card power up status bit (busy)

// 5.3 CSD Register
//  16GB Kingston  40 0E 00 32 5B 59 00 00 73 A7 7F 80 0A 40 00 EB   //  29608 * 512 kB
//  32GB Samsung   40 0E 00 32 5B 59 00 00 EE 7F 7F 80 0A 40 40 55   //  61056 * 512 kB
// 128GB Samsung   40 0E 00 32 5B 59 00 03 B9 FF 7F 80 0A 40 40 AB   // 244224 * 512 kB
#define CSD_0_STRUCTURE                          0b11000000
#define CSD_1_TAAC                               0b11111111
#define CSD_2_NSAC                               0b11111111
#define CSD_3_TRAN_SPEED                         0b11111111
#define CSD_4_CCC                                0b11111111
#define CSD_5_CCC                                0b11110000
#define CSD_5_READ_BL_LEN                        0b00001111
#define CSD_6_READ_BL_PARTIAL                    0b10000000
#define CSD_6_WRITE_BLK_MISALIGN                 0b01000000
#define CSD_6_READ_BLK_MISALIGN                  0b00100000
#define CSD_6_DSR_IMP                            0b00010000
#define CSD_7_C_SIZE                             0b00111111
#define CSD_8_C_SIZE                             0b11111111
#define CSD_9_C_SIZE                             0b11111111
#define CSD_10_ERASE_BLK_EN                      0b01000000
#define CSD_10_SECTOR_SIZE                       0b00111111
#define CSD_11_SECTOR_SIZE                       0b10000000
#define CSD_11_WP_GRP_SIZE                       0b01111111
#define CSD_12_WP_GRP_ENABLE                     0b10000000
#define CSD_12_R2W_FACTOR                        0b00011100
#define CSD_12_WRITE_BL_LEN                      0b00000011
#define CSD_13_WRITE_BL_LEN                      0b11000000
#define CSD_13_WRITE_BL_PARTIAL                  0b00100000
#define CSD_14_FILE_FORMAT_GRP                   0b10000000
#define CSD_14_COPY                              0b01000000
#define CSD_14_PERM_WRITE_PROTECT                0b00100000
#define CSD_14_TMP_WRITE_PROTECT                 0b00010000
#define CSD_14_FILE_FORMAT                       0b00001100
#define CSD_15_CRC                               0b11111110


// 7.3.3.1 Data Response Token
#define SD_TOKEN_DATA_ACCEPTED                     ((uint8_t)0x05) // Data accepted
#define SD_TOKEN_WRITE_CRC_ERROR                   ((uint8_t)0x0b) // Data rejected due to a CRC error
#define SD_TOKEN_WRITE_ERROR                       ((uint8_t)0x0d) // Data rejected due to a write error
// 7.3.3.2 Start Block Tokens and Stop Tran Token
#define SD_TOKEN_START_BLOCK                       ((uint8_t)0xfe) // Start block (single tx, single/multiple rx)
#define SD_TOKEN_START_M_BLOCK                     ((uint8_t)0xfc) // Start multiple block tx
#define SD_TOKEN_STOP_M_BLOCK                      ((uint8_t)0xfd) // Stop multiple block tx
// 7.3.3.3 Data Error Token
#define SD_TOKEN_READ_ERROR                        ((uint8_t)0x01) // Data read error
#define SD_TOKEN_READ_CC_ERROR                     ((uint8_t)0x02) // Internal card controller error
#define SD_TOKEN_READ_ECC_ERROR                    ((uint8_t)0x04) // Card ECC failed
#define SD_TOKEN_READ_RANGE_ERROR                  ((uint8_t)0x08) // Read address out of range

//*****************************************************
//             SD card module settings
//*****************************************************
// Additional state flag definition
#define STA_POWER_ON       0x80   // Power ON flag

// Use for enable CRC check of Tx and Rx data on SPI
// If enable both CRC check, on initialization send SD command - CRC_ON_OFF vs ON
// And Card begin check received data and answer on CRC errors
//#define SD_USE_COMMAND_CRC
//#define SD_USE_DATA_CRC
// Use DMA on sector data Tx to SD card
#ifdef __USE_DISPLAY_DMA__
#define __USE_SDCARD_DMA__
#endif

// Define sector size
#define SD_SECTOR_SIZE      512
// SD card spi bus
#define SD_SPI              SPI1
// Define SD SPI speed on work
#define SD_SPI_SPEED        SPI_BR_DIV2
// div4 give less error and high speed for Rx
#define SD_SPI_RX_SPEED     SPI_BR_DIV2

// Define SD SPI speed on initialization (100-400kHz need)
#define SD_INIT_SPI_SPEED   SPI_BR_DIV256
// Set number of try read or write sector data (1 only one try)
#define SD_READ_WRITE_REPEAT 1
// Local values for SD card state
static DSTATUS Stat = STA_NOINIT;  // Disk Status
static uint8_t CardType  = 0;      // Type 0:MMC, 1:SDC, 2:Block addressing

// Debug
#define DEBUG    0
int shell_printf(const char *fmt, ...);
#define DEBUG_PRINT(...) do { if (DEBUG) shell_printf(__VA_ARGS__); } while (0)
//uint32_t w_cnt;
//uint32_t w_time;
//uint32_t r_cnt;
//uint32_t r_time;
//uint32_t crc_time;


//void testLog(void){
//  DEBUG_PRINT(" Read  speed = %d Byte/s (count %d, time %d)\r\n", r_cnt*512*10000/r_time, r_cnt, r_time);
//  DEBUG_PRINT(" Write speed = %d Byte/s (count %d, time %d)\r\n", w_cnt*512*10000/w_time, w_cnt, w_time);
//  DEBUG_PRINT(" CRC16 time %d\r\n", crc_time);
//}

//*******************************************************
//               SD card SPI functions
//*******************************************************
#define SD_CS_LOW     palClearPad(GPIOB, GPIOB_SD_CS)
#define SD_CS_HIGH    palSetPad(GPIOB, GPIOB_SD_CS)

static void SD_Select_SPI(uint32_t speed) {
  LCD_CS_HIGH;               // Unselect LCD
  SPI_BR_SET(SD_SPI, speed); // Set Baud rate control for SD card
  SD_CS_LOW;                 // Select SD Card
}

static void SD_Unselect_SPI(void) {
  SD_CS_HIGH;                         // Unselect SD Card
  spi_RxByte();                       // Dummy read/write one Byte recommend for SD after CS up
  SPI_BR_SET(LCD_SPI, LCD_SPI_SPEED); // Restore Baud rate for LCD
}

//*******************************************************
//*           SD functions
//*******************************************************
// CRC7 used for commands
#ifdef SD_USE_COMMAND_CRC
#define CRC7_POLY  0x89
#define CRC7_INIT  0x00
//                                        7   3
// CRC7 it's a 7 bit CRC with polynomial x + x + 1
static uint8_t crc7(const uint8_t *ptr, uint16_t count) {
  uint8_t crc = CRC7_INIT;
  uint8_t i = 0;
  while (count--){
    crc ^= *ptr++;
    do{
      if (crc & 0x80) crc^=CRC7_POLY;
      crc = crc << 1;
    } while((++i)&0x7);
  }
  return crc;
}
#endif
// CRC16 used for data
#ifdef SD_USE_DATA_CRC
#define CRC16_POLY  0x1021
#define CRC16_INIT  0x0000
//                                      16   12   5
// This is the CCITT CRC 16 polynomial X  + X  + X  + 1.
static uint16_t crc16(const uint8_t *ptr, uint16_t count) {
  uint16_t crc = CRC16_INIT;
  #if 0
  uint8_t i = 0;
  while(count--){
    crc^= ((uint16_t) *ptr++ << 8);
    do{
      if (crc & 0x8000)
        crc = (crc << 1) ^ CRC16_POLY;
      else
        crc = crc << 1;
    } while((++i)&0x7);
  }
  return __REVSH(crc); // swap bytes
  #else
  while (count--){
    crc^= *ptr++;
    crc^= (crc>> 4)&0x000F;
    crc^= (crc<<12);
    crc^= (crc<< 5)&0x1FE0;
    crc = __REVSH(crc); // swap bytes
  }
  return crc;
  #endif
}
#endif

// Wait and read R1 answer from SD
static inline uint8_t SD_ReadR1(uint32_t cnt) {
  uint8_t r1;
   // 8th bit R1 always zero, check it
  while(((r1=spi_RxByte())&0x80) && --cnt)
    ;
  return r1;
}

// Wait SD ready token answer (wait time in systick, 10 systick = 1ms)
static inline bool SD_WaitDataToken(uint8_t token, uint32_t wait_time) {
  uint8_t res;
  uint32_t time = chVTGetSystemTimeX();
  uint32_t count = 0;
  do{
    if ((res = spi_RxByte()) == token)
      return true;
    count++;
    // Check timeout only every 65536 bytes read (~50ms interval)
    if ((count&0xFFFF) == 0)
      if ((chVTGetSystemTimeX() - time) > wait_time)
        break;
  }while (res == 0xFF);
  return false;
}

static inline uint8_t SD_WaitDataAccept(uint32_t cnt) {
  uint8_t res;
  while ((res = spi_RxByte()) == 0xFF && --cnt)
    ;
  return res&0x1F;
}

// Wait no Busy answer from SD (wait time in systick, 10 systick = 1ms)
static uint8_t SD_WaitNotBusy(uint32_t wait_time) {
  uint8_t res;
  uint32_t time = chVTGetSystemTimeX();
  uint32_t count = 0;
  do{
    if ((res = spi_RxByte()) == 0xFF)
      return res;
    count++;
    // Check timeout only every 65536 bytes read (~50ms interval)
    if ((count&0xFFFF) == 0)
      if ((chVTGetSystemTimeX() - time) > wait_time)
        break;
  }while (1);
  return 0;
}

// Receive data block from SD
static bool SD_RxDataBlock(uint8_t *buff, uint16_t len, uint8_t token) {
  // loop until receive read response token or timeout ~50ms
  if (!SD_WaitDataToken(token, 500)) {
    DEBUG_PRINT(" rx SD_WaitDataToken err\r\n");
    return FALSE;
  }
  // Receive data (Not use rx DMA)
#if 1
  spi_RxBuffer(buff, len);
#else
  spi_DMARxBuffer(buff, len);
#endif
  // Read and check CRC (if enabled)
  uint16_t crc; spi_RxBuffer((uint8_t*)&crc, 2);
#ifdef SD_USE_DATA_CRC
  uint16_t bcrc = crc16(buff, len);
  if (crc!=bcrc){
    DEBUG_PRINT("CRC = %04x , hcalc = %04x, calc = %04x\r\n", (uint32_t)crc, (uint32_t)bcrc, (uint32_t)crc16(buff, len));
    return FALSE;
  }
#endif
  return TRUE;
}

// Transmit data block to SD
static bool SD_TxDataBlock(const uint8_t *buff, uint8_t token) {
  uint8_t resp;
  // Transmit token
  spi_TxByte(token);
#if 0         // Not use multiple block tx
  // if it's not STOP token, transmit data, in multiple block Tx
   if (token == SD_TOKEN_STOP_BLOCK) return TRUE;
#endif

#ifdef __USE_SDCARD_DMA__
  spi_DMATxBuffer((uint8_t*)buff, SD_SECTOR_SIZE);
#else
  spi_TxBuffer((uint8_t*)buff, SD_SECTOR_SIZE);
#endif
  spi_DropRx();
  // Send CRC
#ifdef  SD_USE_DATA_CRC
  uint16_t bcrc = crc16(buff, SD_SECTOR_SIZE);
  spi_TxWord(bcrc);
#else
  spi_TxWord(0xFFFF);
#endif
  // Receive transmit data response token on next 10 bytes
  resp = SD_WaitDataAccept(10);
  if (resp != SD_TOKEN_DATA_ACCEPTED){
    goto error_tx;
  }
#if 0
  // Wait busy (recommended timeout is 250ms (500ms for SDXC) set 250ms
  resp = SD_WaitNotBusy(2500);
  if (resp == 0xFF)
    return TRUE;
#else
  // Continue execute, wait not busy on next command
  return TRUE;
#endif
  DEBUG_PRINT(" Tx busy error = %04\r\n", (uint32_t)resp);
  return FALSE;
error_tx:
  DEBUG_PRINT(" Tx accept error = %04x\r\n", (uint32_t)resp);
  return FALSE;
}

// Transmit command to SD
static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg) {
  uint8_t buf[6];
  uint8_t r1;
  // wait SD ready after last Tx (recommended timeout is 250ms (500ms for SDXC) set 250ms
  if ((r1 = SD_WaitNotBusy(2500)) != 0xFF) {
    DEBUG_PRINT(" SD_WaitNotBusy CMD%d err, %02x\r\n", cmd-0x40, (uint32_t)r1);
    return 0xFF;
  }
  // Transmit command
  buf[0] = cmd;
  buf[1] = (arg >> 24)&0xFF;
  buf[2] = (arg >> 16)&0xFF;
  buf[3] = (arg >>  8)&0xFF;
  buf[4] = (arg >>  0)&0xFF;
#ifdef SD_USE_COMMAND_CRC
  buf[5] = crc7(buf, 5)|0x01;
#else
  uint8_t crc = 0x01;              // Dummy CRC + Stop
       if (cmd == CMD0) crc = 0x95;// Valid CRC for CMD0(0)
  else if (cmd == CMD8) crc = 0x87;// Valid CRC for CMD8(0x1AA)
  buf[5] = crc;
#endif
  spi_TxBuffer(buf, 6);
  spi_DropRx();
// Skip a stuff byte when STOP_TRANSMISSION
//if (cmd == CMD12) SPI_RxByte();
  // Receive response register r1
  r1 = SD_ReadR1(10);
#if 1
  if (r1&(SD_R1_NOT_R1|SD_R1_CRC_ERROR|SD_R1_ERASE_RESET|SD_R1_ERR_ERASE_CLR)){
    DEBUG_PRINT(" SD_SendCmd err CMD%d, 0x%x, 0x%08x\r\n", (uint32_t)cmd-0x40, (uint32_t)r1, arg);
    return r1;
  }
  if (r1&(~SD_R1_IDLE))
    DEBUG_PRINT(" SD_SendCmd CMD%d, 0x%x, 0x%08x\r\n", (uint32_t)cmd-0x40, (uint32_t)r1, arg);
#endif
  return r1;
}

// Power on SD
static void SD_PowerOn(void) {
  uint16_t n;
  LCD_CS_HIGH;
  // Dummy TxRx 80 bits for power up SD
  for (n=0;n<10;n++)
    spi_RxByte();
  SD_Select_SPI(SD_INIT_SPI_SPEED);
  // Set SD card to idle state
  if (SD_SendCmd(CMD0, 0) == SD_R1_IDLE)
    Stat|= STA_POWER_ON;
  else{
    Stat = STA_NOINIT;
  }
  SD_Unselect_SPI();
}

// Power off SD
static inline void SD_PowerOff(void) {
  Stat &= ~STA_POWER_ON;
}

// Check power flag
static inline uint8_t SD_CheckPower(void) {
  return Stat & STA_POWER_ON;
}

//*******************************************************
//       diskio.c functions for file system library
//*******************************************************
// If enable RTC - get RTC time
#if FF_FS_NORTC == 0
DWORD get_fattime (void) {
  return rtc_get_FAT();
}
#endif

// diskio.c - Initialize SD
DSTATUS disk_initialize(BYTE pdrv) {
// Debug counters
//  w_cnt = 0;
//  w_time = 0;
//  r_cnt = 0;
//  r_time = 0;
  if (pdrv != 0) return STA_NOINIT;
  // power on, try detect on bus, set card to idle state
  SD_PowerOn();
  // check disk type
  uint8_t  type = 0;
  uint32_t cnt = 100;
  // Set low SPI bus speed = PLL/256 (on 72MHz =281.250kHz)
  SD_Select_SPI(SD_INIT_SPI_SPEED);
  // send GO_IDLE_STATE command
  if (SD_SendCmd(CMD0, 0) == SD_R1_IDLE)
  {
	DEBUG_PRINT(" CMD0 Ok\r\n");
    // SDC V2+ accept CMD8 command, http://elm-chan.org/docs/mmc/mmc_e.html
    if (SD_SendCmd(CMD8, 0x00001AAU) == SD_R1_IDLE)
    {
      DEBUG_PRINT(" CMD8 Ok\r\n");
      uint32_t ocr; spi_RxBuffer((uint8_t *)&ocr, 4);
      DEBUG_PRINT(" CMD8 0x%x\r\n", ocr);
      // operation condition register voltage range 2.7-3.6V
      if (ocr == _OCR(0x00001AAU))
      {
        // ACMD41 with HCS bit can be up to 200ms wait
        do {
          if (SD_SendCmd(CMD55,                0) <= 1 && // APP_CMD Get Ok or idle state
              SD_SendCmd(ACMD41, SD_OCR_CAPACITY) == 0)   // Check OCR
            break;
          chThdSleepMilliseconds(10);
        } while (--cnt);
        DEBUG_PRINT(" CMD55 + ACMD41 %d\r\n", cnt);
        // READ_OCR
        if (cnt && SD_SendCmd(CMD58, 0) == 0)
        {
          DWORD ocr; spi_RxBuffer((uint8_t *)&ocr, 4);
          DEBUG_PRINT(" CMD58 OCR = 0x%08X\r\n", _OCR(ocr));
          // Check CCS bit, SDv2 (HC or SC)
          type = (ocr & _OCR(SD_OCR_CAPACITY)) ? CT_SD2 | CT_BLOCK : CT_SD2;
        }
      }
#ifdef SD_USE_COMMAND_CRC
#ifdef SD_USE_DATA_CRC
      SD_SendCmd(CMD59, 1); // Enable CRC check on card
#endif
#endif
//      uint8_t csd[16];
//      if (SD_SendCmd(CMD9, 0) == 0 && SD_RxDataBlock(csd, 16, SD_TOKEN_START_BLOCK)){
//        DEBUG_PRINT(" CSD =");
//        for (int i = 0; i<16; i++)
//          DEBUG_PRINT(" %02X", csd[i]);
//        DEBUG_PRINT("\r\n");
//      }
    } else {
      DEBUG_PRINT(" CMD8 Fail\r\n");
      // SDC V1 or MMC
      type = (SD_SendCmd(CMD55,  0) <= 1 &&                  // APP_CMD
              SD_SendCmd(ACMD41, 0) <= 1) ? CT_SD1 : CT_MMC;
      DEBUG_PRINT(" CMD55 %d\r\n", type);
      do{
        if (type == CT_SD1)
        {
          if (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(ACMD41, 0) == 0) break; // ACMD41
        }
        else if (SD_SendCmd(CMD1, 0) == 0) break; // CMD1
        chThdSleepMilliseconds(10);
      } while (--cnt);
      // SET_BLOCKLEN
      if (!cnt || SD_SendCmd(CMD16, SD_SECTOR_SIZE) != 0) type = 0;
      DEBUG_PRINT(" CMD16 %d %d\r\n", cnt, type);
    }
  }
  SD_Unselect_SPI();
  CardType = type;
  DEBUG_PRINT("CardType %d\r\n", type);
  // Clear STA_NOINIT and set Power on
  if (type){
    Stat&= ~STA_NOINIT;
    Stat|=  STA_POWER_ON;
  }
  else // Initialization failed
    SD_PowerOff();
  return Stat;
}

// diskio.c - Return disk status
DSTATUS disk_status(BYTE pdrv) {
  if (pdrv != 0) return STA_NOINIT;
  return Stat;
}

// diskio.c - Read sector
DRESULT disk_read(BYTE pdrv, BYTE* buff, DWORD sector, UINT count) {
  // No disk or wrong block count
  if (pdrv != 0 || count != 1 || (Stat & STA_NOINIT)) return RES_NOTRDY;
  // convert to byte address
  if (!(CardType & CT_BLOCK)) sector *= SD_SECTOR_SIZE;
//  r_cnt++;
//  r_time-= chVTGetSystemTimeX();
  SD_Select_SPI(SD_SPI_RX_SPEED);
  // READ_SINGLE_BLOCK
  uint8_t cnt = SD_READ_WRITE_REPEAT; // read repeat count
  do{
    if ((SD_SendCmd(CMD17, sector) == 0) && SD_RxDataBlock(buff, SD_SECTOR_SIZE, SD_TOKEN_START_BLOCK)){
      count = 0;
      break;
    }
  }while (--cnt);
  SD_Unselect_SPI();
//  r_time+= chVTGetSystemTimeX();
  if (count)
    DEBUG_PRINT(" err READ_BLOCK %d 0x%08X\r\n", count, sector);
#if 0
  else{
    DEBUG_PRINT("Sector read 0x%08X %d \r\n", sector, cnt);
    for (UINT j = 0; j < 32; j++){
      for (UINT i = 0; i < 16; i++)
        DEBUG_PRINT(" 0x%02x", buff[j*16 + i]);
      DEBUG_PRINT("\r\n");
    }
  }
#endif
  return count ? RES_ERROR : RES_OK;
}

// diskio.c - Write sector
DRESULT disk_write(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count) {
  // No disk or wrong count
  if (pdrv != 0 || count != 1 || (Stat & STA_NOINIT)) return RES_NOTRDY;
  // Write protection
  if (Stat & STA_PROTECT) return RES_WRPRT;
  // Convert to byte address if no Block mode
  if (!(CardType & CT_BLOCK)) sector*= SD_SECTOR_SIZE;
#if 0
    DEBUG_PRINT("Sector write 0x%08X, %d\r\n", sector, count);
    for (UINT j = 0; j < 32; j++){
      for (UINT i = 0; i < 16; i++)
        DEBUG_PRINT(" 0x%02X", buff[j*16 + i]);
      DEBUG_PRINT("\r\n");
    }
#endif
//  w_cnt++;
//  w_time-= chVTGetSystemTimeX();
  SD_Select_SPI(SD_SPI_SPEED);
  // WRITE_SINGLE_BLOCK
  uint8_t cnt = SD_READ_WRITE_REPEAT; // write repeat count
  do{
    if ((SD_SendCmd(CMD24, sector) == 0) && SD_TxDataBlock(buff, SD_TOKEN_START_BLOCK)){
      count = 0;
      break;
    }
  } while (--cnt);
  SD_Unselect_SPI();
//  w_time+= chVTGetSystemTimeX();
  if (count)
    DEBUG_PRINT(" WRITE_BLOCK %d 0x%08X\r\n", count, sector);
  return count ? RES_ERROR : RES_OK;
}

// The disk_ioctl function is called to control device specific features and miscellaneous functions other than generic read/write.
// Implement only five device independent commands used by FatFS module
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void* buff) {
  (void)buff;
  DRESULT res = RES_PARERR;
  // No disk or not ready
  if (pdrv != 0 || Stat & STA_NOINIT) return RES_NOTRDY;
  SD_Select_SPI(SD_SPI_RX_SPEED);
  switch (cmd){
    // Makes sure that the device has finished pending write process.
    // If the disk I/O layer or storage device has a write-back cache,
    // the dirty cache data must be committed to media immediately.
    // Nothing to do for this command if each write operation to the media is completed
    // within the disk_write function.
    case CTRL_SYNC:
      if (SD_WaitNotBusy(2000) == 0xFF) res = RES_OK;
    break;
#if FF_USE_TRIM == 1
    // Informs the device the data on the block of sectors is no longer needed and it can be erased.
    // The sector block is specified in an LBA_t array {<Start LBA>, <End LBA>} pointed by buff.
    // This is an identical command to Trim of ATA device. Nothing to do for this command if this function
    // is not supported or not a flash memory device. FatFs does not check the result code and the file function
    // is not affected even if the sector block was not erased well. This command is called on remove a cluster chain
    // and in the f_mkfs function. It is required when FF_USE_TRIM == 1.
    case CTRL_TRIM:
    break;
#endif
#if FF_MAX_SS > FF_MIN_SS
    // Retrieves sector size used for read/write function into the WORD variable pointed by buff.
    // Valid sector sizes are 512, 1024, 2048 and 4096. This command is required only if FF_MAX_SS > FF_MIN_SS.
    // When FF_MAX_SS == FF_MIN_SS, this command will be never used and the read/write function must work in FF_MAX_SS bytes/sector only.
    case GET_SECTOR_SIZE:
      *(uint16_t*) buff = SD_SECTOR_SIZE;
      res = RES_OK;
    break;
#endif
#if FF_USE_MKFS == 1
    // Retrieves erase block size of the flash memory media in unit of sector into the DWORD variable pointed by buff.
    // The allowable value is 1 to 32768 in power of 2. Return 1 if the erase block size is unknown or non flash memory media.
    // This command is used by only f_mkfs function and it attempts to align data area on the erase block boundary.
    // It is required when FF_USE_MKFS == 1.
    case GET_BLOCK_SIZE:
   	 *(uint16_t*) buff = ;//SD_SECTOR_SIZE;
      res = RES_OK;
    break;
    // Retrieves number of available sectors, the largest allowable LBA + 1, on the drive into the LBA_t variable pointed by buff.
    // This command is used by f_mkfs and f_fdisk function to determine the size of volume/partition to be created.
    // It is required when FF_USE_MKFS == 1.
    case GET_SECTOR_COUNT:
    {
      // SEND_CSD
      uint8_t csd[16];
      if ((SD_SendCmd(CMD9, 0) == 0) && SD_RxDataBlock(csd, 16, SD_TOKEN_START_BLOCK)) {
        uint32_t n, csize;
        if ((csd[0] >> 6) == 1)  {  // SDC V2
          csize = ((uint32_t)csd[7]<<16)|((uint32_t)csd[8]<< 8)|((uint32_t)csd[9]<< 0);
          n = 10;
        }
        else {                      // MMC or SDC V1
          csize = ((uint32_t)csd[8]>>6)|((uint32_t)csd[7]<<2)|((uint32_t)(csd[6]&0x03)<<10);
          n = ((csd[5]&0x0F)|((csd[10]&0x80)>>7)|((csd[9]&0x03)<<1)) + 2 - 9;
        }
        *(uint32_t*)buff = (csize+1)<<n;
        res = RES_OK;
      }
    }
    break;
#endif
  }
  SD_Unselect_SPI();
  DEBUG_PRINT("disk_ioctl(%d) = %d,\r\n", cmd, res);
  return res;
}
#endif //__USE_SD_CARD__
