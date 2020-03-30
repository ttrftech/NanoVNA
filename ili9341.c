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
#include "hal.h"
#include "nanovna.h"

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

//
// Pin macros
//
#define RESET_ASSERT  palClearPad(GPIOA, 15)
#define RESET_NEGATE  palSetPad(GPIOA, 15)
#define CS_LOW        palClearPad(GPIOB, 6)
#define CS_HIGH       palSetPad(GPIOB, 6)
#define DC_CMD        palClearPad(GPIOB, 7)
#define DC_DATA       palSetPad(GPIOB, 7)

//*****************************************************************************
//********************************** SPI bus **********************************
//*****************************************************************************
// STM32 SPI transfer mode:
// in 8 bit mode:
// if you write *(uint8_t*)(&SPI1->DR)  = (uint8_t) data, then data send as << data
// if you write *(uint16_t*)(&SPI1->DR) =(uint16_t) data, then data send as << dataLoByte, after send dataHiByte
// in 16 bit mode
// if you write *(uint16_t*)(&SPI1->DR) =(uint16_t) data, then data send as << data

// SPI init in 8 bit mode
#define SPI_CR2_8BIT  0x0700
#define SPI_CR2_16BIT 0x0F00

// SPI bus activity macros

// The RXNE flag is set depending on the FRXTH bit value in the SPIx_CR2 register:
// • If FRXTH is set, RXNE goes high and stays high until the RXFIFO level is greater or equal to 1/4 (8-bit).
#define SPI_RX_IS_NOT_EMPTY  (SPI1->SR&SPI_SR_RXNE)
#define SPI_RX_IS_EMPTY     (((SPI1->SR&SPI_SR_RXNE) == 0))

// The TXE flag is set when transmission TXFIFO has enough space to store data to send.
// 0: Tx buffer not empty, bit is cleared automatically when the TXFIFO level becomes greater than 1/2
// 1: Tx buffer empty, flag goes high and stays high until the TXFIFO level is lower or equal to 1/2 of the FIFO depth
#define SPI_TX_IS_NOT_EMPTY  (((SPI1->SR&(SPI_SR_TXE)) == 0))
#define SPI_TX_IS_EMPTY     (SPI1->SR&SPI_SR_TXE)

// When BSY is set, it indicates that a data transfer is in progress on the SPI (the SPI bus is busy).
#define SPI_IS_BUSY     (SPI1->SR & SPI_SR_BSY)

// SPI send data macros
#define SPI_WRITE_8BIT(data)  *(__IO uint8_t*)(&SPI1->DR) = (uint8_t) data
#define SPI_WRITE_16BIT(data) SPI1->DR = data

// SPI read data macros
#define SPI_READ_DATA        SPI1->DR

#ifdef __USE_DISPLAY_DMA__
static const stm32_dma_stream_t *dmatx =
    STM32_DMA_STREAM(STM32_SPI_SPI1_TX_DMA_STREAM);
static uint32_t txdmamode =
    STM32_DMA_CR_CHSEL(SPI1_TX_DMA_CHANNEL)         // Select SPI1 Tx DMA
    | STM32_DMA_CR_PL(STM32_SPI_SPI1_DMA_PRIORITY)  // Set priority
    | STM32_DMA_CR_DIR_M2P                          // Memory to Spi
    | STM32_DMA_CR_DMEIE                            //
    | STM32_DMA_CR_TEIE;

static void spi_lld_serve_tx_interrupt(SPIDriver *spip, uint32_t flags)
{
  (void)spip;
  (void)flags;
}

static const stm32_dma_stream_t  *dmarx = STM32_DMA_STREAM(STM32_SPI_SPI1_RX_DMA_STREAM);
static uint32_t rxdmamode = STM32_DMA_CR_CHSEL(SPI1_RX_DMA_CHANNEL)
                        | STM32_DMA_CR_PL(STM32_SPI_SPI1_DMA_PRIORITY)
                        | STM32_DMA_CR_DIR_P2M
                        | STM32_DMA_CR_TCIE
                        | STM32_DMA_CR_DMEIE
                        | STM32_DMA_CR_TEIE;

static void spi_lld_serve_rx_interrupt(SPIDriver *spip, uint32_t flags)
{
  (void)spip;
  (void)flags;
}

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

static void spi_init(void)
{
  rccEnableSPI1(FALSE);
  SPI1->CR1 = 0;
  SPI1->CR1 = SPI_CR1_MSTR       // SPI is MASTER
            | SPI_CR1_SSM        // Software slave management (The external NSS pin is free for other application uses)
            | SPI_CR1_SSI;       // Internal slave select (This bit has an effect only when the SSM bit is set. Allow use NSS pin as I/O)
//          | SPI_CR1_BR_1;      // Baud rate control

  SPI1->CR2 = SPI_CR2_8BIT       // SPI data size, set to 8 bit
            | SPI_CR2_FRXTH;     // SPI_SR_RXNE generated every 8 bit data
//          | SPI_CR2_SSOE;      //

#ifdef __USE_DISPLAY_DMA__
  // Tx DMA init
  dmaStreamAllocate(dmatx, STM32_SPI_SPI1_IRQ_PRIORITY, (stm32_dmaisr_t)spi_lld_serve_tx_interrupt, NULL);
  dmaStreamSetPeripheral(dmatx, &SPI1->DR);
  // Rx DMA init
  dmaStreamAllocate(dmarx, STM32_SPI_SPI1_IRQ_PRIORITY, (stm32_dmaisr_t)spi_lld_serve_rx_interrupt, NULL);
  dmaStreamSetPeripheral(dmarx, &SPI1->DR);
  // Enable DMA on SPI
  SPI1->CR2|= SPI_CR2_TXDMAEN    // Tx DMA enable
           |  SPI_CR2_RXDMAEN;   // Rx DMA enable
#endif
  SPI1->CR1|= SPI_CR1_SPE;       //SPI enable
}

// Disable inline for this function
static void __attribute__ ((noinline)) send_command(uint8_t cmd, uint8_t len, const uint8_t *data)
{
  CS_LOW;
  //while (SPI_TX_IS_NOT_EMPTY);
  DC_CMD;
  SPI_WRITE_8BIT(cmd);
  // Need wait transfer complete and set data bit
  while (SPI_IS_BUSY)
    ;
  // Send command data (if need)
  DC_DATA;
  while (len-- > 0) {
    while (SPI_TX_IS_NOT_EMPTY)
      ;
    SPI_WRITE_8BIT(*data++);
  }
  //CS_HIGH;
}

static const uint8_t ili9341_init_seq[] = {
  // cmd, len, data...,
  // SW reset
  ILI9341_SOFTWARE_RESET, 0,
  // display off
  ILI9341_DISPLAY_OFF, 0,
  // Power control B
  ILI9341_POWERB, 3, 0x00, 0x83, 0x30,
  // Power on sequence control
  ILI9341_POWER_SEQ, 4, 0x64, 0x03, 0x12, 0x81,
  //ILI9341_POWER_SEQ, 4, 0x55, 0x01, 0x23, 0x01,
  // Driver timing control A
  ILI9341_DTCA, 3, 0x85, 0x01, 0x79,
  //ILI9341_DTCA, 3, 0x84, 0x11, 0x7a,
  // Power control A
  ILI9341_POWERA, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
  // Pump ratio control
  ILI9341_PUMP_RATIO_CONTROL, 1, 0x20,
  // Driver timing control B
  ILI9341_DTCB, 2, 0x00, 0x00,
  // POWER_CONTROL_1
  ILI9341_POWER_CONTROL_1, 1, 0x26,
  // POWER_CONTROL_2
  ILI9341_POWER_CONTROL_2, 1, 0x11,
  // VCOM_CONTROL_1
  ILI9341_VCOM_CONTROL_1, 2, 0x35, 0x3E,
  // VCOM_CONTROL_2
  ILI9341_VCOM_CONTROL_2, 1, 0xBE,
  // MEMORY_ACCESS_CONTROL
  //ILI9341_MEMORY_ACCESS_CONTROL, 1, 0x48, // portlait
  ILI9341_MEMORY_ACCESS_CONTROL, 1, DISPLAY_ROTATION_0, // landscape
  // COLMOD_PIXEL_FORMAT_SET : 16 bit pixel
  ILI9341_PIXEL_FORMAT_SET, 1, 0x55,
  // Frame Rate
  ILI9341_FRAME_RATE_CONTROL_1, 2, 0x00, 0x1B,
  // Gamma Function Disable
  ILI9341_3GAMMA_EN, 1, 0x08,
  // gamma set for curve 01/2/04/08
  ILI9341_GAMMA_SET, 1, 0x01,
  // positive gamma correction
//ILI9341_POSITIVE_GAMMA_CORRECTION, 15, 0x1F,  0x1A,  0x18,  0x0A,  0x0F,  0x06,  0x45,  0x87,  0x32,  0x0A,  0x07,  0x02,  0x07, 0x05,  0x00,
  // negativ gamma correction
//ILI9341_NEGATIVE_GAMMA_CORRECTION, 15, 0x00,  0x25,  0x27,  0x05,  0x10,  0x09,  0x3A,  0x78,  0x4D,  0x05,  0x18,  0x0D,  0x38, 0x3A,  0x1F,
  // Column Address Set
//ILI9341_COLUMN_ADDRESS_SET, 4, 0x00, 0x00, 0x01, 0x3f, // width 320
  // Page Address Set
//ILI9341_PAGE_ADDRESS_SET, 4, 0x00, 0x00, 0x00, 0xef,   // height 240
  // entry mode
  ILI9341_ENTRY_MODE_SET, 1, 0x06,
  // display function control
  ILI9341_DISPLAY_FUNCTION_CONTROL, 4, 0x0A, 0x82, 0x27, 0x00,
  // Interface Control (set WEMODE=0)
  ILI9341_INTERFACE_CONTROL, 3, 0x00, 0x00, 0x00,
  // control display
  //ILI9341_WRITE_CTRL_DISPLAY, 1, 0x0c,
  // diaplay brightness
  //ILI9341_WRITE_BRIGHTNESS, 1, 0xff,
  // sleep out
  ILI9341_SLEEP_OUT, 0,
  // display on
  ILI9341_DISPLAY_ON, 0,
  0 // sentinel
};

void ili9341_init(void)
{
  spi_init();
  DC_DATA;
  RESET_ASSERT;
  chThdSleepMilliseconds(10);
  RESET_NEGATE;
  const uint8_t *p;
  for (p = ili9341_init_seq; *p; ) {
    send_command(p[0], p[1], &p[2]);
    p += 2 + p[1];
    chThdSleepMilliseconds(5);
  }
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
  while (len-- > 0) {
    while (SPI_TX_IS_NOT_EMPTY)
      ;
  SPI_WRITE_16BIT(color);
  }
}

void ili9341_bulk(int x, int y, int w, int h)
{
  // uint8_t xx[4] = { x >> 8, x, (x+w-1) >> 8, (x+w-1) };
  // uint8_t yy[4] = { y >> 8, y, (y+h-1) >> 8, (y+h-1) };
  uint16_t *buf = spi_buffer;
  uint32_t xx = __REV16(x | ((x + w - 1) << 16));
  uint32_t yy = __REV16(y | ((y + h - 1) << 16));
  send_command(ILI9341_COLUMN_ADDRESS_SET, 4, (uint8_t *)&xx);
  send_command(ILI9341_PAGE_ADDRESS_SET, 4, (uint8_t*)&yy);
  send_command(ILI9341_MEMORY_WRITE, 0, NULL);
  int32_t len = w * h;
  while (len-- > 0) {
    while (SPI_TX_IS_NOT_EMPTY)
      ;
    SPI_WRITE_16BIT(*buf++);
  }
}

static uint8_t ssp_sendrecvdata(void)
{
  // Start RX clock (by sending data)
  SPI_WRITE_8BIT(0);
  while (SPI_RX_IS_EMPTY && SPI_IS_BUSY)
    ;
  return SPI_READ_DATA;
}

void ili9341_read_memory(int x, int y, int w, int h, int len, uint16_t *out)
{
  // uint8_t xx[4] = { x >> 8, x, (x+w-1) >> 8, (x+w-1) };
  // uint8_t yy[4] = { y >> 8, y, (y+h-1) >> 8, (y+h-1) };
  uint32_t xx = __REV16(x | ((x + w - 1) << 16));
  uint32_t yy = __REV16(y | ((y + h - 1) << 16));
  send_command(ILI9341_COLUMN_ADDRESS_SET, 4, (uint8_t *)&xx);
  send_command(ILI9341_PAGE_ADDRESS_SET, 4, (uint8_t*)&yy);
  send_command(ILI9341_MEMORY_READ, 0, NULL);

  // Skip data from rx buffer
  while (SPI_RX_IS_NOT_EMPTY)
    (void) SPI_READ_DATA;
  // require 8bit dummy clock
  ssp_sendrecvdata();
  while (len-- > 0) {
    // read data is always 18bit
    uint8_t r = ssp_sendrecvdata();
    uint8_t g = ssp_sendrecvdata();
    uint8_t b = ssp_sendrecvdata();
    *out++ = RGB565(r, g, b);
  }
  CS_HIGH;
}
#else
//
// Use DMA for send data
//

// Fill region by some color
void ili9341_fill(int x, int y, int w, int h, uint16_t color)
{
  uint32_t xx = __REV16(x | ((x + w - 1) << 16));
  uint32_t yy = __REV16(y | ((y + h - 1) << 16));
  send_command(ILI9341_COLUMN_ADDRESS_SET, 4, (uint8_t *)&xx);
  send_command(ILI9341_PAGE_ADDRESS_SET, 4, (uint8_t *)&yy);
  send_command(ILI9341_MEMORY_WRITE, 0, NULL);

  dmaStreamSetMemory0(dmatx, &color);
  dmaStreamSetMode(dmatx, txdmamode | STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD);
  dmaStreamFlush(w * h);
}

void ili9341_bulk_8bit(int x, int y, int w, int h, uint16_t *palette)
{
  uint32_t xx = __REV16(x | ((x + w - 1) << 16));
  uint32_t yy = __REV16(y | ((y + h - 1) << 16));
  send_command(ILI9341_COLUMN_ADDRESS_SET, 4, (uint8_t *)&xx);
  send_command(ILI9341_PAGE_ADDRESS_SET, 4, (uint8_t *)&yy);
  send_command(ILI9341_MEMORY_WRITE, 0, NULL);

  uint8_t *buf = (uint8_t *)spi_buffer;
  int32_t len = w * h;
  while (len-- > 0) {
    uint16_t color = palette[*buf++];
    while (SPI_TX_IS_NOT_EMPTY)
      ;
    SPI_WRITE_16BIT(color);
  }
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

// Copy screen data to buffer
// Warning!!! buffer size must be greater then 3*len + 1 bytes
void ili9341_read_memory(int x, int y, int w, int h, int len, uint16_t *out)
{
  uint8_t dummy_tx = 0;
  uint8_t *rgbbuf = (uint8_t *)out;
  uint16_t data_size = len * 3 + 1;
  uint32_t xx = __REV16(x | ((x + w - 1) << 16));
  uint32_t yy = __REV16(y | ((y + h - 1) << 16));
  send_command(ILI9341_COLUMN_ADDRESS_SET, 4, (uint8_t *)&xx);
  send_command(ILI9341_PAGE_ADDRESS_SET, 4, (uint8_t *)&yy);
  send_command(ILI9341_MEMORY_READ, 0, NULL);
  // Skip SPI rx buffer
  while (SPI_RX_IS_NOT_EMPTY) (void)SPI_READ_DATA;
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
  dmaStreamEnable(dmatx);
  dmaStreamEnable(dmarx);
  // Wait DMA completion
  dmaWaitCompletion(dmatx);
  dmaWaitCompletion(dmarx);
  CS_HIGH;

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
  while (*str) {
    uint8_t ch = *str++;
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
    ili9341_fill(0, 0, 320, 240, 0);
    for (y = 0; y < 240; y++) {
      ili9341_fill(0, y, 320, 1, RGB(240-y, y, (y + 120) % 256));
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
