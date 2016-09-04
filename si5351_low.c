#include "hal.h"
#include "si5351.h"

#define SI5351_I2C_ADDR   	(0x60<<1)

static void
rcc_gpio_init(void)
{
    // Reset AHB,APB1,APB2
    RCC->AHBRSTR |= 0xffffffff;
    RCC->AHBRSTR = 0;
    RCC->APB1RSTR |= 0xffffffff;
    RCC->APB1RSTR = 0;
    RCC->APB2RSTR |= 0xffffffff;
    RCC->APB2RSTR = 0;

    RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_I2C1EN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->CFGR3 |= RCC_CFGR3_I2C1SW_HSI;

    // STM32F072
    GPIOB->AFRH = 0x55550011;  // PB8,PB9 Alternate Function 1
    // STM32F303
    //GPIOB->AFRH = 0x55550044;  // PB8,PB9 Alternate Function 4
    GPIOB->OTYPER |= 0x0300;   // PB8,PB9 Open drain
    GPIOB->MODER |= 0x000A0000;//
    GPIOB->OSPEEDR |= 0x00050000;//
}

static void
i2c_init(I2C_TypeDef* i2c)
{
	// Disable the I2Cx peripheral
	i2c->CR1 &= ~I2C_CR1_PE;
	while (i2c->CR1 & I2C_CR1_PE);

    // 100kHz @ 8MHz
    i2c->TIMINGR = 0x10420F13;

	// Use 7-bit addresses
	i2c->CR2 &=~ I2C_CR2_ADD10;

	// Enable the analog filter
	i2c->CR1 &= ~I2C_CR1_ANFOFF;

	// Disable NOSTRETCH
	i2c->CR1 |= I2C_CR1_NOSTRETCH;

	// Enable I2Cx peripheral clock.
	// Select APB1 as clock source
	if (i2c == I2C1) {
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	} else if (i2c == I2C2) {
		RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	}

	// Enable the I2Cx peripheral
	i2c->CR1 |= I2C_CR1_PE;
}

static void
i2cSendByte(I2C_TypeDef* i2c, uint8_t addr, const uint8_t *buf, uint8_t len)
{
	i2c->CR2 = (I2C_CR2_SADD & addr) 	// Set the slave address
      | (I2C_CR2_NBYTES & (len << 16))	// Send one byte
      | I2C_CR2_START 					// Generate start condition
      | I2C_CR2_AUTOEND;				// Generate stop condition after sent

	// Send the data
    while (len-- > 0) {
      while (!(i2c->ISR & I2C_ISR_TXIS));
      i2c->TXDR = (I2C_TXDR_TXDATA & *buf++);
    }
}

// register addr, length, data, ...
const uint8_t si5351_configs_low[] = {
  2, SI5351_REG_3_OUTPUT_ENABLE_CONTROL, 0xff,
  4, SI5351_REG_16_CLK0_CONTROL, SI5351_CLK_POWERDOWN, SI5351_CLK_POWERDOWN, SI5351_CLK_POWERDOWN,
  2, SI5351_REG_183_CRYSTAL_LOAD, SI5351_CRYSTAL_LOAD_8PF,
  // setup PLL (26MHz * 32 = 832MHz : 32/2-2=14)
  9, SI5351_REG_26_PLL_A, /*P3*/0, 1, /*P1*/0, 14, 0, /*P3/P2*/0, 0, 0,
  // RESET PLL
  2, SI5351_REG_177_PLL_RESET, SI5351_PLL_RESET_A | SI5351_PLL_RESET_B,
  // setup multisynth (832MHz/8MHz=104,104/2-2=50)
  9, SI5351_REG_58_MULTISYNTH2, /*P3*/0, 1, /*P1*/0, 50, 0, /*P2|P3*/0, 0, 0,
  2, SI5351_REG_18_CLK2_CONTROL, SI5351_CLK_DRIVE_STRENGTH_2MA | SI5351_CLK_INPUT_MULTISYNTH_N | SI5351_CLK_INTEGER_MODE,
  2, SI5351_REG_3_OUTPUT_ENABLE_CONTROL, 0,
  0 // sentinel
};

void
si5351_init_bulk(void)
{
  const uint8_t *p = si5351_configs_low;
  while (*p) {
    uint8_t len = *p++;
    i2cSendByte(I2C1, SI5351_I2C_ADDR, p, len);
    p += len;
  }
}

void
si5351_setup(void)
{
  rcc_gpio_init();
  i2c_init(I2C1);
  si5351_init_bulk();
}
