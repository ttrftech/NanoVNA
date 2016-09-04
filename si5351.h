#define SI5351_PLL_A	0
#define SI5351_PLL_B	1

#define SI5351_MULTISYNTH_DIV_4  4
#define SI5351_MULTISYNTH_DIV_6  6
#define SI5351_MULTISYNTH_DIV_8  8
#define SI5351_R_DIV_1		(0<<4)
#define SI5351_R_DIV_2		(1<<4)
#define SI5351_R_DIV_4		(2<<4)
#define SI5351_R_DIV_8		(3<<4)
#define SI5351_R_DIV_16		(4<<4)
#define SI5351_R_DIV_32		(5<<4)
#define SI5351_R_DIV_64		(6<<4)
#define SI5351_R_DIV_128	(7<<4)
#define SI5351_DIVBY4 		(3<<2)

#define SI5351_REG_3_OUTPUT_ENABLE_CONTROL 3
#define SI5351_REG_16_CLK0_CONTROL	16
#define SI5351_REG_17_CLK1_CONTROL	17
#define SI5351_REG_18_CLK2_CONTROL	18
#define SI5351_REG_26_PLL_A 		26
#define SI5351_REG_34_PLL_B 		34
#define SI5351_REG_42_MULTISYNTH0	42
#define SI5351_REG_50_MULTISYNTH1	50
#define SI5351_REG_58_MULTISYNTH2	58

#define SI5351_CLK_POWERDOWN			    (1<<7)
#define SI5351_CLK_INTEGER_MODE		     	(1<<6)
#define SI5351_CLK_PLL_SELECT_B		     	(1<<5)
#define SI5351_CLK_INVERT				    (1<<4)

#define SI5351_CLK_INPUT_MASK			    (3<<2)
#define SI5351_CLK_INPUT_XTAL			    (0<<2)
#define SI5351_CLK_INPUT_CLKIN			    (1<<2)
#define SI5351_CLK_INPUT_MULTISYNTH_0_4 	(2<<2)
#define SI5351_CLK_INPUT_MULTISYNTH_N	 	(3<<2)

#define SI5351_CLK_DRIVE_STRENGTH_MASK		(3<<0)
#define SI5351_CLK_DRIVE_STRENGTH_2MA		(0<<0)
#define SI5351_CLK_DRIVE_STRENGTH_4MA		(1<<0)
#define SI5351_CLK_DRIVE_STRENGTH_6MA	 	(2<<0)
#define SI5351_CLK_DRIVE_STRENGTH_8MA		(3<<0)


#define SI5351_REG_177_PLL_RESET 	177
#define SI5351_PLL_RESET_B 			(1<<7)
#define SI5351_PLL_RESET_A 			(1<<5)

#define SI5351_REG_183_CRYSTAL_LOAD 183
#define SI5351_CRYSTAL_LOAD_6PF  	(1<<6)
#define SI5351_CRYSTAL_LOAD_8PF  	(2<<6)
#define SI5351_CRYSTAL_LOAD_10PF  	(3<<6)

#define SI5351_CRYSTAL_FREQ_25MHZ 	25000000

void si5351_init(void);

void si5351_setupPLL(uint8_t pll, /* SI5351_PLL_A or SI5351_PLL_B */
                     uint8_t     mult,
                     uint32_t    num,
                     uint32_t    denom);
void si5351_setupMultisynth(uint8_t     output,
                       uint8_t	   pllSource,
                       uint32_t    div,
                       uint32_t    num,
                       uint32_t    denom);

void si5351_set_frequency(int channel, int freq);
