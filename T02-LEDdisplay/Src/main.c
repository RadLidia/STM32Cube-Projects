#define setbit(reg,bit)        		((reg) |= (1U << (bit)))
#define clearbit(reg,bit)      		((reg) &= (~(1U << (bit))))
#define togglebit(reg, bit)         ((reg) ^= (1U << (bit)))
#define getbit(reg,bit)        		((reg & (1U << bit)) >> (bit))
#define	sendserial(log, reg, bit)   ((log) ? setbit(reg, bit) : clearbit(reg, bit))

// boundary addresses of the peripherals (page. 38)
#define PERIPH_BASE   0x40000000
// RCC base address pg 38
#define RCC_BASE      (PERIPH_BASE +0X23800)
#define RCC_CR        ((unsigned long *)(RCC_BASE))
#define RCC_CFGR      ((unsigned long *)(RCC_BASE + 0x08))
// RCC_APB1ENR offset pg 118
#define RCC_AHB1ENR   ((unsigned long *)(RCC_BASE + 0x30))

// GPIOA base address
#define GPIOA_BASE	(PERIPH_BASE + 0x20000)
// MODER address
#define GPIOA_MODER ((unsigned long *)(GPIOA_BASE + 0x00))

// OSPEEDR
#define GPIOA_OSPEEDR	((unsigned long *)(GPIOA_BASE + 0x08))

// ODR address - output data register
#define GPIOA_ODR 	((unsigned long *)(GPIOA_BASE + 0x14))

#define GPIOB_BASE	(GPIOA_BASE + 0x400)
// MODER address
#define GPIOB_MODER ((unsigned long *)(GPIOB_BASE + 0x00))
// OSPEEDR
#define GPIOB_OSPEEDR	((unsigned long *)(GPIOB_BASE + 0x08))
// ODR address
#define GPIOB_ODR 	((unsigned long *)(GPIOB_BASE + 0x14))

#define SYSTICK_BASE	0xE000E010

typedef struct
{
	unsigned long CTRL;      /* SYSTICK control and status register,       Address offset: 0x00 */
	unsigned long LOAD;      /* SYSTICK reload value register,             Address offset: 0x04 */
	unsigned long VAL;       /* SYSTICK current value register,            Address offset: 0x08 */
	unsigned long CALIB;     /* SYSTICK calibration value register,        Address offset: 0x0C */
} SYSTICK_type;

#define SYSTICK ((SYSTICK_type *) SYSTICK_BASE)

static unsigned char chars[10] = {192, 249, 164, 176, 153, 146, 130, 248, 128, 144};
static unsigned char segms[4] = {1, 2, 4, 8};
static unsigned int a = 0;

void digit(unsigned char data, unsigned char segment)
{
	//	GPIOA9 - serial data
	//	GPIOA8 - clock
	//	GPIOB5 - latch

	clearbit(*GPIOB_ODR, 5); //latch = 0

	for(int i = 7; i >= 0; --i) //select the pattern of digits
	{
		clearbit(*GPIOA_ODR, 8); //clk = 0
		sendserial((chars[data] >> i) & 1, *GPIOA_ODR, 9);
		setbit(*GPIOA_ODR, 8); //clk = 1
	}

	for(int i = 7; i >= 0; --i) //select which led to turn on
	{
		clearbit(*GPIOA_ODR, 8);
		sendserial((segms[segment] >> i) & 1, *GPIOA_ODR, 9);
		setbit(*GPIOA_ODR, 8);
	}

	setbit(*GPIOB_ODR, 5); //latch = 1
}

void display (unsigned char s0, unsigned char s1, unsigned char s2, unsigned char s3)
{
	//show s0 on position 0
	digit(s0, 0);
	digit(s1, 1);
	digit(s2, 2);
	digit(s3, 3);

}

int binary_to_bcd(unsigned int val)
{
	int s = 0;
	int bcd = 0;
	while(val > 0)
	{
		bcd += (val % 10) << s;
		s += 4;
		val /= 10;
	}
	return bcd;
}

void init_systick(unsigned long s, unsigned char fast, unsigned char en)
{
	// 0: AHB/8 -> (2 MHz)
	// 1: Processor clock (AHB) -> (16 MHz)
	SYSTICK->CTRL |= (fast << 2); // Currently set to run at 2 Mhz -> 5e-7s
	// Enable callback
	SYSTICK->CTRL |= (en << 1); // if argument 'en' is 1, then interrupt is enabled
	// Load the reload value
	SYSTICK->LOAD = s;
	// Set the current value to 0
	SYSTICK->VAL = 0;
	// Enable SysTick
	SYSTICK->CTRL |= (1 << 0);		// setbit(SYSTICK->CTRL, 1)
}

void SysTick_Handler(void) {
	a++;
}

int main() {

	setbit(*RCC_CR, 16);
	setbit(*RCC_CFGR, 0);
	setbit(*RCC_AHB1ENR, 0);
	setbit(*RCC_AHB1ENR, 1);
	setbit(*GPIOA_MODER, 16);
	setbit(*GPIOA_OSPEEDR, 16);
	setbit(*GPIOA_OSPEEDR, 17);
	setbit(*GPIOA_MODER, 18);
	setbit(*GPIOA_OSPEEDR, 18);
	setbit(*GPIOA_OSPEEDR, 19);
	setbit(*GPIOB_MODER, 10);
	setbit(*GPIOB_OSPEEDR, 10);
	setbit(*GPIOB_OSPEEDR, 11);

    init_systick(2000000, 0, 1);

    unsigned int x;

    while(1)
    {
    	x = binary_to_bcd(a);
    	display(x >> 12 & 0x0f, x >> 8 & 0x0f, x >> 4 & 0x0f, x & 0x0f);
    }
}
