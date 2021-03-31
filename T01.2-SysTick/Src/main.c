/*
 * Task 0:
 * LED on Nucleo STM32F401RE board is connected to 5th pin of GPIOA
 * GPIOA is connected to AHB1 bus - see table on pp. 38
 * Follow steps to activate GPIOA:
 * - enable clock for GPIOA in register RCC_AHB1ENR (pp. 118)
 * - set the mode of GPIOA 5th pin as output in register GPIOA_MODER (pp. 158)
 * - set/clear 5th bit in GPIOA_ODR (pp. 160)
 *
 * Dictionary:
 * GPIO - general purpose input output
 * AHB1 - advanced high-perfomance bus
 * RCC  - reset and clock control
 * AHB1EN - AHB1 enable register
 * MODER - mode register
 * ODR - output data register
 *
 * Notes:
 * - address are in unsigned long format
 */

#include <stdio.h>
// useful macros to set/clear bit of the number at given address
#define setbit(reg,bit) ((reg) |= (1U << (bit)))
#define clearbit(reg,bit) ((reg) &= (~(1U << (bit))))
#define getbit(reg,bit) ((reg & (1U << bit)) >> bit)
#define togglebit(reg, bit) ((reg) ^= (1U << (bit)))

/*
 * GPIOA_IDR -> input data register of GPIOA (16b, here least significant byte)
 * | 0 | 0 | 0 | 1 | 0 | 0 | 1 | 0 |
 * mask          ^ bit position
 *   0   0   0   1   0   0   0   0     ~ 1U << bit  -> X
 *
 *  0: 00000000
 *  1: 00000001      X >> bit
 */

/*
 * RCC_AHB1ENR
 * | 0 | 0 | 0 | 0 | 0 | 0 | 1 | 0 |
 * least significant bit is for GPIOA
 * second least significant bit is of GPIOB
 * RCC_AHB1ENR = 0x01 -> it will send 2nd bit to 0
 *
 */

#define SYSTICK_BASE	0xE000E010

typedef struct
{
	unsigned long CTRL;      /* SYSTICK control and status register,       Address offset: 0x00 */
	unsigned long LOAD;      /* SYSTICK reload value register,             Address offset: 0x04 */
	unsigned long VAL;       /* SYSTICK current value register,            Address offset: 0x08 */
	unsigned long CALIB;     /* SYSTICK calibration value register,        Address offset: 0x0C */
} SYSTICK_type;

#define SYSTICK ((SYSTICK_type *) SYSTICK_BASE)

/*
 * addressing the registers
 * SYSTICK->CTRL ~ address of control and status register
 * SYSTICK->VAL  ~ current value register
 */

// boundary addresses of the peripherals (page. 38)
#define PERIPH_BASE     0x40000000
// stack is on the top of the (S)RAM
// adress of the SRAM base
#define SRAM_BASE       0x20000000
// F401 has 96kB of SRAM
#define SRAM_SIZE	1024*96
// top of the SRAM
#define SRAM_END	(SRAM_BASE + SRAM_SIZE)

// RCC base address (pp. 38)
#define RCC_BASE 	(PERIPH_BASE + 0x23800)
// RCC_APB1ENR offset (pp. 118)
#define RCC_APB1ENR	((unsigned long *)(RCC_BASE + 0x30))

// GPIOA base address
#define GPIOA_BASE	(PERIPH_BASE + 0x20000)
// MODER address
#define GPIOA_MODER ((unsigned long *)(GPIOA_BASE + 0x00))
// IDR address - input data register
#define GPIOA_IDR   ((unsigned long *)(GPIOA_BASE + 0x10))
// ODR address - output data register
#define GPIOA_ODR 	((unsigned long *)(GPIOA_BASE + 0x14))

#define GPIOB_BASE	(PERIPH_BASE + 0x20400)
// MODER address
#define GPIOB_MODER ((unsigned long *)(GPIOB_BASE + 0x00))
// IDR address - input data register
#define GPIOA_IDR   ((unsigned long *)(GPIOA_BASE + 0x10))
// ODR address
#define GPIOB_ODR 	((unsigned long *)(GPIOB_BASE + 0x14))

// function prototypes
int main(void);
void delay(unsigned long count);
void systick_handler(void);
void init_systick(unsigned long s, unsigned char en);
void delay_ms(volatile unsigned long s);

volatile unsigned long seconds = 0;

int t;

// reset vector table (pp. 40 of programming manual)
// Address        Description
// =======        ===========
// 0x0000 0000    Initial Stack Pointer (SP) value
// 0x0000 0004    Reset exception
// 0x0000 0008    NMI - Non Maskable Interrupt
// 0x0000 000C    Hard fault
// 0x0000 0010    Memory management fault
// 0x0000 0014    Bus fault
// 0x0000 0018    Usage fault
//
// minimal version -> stack pointer & reset exception (function main)
// this will place two unsigned long numbers to fixed memory location
unsigned long *vector_table[] __attribute__((section(".isr_vector"))) = {
    (unsigned long *)SRAM_END,   	// 0 initial stack pointer
    (unsigned long *)main,       	// 1 main as Reset_Handler
	0,								// 2 NMI
	0,								// 3 Hard Fault
	0,								// 4 Memory management fault
	0, 								// 5 Bus fault
	0,								// 6 Usage fault
	0,								// 7 Reserved
	0,								// 8 .
	0,								// 9 .
	0,								// 10 Reserved
	0,								// 11 SVCall
	0,								// 12 Reserved for debug
	0,								// 13 Reserved
	0,								// 14 PendSV
	(unsigned long *)systick_handler// 15 Systick
};

int main() {

	printf("init section\n");
    // enable GPIOA clock
    *RCC_APB1ENR = 0x3; // + GPIOA? 0x03
    setbit(*GPIOA_MODER, 10);
    // change mode of 6 - MODER[11:10] = 0x01
    setbit(*GPIOA_MODER, 12);
    setbit(*GPIOA_MODER, 14);
    //*RCC_APB1ENR = 0x2;
    setbit(*GPIOB_MODER, 12);

    // Initialize systick without interrupt
     init_systick(1000, 0);
    // Initialize systick with interrupt
    //  NOTE: comment out inside while loop
    //init_systick(2000000, 1);

    printf("loop begins\n");

    t = 500;
    while(1) {

    	if (getbit(*GPIOA_IDR, 4) == 0){
    		t = 200;
    	}
    	if (getbit(*GPIOA_IDR, 4) == 1){
    	    t = 500;
    	}

    	if (getbit(*GPIOA_IDR, 1) == 0){
    		//two by two snake
    		clearbit(*GPIOA_ODR, 5);
    		delay_ms(t);
    		clearbit(*GPIOA_ODR, 6);
    		delay_ms(t);
    		setbit(*GPIOA_ODR, 5);
    		clearbit(*GPIOA_ODR, 7);
    		delay_ms(t);
    		setbit(*GPIOA_ODR, 6);
    		clearbit(*GPIOB_ODR, 6);
    		delay_ms(t);
    		setbit(*GPIOA_ODR, 7);
    		delay_ms(t);
    		setbit(*GPIOB_ODR, 6);
    	}

    	if (getbit(*GPIOA_IDR, 1) == 1){
    		//normal snake
    		clearbit(*GPIOA_ODR, 5);
    		delay_ms(t);
    		clearbit(*GPIOA_ODR, 6);
    		delay_ms(t);
    		clearbit(*GPIOA_ODR, 7);
    		delay_ms(t);
    		clearbit(*GPIOB_ODR, 6);
          	delay_ms(t);
          	setbit(*GPIOA_ODR, 5);
           	delay_ms(t);
           	setbit(*GPIOA_ODR, 6);
            delay_ms(t);
            setbit(*GPIOA_ODR, 7);
         	delay_ms(t);
         	setbit(*GPIOB_ODR, 6);
          	delay_ms(t);
    	}
    	/*
    	 * if we like to know GPIOA 4 - middle button of the shield is connected there
    	 */
    }
}

// delay function - doing literally nothing
void delay(unsigned long count) {
    while(count--);
}

/*
 *
 * LED D1 - GPIOA 5
   LED D2 - GPIOA 6
   LED D3 - GPIOA 7
   LED D4 - GPIOB 6

   D1 - D3 - already configured, set/reset bits in ODR
   D4 - enable clock for GPIOB in RCC_APB1ENR
      - check address of GPIOB, resp. GPIOB_MODER a GPIOB_ODR
      - set/reset 6th bit of GPIOB_ODR
 */

void systick_handler(void) {
	printf("seconds = %lu\n", seconds++);
	togglebit(*GPIOA_ODR, 6);
}

void init_systick(unsigned long s, unsigned char en)
{
	// Systick - 24b counter - 8388608
	// Main clock source is running with HSI by default which is at 16 Mhz.
	// SysTick clock source can be set with CTRL register (Bit 2)
	// 0: AHB/8 -> (2 MHz)
	// 1: Processor clock (AHB) -> (16 MHz)
	SYSTICK->CTRL |= 0x00000; // Currently set to run at 2 Mhz -> 5e-7s
	// Enable callback
	SYSTICK->CTRL |= (en << 1); // if argument 'en' is 1, then interrupt is enabled
	// Load the reload value
	SYSTICK->LOAD = s;
	// Set the current value to 0
	SYSTICK->VAL = 0;
	// Enable SysTick
	SYSTICK->CTRL |= (1 << 0);		// setbit(SYSTICK->CTRL, 1)
}

/*
 * Millisecond delay function.
 *   volatile keyword is used so that compiler does not optimize it away
 * Polling method (If interrupt is not enabled)
 */
void delay_ms(volatile unsigned long s)
{
	for(s; s>0; s--){
		while(!(SYSTICK->CTRL & (1 << 16))); // Wait until COUNTFLAG is 1
	}
}

/*
 *
 *       value of timer
 * 	 v	 MAX ~ 2^23
 *   v
 *
 *       RELOAD value
 *
 *
 *   v   0       --> interrupt
 *
 *  --> clock
 *
 *  timer is enabled
 *  exception is enabled
 *
 */

