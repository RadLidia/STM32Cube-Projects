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

// useful macros to set/clear bit of the nuber at given address
#define setbit(reg,bit) ((reg) |= (1U << (bit)))
#define clearbit(reg,bit) ((reg) &= (~(1U << (bit))))

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
// ODR address
#define GPIOA_ODR 	((unsigned long *)(GPIOA_BASE + 0x14))

#define GPIOB_BASE	(PERIPH_BASE + 0x20400)
// MODER address
#define GPIOB_MODER ((unsigned long *)(GPIOB_BASE + 0x00))
// ODR address
#define GPIOB_ODR 	((unsigned long *)(GPIOB_BASE + 0x14))

// function prototypes
int main(void);
void delay(unsigned long count);

// reset vector table (pp. 40 of programming manual
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
    (unsigned long *)SRAM_END,   // initial stack pointer
    (unsigned long *)main        // main as Reset_Handler
};

int main() {
    // enable GPIOA clock
    *RCC_APB1ENR = 0x3; // + GPIOA? 0x03
    // change mode of 5 - MODER[11:10] = 0x01
    setbit(*GPIOA_MODER, 10);
    // change mode of 6 - MODER[11:10] = 0x01
    setbit(*GPIOA_MODER, 12);
    setbit(*GPIOA_MODER, 14);
    //*RCC_APB1ENR = 0x2;
    setbit(*GPIOB_MODER, 12);


    while(1) {
    	clearbit(*GPIOA_ODR, 5);
    	delay(200000);
    	clearbit(*GPIOA_ODR, 6);
    	delay(200000);
    	clearbit(*GPIOA_ODR, 7);
    	delay(200000);
    	clearbit(*GPIOB_ODR, 6);
    	delay(200000);
    	setbit(*GPIOA_ODR, 5);
        delay(200000);
    	setbit(*GPIOA_ODR, 6);
    	delay(200000);
    	setbit(*GPIOA_ODR, 7);
    	delay(200000);
    	setbit(*GPIOB_ODR, 6);
        delay(200000);

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
