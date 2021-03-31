#include "mam.h"

#define RCC_CR        ((unsigned long *)(RCC_BASE))
#define RCC_CFGR      ((unsigned long *)(RCC_BASE + 0x08))
#define RCC_AHB1ENR   ((unsigned long *)(RCC_BASE + 0x30))
#define GPIOA_MODER ((unsigned long *)(GPIOA_BASE + 0x00))
#define GPIOA_OSPEEDR	((unsigned long *)(GPIOA_BASE + 0x08))
#define GPIOA_ODR 	((unsigned long *)(GPIOA_BASE + 0x14))
#define GPIOB_MODER ((unsigned long *)(GPIOB_BASE + 0x00))
#define GPIOB_OSPEEDR	((unsigned long *)(GPIOB_BASE + 0x08))
#define GPIOB_ODR 	((unsigned long *)(GPIOB_BASE + 0x14))

typedef struct
{
	unsigned long CTRL;      /* SYSTICK control and status register,       Address offset: 0x00 */
	unsigned long LOAD;      /* SYSTICK reload value register,             Address offset: 0x04 */
	unsigned long VAL;       /* SYSTICK current value register,            Address offset: 0x08 */
	unsigned long CALIB;     /* SYSTICK calibration value register,        Address offset: 0x0C */
} SYSTICK_type;

#define SYSTICK_BASE	0xE000E010
#define SYSTICK ((SYSTICK_type *) SYSTICK_BASE)

void initTIM2(void)
{
    /*
     * TIM2 connected to APB1, running at 16MHz
     */
    setbit(RCC->APB1ENR, 0);  /* enable Clock for TIM2 */
    TIM2->PSC = 16000;            /* prescaler - fill according your needs */
    TIM2->ARR = 1000;            /* autoreload - fill according your needs */
    setbit(TIM2->CR1, 0);     /* enable TIM2 */
    clearbit(TIM2->SR, 0);    /* status TIM2 */
}

static unsigned char chars[10] = {192, 249, 164, 176, 153, 146, 130, 248, 128, 144};
static unsigned char segms[4] = {1, 2, 4, 8};

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

void sendChar (char ch)  {
  // wait till transmit register is empty 7th bit of SR
  while (!getbit(USART2->SR, 7));
  USART2->DR = ch;
}

char receiveChar(char * a) {
   // read from DR while receive register is not empty - 5th bit of SR
   if (getbit(USART2->SR, 5)) {
	   *a = USART2->DR;
	   return 1;
   }
   return 0;
}

/*
 * USART2 init
 * USART2 is connected to APB1 bus
 * USART2 uses PA2 as TX and PA3 as RX
 */
void serialInit(void){
	setbit(RCC->APB1ENR, 17);	// Enable APB1 clock
	setbit(USART2->CR1, 13);	// Enable USART2
	setbit(USART2->CR1, 2);		// USART2 - enable receive
	setbit(USART2->CR1, 3);		// USART2 - enable transmit
	USART2->BRR = (104 << 4); 	// USART2 baudrate, clock/(16*baudrate) should work
/*
  * STM32F401 HSI clock - 16 MHz
  * baudrate: 9600
  */
}

void gpio_init (void)
{
	setbit(RCC->AHB1ENR, 0);	// Enable AHB1 clock
	setbit(GPIOA->MODER, 5);	// PA2 alternate function, p. 157 of reference manual
	setbit(GPIOA->MODER, 7);	// PA3 alternate function

	GPIOA->AFR[0] |= 0x0700;	// alternate function, PA2 will be USART2_TX
								// see p. 162 of reference manual
								// see p. 45 of STM32F401xE datasheet
	GPIOA->AFR[0] |= 0x7000;	// alternate function, PA3
}

void sendString (char * str, int length)
{
	for (int i = 0; i < length; i++)
	{
		sendChar(str[i]);
	}
	sendChar('\n');
	sendChar('\r');
}

int main(void)
{
	char mess[20] = { 0 };
	char a;
	int i = 0;

	setbit(*RCC_CR, 16);
//	setbit(*RCC_CFGR, 0);
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

	unsigned int x;

	initTIM2();
	gpio_init();
	serialInit();

	//1.Send single character from Nucleo to PC
	sendString("Task1: ", 7);
	sendString("s", 1);
	sendChar('\n');
	sendChar('\r');

	//2.Modify template to send string
	sendString("Task2: ", 7);
	sendString("Sentence sent from nucleo to computer.", 38);
	sendChar('\n');
	sendChar('\r');
	sendString("Task3: \n", 8);

	while (1) {
		//sprintf(mess, "a = %i", i++);
		//sendString(mess, 10);
		//for(volatile int j = 0; j < 500; j++);

//#ifdef DEBUG

		if (receiveChar(&a)) {
			if (a != '\r' && a != '\n')
				mess[i++] = a;
			else {
				if (getbit(TIM2->SR, 0)) {
					x = binary_to_bcd(mess[0] - '0');
					display(x >> 12 & 0x0f, x >> 8 & 0x0f, x >> 4 & 0x0f,
							x & 0x0f);
					sendString(mess, i);
					i = 0;
					clearbit(TIM2->SR, 0);
				}
			}
		}
//#endif
	}

}

//USART = Universal Serial Asynchronous Receiver-Transmitter
