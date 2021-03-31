#include "mam.h"

#define BIT8
#define LOSAMP

void initGPIO()
{
    RCC->AHB1ENR |= 0x01;

    /* configure PA0 as ADC_IN0 */
    GPIOA->MODER   |=  0x03;     /* analog mode */
    GPIOA->OSPEEDR |=  0x03;     /* high speed */
}

void initADC1()
{
    // initialize the HSI clock
    setbit(RCC->CR, 0);         // enable HSI
    while (!getbit(RCC->CR, 1));// wait until HSI stable

    // initialize the ADC
    setbit(RCC->APB2ENR, 8);     // enable ADC1 peripheral clock
    ADC->CCR = 0;                 // disable temperature sensor, ADC prescaler = HSI/2
    clearbit(ADC1->SQR3, 0);    // 1st conversion in regular sequence will be from channel0
    clearbit(ADC1->SQR3, 1);    // reset state - all conversions from channel0 (PA_0)
    clearbit(ADC1->SQR3, 2);    // this is just an example
    clearbit(ADC1->SQR3, 3);
    clearbit(ADC1->SQR3, 4);
#ifdef BIT12
    clearbit(ADC1->CR1, 24);     // 12-bit resolution (Tconv = 15 ADCCLK cycles)
    clearbit(ADC1->CR1, 25);    // reset state
#endif
#ifdef BIT10
    setbit(ADC1->CR1, 24);        // 10-bit resolution (Tconv = 13 ADCCLK cycles)
#endif
#ifdef BIT8
    setbit(ADC1->CR1, 25);         // 8-bit resolution (Tconv =  11 ADCCLK cycles)
#endif
    clearbit(ADC1->CR2, 11);     // right alignment, reset state
#ifdef LOSAMP
    clearbit(ADC1->SMPR2, 0);    // channel0 sample rate: 3 cycles
    clearbit(ADC1->SMPR2, 1);    // reset state
    clearbit(ADC1->SMPR2, 2);
#endif
#ifdef HISAMP
    setbit(ADC1->SMPR2, 0);     // channel0 sample rate: 480 cycles
    setbit(ADC1->SMPR2, 1);
    setbit(ADC1->SMPR2, 2);
#endif
    setbit(ADC1->CR2, 0); // enable the ADC
}

void delay(int x)
{
    volatile int i;
    for (i = 0; i < 1000*x; i++);
}

void serialInit(void){
	setbit(RCC->APB1ENR, 17);	// Enable APB1 clock
	setbit(USART2->CR1, 13);	// Enable USART2
	setbit(USART2->CR1, 2);		// USART2 - enable receive
	setbit(USART2->CR1, 3);		// USART2 - enable transmit
	USART2->BRR = (104 << 4); // USART2 baudrate, clock/(16*baudrate) should work
	setbit(RCC->AHB1ENR, 0);	// Enable AHB1 clock
	setbit(GPIOA->MODER, 5);// PA2 alternate function, p. 157 of reference manual
	setbit(GPIOA->MODER, 7);	// PA3 alternate function

	GPIOA->AFR[0] |= 0x0700;	// alternate function, PA2 will be USART2_TX
								// see p. 162 of reference manual
								// see p. 45 of STM32F401xE datasheet
	GPIOA->AFR[0] |= 0x7000;	// alternate function, PA3
}

void sendChar (char ch)  {
  // wait till transmit register is empty 7th bit of SR
  while (!getbit(USART2->SR, 7));
  USART2->DR = ch;
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
int main()
{
    float i = 0;

    initGPIO();
    serialInit();
    initADC1();

    //initTIM2();


    while(1)
    {
        // single conversion mode, sec. 11.3.4 in RM (p. 841)
        setbit(ADC1->CR2, 30);             // software ADC start
        while (!getbit(ADC1->SR, 1));     // wait until conversion end
        i = ADC1->DR;
        //delay(10);

        //value of i
        //
        //for BIT8
        //125/255 * V_REF ~ 1.6176
        //
        //for BIT12
        //2011/4096 * 3.3 ~ 1.6201
        //

		//Send measured data to PC via USART.

        //calculate voltage

        //ADC1->DR / (255|1024|4096) 8
        //V_REF = 3.3
		float volt = i / 255 * 3.3; // full value

		int v_100 = volt * 100;

		int second_decimal = v_100 % 10;
		int aux = v_100 / 10;
		int first_decimal = aux % 10;
		int voltage = aux / 10;

		//to char
		char digit_3 = second_decimal + '0';
		char digit_2 = first_decimal + '0';
		char digit_1 = voltage + '0';

		//46- ascii symbol of "."
		//86 - ascii symbol of "V"
		char arr[5] = {digit_1, 46, digit_2, digit_3, 86};
		sendString(arr, 5);
    }
}
