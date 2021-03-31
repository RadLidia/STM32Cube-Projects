#ifndef MAM_H_
#define MAM_H_

#include <stdint.h>

#define	setbit(reg, bit)      		((reg) |= (1U << (bit)))
#define clearbit(reg, bit)    		((reg) &= (~(1U << (bit))))
#define togglebit(reg, bit)   		((reg) ^= (1U << (bit)))
#define getbit(reg, bit)      		(((reg) & (1U << (bit))) >> (bit))
#define sendserial(log, reg, bit) 	((log) ? setbit(reg, bit) : clearbit(reg, bit))

/**
  * @brief General Purpose I/O
  */

typedef struct
{
	uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
	uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
	uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
	uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
	uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
	uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
	uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
	uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
	uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;

/**
  * @brief System configuration controller
  */

typedef struct
{
	uint32_t MEMRMP;       /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
	uint32_t PMC;          /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
	uint32_t EXTICR[4];    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
	uint32_t RESERVED[2];  /*!< Reserved, 0x18-0x1C                                                          */
	uint32_t CMPCR;        /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
} SYSCFG_TypeDef;

/**
  * @brief Reset and Clock Control
  */

typedef struct
{
	uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
	uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
	uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
	uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
	uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
	uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
	uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
	uint32_t RESERVED0;     /*!< Reserved, 0x1C                                                                    */
	uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
	uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
	uint32_t RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
	uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
	uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
	uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
	uint32_t RESERVED2;     /*!< Reserved, 0x3C                                                                    */
	uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
	uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
	uint32_t RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
	uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
	uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
	uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
	uint32_t RESERVED4;     /*!< Reserved, 0x5C                                                                    */
	uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
	uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
	uint32_t RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
	uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
	uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
	uint32_t RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
	uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
	uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
	uint32_t RESERVED7[1];  /*!< Reserved, 0x88                                                                    */
	uint32_t DCKCFGR;       /*!< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */
} RCC_TypeDef;


/**
  * @brief TIM
  */

typedef struct
{
	uint32_t CR1;         /*!< TIM control register 1,              Address offset: 0x00 */
	uint32_t CR2;         /*!< TIM control register 2,              Address offset: 0x04 */
	uint32_t SMCR;        /*!< TIM slave mode control register,     Address offset: 0x08 */
	uint32_t DIER;        /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
	uint32_t SR;          /*!< TIM status register,                 Address offset: 0x10 */
	uint32_t EGR;         /*!< TIM event generation register,       Address offset: 0x14 */
	uint32_t CCMR1;       /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
	uint32_t CCMR2;       /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
	uint32_t CCER;        /*!< TIM capture/compare enable register, Address offset: 0x20 */
	uint32_t CNT;         /*!< TIM counter register,                Address offset: 0x24 */
	uint32_t PSC;         /*!< TIM prescaler,                       Address offset: 0x28 */
	uint32_t ARR;         /*!< TIM auto-reload register,            Address offset: 0x2C */
	uint32_t RCR;         /*!< TIM repetition counter register,     Address offset: 0x30 */
	uint32_t CCR1;        /*!< TIM capture/compare register 1,      Address offset: 0x34 */
	uint32_t CCR2;        /*!< TIM capture/compare register 2,      Address offset: 0x38 */
	uint32_t CCR3;        /*!< TIM capture/compare register 3,      Address offset: 0x3C */
	uint32_t CCR4;        /*!< TIM capture/compare register 4,      Address offset: 0x40 */
	uint32_t BDTR;        /*!< TIM break and dead-time register,    Address offset: 0x44 */
	uint32_t DCR;         /*!< TIM DMA control register,            Address offset: 0x48 */
	uint32_t DMAR;        /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
	uint32_t OR;          /*!< TIM option register,                 Address offset: 0x50 */
} TIM_TypeDef;

/**
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */

typedef struct
{
	uint32_t SR;         /*!< USART Status register,                   Address offset: 0x00 */
	uint32_t DR;         /*!< USART Data register,                     Address offset: 0x04 */
	uint32_t BRR;        /*!< USART Baud rate register,                Address offset: 0x08 */
	uint32_t CR1;        /*!< USART Control register 1,                Address offset: 0x0C */
	uint32_t CR2;        /*!< USART Control register 2,                Address offset: 0x10 */
	uint32_t CR3;        /*!< USART Control register 3,                Address offset: 0x14 */
	uint32_t GTPR;       /*!< USART Guard time and prescaler register, Address offset: 0x18 */
} USART_TypeDef;

/** @addtogroup Peripheral_memory_map
  * @{
  */
#define FLASH_BASE            0x08000000UL /*!< FLASH(up to 1 MB) base address in the alias region                         */
#define SRAM1_BASE            0x20000000UL /*!< SRAM1(96 KB) base address in the alias region                              */
#define PERIPH_BASE           0x40000000UL /*!< Peripheral base address in the alias region                                */
#define SRAM1_BB_BASE         0x22000000UL /*!< SRAM1(96 KB) base address in the bit-band region                           */
#define PERIPH_BB_BASE        0x42000000UL /*!< Peripheral base address in the bit-band region                             */
#define BKPSRAM_BB_BASE       0x42480000UL /*!< Backup SRAM(4 KB) base address in the bit-band region                      */
#define FLASH_END             0x0807FFFFUL /*!< FLASH end address                                                          */
#define FLASH_OTP_BASE        0x1FFF7800UL /*!< Base address of : (up to 528 Bytes) embedded FLASH OTP Area                */
#define FLASH_OTP_END         0x1FFF7A0FUL /*!< End address of : (up to 528 Bytes) embedded FLASH OTP Area                 */

/* Legacy defines */
#define SRAM_BASE             SRAM1_BASE
#define SRAM_BB_BASE          SRAM1_BB_BASE

/*!< Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000UL)

/*!< APB1 peripherals */
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000UL)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400UL)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800UL)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00UL)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800UL)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00UL)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000UL)
#define I2S2ext_BASE          (APB1PERIPH_BASE + 0x3400UL)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800UL)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00UL)
#define I2S3ext_BASE          (APB1PERIPH_BASE + 0x4000UL)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400UL)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400UL)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800UL)
#define I2C3_BASE             (APB1PERIPH_BASE + 0x5C00UL)
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000UL)

/*!< APB2 peripherals */
#define TIM1_BASE             (APB2PERIPH_BASE + 0x0000UL)
#define USART1_BASE           (APB2PERIPH_BASE + 0x1000UL)
#define USART6_BASE           (APB2PERIPH_BASE + 0x1400UL)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2000UL)
#define ADC1_COMMON_BASE      (APB2PERIPH_BASE + 0x2300UL)
/* Legacy define */
#define ADC_BASE               ADC1_COMMON_BASE
#define SDIO_BASE             (APB2PERIPH_BASE + 0x2C00UL)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000UL)
#define SPI4_BASE             (APB2PERIPH_BASE + 0x3400UL)
#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x3800UL)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x3C00UL)
#define TIM9_BASE             (APB2PERIPH_BASE + 0x4000UL)
#define TIM10_BASE            (APB2PERIPH_BASE + 0x4400UL)
#define TIM11_BASE            (APB2PERIPH_BASE + 0x4800UL)

/*!< AHB1 peripherals */
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000UL)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00UL)
#define CRC_BASE              (AHB1PERIPH_BASE + 0x3000UL)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800UL)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x3C00UL)
#define DMA1_BASE             (AHB1PERIPH_BASE + 0x6000UL)
#define DMA1_Stream0_BASE     (DMA1_BASE + 0x010UL)
#define DMA1_Stream1_BASE     (DMA1_BASE + 0x028UL)
#define DMA1_Stream2_BASE     (DMA1_BASE + 0x040UL)
#define DMA1_Stream3_BASE     (DMA1_BASE + 0x058UL)
#define DMA1_Stream4_BASE     (DMA1_BASE + 0x070UL)
#define DMA1_Stream5_BASE     (DMA1_BASE + 0x088UL)
#define DMA1_Stream6_BASE     (DMA1_BASE + 0x0A0UL)
#define DMA1_Stream7_BASE     (DMA1_BASE + 0x0B8UL)
#define DMA2_BASE             (AHB1PERIPH_BASE + 0x6400UL)
#define DMA2_Stream0_BASE     (DMA2_BASE + 0x010UL)
#define DMA2_Stream1_BASE     (DMA2_BASE + 0x028UL)
#define DMA2_Stream2_BASE     (DMA2_BASE + 0x040UL)
#define DMA2_Stream3_BASE     (DMA2_BASE + 0x058UL)
#define DMA2_Stream4_BASE     (DMA2_BASE + 0x070UL)
#define DMA2_Stream5_BASE     (DMA2_BASE + 0x088UL)
#define DMA2_Stream6_BASE     (DMA2_BASE + 0x0A0UL)
#define DMA2_Stream7_BASE     (DMA2_BASE + 0x0B8UL)


/*!< Debug MCU registers base address */
#define DBGMCU_BASE           0xE0042000UL
/*!< USB registers base address */
#define USB_OTG_FS_PERIPH_BASE               0x50000000UL

#define USB_OTG_GLOBAL_BASE                  0x000UL
#define USB_OTG_DEVICE_BASE                  0x800UL
#define USB_OTG_IN_ENDPOINT_BASE             0x900UL
#define USB_OTG_OUT_ENDPOINT_BASE            0xB00UL
#define USB_OTG_EP_REG_SIZE                  0x20UL
#define USB_OTG_HOST_BASE                    0x400UL
#define USB_OTG_HOST_PORT_BASE               0x440UL
#define USB_OTG_HOST_CHANNEL_BASE            0x500UL
#define USB_OTG_HOST_CHANNEL_SIZE            0x20UL
#define USB_OTG_PCGCCTL_BASE                 0xE00UL
#define USB_OTG_FIFO_BASE                    0x1000UL
#define USB_OTG_FIFO_SIZE                    0x1000UL

#define UID_BASE                     0x1FFF7A10UL           /*!< Unique device ID register base address */
#define FLASHSIZE_BASE               0x1FFF7A22UL           /*!< FLASH Size register base address       */
#define PACKAGE_BASE                 0x1FFF7BF0UL           /*!< Package size register base address     */
/**
  * @}
  */

/** @addtogroup Peripheral_declaration
  * @{
  */
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)
#define TIM5                ((TIM_TypeDef *) TIM5_BASE)
#define RTC                 ((RTC_TypeDef *) RTC_BASE)
#define WWDG                ((WWDG_TypeDef *) WWDG_BASE)
#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)
#define I2S2ext             ((SPI_TypeDef *) I2S2ext_BASE)
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)
#define SPI3                ((SPI_TypeDef *) SPI3_BASE)
#define I2S3ext             ((SPI_TypeDef *) I2S3ext_BASE)
#define USART2              ((USART_TypeDef *) USART2_BASE)
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#define I2C3                ((I2C_TypeDef *) I2C3_BASE)
#define PWR                 ((PWR_TypeDef *) PWR_BASE)
#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define USART1              ((USART_TypeDef *) USART1_BASE)
#define USART6              ((USART_TypeDef *) USART6_BASE)
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define ADC1_COMMON         ((ADC_Common_TypeDef *) ADC1_COMMON_BASE)
/* Legacy define */
#define ADC                  ADC1_COMMON
#define SDIO                ((SDIO_TypeDef *) SDIO_BASE)
#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define SPI4                ((SPI_TypeDef *) SPI4_BASE)
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define TIM9                ((TIM_TypeDef *) TIM9_BASE)
#define TIM10               ((TIM_TypeDef *) TIM10_BASE)
#define TIM11               ((TIM_TypeDef *) TIM11_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)
#define CRC                 ((CRC_TypeDef *) CRC_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
#define DMA1                ((DMA_TypeDef *) DMA1_BASE)
#define DMA1_Stream0        ((DMA_Stream_TypeDef *) DMA1_Stream0_BASE)
#define DMA1_Stream1        ((DMA_Stream_TypeDef *) DMA1_Stream1_BASE)
#define DMA1_Stream2        ((DMA_Stream_TypeDef *) DMA1_Stream2_BASE)
#define DMA1_Stream3        ((DMA_Stream_TypeDef *) DMA1_Stream3_BASE)
#define DMA1_Stream4        ((DMA_Stream_TypeDef *) DMA1_Stream4_BASE)
#define DMA1_Stream5        ((DMA_Stream_TypeDef *) DMA1_Stream5_BASE)
#define DMA1_Stream6        ((DMA_Stream_TypeDef *) DMA1_Stream6_BASE)
#define DMA1_Stream7        ((DMA_Stream_TypeDef *) DMA1_Stream7_BASE)
#define DMA2                ((DMA_TypeDef *) DMA2_BASE)
#define DMA2_Stream0        ((DMA_Stream_TypeDef *) DMA2_Stream0_BASE)
#define DMA2_Stream1        ((DMA_Stream_TypeDef *) DMA2_Stream1_BASE)
#define DMA2_Stream2        ((DMA_Stream_TypeDef *) DMA2_Stream2_BASE)
#define DMA2_Stream3        ((DMA_Stream_TypeDef *) DMA2_Stream3_BASE)
#define DMA2_Stream4        ((DMA_Stream_TypeDef *) DMA2_Stream4_BASE)
#define DMA2_Stream5        ((DMA_Stream_TypeDef *) DMA2_Stream5_BASE)
#define DMA2_Stream6        ((DMA_Stream_TypeDef *) DMA2_Stream6_BASE)
#define DMA2_Stream7        ((DMA_Stream_TypeDef *) DMA2_Stream7_BASE)
#define DBGMCU              ((DBGMCU_TypeDef *) DBGMCU_BASE)
#define USB_OTG_FS          ((USB_OTG_GlobalTypeDef *) USB_OTG_FS_PERIPH_BASE)

#endif /* MAM_H_ */
