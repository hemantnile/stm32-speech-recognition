/**
 ******************************************************************************
 * @file    Src/main.c 
 * @author  Hemant Nile
 * @version V1
 * @date    04-Feb-2017
 * @brief   Simple implementation of accessing LIS3DSH accelerometer on STM32F4 
 Discovery board using SPI interface. Four LEDs present on the board 
 lit up when board is tilted in their direction.
 
 ******************************************************************************
 */

#include "main.h"

static void Configure_LEDS(void);
static void Configure_EXTI0(void);
static void Configure_I2S2(void);

void Audio_Play(void);
void Audio_Stop(void);
void Audio_Pause(void);
void Audio_Resume(void);

uint16_t buffer[2][DECIMATION_FACTOR];
float PCM_buffer[SAMPLES];

uint32_t counter, delay, result;

int main(void) {
// Configure peripherals
	Configure_LEDS();
	Configure_I2S2();
	Configure_EXTI0();

//	Initiating 
	init_mfcc();		//hem
	Audio_Play();

// Infinite loop
	while (1) {
	}
}
static void Configure_EXTI0(void) {
	CLK_ENABLE(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
	MODIFY_REG(GPIOA->MODER, (uint32_t) 0x00000003U, (uint32_t) 0x00000000U);
	MODIFY_REG(GPIOA->OTYPER, (uint32_t) 0x00000001U, (uint32_t) 0x00000000U);
	MODIFY_REG(GPIOA->OSPEEDR, (uint32_t) 0x00000003U, (uint32_t) 0x00000000U);
	MODIFY_REG(GPIOA->PUPDR, (uint32_t) 0x00000003U, (uint32_t) 0x00000000U);

	NVIC_SetPriority(EXTI0_IRQn, 2);
	NVIC_EnableIRQ(EXTI0_IRQn);

	// exti0 connection to 	PORT A
	CLK_ENABLE(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
	SYSCFG->EXTICR[0] = 0x00000000U;
	// rising edge
	SET_BIT(EXTI->RTSR, EXTI_RTSR_TR0);
	//unmask exti0
	SET_BIT(EXTI->IMR, EXTI_IMR_MR0);
}

static void Configure_I2S2(void) {
	CLK_ENABLE(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);
	CLK_ENABLE(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);
//PB10
	MODIFY_REG(GPIOB->MODER, (uint32_t) 0x00300000U, (uint32_t) 0x00200000U);
	MODIFY_REG(GPIOB->OTYPER, (uint32_t) 0x00000400U, (uint32_t) 0x00000000U);
	MODIFY_REG(GPIOB->OSPEEDR, (uint32_t) 0x00300000U, (uint32_t) 0x00300000U);	//0x00200000U
	MODIFY_REG(GPIOB->PUPDR, (uint32_t) 0x00300000U, (uint32_t) 0x00000000U);
	MODIFY_REG(GPIOB->AFR[1], (uint32_t) 0x00000F00U, (uint32_t) 0x00000500U);
//PC3		
	MODIFY_REG(GPIOC->MODER, (uint32_t) 0x000000C0U, (uint32_t) 0x00000080U);
	MODIFY_REG(GPIOC->OTYPER, (uint32_t) 0x00000008U, (uint32_t) 0x00000000U);
	MODIFY_REG(GPIOC->OSPEEDR, (uint32_t) 0x000000C0U, (uint32_t) 0x000000C0U);	//0x00000080U
	MODIFY_REG(GPIOC->PUPDR, (uint32_t) 0x000000C0U, (uint32_t) 0x00000000U);
	MODIFY_REG(GPIOC->AFR[0], (uint32_t) 0x0000F000U, (uint32_t) 0x00005000U);

	//PLL I2S config along with reg SPI2->I2SPR: 0x60003000U : N=192,R=6,I2SDIV=15,ODD=1; 0x50004000U: N=256,R=5,I2SDIV=25,ODD=0;
	MODIFY_REG(RCC->PLLI2SCFGR, RCC_PLLI2SCFGR_PLLI2SN | RCC_PLLI2SCFGR_PLLI2SR,
			0x50004000U);	//0x50004000U	0x10004000U
	SET_BIT(RCC->CR, RCC_CR_PLLI2SON);
	while (READ_BIT(RCC->CR, RCC_CR_PLLI2SRDY) != RCC_CR_PLLI2SRDY) {
	};
//DMA1 Stream 3 Channel 0 Init
	CLK_ENABLE(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN);

	WRITE_REG(DMA1_Stream3->CR, (uint32_t) 0x00062D00U);//DBM : 0x00062D00U		8dbm: 0x00060D00U	32dbm: 0x00064D00U	no DBM:0x00022C00U		DBM with MBURST: 0x01062D00U
	WRITE_REG(DMA1_Stream3->FCR, (uint32_t) 0x00000000U);//	0x00000007U		direct mode only :0x00000000U
	WRITE_REG(DMA1_Stream3->PAR, (uint32_t) & (SPI2->DR));// Peripheral address 0x4000380CU 

	NVIC_SetPriority(DMA1_Stream3_IRQn, 0);		//highest priority
	NVIC_EnableIRQ(DMA1_Stream3_IRQn);
//I2S2 init		
	CLK_ENABLE(RCC->APB1ENR, RCC_APB1ENR_SPI2EN);

	WRITE_REG(SPI2->I2SPR, (uint32_t) 0x0000010CU);	//1.024Mhz:	0x00000019U:I2SDIV=25,ODD=0;0x0000010FU: I2SDIV=15,ODD=1;
													//2.048Mhz:0x0000010CU:I2SDIV=12,ODD=1	;0x0000007DU:I2SDIV=125,ODD=0;
	WRITE_REG(SPI2->I2SCFGR, (uint32_t) 0x00000B28U);	//0x00000B28U

}

void Audio_Play(void) {
	counter = 0;
	while (READ_BIT(DMA1_Stream3->CR, DMA_SxCR_EN) != 0) {
	}
	SET_BIT(DMA1->LIFCR, (uint32_t) 0x0F400000U);// Reset flags for DMA1 Stream 7
	SET_BIT(DMA1_Stream3->CR, DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);//enable interrupts
	SET_BIT(DMA1_Stream3->FCR, DMA_SxFCR_FEIE);		//enable interrupts
	WRITE_REG(DMA1_Stream3->M0AR, (uint32_t) & buffer[0][0]);// MEMORY address
	WRITE_REG(DMA1_Stream3->M1AR, (uint32_t) & buffer[1][0]);// MEMORY address

	WRITE_REG(DMA1_Stream3->NDTR, (uint32_t) DECIMATION_FACTOR);//64:0x00000040U		128:0x00000080U
	SET_BIT(DMA1_Stream3->CR, DMA_SxCR_EN);		//DMA on	
	SET_BIT(SPI2->I2SCFGR, SPI_I2SCFGR_I2SE);	//I2S2 On

//500ms delay
	delay = 0;
	while (delay < 14000000) {
		delay++;
	}

	SET_BIT(SPI2->CR2, SPI_CR2_RXDMAEN);	//I2S2 DMA Buffer enable
}
void Audio_Stop(void) {
	CLEAR_BIT(SPI2->CR2, SPI_CR2_RXDMAEN);	//I2S2 DMA Buffer disable
	CLEAR_BIT(DMA1_Stream3->CR, DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);//disable interrupts
	CLEAR_BIT(DMA1_Stream3->FCR, DMA_SxFCR_FEIE);		//disable interrupts
	CLEAR_BIT(DMA1_Stream3->CR, DMA_SxCR_EN);			//disable DMA and wait
	while (READ_BIT(DMA1_Stream3->CR, DMA_SxCR_EN) != 0) {
	}
	SET_BIT(DMA1->LIFCR, (uint32_t) 0x0F400000U);// Reset flags for DMA1 Stream 3
	CLEAR_BIT(SPI2->I2SCFGR, SPI_I2SCFGR_I2SE);	//I2S2 Off
}
void Audio_Pause(void) {
	CLEAR_BIT(SPI2->CR2, SPI_CR2_RXDMAEN);	//I2S2 DMA Buffer disable
}
void Audio_Resume(void) {
	SET_BIT(SPI2->CR2, SPI_CR2_RXDMAEN);	//I2S2 DMA Buffer enable
	SET_BIT(SPI2->I2SCFGR, SPI_I2SCFGR_I2SE);	//I2S2 On
}

void DMA1_Rxc_Callback(void) {
	if (counter < SAMPLES) {
		if (READ_BIT(DMA1_Stream3->CR, DMA_SxCR_CT) == 0) {
			PDM_to_PCM(&buffer[1][0], &PCM_buffer[counter]);
			counter += 16;
		} else {
			PDM_to_PCM(&buffer[0][0], &PCM_buffer[counter]);
			counter += 16;
		}
		if (counter == 512) {
			cost_calc(PCM_buffer, &counter);
		}
		if (counter == 1024) {
			cost_calc(PCM_buffer, &counter);
		}
	} else {
		Audio_Pause();
		counter = 0;

		PCM_post_processing(PCM_buffer);

		calc_mfcc(PCM_buffer);		//hem
		result = result_mfcc();		//hem
		
		switch (result) {
		case PLAY:
			play();
			break;
		case STOP:
			stop();
			break;
		case RED:
			red();
			break;
		case BLUE:
			blue();
			break;
		case GREEN:
			green();
			break;
		case ORANGE:
			orange();
			break;
		default:
			break;
		}

		Audio_Resume();
	}
}
void play(void) {
	SetPin(GPIOD, PIN_14);
	SetPin(GPIOD, PIN_15);
	SetPin(GPIOD, PIN_12);
	SetPin(GPIOD, PIN_13);
}
void stop(void) {
	ResetPin(GPIOD, PIN_14);
	ResetPin(GPIOD, PIN_15);
	ResetPin(GPIOD, PIN_12);
	ResetPin(GPIOD, PIN_13);
}
void red(void) {
	SetPin(GPIOD, PIN_14);
	ResetPin(GPIOD, PIN_15);
	ResetPin(GPIOD, PIN_12);
	ResetPin(GPIOD, PIN_13);
}
void blue(void) {
	ResetPin(GPIOD, PIN_14);
	SetPin(GPIOD, PIN_15);
	ResetPin(GPIOD, PIN_12);
	ResetPin(GPIOD, PIN_13);
}
void green(void) {
	ResetPin(GPIOD, PIN_14);
	ResetPin(GPIOD, PIN_15);
	SetPin(GPIOD, PIN_12);
	ResetPin(GPIOD, PIN_13);
}
void orange(void) {
	ResetPin(GPIOD, PIN_14);
	ResetPin(GPIOD, PIN_15);
	ResetPin(GPIOD, PIN_12);
	SetPin(GPIOD, PIN_13);
}

void UserButton_Callback(void) {
//		Audio_Play();
}
/**  
 *   System Clock Configuration
 *   The system Clock is configured as follows in MAIN.H : 
 *
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 168000000
 *            HCLK(Hz)                       = 168000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 4
 *            APB2 Prescaler                 = 2
 *            HSE Frequency(Hz)              = 8000000
 *            PLL_M                          = 8
 *            PLL_N                          = 336
 *            PLL_P                          = 2
 *            PLL_Q                          = 7
 *            VDD(V)                         = 3.3
 *            Main regulator output voltage  = Scale1 mode
 *            Flash Latency(WS)              = 5
 */
void SystemInit(void) {
// FPU settings
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2)); /* set CP10 and CP11 Full Access */
#endif

// Configure the Vector Table location add offset address
#ifdef VECT_TAB_SRAM
	SCB->VTOR = SRAM_BASE; /* Vector Table Relocation in Internal SRAM */
#else
	SCB->VTOR = FLASH_BASE; /* Vector Table Relocation in Internal FLASH */
#endif

// Enable PWR CLK and set voltage scale 1
	CLK_ENABLE(RCC->APB1ENR, RCC_APB1ENR_PWREN);
	SET_BIT(PWR->CR, PWR_CR_VOS);

// Flash latency, prefech, instruction cache , data cache
	MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_5WS);
	SET_BIT(FLASH->ACR, FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN);

// Enable HSE and wait for it	
	SET_BIT(RCC->CR, RCC_CR_HSEON);
	while (READ_BIT(RCC->CR, RCC_CR_HSERDY) != RCC_CR_HSERDY) {
	}
	WRITE_REG(RCC->PLLCFGR,
			RCC_PLLCFGR_PLLSRC_HSE | PLL_M | PLL_N << 6
					| ((PLL_P >> 1U) - 1U) << 16 | PLL_Q << 24);
// Enable PLL and wait for it	
	SET_BIT(RCC->CR, RCC_CR_PLLON);
	while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) != RCC_CR_PLLRDY) {
	}
	MODIFY_REG(RCC->CFGR,
			RCC_CFGR_PPRE2 | RCC_CFGR_PPRE1 | RCC_CFGR_HPRE | RCC_CFGR_SW_PLL,
			APB2_PRE | APB1_PRE | AHB_PRE | RCC_CFGR_SW_PLL);
	while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
	}
// Disable HSI	
	CLEAR_BIT(RCC->CR, RCC_CR_HSION);
}

static void Configure_LEDS(void) {
	CLK_ENABLE(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);
	MODIFY_REG(GPIOD->MODER, (uint32_t) 0xFF000000U, (uint32_t) 0x55000000U); // pins 12,13,14,15 as output
	MODIFY_REG(GPIOD->OTYPER, (uint32_t) 0xFF000000U, (uint32_t) 0x00000000U); // GPIOD->OTYPER - PUSH PULL 
	MODIFY_REG(GPIOD->OSPEEDR, (uint32_t) 0xFF000000U, (uint32_t) 0xFF000000U); // pins 12,13,14,15 very high speed 
	MODIFY_REG(GPIOD->PUPDR, (uint32_t) 0xFF000000U, (uint32_t) 0x00000000U); // GPIOD->PUPDR - NO PULL
}

void Error_Handler(void) {
// User may add here some code to deal with this error
	while (1) {
	}
}
