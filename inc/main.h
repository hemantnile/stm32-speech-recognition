/**
 ******************************************************************************
 * @file    inc/main.h 
 * @author  Hemant Nile
 * @version V1
 * @date    04-Feb-2017
 * @brief   Header for main.c module
 
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#include "common.h"
#include <string.h>
#include "dsp.h"

#define PLL_M				8
#define PLL_N				336
#define PLL_P				2
#define PLL_Q				7
#define AHB_PRE			RCC_CFGR_HPRE_DIV1
#define APB1_PRE		RCC_CFGR_PPRE1_DIV4	
#define APB2_PRE		RCC_CFGR_PPRE2_DIV2		

#define SYSTEM_CORE_CLOCK		168000000

void Error_Handler(void);
void DMA1_Rxc_Callback(void);
void UserButton_Callback(void);

void play(void);
void stop(void);
void red(void);
void blue(void);
void green(void);
void orange(void);

#define PLAY					((uint32_t)0x00)
#define STOP					((uint32_t)0x01)
#define RED						((uint32_t)0x02)
#define BLUE					((uint32_t)0x03)
#define GREEN					((uint32_t)0x04)
#define ORANGE				((uint32_t)0x05)


#endif /* __MAIN_H */
