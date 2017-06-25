/**
 ******************************************************************************
 * @file    inc/common.h 
 * @author  Hemant Nile
 * @version V1
 * @date    04-Feb-2017
 * @brief   Frequently used Macros and inline functions
 
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMMON_H
#define __COMMON_H

#include "stm32f407xx.h"

//macros(same as in stm32f4xx.h)

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL))) 

//macros from stm32 HAL library

#define UNUSED(x) ((void)(x))

#define CLK_ENABLE(REGISTERWA, BITWA)     do { \
                                        __IO uint32_t tmpreg = 0x00U; \
                                        SET_BIT(REGISTERWA, BITWA);\
                                        tmpreg = READ_BIT(REGISTERWA, BITWA);\
                                        UNUSED(tmpreg); \
                                          } while(0)

#define CLK_DISABLE(REGISTERWA, BITWA)    (REGISTERWA &= ~(BITWA)) 

/* GPIO Pins manipulation   
 usage :	SetPin( GPIOA, PIN_1 | PIN_10 | PIN_12); sets pins 1,10,12 of GPIO A
 */
#define PIN_0         ((uint32_t)0x0001U) /* Pin 0 selected  */
#define PIN_1         ((uint32_t)0x0002U) /* Pin 1 selected  */
#define PIN_2         ((uint32_t)0x0004U) /* Pin 2 selected  */
#define PIN_3         ((uint32_t)0x0008U) /* Pin 3 selected  */
#define PIN_4         ((uint32_t)0x0010U) /* Pin 4 selected  */
#define PIN_5         ((uint32_t)0x0020U) /* Pin 5 selected  */
#define PIN_6         ((uint32_t)0x0040U) /* Pin 6 selected  */
#define PIN_7         ((uint32_t)0x0080U) /* Pin 7 selected  */
#define PIN_8         ((uint32_t)0x0100U) /* Pin 8 selected  */
#define PIN_9         ((uint32_t)0x0200U) /* Pin 9 selected  */
#define PIN_10        ((uint32_t)0x0400U) /* Pin 10 selected  */
#define PIN_11        ((uint32_t)0x0800U) /* Pin 11 selected  */
#define PIN_12        ((uint32_t)0x1000U) /* Pin 12 selected  */
#define PIN_13        ((uint32_t)0x2000U) /* Pin 13 selected  */
#define PIN_14        ((uint32_t)0x4000U) /* Pin 14 selected  */
#define PIN_15        ((uint32_t)0x8000U) /* Pin 15 selected  */
#define PIN_All       ((uint32_t)0xFFFFU) /* All pins selected */

__STATIC_INLINE void ResetPin(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
	SET_BIT(GPIOx->BSRR, Pin << 16);
}
__STATIC_INLINE void SetPin(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
	SET_BIT(GPIOx->BSRR, Pin);
}
__STATIC_INLINE void TogglePin(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
	GPIOx->ODR ^= Pin;
}

#endif /* __COMMON_H */
