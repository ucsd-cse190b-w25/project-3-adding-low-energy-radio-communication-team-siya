/*
 * leds.c
 *
 *  Created on: Oct 3, 2023
 *      Author: schulman
 */


/* Include memory map of our MCU */
#include <stm32l475xx.h>

void leds_init()
{
	// We need to enable clocks for GPIO A and B
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	// We need to enable clocks for GPIO A and B
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
  /* Configure PA5 as an output by clearing all bits and setting the mode */
  GPIOA->MODER &= ~GPIO_MODER_MODE5;
  GPIOA->MODER |= GPIO_MODER_MODE5_0;

  /* Configure the GPIO output as push pull (transistor for high and low) */
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT5;

  /* Disable the internal pull-up and pull-down resistors */
  GPIOA->PUPDR &= GPIO_PUPDR_PUPD5;

  /* Configure the GPIO to use low speed mode */
  GPIOA->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED5_Pos);

  /* Turn off the LED */
  GPIOA->ODR &= ~GPIO_ODR_OD5;

  /* Configure PA14 as an output by clearing all bits and setting the mode */
  GPIOB->MODER &= ~GPIO_MODER_MODE14;
  GPIOB->MODER |= GPIO_MODER_MODE14_0;

  /* Configure the GPIO output as push pull (transistor for high and low) */
  GPIOB->OTYPER &= ~GPIO_OTYPER_OT14;

  /* Disable the internal pull-up and pull-down resistors */
  GPIOB->PUPDR &= GPIO_PUPDR_PUPD14;

  /* Configure the GPIO to use low speed mode */
  GPIOB->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED14_Pos);

  /* Turn off the LED */
  GPIOB->ODR &= ~GPIO_ODR_OD14;

  /* Configure PA14 as an output by clearing all bits and setting the mode */
  GPIOB->MODER &= ~GPIO_MODER_MODE14;
  GPIOB->MODER |= GPIO_MODER_MODE14_0;

  /* Configure the GPIO output as push pull (transistor for high and low) */
  GPIOB->OTYPER &= ~GPIO_OTYPER_OT14;

  /* Disable the internal pull-up and pull-down resistors */
  GPIOB->PUPDR &= GPIO_PUPDR_PUPD14;

  /* Configure the GPIO to use low speed mode */
  GPIOB->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED14_Pos);

  /* Turn off the LED */
  GPIOB->ODR &= ~GPIO_ODR_OD14;
}

void leds_set(uint8_t led)
{
	unsigned int pin14 = (led & 2);
	pin14 = pin14 >> 1;
	unsigned int pin5 = (led & 1);

	unsigned int currPin5 = (GPIOA->ODR & GPIO_ODR_OD5) >> 5;
	unsigned int currPin14 = (GPIOB->ODR & GPIO_ODR_OD14) >> 14;

	if(pin5 != currPin5){
		if(pin5){
			//when we need to turn in on
			GPIOA->ODR |= GPIO_ODR_OD5;
		} else{
			//when we need to turn it off
			GPIOA->ODR &= ~GPIO_ODR_OD5;
		}
	}
	if(pin14 != currPin14){
		if(pin14){
			//when we need to turn in on
			GPIOB->ODR |= GPIO_ODR_OD14;
		} else{
			//when we need to turn it off
			GPIOB->ODR &= ~GPIO_ODR_OD14;
		}
	}
}
