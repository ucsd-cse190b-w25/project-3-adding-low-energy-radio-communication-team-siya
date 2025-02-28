/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "timer.h"


void timer_init(TIM_TypeDef *timer) {
    // Enable clock for TIM2
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

	// Step 1:
    // Reset timer
	timer->CR1 &= ~TIM_CR1_CEN;     // Stop the timer and reset control register
    timer->SR = 0;	    			// Reset status register
    timer->CNT = 0;     			// Reset counter

    // Step 2:
    // Set auto-reload to max initially (will be configured in timer_set_ms)
    timer->ARR = 0xFFFFFFFF;

    // Step 3:
    // Enable interrupt for update event
    timer->DIER |= TIM_DIER_UIE;

    // Enable TIM2 interrupt in NVIC
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 1);

    // Step 4:
    // Prescaler for a 1 kHz clock (8 MHz / 8000)
    timer->PSC = 7999;  // Prescaler is off by 1 because it’s 0-based

    // Step 5:
    // Enable the timer
    timer->CR1 |= TIM_CR1_CEN;
}

void timer_reset(TIM_TypeDef *timer) {
    timer->CNT = 0;  // Reset counter to 0
}

void timer_set_ms(TIM_TypeDef *timer, uint16_t period_ms) {

	timer_reset(timer);

    // Set the auto-reload value for the desired period (1 kHz clock, so period = ms)
    timer->ARR = period_ms - 1;
}
