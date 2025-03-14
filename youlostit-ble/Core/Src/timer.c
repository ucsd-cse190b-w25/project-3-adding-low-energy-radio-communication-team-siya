#include "timer.h"

void timer_init(LPTIM_TypeDef *timer) {
    RCC->CIER |= RCC_CIER_LSIRDYIE;	 			// Enable LSI ready interrupt
    RCC->CSR |= RCC_CSR_LSION;					// Enable LSI oscillator
    while ((RCC->CSR & RCC_CSR_LSIRDY) == 0);

    RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;		// Enable clock for LPTIM1

    RCC->CCIPR &= ~RCC_CCIPR_LPTIM1SEL;			// Clear LPTIM1 clock source bits
    RCC->CCIPR |= RCC_CCIPR_LPTIM1SEL_0;		// Set LPTIM1SEL to 01 (LSI)

    timer->CR &= ~LPTIM_CR_ENABLE;				// Disable the LPTIM
    while(LPTIM1->CR & LPTIM_CR_ENABLE);

    timer->ICR = LPTIM_ICR_CMPMCF | LPTIM_ICR_ARRMCF | LPTIM_ICR_EXTTRIGCF | LPTIM_ICR_CMPOKCF | LPTIM_ICR_ARROKCF | LPTIM_ICR_DOWNCF;

    timer->CFGR = 0;	// Default prescaling

    timer->CNT = 0;		// Reset counter

    timer->IER |= LPTIM_IER_ARRMIE;				// Enable the interrupt for ARR match

    // Enable LPTIM1 interrupt in NVIC
    NVIC_SetPriority(LPTIM1_IRQn, 0);
    NVIC_EnableIRQ(LPTIM1_IRQn);

    timer->CR |= LPTIM_CR_ENABLE;  // Enable LPTIM1

    while(!(LPTIM1->CR & LPTIM_CR_ENABLE));

    LPTIM1->CR |= LPTIM_CR_CNTSTRT;
}

void timer_reset(LPTIM_TypeDef *timer) {
    timer->CNT = 0;  // Reset counter to 0
}

void timer_set_ms(LPTIM_TypeDef *timer, uint16_t period_ms) {
    timer_reset(timer);

    uint32_t arr_value = ((uint32_t)period_ms * 32);

    // Set the auto-reload value
    timer->ARR = (uint16_t)arr_value - 1;
}
