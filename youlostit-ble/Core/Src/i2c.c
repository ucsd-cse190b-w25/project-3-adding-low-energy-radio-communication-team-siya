/*
 * i2c.c
 *
 *  Created on: Jan 24, 2025
 *      Author: Dionisio
 */

#include <i2c.h>

void i2c_init(){
	// Enable I2C Clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
	//Enable Clock for GPIO B
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	/* Configure PB10 and PB11 as an output by clearing all bits and setting the mode */
	GPIOB->MODER &= ~GPIO_MODER_MODE10;
	GPIOB->MODER |= GPIO_MODER_MODE10_1;
	GPIOB->MODER &= ~GPIO_MODER_MODE11;
	GPIOB->MODER |= GPIO_MODER_MODE11_1;

	/* Configure the GPIO with open drain mode */
	GPIOB->OTYPER |= GPIO_OTYPER_OT10;
	GPIOB->OTYPER |= GPIO_OTYPER_OT11;
	/*Configure the GPIO with high speed mode */
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED10_Msk;
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED11_Msk;
	/*Enable Pull up resistors*/
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD10_0;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD11_0;
	/*Set alternate function mapping*/
	GPIOB->AFR[1] |= GPIO_AFRH_AFSEL10_2;
	GPIOB->AFR[1] |= GPIO_AFRH_AFSEL11_2;

	/* I2C configuration*/
	// Hardware Reset
	I2C2->CR1 |= I2C_CR1_SWRST;
	I2C2->CR1 &= ~I2C_CR1_SWRST;
	//turn the peripheral off
	I2C2->CR1 &= ~I2C_CR1_PE;

	// Set BAUD RATE to ~20-21 kHz
	I2C2->TIMINGR = (7 << 28) |  // PRESC = 7
	                (2 << 24) |  // SCLDEL = 2
	                (1 << 20) |  // SDADEL = 1
	                (11 << 8) |  // SCLH = 11
	                (11 << 0);   // SCLL = 11


	// Enabling I2C2 peripheral
	I2C2->CR1 |= I2C_CR1_PE;
}

uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len) {
	while (I2C2->ISR & I2C_ISR_BUSY);
	I2C2->CR2 &= ~I2C_CR2_SADD;
	I2C2->CR2 &= ~I2C_CR2_RD_WRN;
	I2C2->CR2 &= ~I2C_CR2_NBYTES_Msk;
	if(dir){ //do a read transaction
		//configure CR2 register
		I2C2->CR2 |= (address << 1) | (1<< 10) | (len << 16);
		I2C2->CR2 |= I2C_CR2_START;


		for (uint8_t i = 0; i < len; i++) {

			while (!(I2C2->ISR & I2C_ISR_RXNE)){
				if (I2C2->ISR & I2C_ISR_NACKF) {
					I2C2->ICR |= I2C_ICR_NACKCF;
					I2C2->CR2 |= I2C_CR2_STOP;
					return 0; // NACK received
				}
			}
			data[i] = I2C2->RXDR;
		}


		//check if transfer is complete
		while (!(I2C2->ISR & I2C_ISR_TC));
		I2C2->CR2 |= I2C_CR2_STOP;
		return 1;

	} else { //do a write transaction
		// Set the address and direction
		I2C2->CR2 |= (address << 1) | (0<< 10) | (len << 16);
		I2C2->CR2 |= I2C_CR2_START;


		for (uint8_t i = 0; i < len; i++) {
			while (!(I2C2->ISR & I2C_ISR_TXIS)) {
				if (I2C2->ISR & I2C_ISR_NACKF) {//Check NACK flag
					I2C2->ICR |= I2C_ICR_NACKCF;
					I2C2->CR2 |= I2C_CR2_STOP;
					return 0; // NACK received
				}
			}

			I2C2->TXDR = data[i];
		}
		//check if transfer is complete
		while (!(I2C2->ISR & I2C_ISR_TC));
		I2C2->CR2 |= I2C_CR2_STOP;

		return 1;
	}

}


