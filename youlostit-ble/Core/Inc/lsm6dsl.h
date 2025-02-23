
/*
 * lsm6dsl.h
 *
 *  Created on: Feb 7, 2025
 *      Author: sierramyers
 */

#ifndef LSM6DSL_H_
#define LSM6DSL_H_

#include <stdint.h>

/* Include the type definitions for the timer peripheral */
#define LSM6DSL_ADDR    0x6A
#define CTRL1_XL 		0x10
#define ODR_XL 			0x60		// Output data rate acc.
#define OUTX_L_XL		0x28
#define CTRL3_C			0x12
#define IF_INC			0x04

uint8_t write_to_reg(uint8_t address, uint8_t subaddress, uint8_t val);
uint8_t read_from_reg(uint8_t address, uint8_t *subaddress, uint8_t *val, uint8_t len);
void lsm6dsl_init();
void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z);

#endif /* LSM6DSL_H_ */
