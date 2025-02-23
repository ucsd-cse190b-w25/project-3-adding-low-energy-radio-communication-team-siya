
/*
 * lsm6dsl.c
 *
 *  Created on: Jan 24, 2025
 *      Author: Dionisio
 */

#include <lsm6dsl.h>
#include <i2c.h>
#include <stdio.h>
#include <stdint.h>


uint8_t write_to_reg(uint8_t address, uint8_t subaddress, uint8_t val){
	uint8_t data[2] = {subaddress, val};
	return i2c_transaction(address,0, data, 2);
}
uint8_t read_from_reg(uint8_t address, uint8_t *subaddress, uint8_t *val, uint8_t len){
	i2c_transaction(address, 0, subaddress, 1);
	return i2c_transaction(address, 1, val, len);
}
void lsm6dsl_init(){
	uint8_t err;
	/* Enable accelerometer*/
	err = write_to_reg(LSM6DSL_ADDR, CTRL1_XL, ODR_XL);
	if(err== 0){
		printf("Error with enabling accelerometer\n");
	}
	/*Enable auto-increment on accelerometer for i2c*/
	err = write_to_reg(LSM6DSL_ADDR, CTRL3_C, IF_INC);
	if(err == 0){
		printf("Error with auto-increment for accelerometer\n");
	}
}

void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z){
	uint8_t data[6];
	uint8_t reg = OUTX_L_XL;
	read_from_reg(LSM6DSL_ADDR, &reg, data, 6);
	//Getting value of x
	*x = (int16_t)(data[1] << 8 | data[0]);
	//Getting value of y
	*y = (int16_t)(data[3] << 8 | data[2]);
	//Getting value of z
	*z = (int16_t)(data[5] << 8 | data[4]);
}
