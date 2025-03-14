/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
//#include "ble_commands.h"
#include "ble.h"

#include <stdlib.h>

#include <string.h>

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
/* Include memory map of our MCU */
#include <stm32l475xx.h>

/* Include LED driver */
#include "leds.h"

/* Include timer driver */
#include "timer.h"

/* Include i2c */
#include "i2c.h"

/* Include accelerometer */
#include <lsm6dsl.h>

int dataAvailable = 0;

SPI_HandleTypeDef hspi3;

void SystemClock_Config(void);
void switch_to_100kHz();
void switch_to_8MHz();
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
#define THRESHOLD 1000000  // Min amount of change detected to be considered moving (set to 1,000,000)
#define TIME_LIMIT 60  // 1 minute (1s * 60 cycles)
#define LOST_MODE_DELAY 10 // 10 seconds (1s * 10 cycles)

volatile int cycles_still = 0;
volatile bool lost = false;
volatile int lost_mode_counter = 0;
volatile int16_t x, y, z;
volatile int16_t px, py, pz;
volatile uint8_t min_lost = 0;
volatile uint16_t transmission_data[2] = {0x99, 0x23D5};
volatile bool discoverable = true;

void update_transmission_data() {
    transmission_data[1] = (0x23D5 & 0xFF00) | (min_lost & 0x00FF);
}

void lost_mode() {
    if (lost_mode_counter >= LOST_MODE_DELAY) {
        char test_str[] = "SiyaTag missing for ";  // Ensure enough space for the message and number
        updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, sizeof(test_str) - 1, (unsigned char*)test_str);
        char secs_str[50];  // Ensure enough space for the message
        // Convert cycles still from 50ms cycles to seconds and subtract the minute it takes to get to lost mode
        int seconds = (cycles_still - TIME_LIMIT + 1);
        sprintf(secs_str, "%d secs", seconds);
        updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, strlen(secs_str), (unsigned char*)secs_str);
        lost_mode_counter = 0;
    }
}

void LPTIM1_IRQHandler() {
    if (LPTIM1->ISR & LPTIM_ISR_ARRM) {  // Check if Auto-Reload Match flag is set
        LPTIM1->ICR |= LPTIM_ICR_ARRMCF;  // Clear the ARR match interrupt flag
        lsm6dsl_read_xyz(&x, &y, &z);
        int vx = (x - px) * (x - px);
        int vy = (y - py) * (y - py);
        int vz = (z - pz) * (z - pz);

        if (vx < THRESHOLD && vy < THRESHOLD && vz < THRESHOLD) {
            cycles_still++;
            if (cycles_still >= TIME_LIMIT) {
                lost = true;
                lost_mode_counter++;
            }
        } else {
            cycles_still = 0;
            lost = false;
            lost_mode_counter = 0;
        }

        px = x;
        py = y;
        pz = z;
    }
}

void stop2_mode(void)
{
    // Is this sufficient enough for stop mode? What about setting lpms in 001 in pwr control register, what about setting sleepdeep bit?

    HAL_SuspendTick(); // Stop SysTick to reduce power consumption

    // Set the system to Stop 2 mode (Regulator OFF, Wait for Interrupt)
   // HAL_PWR_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI); //Maybe it is this one

    // System wakes up here after an interrupt

    // Restore system clock after wake-up
    SystemClock_Config();

    HAL_ResumeTick();
}



int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI3_Init();

    HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);
    ble_init();
    HAL_Delay(10);
    i2c_init();
    lsm6dsl_init();

    timer_init(LPTIM1);  // Using LPTIM1 now
    timer_set_ms(LPTIM1, 1000);  // Set timer to trigger every 1s

    x = 0;
    y = 0;
    z = 0;
    px = 0;
    py = 0;
    pz = 0;
    setDiscoverability(0);

    while (1) {
        if (discoverable && HAL_GPIO_ReadPin(BLE_INT_GPIO_Port, BLE_INT_Pin)) {
            catchBLE();
        }
        if (lost) {
        	if(!discoverable){
				setDiscoverability(1);
				discoverable = true;
        	}
        	lost_mode();
        } else {
        	if(discoverable){
                disconnectBLE();
                setDiscoverability(0);
                discoverable = false;
        	}
        }
       stop2_mode();

    }
}


/**
  * @brief System Clock Configuration
  * @attention This changes the System clock frequency, make sure you reflect that change in your timer
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  // This lines changes system clock frequency
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_LED1_GPIO_Port, GPIO_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port, BLE_CS_Pin, GPIO_PIN_SET);


  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BLE_INT_Pin */
  GPIO_InitStruct.Pin = BLE_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLE_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_LED1_Pin BLE_RESET_Pin */
  GPIO_InitStruct.Pin = GPIO_LED1_Pin|BLE_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_CS_Pin */
  GPIO_InitStruct.Pin = BLE_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BLE_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
