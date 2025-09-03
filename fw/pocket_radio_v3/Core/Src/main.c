/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADDR 0x22
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint16_t volume = 0x001E;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Print_UART(char *str);
void Power_Up(void);
void Get_Rev(void);
void Set_Property(uint16_t property, uint16_t value);
void Get_Property(void);
void Tune_Freq(uint16_t channel);
void Tune_Status(void);
void Seek_Start(uint8_t up);
void GPIO_Ctl(void);
void GPIO_Set(void);
uint8_t Read_Status(void);
void Wait_For_CTS(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == CHAN_M_Pin)
	{
		Seek_Start(0);   // Seek down
	}
	else if (GPIO_Pin == CHAN_P_Pin)
	{
		Seek_Start(1);   // Seek up
	}
	else if (GPIO_Pin == VOL_M_Pin)
	{
		volume -= 5;
		if (volume < 5)
		{
			volume = 5;
		}
		// RX_VOLUME 0x4000 allowed 0-63, set to volume
	    Set_Property(0x4000, volume);
	}
	else if (GPIO_Pin == VOL_P_Pin)
	{
		volume += 5;
		if (volume > 60)
		{
			volume = 60;
		}
		// RX_VOLUME 0x4000 allowed 0-63, set to volume
		Set_Property(0x4000, volume);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_WritePin(GPIOB, IND_Pin, GPIO_PIN_SET);
	  HAL_Delay(1000);

	  char msg[64];
	  sprintf(msg, "I'M ALIVEEEE\r\n");
	  Print_UART(msg);

	  Power_Up();
	  Get_Rev();

	  // RX_VOLUME 0x4000 allowed 0-63, set to 30 (0x001E)
	  Set_Property(0x4000, 0x001E);
	  HAL_Delay(100);

	  // FM_ANTENNA_INPUT 0x1107 0x0001 (LPI antenna input)
	  Set_Property(0x1107, 0x0001);
	  HAL_Delay(100);

	  Tune_Freq(10350);
	  Tune_Status();

	  while (1)
	  {
		  HAL_Delay(10000);
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10805D88;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIG1_Pin|DIG2_Pin|DIG4_Pin|DIG8_Pin
                          |IND_Pin|ADDR_SEL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : VOL_P_Pin CHAN_M_Pin CHAN_P_Pin VOL_M_Pin */
  GPIO_InitStruct.Pin = VOL_P_Pin|CHAN_M_Pin|CHAN_P_Pin|VOL_M_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DIG1_Pin DIG2_Pin DIG4_Pin DIG8_Pin
                           IND_Pin */
  GPIO_InitStruct.Pin = DIG1_Pin|DIG2_Pin|DIG4_Pin|DIG8_Pin
                          |IND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ADDR_SEL_Pin */
  GPIO_InitStruct.Pin = ADDR_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADDR_SEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPO1_Pin GPO2_Pin */
  GPIO_InitStruct.Pin = GPO1_Pin|GPO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Print_UART(char *str)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

/*
* CMD 0x01 (POWER_UP)
* ARG1 0x10
* 	CTSIEN 0 (disable interrupt)
* 	GPO2OEN 0 (disable GPO2) TODO: apparently Hi-Z draws current due to oscillations
* 	PATCH 0 (boot normally)
* 	XOSCEN 1 (use crystal oscillator)
* 	FUNC 0 (FM receive)
* ARG2 0x05 (00000101 analog audio output)
*/
void Power_Up(void)
{
  uint8_t cmd[3] = {0x01, 0x10, 0x05};  // CMD, ARG1, ARG2
  HAL_I2C_Master_Transmit(&hi2c1, ADDR, cmd, sizeof(cmd), HAL_MAX_DELAY);
  HAL_Delay(100);  // wait for CTS to go high hopefully
}

/*
* CMD 0x10 (GET_REV)
* Response: STATUS + 15 bytes
*/
void Get_Rev(void)
{
  uint8_t cmd = 0x10;
  uint8_t resp[16] = {0};

  HAL_I2C_Master_Transmit(&hi2c1, ADDR, &cmd, 1, HAL_MAX_DELAY);

  HAL_Delay(100); // Hopefully CTS set by now

  HAL_I2C_Master_Receive(&hi2c1, ADDR, resp, 16, HAL_MAX_DELAY);

  char msg[64];
  sprintf(msg, "STATUS: 0x%02X\r\n", resp[0]);
  Print_UART(msg);
  sprintf(msg, "Part Number: %d\r\n", resp[1]);
  Print_UART(msg);
  sprintf(msg, "FW Major: %d\r\n", resp[2]);
  Print_UART(msg);
  sprintf(msg, "FW Minor: %d\r\n", resp[3]);
  int patchID = (resp[4] << 8) | resp[5];
  sprintf(msg, "Patch ID: %d\r\n", patchID);
  Print_UART(msg);
  sprintf(msg, "Comp Major: %d\r\n", resp[6]);
  Print_UART(msg);
  sprintf(msg, "Comp Minor: %d\r\n", resp[7]);
  Print_UART(msg);
  sprintf(msg, "Chip Rev: %d\r\n", resp[8]);
  Print_UART(msg);
}

/*
* CMD 0x12 (SET_PROPERTY)
* ARG1 0x00
* ARG2 (property high byte)
* ARG3 (property low byte)
* ARG4 (value high byte)
* ARG5(value low byte)
*
* Properties:
* 	FM_ANTENNA_INPUT 0x1107 0x0001 (LPI antenna input)
* 	RX_VOLUME 0x4000 0-63 (dec)
*/
void Set_Property(uint16_t property, uint16_t value)
{
  uint8_t cmd[6] = {0x12, 0x00, (property >> 8) & 0xFF, property & 0xFF, (value >> 8) & 0xFF, value & 0xFF};
  HAL_I2C_Master_Transmit(&hi2c1, ADDR, cmd, sizeof(cmd), HAL_MAX_DELAY);
//  HAL_Delay(100);
}

void Get_Property(void)
{
  // TODO
}

/*
* CMD 0x20 (FM_TUNE_FREQ)
* ARG1 0x00
* 	FREEZE 0 (bit 1)
* 	FAST 0 (bit 0)
* ARG2 (channel high byte)
* ARG3 (channel low byte)
* ARG4 0x00 (set antenna tuning cap automatically)
*
* @param channel in 10kHz units from 64-108MHz
*/
void Tune_Freq(uint16_t channel)
{
  uint8_t FREQ_H = (channel >> 8) & 0xFF;
  uint8_t FREQ_L = channel & 0xFF;
  uint8_t cmd[5] = {0x20, 0x00, FREQ_H, FREQ_L, 0x00};
  HAL_I2C_Master_Transmit(&hi2c1, ADDR, cmd, sizeof(cmd), HAL_MAX_DELAY);
  HAL_Delay(100);
}

/*
* CMD 0x22
* ARG1 0x00
* 	CANCEL 0 (don't cancel seek) (bit 1)
* 	INTACK 0 (don't clear seek/tune interrupt status indicator) (bit 0)
*/
void Tune_Status(void)
{
  uint8_t cmd[2] = {0x22, 0x00};
  HAL_I2C_Master_Transmit(&hi2c1, ADDR, cmd, sizeof(cmd), HAL_MAX_DELAY);
  HAL_Delay(100);

  uint8_t resp[8] = {0};
  HAL_I2C_Master_Receive(&hi2c1, ADDR, resp, 8, HAL_MAX_DELAY);

  char msg[64];
  sprintf(msg, "RESP1: 0x%02X\r\n", resp[1]);
  Print_UART(msg);
  int freq = (resp[2] << 8) | resp[3];
  sprintf(msg, "Read frequency: %d\r\n", freq);
  Print_UART(msg);
  sprintf(msg, "RSSI: %d\r\n", resp[4]);
  Print_UART(msg);
  sprintf(msg, "SNR: %d\r\n", resp[5]);
  Print_UART(msg);
  sprintf(msg, "READANTCAP: %d\r\n", resp[7]);
  Print_UART(msg);
}

/*
 * CMD 0x21 (FM_SEEK_START)
 * ARG1 0x0C
 *	 SEEKUP 1 (bit 3)
 *	 WRAP 1 (bit 2)
 *
 * @param up: 1 to seek up, 0 to seek down
 */
void Seek_Start(uint8_t up)
{
    uint8_t arg1 = 0x04;
    if (up) {
        arg1 |= 0x08;
    }

	uint8_t cmd[2] = {0x21, arg1};
	HAL_I2C_Master_Transmit(&hi2c1, ADDR, cmd, sizeof(cmd), HAL_MAX_DELAY);

//	char msg[64];
//	sprintf(msg, "Seeking...\r\n");
//	Print_UART(msg);
}

void GPIO_Ctl(void)
{
  // TODO
}

void GPIO_Set(void)
{
  //TODO
}

uint8_t Read_Status(void)
{
  uint8_t status = 0;
  HAL_I2C_Master_Receive(&hi2c1, ADDR, &status, 1, HAL_MAX_DELAY);
  return status;
}

void Wait_For_CTS(void)
{
  uint8_t status;
  do {
	  status = Read_Status();
	  HAL_Delay(100);
  } while ((status & 0x80) == 0); // While CTS bit low

  // Print status
  char msg[32];
  sprintf(msg, "CTS high, STATUS = 0x%02X\r\n", status);
  Print_UART(msg);
}

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
