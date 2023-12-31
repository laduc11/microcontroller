/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*Private function-----------------------------------------------------------*/
/*toggle port*/
void toggle_port(GPIO_TypeDef *port, uint16_t pin)
{
	HAL_GPIO_TogglePin(port, pin);
}

/*Public function------------------------------------------------------------*/
/*run traffic light counter*/
void run_traffic_light(TRAFFIC_LIGHT *light, STATE *light_state)
{
	  switch (*light_state)
	  {
	  case red:
		  if (light->red_time == 0)
		  {
			  toggle_port(light->LED_RED_Port, light->LED_RED_Pin);
			  *light_state = green;
			  light->green_time = GREEN;
			  toggle_port(light->LED_GREEN_Port, light->LED_GREEN_Pin);
		  }
		  else
		  {
			  light->red_time--;
			  break;
		  }

	  case green:
		  if (light->green_time == 0)
		  {
			  toggle_port(light->LED_GREEN_Port, light->LED_GREEN_Pin);
			  *light_state = yellow;
			  light->yellow_time = YELLOW;
			  toggle_port(light->LED_YELLOW_Port, light->LED_YELLOW_Pin);
		  }
		  else
		  {
			  light->green_time--;
			  break;
		  }
	  case yellow:
		  if (light->yellow_time == 0)
		  {
			  toggle_port(light->LED_YELLOW_Port, light->LED_YELLOW_Pin);
			  *light_state = red;
			  light->red_time = RED;
			  toggle_port(light->LED_RED_Port, light->LED_RED_Pin);
		  }
		  else
		  {
			  light->yellow_time--;
			  break;
		  }
	  default:
		  light->red_time--;
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  TRAFFIC_LIGHT v_light, h_light;
  STATE v_light_state = red, h_light_state = green;

  HAL_GPIO_WritePin(V_RED_GPIO_Port, V_RED_Pin, RESET);
  HAL_GPIO_WritePin(V_YELLOW_GPIO_Port, V_YELLOW_Pin, SET);
  HAL_GPIO_WritePin(V_GREEN_GPIO_Port, V_GREEN_Pin, SET);

  HAL_GPIO_WritePin(H_RED_GPIO_Port, H_RED_Pin, SET);
  HAL_GPIO_WritePin(H_YELLOW_GPIO_Port, H_YELLOW_Pin, SET);
  HAL_GPIO_WritePin(H_GREEN_GPIO_Port, H_GREEN_Pin, RESET);

  v_light.LED_RED_Port = V_RED_GPIO_Port;
  v_light.LED_YELLOW_Port = V_YELLOW_GPIO_Port;
  v_light.LED_GREEN_Port = V_GREEN_GPIO_Port;
  v_light.LED_RED_Pin = V_RED_Pin;
  v_light.LED_YELLOW_Pin = V_YELLOW_Pin;
  v_light.LED_GREEN_Pin = V_GREEN_Pin;
  v_light.red_time = RED;
  v_light.yellow_time = 0;
  v_light.green_time = 0;

  h_light.LED_RED_Port = H_RED_GPIO_Port;
  h_light.LED_YELLOW_Port = H_YELLOW_GPIO_Port;
  h_light.LED_GREEN_Port = H_GREEN_GPIO_Port;
  h_light.LED_RED_Pin = H_RED_Pin;
  h_light.LED_YELLOW_Pin = H_YELLOW_Pin;
  h_light.LED_GREEN_Pin = H_GREEN_Pin;
  h_light.red_time = 0;
  h_light.yellow_time = 0;
  h_light.green_time = GREEN;
  while (1)
  {
	  run_traffic_light(&v_light, &v_light_state);
	  run_traffic_light(&h_light, &h_light_state);

//	  toggle_port(h_light.LED_GREEN_Port, h_light.LED_GREEN_Pin);
//	  toggle_port(h_light.LED_RED_Port, h_light.LED_RED_Pin);
//	  toggle_port(h_light.LED_YELLOW_Port, h_light.LED_YELLOW_Pin);

//	  HAL_GPIO_TogglePin(H_GREEN_GPIO_Port, H_GREEN_Pin);
//	  HAL_GPIO_TogglePin(H_RED_GPIO_Port, H_RED_Pin);
//	  HAL_GPIO_TogglePin(H_YELLOW_GPIO_Port, H_YELLOW_Pin);

    /* USER CODE END WHILE */
	  HAL_Delay(1000);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, H_RED_Pin|H_YELLOW_Pin|H_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, V_RED_Pin|V_YELLOW_Pin|V_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : H_RED_Pin H_YELLOW_Pin H_GREEN_Pin */
  GPIO_InitStruct.Pin = H_RED_Pin|H_YELLOW_Pin|H_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : V_RED_Pin V_YELLOW_Pin V_GREEN_Pin */
  GPIO_InitStruct.Pin = V_RED_Pin|V_YELLOW_Pin|V_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
