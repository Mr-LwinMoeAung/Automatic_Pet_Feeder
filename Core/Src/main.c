/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "strings.h"
#include "i2c-lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define stepsperrev 4096
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t startTime = 0;
uint32_t holdTime = 0;
uint8_t pirState = 0;
uint32_t currentTime = 0;
uint32_t waitTime = 0;
uint32_t tempTime = 0;
uint8_t count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Ultra Sonic Sensor
void delay(uint16_t time){
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while(__HAL_TIM_GET_COUNTER(&htim1)<time);
}

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;

#define TRIG_PIN GPIO_PIN_8
#define TRIG_PORT GPIOE

// Let's write the callback function

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
	}
}


void HCSR04_Read (void)
{
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}

//stepper motor
void stepper_delay (uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}

void stepper_set_rpm (int rpm)  // Set rpm--> max 13, min 1,,,  went to 14 rev/min
{
	delay(60000000/stepsperrev/rpm);
}

void stepper_half_drive (int step)
{
	switch (step){
	         case 0:
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);   // IN1
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);   // IN4
			  break;

		  case 1:
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);   // IN1
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);   // IN2
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);   // IN4
			  break;

	          case 2:
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);   // IN2
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);   // IN4
			  break;

		  case 3:
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);   // IN2
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);   // IN3
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);   // IN4
			  break;

		  case 4:
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);   // IN3
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);   // IN4
			  break;

		  case 5:
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);   // IN3
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);   // IN4
			  break;

		  case 6:
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);   // IN4
			  break;

		  case 7:
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);   // IN1
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);   // IN4
			  break;
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
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start(&htim2);
  lcd_init();
  lcd_put_cur(0,0);
  lcd_send_string("Hi Pet Lover!");
  HAL_Delay(1000);
  lcd_clear();
  lcd_put_cur(0,0);
  lcd_send_string("Start Detecting");
  HAL_Delay(1000);
  //lcd_send_string("Hello World");
  //HAL_Delay(1000);
  lcd_clear();
 // lcd_put_cur(1, 0);
  //lcd_send_string("From O2");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HCSR04_Read();
	  char msg[10];
	  sprintf(msg, "%d\r\n", Distance);
	  HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	  HAL_Delay(500);

	  if (Distance < 5) {
		  uint8_t currentState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);

		  if (currentState && !pirState) {
			// Motion detected, start the timer
				startTime = HAL_GetTick();
				pirState = 1;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // Turn on LED

		  }
		  else if (!currentState && pirState) {
				// Motion ended, calculate hold time
				holdTime = HAL_GetTick() - startTime;
				pirState = 0;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // Turn off LED
				lcd_put_cur(0,0);
				lcd_send_string("Motion Detected");
				HAL_Delay(500);
				lcd_clear();

				// Print hold time to UART
				char msg[50];
				sprintf(msg, "Hold Time: %.2f seconds\r\n", holdTime / 1000.0);
				HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
				//Get Current Time
				currentTime = HAL_GetTick();
				//Get Waiting Time for count
				waitTime = currentTime - tempTime;
				char currentMsg[50];
				sprintf(currentMsg, "Current Time: %.2f seconds\r\n", currentTime / 1000.0);
				HAL_UART_Transmit(&huart3, (uint8_t*)currentMsg, strlen(currentMsg), HAL_MAX_DELAY);
				char waitMsg[50];
				sprintf(waitMsg, "Wait Time: %.2f seconds\r\n", waitTime / 1000.0);
				HAL_UART_Transmit(&huart3, (uint8_t*)waitMsg, strlen(waitMsg), HAL_MAX_DELAY);
				tempTime = currentTime;
				// Check if the dog is near
			if (waitTime/1000.0 < 10){
				count += 1;
				lcd_put_cur(0,0);
				lcd_send_string("Motion Detected");
				HAL_Delay(500);
				lcd_clear();
			}
			// If the dog passes the machine, reset count
			else if(waitTime/1000.0 > 10){
				lcd_put_cur(0,0);
				lcd_send_string("No Motion");
				HAL_Delay(500);
				lcd_clear();
				count = 0;
			}
			// if the dog waits for 1.22sec(hold time) * count(3) dispense food
			if (count == 3){
				count = 0;
				HAL_UART_Transmit(&huart3, (uint8_t*)"Dispense Food", 13, HAL_MAX_DELAY);
				lcd_put_cur(0,0);
				lcd_send_string("Dispensing Food");
				//5V stepper motor
				  for (int step = 0; step < stepsperrev; step++) {
					          stepper_half_drive(step % 8);
					          stepper_set_rpm(13);  // Set the desired RPM
					      }
				  lcd_clear();
			}
		  }
	  }
	  else {
	      // No motion detected or distance >= 10
	      HAL_UART_Transmit(&huart3, (uint8_t*) "No Food\r\n", 9, HAL_MAX_DELAY);
	      lcd_put_cur(0,0);
	      lcd_send_string("No Food");
	      HAL_Delay(500);
	      lcd_clear();
	  }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
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
