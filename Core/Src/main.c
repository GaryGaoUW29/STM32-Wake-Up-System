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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_74HC595_CHIPS 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const uint8_t data[NUM_74HC595_CHIPS] = { 0x00, 0x00, 0x00, 0x00 };

//Segment Definition (Common Anode)
const uint8_t segmentMap[10] = { 0xC0, // 0
		0xF9, // 1
		0xA4, // 2
		0xB0, // 3
		0x99, // 4
		0x92, // 5
		0x82, // 6
		0xF8, // 7
		0x80, // 8
		0x90  // 9
		};

//Inverted Segment Definition (Common Anode)
const uint8_t segmentMapInverted[10] = { 0x3F, // 0
		0x06, // 1
		0x5B, // 2
		0x4F, // 3
		0x66, // 4
		0x6D, // 5
		0x7D, // 6
		0x07, // 7
		0x7F, // 8
		0x6F  // 9
		};

// Define time structure
typedef struct {
	uint8_t hour;
	uint8_t minute;
} Time;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void PWM_Motor_Init(void);
void PWM_Motor_SetSpeed(uint16_t speed);
void checkButtons(Time *time);
void displayTime(Time *time);
void controlLED(uint8_t mode);
void communicateWithOtherSTM32(Time *time);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	// ... initialization code

	// Initialize PWM motor
	PWM_Motor_Init();

	// Initialize other peripherals

	Time currentTime = { 0, 0 }; // Initialize time

	while (1) {
		checkButtons(&currentTime); // Check button status
		displayTime(&currentTime);  // Display time
		controlLED(buttonMode);     // Control LED
		communicateWithOtherSTM32(&currentTime); // Communicate with other STM32
	}
	/* USER CODE END 3 */
}
/* USER CODE BEGIN sendData */
void sendData() {
	HAL_GPIO_WritePin(GPIO_PORT, RCLK_Pin, GPIO_PIN_RESET);
	for (int i = 0; i < NUM_74HC595_CHIPS; i++) {
		for (int j = 0; j < 8; j++) {
			HAL_GPIO_WritePin(GPIO_PORT, SRCLK_Pin, GPIO_PIN_RESET);

			uint8_t bitValue = (data[i] & (1 << (7 - j)));

			if (bitValue) {
				HAL_GPIO_WritePin(GPIO_PORT, DATA_Pin, GPIO_PIN_SET);
			} else {
				HAL_GPIO_WritePin(GPIO_PORT, DATA_Pin, GPIO_PIN_RESET);
			}
		}
	}
	HAL_GPIO_WritePin(GPIO_PORT, RCLK_Pin, GPIO_PIN_SET);
}
/* USER CODE END sendData */

/* USER CODE BEGIN displayTime */
void displayTime(uint8_t hours, uint8_t minutes) {

	data[0] = segmentMap[hours / 10]; //Tens of Hours
	data[1] = segmentMap[hours % 10]; //Digits of Hours

	//It may need to be inverted
	//bc the third Segment Displays needs to be placed upside down
	//to keep the dot up for simulating a colon HH:MM
	//data[2] = segmentMapInverted[minutes / 10];
	data[2] = segmentMap[minutes / 10]; //Tens of Minutes

	data[3] = segmentMap[minutes % 10]; //Digits of Minutes

	sendData();
}
/* USER CODE END displayTime */

// Check button status function
void checkButtons(Time *time) {
	if (HAL_GPIO_ReadPin(Button_Mode_GPIO_Port, Button_Mode_Pin)
			== GPIO_PIN_RESET) {
		// Enter edit mode
		buttonMode = 1;
		// ... other edit mode operations
	} else {
		buttonMode = 0;
	}

	if (buttonMode == 1) {
		if (HAL_GPIO_ReadPin(Button_Minute_GPIO_Port, Button_Minute_Pin)
				== GPIO_PIN_RESET) {
			time->minute = (time->minute + 1) % 60;
		} else if (HAL_GPIO_ReadPin(Button_Hour_GPIO_Port, Button_Hour_GPIO_Pin)
				== GPIO_PIN_RESET) {
			time->hour = (time->hour + 1) % 24;
		}
	}
}

void controlLED(uint8_t mode) {
	HAL_GPIO_WritePin(LED_Mode_GPIO_Port, LED_Mode_Pin,
			mode ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(DATA_GPIO_Port, DATA_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, RCLK_Pin | SRCLK_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : DATA_Pin */
	GPIO_InitStruct.Pin = DATA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DATA_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RCLK_Pin SRCLK_Pin */
	GPIO_InitStruct.Pin = RCLK_Pin | SRCLK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

// PWM motor initialization function
void PWM_Motor_Init(void) {
	/// Configure timer and GPIO for PWM output
	TIM_Base_InitTypeDef sConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	// Assuming using TIM2 channel 1, please modify according to your hardware
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 83; // Adjust prescaler based on your clock configuration
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 20000; // PWM period of 20ms
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse   = 1500; // Initial pulse width for 90 degrees
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

// Set PWM motor speed
void setServoAngle(uint16_t speed) {
// Map angle to pulse width
	uint16_t pulseWidth = map(angle, 0, 180, 500, 2500);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulseWidth);
}

// Map function for converting angle to pulse width
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min,
		uint16_t out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void communicateWithOtherSTM32(Time *time) {
	// Check if current time matches the set time
	if (time->hour == setHour && time->minute == setMinute) {
		// Prepare data to send (e.g., a simple flag)
		uint8_t wakeupSignal = 1;

		// Send the wake-up signal over UART
		HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, &wakeupSignal, 1,
				HAL_MAX_DELAY);

		// Check for transmission errors
		if (status != HAL_OK) {
			// Handle transmission error, e.g., log an error message
			Error_Handler();
		} else {
			// Transmission successful, you can add additional actions here
			printf("Wake-up signal sent successfully.\n");
		}
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
