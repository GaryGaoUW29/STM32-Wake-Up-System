/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for STM32 Board 2 (Motor Control)
  * @author         : Gary Gao (g44gao@uwaterloo.ca)
  * @brief          : Refactored to be event-driven.
  * : Waits for a UART signal from Board 1, then
  * : gradually opens the blinds using PWM,
  * : checking for a limit switch.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Define constants for PWM signal parameters
#define PWM_FREQUENCY 50       // 50 Hz for 20ms period
#define PWM_MIN_DUTY_CYCLE 5   // 0% open (e.g., 1ms pulse)
#define PWM_MAX_DUTY_CYCLE 10  // 100% open (e.g., 2ms pulse)

// Timer and Channel
#define SERVO_PWM_CHANNEL TIM_CHANNEL_1

// Magic Byte from Board 1
#define WAKEUP_SIGNAL 0xAA

// Limit Switch Pin
// !! Modify these pins based on your hardware connection !!
#define LIMIT_SWITCH_UPPER_Port GPIOA
#define LIMIT_SWITCH_UPPER_Pin  GPIO_PIN_5 // Example: PA5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2; // Timer handle for PWM
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile bool g_wakeup_signal_received = false; // Flag set by UART interrupt
uint8_t g_uart_rx_buffer[1];                   // 1-byte buffer for UART data
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void); // Added TIM2 Init prototype
void Servo_Init(void);
void Servo_Control_Percentage(float percentage);
void Gradual_Open_Blinds(void);
bool Check_Limit_Switch_Upper(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_TIM2_Init(); // Initialize Timer 2 for PWM
  /* USER CODE BEGIN 2 */
  Servo_Init(); // Start the PWM signal

  // Start listening for 1 byte of data via UART interrupt
  HAL_UART_Receive_IT(&huart2, g_uart_rx_buffer, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (g_wakeup_signal_received)
    {
      g_wakeup_signal_received = false; // Clear the flag
      Gradual_Open_Blinds();          // Run the motor sequence
    }

    // The MCU will idle here, consuming minimal power,
    // until the UART interrupt sets the flag.
    HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
 * @brief  UART Receive Complete Interrupt Callback
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    // Check if we received the correct "Magic Byte"
    if (g_uart_rx_buffer[0] == WAKEUP_SIGNAL)
    {
      g_wakeup_signal_received = true;
    }

    // Re-arm the UART interrupt to listen for the next byte
    HAL_UART_Receive_IT(&huart2, g_uart_rx_buffer, 1);
  }
}

/**
 * @brief  Gradually opens the blinds over a 10-minute period
 * @retval None
 */
void Gradual_Open_Blinds(void)
{
  // Simulate a 10-minute (600,000 ms) sunrise
  const int total_duration_ms = 10 * 60 * 1000;
  const int steps = 100; // Open in 100 small steps (0% to 100%)
  const int delay_per_step_ms = total_duration_ms / steps;

  for (int i = 0; i <= steps; i++)
  {
    // Safety Check: Stop if the upper limit switch is hit
    if (Check_Limit_Switch_Upper())
    {
      break; // Exit the loop
    }

    Servo_Control_Percentage((float)i); // i is the percentage (0 to 100)
    HAL_Delay(delay_per_step_ms);
  }

  // Optional: Stop PWM or set to a holding position
  // For now, we just hold the 100% position.
}

/**
 * @brief  Function to initialize the PWM signal
 * @retval None
 */
void Servo_Init()
{
  // Start PWM for the servo motor
  HAL_TIM_PWM_Start(&htim2, SERVO_PWM_CHANNEL);
}

/**
 * @brief  Function to control the servo motor to a specific percentage (0-100)
 * @param  percentage: 0.0f to 100.0f
 * @retval None
 */
void Servo_Control_Percentage(float percentage)
{
  if (percentage < 0.0f) percentage = 0.0f;
  if (percentage > 100.0f) percentage = 100.0f;

  // Map percentage (0-100) to duty cycle (e.g., 5-10)
  float duty_cycle_percent = PWM_MIN_DUTY_CYCLE + (percentage / 100.0f) * (PWM_MAX_DUTY_CYCLE - PWM_MIN_DUTY_CYCLE);

  // Calculate the raw compare value
  uint32_t autoreload = __HAL_TIM_GET_AUTORELOAD(&htim2);
  uint16_t compare_value = (uint16_t)((duty_cycle_percent / 100.0f) * (autoreload + 1));

  __HAL_TIM_SET_COMPARE(&htim2, SERVO_PWM_CHANNEL, compare_value);
}

/**
 * @brief  Checks the state of the upper limit switch
 * @retval bool: true if the switch is pressed, false otherwise
 */
bool Check_Limit_Switch_Upper(void)
{
  // Assuming the switch pulls to GND when pressed (Active LOW)
  // and is configured with an internal or external PULL_UP.
  return HAL_GPIO_ReadPin(LIMIT_SWITCH_UPPER_Port, LIMIT_SWITCH_UPPER_Pin) == GPIO_PIN_RESET;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84 - 1;  // Assuming 84MHz clock, (84M / 84) = 1MHz timer clock
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000 - 1;  // 1MHz / 20000 = 50 Hz (20ms period)
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;  // Initial pulse width (0% duty cycle)
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, SERVO_PWM_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2); // This will configure the PWM GPIO pin
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  // Note: Board 1 (TX) uses 9600 Baud
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600; // Must match Board 1's Baud Rate (9600)
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_RX; // Board 2 only needs to receive
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LIMIT_SWITCH_UPPER_Pin */
  // This configures the example pin PA5
  GPIO_InitStruct.Pin = LIMIT_SWITCH_UPPER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP; // Assuming switch pulls to GND
  HAL_GPIO_Init(LIMIT_SWITCH_UPPER_Port, &GPIO_InitStruct);


/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// This file (stm32f4xx_hal_msp.c) should handle the TIM2 GPIO Init
// We need to add the HAL_TIM_MspPostInit call in MX_TIM2_Init
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
  * where the assert_param error has occurred.
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