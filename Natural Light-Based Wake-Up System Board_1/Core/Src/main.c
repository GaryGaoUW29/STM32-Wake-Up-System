/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body for STM32 Board 1 (Time Control)
 * @author         : Gary Gao (g44gao@uwaterloo.ca)
 * @brief          : Refactored to integrate DS1302 RTC and TM1637 Display.
 * : All 74HC595 and PWM Motor code has been removed.
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ds1302.h"  // Include DS1302 RTC driver header
#include "TM1637.h"  // Include TM1637 7-Segment driver header
#include <stdbool.h> // For bool type
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- TM1637 (7-Segment Display) Pin Definitions ---
// !! Modify these pins based on your hardware connection !!
#define TM1637_CLK_GPIO_Port  GPIOB
#define TM1637_CLK_Pin_Num    8            // Corresponds to clkPin (0-15) in TM1637.c
#define TM1637_DIO_GPIO_Port  GPIOB
#define TM1637_DIO_Pin_Num    9            // Corresponds to dioPin (0-15) in TM1637.c

// --- Button Pin Definitions ---
// !! Modify these pins based on your hardware connection !!
#define Button_Mode_GPIO_Port   GPIOA
#define Button_Mode_Pin         GPIO_PIN_0
#define Button_Hour_GPIO_Port   GPIOA
#define Button_Hour_Pin         GPIO_PIN_1
#define Button_Minute_GPIO_Port GPIOA
#define Button_Minute_Pin       GPIO_PIN_4

// --- LED Pin Definitions ---
// !! Modify these pins based on your hardware connection !!
#define LED_Mode_GPIO_Port      GPIOC
#define LED_Mode_Pin            GPIO_PIN_7
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
UART_HandleTypeDef huart1; // UART handle for communication with Board 2

Time_s currentTime;      // Stores the current time read from RTC
Time_s alarmTime;        // Stores the user-set alarm time
bool systemMode = 0;     // 0 = Display current time, 1 = Set alarm time
bool alarmTriggered = false; // Prevents repeated UART signals within the alarm minute
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART1_Init(void);
void checkButtons(Time_s *almTime, bool *mode);
void Send_Wakeup_Signal_UART(void);
void controlLED(bool mode);
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
    MX_UART1_Init(); // Initialize UART

    /* USER CODE BEGIN 2 */
    // --- Initialize peripheral drivers ---
    ds1302_init();    // Initialize DS1302 RTC module
    TM1637Init(TM1637_CLK_GPIO_Port, TM1637_DIO_GPIO_Port, TM1637_CLK_Pin_Num, TM1637_DIO_Pin_Num);
    TM1637SetBrightness(7); // Set display brightness (0-7)

    // --- Set default alarm ---
    alarmTime.hour = 7;
    alarmTime.min = 0;
    alarmTime.sec = 0;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        // 1. Check button inputs, update alarm time or system mode
        checkButtons(&alarmTime, &systemMode);

        // 2. Get current time from DS1302 RTC hardware
        ds1302_get_time(&currentTime);

        // 3. Update 7-segment display based on system mode
        if (systemMode == 0) {
            // Mode 0: Display current time (Format: HHMM)
            TM1637DisplayNumber(currentTime.hour * 100 + currentTime.min, true); // true = show colon
        } else {
            // Mode 1: Display alarm time (Format: HHMM)
            TM1637DisplayNumber(alarmTime.hour * 100 + alarmTime.min, true);
        }

        // 4. Control LED indicator
        controlLED(systemMode);

        // 5. Check if alarm is triggered
        if (systemMode == 0 &&
            currentTime.hour == alarmTime.hour &&
            currentTime.min == alarmTime.min)
        {
            if (!alarmTriggered) {
                Send_Wakeup_Signal_UART(); // Send UART signal to Board 2
                alarmTriggered = true;     // Mark as sent to prevent repetition
            }
        } else {
            alarmTriggered = false; // Reset trigger flag
        }

        HAL_Delay(200); // Add delay for debouncing and to reduce CPU load
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
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
    __HAL_RCC_GPIOC_CLK_ENABLE(); // Ensure clock for LED is also enabled

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LED_Mode_GPIO_Port, LED_Mode_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : LED_Mode_Pin */
    GPIO_InitStruct.Pin = LED_Mode_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_Mode_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : Button_Mode_Pin Button_Hour_Pin Button_Minute_Pin */
    GPIO_InitStruct.Pin = Button_Mode_Pin | Button_Hour_Pin | Button_Minute_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // Assuming pull-up resistors (LOW when pressed)
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    // GPIO for TM1637 and DS1302
    // will be handled by their respective Init functions in TM1637.c and ds1302.c
    /* USER CODE END MX_GPIO_Init_2 */
}

/**
 * @brief UART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART1_Init(void) {
    /* USER CODE BEGIN UART1_Init 0 */

    /* USER CODE END UART1_Init 0 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX; // Board 1 only needs to transmit
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN UART1_Init 2 */

    /* USER CODE END UART1_Init 2 */
}


/**
 * @brief Checks button status and updates settings
 * @param almTime Pointer to the alarm time struct
 * @param mode Pointer to the system mode
 * @retval None
 */
void checkButtons(Time_s *almTime, bool *mode) {
    // Check mode button
    if (HAL_GPIO_ReadPin(Button_Mode_GPIO_Port, Button_Mode_Pin) == GPIO_PIN_RESET) {
        HAL_Delay(100); // Simple delay for debouncing
        if (HAL_GPIO_ReadPin(Button_Mode_GPIO_Port, Button_Mode_Pin) == GPIO_PIN_RESET) {
            *mode = !(*mode); // Toggle mode
            while(HAL_GPIO_ReadPin(Button_Mode_GPIO_Port, Button_Mode_Pin) == GPIO_PIN_RESET); // Wait for button release
        }
    }

    // If in alarm setting mode (mode == 1)
    if (*mode == true) {
        // Check "Hour" button
        if (HAL_GPIO_ReadPin(Button_Hour_GPIO_Port, Button_Hour_Pin) == GPIO_PIN_RESET) {
            HAL_Delay(100); // Debounce
            if (HAL_GPIO_ReadPin(Button_Hour_GPIO_Port, Button_Hour_Pin) == GPIO_PIN_RESET) {
                almTime->hour = (almTime->hour + 1) % 24; // Hour + 1, 24-hour cycle
            }
        }

        // Check "Minute" button
        if (HAL_GPIO_ReadPin(Button_Minute_GPIO_Port, Button_Minute_Pin) == GPIO_PIN_RESET) {
            HAL_Delay(100); // Debounce
            if (HAL_GPIO_ReadPin(Button_Minute_GPIO_Port, Button_Minute_Pin) == GPIO_PIN_RESET) {
                almTime->min = (almTime->min + 1) % 60; // Minute + 1, 60-minute cycle
            }
        }
    }
}

/**
 * @brief Controls the state of the LED
 * @param mode The current system mode (0 or 1)
 * @retval None
 */
void controlLED(bool mode) {
    // LED ON when in Mode 1 (setting alarm), OFF in Mode 0 (displaying time)
    HAL_GPIO_WritePin(LED_Mode_GPIO_Port, LED_Mode_Pin,
            mode ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief Sends a wakeup signal to Board 2 via UART
 * @param None
 * @retval None
 */
void Send_Wakeup_Signal_UART(void) {
    uint8_t wakeupSignal = 0xAA; // Send a "Magic Byte"
    HAL_UART_Transmit(&huart1, &wakeupSignal, 1, 100); // Transmit 1 byte, 100ms timeout
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