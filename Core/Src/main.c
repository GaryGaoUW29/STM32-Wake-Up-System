#include "main.h"
#include "ds1302.h"
#include <stdio.h>

void SystemClock_Config(void);
void MX_GPIO_Init(void);

GPIO_PORT = GPIOB;
#define NUM_74HC595_CHIPS 4
uint8_t data[NUM_74HC595_CHIPS] = { 0xFF, 0xFF, 0xFF, 0xFF };

const uint8_t segmentMap[10] = {
		0x67, // 0
		0x67, // 1
		0xA4, // 2
		0xB0, // 3
		0x99, // 4
		0x92, // 5
		0x82, // 6
		0xF8, // 7
		0x80, // 8
		0x90  // 9
		};

const uint8_t segmentMapInverted[10] = {
		0x3F, // 0
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

int main(void) {
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	My_ds1302_init();

	Time_s user_seted_time = { 0 };

	while (1) {
		//displayTime(0,0);
		sendData();
		HAL_Delay(1000);
	}
}

void sendData() {
    HAL_GPIO_WritePin(GPIO_PORT, RCLK_Pin, GPIO_PIN_RESET);
    for (int i = 0; i < NUM_74HC595_CHIPS; i++) {
        for (int j = 0; j < 8; j++) {
            uint8_t bitValue = (data[i] & (1 << (7 - j)));

            if (bitValue) {
                HAL_GPIO_WritePin(GPIO_PORT, DATA_Pin, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(GPIO_PORT, DATA_Pin, GPIO_PIN_RESET);
            }

            HAL_GPIO_WritePin(GPIO_PORT, SRCLK_Pin, GPIO_PIN_SET);
            HAL_Delay(20);
            HAL_GPIO_WritePin(GPIO_PORT, SRCLK_Pin, GPIO_PIN_RESET);
        }
    }
    HAL_GPIO_WritePin(GPIO_PORT, RCLK_Pin, GPIO_PIN_SET);
}

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

void My_ds1302_init(void) {
	/* Initialize microcontroller configuration */
	ds1302_init();
	ds1302_clear_ram();
	Time_s def_time = { 0 };
	def_time.day = MONDAY;
	def_time.year = 24;
	def_time.month = 1;
	def_time.date = 1;
	def_time.clockSystem = DS1302_CLK_SYSTEM_12;
	def_time.clockPeriod = DS1302_CLK_PM_PERIOD;
	def_time.hour = 7;
	def_time.min = 0;
	def_time.sec = 0;
	ds1302_set_time(&def_time);
}

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

void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			RCLK_Pin | SRCLK_Pin | DATA_Pin | DS1302_PIN_SCLK_Pin
					| DS1302_PIN_SDA_Pin | DS1302_PIN_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_Mode_GPIO_Port, LED_Mode_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : Button_Mode_Pin Button_Minute_Pin Button_Hour_Pin */
	GPIO_InitStruct.Pin = Button_Mode_Pin | Button_Minute_Pin | Button_Hour_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : RCLK_Pin_Pin SRCLK_Pin_Pin DATA_Pin_Pin DS1302_PIN_SCLK_Pin
	 DS1302_PIN_SDA_Pin DS1302_PIN_RST_Pin */
	GPIO_InitStruct.Pin = RCLK_Pin | SRCLK_Pin | DATA_Pin
			| DS1302_PIN_SCLK_Pin | DS1302_PIN_SDA_Pin | DS1302_PIN_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_Mode_Pin */
	GPIO_InitStruct.Pin = LED_Mode_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_Mode_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
