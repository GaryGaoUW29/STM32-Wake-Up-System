#include "main.h"
#include "ds1302.h"

void SystemClock_Config(void);
void MX_GPIO_Init(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  My_ds1302_init();

  Time_s user_seted_time = {0};

  volatile uint16_t s = 0;
    uint8_t text[] = ",-.0123456789abcdefghijklmnopqrstuvwxyz_ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    uint8_t textSize = sizeof(text) - 4;
    uint8_t *ptr = text;
    TM1637Init(GPIOA, GPIOA, 1, 2);
    TM1637SetBrightness(Brightness1);
    while (1)
    {
  	  HAL_GPIO_TogglePin(GPIOC, (1<<13));
  	  TM1637Ticker(750, 375, "    hello world, its %04d pm", 420);
  //	  TM1637DisplayText(ptr, 4, false);
  //	  HAL_Delay(500);
  //	  TM1637DisplayNumber(s, s&1);
  //	  HAL_Delay(500);
  	  ++ptr;
  	  ++s;
  	  if (s > textSize) {s = 0; ptr = text;}

      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
    }
}

void My_ds1302_init(void){
/* Initialize microcontroller configuration */
  ds1302_init();
  ds1302_clear_ram();
  Time_s def_time = {0};
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



void Error_Handler(void)
{
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif

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

void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Mode_GPIO_Port, LED_Mode_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DS1302_PIN_SCLK_Pin|DS1302_PIN_SDA_Pin|DS1302_PIN_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Button_Mode_Pin Button_Minute_Pin Button_Hour_Pin */
  GPIO_InitStruct.Pin = Button_Mode_Pin|Button_Minute_Pin|Button_Hour_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Mode_Pin */
  GPIO_InitStruct.Pin = LED_Mode_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Mode_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DS1302_PIN_SCLK_Pin DS1302_PIN_SDA_Pin DS1302_PIN_RST_Pin */
  GPIO_InitStruct.Pin = DS1302_PIN_SCLK_Pin|DS1302_PIN_SDA_Pin|DS1302_PIN_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
