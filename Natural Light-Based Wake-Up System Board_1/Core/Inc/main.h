/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Button_Mode_Pin GPIO_PIN_5
#define Button_Mode_GPIO_Port GPIOC
#define RCLK_Pin GPIO_PIN_13
#define RCLK_Pin_GPIO_Port GPIOB
#define SRCLK_Pin GPIO_PIN_14
#define SRCLK_Pin_GPIO_Port GPIOB
#define DATA_Pin GPIO_PIN_15
#define DATA_Pin_GPIO_Port GPIOB
#define Button_Minute_Pin GPIO_PIN_6
#define Button_Minute_GPIO_Port GPIOC
#define Button_Hour_Pin GPIO_PIN_8
#define Button_Hour_GPIO_Port GPIOC
#define LED_Mode_Pin GPIO_PIN_12
#define LED_Mode_GPIO_Port GPIOA
#define DS1302_PIN_SCLK_Pin GPIO_PIN_3
#define DS1302_PIN_SCLK_GPIO_Port GPIOB
#define DS1302_PIN_SDA_Pin GPIO_PIN_4
#define DS1302_PIN_SDA_GPIO_Port GPIOB
#define DS1302_PIN_RST_Pin GPIO_PIN_5
#define DS1302_PIN_RST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
