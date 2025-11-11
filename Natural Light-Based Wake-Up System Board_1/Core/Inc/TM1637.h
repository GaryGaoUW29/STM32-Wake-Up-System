/*
 * TM1637.h
 *
 * This library provides functionality for communication
 * between a STM32F4 MCU and a 7-segment 4-digit display
 * based on TM1637 chip.
 *
 * It basically implements a minimal I2C protocol, that
 * is enough for sending data to a slave device. It
 * doesn't read from slave and ignores ACK bits. (#TODO)
 *
 * It is aimed for cases when one wants to be more flexible
 * with selection of CLK/DIO pins, or when I2C pins provided
 * by the MCU are already busy.
 *
 * Usage:
 *
 *	int main(void)
 *	{
 *		SystemClock_Config();
 *		volatile uint16_t s = 0;
 *		uint8_t text[] = "0123456789ABCDEFG";
 *	  	uint8_t textSize = sizeof(text) - 4;
 *	  	uint8_t *ptr = text;					// used for ticker/crawler effect
 *		TM1637Init(GPIOA, GPIOA, 1, 2);			// works with any combination of two distinct available pins
 *		TM1637SetBrightness(Brightness1);
 *		while (1)
 *		{
 *			TM1637Crowl(750, 375, "    hello world, its %04d pm", 420); // display formatted text as crowler
 *			//TM1637DisplayText(ptr, textSize);	// display text
 *			//HAL_Delay(500);
 *			//TM1637DisplayNumber(s, s&1);		// display number with and without delimiter
 *			//HAL_Delay(500);
 *			//++ptr;								// move beginning of the displayed text to the next character
 *			++s;
 *			if (s > textSize) {s = 0; ptr = text;} //start from beginning
 *		}
 *		return 0;
 *	}
 *
 *	TODO: implement delays using timers
 *	TODO: add support for reading from slave devices
 *
 *  Created on: Nov 30, 2023
 *      Author: k-off
 *     Licence: MIT
 */

#ifndef TM1637_H
# define TM1637_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

/**
 * TM1637 supports 8 brightness settings
 */
enum Brightness {
	Disabled = 0x87,
	Brightness0,
	Brightness1,
	Brightness2,
	Brightness3,
	Brightness4,
	Brightness5,
	Brightness6,
	Brightness7
};

#define BUF_LEN 128

/**
  * @brief	Initialize desired port(s) and pins for communication with
  * 		the TM1637 display
  * @param	clkPort - port of the clock pin (i.e. GPIOA.. GPIOH)
  * @param	dioPort - port of the data pin, may differ from clkPort
  * @param	clkPin  - desired clock pin number on clkPort
  * @param	dioPin  - desired data pin number on dioPort
  * @retval None
 */
void TM1637Init(GPIO_TypeDef *clkPort, GPIO_TypeDef *dioPort, uint8_t clkPin, uint8_t dioPin);

/**
  * @brief	Set display brightness (see Brightness enum above)
  * @param	br - desired brightness in range 'Disabled' to 'Brightness7'
  * @retval None
 */
void TM1637SetBrightness(uint8_t br);

/**
  * @brief	Displays up to 4 first characters of the provided text buffer
  * 		Check out TM1637.c to see supported characters in ASCIIMap
  * @param	txt - array of unsigned characters
  * @param	len - length of the provided array
  * @param	displaySeparator - boolean flag, enable/disable separator
  * @retval None
 */
void TM1637DisplayText(uint8_t *txt, uint8_t len, bool displaySeparator);

/**
  * @brief	Displays up to 4 least significant digits of the provided
  * 		unsigned number, i.e. 12345 will display '2345'
  * 		Use TM1637Crowl() as analog of printf() for TM1637 display
  * @param	v - unsigned number to display
  * @param	displaySeparator - boolean flag, enable/disable separator
  * @retval None
 */
void TM1637DisplayNumber(uint16_t v, bool displaySeparator);

/**
  * @brief	Displays  formatted string with user provided initial delay
  * 		and a delay after each character.
  * 		It is printf + crawler for TM1637 display.
  * 		Usage: TM1637Crowl(500, 250, "Hi %d, %s", 5, "folks");
  * 		Update BUF_LEN in TM1637.h if really long messages are expected
  * @param	initialDelay - delay after first 4 characters appear on dispay, ms
  * @param	charDelay - delay after next character was displayed, ms
  * @param	fmt - format string, as in printf()
  * @param	... - variable number of arguments that match format string, as in printf()
  * @retval None
 */
void TM1637Ticker(uint16_t initialDelay, uint16_t charDelay, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif // !TM1637_H
