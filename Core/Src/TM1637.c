
/*
 * TM1637.c
 *
 *  Created on: Nov 30, 2023
 *      Author: k-off
 *     Licence: MIT
 */

#include "TM1637.h"

enum PinState { DOWN=0, UP };
#define PinDelay 2

/**
 * Maps ascii table to the 7-segment display representation
 * So ASCIIMap['a'] will return the byte that has to be sent
 * to the display.
 * Lower case and upper case character values are identical.
 */
const 	uint8_t ASCIIMap[128] = {
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0,
		0x8, // , (_)
		0x40, // -
		0x8, // . (_)
		0,
		0x3f, // 0
		0x06, // 1
		0x5b, // 2
		0x4f, // 3
		0x66, // 4
		0x6d, // 5
		0x7d, // 6
		0x07, // 7
		0x7f, // 8
		0x6f, // 9
		0, 0, 0, 0, 0, 0, 0,
		0x77, // a
		0x7c, // b
		0x39, // c
		0x5e, // d
		0x79, // e
		0x71, // f
		0x3d, // g
		0x76, // h
		0x04, // i
		0x1e, // j
		0x75, // k
		0x38, // l
		0x49, // m
		0x54, // n
		0x5c, // o
		0x73, // p
		0x67, // q
		0x50, // r
		0x6d, // s
		0x78, // t
		0x1c, // u
		0x3e, // V
		0x7e, // W
		0x76, // x (H)
		0x6e, // y
		0x5b, // z (2)
		0, 0, 0, 0,
		0x8, // _
		0,
		0x77, // a
		0x7c, // b
		0x39, // c
		0x5e, // d
		0x79, // e
		0x71, // f
		0x3d, // g
		0x76, // h
		0x04, // i
		0x1e, // j
		0x75, // k
		0x38, // l
		0x49, // m
		0x54, // n
		0x5c, // o
		0x73, // p
		0x67, // q
		0x50, // r
		0x6d, // s
		0x78, // t
		0x1c, // u
		0x3e, // V
		0x7e, // W
		0x76, // x (H)
		0x6e, // y
		0x5b, // z (2)
		0
	};

// some global vars, they are only visible in the scope
// of this file
GPIO_TypeDef *gClkPort;
GPIO_TypeDef *gDioPort;
uint8_t gClkPin;
uint8_t gDioPin;

/**
  * @brief	Helper function that enables clocks for the ports
  * 		required by user
  * @param	port - pointer to the port(i.e. GPIOA.. GPIOH)
  * @retval None
 */
static void InitGPIOClock(GPIO_TypeDef *port){
	if (false) {}
#ifdef GPIOA
	else if (port == GPIOA)
		__HAL_RCC_GPIOA_CLK_ENABLE();
#endif
#ifdef GPIOB
	else if (port == GPIOB)
		__HAL_RCC_GPIOB_CLK_ENABLE();
#endif
#ifdef GPIOC
	else if (port == GPIOC)
		__HAL_RCC_GPIOC_CLK_ENABLE();
#endif
#ifdef GPIOD
	else if (port == GPIOD)
		__HAL_RCC_GPIOD_CLK_ENABLE();
#endif
#ifdef GPIOE
	else if (port == GPIOE)
		__HAL_RCC_GPIOE_CLK_ENABLE();
#endif
#ifdef GPIOF
	else if (port == GPIOF)
		__HAL_RCC_GPIOF_CLK_ENABLE();
#endif
#ifdef GPIOG
	else if (port == GPIOG)
		__HAL_RCC_GPIOG_CLK_ENABLE();
#endif
#ifdef GPIOH
	else if (port == GPIOH)
		__HAL_RCC_GPIOH_CLK_ENABLE();
#endif
}


/**
  * @brief	Delay for 'us' microseconds - blocking variant
  * 		Delays are required, since the change of the
  * 		voltage on pins is not instantaneous. Default
  * 		delay function only resolves to milliseconds,
  * 		which makes all stuff too slow.
  * @param	us - amount of microseconds to delay for
  * @retval None
 */
static void DelayUSb(uint32_t us) {
    while (us) {
        for (uint8_t i = 0; i < 10; ++i) {
            __asm__ __volatile__("nop\n\t":::"memory");
        }
    	--us;
    }
}

/**
  * @brief	#TODO: implement delays using timers
  * 		Delay for 'us' microseconds - non-blocking variant
  * 		Delays are required, since the change of the
  * 		voltage on pins is not instantaneous. Default
  * 		delay function only resolves to milliseconds,
  * 		which makes all stuff too slow.
  * @param	us - amount of microseconds to delay for
  * @retval None
 */
static void DelayUSn(uint16_t us) {
	DelayUSb(us);
}

/**
  * @brief	Set or reset clock pin and delay for PinDelay
  * @param	set - desired state of the pin
  * @retval None
 */
static void ClockPin(bool set) {
	HAL_GPIO_WritePin(gClkPort, gClkPin, set);
	DelayUSn(PinDelay);
}

/**
  * @brief	Set or reset data pin and delay for PinDelay
  * @param	set - desired state of the pin
  * @retval None
 */
static void DataPin(bool set) {
	HAL_GPIO_WritePin(gDioPort, gDioPin, set);
	DelayUSn(PinDelay);
}

/**
  * @brief	Set start condition of the I2C protocol
  * @retval None
 */
static void StartTransmision(void) {
    ClockPin(UP);
    DataPin(UP);
    DataPin(DOWN);
}

/**
  * @brief	Set stop condition of the I2C protocol
  * @retval None
 */
static void StopTransmision(void) {
    ClockPin(DOWN);
    DataPin(DOWN);
    ClockPin(UP);
    DataPin(UP);
}

/**
  * @brief	Skip ACK bit that is sent by the display
  * 		back to the MCU, since current implementation
  * 		doesn't support read
  * @retval None
 */
static void SkipAck(void) {
    ClockPin(DOWN);
    ClockPin(UP);
    ClockPin(DOWN);
}

/**
  * @brief	Send a bit to the display
  * @param	b - 0 or 1
  * @retval None
 */
static void WriteBit(bool b) {
    ClockPin(DOWN);
    DataPin(b);
    ClockPin(UP);
}

/**
  * @brief	Send a byte to the display
  * @param	b - byte to send
  * @retval None
 */
static void WriteByte(uint8_t b) {
    for (int i = 0; i < 8; ++i) {
    	WriteBit(b & (1 << i));
    }
}

void TM1637Init(GPIO_TypeDef *clkPort, GPIO_TypeDef *dioPort, uint8_t clkPin, uint8_t dioPin) {
	InitGPIOClock(clkPort);
	InitGPIOClock(dioPort);
	gClkPort = clkPort;
	gDioPort = dioPort;
	gClkPin = (1 << clkPin);
	gDioPin = (1 << dioPin);
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = gClkPin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(clkPort, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = gDioPin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(dioPort, &GPIO_InitStruct);
}

void TM1637SetBrightness(uint8_t br) {
    StartTransmision();
    WriteByte(br);
    SkipAck();
    StopTransmision();
}

void TM1637DisplayText(uint8_t *txt, uint8_t len, bool displaySeparator) {
	StartTransmision();
	WriteByte(0x40);
	SkipAck();
	StopTransmision();

	StartTransmision();
	WriteByte(0xc0);
	SkipAck();

	for (uint8_t i = 0; i < ((len > 4 ? 4 : len)); ++i) {
		if (i == 1 && displaySeparator) {
			WriteByte(ASCIIMap[txt[i] & 127] | (1 << 7));
		}
		else{
			WriteByte(ASCIIMap[txt[i] & 127]);
		}
		SkipAck();
	}
	StopTransmision();
}

void TM1637DisplayNumber(uint16_t v, bool displaySeparator)
{
    uint8_t digitArr[4] = {};
    for (uint8_t i = 0; i < 4; ++i) {
        digitArr[3 - i] = '0' + (v%10);
        v /= 10;
    }
    TM1637DisplayText(digitArr, sizeof(digitArr), displaySeparator);
}

void TM1637Ticker(uint16_t initialDelay, uint16_t charDelay, const char *fmt, ...) {
	char buf[BUF_LEN + 4] = {};
	uint8_t *ptr = (uint8_t*)buf; // ptr used to roll through the buffer
	int actualLen = 0;
	va_list vl;

	va_start(vl, fmt);
	actualLen = vsnprintf(buf, BUF_LEN, fmt, vl);
	va_end(vl);

	if (actualLen < 1) return ;
	TM1637DisplayText(ptr, 4, false);
	HAL_Delay(initialDelay);
	++ptr;
	for (int i = 1; i < actualLen + 4; ++i) {
		TM1637DisplayText(ptr, 4, false);
		HAL_Delay(charDelay);
		++ptr;
	}
	HAL_Delay(initialDelay);
}
