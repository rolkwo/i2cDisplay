/*
 * i2cDisplay.h
 *
 *  Created on: Feb 13, 2016
 *      Author: roland
 */

#ifndef I2CDISPLAY_H_
#define I2CDISPLAY_H_

#include <stm32f10x.h>

class I2cDisplay
{
public:
	I2cDisplay(I2C_TypeDef* i2c, bool initializeI2c = true);

	void cls();

	void write(const char* str);
	void gotoPos(uint8_t line, uint8_t col);

private:
	void initLcd();
	void init4bit();
	void initI2c();

	void sendByte(uint8_t byte);
	void sendCmd(uint8_t byte);
	void sendChar(uint8_t c);
	void sendHalfword(uint8_t half, bool rs = false);

	I2C_TypeDef* _i2c;

	static const uint8_t RS;
	static const uint8_t RW;
	static const uint8_t E;
	static const uint8_t DATA_OFFSET;

};


#endif /* I2CDISPLAY_H_ */
