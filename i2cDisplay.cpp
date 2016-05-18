/*
 * i2cDisplay.cpp
 *
 *  Created on: Feb 13, 2016
 *      Author: roland
 */

#include "i2cDisplay.h"

#include "Delay.h"

const uint8_t I2cDisplay::RS = 0x01;
const uint8_t I2cDisplay::RW = 0x02;
const uint8_t I2cDisplay::E = 0x04;
const uint8_t I2cDisplay::BACK_LIGHT = 0x08;
const uint8_t I2cDisplay::DATA_OFFSET = 4;

I2cDisplay::I2cDisplay(I2C_TypeDef* i2c, bool initializeI2c)
	: _i2c(i2c), _backlight(false)
{
	if(initializeI2c)
		initI2c();

	initLcd();
}

void I2cDisplay::cls()
{
	sendCmd(0x01);
	delay(1520);
}

void I2cDisplay::write(const char* str)
{
	while(*str != '\0')
	{
		sendChar(*str);
		++str;
	}
}

void I2cDisplay::gotoPos(uint8_t line, uint8_t col)
{
	uint8_t add = line * 0x40 + col;

	sendCmd(0x80 | add);
}

void I2cDisplay::setBacklight(bool state)
{
    _backlight = state;

    while(I2C_GetFlagStatus(_i2c, I2C_FLAG_BUSY))
        ;
    I2C_GenerateSTART(_i2c, ENABLE);
    while(!I2C_CheckEvent(_i2c, I2C_EVENT_MASTER_MODE_SELECT))
        ;
    I2C_Send7bitAddress(_i2c, 0x3f<<1, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(_i2c, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
        ;
    if(_backlight)
        I2C_SendData(_i2c, BACK_LIGHT);
    else
        I2C_SendData(_i2c, 0x00);
    while(!I2C_CheckEvent(_i2c, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
        ;
    I2C_GenerateSTOP(_i2c, ENABLE);

}

void I2cDisplay::initLcd()
{
	delay(15200);
	init4bit();
	delay(37);
	sendCmd(0x28);
	delay(37);
	sendCmd(0x0c);

}

void I2cDisplay::init4bit()
{
	sendHalfword(0x02);
}

void I2cDisplay::initI2c()
{
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Mode = GPIO_Mode_AF_OD;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;


	if(_i2c == I2C1)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

		gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_Init(GPIOB, &gpio);
	}
	else if(_i2c == I2C2)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

		gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
		GPIO_Init(GPIOB, &gpio);
	}

	I2C_InitTypeDef i2ci;
	I2C_StructInit(&i2ci);
	i2ci.I2C_ClockSpeed = 50000;
	I2C_Init(_i2c, &i2ci);
	I2C_Cmd(_i2c, ENABLE);
}

void I2cDisplay::sendByte(uint8_t byte)
{
    if(_backlight)
        byte |= BACK_LIGHT;
    else
        byte &= ~(BACK_LIGHT);

	while(I2C_GetFlagStatus(_i2c, I2C_FLAG_BUSY))
		;
	I2C_GenerateSTART(_i2c, ENABLE);
	while(!I2C_CheckEvent(_i2c, I2C_EVENT_MASTER_MODE_SELECT))
		;
	I2C_Send7bitAddress(_i2c, 0x3f<<1, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(_i2c, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		;
	I2C_SendData(_i2c, byte);
	while(!I2C_CheckEvent(_i2c, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;
	I2C_GenerateSTOP(_i2c, ENABLE);
}

void I2cDisplay::sendCmd(uint8_t byte)
{
	uint8_t upper = (byte & 0xf0) >> 4;
	uint8_t lower = (byte & 0x0f);

	sendHalfword(upper);
	sendHalfword(lower);
}

void I2cDisplay::sendChar(uint8_t c)
{
	uint8_t upper = (c & 0xf0) >> 4;
	uint8_t lower = (c & 0x0f);

	sendHalfword(upper, true);
	sendHalfword(lower, true);
}

void I2cDisplay::sendHalfword(uint8_t half, bool rs)
{
	half <<= DATA_OFFSET;

	if(rs)
		half |= RS;

	sendByte(half);
	delay(100);
	sendByte(half | E);
	delay(100);
	sendByte(half);
	delay(100);
}


