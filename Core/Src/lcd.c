/*
 * lcd.c
 *
 *  Created on: Jun 3, 2020
 *      Author: pietro
 */

#include <string.h>
#include "lcd.h"

#define SET_TRIS    0x01
#define SET_LAT     0x02
#define GET_PORT    0x03
#define GET_BUT     0x05
#define GET_ID      0x20
#define GET_FRM     0x21
#define LCD_CLR     0x60
#define LCD_WR      0x61
#define SET_BL		0x62
#define UART_EN     0x10

#define BOARD_ID    0x65
#define ADDRESS     0x30

static uint8_t _col, _row;
static I2C_HandleTypeDef *_hi2c;

void lcdInit(I2C_HandleTypeDef *hi2c) {
	_col = _row = 0;
	_hi2c = hi2c;
}

uint8_t lcdGetID() {
    uint8_t data = GET_ID;
    uint8_t id = 0;
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(_hi2c, ADDRESS << 1, &data, 1, 1000);
    status = HAL_I2C_Master_Receive(_hi2c, ADDRESS << 1, &id, 1, 1000);
    return id;
}

uint8_t lcdGetFirmwareVersion() {
    uint8_t data = GET_FRM;
    uint8_t firm = 0;
    HAL_I2C_Master_Transmit(_hi2c, ADDRESS << 1, &data, 1, 1000);
    HAL_I2C_Master_Receive(_hi2c, ADDRESS << 1, &firm, 1, 1000);
    return firm;
}

void lcdUartEnable(int state) {
    uint8_t data[2];
    data[0] = UART_EN;
    data[1] = (state ? 0x01 : 0x00);
    HAL_I2C_Master_Transmit(_hi2c, ADDRESS << 1, data, 2, 1000);
}

void lcdPinMode(uint8_t pin, uint8_t direction) {
    uint8_t data[3];
    data[0] = SET_TRIS;
    data[1] = pin;
    data[2] = !direction;
    HAL_I2C_Master_Transmit(_hi2c, ADDRESS << 1, data, 3, 1000);
}

void lcdDigitalWrite(uint8_t pin, uint8_t level) {
    uint8_t data[3];
    data[0] = SET_LAT;
    data[1] = pin;
    data[2] = level;
    HAL_I2C_Master_Transmit(_hi2c, ADDRESS << 1, data, 3, 1000);
}

uint8_t lcdDigitalRead(uint8_t pin) {
    uint8_t data[2];
    uint8_t port;
    data[0] = GET_PORT;
    data[1] = pin;
    HAL_I2C_Master_Transmit(_hi2c, ADDRESS << 1, data, 2, 1000);
    HAL_I2C_Master_Receive(_hi2c, ADDRESS << 1, &port, 1, 1000);
    return port;
}

uint8_t lcdReadButtons() {
    uint8_t data = GET_BUT;
    uint8_t buttons = 0;
    HAL_I2C_Master_Transmit(_hi2c, ADDRESS << 1, &data, 1, 1000);
    HAL_I2C_Master_Receive(_hi2c, ADDRESS << 1, &buttons, 1, 1000);
    return buttons;
}

void lcdSetBacklight(uint8_t level) {
    uint8_t data[2];
    data[0] = SET_BL;
    data[1] = level;
    HAL_I2C_Master_Transmit(_hi2c, ADDRESS << 1, data, 2, 1000);
}

void lcdClear() {
    uint8_t data = LCD_CLR;
    HAL_I2C_Master_Transmit(_hi2c, ADDRESS << 1, &data, 1, 1000);
    HAL_Delay(100);
    _col = _row = 0;
}

void lcdSetCursor(uint8_t col, uint8_t row) {
    if (col > 15 || col < 0) {
        return;
    }

    if (row > 1 || row < 0) {
        return;
    }
    _col = col;
    _row = row;
}

void lcdPrint(const char string[]) {
	uint8_t len = strlen(string);

	len = strlen(string);
	for (int i = 0; i < len; i++){
	    uint8_t data[4];
	    data[0] = LCD_WR;
	    data[1] = 1 - _row;
	    data[2] = _col;
	    data[3] = string[i];
	    HAL_I2C_Master_Transmit(_hi2c, ADDRESS << 1, data, 4, 1000);
		HAL_Delay(20);
		++_col;
		if (_col > 15){
			_col = 0;
			++_row;
			if (_row > 2)
				return;
		}
	}
}

