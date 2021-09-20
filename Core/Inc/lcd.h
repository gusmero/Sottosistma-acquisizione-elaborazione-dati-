/*
 * lcd.h
 *
 *  Created on: Jun 3, 2020
 *      Author: pietro
 */

#ifndef SRC_LCD_H_
#define SRC_LCD_H_

#include "stm32f7xx_hal.h"

void lcdInit(I2C_HandleTypeDef *);

uint8_t lcdGetID();

uint8_t lcdGetFirmwareVersion();

void lcdUartEnable(int state);

/**
 * Set direction of GPIO.
 * @param pin   The pin number according to schematic: GPIO1, GPIO2, etc...
 * @param direction     The direction of the GPIO: OUTPUT or INPUT.
 */
void lcdPinMode(uint8_t pin, uint8_t direction);

/**
 * If pin is set as output this method define the level of the GPIO.
 * If pin is input - it enables internal pull-up resistor (only available
 * for PORTB pins).
 * @param pin   The number of the desired GPIO: GPIO1, GPIO2, etc...
 * @param level The output level: HIGH or LOW
 */
void lcdDigitalWrite(uint8_t pin, uint8_t level);

/**
 * Read the state of individual GPIO, if configured as input.
 * @param pin   The number of the GPIO: GPIO1, GPIO2, etc...
 * @return      If input level is high - 1, else - 0.
 */
uint8_t lcdDigitalRead(uint8_t pin);

/**
 * Read the state of the 4 buttons.
 * @return      Bitmask with the 4 values: LSB - BUT1, MSB - BUT4
 */
uint8_t lcdReadButtons();

void lcdSetBacklight(uint8_t level);

/**
 * Clear the LCD screen.
 */
void lcdClear();

/**
 * Position the cursor of the LCD to a given pair of coordinates.
 * @param col   column coordinate
 * @param row   row coordinate
 */
void lcdSetCursor(uint8_t col, uint8_t row);

void lcdPrint(const char string[]);

#endif /* SRC_LCD_H_ */
