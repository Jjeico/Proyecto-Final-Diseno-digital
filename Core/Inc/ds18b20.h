/*
 * ds18b20.h
 *
 *  Created on: May 5, 2023
 *      Author: Jacob
 */
#include "stdio.h"
#include "main.h"

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_

#define PIN_LOW 		HAL_GPIO_WritePin (DS18B20_GPIO_Port, DS18B20_Pin, 0);
#define PIN_HIGH		HAL_GPIO_WritePin (DS18B20_GPIO_Port, DS18B20_Pin, 1);



uint8_t DS18B20_Start (void);
void DS18B20_Write (uint8_t data);
uint8_t DS18B20_Read (void);


//SENSOR DS18B20 TEMPERATURA
//Vcc = +5V, DAT = A1


#endif /* INC_DS18B20_H_ */
