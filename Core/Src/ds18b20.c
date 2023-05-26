/*
 * ds18b20.c
 *
 *  Created on: May 5, 2023
 *      Author: Jacob
 */
#include "ds18b20.h"
#include "main.h"




void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; //Push pull for 0's and 1's
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}


uint8_t DS18B20_Start (void)
{
	uint8_t Response = 0;
	Set_Pin_Output(DS18B20_GPIO_Port, DS18B20_Pin);   // set the pin as output
	PIN_LOW;  // pull the pin low (write a 0)
	delay (481);   // delay according to datasheet, 480us minimum

	Set_Pin_Input(DS18B20_GPIO_Port, DS18B20_Pin);    // set the pin as input
	delay (80);    // delay according to datasheet 60us minimum

	if (!(HAL_GPIO_ReadPin (DS18B20_GPIO_Port, DS18B20_Pin))) Response = 1;    // if the pin is low i.e the presence pulse is detected
	else Response = -1;
	delay (400); // 480 us delay totally, so 400+ the previous 80
	return Response; //Best option is to set it up with booleans (true,false)
}
void DS18B20_Write (uint8_t data)
{
	Set_Pin_Output(DS18B20_GPIO_Port, DS18B20_Pin);  // set as output
	for (int i=0; i<8; i++)
	{
		if ((data & (1<<i))!=0)  // if the bit is high (checks if the bits are 1's or 0's with and operator - for example 0data and 1i = 0, 1data and 1i = 1, "<<" moves bits to the left "i" times)
		{
			// write 1
			Set_Pin_Output(DS18B20_GPIO_Port, DS18B20_Pin);  // set as output
			PIN_LOW;  // pull the pin LOW
			delay (1);  // wait for less than 15us
			Set_Pin_Input(DS18B20_GPIO_Port, DS18B20_Pin);  // set as input
			delay (60);  // wait for 60 us
		}
		else  // if the bit is low
		{
			// write 0
			Set_Pin_Output(DS18B20_GPIO_Port, DS18B20_Pin); // set as output
			PIN_LOW;  // pull the pin LOW
			delay (60);  // wait for 60 us
			Set_Pin_Input(DS18B20_GPIO_Port, DS18B20_Pin); // set as input
		}
	}
}
uint8_t DS18B20_Read (void)
{
	uint8_t value=0;
	Set_Pin_Input(DS18B20_GPIO_Port, DS18B20_Pin);
	for (int i=0;i<8;i++) //or operator to change bits
	{
		Set_Pin_Output(DS18B20_GPIO_Port, DS18B20_Pin);   // set as output
		PIN_LOW;  // pull the data pin LOW (0)
		delay (2);  // wait for > 1us
		Set_Pin_Input(DS18B20_GPIO_Port, DS18B20_Pin);  // set as input
		delay (2);  // wait for > 1us to not read so fast (give time to tiempo de subida)
		if (HAL_GPIO_ReadPin (DS18B20_GPIO_Port, DS18B20_Pin))  // if the pin is HIGH (detects if its receiving a 1)
		{
			value |= 1<<i;  // read = 1
		}
		delay (60);  // wait for 60 us
	}
	return value;
}
