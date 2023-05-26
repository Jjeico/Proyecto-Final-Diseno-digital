/*
 * Flash.c
 *
 *  Created on: May 20, 2023
 *      Author: vapat+Jacob
 */
#include"Flash.h"
#include <string.h>
#include <stdio.h>

uint8_t read_flash_Byte[ 255 ];
extern SPI_HandleTypeDef hspi2;

void Flash_read_identification_id()
{
	uint8_t spiBuf[5];

	spiBuf[0] = READ_ID;
	spiBuf[1] = 0;
	spiBuf[2] = 0;
	spiBuf[3] = 0;
	spiBuf[4] = 0;

	//Put CSN low
	flash_csn(0);
	//Transmit register address

	HAL_SPI_Transmit(&hspi2, &spiBuf[0], 5, 1000);
	//Receive data
	HAL_SPI_Receive(&hspi2, &read_flash_Byte[0], 8, 1000);
	//Bring CSN high
	flash_csn(1);
}


void flash_csn( uint8_t val){
	if ( val == 0 )
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

}


void Flash_read_status1_register( uint8_t *reg_to_save)
{
	uint8_t spiBuf[1];
	//Put CSN low
	spiBuf[0] = READ_STATUS_REG_1;

	flash_csn(0);

	HAL_SPI_Transmit(&hspi2, &spiBuf[0], 1, 1000);
	//Receive data
	HAL_SPI_Receive(&hspi2, reg_to_save, 1, 1000);
	//Bring CSN high
	flash_csn(1);
}

void Flash_read_status2_register( uint8_t *reg_to_save)
{
	uint8_t spiBuf[1];
	//Put CSN low
	spiBuf[0] = READ_STATUS_REG_2;

	flash_csn(0);

	HAL_SPI_Transmit(&hspi2, &spiBuf[0], 1, 1000);
	//Receive data
	HAL_SPI_Receive(&hspi2, reg_to_save, 1, 1000);
	//Bring CSN high
	flash_csn(1);
}

void Flash_read_status3_register( uint8_t *reg_to_save)
{
	uint8_t spiBuf[1];
	//Put CSN low
	spiBuf[0] = READ_STATUS_REG_3;

	flash_csn(0);

	HAL_SPI_Transmit(&hspi2, &spiBuf[0], 1, 1000);
	//Receive data
	HAL_SPI_Receive(&hspi2, reg_to_save, 1, 1000);
	//Bring CSN high
	flash_csn(1);
}

void Flash_activate_deactivate_block_protect()
{
	Write_flash_enable();
	uint8_t spiBuf[2];
	spiBuf[0] = WRITE_STATUS_REG_1;
	spiBuf[1] = 0b01100000;
	flash_csn(0);
	//Transmit register address and data
	HAL_SPI_Transmit(&hspi2, &spiBuf[0], 2 , 100);
	//Bring CSN high
	flash_csn(1);
}

void Write_flash_enable(){
	uint8_t spiBuf = WREN;
	//Put CSN low
	flash_csn(0);
	//Transmit register address
	HAL_SPI_Transmit(&hspi2, &spiBuf, 1, 1000);
	flash_csn(1);
}


void Flash_verificate_status()
{
	uint8_t spiBuf[1];
	//Put CSN low
	spiBuf[0] = READ_STATUS_REG_1;
	uint8_t bussy = 1;
	while (bussy == 1){
		flash_csn(0);
		//Transmit register address
		HAL_SPI_Transmit(&hspi2, &spiBuf[0], 1, 1000);
		//Receive data
		HAL_SPI_Receive(&hspi2, &read_flash_Byte[0], 1, 1000);
		//Bring CSN high
		flash_csn(1);
		bussy = read_flash_Byte[0] & 0X01;
	}
}



void Flash_write_page(uint8_t address_1, uint8_t address_2, uint8_t address_3, uint8_t *value_to_write, uint8_t length)
{
	Flash_verificate_status();

	Write_flash_enable();

	uint8_t spiBuf[length + 4];

	spiBuf[0] = PAGE_PROGRAM;
	// concatenate all the data to be sent
	sprintf( (char*)&spiBuf[1], "%c%c%c", address_1, address_2, address_3 );
	strncpy( (char*)&spiBuf[4], (char*)value_to_write, length );
	//Put CSN low
	flash_csn(0);
	//Transmit register address and data
	HAL_SPI_Transmit(&hspi2, &spiBuf[0], length+4 , 100);
	//Bring CSN high
	flash_csn(1);
}


void Flash_read_page(uint8_t address_1, uint8_t address_2, uint8_t address_3, uint8_t aux_length)
{
	uint8_t spiBuf[5];
	Flash_verificate_status();
	//Put CSN low
	flash_csn(0);
	// concatenate all the data to be sent
	spiBuf[0] = READ_PAGE;
	sprintf( (char*)&spiBuf[1], "%c%c%c", address_1, address_2, address_3 );
	//Transmit register address
	HAL_SPI_Transmit(&hspi2, &spiBuf[0], 4, 1000);
	//Receive data
	HAL_SPI_Receive(&hspi2, &read_flash_Byte[0], aux_length, 1000);
	//Bring CSN high
	flash_csn(1);
}



void Flash_sector_erase(uint8_t address_1, uint8_t address_2, uint8_t address_3)
{
	Flash_verificate_status();
	// Activate WREN
	Write_flash_enable();
	uint8_t spiBuf[5];

	//Put CSN low
	flash_csn(0);
	// concatenate all the data to be sent
	spiBuf[0] = SECTOR_ERASE;
	sprintf((char*) &spiBuf[1], "%c%c%c", address_1, address_2, address_3 );
	//Transmit register address
	HAL_SPI_Transmit(&hspi2, &spiBuf[0], 4, 1000);
	//Bring CSN high
	flash_csn(1);
}


void Flash_block_erase(uint8_t address_1, uint8_t address_2, uint8_t address_3)
{
	Flash_verificate_status();
	// Activate WREN
	Write_flash_enable();
	uint8_t spiBuf[5];

	//Put CSN low
	flash_csn(0);
	// concatenate all the data to be sent
	spiBuf[0] = BLOCK_ERASE;
	sprintf((char*) &spiBuf[1], "%c%c%c", address_1, address_2, address_3 );
	//Transmit register address
	HAL_SPI_Transmit(&hspi2, &spiBuf[0], 4, 1000);
	//Bring CSN high
	flash_csn(1);
}



void erase_all (){
	Flash_verificate_status();
	// Activate WREN
	Write_flash_enable();

	uint8_t spiBuf[1];

	//Transmit register address
	spiBuf[0] = ERASE_ALL_COMAND;

	flash_csn(0);
	HAL_SPI_Transmit(&hspi2, &spiBuf[0], 1, 1000);
	flash_csn(1);

	Flash_verificate_status();
}
