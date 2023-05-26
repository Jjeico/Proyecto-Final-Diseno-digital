/*
 * Flash.h
 *
 *  Created on: May 20, 2023
 *      Author: vapat+Jacob
 */

#include "main.h"
#include "stm32l4xx_hal.h"

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#define WREN 											0x06
#define WRDI 											0x04
#define PAGE_PROGRAM 									0x02

#define READ_PAGE 										0x03
#define READ_ID 										0x4b //4b


#define ERASE_ALL_COMAND 								0xC7
#define BLOCK_ERASE 									0x52
#define SECTOR_ERASE 									0x20

#define READ_STATUS_REG_1								0x05
#define READ_STATUS_REG_2								0x35
#define READ_STATUS_REG_3								0x15

#define WRITE_STATUS_REG_1								0x01

void Flash_read_identification_id();

void Write_flash_enable();

void flash_csn( uint8_t val);
/**
 * @brief Read the value of the status register 1
 * @param reg_to_save pointer where will be saved the value of the status register 1
 * @retval None
 */
void Flash_read_status1_register( uint8_t *reg_to_save);

/**
 * @brief Read the value of the status register 2
 * @param reg_to_save pointer where will be saved the value of the status register 2
 * @retval None
 */
void Flash_read_status2_register( uint8_t *reg_to_save);

/**
 * @brief Read the value of the status register 3
 * @param reg_to_save pointer where will be saved the value of the status register 3
 * @retval None
 */
void Flash_read_status3_register( uint8_t *reg_to_save);

/**
* @brief Flash_verificate_status
* Verificate if the Flash is able to receive a new write command
* @retval None
* */
void Flash_verificate_status();

/**
* @brief Flash_write_page
* This function write on a page
* @param address_1: more significant address byte of the memory page
* @param address_2: second more significant address byte of the memory page
* @param address_3: less significant address byte of the memory page
* @param value_to_write: direction to buffer to write
* @param length: length of the data to be write
* @retval None
* */
void Flash_write_page(uint8_t address_1, uint8_t address_2, uint8_t address_3, uint8_t *value_to_write, uint8_t length);


/**
* @brief Flash_read_page
* This function read a page an save the information in &read_flash_Byte which is defined as global variable
* @param address_1: more significant address byte of the memory page
* @param address_2: second more significant address byte of the memory page
* @param address_3: less significant address byte of the memory page
* @param aux_length: length of the data to be read
* @retval None
*
* */
void Flash_read_page(uint8_t address_1, uint8_t address_2, uint8_t address_3, uint8_t aux_length);

/**
* @brief Flash_sector_erase
* Erase all a sector of 4KB
* @param address_1: more significant address byte of the memory page
* @param address_2: second more significant address byte of the memory page
* @param address_3: less significant address byte of the memory page
* @retval None
*
* */
void Flash_sector_erase(uint8_t address_1, uint8_t address_2, uint8_t address_3);
/**
* @brief Block 32kb  erase
* Erase all the 32Kb data
* @param address_1: more significant address byte of the memory page
* @param address_2: second more significant address byte of the memory page
* @param address_3: less significant address byte of the memory page
* @retval None
*
* */
void Flash_block_erase(uint8_t address_1, uint8_t address_2, uint8_t address_3);

/**
* @brief erase_all
* This function erase all in the memory
* @retval None
* */
void erase_all ();

/**
* @brief Flash_activate_deactivate_block_protect
* deactivate the block protect of the Flash
* @retval None
* */
void Flash_activate_deactivate_block_protect();
#endif /* INC_FLASH_H_ */
