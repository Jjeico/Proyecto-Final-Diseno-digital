/*
 * machine_control.h
 *
 *  Created on: May 11, 2023
 *      Author: Jacob
 */

#ifndef INC_MACHINE_CONTROL_H_
#define INC_MACHINE_CONTROL_H_

#define LED_OFF 				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
#define LED_ON 					HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);


#include "main.h"

void get_total( void );
void erase_command( void );
void get_avg_temp( void );
void get_max( void );
void get_min( void );
void get_period( void );
void compare_string( void );
void compare_max_temp(void);
void compare_min_temp(void);



#endif /* INC_MACHINE_CONTROL_H_ */
