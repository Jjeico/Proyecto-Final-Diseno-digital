/*
 * machine_control.c
 *
 *  Created on: May 11, 2023
 *      Author: Jacob
 */

#include "machine_control.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "Flash.h"

extern uint16_t counter_1ms;
extern UART_HandleTypeDef huart2;
extern char rx_buffer [RX_BUFFER_LEN] ;
extern uint8_t rx_position_counter;
extern uint16_t value;  //Con esto imprimo msj y vario frecuencia
extern uint16_t value_max;
extern uint16_t value_min;
extern float Prom_temp;

//FLASH
extern uint8_t transmit_text[ 64 ];
extern uint8_t read_flash_Byte[ 255 ];
extern uint8_t things_to_write[280];

extern uint8_t status_reg1;
extern uint8_t status_reg2;
extern uint8_t status_reg3;

extern uint8_t total_size;
extern uint8_t Conteo_memoria;


static void re_init_comunication (void){
	 HAL_UART_AbortReceive_IT(&huart2);
	 rx_position_counter = 0;
	 memset((void *) &rx_buffer[0], '\0', RX_BUFFER_LEN);
	 HAL_UART_Receive_IT(&huart2, (uint8_t *) &rx_buffer[rx_position_counter], 1);
}

void get_total(void){ //Para tener todos los promedios de temperatura
    char data_to_transmit[TX_BUFFER_LEN];
    uint8_t str_len; //Uart
    char* command = "TOTAL"; // Comando
    char* check_command = strstr(rx_buffer, command);	//Miro si el comando existe en el buffer

    if (check_command != NULL) {
    	Flash_read_page(0x00, 0x02, 0x00, 255); //Leo la pagina 2 registro 0
    	str_len = sprintf(&data_to_transmit[0], "%s", read_flash_Byte); //Del texto se encarga solo el uart
    	HAL_UART_Transmit(&huart2, (const uint8_t *)&data_to_transmit[0], str_len, 100);
    	re_init_comunication();
    }
}

void erase_command(void) { //Para borrar flash
      char data_to_transmit[TX_BUFFER_LEN];
      uint8_t str_len; //Uart
      char* command = "BORRAR"; // Comando
      char* check_command = strstr(rx_buffer, command);	//Miro si el comando existe en el buffer

      if (check_command != NULL) {
    	  str_len = sprintf(data_to_transmit, "Se ha borrado toda la informacion \r\n");	// Transmits message UART
    	  HAL_UART_Transmit(&huart2, (const uint8_t*)data_to_transmit, str_len, 10);
    	  Flash_block_erase(0x00, 0x02, 0x00); //Borro toda la pagina 2 (la unica que uso)
    	  Prom_temp = 0; //Pongo esto solo para antes de reiniciar, lo que pasa es que los borro de la memoria pero no del nucleo, para borrarlos tendria que reiniciarlo
    	  value_max = 0;
    	  value_min = 0;
    	  value = 0;
    	  Conteo_memoria = 0;
    	  total_size = -3;
    	  re_init_comunication();
      }
}

void get_avg_temp(void){ //Para obtener la temperatura promedio actual
    char data_to_transmit[TX_BUFFER_LEN];
    uint8_t str_len; //Uart
    char* command = "PROMEDIO"; // Comando
    char* check_command = strstr(rx_buffer, command);	//Miro si el comando existe en el buffer

    if (check_command != NULL) {
    	str_len = sprintf(data_to_transmit, "Temperatura promedio: %f C \r\n", Prom_temp); // Transmits message UART
    	HAL_UART_Transmit(&huart2, (const uint8_t*)data_to_transmit, str_len, 10);
    	re_init_comunication();
    }
}


void get_max(void){ //Para obtener el ultimo valor de maxima temperatura guardado
    char data_to_transmit[TX_BUFFER_LEN];
    uint8_t str_len; //Uart
    char* command = "MAX"; // Comando
    char* check_command = strstr(rx_buffer, command);	//Miro si el comando existe en el buffer

    if (check_command != NULL) {
  	  str_len = sprintf(data_to_transmit, "Temperatura maxima guardada: %d \r\n", value_max); // Transmits message UART
  	  HAL_UART_Transmit(&huart2, (const uint8_t*)data_to_transmit, str_len, 10);
  	  re_init_comunication();
    }
}

void get_min(void){ //Para obtener el ultimo valor de temperatura minima guardado
	char data_to_transmit[TX_BUFFER_LEN];
	uint8_t str_len; //Uart
	char* command = "MIN"; // Comando
	char* check_command = strstr(rx_buffer, command);	//Miro si el comando existe en el buffer

    if (check_command != NULL) {
    	str_len = sprintf(data_to_transmit, "Temperatura minima guardada: %d \r\n", value_min);	// Transmits message UART
    	HAL_UART_Transmit(&huart2, (const uint8_t*)data_to_transmit, str_len, 10);
    	re_init_comunication();
    }
}

void get_period(void){ //Para obtener el ultimo valor de periodo guardado
    char data_to_transmit[TX_BUFFER_LEN];
    uint8_t str_len; //Uart
    char* command = "PERIODO"; // Comando
    char* check_command = strstr(rx_buffer, command);	//Miro si el comando existe en el buffer

    if (check_command != NULL) {
    	float valuef = value;
    	str_len = sprintf(data_to_transmit, "Periodo guardado: %f \r\n", valuef/10);	// Transmits message UART
    	HAL_UART_Transmit(&huart2, (const uint8_t*)data_to_transmit, str_len, 10);
    	re_init_comunication();
    }
}



void compare_string(void) {
     char data_to_transmit[TX_BUFFER_LEN];
     uint8_t str_len; //Uart
     uint8_t size_to_send; //Flash
     memset(&things_to_write, 0x00,280);//borra buffer
     char* Init_limit = "#LT$"; // Init delimiter
     char* End_limit = "$"; // End delimiter

     char* Init_pos = strstr(rx_buffer, Init_limit);	// Search for init delimiter position

     if (Init_pos != NULL) {
         Init_pos += strlen(Init_limit); // Advances until #LT$, me ubico en el número
         char* End_pos = strstr(Init_pos, End_limit); // Looks for end limit $ so I have the complete number

         if (End_pos != NULL) {
        	 Flash_sector_erase(0x00, 0x32, 0x00); //Borro el sector 3 pag 2 para reescribir
             int num_len = End_pos - Init_pos;	// Calculates number len
             char num[num_len + 1];	// Gets the number
             strncpy(num, Init_pos, num_len);
             num[num_len] = '\0';
             value = atoi(num);	// Converts ascii to int
             float valuef = value; //Flotante para que me imprima periodos de 0.1 a 0.9 s en uart
			 str_len = sprintf(data_to_transmit, "Periodo: %f s \r\n", valuef/10);	// Transmits message UART
			 HAL_UART_Transmit(&huart2, (const uint8_t*)data_to_transmit, str_len, 10);
			 //FLASH
			 size_to_send = sprintf((char*) things_to_write, "%d", value ); //Para guardar en Flash SOLO el dato de frecuencia
			 Flash_write_page( 0x00, 0x32, 0x00 , &things_to_write[0] , size_to_send ); //Escribo en el registro 3 pagina 2 la frecuencia
			 //END FLASH
             re_init_comunication();
             counter_1ms = 0; //Para que apenas mande mi frecuencia empiece a funcionar el LED
         }

     }

}

void compare_max_temp(void){
    char data_to_transmit[TX_BUFFER_LEN];
    uint8_t str_len;
    uint8_t size_to_send; //Flash
    memset(&things_to_write, 0x00,280);//borra buffer
    char* Init_limit = "#TMPMAX$"; // Init delimiter
    char* End_limit = "$"; // End delimiter

    char* Init_pos = strstr(rx_buffer, Init_limit);	// Search for init delimiter position

    if (Init_pos != NULL) {
        Init_pos += strlen(Init_limit); // Advances until #TMPMAX$, me ubico en el número
        char* End_pos = strstr(Init_pos, End_limit); // Looks for end limit $ so I have the complete number

        if (End_pos != NULL) {
        	Flash_sector_erase(0x00, 0x42, 0x00); //Borro el sector 4 pag 2 para reescribir
            int num_len = End_pos - Init_pos;	// Calculates number len
            char num[num_len + 1];	// Gets the number
            strncpy(num, Init_pos, num_len);
            num[num_len] = '\0';
            value_max = atoi(num);	// Converts ascii to int
            str_len = sprintf(data_to_transmit, "Temperatura maxima: %d [deg] \r\n", value_max);	// Transmits message UART
            HAL_UART_Transmit(&huart2, (const uint8_t*)data_to_transmit, str_len, 10);
			//FLASH
			size_to_send = sprintf((char*) things_to_write, "%d", value_max ); //Para guardar en Flash SOLO el dato de temp max
			Flash_write_page( 0x00, 0x42, 0x00 , &things_to_write[0] , size_to_send ); //Escribo en el registro 4 pagina 2 la temp max
			//END FLASH
            re_init_comunication();
            counter_1ms=0;
        }

    }
}

void compare_min_temp(void){
    char data_to_transmit[TX_BUFFER_LEN];
    uint8_t str_len;
    uint8_t size_to_send; //Flash
    memset(&things_to_write, 0x00,280);//borra buffer
    char* Init_limit = "#TMPMIN$"; // Init delimiter
    char* End_limit = "$"; // End delimiter

    char* Init_pos = strstr(rx_buffer, Init_limit);	// Search for init delimiter position

    if (Init_pos != NULL) {
        Init_pos += strlen(Init_limit); // Advances until #TMPMIN$, me ubico en el número
        char* End_pos = strstr(Init_pos, End_limit); // Looks for end limit $ so I have the complete number

        if (End_pos != NULL) {
        	Flash_sector_erase(0x00, 0x52, 0x00); //Borro el sector 5 pag 2 para reescribir
            int num_len = End_pos - Init_pos;	// Calculates number len
            char num[num_len + 1];	// Gets the number
            strncpy(num, Init_pos, num_len);
            num[num_len] = '\0';
            value_min = atoi(num);	// Converts ascii to int
            str_len = sprintf(data_to_transmit, "Temperatura minima: %d [deg] \r\n", value_min);	// Transmits message UART
            HAL_UART_Transmit(&huart2, (const uint8_t*)data_to_transmit, str_len, 10);
			//FLASH
			size_to_send = sprintf((char*) things_to_write, "%d", value_min ); //Para guardar en Flash SOLO el dato de temp min
			Flash_write_page( 0x00, 0x52, 0x00 , &things_to_write[0] , size_to_send ); //Escribo en el registro 5 pagina 2 la temp min
			//END FLASH
            re_init_comunication();
            counter_1ms=0;
        }

    }
}
