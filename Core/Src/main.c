/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "Flash.h"
#include "ds18b20.h"
#include "machine_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t counter_1ms_a;
//UART
char rx_buffer[ RX_BUFFER_LEN ];
char tx_buffer[ TX_BUFFER_LEN ];
flag_states uart_flag;

//TEMP
uint8_t Temp_byte1, Temp_byte2;
uint16_t TEMP;
float Temperature = 0;
uint16_t Temp_int; //Temp como entero
uint8_t Presence = 0;

//Machine control (sensor)
uint8_t rx_position_counter;
uint16_t value;
uint16_t value_max;
uint16_t value_min;
uint16_t counter_1ms;

//FLASH
uint8_t transmit_text[ 64 ];
extern uint8_t read_flash_Byte[ 255 ];
volatile uint8_t address_to_write[3] ;
uint8_t things_to_write[280];
uint8_t total_things_to_write[280];

//PROMEDIO
float Temp_suma;
float Prom_temp;
uint8_t Conteo_Promedio = 0;
uint8_t Conteo_memoria;
uint8_t total_size;
uint8_t Temp_promedio; //Temperatura promedio como entero para guardar
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay (uint32_t us)
{
    __HAL_TIM_SET_COUNTER(&htim3,0); //Counter value at 0
    while ((__HAL_TIM_GET_COUNTER(&htim3))<us); //Wait for the counter to reach the value
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3); //Without interruption
  HAL_TIM_Base_Start_IT(&htim4); //With interruption

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //FLASH
  Flash_read_identification_id();
  Flash_activate_deactivate_block_protect();
  uint8_t status_reg1;
  uint8_t status_reg2;
  uint8_t status_reg3;

  Flash_read_status1_register( &status_reg1 );
  Flash_read_status2_register( &status_reg2 );
  Flash_read_status3_register( &status_reg3 );

  //Guardados
  //Frecuencia
  Flash_read_page(0x00, 0x32, 0x00, 3); //Leo la pagina 2 registro 3, no necesito más de 3 digitos
  value = atoi((char*)read_flash_Byte);//La lectura la guardo en el value Con esto me aseguro que si reinicio el nucleo me quede guardado el valor mas reciente guardado en la memoria

  //Temperatura maxima
  Flash_read_page(0x00, 0x42, 0x00, 3); //Leo la pagina 2 registro 4
  value_max = atoi((char*)read_flash_Byte);

  //Temperatura minima
  Flash_read_page(0x00, 0x52, 0x00, 3); //Leo la pagina 2 registro 5
  value_min = atoi((char*)read_flash_Byte);

  //Conteo memoria
  Flash_read_page(0x00, 0x12, 0x00, 3); //Leo la pagina 2 registro 1, solo leo 3 datos porque el numero solo llega a 60
  Conteo_memoria = atoi((char*)read_flash_Byte);

  //Total_size para ir moviendome en un solo registro
  Flash_read_page(0x00, 0x22, 0x00, 20); //Leo la pagina 2 registro 2
  total_size = atoi((char*)read_flash_Byte);

  //UART
  rx_position_counter = 0;
  HAL_UART_Receive_IT(&huart2, (uint8_t *) &rx_buffer[rx_position_counter], 1);

  uint16_t tx_len = sprintf(&tx_buffer[0], "Digite el periodo del led de 0 a 200[*100 ms] como #LT$VALOR$\r\nDigite el rango minimo de temperatura de 0 a 125[deg] como #TMPMIN$VALOR$\r\nDigite el rango maximo de temperatura de 0 a 125[deg] como #TMPMAX$VALOR$\r\nSi desea borrar la memoria digite BORRAR \r\nSi desea saber la temperatura promedio actual digite PROMEDIO \r\nSi desea saber la temperatura maxima guardada digite MAX \r\nSi desea saber la temperatura minima guardada digite MIN \r\nSi desea saber el periodo guardado digite PERIODO \r\nSi desea imprimir todos los promedios de temperatura guardados digite TOTAL \r\n");
  HAL_UART_Transmit(&huart2, (const uint8_t *)&tx_buffer[0], tx_len, 100);
  while (1)
  {

	  //TEMP
	  Presence = DS18B20_Start ();

	  DS18B20_Write (0xCC);  // skip ROM
	  DS18B20_Write (0x44);  // convert t

	  //if (counter_1ms_a >= 100){ //Interrupt para leer, en este caso de 100ms como lo pide el ejercicio
	  Presence = DS18B20_Start ();
	  DS18B20_Write (0xCC);  // skip ROM
	  DS18B20_Write (0xBE);  // Read Scratch-pad
	  Temp_byte1 = DS18B20_Read();
	  Temp_byte2 = DS18B20_Read();
	  TEMP = (Temp_byte2<<8)|Temp_byte1;
	  Temperature = (float)TEMP/16;
	  //}

	  Temp_suma += Temperature; //Voy sumando todos los valores de temperatura
	  Conteo_Promedio++; //Espero a 10 valores para hacer un promedio, cada valor toma 100ms en sacar (total 1s)

	  if (Conteo_Promedio == 10) {
		  Prom_temp = Temp_suma / 10;
		  uint8_t size_to_send;
		  uint16_t total_size_to_send;

	      //FLASH
	      if (Conteo_memoria <=  60) { //Me aseguro que solo guarde 60 datos
	    	  Flash_sector_erase(0x00, 0x12, 0x00); //Borro el dato anterior del conteo_memoria para luego sobreescribir
	    	  Flash_sector_erase(0x00, 0x22, 0x00); //Borro la memoria para sobreescribir el total_size


	    	  memset(&things_to_write, 0x00,280);//borra buffer para usarlo en el conteo
	    	  Conteo_memoria++;//Debo guardar este conteo en la flash para que siga en caso de reiniciar
	    	  size_to_send = sprintf((char*)things_to_write, "%d", Conteo_memoria);
	    	  Flash_write_page( 0x00, 0x12, 0x00 , &things_to_write[0] , size_to_send ); //Guardo el conteo en el registro 1 pag 2

	    	  memset(&things_to_write, 0x00,280);//borra buffer para usarlo en los promedios
	    	  Temp_promedio = Prom_temp; //Guardo como entero en la flash
	    	  size_to_send = sprintf((char*)things_to_write, "%d$", Temp_promedio); //No mando mas texto a la flash para no llenarla de basura, el $ me ayuda a separar los valores
	    	  total_size += size_to_send; //Voy corriendo los valores

	    	  //Aqui guardo el total_size en la flash para no sobreescribir datos en caso de reiniciar
	    	  memset(&total_things_to_write, 0x00,280);//borra buffer
	    	  total_size_to_send = sprintf((char*)total_things_to_write, "%d", total_size);
	    	  Flash_write_page( 0x00, 0x22, 0x00 , &total_things_to_write[0] , total_size_to_send ); //Guardo en registro 2 pag 2

	    	  Flash_write_page(0x00, 0x02, total_size, &things_to_write[0], size_to_send); //Escribo promedios en registro 0 pagina 2, el total_size es para no reescribir en donde ya tenia datos, luego solo necesito 1 registro para guardar los 60 datos q crack
	      }
	      else if (Conteo_memoria > 60){

	    	  tx_len = sprintf(&tx_buffer[0], "Se ha reiniciado la memoria de promedios de temperatura \r\n");
	    	  HAL_UART_Transmit(&huart2, (const uint8_t *)&tx_buffer[0], tx_len, 10);
	    	  Flash_sector_erase(0x00, 0x02, 0x00); //Borro desde el inicio 0x00 (TODOS)
	    	  total_size = -size_to_send; //Reinicio total_size, -size_to_send para que empiece en el byte 0
	    	  Conteo_memoria = 0; //Reinicio conteo despues de borrar
	      }
		  //Reset
		  Temp_suma = 0.0;
		  Conteo_Promedio = 0;

	  }
	  Flash_read_page(0x00, 0x02, 0x00, 255); //Siempre estoy leyendo la memoria de los datos de temperatura
	  HAL_Delay(100);

#ifdef UART_ACTIVE //Si mando cualquier señal por UART entro aquí

	  if (uart_flag == ACTIVATED){
		  compare_string(); //Para mandar valores de frecuencia
		  compare_max_temp(); //Para mandar valores de temp maxima
		  compare_min_temp(); //Para mandar valores de temp minima
		  get_avg_temp(); //Para temperatura promedio
		  get_max(); //Para temperatura maxima
		  get_min(); //Para temperatura minima
		  get_period(); //Para periodo
		  get_total(); //Para tener todos los promedios
		  erase_command(); //Para borrar flash
		  uart_flag = RELEASED;
	  }
#endif

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DS18B20_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DS18B20_Pin LD2_Pin */
  GPIO_InitStruct.Pin = DS18B20_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
/*
#ifdef BUTTON_ACTIVE
  if (GPIO_Pin == B1_Pin){
	  change_to_next_state();
  }
#endif
*/
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}
#ifdef UART_ACTIVE
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if ( huart == &huart2){
		rx_position_counter++;
		HAL_UART_Receive_IT(&huart2, (uint8_t *) &rx_buffer[rx_position_counter], 1);
		uart_flag = ACTIVATED;
	}

}
#endif
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
