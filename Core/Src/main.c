/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include "string.h"

//#include "ssd1306.h"
//#include "ssd1306_fonts.h"

//#include "ring_buffer.h"
#include "keypad.h"
#include "tesla.h"
#include "val_clave.h"
#include "ring_buffer.h"
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//#define USART2_BUFFER_SIZE 8
//uint8_t usart2_buffer[USART2_BUFFER_SIZE];
//ring_buffer_t usart2_rb;
//uint8_t usart2_rx;
//
//uint32_t left_toggles = 0;
uint32_t left_last_press_tick = 0;
uint32_t right_last_press_tick = 0;
uint32_t stop_last_press_tick = 0;
uint8_t presion_doble = 0;
uint8_t presion = 0;
uint32_t cantidad_cambios1 = 0;
uint32_t cantidad_cambios2 = 0;
uint32_t cantidad_cambios3 = 0;
uint32_t tiempo_cambio;
uint8_t estacionamiento_previo = 0;
uint8_t numero_estacionamiento = 0;
volatile flag_enum presion_teclado;
volatile uint8_t tecla_presionada;
uint32_t tiempo_presion_num = 0;
boolean_enum state = FALSE;
boolean_enum eliminar_interrup = TRUE;
uint8_t clave[TAM_CLAVE];
ring_buffer_t ring_clave;
uint8_t validacion_clave = 0xFF;
uint8_t numeral_dos;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//int _write(int file, char *ptr, int len)
//{
//  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 10);
//  return len;
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Data received in USART2 */
//  if (huart->Instance == USART2) {
//	  usart2_rx = USART2->RDR; // leyendo el byte recibido de USART2
//	  ring_buffer_write(&usart2_rb, usart2_rx); // put the data received in buffer
//	  //HAL_UART_Receive_IT(&huart2, &usart2_rx, 1); // enable interrupt to continue receiving
//	  ATOMIC_SET_BIT(USART2->CR1, USART_CR1_RXNEIE); // usando un funcion mas liviana para reducir memoria
//  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{


	if(GPIO_Pin == S1_Pin && HAL_GetTick() > (left_last_press_tick + 200)){ // Ponemos los condicionales para
																			// para cada presión de los botones
		HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, 1); // Ponemos ambos leds en apagado, para que no interfiera en
		HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, 1); // en la siguiente instrucción dada
		presion = 1;
		cantidad_cambios1 = 6; //Asignamos el valor de 6 para indicar los 3 parpadeos que debe hacer
		estacionamiento_previo = 0;
		numero_estacionamiento = 0;
		HAL_UART_Transmit(&huart2, "Direccional izquierdo\r\n", 23, 10);
		if (HAL_GetTick() < (left_last_press_tick + 400)) {
			presion_doble = 1;
			cantidad_cambios1 = 0xFFFFFF; // Indicamos el valor 0xFFFFFF como indicador de un número infinito
		}
		left_last_press_tick = HAL_GetTick();
		return;

// Para los siguientes dos condicionales es el mismo funcionamiento que el explicado anteriormente
	} else if(GPIO_Pin == S2_Pin && HAL_GetTick() > (right_last_press_tick + 200)){ // El condicional del HAL_GetTick
																					// es para evitar el rebote
		HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, 1);
		HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, 1);
		presion = 2;
		cantidad_cambios2 = 6;
		estacionamiento_previo = 0;
		numero_estacionamiento = 0;
		HAL_UART_Transmit(&huart2, "Direccional derecho\r\n", 21, 10);
		if (HAL_GetTick() < (right_last_press_tick + 400)) {
			presion_doble = 2;
			cantidad_cambios2 = 0xFFFFFF;
		}
		right_last_press_tick = HAL_GetTick();
		return;

	}else if(GPIO_Pin == S3_Pin && HAL_GetTick() > (stop_last_press_tick + 200)){

		HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, 1);
		HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, 1);
		presion = 3;
		cantidad_cambios3 = 0xFFFFFF;
		if(	estacionamiento_previo == 0 && numero_estacionamiento > 0){ //Realizamos una condición para saber cuál
			estacionamiento_previo = 1;									//era el estado del estacionamiento y así
		}else{															//que se apague si estaba prendido o viceversa
			estacionamiento_previo = 0;
			numero_estacionamiento++;
		}
		HAL_UART_Transmit(&huart2, "Estacionamiento\r\n", 17, 10);
		stop_last_press_tick = HAL_GetTick();
		return;
	}

	uint8_t key_pressed = keypad_scan(GPIO_Pin);
	if (key_pressed != 0xFF ) {
		char* tx[15];
		uint8_t len=sprintf(&tx, "Se presionó: %c\r\n", key_pressed);
		HAL_UART_Transmit(&huart2, &tx, len, 10);
//		strcpy(&tecla_presionada, &key_pressed);
		tecla_presionada = key_pressed;
		presion_teclado = PRESION;
		return;

	}

}

//void low_power_mode()
//{
//#define AWAKE_TIME (10 * 1000) // 10 segundos
//	static uint32_t sleep_tick = AWAKE_TIME;
//
//	if (sleep_tick > HAL_GetTick()) {
//		return;
//	}
//	printf("Sleeping\r\n");
//	sleep_tick = HAL_GetTick() + AWAKE_TIME;
//
//	RCC->AHB1SMENR  = 0x0;
//	RCC->AHB2SMENR  = 0x0;
//	RCC->AHB3SMENR  = 0x0;
//
//	RCC->APB1SMENR1 = 0x0;
//	RCC->APB1SMENR2 = 0x0;
//	RCC->APB2SMENR  = 0x0;
//
//	/*Suspend Tick increment to prevent wakeup by Systick interrupt.
//	Otherwise the Systick interrupt will wake up the device within 1ms (HAL time base)*/
//	HAL_SuspendTick();
//
//	/* Enter Sleep Mode , wake up is done once User push-button is pressed */
//	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
//
//	/* Resume Tick interrupt if disabled prior to SLEEP mode entry */
//	HAL_ResumeTick();
//
//	printf("Awake\r\n");
//}
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  ring_buffer_init(&ring_clave, clave, TAM_CLAVE);
//  ssd1306_Init();
//  ssd1306_SetCursor(25, 30);
//  ssd1306_WriteString("Hello World!", Font_7x10, White);
//  ssd1306_UpdateScreen();
//
//  ring_buffer_init(&usart2_rb, usart2_buffer, USART2_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  presion_teclado = NO_PRESION;
  tecla_presionada = 0;
  numeral_dos = 0;
//  numero_estacionamiento = 0;
//  printf("Starting...\r\n");
  //HAL_UART_Receive_IT(&huart2, &usart2_rx, 1); // enable interrupt for USART2 Rx
//  ATOMIC_SET_BIT(USART2->CR1, USART_CR1_RXNEIE); // usando un funcion mas liviana para reducir memoria
  while (1) {

//	  char* tx2[15];
//		uint8_t len2=sprintf(&tx2, "PT=%d--Es=%d--TP=%c\r\n", presion_teclado, state, tecla_presionada);
//		HAL_UART_Transmit(&huart2, &tx2, len2, 100);

	  //Validamos la clave ingresada
	  if(presion_teclado == PRESION && tecla_presionada != '#' && tecla_presionada != '*'){//Para que escriba en el buffer
	  		  presion_teclado = NO_PRESION;													//cuando sea un valor valido
	  		  ring_buffer_write(&ring_clave, tecla_presionada);
	  }else if(presion_teclado == PRESION && tecla_presionada == '*'){//Resetear el buffer cuando se presione "*"
			  presion_teclado = NO_PRESION;
			  ring_buffer_reset(&ring_clave);
	  }else if(presion_teclado == PRESION && tecla_presionada == '#'){//Validar lo que esté en el buffer cuando
		  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  //se presione "#"
		  	  validacion_clave = validar_clave(&ring_clave);
		  	  ring_buffer_reset(&ring_clave);//De una limpie el buffer cuando valide
//		      printf("Validacion: %d\r\n", validacion_clave);
		  	  if(numeral_dos == 1){ //Este condicional nos sirve para indicar que la clave sigue siendo valida
				  validacion_clave = 0;//cuando se presione por segunda vez el numeral
			  }
		      if(validacion_clave == 1){ // Para que cambie el estado de la presion  incluso
		    	  presion_teclado = NO_PRESION;//incluso cuando no entre en el condicional siguiente (de la clave)
		      }
	  }


	  //Condicionamos el funcionamiento del Tesla (encendido o apagado)
	  if((tecla_presionada == '#') && (presion_teclado == PRESION) && (validacion_clave == 0)){

		  numeral_dos = 1;

		  if(state == FALSE){ // Para que si se presionó los botones de tesla mientras estaba bloqueado
			  cantidad_cambios1 = 0;//los resetee
			  cantidad_cambios2 = 0;
			  cantidad_cambios3 = 0;
		  }

		  HAL_UART_Transmit(&huart2, "funcionando\r\n", 13, 10);
		  if(eliminar_interrup == TRUE){// Es para corregir un error de que la variable no inicia en el mismo valor
			 tiempo_cambio = 1;//que se le definió globalmente
			 eliminar_interrup = FALSE;
		  }
		  state = TRUE; //Activamos en tesla

		  if( HAL_GetTick() < (tiempo_presion_num + 400) ){//Si se presiona el botón "#" seguido dos veces
			  HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, 1);// antes de 400 ms y está encendido, se apague
			  HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, 1);
			  HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, 1);
			  cantidad_cambios1 = 0;
			  cantidad_cambios2 = 0;
			  cantidad_cambios3 = 0;
			  HAL_UART_Transmit(&huart2, "no funcional\r\n", 14, 10);
			  state = FALSE;
			  numeral_dos = 0;
		  }

		  tiempo_presion_num = HAL_GetTick();
		  tecla_presionada = 0xFF;
		  presion_teclado = NO_PRESION;
		  validacion_clave = 0xFF;
	  }


	  if(state == TRUE)
	   {

//		  char* tx2[15];
//		uint8_t len2=sprintf(&tx2, "PT=%d--Es=%d--TP=%c\r\n", presion_teclado, state, tecla_presionada);
//		HAL_UART_Transmit(&huart2, &tx2, len2, 100);

		  heartbeat();

		  if(presion == 1 && cantidad_cambios1 > 0 && HAL_GetTick() > tiempo_cambio)
		  { // Si se presiona el botón de luz izquierda
			  if(cantidad_cambios2 == 0){ // Esta condición es para que se cumpla si está prendido la luz del otro lado, entonces apague ambas
				  HAL_GPIO_TogglePin(D3_GPIO_Port, D3_Pin);
				  cantidad_cambios1--;
				  tiempo_cambio = HAL_GetTick() + 500;
			  }else{
				  cantidad_cambios1 = 0;
				  cantidad_cambios2 = 0;
			  }
	  //El mismo funcionamiento del anterior está para este que es el led derecho
		  }	else if(presion == 2 && cantidad_cambios2 > 0 && HAL_GetTick() > tiempo_cambio)
		  { // Si se presiona el botón de luz derecha
			  if(cantidad_cambios1 == 0){
				  HAL_GPIO_TogglePin(D4_GPIO_Port, D4_Pin);
				  cantidad_cambios2--;
				  tiempo_cambio = HAL_GetTick() + 500;
			  }else{
				  cantidad_cambios1 = 0;
				  cantidad_cambios2 = 0;
			  }
	  // Y para el funcionamiento de la estacionaria es similar solo que con ambos leds
		  }else if(presion == 3 && cantidad_cambios3 > 0 && HAL_GetTick() > tiempo_cambio)
		  { // Si se presiona el botón de estacionar
			  if(estacionamiento_previo == 0){
				  HAL_GPIO_TogglePin(D3_GPIO_Port, D3_Pin);
				  HAL_GPIO_TogglePin(D4_GPIO_Port, D4_Pin);
				  cantidad_cambios3--;
				  tiempo_cambio = HAL_GetTick() + 500;
				  cantidad_cambios1 = 0;
				  cantidad_cambios2 = 0;
			  }else if(	estacionamiento_previo == 1){
				  cantidad_cambios3 = 0;
			  }
		  }
	  }


//	  if (ring_buffer_is_full(&usart2_rb) != 0) {
//		  printf("Received:\r\n");
//		  while (ring_buffer_is_empty(&usart2_rb) == 0) {
//			  uint8_t data;
//			  ring_buffer_read(&usart2_rb, &data);
//			  HAL_UART_Transmit(&huart2, &data, 1, 10);
//		  }
//		  printf("\r\n");
//	  }
//	  low_power_mode();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.BaudRate = 256000;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D1_Pin|ROW_1_Pin|D3_Pin|ROW_4_Pin
                          |ROW_3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ROW_2_GPIO_Port, ROW_2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : S1_Pin S2_Pin */
  GPIO_InitStruct.Pin = S1_Pin|S2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D1_Pin ROW_1_Pin D3_Pin ROW_4_Pin
                           ROW_3_Pin */
  GPIO_InitStruct.Pin = D1_Pin|ROW_1_Pin|D3_Pin|ROW_4_Pin
                          |ROW_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : S3_Pin */
  GPIO_InitStruct.Pin = S3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(S3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : COLUMN_3_Pin COLUMN_4_Pin COLUMN_1_Pin */
  GPIO_InitStruct.Pin = COLUMN_3_Pin|COLUMN_4_Pin|COLUMN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ROW_2_Pin */
  GPIO_InitStruct.Pin = ROW_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ROW_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : COLUMN_2_Pin */
  GPIO_InitStruct.Pin = COLUMN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(COLUMN_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : D4_Pin */
  GPIO_InitStruct.Pin = D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(D4_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
