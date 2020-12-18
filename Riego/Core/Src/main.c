/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "i2c-lcd.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// PUERTOS DS18B20 - B
#define DS18B20_PORT GPIOB
#define DS18B20_PIN GPIO_PIN_11 // Datos  DS18B20 - Sensor de temperatura

// PUERTO DATOS DHT11 - A -- INACTIVO
//#define DHT11_PORT GPIOA
//#define DHT11_PIN GPIO_PIN_14     // Datos DHT11 - Sensor de temperatura y humedad

//MOTOR
#define MOTOR_PORT GPIOC
#define INI1_PIN GPIO_PIN_7
#define INI2_PIN GPIO_PIN_9


// PUERTO LEDS - E
#define PORT_LED GPIOE
#define LED_MANUAL GPIO_PIN_4     // Led indicador modo manual
#define LED_AUTOM GPIO_PIN_2   	// led indicador modo automatico

// PUERTO PUSH - D
#define PORT_PUSH GPIOD
#define PUSH GPIO_PIN_1    // Pulsador para cambiar de modo

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim9;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM9_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/*************************************************** FUNCIONES ******************************************/

/****************** DELAY CON TIM6 Y CLOCK INTERNO ******************/

void delay (uint16_t time)
{
	/* change your code here for the delay in microseconds */
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim6))<time);
}

/********************** SET PIN (INPUT Y OUTPUT)************************/

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/******************** FUNCIONES DISPLAY LCD ********************/

void Display_ModoAutom ()
{
	char str[20] = {0};
	lcd_put_cur(0, 2);

	sprintf (str, " MODO AUTOM. ");
	lcd_send_string(str);
}

void Display_ModoManual ()
{
	char str[20] = {0};
	lcd_put_cur(0, 2);

	sprintf (str, " MODO MANUAL ");
	lcd_send_string(str);
}

void Display_Temp (float Temp)
{
	char str[20] = {0};
	lcd_put_cur(1, 1);

	sprintf (str, "TEMP: %.2f ", Temp);
	lcd_send_string(str);
	lcd_send_data('C');
}

void Display_Pot (uint32_t Pot)
{
	char str[20] = {0};
	lcd_put_cur(1, 4);

	sprintf (str, "POT: %d ", Pot);
	lcd_send_string(str);
	lcd_send_data('%');
}


/************************ FUNCIONES DS18B20 *******************************/

uint8_t DS18B20_Start (void)
{
	uint8_t Response = 0;
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // set the pin as output
	HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin low
	delay (480);   // delay according to datasheet

	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);    // set the pin as input
	delay (80);    // delay according to datasheet

	if (!(HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))) Response = 1;    // if the pin is low i.e the presence pulse is detected
	else Response = -1;

	delay (400); // 480 us delay totally.

	return Response;
}

void DS18B20_Write (uint8_t data)
{
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1

			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
			delay (1);  // wait for 1 us

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // set as input
			delay (50);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0

			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
			delay (50);  // wait for 60 us

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
		}
	}
}

uint8_t DS18B20_Read (void)
{
	uint8_t value=0;

	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);

	for (int i=0;i<8;i++)
	{
		Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // set as output

		HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the data pin LOW
		delay (1);  // wait for > 1us

		Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // set as input
		if (HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		delay (50);  // wait for 60 us
	}
	return value;
}


/*********************** FUNCIONES DE MOTOR ****************************/
void avanceMotor(int s)
{
	//TIM9->CCR1=s;
	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, s);
	HAL_GPIO_WritePin(MOTOR_PORT, INI2_PIN,GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_PORT, INI1_PIN,GPIO_PIN_RESET);
}

void pareMotor()
{
	//TIM9->CCR1=s;
	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
	HAL_GPIO_WritePin(MOTOR_PORT, INI2_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_PORT, INI1_PIN,GPIO_PIN_RESET);
}

/*********************** PORCENTAJE DEL MOTOR *****************************/
int porcentajePot(int valPot)
{
	return (((valPot-430)*100)/(4095-430));
}

/*********************************************** VARIABLES ******************************************************/

/***************************** VARIABLES MODOS ************************/
// MODE 0 AUTOMATICO (PREDETERMIANDO O INICIAL)
// MODE 1 MANUAL
volatile int modo=0;

/*********************** VARIABLES DHT11 Y DS18B20 ******************/
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM, RH, TEMP;

float Temperature = 0;
float Humidity = 0;
uint8_t Presence = 0;

/*********************** VARIABLES DHT11 Y DS18B20 ******************/
int valPot;
int valPotPorcentaje;

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
  MX_I2C1_Init(); // PANTALLA LCD
  MX_TIM6_Init(); // RELOJ EXTERNO
  MX_TIM9_Init(); // PWM MOTOR
  MX_ADC1_Init(); // POTENCIOMETRO
  /* USER CODE BEGIN 2 */

  // PWM MOTOR
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

  // RELOJ INTERNO PARA DELAY
  HAL_TIM_Base_Start(&htim6);

  // INICIO DE LCD
   lcd_init();
   lcd_send_string("INICIANDO>>>>");
   HAL_Delay(2000);
   lcd_clear ();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(modo==0) // MODO AUTOMATICO
	  {
		  HAL_GPIO_WritePin(PORT_LED,LED_AUTOM,1); // Enciendo led autom
		  HAL_GPIO_WritePin(PORT_LED,LED_MANUAL,0);


		  /********* LCD ********/
		  Display_ModoAutom();
		  Display_Temp(Temperature);


		  /********************** DS18B20 ***********************/

		  Presence = DS18B20_Start ();
		  HAL_Delay (1);
		  DS18B20_Write (0xCC);  // skip ROM
		  DS18B20_Write (0x44);  // convert t
		  HAL_Delay (800);

		  Presence = DS18B20_Start ();
		  HAL_Delay(1);
		  DS18B20_Write (0xCC);  // skip ROM
		  DS18B20_Write (0xBE);  // Read Scratch-pad

		  Temp_byte1 = DS18B20_Read();
		  Temp_byte2 = DS18B20_Read();
		  TEMP = (Temp_byte2<<8)|Temp_byte1;
		  Temperature = (float)TEMP/16;


		  /****** MOTOR ********/
		  if(Temperature<25.0)
			  pareMotor();
		  else if((Temperature>=25.0)&&(Temperature<30.0))
			  avanceMotor(500); // (0-2000)
		  else if((Temperature>=30.0)&&(Temperature<35.0))
			  avanceMotor(1000); // (0-2000)
		  else if(Temperature>=35.0)
			  avanceMotor(1500); // (0-2000)


		  lcd_clear (); // REFRESCAMOS LA LCD
	  }
	  else if(modo==1)
	  {
		  HAL_GPIO_WritePin(PORT_LED,LED_MANUAL,1); // Enciendo led manual
		  HAL_GPIO_WritePin(PORT_LED,LED_AUTOM,0);

		  /********* LCD ********/
		  Display_ModoManual();
		  Display_Pot(valPotPorcentaje);
		  HAL_Delay(250);

		  /***** POTENCIOMETRO ******/

		  HAL_ADC_Start(&hadc1);
		  if(HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY)==HAL_OK)
		  {
			  valPot=HAL_ADC_GetValue(&hadc1);
		  }
		  valPotPorcentaje=porcentajePot(valPot);


		  /****** PWM MOTOR ******/
		  pareMotor();

		  if((valPotPorcentaje>=0)&&(valPotPorcentaje<20))
			  avanceMotor(200); // (0-2000)
		  else if((valPotPorcentaje>=20)&&(valPotPorcentaje<40))
			  avanceMotor(600); // (0-2000)
		  else if((valPotPorcentaje>=40)&&(valPotPorcentaje<60))
			  avanceMotor(1000); // (0-2000)
		  else if((valPotPorcentaje>=60)&&(valPotPorcentaje<80))
			  avanceMotor(1400); // (0-2000)
		  else if((valPotPorcentaje>=80)&&(valPotPorcentaje<=100))
			  avanceMotor(1800); // (0-2000)


		  lcd_clear (); // REFRESCAMOS LA LCD

	  }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 50-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xffff-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 500-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 2000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_AUTOM_Pin|LED_MANUAL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DATA_DS18B20_GPIO_Port, DATA_DS18B20_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MOTOR_INI1_Pin|MOTOR_INI2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_AUTOM_Pin LED_MANUAL_Pin */
  GPIO_InitStruct.Pin = LED_AUTOM_Pin|LED_MANUAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : DATA_DS18B20_Pin */
  GPIO_InitStruct.Pin = DATA_DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DATA_DS18B20_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_INI1_Pin MOTOR_INI2_Pin */
  GPIO_InitStruct.Pin = MOTOR_INI1_Pin|MOTOR_INI2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PULSADOR_MODE_Pin */
  GPIO_InitStruct.Pin = PULSADOR_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PULSADOR_MODE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	  if(GPIO_Pin==GPIO_PIN_1)
	  	{
		  if(HAL_GPIO_ReadPin(PORT_PUSH,PUSH)&&(modo==0)) 		// Si estoy en modo automatico
		  {
			  modo=1; 											// Cambio a modo manual
		  }
		  else if(HAL_GPIO_ReadPin(PORT_PUSH,PUSH)&&(modo==1)) 	// Si estoy en modo manual
		  {
			  modo=0; 											// Cambio a modo automatico
		  }
	  	}
}


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
