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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef enum Sys_States_e {
	IDLE = 0, RECIEVE_USER_COMMAND, POWER_ON_DAC, START_ADC, TRANSMIT_ADC_CMD, POWER_OFF_DAC_ADC,
} Sys_States_e;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define POTENTIOSTAT_V2 1 //0 if STM32 and 1 for V2.0

#define RX_BUFFER_SIZE 10
#define TX_BUFFER_SIZE 25
#define FILTER_SIZE 1000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#if POTENTIOSTAT_V2==1
const float INITIAL_DAC_VOLTAGE=((0.901*4095)/3.3);
const float RES_IN_SERIES=92.36;
const float DAC2_VOLTAGE=(1.5*4095)/3.3;
const uint32_t MICRO_TO_NANO=100000;
#else
const float INITIAL_DAC_VOLTAGE=((1.245*4095)/3.3);
const float RES_IN_SERIES=128.6;
const uint32_t MICRO_TO_NANO=10000;
#endif

uint8_t rx_buffer[RX_BUFFER_SIZE]={0};
uint8_t tx_buffer[TX_BUFFER_SIZE]={0};
uint8_t timer_started=0;
uint32_t time_counter=0;
float adc_value=0;
int32_t current_value=0;
uint8_t first_run=0;
float applied_voltage=INITIAL_DAC_VOLTAGE;
uint32_t time_duration=0;
Sys_States_e operating_mode=IDLE;
uint32_t adc_buffer[FILTER_SIZE] = {0};
uint32_t buffer_index = 0;
uint32_t sum = 0;
uint32_t temp_adc[2]={0};
uint32_t filtered_adc_value=0;
uint32_t prev_filter_value=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void Mode_of_operation();
void ideal_state();
void check_for_uart_command();
void set_dac_voltage();
void start_adc();
void transmit_adc_data();
void reset_dac_and_adc();
void CaseDefaultHandler();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_DAC_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(!timer_started)
	  {
		  TIM3->SR &= ~TIM_SR_UIF;
		  HAL_TIM_Base_Start_IT(&htim3);
	  	  timer_started=1;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 48-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim3.Init.Prescaler = 48000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void moving_average_filter(uint32_t new_adc_value)
{
    sum -= adc_buffer[buffer_index];
    sum += new_adc_value;
    adc_buffer[buffer_index] = new_adc_value;
    buffer_index = (buffer_index + 1) % FILTER_SIZE;
    filtered_adc_value=(uint32_t)(sum / FILTER_SIZE);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim==&htim3)
	{
		time_counter++;
		Mode_of_operation();
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	adc_value=((temp_adc[0]*3300)/4095);
	moving_average_filter(adc_value);
}

void Mode_of_operation()
{
	switch (operating_mode)
	{
	case IDLE:
		ideal_state();
		break;

	case RECIEVE_USER_COMMAND:
		check_for_uart_command();
		break;

	case POWER_ON_DAC:
		set_dac_voltage();
		break;

	case START_ADC:
		start_adc();
		break;

	case TRANSMIT_ADC_CMD:
		transmit_adc_data();
		break;

	case POWER_OFF_DAC_ADC:
		reset_dac_and_adc();
		break;

	default:
		CaseDefaultHandler();
		break;

	}
}

void ideal_state()
{
	HAL_StatusTypeDef state=HAL_BUSY;
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, applied_voltage);
#if POTENTIOSTAT_V2==1
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2,DAC_ALIGN_12B_R, DAC2_VOLTAGE);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
#endif
	state=HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	if(state==HAL_OK)
	{
		//Switch to UART reception state
		operating_mode=RECIEVE_USER_COMMAND;
	}
}

void check_for_uart_command()
{
	HAL_StatusTypeDef state=HAL_BUSY;
	state=HAL_UART_Receive_IT(&huart2, rx_buffer, sizeof(rx_buffer));
	if(state==HAL_OK && first_run>0)
	{
		int32_t temp[2];
		uint8_t count=0;
		char *token;
		token=strtok(rx_buffer,",");
		while(token!=NULL && count<2)
		{
			temp[count]=atoi(token);
			count++;
			token=strtok(NULL,",");
		}
#if POTENTIOSTAT_V2==1
		applied_voltage=INITIAL_DAC_VOLTAGE-((temp[0]*4095)/3300);
#else
		applied_voltage=INITIAL_DAC_VOLTAGE-(((temp[0]*4095)/3300))*2;
#endif
		time_duration=temp[1]/50;
		first_run=0;
		operating_mode=POWER_ON_DAC;
	}
	else
	{
		operating_mode=RECIEVE_USER_COMMAND;
	}
	first_run++;
}

void set_dac_voltage()
{
	HAL_StatusTypeDef state=HAL_BUSY;
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R, applied_voltage);
#if POTENTIOSTAT_V2==1
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2,DAC_ALIGN_12B_R, DAC2_VOLTAGE);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
#endif
	state=HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	if(state==HAL_OK)
	{
		operating_mode=START_ADC;
	}
	else
	{
		operating_mode=POWER_ON_DAC;
	}
}

void start_adc()
{
	HAL_StatusTypeDef state=HAL_BUSY;
	HAL_ADCEx_Calibration_Start(&hadc);
	state=HAL_ADC_Start_DMA(&hadc, temp_adc, 2);
	HAL_TIM_Base_Start(&htim1);
	if(state==HAL_OK)
	{
		time_counter=0;
		operating_mode=TRANSMIT_ADC_CMD;
	}
	else
	{
		operating_mode=START_ADC;
	}
}

void transmit_adc_data()
{
	HAL_ADC_Stop_DMA(&hadc);
	HAL_TIM_Base_Stop(&htim1);
#if POTENTIOSTAT_V2==1
	current_value=(((filtered_adc_value/1.00319)-1494)/RES_IN_SERIES)*MICRO_TO_NANO;
#else
	current_value=((filtered_adc_value-1504)/RES_IN_SERIES)*MICRO_TO_NANO;
#endif
	sprintf(tx_buffer,"I=%ld,T=%lu,",current_value,(time_counter*50));
	HAL_UART_Transmit(&huart2, (uint8_t*)tx_buffer, sizeof(tx_buffer),100);
	HAL_ADC_Start_DMA(&hadc, temp_adc, 2);
	HAL_TIM_Base_Start(&htim1);

	if(time_counter>=time_duration)
	{
		operating_mode=POWER_OFF_DAC_ADC;
	}
	else
	{
		operating_mode=TRANSMIT_ADC_CMD;
	}

}

void reset_dac_and_adc()
{
	HAL_StatusTypeDef state=HAL_BUSY;
	state=HAL_ADC_Stop_DMA(&hadc);

	if(state==HAL_OK)
	{
		timer_started=0;
		HAL_TIM_Base_Stop(&htim1);
		HAL_TIM_Base_Stop_IT(&htim3);
		time_counter=0;
		memset(rx_buffer,0,sizeof(rx_buffer));
		__HAL_UART_FLUSH_DRREGISTER(&huart2);
		applied_voltage=INITIAL_DAC_VOLTAGE;
		current_value=0;
		operating_mode=IDLE;
	}
	else
	{
		operating_mode=POWER_OFF_DAC_ADC;
	}
}

void CaseDefaultHandler()
{
	operating_mode=IDLE;
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
