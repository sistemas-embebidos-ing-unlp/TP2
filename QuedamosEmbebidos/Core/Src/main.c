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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "math.h"
#include "stdint.h"
#include "stdio.h"
#include "fft.h"
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	ENC_IDLE,
    CW00,
	CW01,
	CW10,
	CCW00,
	CCW01,
	CCW10
} EncoderState;

typedef enum
{
	STATE_A_LOW_B_LOW,
	STATE_A_LOW_B_HIGH,
	STATE_A_HIGH_B_LOW,
	STATE_A_HIGH_B_HIGH
}EncoderEvento;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NSEN 100
#define PI 3.14159265
#define BUFFER_SIZE 256
#define FFT_SIZE 256

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim1_up;

UART_HandleTypeDef huart1;

/* Definitions for Bornera */
osThreadId_t BorneraHandle;
const osThreadAttr_t Bornera_attributes = {
  .name = "Bornera",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for UI */
osThreadId_t UIHandle;
const osThreadAttr_t UI_attributes = {
  .name = "UI",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DAC */
osThreadId_t DACHandle;
const osThreadAttr_t DAC_attributes = {
  .name = "DAC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Semaforo1 */
osSemaphoreId_t Semaforo1Handle;
const osSemaphoreAttr_t Semaforo1_attributes = {
  .name = "Semaforo1"
};
/* USER CODE BEGIN PV */

/* VARIABLES PARA LA UI*/
volatile uint8_t Screen = 1;
volatile uint8_t ConfSub = 0;
volatile uint8_t Boton2 = 0;
volatile int encoder_c = 0;
volatile int enc_uart = 0;
volatile int enc_graf = 0;
int encoder_position = 0;
int flag_salto_screen=0;
int flag_cursor_quieto=0;

/*FLAG PARA SUBMAQUINA 1*/
volatile uint8_t flag_frec = 0;
volatile uint8_t flag_amp = 0;
uint8_t flag_enc = 0;
uint8_t flag_uart = 0;


/*VARIABLES QUE SE MUESTRAN */
uint8_t amplitud = 255;
float frecuencia = 5.0;
uint8_t sine_wave[NSEN] = {0};
uint16_t adc_buffer[BUFFER_SIZE] = {0};
struct cmpx fft[FFT_SIZE];
float fft_R[128];
uint16_t f[128];
uint8_t cont = 0;

/*VARIABLES PARA EL STACK*/
uint16_t FreeBornera,FreeUI,FreeHeap,FreeDAC=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
void StartBorneraTask(void *argument);
void StartTaskUI(void *argument);
void StartDAC(void *argument);

/* USER CODE BEGIN PFP */

void SSD1306_FillArea(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t color);	// RECTANGULO BLANCO DE LA OPCION
void Encoder_UpdatePosition(void); 														// DEVUELVE CAMBIO DEL ENCODER
void Init_PWM_DMA(void);																// INCIA LA PWM Y GENERA EL REQUEST DEL DMA
void DibujarFFT(float *data);


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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  SSD1306_Init();
  Init_PWM_DMA();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of Semaforo1 */
  Semaforo1Handle = osSemaphoreNew(1, 1, &Semaforo1_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Bornera */
  BorneraHandle = osThreadNew(StartBorneraTask, NULL, &Bornera_attributes);

  /* creation of UI */
  UIHandle = osThreadNew(StartTaskUI, NULL, &UI_attributes);

  /* creation of DAC */
  DACHandle = osThreadNew(StartDAC, NULL, &DAC_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
  hi2c1.Init.ClockSpeed = 400000;
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
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void callback_in(int tag)
{
	switch (tag)
	{
		case TAG_TASK_TAREA0: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); break;
		case TAG_TASK_UI: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); break;
		case TAG_TASK_BORNERA: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); break;
		case TAG_TASK_DAC: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); break;
	}
}
void callback_out(int tag)
{
	switch (tag)
	{
		case TAG_TASK_TAREA0: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); break;
		case TAG_TASK_UI: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); break;
		case TAG_TASK_BORNERA: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); break;
		case TAG_TASK_DAC: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); break;
	}
}

void Init_PWM_DMA(void)
	{
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		HAL_DMA_Start(&hdma_tim1_up, (uint32_t) sine_wave,(uint32_t)&(TIM2->CCR1), NSEN);
		__HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
		HAL_TIM_Base_Start(&htim1);

		HAL_TIM_Base_Start(&htim3);
		HAL_ADCEx_Calibration_Start(&hadc1);  //calibrar el ADC
		HAL_ADC_Start_IT(&hadc1); // Iniciar ADC con interrupciones
	}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
		adc_buffer[cont] = HAL_ADC_GetValue(&hadc1);
		cont ++;
		if(cont == 255){
		HAL_ADC_Stop_IT(&hadc1);
		osSemaphoreRelease(Semaforo1Handle);   // LIBERO SEMAFORO PORQUE TENGO LAS MUESTRAS PARA GRAFICAR
		cont = 0;
		}
	}


void Encoder_UpdatePosition(void)
{
		static EncoderState encoder_state = ENC_IDLE; 			// VARIABLE ESTATICA
		uint8_t A_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
		uint8_t B_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
		EncoderEvento encoder_evento;

        // Determinar el nuevo evento del encoder
        switch ((A_state << 1) | B_state)
        {
            case 0b00:
            	encoder_evento = STATE_A_LOW_B_LOW;
                break;
            case 0b01:
            	encoder_evento = STATE_A_LOW_B_HIGH;
                break;
            case 0b10:
            	encoder_evento = STATE_A_HIGH_B_LOW;
                break;
            case 0b11:
            	encoder_evento = STATE_A_HIGH_B_HIGH;
                break;
            default:
                return;
        }
        switch (encoder_state)
        {
        	case ENC_IDLE:
        		switch(encoder_evento)
        		{
        			case STATE_A_LOW_B_HIGH: encoder_state = CW01;  break;
        			case STATE_A_HIGH_B_LOW: encoder_state = CCW10; break;
        			default: break;
        		}
        		break;
        	case CW01:
        		switch(encoder_evento)
        		{
        			case STATE_A_HIGH_B_HIGH: encoder_state = ENC_IDLE; break;
        			case STATE_A_LOW_B_LOW: encoder_state = CW00; break;
        			default: break;
        		}
        		break;
        	case CW00:
        		switch(encoder_evento)
        		{
        			case STATE_A_HIGH_B_HIGH: encoder_state = ENC_IDLE; break;
        			case STATE_A_LOW_B_HIGH: encoder_state = CW01; break;
        			case STATE_A_HIGH_B_LOW: encoder_state = CW10; break;
        			default: break;
        		}
        		break;
        	case CW10:
        		switch(encoder_evento)
        		{
        			case STATE_A_HIGH_B_HIGH:
        				if(flag_cursor_quieto==0)
        				{
        					encoder_position++;
        				}
        					if (flag_frec == 1)
        					{
        						frecuencia = frecuencia + 0.1;
        						flag_frec = 0;
        					}
							if (amplitud < 255 && flag_amp == 1)
							{
								amplitud ++;
								flag_amp = 0;
							}
        				if(flag_enc == 1)
        				{
        					encoder_c = (encoder_c + 1) % 2;
        					flag_enc = 0;
        				}
        				if(flag_uart == 1)
        				{
        					enc_uart = (enc_uart + 1) % 2;;
        					flag_uart = 0;
        				}
        				encoder_state = ENC_IDLE;
        			break;
        			case STATE_A_LOW_B_LOW: encoder_state = CW00; break;
        			default: break;
        		}
        		break;
        	case CCW00:
        		switch(encoder_evento)
        		{
        			case STATE_A_HIGH_B_HIGH: encoder_state = ENC_IDLE;  break;
        			case STATE_A_LOW_B_HIGH: encoder_state = CCW01; break;
        			case STATE_A_HIGH_B_LOW: encoder_state = CCW10; break;
        			default: break;
        		}
        		break;
        	case CCW01:
        		switch(encoder_evento)
        		{
        			case STATE_A_LOW_B_LOW: encoder_state = CCW00; break;
        			case STATE_A_HIGH_B_HIGH:
        				if(flag_cursor_quieto==0)
        				{
        					encoder_position--;
        				}
        					if (flag_frec == 1)
        					{
        						frecuencia = frecuencia - 0.1;
        						flag_frec = 0;
        					}
							if(amplitud > 0 && flag_amp == 1)
							{
								amplitud -- ;
								flag_amp = 0;
							}

        				if(flag_enc == 1)
        				{
        					encoder_c = (encoder_c + 1) % 2;
        					flag_enc = 0;
        				}
        				if(flag_uart == 1)
        				{
        					enc_uart = (enc_uart + 1) % 2;;
        					flag_uart = 0;
        				}

        				encoder_state = ENC_IDLE;
        			break;
        			default: break;
        		}
        		break;
        	case CCW10:
        		switch(encoder_evento)
        		{
        			case STATE_A_HIGH_B_HIGH: encoder_state = ENC_IDLE;  break;
        			case STATE_A_LOW_B_LOW: encoder_state = CCW00; break;
        			default: break;
        		}
        		break;
            }
        // LIMITO VALOR 0 y 128 (TAMAÑO DE LA PANTALLA)
        if (encoder_position < 0)
        {
        	encoder_position = 0;
        }
        else if (encoder_position > 127)
        {
        	encoder_position = 127;
        }

        // LIMITO VALOR ENTRE 10 y 500
        if (frecuencia < 0.1)
        {
        	frecuencia = 0.1;
        }
        else if (frecuencia > 50)
        {
        	frecuencia = 50;
        }
}

void SSD1306_FillArea(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t color)
{
    for (uint16_t i = x; i < x + w; i++)
    {
        for (uint16_t j = y; j < y + h; j++)
        {
            SSD1306_DrawPixel(i, j, color);
        }
    }
}

void DibujarFFT(float *data)
	{
		uint8_t prevY = 63 - (uint8_t)(data[0]*38.1818);
		if (prevY < 0) prevY = 0;
		if (prevY > 63) prevY = 63;
		for (int x = 1; x < 128; x++){
			uint8_t y = 63 - (uint8_t)(data[x]*38.1818);
			if (y < 0) y = 0;
			if (y > 63) y = 63;
			SSD1306_DrawLine(x-1, prevY, x, y, SSD1306_COLOR_WHITE);
			prevY = y;
		}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartBorneraTask */
/**
  * @brief  Function implementing the Bornera thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBorneraTask */
void StartBorneraTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	vTaskSetApplicationTaskTag( NULL, (void*) TAG_TASK_BORNERA);
  /* Infinite loop */
  for(;;)
  {
	  Encoder_UpdatePosition();
	  //LOGICA BOTON ENCODER
	  static uint32_t last_screen_change_time = 0;
	  uint32_t current_time_screen = HAL_GetTick();
	  if ((current_time_screen - last_screen_change_time) > 200){
		  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET){
	  			  Screen = (Screen + 1) % 3;
	  			  last_screen_change_time = current_time_screen;
		  }
	  }
	  //LOGICA BOTON CONFSUB
	  static uint32_t last_conf_change_time = 0;
	  uint32_t current_time_conf = HAL_GetTick();
	  if ((current_time_conf - last_conf_change_time) > 200){
		  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET)
	  		  {
	  			  ConfSub = (ConfSub + 1) % 4;
	  			  last_conf_change_time = current_time_conf;
	  		  }
	  }
	  static uint32_t last_conf2_change_time = 0;
	  uint32_t current_time_conf2 = HAL_GetTick();
	  if ((current_time_conf2 - last_conf2_change_time) > 200){
		  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET){
	  			  Boton2 = (Boton2 + 1) % 2;
	  			  last_conf2_change_time = current_time_conf2;
		  }
	  }
	  FreeBornera= 4*osThreadGetStackSpace(BorneraHandle);
	  osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskUI */
/**
* @brief Function implementing the UI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskUI */
void StartTaskUI(void *argument)
{
  /* USER CODE BEGIN StartTaskUI */
	vTaskSetApplicationTaskTag( NULL, (void*) TAG_TASK_UI);
	char frecuencia_display[4];
	char amplitud_display[6];
	char fft_buffer_display[50];
	char amplitud_pantalla[6];
	char frec_pantalla[6];
	int integerPart_A=0;
	int decimalPart_A=0;
	int integerPart_F=0;
	int decimalPart_F=0;
	int integerPart;
	int decimalPart;

  /* Infinite loop */
  for(;;)
  {
	SSD1306_Fill(0);
	flag_enc = 0;
	flag_uart = 0;
	flag_frec = 0;
	flag_amp = 0;
	integerPart = (amplitud*330/255) / 100;
	decimalPart = (amplitud*330/255) % 100;
	itoa(frecuencia*10, frecuencia_display, 10);	//ACA LA FRECUENCIA PASA A SER 50 , MULTIPLICO POR 10 Y GUARDO EN ARREGLO
	switch(Screen)
	{
	   case 0:
		if (osSemaphoreAcquire(Semaforo1Handle, 1000) == osOK){
	  		for (int i = 0; i < BUFFER_SIZE; i++){
	  			fft[i].real = adc_buffer[i] * 3.3 / 4096.0; //ESCALAMIENTO ADC 12 BITS
	  			fft[i].imag = 0;
	  		}
	  		FFT(fft, BUFFER_SIZE);
	  		for(int i = 0; i < 128; i++){
	  			if(i != 0 && i != 128){
	  			fft_R[i] = (sqrt(fft[i].real*fft[i].real+fft[i].imag*fft[i].imag)/256)*2; //ESCALAMIENTO DESPUES DE HACER LA FFT
	  			}
	  			else{
	  			fft_R[i] = (sqrt(fft[i].real*fft[i].real+fft[i].imag*fft[i].imag)/256);
	  			}

	  			f[i] = 10000*i/255;	 // RESHAPE
	  		}
	  		DibujarFFT(fft_R);
	  		HAL_ADC_Start_IT(&hadc1);
	  		osSemaphoreRelease(Semaforo1Handle);
	  	}
		//MUESTRA EN PANTALLA LA CONFIGURACION DEL CURSOR
		integerPart_A = abs(fft_R[encoder_position]*1000) / 1000;
	  	decimalPart_A = abs(fft_R[encoder_position]*1000) % 1000;
	  	snprintf(amplitud_pantalla, sizeof(amplitud_pantalla), "%d.%03d", integerPart_A, decimalPart_A);
	  	integerPart_F = f[encoder_position] / 10;
	  	decimalPart_F = f[encoder_position] % 10;
	  	snprintf(frec_pantalla, sizeof(frec_pantalla), "%03d.%01d", integerPart_F, decimalPart_F);
	  	if(enc_graf == 1){
	  		SSD1306_DrawLine(encoder_position, 0, encoder_position, 64, SSD1306_COLOR_WHITE);
	  	}
	  	SSD1306_GotoXY (70, 0);
	  	SSD1306_Puts (amplitud_pantalla, &Font_7x10, 1);
		SSD1306_GotoXY (70, 10);
		SSD1306_Puts (frec_pantalla, &Font_7x10, 1);
		SSD1306_GotoXY (110, 0);
		SSD1306_Puts("V", &Font_7x10, 1);
		SSD1306_GotoXY (110, 10);
		SSD1306_Puts("Hz", &Font_7x10, 1);
		if(ConfSub == 1){
			flag_frec = 1;
	  	}
	  	if(ConfSub == 2){
	  	flag_amp = 1;
	  	}
	  	if (Boton2== 0 && flag_salto_screen==1){
	  		Screen=1;
	  		flag_salto_screen==0;
	  		flag_cursor_quieto=0;
	  	}
	  break;
	  case 1:
		SSD1306_GotoXY(0, 0);
	  	SSD1306_Puts("Configuracion", &Font_7x10, 1);
	  	SSD1306_GotoXY(0, 20);
	  	SSD1306_Puts("Cursor", &Font_7x10, 1);
	  	SSD1306_GotoXY(0, 30);
	  	SSD1306_Puts("Frecuencia:", &Font_7x10, 1);
	  	SSD1306_GotoXY(0, 40);
	  	SSD1306_Puts("Amplitud:", &Font_7x10, 1);
	  	SSD1306_GotoXY(0, 50);
	  	SSD1306_Puts("Imp por UART:", &Font_7x10, 1);
	  	//SWITCH DE LA PSEUDOMAQUINA DEL SCREEN1
	  	switch (ConfSub)
	  	{
	  		case 0:
	  			flag_enc = 1;
	  			flag_uart = 0;
	  			flag_frec = 0;
	  			flag_amp = 0;
	  			SSD1306_FillArea(0, 20, SSD1306_WIDTH, 10, 1);
	  			SSD1306_GotoXY(0, 20);
				SSD1306_Puts("Cursor", &Font_7x10, 0);
				SSD1306_GotoXY(70, 20);
	  			if (encoder_c == 1){
	  				SSD1306_Puts("si", &Font_7x10, 0);
	  				enc_graf = 1;
	  				flag_salto_screen=1;
	  				flag_cursor_quieto=0;
	  				}
	  			else if (encoder_c == 0){
	  				SSD1306_Puts("no", &Font_7x10, 0);
	  				enc_graf = 0;
	  				flag_salto_screen=0;
	  				flag_cursor_quieto=1;
	  			}
	  			if (Boton2== 1 && flag_salto_screen==1)
	  			{
	  				Screen=0;
	  			}
	  		break;
	  		case 1:
	  			flag_enc = 0;
	  			flag_uart = 0;
	  			flag_frec = 1;
	  			flag_amp = 0;
	  			flag_cursor_quieto=1;
	  			SSD1306_FillArea(0, 30, SSD1306_WIDTH, 10, 1);
	  			SSD1306_GotoXY(0, 30);
	  			SSD1306_Puts("Frecuencia:", &Font_7x10, 0);
	  			SSD1306_GotoXY(90, 30);
	  			SSD1306_Puts(frecuencia_display, &Font_7x10, 0);
	  			flag_salto_screen=1;
	  			if (Boton2== 1 && flag_salto_screen==1){
	  				Screen=0;
	  			}
	  		break;
	  		case 2:
	  			flag_enc = 0;
	  			flag_uart = 0;
	  			flag_frec = 0;
	  			flag_amp = 1;
	  			flag_cursor_quieto=1;
	  			SSD1306_FillArea(0, 40, SSD1306_WIDTH, 10, 1);
	  			SSD1306_GotoXY(0, 40);
	  			SSD1306_Puts("Amplitud:", &Font_7x10, 0);
	  			SSD1306_GotoXY(90, 40);
	  			snprintf(amplitud_display, sizeof(amplitud_display), "%d.%02d", integerPart, decimalPart);
	  			SSD1306_Puts(amplitud_display, &Font_7x10, 0);
	  			SSD1306_Puts(frecuencia_display, &Font_7x10, 0);
	  			flag_salto_screen=1;
	  			if (Boton2== 1 && flag_salto_screen==1)
	  			{
	  				flag_cursor_quieto=1;
	  				Screen=0;
	  			}

	  		break;
	  		case 3:
	  			flag_enc = 0;
	  			flag_uart = 1;
	  			flag_frec = 0;
	  			flag_amp = 0;
	  			flag_cursor_quieto=1;
	  			SSD1306_FillArea(0, 50, SSD1306_WIDTH, 10, 1);
	  			SSD1306_GotoXY(0, 50);
	  			SSD1306_Puts("Imp por UART:", &Font_7x10, 0);
	  			SSD1306_GotoXY(100, 50);
	  			if (enc_uart == 1){
	  				SSD1306_Puts("Si", &Font_7x10, 0);
	  				for (int i = 0; i < 128; i++){
	  					int integerPartR = (fft[i].real*10000) / 10000;
	  					int decimalPartR = (uint32_t)(fft[i].real*10000) % 10000;
	  					int integerPartI = (fft[i].imag*10000) / 10000;
	  					int decimalPartI = (uint32_t)(fft[i].imag*10000) % 10000;
	  					snprintf(fft_buffer_display, sizeof(fft_buffer_display), "%d.%04d + %d.%04di \r\n", integerPartR, decimalPartR, integerPartI, decimalPartI);
	  					HAL_UART_Transmit(&huart1, (uint8_t *)fft_buffer_display, strlen(fft_buffer_display), HAL_MAX_DELAY);
	  				}
	  			enc_uart = 0;
	  			}
	  			else if (enc_uart == 0)
	  			{
	  			SSD1306_Puts("No", &Font_7x10, 0);
	  			}

	  		break;
	  	}
	  	break;
	  	case 2:
	  		flag_enc = 0;
	  		flag_uart = 1;
	  		flag_frec = 0;
	  		flag_amp = 0;
	  		flag_salto_screen=0;
	  		snprintf(amplitud_pantalla, sizeof(amplitud_pantalla), "%d.%03d", integerPart_A, decimalPart_A);
	  		snprintf(frec_pantalla, sizeof(frec_pantalla), "%03d.%01d", integerPart_F, decimalPart_F);
	  		snprintf(amplitud_display, sizeof(amplitud_display), "%d.%02d", integerPart, decimalPart);
	  		itoa(frecuencia*10, frecuencia_display, 10);
	  		SSD1306_GotoXY(10,0);
	  		SSD1306_Puts("Informacion", &Font_7x10, 1);
	  		SSD1306_GotoXY(0, 10);
	  		SSD1306_Puts("Cursor:", &Font_7x10, 1);
	  		SSD1306_GotoXY(0, 40);
	  		SSD1306_Puts("Generador de senal:", &Font_7x10, 1);
	  		SSD1306_GotoXY(0, 20);
	  		SSD1306_Puts (amplitud_pantalla, &Font_7x10, 1);
	  		SSD1306_GotoXY (60, 20);
	  		SSD1306_Puts (frec_pantalla, &Font_7x10, 1);
	  		SSD1306_GotoXY(0, 50);
	  		SSD1306_Puts (amplitud_display, &Font_7x10, 1);
	  		SSD1306_GotoXY (70, 50);
	  		SSD1306_Puts (frecuencia_display, &Font_7x10, 1);
	  		SSD1306_GotoXY(40, 20);
	  		SSD1306_Puts ("V  ", &Font_7x10, 1);
	  		SSD1306_GotoXY(40, 50);
	  		SSD1306_Puts ("V", &Font_7x10, 1);
	  		SSD1306_GotoXY (100, 20);
	  		SSD1306_Puts ("Hz ", &Font_7x10, 1);
	  		SSD1306_GotoXY (100, 50);
	  		SSD1306_Puts ("Hz", &Font_7x10, 1);
	  	break;
	  	}
	  	SSD1306_UpdateScreen();
	  	FreeUI= 4*osThreadGetStackSpace(UIHandle);
	  	osDelay(80);
  }
  /* USER CODE END StartTaskUI */
}

/* USER CODE BEGIN Header_StartDAC */
/**
* @brief Function implementing the DAC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDAC */
void StartDAC(void *argument)
{
  /* USER CODE BEGIN StartDAC */
	vTaskSetApplicationTaskTag( NULL, (void*) TAG_TASK_DAC);
  /* Infinite loop */
  for(;;)
  {
	  for(int i = 0; i < NSEN ; i++){
	  		  sine_wave[i] = (uint8_t)((float)amplitud/2*(sin(2.0*PI*i*frecuencia/(float)NSEN)+1.0));
	  }
	  FreeDAC=4*osThreadGetStackSpace(DACHandle);
	  FreeHeap=xPortGetFreeHeapSize();
	  osDelay(10);

  }
  /* USER CODE END StartDAC */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
