/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "oled.h"
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for EncoderTask */
osThreadId_t EncoderTaskHandle;
const osThreadAttr_t EncoderTask_attributes = {
  .name = "EncoderTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for show_OLED */
osThreadId_t show_OLEDHandle;
const osThreadAttr_t show_OLED_attributes = {
  .name = "show_OLED",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Gyrotask */
osThreadId_t GyrotaskHandle;
const osThreadAttr_t Gyrotask_attributes = {
  .name = "Gyrotask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UltrasoundTask */
osThreadId_t UltrasoundTaskHandle;
const osThreadAttr_t UltrasoundTask_attributes = {
  .name = "UltrasoundTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void StartDefaultTask(void *argument);
void motor(void *argument);
void encoder_task(void *argument);
void show(void *argument);
void GyroTask(void *argument);
void Ultrasound(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t aRxBuffer[4] = "1234";
uint8_t aTxBuffer[5];
int defaultPower = 2000;
int motorcmd = 0;
uint8_t buff[20];
uint8_t ICMAddr = 0x68;
double total_angle = 0;
double global_angle = 0;
int total_distanceA ,total_distanceB;
uint8_t disA[20];
uint8_t disB[20];
uint8_t speed[20];
int wheelDist;

double averageVolt1 = 0; //newnew
double averageVolt2 = 0; //newnew
int sFlag = 0;
int vFlag = 0;

void readByte(uint8_t addr, uint8_t* data){
	buff[0] = addr;
	HAL_I2C_Master_Transmit(&hi2c1, ICMAddr<<1, buff, 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, ICMAddr<<1, data, 2, 20);

}

void writeByte(uint8_t addr, uint8_t data){
	buff[0] = addr;
	buff[1] = data;
	HAL_I2C_Master_Transmit(&hi2c1, ICMAddr << 1, buff, 2, 20);
}

void gyroStart(){
	writeByte(0x07, 0x07);
	osDelay(10);
	writeByte(0x07, 0x00);
	osDelay(10);

}
void gyroInit(){

	writeByte(0x06, 0x00);
	osDelay(10);
	writeByte(0x03, 0x80);
	osDelay(10);
	writeByte(0x07, 0x07);
	osDelay(10);
	writeByte(0x06, 0x01);
	osDelay(10);
	writeByte(0x7F, 0x20);
	osDelay(10);
	writeByte(0x01, 0x2F);
	osDelay(10);
	writeByte(0x0, 0x00);
	osDelay(10);
	writeByte(0x7F, 0x00);
	osDelay(10);
	writeByte(0x07, 0x00);
	osDelay(10);

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
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  HAL_UART_Receive_IT(&huart3, (uint8_t *) aRxBuffer,sizeof(aRxBuffer));
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(motor, NULL, &MotorTask_attributes);

  /* creation of EncoderTask */
  EncoderTaskHandle = osThreadNew(encoder_task, NULL, &EncoderTask_attributes);

  /* creation of show_OLED */
  show_OLEDHandle = osThreadNew(show, NULL, &show_OLED_attributes);

  /* creation of Gyrotask */
  GyrotaskHandle = osThreadNew(GyroTask, NULL, &Gyrotask_attributes);

  /* creation of UltrasoundTask */
  UltrasoundTaskHandle = osThreadNew(Ultrasound, NULL, &UltrasoundTask_attributes);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|LED_DC_Pin
                          |LED3_Pin|CIN1_Pin|Gyro_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin LED_DC_Pin
                           LED3_Pin Gyro_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|LED_DC_Pin
                          |LED3_Pin|Gyro_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CIN2_Pin */
  GPIO_InitStruct.Pin = CIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CIN2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CIN1_Pin */
  GPIO_InitStruct.Pin = CIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CIN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
char direction;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(huart);
	//HAL_UART_Transmit(&huart3,(uint8_t *) aTxBuffer,5,0xFFFFF);



	HAL_UART_Receive_IT(&huart3, (uint8_t *) aRxBuffer,sizeof(aRxBuffer));
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  //uint8_t ch = 'A';
  for(;;)
  {
//	HAL_UART_Transmit(&huart3,(uint8_t *) &ch,1,0xFFFF);
//	if(ch < 'Z'){
//		ch++;
//	}else{
//		ch = 'A';
//	}
	HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);

	osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_motor */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor */
void motor(void *argument)
{
  /* USER CODE BEGIN motor */
	uint16_t pwnVal = 0;
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	char direction;
	int magnitude;
	uint8_t opop[6];




	/* Infinite loop */
  for(;;)
  {
	  //emcpy(temp,aRxBuffer[0],sizeof(aRxBuffer[0]));
	  direction = aRxBuffer[0];
	  //HAL_UART_Transmit(&huart3,(uint8_t *) aRxBuffer[0],sizeof(aRxBuffer[0]),0xFF);
	  //OLED_ShowString(10,50,direction);
	//clockwise
//	while(pwnVal < 4000)
//	{
//		HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
//		pwnVal++;
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwnVal); //Modify comparison value for the duty cycle
//		osDelay(10);
//	}
//	//anticlockwise
//	while(pwnVal > 0)
//	{
//		HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
//		pwnVal--;
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwnVal); //Modify comparison value for the duty cycle
//		osDelay(10);
//	}
//    osDelay(1000);

	  // if(strlen(aRxBuffer) != 0)
		//  strcpy(motorcmd, &aRxBuffer[0]);
//	  if(strcmp(aRxBuffer,"    ") == 0){
//		  osDelay(5000);
//		  stop();
//	  }

	  if (strncmp(aRxBuffer, "R",1) == 0){

		  //HAL_UART_Transmit(&huart3,opop,sizeof(opop),0xFF);
		  OLED_ShowString(10,50,"R\0");
		  magnitude = (int)(aRxBuffer[1] - '0')*100 + (int)(aRxBuffer[2] - '0')*10 + (int)(aRxBuffer[3] - '0');
		  sprintf(opop,"%d\0", magnitude);
		  OLED_ShowString(10,50,opop);
		  //reverse1(35.00,defaultPower);
		  //osDelay(500);
		  Right(magnitude*-1,defaultPower);
		  //osDelay(500);
		  osDelay(10);
		  reverse1(190.00,defaultPower);
		  osDelay(10);
		  magnitude = 0;
		  HAL_UART_Transmit(&huart3,"S11",3,0xFF);
	  }
//	  else if(strncmp(aRxBuffer, "T",1 )== 0) {
//		  OLED_ShowString(10,50,"R\0");
//		  magnitude = (int)(aRxBuffer[1] - '0')*100 + (int)(aRxBuffer[2] - '0')*10 + (int)(aRxBuffer[3] - '0');
//		  sprintf(opop,"%d\0", magnitude);
//		  OLED_ShowString(10,50,opop);
//		  Wheelangle(magnitude);
//		  osDelay(100);
//		  DCControl(2);
//		  while(strncmp(aRxBuffer,"T", 1) == 0);
//		  //osDelay(10);
//		  stop1();


//		  osDelay(10);
//		  magnitude = 0;
//		  HAL_UART_Transmit(&huart3,"S11",3,0xFF);
//	  } else if(strncmp(aRxBuffer, "Y",1 )== 0) {
//		  OLED_ShowString(10,50,"R\0");
//		  magnitude = (int)(aRxBuffer[1] - '0')*100 + (int)(aRxBuffer[2] - '0')*10 + (int)(aRxBuffer[3] - '0');
//		  sprintf(opop,"%d\0", magnitude);
//		  OLED_ShowString(10,50,opop);
//		  Wheelangle(magnitude);
//		  osDelay(100);
//		  DCControl(-1);
//		  while(strncmp(aRxBuffer,"Y", 1) == 0);
//		  //osDelay(10);
//		  stop1();

//		  osDelay(10);
//		  magnitude = 0;
//		  HAL_UART_Transmit(&huart3,"S11",3,0xFF);
	  //}
	  else if (strncmp(aRxBuffer, "E",1) == 0){

	 		  //HAL_UART_Transmit(&huart3,opop,sizeof(opop),0xFF);
	 		  OLED_ShowString(10,50,"R\0");
	 		  magnitude = (int)(aRxBuffer[1] - '0')*100 + (int)(aRxBuffer[2] - '0')*10 + (int)(aRxBuffer[3] - '0');
	 		  sprintf(opop,"%d\0", magnitude);
	 		  OLED_ShowString(10,50,opop);
	 		 osDelay(10);
	 		 forward1(190.00,defaultPower);
	 		 osDelay(500);
	 		 //reverse1(70.00,defaultPower);
	 		 back_Right(magnitude,defaultPower);
	 		 osDelay(500);
	 		 //reverse1(90.00,defaultPower);
	 		forward1(50.00,defaultPower);
	 		  osDelay(10);
	 		  magnitude = 0;
	 		  HAL_UART_Transmit(&huart3,"S11",3,0xFF);
	 	  }else if (strncmp(aRxBuffer, "L",1) == 0){
		  OLED_ShowString(10,50,"L\0");
		  magnitude = (int)(aRxBuffer[1] - '0')*100 + (int)(aRxBuffer[2] - '0')*10 + (int)(aRxBuffer[3] - '0');
		  sprintf(opop,"%d\0", magnitude);
		  OLED_ShowString(10,50,opop);
		  forward1(35.00,defaultPower);
		  //osDelay(1500);
		  osDelay(10);
		  Left(magnitude,defaultPower);
 		  //osDelay(1500);
		  osDelay(10);
 		  reverse1(120.00,defaultPower);
 		  osDelay(10);
		  magnitude = 0;
		  HAL_UART_Transmit(&huart3,"S11",3,0xFF);
	  }else if (strncmp(aRxBuffer, "K",1) == 0){
		  OLED_ShowString(10,50,"L\0");
		  magnitude = (int)(aRxBuffer[1] - '0')*100 + (int)(aRxBuffer[2] - '0')*10 + (int)(aRxBuffer[3] - '0');
		  sprintf(opop,"%d\0", magnitude);
		  OLED_ShowString(10,50,opop);
		  //osDelay(500);
		  //reverse1(70.00,defaultPower);
		  forward1(100.00,defaultPower);
		  //osDelay(1000);
		  osDelay(10);
		  back_Left(magnitude*-1,defaultPower);
		  //osDelay(500);
		  osDelay(10);
		  reverse1(40.00,defaultPower);
		  //osDelay(500);

		  magnitude = 0;
		  HAL_UART_Transmit(&huart3,"S11",3,0xFF);
//	  }else if (strncmp(aRxBuffer, "T",1) == 0){
//		  OLED_ShowString(10,50,"L\0");
//		  //magnitude = (int)(aRxBuffer[1] - '0')*100 + (int)(aRxBuffer[2] - '0')*10 + (int)(aRxBuffer[3] - '0');
//		  //sprintf(opop,"%d\0", magnitude);
//		  //OLED_ShowString(10,50,opop);
//		  Left(magnitude,defaultPower);
// 		  osDelay(500);
// 		  reverse1(35.00,defaultPower);
// 		  osDelay(10);
//		  magnitude = 0;
//	  }
	  }else if(strncmp(aRxBuffer,"F",1)== 0){ //Forward
		  magnitude = (double) 10.00*((double)(aRxBuffer[1] - '0')*100.00 + (double)(aRxBuffer[2] - '0')*10.00 + (double)(aRxBuffer[3] - '0')*1.00);
		  //reverse(300,defaultPower);
		  forward((double)magnitude,defaultPower);
		  osDelay(10);
		  HAL_UART_Transmit(&huart3,"S11",3,0xFF);
	  }else if(strncmp(aRxBuffer,"Z",1)== 0){ //Reverse
		  magnitude = (double) 10.00*((double)(aRxBuffer[1] - '0')*100.00 + (double)(aRxBuffer[2] - '0')*10.00 + (double)(aRxBuffer[3] - '0')*1.00);
		  //reverse(300,defaultPower);
		  reverse((double)magnitude,defaultPower);
		  osDelay(10);
		  HAL_UART_Transmit(&huart3,"S11",3,0xFF);
	  }else if(strcmp(aRxBuffer, "C111") == 0){ //Center Servo
		  center();
		  osDelay(10);
	  }
	  else if(strcmp(aRxBuffer, "U111") == 0){ //first obstacle Left
		  fullLeft();

		  //Left(90,defaultPower);
		  osDelay(10);
		  HAL_UART_Transmit(&huart3,"S11",3,0xFF);
	  }else if(strncmp(aRxBuffer, "T",1) == 0){ //second obstacle left
		  magnitude = (double) 10.00*((double)(aRxBuffer[1] - '0')*100.00 + (double)(aRxBuffer[2] - '0')*10.00 + (double)(aRxBuffer[3] - '0')*1.00);
		  halfLeft((double)magnitude);
		  osDelay(10);
		  HAL_UART_Transmit(&huart3,"S11",3,0xFF);
	  }else if(strcmp(aRxBuffer, "I111") == 0){ //first obstacle right
		  //Right(-90,defaultPower);
		  fullRight();
		  //	sprintf(aRxBuffer, "0000");
		  osDelay(10);
		  HAL_UART_Transmit(&huart3,"S11",3,0xFF);
	  }else if(strncmp(aRxBuffer, "Y",1) == 0){ //second obstacle right
		  magnitude = (double) 10.00*((double)(aRxBuffer[1] - '0')*100.00 + (double)(aRxBuffer[2] - '0')*10.00 + (double)(aRxBuffer[3] - '0')*1.00);
		  halfRight((double)magnitude);
		  osDelay(10);
		  HAL_UART_Transmit(&huart3,"S11",3,0xFF);
	  }else if(strcmp(aRxBuffer, "TTTT") == 0){
		  osDelay(100);
		  forward(2000);
		  while(wheelDist < 4000){
			  printf("Hello");
		  }
		  stop();
		  osDelay(5000);
		  reverse(defaultPower);
		  while(wheelDist > 0){
			  printf("Hello");
		  }
		  stop();
	  }else if(strcmp(aRxBuffer, "STOP") == 0){
		  stop1();
		  osDelay(100);



		  osDelay(100);
		  //HAL_UART_Transmit(&huart3,"JJJJ\0",2,0xFF);

	  }
	  else{
		  //Right(90);
		  //reverse(200.00);
		  //fullLeft();
		  //fullRight();
		  //osDelay(10000);
		  //reverse(200.00);
		  //halfRight(1000.00);

		  //halfLeft(1000.00);
		  //osDelay(10000);
		 // halfLeft();
		  //osDelay(1000);
		  //forward(200.00);
		  //break;
		  OLED_ShowString(10,40,aRxBuffer);

		  //OLED_ShowString(10,10,motorcmd);
	  }
//	  else{
//	  		  OLED_ShowString(10,40,aRxBuffer);
//	  	  }
	  magnitude = 0;
	  memset(aRxBuffer, 0, sizeof(aRxBuffer));
	  memset(aTxBuffer, 0, sizeof(aTxBuffer));
	  osDelay(5);
  }

  /* USER CODE END motor */
}

/* USER CODE BEGIN Header_encoder_task */
/**
* @brief Function implementing the EncoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encoder_task */
void encoder_task(void *argument)
{
  /* USER CODE BEGIN encoder_task */
  /* Infinite loop */
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);

	int cnt1,cnt2,diff1,diff2;
	int dirL,dirR;
	uint32_t tick = 0;
	total_distanceA = 0;
	total_distanceB = 0;
	cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
	tick = HAL_GetTick();
	uint8_t strWheelDist[20];
	uint8_t hello2[20];
	uint16_t dir;
  for(;;)
  {
//	cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
//	cnt2 = _HAL_TIM_GET_COUNTER(&htim3);
//	total_distA = cnt1;
//	total_distB = cnt2;
//
//	sprintf(strWheelDist,"Dir:%5d\0",cnt1);
//	//OLED_ShowString(10,30,strWheelDist); //Have to send this to algo?
//
//	if(HAL_GetTick()-tick > 1000L)
//	{
//		cnt2 = __HAL_TIM_GET_COUNTER(&htim2);
//		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){
//			if(cnt2<cnt1){
//				diff = cnt1 - cnt2;
//			}
//			else{
//				diff = (65535 - cnt2)+cnt1;
//			}
//		}
//		else{
//			if (cnt2 > cnt1){
//				diff = cnt2 - cnt1;
//				}
//			else{
//				diff = (65535 - cnt1) + cnt2;
//				}
//			}
//
//		if(diff == 65535) diff = 0;
//		sprintf(speed,"Speed:%5d\0",diff);
//		OLED_ShowString(10,20,speed);
//		dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
////		sprintf(hello2,"Dir:%5d\0",cnt2);
////		OLED_ShowString(10,10,hello2);
//		OLED_Refresh_Gram();

		//tick = HAL_GetTick();
		 if(HAL_GetTick() - tick >= 10){
				  cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
				  cnt2 =  __HAL_TIM_GET_COUNTER(&htim3);
					if(cnt1 > 32000){
						  diff1 = (65536 - cnt1);
						  dirL = 1;


						  }
					 else {
						  diff1 = cnt1;
						  dirL = -1;

					  }


					  if(cnt2 > 32000){
						  diff2 = (65536 - cnt2);
						  dirR = -1;
					  }
					  else {
						 diff2 = cnt2;
						 dirR = 1;
					  }


                      if(dirR == 1){
	  		  			  total_distanceB += diff2;
                         }
                      else {
                       	  total_distanceB -= diff2;
                         }

                         if(dirL == 1){
	  		  			  total_distanceA += diff1;
                         }
                         else {
                       	  total_distanceA -= diff1;
                         }

		  			__HAL_TIM_SET_COUNTER(&htim2, 0);
		  			__HAL_TIM_SET_COUNTER(&htim3, 0);
		  			tick = HAL_GetTick();
		  			sprintf(disA,"DisA:%5d",total_distanceA);
		  			sprintf(disB,"DisB:%5d",total_distanceB);
		  			//OLED_ShowString(10,20,disB);
		  			//OLED_ShowString(10,50,disA);
		  			OLED_Refresh_Gram();

		}
	}


    osDelay(1);
  /* USER CODE END encoder_task */
}

/* USER CODE BEGIN Header_show */
/**
* @brief Function implementing the show_OLED thread.
* @param argument: Not used
* @retval None
*/

/* Movement Functions BEGIN Header_show */
void usDelay(uint16_t us){
	__HAL_TIM_SET_COUNTER(&htim4,0);
	while(__HAL_TIM_GET_COUNTER(&htim4)<us)
	printf("");

}
void Wheelangle(int angle ){
	htim1.Instance -> CCR4 = angle;
}
//void DCControl(int op){
//if (op >1){
//	HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,2000);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,2000);
//	}
//else{
//		HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,2000);
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,2000);
//	}
//}

void forward(double target_distance,int pwnVal){
	HAL_UART_Transmit(&huart3,"F11",3,0xFF);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		__HAL_TIM_SET_COUNTER(&htim3, 0);
	  total_distanceB = 0.00;
	  total_distanceA = 0.00;
	//OLED_ShowString(10,10,"           ");
	  double encoder_count = 1559.00;
	  uint8_t printprint[20];
	  OLED_ShowString(10,10,"Start\0");
	  uint8_t totaldist[20];

	  double stopdistance = (double) (((double)(target_distance)/(215.00))*encoder_count);
	  stopdistance = stopdistance - 217.53488;
	  sprintf(printprint,"DisA:%5d",stopdistance);
	  //OLED_ShowString(10, 20, printprint);
	  htim1.Instance -> CCR4 = 149;
	  osDelay(1000);


	  HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET);
	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,2000);
	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,2250);
	  total_angle = 0;
	  int counter = 0;
	  while (1){

//		  if(total_angle>= 1){
//			  htim1.Instance -> CCR4 = 159;
//			  osDelay(20);
//
//		  }
//		  else if (total_angle <= -1){
//			  htim1.Instance -> CCR4 = 139;
//			  osDelay(100);
//
//		  }
//		  else{
//			  htim1.Instance -> CCR4 = 147;
//
//		  }

		  double pee = (double) total_distanceB;
		  //sprintf(totaldist,"DisB:%5d",pee);
		  //OLED_ShowString(10,50,totaldist);
		  if(pee <  stopdistance){
			  htim1.Instance -> CCR4 = 139;
			  osDelay(60);
			  htim1.Instance -> CCR4 = 159;
			  osDelay(50);
			  //OLED_ShowString(10, 50, "going");
		  }


		  else if(pee >= stopdistance){
			  //OLED_ShowString(10, 50, "hello");
			  stop1();
			  //stop1();
			//OLED_ShowString(10, 50, "     ");
				break;
			}
//		  else if(pee <= 0.5* stopdistance){
//			  HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
//			  HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
//			  HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET);
//			  HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET);
//			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,3000);
//			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,3250);
//
//		  }
	  }
	  //stop(6650,7000);
	  //
	  }

void forward1(double target_distance,int pwnVal){
	  total_distanceB = 0.00;
	  total_distanceA = 0.00;
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		__HAL_TIM_SET_COUNTER(&htim3, 0);
	HAL_UART_Transmit(&huart3,"F11",3,0xFF);
	//OLED_ShowString(10,10,"           ");
	  double encoder_count = 1559.00;
	  char printprint[10];
	  OLED_ShowString(10,10,"Start\0");
	  uint8_t totaldist[20];

	  double stopdistance = (double) (((double)(target_distance)/(215.00))*encoder_count);
	  stopdistance = stopdistance - 217.53488;
	  htim1.Instance -> CCR4 = 149;
	  osDelay(1000);


	  HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET);
	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwnVal);
	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,2250);
	  total_angle = 0;
	  int counter = 0;
	  while (1){

		  //		  if(total_angle>= 1){
		  //			  htim1.Instance -> CCR4 = 159;
		  //			  osDelay(20);
		  //
		  //		  }
		  //		  else if (total_angle <= -1){
		  //			  htim1.Instance -> CCR4 = 139;
		  //			  osDelay(100);
		  //
		  //		  }
		  //		  else{
		  //			  htim1.Instance -> CCR4 = 147;
		  //
		  //		  }

		  		  double pee = (double) total_distanceB;

		  		  if(pee <  stopdistance){
		  			  htim1.Instance -> CCR4 = 139;
		  			  osDelay(60);
		  			  htim1.Instance -> CCR4 = 159;
		  			  osDelay(50);
		  		  }


		  		  else if(pee >= stopdistance){
		  				stop1();

		  				break;
		  			}
		  	  }
	  stop1();
	 // HAL_UART_Transmit(&huart3,"S11",3,0xFF);
	  total_distanceB = 0.00;
	  total_distanceA = 0.00;
}


void stop(int pwn1, int pwn2){
	HAL_UART_Transmit(&huart3,"S00",3,0xFF);
	//OLED_ShowString(10,10,"           ");
	  OLED_ShowString(10,10,"Stop\0");
	  //int pwnVal = 0;
	  total_distanceA = 0;
	  total_distanceB = 0;
	  total_angle = 0;
	  htim1.Instance -> CCR4 = 151;
	  HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);

	  while ((pwn1 > 0) && (pwn2 > 0)){
		  pwn1 -=500;
		  if (pwn1 < 0){
			  pwn1 = 0;
		  }
		  pwn2-=500;
		  if(pwn2 < 0 ){
			  pwn2 = 0;
		  }
		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwn1);
		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwn2);
		  osDelay(1);
		  OLED_ShowString(10,50,"slow down");
	  }
	  }
void stop1(){
	HAL_UART_Transmit(&huart3,"S00",3,0xFF);
	//OLED_ShowString(10,10,"           ");
	  OLED_ShowString(10,10,"Stop\0");
	  int pwnVal = 0;
	  total_distanceA = 0;
	  total_distanceB = 0;
	  total_angle = 0;
	  htim1.Instance -> CCR4 = 151;
	  HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwnVal);
	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwnVal);
	  osDelay(50);
	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
}

void reverse(double target_distance,int pwnVal){
	  total_distanceB = 0.00;
	  total_distanceA = 0.00;
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		__HAL_TIM_SET_COUNTER(&htim3, 0);
	HAL_UART_Transmit(&huart3,"REV",3,0xFF);
	//OLED_ShowString(10,10,"           ");
	  OLED_ShowString(10,10,"Reverse\0");
	  total_distanceA = 0;
	  total_distanceB = 0;
	  total_angle = 0;
	  htim1.Instance -> CCR4 = 151;
	  osDelay(500);
	  HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,2000);
	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,2250);
	  double encoder_count = -1559.00;
	  double stopdistance = (double) (((double)(target_distance-1.00)/(215.00))*encoder_count);
	  stopdistance = stopdistance + 217.53488;
	  while(1){
		  osDelay(10);
		  if(total_angle>= 1){
			  htim1.Instance -> CCR4 = 139;
			  osDelay(20);

		  }
		  else if (total_angle <= -1){
			  htim1.Instance -> CCR4 = 159;
			  osDelay(20);

		  }
		  else{
			  htim1.Instance -> CCR4 = 151;
		  }
		  double pee = (double) total_distanceB;
		  if(pee <=  stopdistance){
				stop1();
				break;
			}
	  }
	  stop1();
		//total_distanceA = 0;
		//total_distanceB = 0;


}
void reverse1(double target_distance,int pwnVal){
	  total_distanceB = 0.00;
	  total_distanceA = 0.00;
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		__HAL_TIM_SET_COUNTER(&htim3, 0);
	HAL_UART_Transmit(&huart3,"REV",3,0xFF);
	//OLED_ShowString(10,10,"           ");
	  OLED_ShowString(10,10,"Reverse\0");
	  total_distanceA = 0;
	  total_distanceB = 0;
	  total_angle = 0;
	  htim1.Instance -> CCR4 = 151;
	  osDelay(500);
	  HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,7000);
	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,6975);
	  double encoder_count = -1559.00;
	  double stopdistance = (double) (((double)(target_distance-1.00)/(215.00))*encoder_count);
	  stopdistance = stopdistance + 217.53488;
	  while(1){
		  osDelay(10);
		  if(total_angle>= 1){
			  htim1.Instance -> CCR4 = 139;
			  osDelay(20);

		  }
		  else if (total_angle <= -2){
			  htim1.Instance -> CCR4 = 159;
			  osDelay(20);

		  }
		  else{
			  htim1.Instance -> CCR4 = 151;
		  }
		  double pee = (double) total_distanceB;
		  if(pee <=  stopdistance){
			    stop(7000,6990);
				break;
			}
	  }
	  stop1();
		//total_distanceA = 0;
		//total_distanceB = 0;

}


void center(){
	HAL_UART_Transmit(&huart3,"C11",3,0xFF);
	//OLED_ShowString(10,10,"           ");
	  OLED_ShowString(10,10,"Center\0");
	  htim1.Instance->CCR4 = 151;
	  HAL_UART_Transmit(&huart3,"S11",3,0xFF);
}

void Left(int target_angle,int pwnVal){
	HAL_UART_Transmit(&huart3,"L11",3,0xFF);
	//OLED_ShowString(10,10,"           ");
	 HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET);

	  htim1.Instance->CCR4 = 95;
	  osDelay(1500);
		 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1333);
		 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwnVal);

	 total_angle = 0;

	  OLED_ShowString(10,10,"Full Left\0");

	  while(strncmp(aRxBuffer,"L", 1) == 0){
		  osDelay(10);
		  if (total_angle >= target_angle-3){
			  break;
		  }
	  }
	  total_angle = 0;
	  htim1.Instance->CCR4 = 151;
	  //osDelay(10);
	  stop1();
}

void back_Left(int target_angle,int pwnVal){
	HAL_UART_Transmit(&huart3,"K11",3,0xFF);
	//OLED_ShowString(10,10,"           ");
	 HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);
	  htim1.Instance->CCR4 = 95;
	  osDelay(500);
		 HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
		 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1333);

	 total_angle = 0;
	 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwnVal);
	  OLED_ShowString(10,10,"Full Left\0");

	  while(strncmp(aRxBuffer,"K", 1) == 0){
		  osDelay(10);
		  if (total_angle <= target_angle+4){
			  break;
		  }
	  }
	  total_angle = 0;
	  htim1.Instance->CCR4 = 151;
	  //osDelay(10);
	  stop1();
}

void fullLeft(){
	HAL_UART_Transmit(&huart3,"L11",3,0xFF); //rev
		HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);

		htim1.Instance->CCR4 = 151;
		osDelay(1000);

		//1
		htim1.Instance->CCR4 = 235;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,2300);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1000);
		osDelay(1200);

		//2
		htim1.Instance->CCR4 = 95;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1333);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,2300);
		osDelay(1400);

		//3
		htim1.Instance->CCR4 = 151; //moveforward
		HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET);

		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1100);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1350);

		osDelay(1000);

		HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET); //rev

		//4
		htim1.Instance->CCR4 = 95;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1333);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,2100);
		osDelay(1000);

		//5
		htim1.Instance->CCR4 = 235;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,2100);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1333);
		osDelay(1290);

		//6
		htim1.Instance->CCR4 = 151;
		osDelay(400);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);


}

void halfLeft(double target_distance){
	HAL_UART_Transmit(&huart3,"L11",3,0xFF);
		HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);

		htim1.Instance->CCR4 = 151;
		osDelay(850);

		//1
		htim1.Instance->CCR4 = 235;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,3300);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,225);
		osDelay(1250);
		//reset
		htim1.Instance->CCR4 = 151;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
		osDelay(400);
		// first 90
		htim1.Instance->CCR4 = 95;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1333);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1900);
		osDelay(2000);
		// idle
		htim1.Instance->CCR4 = 151;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
		osDelay(400);

		// second 90
		htim1.Instance->CCR4 = 95;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1333);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1900);
		osDelay(1620);

		//4
		htim1.Instance->CCR4 = 151;
        __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
        __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
        osDelay(400);
        //
        total_angle = 0;
		reverse(600.00,0);
		osDelay(10);


		//6 turn to face end
		htim1.Instance->CCR4 = 95;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1333);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1900);
		osDelay(1760);

		htim1.Instance->CCR4 = 151;
        __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
        __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
        osDelay(400);
        //
        total_angle = 0;
		//reverse(150.00,0);
		osDelay(10);
		//7 idle
		htim1.Instance->CCR4 = 151;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
		osDelay(400);
		//8 chiong back
		reverse1(target_distance,0);
		osDelay(10);
		//9 end sequence
		//aligning to carpark (add accordingly)
		htim1.Instance->CCR4 = 95;
		osDelay(10);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1333);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1900);
		osDelay(1900);
		//IDLE
	 	htim1.Instance->CCR4 = 151;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
		osDelay(400);
		//turn to face carpark
		forward(100.0,0);
		htim1.Instance->CCR4 = 235;
		osDelay(10);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1900);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1333);
		osDelay(2300);
		//4
		htim1.Instance->CCR4 = 151;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
		osDelay(400);




}

void Right(int target_angle,int pwnVal){
	HAL_UART_Transmit(&huart3,"R11",3,0xFF);
	HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET);
		  htim1.Instance->CCR4 = 235;
		  osDelay(500);
			 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwnVal);
			 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1333);
		  OLED_ShowString(10,10,"Right\0");
		  total_angle = 0;

		  while(strncmp(aRxBuffer,"R",1)==0){
			  osDelay(10);
			  if (total_angle <= target_angle+2){
				  	 break;
			  }
		  }
		  total_angle = 0;
		  htim1.Instance->CCR4 = 151;
		  //osDelay(10);
		  stop1();
}
void back_Right(int target_angle,int pwnVal){
	HAL_UART_Transmit(&huart3,"E11",3,0xFF);
	HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);

		  htim1.Instance->CCR4 = 235;
		  osDelay(500);
			 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwnVal-100);
			 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1333);
		  OLED_ShowString(10,10,"Back Right\0");
		  total_angle = 0;

		  while(strncmp(aRxBuffer,"E",1)==0){
			  osDelay(10);
			  if (total_angle >= target_angle-3){
				  break;

			  }
		  }
		  total_angle = 0;
		  htim1.Instance->CCR4 = 151;
		  //osDelay(10);
		  stop1();
}

void fullRight(){

	HAL_UART_Transmit(&huart3,"R11",3,0xFF); //rev
	HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);

	htim1.Instance->CCR4 = 151;
	osDelay(1000);

	//1
	htim1.Instance->CCR4 = 95;
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1333);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1900);
	osDelay(1200);

	//2
	htim1.Instance->CCR4 = 220;
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1900);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1333);
	osDelay(1820);

	//3
	htim1.Instance->CCR4 = 151; //moveforward
	HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1100);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1350);

	osDelay(1500);

	HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET); //rev

	//4
	htim1.Instance->CCR4 = 220;
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1900);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1333);
	osDelay(1500);

	//5
	htim1.Instance->CCR4 = 95;
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1333);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1900);
	osDelay(1410);

	//6
	htim1.Instance->CCR4 = 151;
	osDelay(400);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);


}

void halfRight(double target_distance){
	HAL_UART_Transmit(&huart3,"R22",3,0xFF);
		HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);

		htim1.Instance->CCR4 = 151;
		osDelay(1000);

		//1
		htim1.Instance->CCR4 = 95;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,225);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,2600);
		osDelay(1380);
		//reset
		htim1.Instance->CCR4 = 151;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
		osDelay(400);
		reverse(100.00,0);
		//first 90
		htim1.Instance->CCR4 = 235;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1900);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1333);
		osDelay(2100);
		//Idle
		htim1.Instance->CCR4 = 151;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
		osDelay(400);
		//second 90
		htim1.Instance->CCR4 = 235;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1900);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1333);
		osDelay(2100);
		//5
		htim1.Instance->CCR4 = 151;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
		osDelay(400);
		total_angle = 0;

		reverse(400.00,0);
		osDelay(10);
		//6
		htim1.Instance->CCR4 = 235;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1900);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1333);
		osDelay(2000);
		htim1.Instance->CCR4 = 151;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
		osDelay(400);
		total_angle = 0;

		reverse(200.00,0);
		osDelay(10);
		//7
		htim1.Instance->CCR4 = 151;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
		osDelay(400);

		//8 chiong back
		reverse1(target_distance,0);
		osDelay(10);
		//9 end sequence
		htim1.Instance->CCR4 = 235;
		osDelay(10);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1900);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1333);
		osDelay(2000);
		//pause
 	    htim1.Instance->CCR4 = 151;
		osDelay(400);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
        //
		forward(300.0,0);
		htim1.Instance->CCR4 = 95;
		osDelay(10);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1333);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1900);
		osDelay(2000);


 	    htim1.Instance->CCR4 = 151;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
		osDelay(400);







}



/* Movement Functions END show */

/* Sensor Functions */
// returns mV
int binToVolt(uint16_t bin){
	float voltage;
	voltage = (bin * 8.00 );
	return voltage;
}

void setAverageVolt(){ //starts whole correction thing
	sFlag = 1;
}

void resetAverageVolt(){
  averageVolt1 = 0;
  averageVolt2 = 0;
}

int needToCorrect(double avgvolt1, double avgvolt2){ //newnew
	while(vFlag != 1){
		printf("");
	}
  int diffVolt = avgvolt1 - avgvolt2;
  //checks to correct left or right
  if (diffVolt > 5000){ //adjustable value
	  vFlag = 0;
    return 2;
  }else if (diffVolt < -5000){ //adjustable value
	  vFlag = 0;
    return 1;
  }else{
	  vFlag = 0;
    return 0;
  }
}

void moveWheel(int lr){ //newnew
  //needs to be adjusted
  if (lr == 1){
    HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,1000);
    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
  } else if (lr == 2){
    HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
	  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1000);
  }
}

void correctiveMovement(){ //newnew
  setAverageVolt();
  osDelay(5000);
    int correctFlag = 0;
    correctFlag = needToCorrect(averageVolt1, averageVolt2);

    if (correctFlag == 2){
    	moveWheel(2);
      while(correctFlag != 0){
        setAverageVolt();
        correctFlag = needToCorrect(averageVolt1, averageVolt2);
      }
    }else if (correctFlag == 1){
    	moveWheel(1);
      while(correctFlag != 0){
          setAverageVolt();
          correctFlag = needToCorrect(averageVolt1, averageVolt2);
        }
    }
    stop1();
}



/* Sensor Functions End*/




/* USER CODE END Header_show */
void show(void *argument)
{
  /* USER CODE BEGIN show */
	uint16_t Adc_value;
	uint16_t Adc_value1;
	char msg[20];
	int count = 0;
	int t1 = 0;
	int t2 = 0;



	HAL_TIM_Base_Start(&htim4);
	int check = __HAL_TIM_GET_COUNTER(&htim4);

  /* Infinite loop */
  for(;;)
  {
//	  //IR 1
//	 	  HAL_ADC_Start(&hadc1);
//	 	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//	 	  Adc_value = HAL_ADC_GetValue(&hadc1);
//
//	     //IR 2
//	      HAL_ADC_Start(&hadc2);
//	 	  HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
//	 	  Adc_value1 = HAL_ADC_GetValue(&hadc2);
//
//	 	  t1 = binToVolt(Adc_value1);
//	 	  t2 = binToVolt(Adc_value);
//	 	  //sFlag = 1;
//
//	 	 //sprintf(msg, "Volt2: %d, %d\r\n \0", (int) t1, (int) t2);
//	 	 //HAL_UART_Transmit(&huart3,(uint8_t *) msg,strlen(msg),0xFFFFF);
//	 	 osDelay(1000);
//	 	  if (sFlag == 1 && count != 100){
//	       resetAverageVolt();
//	       averageVolt1 += binToVolt(Adc_value);
//	       averageVolt2 += binToVolt(Adc_value1);
//	 		  count += 1;
//	 		 OLED_ShowString(10,50,"in");
//	 	  }else if (count >= 100 && sFlag == 1){
//	 		 OLED_ShowString(10,50,"in1");
//	 		 averageVolt1 = averageVolt1/100.00;
//	 		 averageVolt2 = averageVolt2/100.00;
//
//	 		sprintf(msg, "V: %d, %d \0", (int) averageVolt2, (int) averageVolt1);
//	 		vFlag = 1;
//	 		HAL_UART_Transmit(&huart3,(uint8_t *) msg,strlen(msg),0xFFFFF);
//	 	  }else if (sFlag == 1 && vFlag == 0){ //Done collecting
//	 		 OLED_ShowString(10,50,"in1");
//	 		  vFlag = 0;
//	 		  count = 0;
//	 		  sFlag = 0;
//	 		  resetAverageVolt();
//	 	  }
//	 	  //sprintf(msg, "%hu\r\n \0", Adc_value);
//	 	  //HAL_UART_Transmit(&huart3,(uint8_t *) msg,strlen(msg),0xFFFFF);
//	 	  //HAL_Delay(1);
  }
  /* USER CODE END show */
}

/* USER CODE BEGIN Header_GyroTask */
/**
* @brief Function implementing the Gyrotask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GyroTask */
void GyroTask(void *argument)
{
  /* USER CODE BEGIN GyroTask */
	uint8_t val[2] = {0,0};

	 char hellox[20];
	 char hello1 [20];
	 int16_t angular_speed = 0;



	 uint32_t tick = 0;
	 gyroInit();
	 int dir;
	 int16_t offset = 0;
	 tick = HAL_GetTick();
	 osDelay(10);
	 uint32_t tick2 = HAL_GetTick();
	 uint32_t totaltick = 0.00;
	 int tickFlag = 0;
	 int curangle = 0;
	 char toprint[20];


  /* Infinite loop */
  for(;;)
  {
      osDelay(10);

      if(HAL_GetTick() - tick >= 100){
   readByte(0x37, val);
   osDelay(1);
   angular_speed = ((int16_t)((int8_t)val[0]))*256 +  (uint16_t)val[1];
   angular_speed = (val[0] << 8) | val[1];






   total_angle +=(double)(angular_speed + 5)*((HAL_GetTick() - tick)/16400.0)*1.17;
   //total_angle +=(double)(angular_speed + drift_correction)*((HAL_GetTick() - tick)/16400.0)*turning_accuracy(change here);

   global_angle += (double)(angular_speed + 5)*((HAL_GetTick() - tick)/16400.0)*1.17;
   //global_angle +=(double)(angular_speed + drift_correction)*((HAL_GetTick() - tick)/16400.0)*turning_accuracy(change here);

   //calibrates drift duration fark it drift is not consistant
//   if (tickFlag == 1){
//	   if (HAL_GetTick() - tick2 >= totaltick){
//		   tick2 = HAL_GetTick();
//		   total_angle -= 1;
//		   global_angle -= 1;
//	   }
//   }else{
//	   OLED_ShowString(10,10, "Calibrating \0");
//	   if (curangle < 5){
//		   if ((int) global_angle != curangle ){
//			   curangle += 1;
//			   totaltick += HAL_GetTick();
//			   sprintf(toprint, "%d \0",(int) totaltick);
//			   OLED_ShowString(10,50, toprint);
//		   }
//	   }else{
//		   totaltick = totaltick/ (uint32_t)5.00;
//		   tickFlag = 1;
//		   OLED_ShowString(10,10, "Done Cal     \0");
//		   tick2 = HAL_GetTick();
//	   }
//   }


   //removes drift
//   if(tickFlag == 1){
//	   if (HAL_GetTick() - tick2 >= totaltick){
//		   tick2 = HAL_GetTick();
//		   total_angle -= 1;
//		   global_angle -= 1;
//
//	   }
//   }

   //prevSpeed = angular_speed;
   if(total_angle >= 720){
    total_angle = 0;
   }
   if(total_angle <= -720){
    total_angle = 0;
   }
   sprintf(hellox, "angle %5d \0", (int)(total_angle));
   OLED_ShowString(10,30, hellox);
   sprintf(hello1, "GAngle %5d \0", (int)(global_angle));
   OLED_ShowString(10,20, hello1);
   //OLED_Refresh_Gram();
   tick = HAL_GetTick();
      }

  }
  /* USER CODE END GyroTask */
}

/* USER CODE BEGIN Header_Ultrasound */
/**
* @brief Function implementing the UltrasoundTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Ultrasound */
void Ultrasound(void *argument)
{
  /* USER CODE BEGIN Ultrasound */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Ultrasound */
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
