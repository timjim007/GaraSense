/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../Application/Inc/ao.h"
#include "../../Application/Inc/ao_broker.h"
#include "../../Application/Inc/ao_display.h"
#include "../../Application/Inc/ao_distance.h"
#include "../../Application/Inc/ao_env.h"
#include "../../Application/Inc/ao_logger.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_MS 20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

/* Definitions for DistanceSensor */
osThreadId_t DistanceSensorHandle;
uint32_t DistanceSensorBuffer[ 1024 ];
osStaticThreadDef_t DistanceSensorControlBlock;
const osThreadAttr_t DistanceSensor_attributes = {
  .name = "DistanceSensor",
  .cb_mem = &DistanceSensorControlBlock,
  .cb_size = sizeof(DistanceSensorControlBlock),
  .stack_mem = &DistanceSensorBuffer[0],
  .stack_size = sizeof(DistanceSensorBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for EnvMonitor */
osThreadId_t EnvMonitorHandle;
uint32_t EnvMonitorBuffer[ 256 ];
osStaticThreadDef_t EnvMonitorControlBlock;
const osThreadAttr_t EnvMonitor_attributes = {
  .name = "EnvMonitor",
  .cb_mem = &EnvMonitorControlBlock,
  .cb_size = sizeof(EnvMonitorControlBlock),
  .stack_mem = &EnvMonitorBuffer[0],
  .stack_size = sizeof(EnvMonitorBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DataLogger */
osThreadId_t DataLoggerHandle;
uint32_t DataLoggerBuffer[ 1024 ];
osStaticThreadDef_t DataLoggerControlBlock;
const osThreadAttr_t DataLogger_attributes = {
  .name = "DataLogger",
  .cb_mem = &DataLoggerControlBlock,
  .cb_size = sizeof(DataLoggerControlBlock),
  .stack_mem = &DataLoggerBuffer[0],
  .stack_size = sizeof(DataLoggerBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Display */
osThreadId_t DisplayHandle;
uint32_t DisplayBuffer[ 512 ];
osStaticThreadDef_t DisplayControlBlock;
const osThreadAttr_t Display_attributes = {
  .name = "Display",
  .cb_mem = &DisplayControlBlock,
  .cb_size = sizeof(DisplayControlBlock),
  .stack_mem = &DisplayBuffer[0],
  .stack_size = sizeof(DisplayBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Broker */
osThreadId_t BrokerHandle;
uint32_t BrokerBuffer[ 256 ];
osStaticThreadDef_t BrokerControlBlock;
const osThreadAttr_t Broker_attributes = {
  .name = "Broker",
  .cb_mem = &BrokerControlBlock,
  .cb_size = sizeof(BrokerControlBlock),
  .stack_mem = &BrokerBuffer[0],
  .stack_size = sizeof(BrokerBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Sysmonitor */
osThreadId_t SysmonitorHandle;
const osThreadAttr_t Sysmonitor_attributes = {
  .name = "Sysmonitor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DistanceQueue */
osMessageQueueId_t DistanceQueueHandle;
uint8_t DistanceQueueBuffer[ 16 * sizeof( Event ) ];
osStaticMessageQDef_t DistanceQueueControlBlock;
const osMessageQueueAttr_t DistanceQueue_attributes = {
  .name = "DistanceQueue",
  .cb_mem = &DistanceQueueControlBlock,
  .cb_size = sizeof(DistanceQueueControlBlock),
  .mq_mem = &DistanceQueueBuffer,
  .mq_size = sizeof(DistanceQueueBuffer)
};
/* Definitions for EnvMonitorQueue */
osMessageQueueId_t EnvMonitorQueueHandle;
uint8_t EnvMonitorQueueBuffer[ 16 * sizeof( Event ) ];
osStaticMessageQDef_t EnvMonitorQueueControlBlock;
const osMessageQueueAttr_t EnvMonitorQueue_attributes = {
  .name = "EnvMonitorQueue",
  .cb_mem = &EnvMonitorQueueControlBlock,
  .cb_size = sizeof(EnvMonitorQueueControlBlock),
  .mq_mem = &EnvMonitorQueueBuffer,
  .mq_size = sizeof(EnvMonitorQueueBuffer)
};
/* Definitions for DataLoggerQueue */
osMessageQueueId_t DataLoggerQueueHandle;
uint8_t DataLoggerQueueBuffer[ 20 * sizeof( Event ) ];
osStaticMessageQDef_t DataLoggerQueueControlBlock;
const osMessageQueueAttr_t DataLoggerQueue_attributes = {
  .name = "DataLoggerQueue",
  .cb_mem = &DataLoggerQueueControlBlock,
  .cb_size = sizeof(DataLoggerQueueControlBlock),
  .mq_mem = &DataLoggerQueueBuffer,
  .mq_size = sizeof(DataLoggerQueueBuffer)
};
/* Definitions for DisplayQueue */
osMessageQueueId_t DisplayQueueHandle;
uint8_t DisplayQueueBuffer[ 30 * sizeof( Event ) ];
osStaticMessageQDef_t DisplayQueueControlBlock;
const osMessageQueueAttr_t DisplayQueue_attributes = {
  .name = "DisplayQueue",
  .cb_mem = &DisplayQueueControlBlock,
  .cb_size = sizeof(DisplayQueueControlBlock),
  .mq_mem = &DisplayQueueBuffer,
  .mq_size = sizeof(DisplayQueueBuffer)
};
/* Definitions for BrokerQueue */
osMessageQueueId_t BrokerQueueHandle;
uint8_t BrokerQueueBuffer[ 30 * sizeof( Event ) ];
osStaticMessageQDef_t BrokerQueueControlBlock;
const osMessageQueueAttr_t BrokerQueue_attributes = {
  .name = "BrokerQueue",
  .cb_mem = &BrokerQueueControlBlock,
  .cb_size = sizeof(BrokerQueueControlBlock),
  .mq_mem = &BrokerQueueBuffer,
  .mq_size = sizeof(BrokerQueueBuffer)
};
/* Definitions for logTimer */
osTimerId_t logTimerHandle;
const osTimerAttr_t logTimer_attributes = {
  .name = "logTimer"
};
/* Definitions for deleteTimer */
osTimerId_t deleteTimerHandle;
const osTimerAttr_t deleteTimer_attributes = {
  .name = "deleteTimer"
};
/* Definitions for readEnvTimer */
osTimerId_t readEnvTimerHandle;
const osTimerAttr_t readEnvTimer_attributes = {
  .name = "readEnvTimer"
};
/* Definitions for updateDisplayTimer */
osTimerId_t updateDisplayTimerHandle;
const osTimerAttr_t updateDisplayTimer_attributes = {
  .name = "updateDisplayTimer"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
void StartDistanceSensor(void *argument);
void StartEnvMonitor(void *argument);
void StartDataLogger(void *argument);
void StartDisplay(void *argument);
void StartBroker(void *argument);
void StartMonitor(void *argument);
void LogTimerCallback(void *argument);
void DeleteLogCallback(void *argument);
void readEnvTimerCallback(void *argument);
void updateDisplayTimerCallback(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char TxBuffer[TX_BUFFER_SIZE];
volatile uint8_t tx_busy = 0;

static uint32_t last_press_ms = 0;



Broker broker;
Display display;
DistanceSensor distance;
EnvMonitor envmonitor;
DataLogger logger;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
    	tx_busy = 0;
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
	HAL_SPI_DMAStop(hspi);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	uint32_t now = osKernelGetTickCount();


	if((now - last_press_ms) >= DEBOUNCE_MS){
		last_press_ms = now;
		Event buttonEvt;
		switch(GPIO_Pin){
			case Right_Button_Pin:
				if (HAL_GPIO_ReadPin(GPIOD, GPIO_Pin) == GPIO_PIN_RESET) {
					buttonEvt.eventsig = RIGHT_BUTTON_EVT;
					buttonEvt.dest = DISPLAY;
				}

				break;
			case Right_ButtonD3_Pin:
				if (HAL_GPIO_ReadPin(GPIOD, GPIO_Pin) == GPIO_PIN_RESET) {
					buttonEvt.eventsig = LEFT_BUTTON_EVT;
					buttonEvt.dest     = DISPLAY;
				//left
				}
				break;
			case Center_Button_Pin:
				if (HAL_GPIO_ReadPin(GPIOD, GPIO_Pin) == GPIO_PIN_RESET) {
					buttonEvt.eventsig = CONFIRM_BUTTON_EVT;
					buttonEvt.dest     = DISPLAY;
				}
				break;
			default:
				return;
		}
		Active_post(&broker.super, &buttonEvt);
	}

}



void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM2){
      HCSR04_TMR_IC_ISR(htim);
      Event distEvt = {.eventsig = UPDATE_LED, .dest = DIST_SENSOR};
      Active_post(&broker.super, &distEvt);
	 }
  }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	 if (htim->Instance == TIM2) {
	        HCSR04_TMR_OVF_ISR(htim);
	} else if (htim->Instance == TIM3) {
		HCSR04_Trigger(HCSR04_SENSOR1);
	}

}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	 if (hadc->Instance == ADC1)
	    {
		 MQ135_AverageRaw();

	        // Stop DMA after one block
	        HAL_ADC_Stop_DMA(hadc);
	    }
}





///* LogTimerCallback function */
//void LogTimerCallback(void *argument)
//{
//	Event envEvt = {.eventsig = SENSOR_TIMER, .dest = ENV_MONITOR};
//	Active_post(&broker.super, &envEvt);
//	Event tempEvt = {.eventsig = SENSOR_TIMER, .dest = DIST_SENSOR};
//	Active_post(&broker.super, &tempEvt);
//}
//
///* DeleteLogCallback function */
//void DeleteLogCallback(void *argument)
//{
//	Event e = {.eventsig = DELETE_LOG, .dest = DATA_LOGGER};
//	Active_post(&broker.super, &e);
//}

/* readEnvTimerCallback function */
void readEnvTimerCallback(void *argument)
{

	Event tempEvt = {.eventsig = TEMP_REQ, .dest = ENV_MONITOR};
	Active_post(&broker.super, &tempEvt);
	Event ppmEvt = {.eventsig = PPM_REQ, .dest = ENV_MONITOR};
	Active_post(&broker.super, &ppmEvt);

}

/* updateDisplayTimerCallback function */
void updateDisplayTimerCallback(void *argument)
{
	Event distEvt = {.eventsig = DIST_REQ, .dest = DIST_SENSOR};
	Active_post(&broker.super, &distEvt);
}

static void App_TimerInit(osTimerId_t timerHandle,uint32_t ticks){
	osTimerStart(timerHandle,ticks);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim3);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of logTimer */
//  logTimerHandle = osTimerNew(LogTimerCallback, osTimerPeriodic, NULL, &logTimer_attributes);

  /* creation of deleteTimer */
//  deleteTimerHandle = osTimerNew(DeleteLogCallback, osTimerPeriodic, NULL, &deleteTimer_attributes);

  /* creation of readEnvTimer */
  readEnvTimerHandle = osTimerNew(readEnvTimerCallback, osTimerPeriodic, NULL, &readEnvTimer_attributes);

  /* creation of updateDisplayTimer */
  updateDisplayTimerHandle = osTimerNew(updateDisplayTimerCallback, osTimerPeriodic, NULL, &updateDisplayTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of DistanceQueue */
  DistanceQueueHandle = osMessageQueueNew (16, sizeof(Event), &DistanceQueue_attributes);

  /* creation of EnvMonitorQueue */
  EnvMonitorQueueHandle = osMessageQueueNew (16, sizeof(Event), &EnvMonitorQueue_attributes);

  /* creation of DataLoggerQueue */
//  DataLoggerQueueHandle = osMessageQueueNew (20, sizeof(Event), &DataLoggerQueue_attributes);

  /* creation of DisplayQueue */
  DisplayQueueHandle = osMessageQueueNew (30, sizeof(Event), &DisplayQueue_attributes);

  /* creation of BrokerQueue */
  BrokerQueueHandle = osMessageQueueNew (30, sizeof(Event), &BrokerQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */

  /* creation of Sysmonitor */
  SysmonitorHandle = osThreadNew(StartMonitor, NULL, &Sysmonitor_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  	Broker_Init(&broker);
    Active_start(&broker.super, &Broker_attributes, BrokerQueueHandle);

    Display_Init(&display);
	Active_start(&display.super, &Display_attributes, DisplayQueueHandle);

	DistanceSensor_Init(&distance);
	Active_start(&distance.super, &DistanceSensor_attributes, DistanceQueueHandle);

	EnvMonitor_Init(&envmonitor);
	Active_start(&envmonitor.super, &EnvMonitor_attributes, EnvMonitorQueueHandle);

//	DataLogger_Init(&logger);
//	Active_start(&logger.super, &DataLogger_attributes, DataLoggerQueueHandle);
//
//	App_TimerInit(logTimerHandle,600000);
//	App_TimerInit(deleteTimerHandle,1800000);
	App_TimerInit(readEnvTimerHandle,100);
	App_TimerInit(updateDisplayTimerHandle,50);


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 7;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  sConfig.Channel = ADC_CHANNEL_1;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 124;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 49;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49999;
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_pin_GPIO_Port, CS_pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Ultrasonic_Trigger_GPIO_Port, Ultrasonic_Trigger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin CS_pin_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|CS_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT11_input_Pin */
  GPIO_InitStruct.Pin = DHT11_input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT11_input_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Ultrasonic_Trigger_Pin */
  GPIO_InitStruct.Pin = Ultrasonic_Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Ultrasonic_Trigger_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Right_Button_Pin Center_Button_Pin Right_ButtonD3_Pin */
  GPIO_InitStruct.Pin = Right_Button_Pin|Center_Button_Pin|Right_ButtonD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */


/* USER CODE BEGIN Header_StartMonitor */
/**
* @brief Function implementing the Sysmonitor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMonitor */
void StartMonitor(void *argument)
{
  /* USER CODE BEGIN StartMonitor */
  /* Infinite loop */
  for(;;)
  {

	  AO_ReportStack(&broker.super,    "Broker");
	  AO_ReportQueue(&broker.super, "Broker");

	  AO_ReportStack(&distance.super,  "Distance");
	  AO_ReportQueue(&distance.super, "Distance");

	  AO_ReportStack(&envmonitor.super,"EnvMonitor");
	  AO_ReportQueue(&envmonitor.super, "EnvMonitor");

	  AO_ReportStack(&logger.super,    "Logger");
	  AO_ReportQueue(&logger.super, "Logger");

	  AO_ReportStack(&display.super,  "Display");
	  AO_ReportQueue(&display.super, "Display");




  }
  /* USER CODE END StartMonitor */
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
