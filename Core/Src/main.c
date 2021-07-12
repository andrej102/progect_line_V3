/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  ******************************************************************************
  * Line V3.1 - Version for H723 and Castom line
  *
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NUM_PICES_PERIOD 8 // must equal power 2,  = 8, 16, 32, 64 ...
#define MIN_PICE_PERIOD 30


#define LINE_DUMMY 20
#define LINE_SIZE 1536
#define LINE_SIZE_WITH_DUMMY 1556

#define MUX_SIZE 16

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

ETH_TxPacketConfig TxConfig;
COMP_HandleTypeDef hcomp1;
COMP_HandleTypeDef hcomp2;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim17;
DMA_HandleTypeDef hdma_tim17_ch1;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

osEventFlagsId_t event_group_1_id = NULL;

#define FLAG_REQUEST_LCD_UPDATE 0x00000001
#define FLAG_REQUEST_SCANER		0x00000002
#define SECOND_BUFFER_LINE_1	0x00000004
#define ERROR_COUNT_OBJECTS		0x00000008

osEventFlagsId_t event_group_2_id = NULL;

osThreadId_t lcdTaskHandle;
const osThreadAttr_t lcdTask_attributes = {
  .name = "lcdTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t keyboardTaskHandle;
const osThreadAttr_t keyboardTask_attributes = {
  .name = "keyboardTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t  container_detectTaskHandle;
const osThreadAttr_t container_detectTask_attributes = {
  .name = "container_detectTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t scanerTaskHandle;
const osThreadAttr_t scanerTask_attributes = {
  .name = "scanerTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

// common

uint32_t Start_timer = 0;

const uint16_t Freq[8] = {11, 22, 44, 88, 112, 176, 352, 704};

// PA1 - S0, PA2 - S1, PA3 - S2, PA4 - S3

const uint32_t mux_control_data[MUX_SIZE] = {0x00000002, 0x00000004, 0x00000006, 0x00000008,
											0x0000000A, 0x0000000C, 0x0000000E, 0x00000010,
											0x00000012, 0x00000014, 0x00000016, 0x00000018,
											0x0000001A, 0x0000001C, 0x0000001E, 0x00000000};
uint32_t comp_out_data[MUX_SIZE];
uint8_t  comp_out_data_busy = 0;

uint32_t BufferCOMP1[LINE_SIZE_WITH_DUMMY], BufferCOMP2[LINE_SIZE_WITH_DUMMY];
uint8_t last_line[LINE_SIZE], NumObjectsInLastLine = 0;
uint16_t CurrentFrequencyFrame = 112;
uint32_t LCD_show_count_counter = 0;


uint32_t counter_num_extra_count = 0;
uint32_t numObjects = 0;

line_object_t *p_objects_last_line[LINE_SIZE /2];
line_object_t objects_last_line[LINE_SIZE / 2];

uint8_t objects_area_last_line[LINE_SIZE];

uint8_t current_line[LINE_SIZE], NumObjectsInCurrentLine = 0;
uint8_t lastbit = 0;
line_object_t objects_current_line[LINE_SIZE / 2];
line_object_t *p_objects_current_line[LINE_SIZE /2];

uint16_t Objects_area[1000];
uint32_t num_show_object_area = 0;
uint32_t max_area = 0;

uint8_t PositionLCD[8];
uint32_t pices_time[NUM_PICES_PERIOD];
uint32_t system_time = 0;
uint32_t pice_period = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_HS_USB_Init(void);
static void MX_TIM3_Init(void);
static void MX_COMP1_Init(void);
static void MX_COMP2_Init(void);
static void MX_TIM17_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

void StartLCDTask(void *argument);
void StartKeyboardTask(void *argument);
void StartContainerDetectTask(void *argument);
void StartScanerTask(void *argument);

void Nybble(void);
void WriteLCD(char i);
void CommandLCD(uint8_t i);
void TuningLCD (void);
void Clear_Counter (void);

void ShowTextOnLCD (const char *text);
void ShowNumberOnLCD (uint32_t number, uint16_t num_areas);

void Delay_us(uint32_t us);

void DMA1_Stream0_Complete_Callback(DMA_HandleTypeDef *hdma_tim17_ch1);

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

  ShowTextOnLCD("2.5    0");

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_HS_USB_Init();
  MX_TIM3_Init();
  MX_COMP1_Init();
  MX_COMP2_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

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

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  lcdTaskHandle = osThreadNew(StartLCDTask, NULL, &lcdTask_attributes);
  keyboardTaskHandle = osThreadNew(StartKeyboardTask, NULL, &keyboardTask_attributes);
  container_detectTaskHandle = osThreadNew(StartContainerDetectTask, NULL, &container_detectTask_attributes);
  scanerTaskHandle = osThreadNew(StartScanerTask, NULL, &scanerTask_attributes);

  event_group_1_id = osEventFlagsNew(NULL);

  __HAL_TIM_ENABLE_DMA(&htim17,TIM_DMA_ID_CC1);
  HAL_DMA_RegisterCallback(&hdma_tim17_ch1, HAL_DMA_XFER_CPLT_CB_ID, DMA1_Stream0_Complete_Callback);
  HAL_COMP_Start(&hcomp1);
  HAL_TIM_PWM_Start_IT (&htim3, TIM_CHANNEL_1);

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 34;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InvertingInput = COMP_INPUT_MINUS_IO1;
  hcomp1.Init.NonInvertingInput = COMP_INPUT_PLUS_IO2;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_MEDIUM;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp1.Init.Mode = COMP_POWERMODE_HIGHSPEED;
  hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}

/**
  * @brief COMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP2_Init(void)
{

  /* USER CODE BEGIN COMP2_Init 0 */

  /* USER CODE END COMP2_Init 0 */

  /* USER CODE BEGIN COMP2_Init 1 */

  /* USER CODE END COMP2_Init 1 */
  hcomp2.Instance = COMP2;
  hcomp2.Init.InvertingInput = COMP_INPUT_MINUS_IO1;
  hcomp2.Init.NonInvertingInput = COMP_INPUT_PLUS_IO2;
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.Hysteresis = COMP_HYSTERESIS_MEDIUM;
  hcomp2.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp2.Init.Mode = COMP_POWERMODE_HIGHSPEED;
  hcomp2.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP2_Init 2 */

  /* USER CODE END COMP2_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.MACAddr[0] =   0x00;
  heth.Init.MACAddr[1] =   0x80;
  heth.Init.MACAddr[2] =   0xE1;
  heth.Init.MACAddr[3] =   0x00;
  heth.Init.MACAddr[4] =   0x00;
  heth.Init.MACAddr[5] =   0x00;
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 275;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 400;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

/*
    TIM17->CNT = 0;
    TIM17->PSC = 0;                 // divisor PSC+1, Fsysclk = 72 Mhz => Fck = 72/(0+1) = 72 Mhz
    TIM17->ARR = (CurrentFrequencyFrame*2)-1;;              // 22*(1/72 MHz) = 305.55 ns period
    TIM17->CCR1 =  CurrentFrequencyFrame;               // 152.7 ns pulse 50 % PWM
    TIM17->CCMR1 |= TIM_CCMR1_OC1M_1 |TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1FE | TIM_CCMR1_OC1PE;; //OC1 is PWM mode1
    TIM17->CR1 |= TIM_CR1_ARPE;     //
    TIM17->CCER |= TIM_CCER_CC1E;   // OC1 signal is output on the corresponding output pin depending on MOE, OSSI, OSSR, OIS1, OIS1N and CC1NE bits
    TIM17->BDTR |= TIM_BDTR_MOE;    // main output enable
    TIM17->DIER |= TIM_DIER_UDE;    // Update DMA request enable
	*/

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 110;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 55;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

//  TIM17->DIER |= TIM_DIER_UDE;    // Update DMA request enable



  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, RW_LCD_Pin|D0_LCD_Pin|D1_LCD_Pin|D2_LCD_Pin
                          |D3_LCD_Pin|RS_LCD_Pin|E_LCD_Pin|LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, S0_Pin|S1_Pin|S2_Pin|S3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LIGHT_CONTROL_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_FS_PWR_EN_Pin|ROW0_Pin|ROW1_Pin|ROW2_Pin
                          |ROW3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RW_LCD_Pin D0_LCD_Pin D1_LCD_Pin D2_LCD_Pin
                           D3_LCD_Pin RS_LCD_Pin E_LCD_Pin LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = RW_LCD_Pin|D0_LCD_Pin|D1_LCD_Pin|D2_LCD_Pin
                          |D3_LCD_Pin|RS_LCD_Pin|E_LCD_Pin|LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MUX1_Pin MUX2_Pin MUX3_Pin MUX4_Pin
                           MUX5_Pin MUX6_Pin MUX7_Pin MUX8_Pin */
  GPIO_InitStruct.Pin = MUX1_Pin|MUX2_Pin|MUX3_Pin|MUX4_Pin
                          |MUX5_Pin|MUX6_Pin|MUX7_Pin|MUX8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : S0_Pin S1_Pin S2_Pin S3_Pin */
  GPIO_InitStruct.Pin = S0_Pin|S1_Pin|S2_Pin|S3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LIGHT_CONTROL_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LIGHT_CONTROL_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CONTAINER_DETECT_Pin */
  GPIO_InitStruct.Pin = CONTAINER_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CONTAINER_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LINE1_EOS_Pin LINE2_EOS_Pin */
  GPIO_InitStruct.Pin = LINE1_EOS_Pin|LINE2_EOS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_FS_PWR_EN_Pin ROW0_Pin ROW1_Pin ROW2_Pin
                           ROW3_Pin */
  GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin|ROW0_Pin|ROW1_Pin|ROW2_Pin
                          |ROW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(USB_FS_ID_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : COL0_Pin COL1_Pin COL2_Pin COL3_Pin */
  GPIO_InitStruct.Pin = COL0_Pin|COL1_Pin|COL2_Pin|COL3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*
void DMATuning(void)
{
    // for change addresses DMA chanal must disable !!!
	while (DMA1_Stream0->CR & DMA_SxCR_EN)
	{
		DMA1_Stream0->CR &=~ DMA_SxCR_EN;
	}
	DMA1->LISR = 0;
	DMA1->HISR = 0;
	DMA1_Stream0->PAR = (uint32_t)&COMP12->SR;
	DMA1_Stream0->M0AR = (uint32_t)&BufferCOMP1[0];
	//DMA1_Stream0->M1AR = (uint32_t)&BufferCOMP2[0];
	DMA1_Stream0->NDTR = LINE_SIZE_WITH_DUMMY;

	DMAMUX1

	DMA1_Stream0->CR |= DMA_SxCR_TCIE | DMA_SxCR_MINC |DMA_SxCR_PL_0 | DMA_SxCR_PL_1 | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_EN;
}
*/
/**
  * @brief  Function implementing the LCDTask thread.
  * @param  argument: Not used
  * @retval None
  */
void StartLCDTask(void *argument)
{
  /* Infinite loop */

	TuningLCD();

	for(;;)
	{
	//	osEventFlagsWait(event_group_1_id, FLAG_REQUEST_LCD_UPDATE, osFlagsWaitAny, osWaitForever);

		CommandLCD(0x01);   		// clear LCD pointer to 0 adsress SDRAM
		osDelay(2);
		WriteLCD(PositionLCD[0]);
		osDelay(1);//Delay_us(1000);//osDelay(2);
		WriteLCD(PositionLCD[1]);
		osDelay(1);//Delay_us(1000);//osDelay(2);
		WriteLCD(PositionLCD[2]);
		osDelay(1);//Delay_us(1000);//osDelay(2);
		WriteLCD(PositionLCD[3]);
		osDelay(1);//Delay_us(1000);//osDelay(2);
		WriteLCD(PositionLCD[4]);
		osDelay(1);//Delay_us(1000);//osDelay(2);
		WriteLCD(PositionLCD[5]);
		osDelay(1);//Delay_us(1000);//osDelay(2);
		WriteLCD(PositionLCD[6]);
		osDelay(1);//Delay_us(1000);//osDelay(2);
		WriteLCD(PositionLCD[7]);
		osDelay(250);
	}

}



/**
  * @brief  Function implementing the LCDTask thread.
  * @param  argument: Not used
  * @retval None
  */
void StartKeyboardTask(void *argument)
{
	uint8_t i = 0;
	uint32_t key_current_state = 0, key_previous_state = 0, key_current_status = 0, key_previous_status = 0, key_event_status = 0;

	/* Infinite loop */
  for(;;)
  {
	  /* polling keys :
	   *
	   * 	row0 - row3 is bit0 - bit3
	   * 	col0 - col3 is bit4 - bit7
	   *
	   *					col0 col1 col2 col3
	   *
	   * 				row0  1    2    3    A
	   * 				row1  4    5    6    B
	   * 				row2  7    8    9    C
	   * 				row3  *    0    #    D
	   *
	   *	in memory:
	   *
	   *				D#0*C987B654A321
	   */

	  key_current_state = 0;

	  for(i=0; i<4; i++)
	  {
		  KEY_GPIO_Port->BSRR |= (ROW0_Pin << i); // set row
		  HAL_Delay(1);
		  key_current_state |= ( ((KEY_GPIO_Port->IDR & 0x000000f0) >> 4) << (i * 4) );
		  KEY_GPIO_Port->BSRR |= (ROW0_Pin << (i + 16)); // clear row
		  HAL_Delay(1);
	  }

	  for (i=0; i<16; i++)
	  {
		  if (key_current_state & (1 << i))
		  {
			  if (key_previous_state & (1 << i))
			  {
				  key_current_status |= (1 << i);
			  }
		  }
		  else
		  {
			  if (!(key_previous_state & (1 << i)))
			  {
				  key_current_status &= ~(1 << i);
			  }
		  }

		  if ( (key_current_status & (1 << i)) && (!(key_previous_status & (1 << i))) )
		  {
			  key_event_status |= (1 << i);
		  }
	  }


	  key_previous_state = key_current_state;

	  key_previous_status = key_current_status;

	  // defining button events

	  if (key_event_status & KEY_1)
	  {
		  key_event_status &= ~KEY_1;

		  numObjects = 1;
		  ShowNumberOnLCD((uint32_t)numObjects, 0);
	  }

	  if (key_event_status & KEY_2)
	  {
		  key_event_status &= ~KEY_2;

		  numObjects = 2;
		  ShowNumberOnLCD((uint32_t)numObjects, 0);
	  }

	  if (key_event_status & KEY_3)
	  {
		  key_event_status &= ~KEY_3;

		  numObjects = 3;
		  ShowNumberOnLCD((uint32_t)numObjects, 0);
	  }

	  if (key_event_status & KEY_4)
	  {
		  key_event_status &= ~KEY_4;

		  numObjects = 4;
		  ShowNumberOnLCD((uint32_t)numObjects, 0);
	  }

	  if (key_event_status & KEY_5)
	  {
		  key_event_status &= ~KEY_5;

		  numObjects = 5;
		  ShowNumberOnLCD((uint32_t)numObjects, 0);
	  }

	  if (key_event_status & KEY_6)
	  {
		  key_event_status &= ~KEY_6;

		  numObjects = 6;
		  ShowNumberOnLCD((uint32_t)numObjects, 0);
	  }

	  if (key_event_status & KEY_7)
	  {
		  key_event_status &= ~KEY_7;

		  numObjects = 7;
		  ShowNumberOnLCD((uint32_t)numObjects, 0);
	  }

	  if (key_event_status & KEY_8)
	  {
		  key_event_status &= ~KEY_8;

		  numObjects = 8;
		  ShowNumberOnLCD((uint32_t)numObjects, 0);
	  }

	  if (key_event_status & KEY_9)
	  {
		  key_event_status &= ~KEY_9;

		  numObjects = 9;
		  ShowNumberOnLCD((uint32_t)numObjects, 0);
	  }

	  if (key_event_status & KEY_0)
	  {
		  key_event_status &= ~KEY_0;

		  numObjects = 10;
		  ShowNumberOnLCD((uint32_t)numObjects, 0);
	  }

	  if (key_event_status & KEY_A)
	  {
		  key_event_status &= ~KEY_A;

		  numObjects = 11;
		  ShowNumberOnLCD((uint32_t)numObjects, 0);
	  }

	  if (key_event_status & KEY_B)
	  {
		  key_event_status &= ~KEY_B;

		  numObjects = 12;
		  ShowNumberOnLCD((uint32_t)numObjects, 0);
	  }

	  // C key

	  if (key_event_status & KEY_C)
	  {
		  key_event_status &= ~KEY_C;

		  Clear_Counter();
		  ShowNumberOnLCD((uint32_t)numObjects, 0);
	  }

	  // SRAT key

	  if (key_event_status & KEY_STAR)
	  {
		  key_event_status &= ~KEY_STAR;

		  if (numObjects)
		  {
				if(num_show_object_area < numObjects)
				{
					num_show_object_area++;
				}
				else
				{
					num_show_object_area = 1;
				}
		  }
	  }

	  // GIRD key

	  if (key_event_status & KEY_GRID)
	  {
		  key_event_status &= ~KEY_GRID;

		  num_show_object_area = 0;
	  }

	  osDelay(100);
  }
}

/**
  * @brief  Function implementing the LCD Task thread.
  * @param  argument: Not used
  * @retval None
  */
void StartContainerDetectTask(void *argument)
{
	static uint8_t previous_state = 1, event_state = 0;

	/* Infinite loop */
  for(;;)
  {
	  if (HAL_GPIO_ReadPin(CONTAINER_DETECT_GPIO_Port, CONTAINER_DETECT_Pin))
	  {
		  previous_state = 1;
		  event_state = 0;
	  }
	  else
	  {
			if (!previous_state)
			{
				if(!event_state)
				{
					event_state = 1;

					Clear_Counter();
				}
			}

			previous_state = 0;
		}

	  osDelay(100);
  }

}

/**
  * @brief  Function implementing the Scaner Task thread.
  * @param  argument: Not used
  * @retval None
  */
void StartScanerTask(void *argument)
{
	uint32_t j, p, i;

	/* Infinite loop */
	for(;;)
	{
		osEventFlagsWait(event_group_1_id, FLAG_REQUEST_SCANER, osFlagsWaitAny | osFlagsNoClear, osWaitForever);

	  	NumObjectsInCurrentLine = 0;
	  	lastbit = 0;

	  	for (j = 0; j < LINE_SIZE; j++)
	  	{
	  		if(BufferCOMP2[j + LINE_DUMMY] & COMP_SR_C1VAL)
	  		{
	  			current_line[j] = 0;
	  			lastbit = 0;
	  		}
	  		else
	  		{
	  			if(!lastbit)
	  			{
	  				NumObjectsInCurrentLine++;

	  				p_objects_current_line[NumObjectsInCurrentLine-1] = &objects_current_line[NumObjectsInCurrentLine-1];

	  				p_objects_current_line[NumObjectsInCurrentLine-1]->area = 0;
	  			}

	  			current_line[j] = NumObjectsInCurrentLine;
	  			p_objects_current_line[NumObjectsInCurrentLine-1]->area++;
	  			lastbit = 1;

	  			if(last_line[j])
	  			{
	  				p_objects_last_line[last_line[j]-1]->cont = 1;

	  				if (!p_objects_last_line[last_line[j]-1]->sl)
	  				{
	  					p_objects_last_line[last_line[j]-1]->sl = current_line[j];
	  					p_objects_current_line[current_line[j]-1]->area += p_objects_last_line[last_line[j]-1]->area;
	  				}
	  				else
	  				{
	  					if (p_objects_current_line[p_objects_last_line[last_line[j]-1]->sl - 1] != p_objects_current_line[current_line[j]-1])
	  					{
	  						p_objects_current_line[p_objects_last_line[last_line[j]-1]->sl-1]->area += p_objects_current_line[current_line[j]-1]->area;

	  						p_objects_current_line[current_line[j]-1] = p_objects_current_line[p_objects_last_line[last_line[j]-1]->sl - 1];
	  					}
	  				}
	  			}
	  		}

	  		// анализируем �?в�?занно�?ть объектов текущей линии �? �?объектами предыдущей и �?тавим и ра�?�?тавл�?ем �?оответ�?твующие признаки

	  		last_line[j] = current_line[j];
	  	}

	  	// провер�?ем е�?ть ли закончившие�?�? объекты по предыдущей линии

	  	for (j=0; j < NumObjectsInLastLine; j++)
	  	{
	  		if (!p_objects_last_line[j]->cont)
	  		{
	  			if(numObjects == 1000)
	  			{
	  				numObjects = 0;
	  			}

	  			if (numObjects == 10)
	  			{
	  				for (i=0;i<10;i++)
	  				{
	  					max_area += Objects_area[i];
	  				}
	  				max_area /=10;

	  				if(max_area < 70)
	  				{
	  					max_area *=2.4;
	  				}
	  				else
	  				{
	  					max_area *=1.95;
	  				}
	  			}

	  			if(p_objects_last_line[j]->area < 2000)
	  			{
	  				if (max_area)
	  				{
	  					while (p_objects_last_line[j]->area)
	  					{
	  						if (p_objects_last_line[j]->area > max_area)
	  						{
	  							Objects_area[numObjects] = max_area;
	  							p_objects_last_line[j]->area -= max_area;
	  						}
	  						else
	  						{
	  							Objects_area[numObjects] = p_objects_last_line[j]->area;
	  							p_objects_last_line[j]->area = 0;
	  						}
	  						numObjects++;
	  					}
	  				}
	  				else
	  				{
	  					Objects_area[numObjects] = p_objects_last_line[j]->area;
	  					numObjects++;
	  				}

	  				for (p=1; p < NUM_PICES_PERIOD; p++)
	  				{
	  					pices_time[p-1] = pices_time[p];
	  				}

	  				pices_time[NUM_PICES_PERIOD - 1]  = system_time;

	  				if (numObjects > (NUM_PICES_PERIOD - 1))
	  				{
	  					pice_period = (pices_time[NUM_PICES_PERIOD - 1] - pices_time[0]) / NUM_PICES_PERIOD;
	  					if (pice_period < MIN_PICE_PERIOD)
	  					{
	  						if (counter_num_extra_count < 5)
	  						{
	  							counter_num_extra_count++;
	  							//ShowNumExtraCountOnLeds(counter_num_extra_count);

	  							for (p=0; p < NUM_PICES_PERIOD; p++)
	  							{
	  								pices_time[p] = 0;
	  							}
	  						}
	  					}
	  				}

	  			}
	  			else
	  			{
	  				osEventFlagsSet(event_group_1_id, ERROR_COUNT_OBJECTS);
	  			}
	  		}
	  	}

	  	// перено�?им объекты текущей линии в предыдущую

	  	for (j=0; j < NumObjectsInCurrentLine; j++)
	  	{
	  		p_objects_last_line[j] = &objects_last_line[0] + (p_objects_current_line[j] - &objects_current_line[0]);

	  		p_objects_last_line[j]->area = p_objects_current_line[j]->area;
	  		p_objects_last_line[j]->cont = 0;
	  		p_objects_last_line[j]->sl = 0;
	  	}

	  	NumObjectsInLastLine = NumObjectsInCurrentLine;

	  	// Мен�?ем буфер который надо заполн�?ть

	  	osEventFlagsClear(event_group_1_id, FLAG_REQUEST_SCANER);
  }

}

/*
 * *************** General Purpose Functions ***************
 */

void Clear_Counter (void)
{
	uint8_t p;

	counter_num_extra_count = 0;
	numObjects = 0;
	num_show_object_area = 0;
	max_area = 0;

	for (p=0; p < NUM_PICES_PERIOD; p++)
	{
		pices_time[p] = 0;
	}

	osEventFlagsSet(event_group_1_id, FLAG_REQUEST_LCD_UPDATE);
}

/*
 * Tuning LCD
 */

void TuningLCD (void)
{
	//4-bit Initialization:

	HAL_Delay(40);
	LCD_GPIO_Port->ODR &= ~(LCD_GPIO_DATA_Pins);
	LCD_GPIO_Port->ODR |= ( 0x03 << 3); // write 0x03
	HAL_Delay(5);
	Nybble(); 			//command 0x30 = Wake up
	HAL_Delay(1);
	Nybble(); 			//command 0x30 = Wake up #2
	HAL_Delay(1);
	Nybble(); 			//command 0x30 = Wake up #3
	HAL_Delay(10); 		//can check busy flag now instead of delay
	LCD_GPIO_Port->ODR &= ~(LCD_GPIO_DATA_Pins);
	LCD_GPIO_Port->ODR |= ( 0x02 << 3); // write 0x02
	HAL_Delay(1);
	Nybble(); 			//Function set: 4-bit interface
	HAL_Delay(1); 		//can check busy flag now instead of delay
	CommandLCD(0x28); 	//Function set: 4-bit/2-line
	HAL_Delay(1);
	CommandLCD(0x10); 	//Set cursor
	HAL_Delay(1);
	CommandLCD(0x0F); 	//Display ON
	HAL_Delay(1);
	CommandLCD(0x06); 	//Entry Mode set
	HAL_Delay(1);
}

/*
 *
 */

void CommandLCD(uint8_t i)
{

	LCD_GPIO_Port->ODR &= ~(LCD_GPIO_DATA_Pins);
	LCD_GPIO_Port->ODR |=  ( (i>>4) << 3) ;	// set lower bits
	RS_LCD_GPIO_Port->BSRR |= (RS_LCD_Pin << 16); // RS low - command
	RW_LCD_GPIO_Port->BSRR |= ( RW_LCD_Pin << 16); //R/W low
	Delay_us(1);
	Nybble(); //Send lower 4 bits
	Delay_us(1);
	LCD_GPIO_Port->ODR &= ~(LCD_GPIO_DATA_Pins);
	LCD_GPIO_Port->ODR |= ( (i & 0x0f) << 3); // set upper bits
	Delay_us(1);
	Nybble(); //Send upper 4 bits
	Delay_us(1);
}

/*
 *
 */

void WriteLCD(char i)
{
	LCD_GPIO_Port->ODR &= ~(LCD_GPIO_DATA_Pins);
	LCD_GPIO_Port->ODR |= ( (i>>4) << 3);	// set lower bits
	RS_LCD_GPIO_Port->BSRR |= RS_LCD_Pin; // RS high - data
	RW_LCD_GPIO_Port->BSRR |= ( RW_LCD_Pin << 16); // R/W low
	Delay_us(1);
	Nybble(); //Clock lower 4 bits
	Delay_us(1);
	LCD_GPIO_Port->ODR &= ~(LCD_GPIO_DATA_Pins);
	LCD_GPIO_Port->ODR |= ( (i & 0x0f) << 3); // set upper bits
	Delay_us(1);
	Nybble(); //Send upper 4 bits
	Delay_us(1);
}

/*
 *
 */

void Nybble(void)
{
	E_LCD_GPIO_Port->BSRR |= E_LCD_Pin;						// E = high

	for (uint8_t i=0; i== 55; i++) //140
	{
		asm ("nop");
	}

	E_LCD_GPIO_Port->BSRR |= (E_LCD_Pin << 16);				// E = low
}

/*
 *
 */

void Delay_us(uint32_t us)
{
	uint32_t delay_time = (550*us)/3;

	for (uint32_t i=0; i== delay_time; i++)
	{
		asm ("nop");
	}
}

/*
 *
 */

void ShowTextOnLCD (const char *text)
{
	PositionLCD[0] = (uint8_t)text[0];
	PositionLCD[1] = (uint8_t)text[1];
	PositionLCD[2] = (uint8_t)text[2];
	PositionLCD[3] = (uint8_t)text[3];
	PositionLCD[4] = (uint8_t)text[4];
	PositionLCD[5] = (uint8_t)text[5];
	PositionLCD[6] = (uint8_t)text[6];
	PositionLCD[7] = (uint8_t)text[7];

	osEventFlagsSet(event_group_1_id, FLAG_REQUEST_LCD_UPDATE);
}

void ShowNumberOnLCD (uint32_t number, uint16_t num_areas)
{
	uint8_t i;

	memset(PositionLCD, 0x30, sizeof(PositionLCD));

	while (number > 9999999) { number -=10000000, PositionLCD[0]++;}
	while (number > 999999) { number -=1000000, PositionLCD[1]++;}
	while (number > 99999) { number -=100000, PositionLCD[2]++;}
	while (number > 9999) { number -=10000, PositionLCD[3]++;}
	while (number > 999) { number -=1000, PositionLCD[4]++;}
	while (number > 99) { number -=100, PositionLCD[5]++;}
	while (number > 9) { number -=10, PositionLCD[6]++;}
	PositionLCD[7] += (uint8_t)number;

	for(i=0; i < 7; i++)
	{
		if (PositionLCD[i] == 0x30)
		{
			PositionLCD[i] = 0x20;
		}
		else
		{
			break;
		}
	}

	if (num_areas)
	{
		PositionLCD[0] = 0x30;
		PositionLCD[1] = 0x30;
		PositionLCD[2] = 0x30;
		PositionLCD[3] = 0x20;

		while (num_areas > 99) { num_areas -=100, PositionLCD[0]++;}
		while (num_areas > 9) { num_areas -=10, PositionLCD[1]++;}
		PositionLCD[2] += num_areas;

		for(i=0; i < 2; i++)
		{
			if (PositionLCD[i] == 0x30)
			{
				PositionLCD[i] = 0x20;
			}
			else
			{
				break;
			}
		}
	}

	osEventFlagsSet(event_group_1_id, FLAG_REQUEST_LCD_UPDATE);
}

/*
 *
 */

void TimersTuning(void)
{
        // timer for line polling

    TIM17->CNT = 0;
    TIM17->PSC = 0;                 // divisor PSC+1, Fsysclk = 72 Mhz => Fck = 72/(0+1) = 72 Mhz
    TIM17->ARR = (CurrentFrequencyFrame*2)-1;;              // 22*(1/72 MHz) = 305.55 ns period
    TIM17->CCR1 =  CurrentFrequencyFrame;               // 152.7 ns pulse 50 % PWM
    TIM17->CCMR1 |= TIM_CCMR1_OC1M_1 |TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1FE | TIM_CCMR1_OC1PE;; //OC1 is PWM mode1
    TIM17->CR1 |= TIM_CR1_ARPE;     //
    TIM17->CCER |= TIM_CCER_CC1E;   // OC1 signal is output on the corresponding output pin depending on MOE, OSSI, OSSR, OIS1, OIS1N and CC1NE bits
    TIM17->BDTR |= TIM_BDTR_MOE;    // main output enable
    TIM17->DIER |= TIM_DIER_UDE;    // Update DMA request enable

        // timer general time count 1 ms period

    TIM2->PSC = 71;             // divisor PSC+1, Fsysclk = 72 Mhz => Fck = 72/(71+1) = 1 Mhz
    TIM2->ARR = 1000;           // 1 ms period
    TIM2->DIER |= TIM_DIER_UIE; // enable TIM2 update interrupt

        // timer for LCD pause 300 nS pause for E signal

    TIM3->PSC = 71;                 // divisor PSC+1, Fck = 72/(71+1) = 1 Mhz
    //TIM3->CR1 |= TIM_CR1_CEN; // start TIM3
}

/*
 *
 */

void ComparatorsTuning(void)
{
	RCC->APB4ENR |= RCC_APB4ENR_COMP12EN;

	COMP1->CFGR = COMP_CFGRx_HYST_1 | COMP_CFGRx_INMSEL_1 | COMP_CFGRx_INMSEL_2 | COMP_CFGRx_EN; // COMP1: PB1 - minus, HighSpeed, Medium hysteresis, enable
}

void DMATuning(void)
{
    // for change addresses DMA chanal must disable !!!

	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;   // DMA2 clock enable;

    DMA1_Stream0->CR &=~DMA_SxCR_EN;
    DMA1_Stream0->PAR =(uint32_t)&COMP12->SR;
    DMA1_Stream0->M0AR =(uint32_t)&BufferCOMP1[0];
    DMA1_Stream0->M1AR =(uint32_t)&BufferCOMP2[0];
    DMA1_Stream0->NDTR = /*LINE_SIZE + LINE_DUMMY*/10;
    DMA1_Stream0->CR = DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC_1 | DMA_SxCR_TCIE | DMA_SxCR_PL_0 | DMA_SxCR_PL_1;
    DMAMUX1_Channel0->CCR = ( 111 << DMAMUX_CxCR_DMAREQ_ID_Pos); // TIM17_CH1 request
	DMA1_Stream0->CR |= DMA_SxCR_EN;

}

//********************************

void SystemInterruptsTuning(void)
{


    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_SetPriority(TIM3_IRQn, 10);

    /*
    EXTI->IMR |= EXTI_IMR_MR0;
    //EXTI->EMR |= EXTI_EMR_MR0;
    NVIC_SetPriority(EXTI0_IRQn, 1);
    NVIC_EnableIRQ(EXTI0_IRQn);
     */

    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    NVIC_SetPriority(DMA1_Stream0_IRQn, 10);
}

/*
void DMA1_Stream0_IRQHandler (void)
{

}
*/

/*
 *
 */

void HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef * htim)
{
	if (htim->Instance == TIM3)
	{
		//DMA_REQUEST_TIM17_CH1
		//HAL_DMA_Abort_IT (htim17.hdma[TIM_DMA_ID_CC1]);
		//__HAL_TIM_ENABLE_DMA(&htim17,TIM_DMA_ID_CC1);
		HAL_DMA_Start_IT (&hdma_tim17_ch1/*htim17.hdma[TIM_DMA_ID_CC1]*/, (uint32_t)&COMP12->SR, (uint32_t)BufferCOMP1, /*LINE_SIZE + LINE_DUMMY*/10);
		HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
	}
}

/*
 *
 */

void DMA1_Stream0_Complete_Callback(DMA_HandleTypeDef *hdma)
{
	HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);

	if (!(osEventFlagsGet(event_group_1_id) & FLAG_REQUEST_SCANER))
	{
		memcpy(BufferCOMP2, BufferCOMP1, sizeof(BufferCOMP2));
		osEventFlagsSet(event_group_1_id, FLAG_REQUEST_SCANER);
	}
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
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  if (htim->Instance == TIM2) {

  	  }

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
