/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "cobs.h"
#include "crc16.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define MAX_PACKET_SIZE 64
struct message
{
  uint8_t dest;
  uint8_t src;
  uint8_t flags;
  uint8_t ptype;
  uint16_t len;
  uint8_t rx_int;
  uint8_t tx_mask;
  char data[MAX_PACKET_SIZE];
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MSG_DEST_BCAST 0xff

#define MSG_FLAG_RADIO 0x10

#define MSG_TYPE_COMMAND 0x08
#define MSG_TYPE_DIO_STATE 0x10
#define MSG_TYPE_DIO_SET_BIT 0x11
#define MSG_TYPE_ANALOG_VALUE 0x20
#define MSG_TYPE_PING_REQ 0xfe
#define MSG_TYPE_PING_RESP 0xff

#define MSG_TX_MASK(x) (1 << (x))
#define MSG_INV_TX_MASK(x) ((~(1 << (x))) & 0xff)

#define MSG_RX_UART3 0x00
#define MSG_TX_UART3 MSG_TX_MASK(MSG_RX_UART3)

#define MSG_RX_UART4 0x01
#define MSG_TX_UART4 MSG_TX_MASK(MSG_RX_UART4)

#define MSG_RX_USB 0x02
#define MSG_TX_USB MSG_TX_MASK(MSG_RX_USB)

#define MSG_TX_ALL 0xff
#define MSG_TX_NONE 0x00

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 512 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId messageHandlerHandle;
uint32_t messageHandlerBuffer[ 1024 ];
osStaticThreadDef_t messageHandlerControlBlock;
osThreadId sgReadHandle;
uint32_t sgReadBuffer[ 512 ];
osStaticThreadDef_t sgReadControlBlock;
/* USER CODE BEGIN PV */

uint8_t usart3_rx_hw_buffer[1];
uint8_t usart3_rx_buffer[128];
StaticQueue_t usart3_rx_queue;
QueueHandle_t usart3_rx_queue_handle;
uint8_t usart3_active;
uint8_t usart3_tx_hw_buffer[1];
uint8_t usart3_tx_buffer[128];
StaticQueue_t usart3_tx_queue;
QueueHandle_t usart3_tx_queue_handle;

uint8_t usart4_rx_hw_buffer[1];
uint8_t usart4_rx_buffer[128];
StaticQueue_t usart4_rx_queue;
QueueHandle_t usart4_rx_queue_handle;
uint8_t usart4_active;
uint8_t usart4_tx_hw_buffer[1];
uint8_t usart4_tx_buffer[128];
StaticQueue_t usart4_tx_queue;
QueueHandle_t usart4_tx_queue_handle;

uint8_t usb_cdc_rx_buffer[128];
StaticQueue_t usb_cdc_rx_queue;
QueueHandle_t usb_cdc_rx_queue_handle;

uint8_t usb_cdc_tx_buffer[128];
int usb_cdc_tx_buffer_ptr = 0;

uint8_t usart3_pkt_buffer[128];
size_t usart3_pkt_buffer_ptr = 0;

uint8_t usart4_pkt_buffer[128];
size_t usart4_pkt_buffer_ptr = 0;

uint8_t usb_cdc_pkt_buffer[128];
size_t usb_cdc_pkt_buffer_ptr = 0;


// message queues
#define RX_MSG_QUEUE_LEN 8
uint8_t rx_msg_buffer[RX_MSG_QUEUE_LEN*(sizeof(struct message))];
StaticQueue_t rx_msg_queue;
QueueHandle_t rx_msg_queue_handle;

#define TX_MSG_QUEUE_LEN 8
uint8_t tx_msg_buffer[TX_MSG_QUEUE_LEN*(sizeof(struct message))];
StaticQueue_t tx_msg_queue;
QueueHandle_t tx_msg_queue_handle;


uint16_t adc_dma_buffer[16];

uint16_t solenoid_state = 0x8000;

uint8_t arm_state = 0;
uint8_t fire_cmd = 0;

uint8_t dev_id = 0x40;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);
void startMessageHandler(void const * argument);
void startSgRead(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// printf() via SWO
void swo_printf(const char *fmt, ...)
{
  char buf[128];
  va_list argp;
  va_start(argp, fmt);
  if (0 < vsnprintf(buf, sizeof(buf), fmt, argp))
  {
    for (char *ptr = buf; *ptr; ptr++)
    {
      ITM_SendChar(*ptr);
    }
  }
  va_end(argp);
}

// Event handlers

//void tick_handler(void)
//{
//  // HAL_GetTick();
//}

//void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
//void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
//void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c);
//void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);
//void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);
//void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c);
//void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
//void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
//void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
//void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c);

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
//void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart);
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
//void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
//void HAL_UART_AbortCpltCallback (UART_HandleTypeDef *huart);
//void HAL_UART_AbortTransmitCpltCallback (UART_HandleTypeDef *huart);
//void HAL_UART_AbortReceiveCpltCallback (UART_HandleTypeDef *huart);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3)
  {
    // TX complete for UART3
    if (uxQueueMessagesWaitingFromISR(usart3_tx_queue_handle))
    {
      usart3_active = 1;
      xQueueReceiveFromISR(usart3_tx_queue_handle, usart3_tx_hw_buffer, NULL);
      HAL_UART_Transmit_IT(&huart3, usart3_tx_hw_buffer, 1);
    }
    else
    {
      usart3_active = 0;
    }
  }
  else if (huart == &huart4)
//  if (huart == &huart4)
  {
    // TX complete for UART4
    if (uxQueueMessagesWaitingFromISR(usart4_tx_queue_handle))
    {
      usart4_active = 1;
      xQueueReceiveFromISR(usart4_tx_queue_handle, usart4_tx_hw_buffer, NULL);
      HAL_UART_Transmit_IT(&huart4, usart4_tx_hw_buffer, 1);
    }
    else
    {
      usart4_active = 0;
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3)
  {
    // RX complete for USART3
    xQueueSendFromISR(usart3_rx_queue_handle, usart3_rx_hw_buffer, NULL);
    HAL_UART_Receive_IT(&huart3, usart3_rx_hw_buffer, 1);
  }
  else if (huart == &huart4)
  //if (huart == &huart4)
  {
    // RX complete for UART4
    xQueueSendFromISR(usart4_rx_queue_handle, usart4_rx_hw_buffer, NULL);
    HAL_UART_Receive_IT(&huart4, usart4_rx_hw_buffer, 1);
  }
}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
//void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);
//void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc);
//void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc);

uint32_t adc_ref_acc;
uint32_t adc_acc[8];

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc == &hadc1)
  {
    // exponential moving average

    adc_ref_acc = adc_ref_acc * 0.9999 + (adc_dma_buffer[0] << 16) * 0.0001;
    //adc_ref_acc += ((adc_dma_buffer[0] << 16) - adc_ref_acc) * 0.0001;

    for (int i = 0; i < 8; i++)
    {
      adc_acc[i] = adc_acc[i] * 0.9999 + (adc_dma_buffer[i+1] << 16) * 0.0001;
      //adc_acc[i] += ((adc_dma_buffer[i+1] << 16) - adc_acc[i]) * 0.0001;
    }

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, 9);
  }
}

void usb_cdc_rx_callback(uint8_t *buf, uint32_t len)
{
  for (int i = 0; i < len; i++)
  {
    xQueueSendFromISR(usb_cdc_rx_queue_handle, &buf[i], NULL);
  }
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

  HAL_Delay(100);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  usart3_rx_queue_handle = xQueueCreateStatic(sizeof(usart3_rx_buffer), 1, usart3_rx_buffer, &usart3_rx_queue);
  usart3_tx_queue_handle = xQueueCreateStatic(sizeof(usart3_tx_buffer), 1, usart3_tx_buffer, &usart3_tx_queue);

  HAL_UART_Receive_IT(&huart3, usart3_rx_hw_buffer, 1);

  usart4_rx_queue_handle = xQueueCreateStatic(sizeof(usart4_rx_buffer), 1, usart4_rx_buffer, &usart4_rx_queue);
  usart4_tx_queue_handle = xQueueCreateStatic(sizeof(usart4_tx_buffer), 1, usart4_tx_buffer, &usart4_tx_queue);

  HAL_UART_Receive_IT(&huart4, usart4_rx_hw_buffer, 1);

  usb_cdc_rx_queue_handle = xQueueCreateStatic(sizeof(usb_cdc_rx_buffer), 1, usb_cdc_rx_buffer, &usb_cdc_rx_queue);

  rx_msg_queue_handle = xQueueCreateStatic(RX_MSG_QUEUE_LEN, sizeof(struct message), rx_msg_buffer, &rx_msg_queue);
  tx_msg_queue_handle = xQueueCreateStatic(TX_MSG_QUEUE_LEN, sizeof(struct message), tx_msg_buffer, &tx_msg_queue);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, 9);



  /* USER CODE END 2 */

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
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of messageHandler */
  osThreadStaticDef(messageHandler, startMessageHandler, osPriorityNormal, 0, 1024, messageHandlerBuffer, &messageHandlerControlBlock);
  messageHandlerHandle = osThreadCreate(osThread(messageHandler), NULL);

  /* definition and creation of sgRead */
  osThreadStaticDef(sgRead, startSgRead, osPriorityNormal, 0, 512, sgReadBuffer, &sgReadControlBlock);
  sgReadHandle = osThreadCreate(osThread(sgRead), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 9;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, EM_MAIN_GO_Pin|V2_PWR_EN_Pin|S_CTL_1_Pin|S_CTL_2_Pin 
                          |S_CTL_3_Pin|S_CTL_4_Pin|S_CTL_5_Pin|S_CTL_6_Pin 
                          |S_CTL_7_Pin|S_CTL_8_Pin|S_CTL_9_Pin|EM_BACK_GO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, S_CTL_10_Pin|S_CTL_11_Pin|S_CTL_12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin 
                          |SG2_CLK_Pin|SG1_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SG3_CLK_GPIO_Port, SG3_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EM_MAIN_GO_Pin V2_PWR_EN_Pin S_CTL_1_Pin S_CTL_2_Pin 
                           S_CTL_3_Pin S_CTL_4_Pin S_CTL_5_Pin S_CTL_6_Pin 
                           S_CTL_7_Pin S_CTL_8_Pin S_CTL_9_Pin EM_BACK_GO_Pin */
  GPIO_InitStruct.Pin = EM_MAIN_GO_Pin|V2_PWR_EN_Pin|S_CTL_1_Pin|S_CTL_2_Pin 
                          |S_CTL_3_Pin|S_CTL_4_Pin|S_CTL_5_Pin|S_CTL_6_Pin 
                          |S_CTL_7_Pin|S_CTL_8_Pin|S_CTL_9_Pin|EM_BACK_GO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : EM_MAIN_SENSE_Pin KEY_Pin EM_BACK_SENSE_Pin */
  GPIO_InitStruct.Pin = EM_MAIN_SENSE_Pin|KEY_Pin|EM_BACK_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S_CTL_10_Pin S_CTL_11_Pin S_CTL_12_Pin */
  GPIO_InitStruct.Pin = S_CTL_10_Pin|S_CTL_11_Pin|S_CTL_12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin 
                           SG2_CLK_Pin SG1_CLK_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin 
                          |SG2_CLK_Pin|SG1_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SG3_DAT_Pin */
  GPIO_InitStruct.Pin = SG3_DAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SG3_DAT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SG3_CLK_Pin */
  GPIO_InitStruct.Pin = SG3_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SG3_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SG1_DAT_Pin SG2_DAT_Pin */
  GPIO_InitStruct.Pin = SG1_DAT_Pin|SG2_DAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 5 */
  struct message msg;

  uint32_t next_update = 0;
  uint8_t update = 0;

  // size_t len;
  // uint16_t crc;
  // uint8_t ch;

  uint32_t ref;
  uint32_t vals[8];

  uint16_t state;
  int fire_release_time = 0;

  while (1)
  {
    update = 0;

    msg.dest    = MSG_DEST_BCAST;
    msg.src     = dev_id;
    msg.flags   = 0x00;
    //msg.tx_mask = 0;
    msg.tx_mask = MSG_TX_UART3 | MSG_TX_USB;

    if (next_update <= osKernelSysTick())
    {
      update = 1;
      next_update += 100;
      //msg.tx_mask |= MSG_TX_UART3;
    }

    if (update)
    {
      // Read PT with ADC
      ref = adc_ref_acc;
      for (int i = 0; i < 8; i++)
      {
        // rescale to 10,000 counts per volt
        // assuming ref is 1.21 volts
        //vals[i] = (((uint32_t)adc_dma_buffer[i+1])*12100)/ref;
        //vals[i] = adc_dma_buffer[i+1];
        vals[i] = (((uint64_t)adc_acc[i])*12100)/ref;
        //vals[i] = adc_acc[i] >> 16;
      }

      msg.ptype   = 0x20;
      msg.len     = 0;
      msg.data[msg.len++] = 0;
      msg.data[msg.len++] = 0;
      for (int i = 0; i < 8 ; i++)
      {
        msg.data[msg.len++] = vals[i] & 0xff;
        msg.data[msg.len++] = (vals[i] >> 8) & 0xff;
      }
      xQueueSend(tx_msg_queue_handle, &msg, 0);

      //HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, 8);
      //HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, 7);

      // Send solenoid states
      msg.ptype   = 0x10;
      msg.len     = 0;
      msg.data[msg.len++] = 0;
      msg.data[msg.len++] = solenoid_state & 0xff;
      msg.data[msg.len++] = solenoid_state >> 8;
      xQueueSend(tx_msg_queue_handle, &msg, 0);

      // Send e-match states
      state = 0;

      if (arm_state) state |= 1 << 8;
      if (fire_cmd) state |= 1 << 9;
      if (HAL_GPIO_ReadPin(EM_MAIN_SENSE_GPIO_Port, EM_MAIN_SENSE_Pin)) state |= 1 << 0;
      if (HAL_GPIO_ReadPin(EM_BACK_SENSE_GPIO_Port, EM_BACK_SENSE_Pin)) state |= 1 << 1;
      if (HAL_GPIO_ReadPin(EM_MAIN_GO_GPIO_Port, EM_MAIN_GO_Pin)) state |= 1 << 2;
      if (HAL_GPIO_ReadPin(EM_BACK_GO_GPIO_Port, EM_BACK_GO_Pin)) state |= 1 << 3;
      if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) state |= 1 << 4;

      // swo_printf("state: 0x%04x\n", state);

      msg.ptype   = 0x10;
      msg.len     = 0;
      msg.data[msg.len++] = 1;
      msg.data[msg.len++] = state & 0xff;
      msg.data[msg.len++] = state >> 8;
      xQueueSend(tx_msg_queue_handle, &msg, 0);
    }

    // Update solenoid outputs
    HAL_GPIO_WritePin(S_CTL_1_GPIO_Port, S_CTL_1_Pin, solenoid_state & (1 << 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(S_CTL_2_GPIO_Port, S_CTL_2_Pin, solenoid_state & (1 << 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(S_CTL_3_GPIO_Port, S_CTL_3_Pin, solenoid_state & (1 << 2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(S_CTL_4_GPIO_Port, S_CTL_4_Pin, solenoid_state & (1 << 3) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(S_CTL_5_GPIO_Port, S_CTL_5_Pin, solenoid_state & (1 << 4) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(S_CTL_6_GPIO_Port, S_CTL_6_Pin, solenoid_state & (1 << 5) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(S_CTL_7_GPIO_Port, S_CTL_7_Pin, solenoid_state & (1 << 6) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(S_CTL_8_GPIO_Port, S_CTL_8_Pin, solenoid_state & (1 << 7) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(S_CTL_9_GPIO_Port, S_CTL_9_Pin, solenoid_state & (1 << 8) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(S_CTL_10_GPIO_Port, S_CTL_10_Pin, solenoid_state & (1 << 9) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(S_CTL_11_GPIO_Port, S_CTL_11_Pin, solenoid_state & (1 << 10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(S_CTL_12_GPIO_Port, S_CTL_12_Pin, solenoid_state & (1 << 11) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(V2_PWR_EN_GPIO_Port, V2_PWR_EN_Pin, solenoid_state & (1 << 15) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // e-match logic
    if (fire_cmd)
    {
      fire_cmd = 0;

      if (arm_state && HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
      {
        HAL_GPIO_WritePin(EM_MAIN_GO_GPIO_Port, EM_MAIN_GO_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(EM_BACK_GO_GPIO_Port, EM_BACK_GO_Pin, GPIO_PIN_SET);

        fire_release_time = osKernelSysTick() + 3000;
      }
    }

    if (!arm_state || !HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) || fire_release_time <= osKernelSysTick())
    {
      HAL_GPIO_WritePin(EM_MAIN_GO_GPIO_Port, EM_MAIN_GO_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(EM_BACK_GO_GPIO_Port, EM_BACK_GO_Pin, GPIO_PIN_RESET);
    }

    osDelay(10);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_startMessageHandler */
/**
* @brief Function implementing the messageHandler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startMessageHandler */
void startMessageHandler(void const * argument)
{
  /* USER CODE BEGIN startMessageHandler */
  struct message msg;
  uint8_t pkt_buffer[128];
  uint8_t pkt_buffer_2[128];
  uint8_t ch;
  size_t len;
  uint16_t crc;

  while(1)
  {
    // USART 3 (GSE control)
    while (uxQueueMessagesWaiting(usart3_rx_queue_handle))
    {
      xQueueReceive(usart3_rx_queue_handle, &ch, 0);

      if (ch == 0)
      {
        len = cobs_decode(usart3_pkt_buffer, usart3_pkt_buffer_ptr, pkt_buffer);

        // swo_printf("Got packet [UART3]\n");

        // for (int i = 0; i < usart3_pkt_buffer_ptr; i++)
        // {
        //   swo_printf("%02x ", usart3_pkt_buffer[i]);
        // }
        // swo_printf("\n");

        if (len > 6)
        {

          if (crc16_block(pkt_buffer, len) == 0)
          {
            // swo_printf("Got packet [UART3]\n");

            // for (int i = 0; i < len; i++)
            // {
            //   swo_printf("%02x ", pkt_buffer[i]);
            // }
            // swo_printf("\n");

            msg.dest    = pkt_buffer[0];
            msg.src     = pkt_buffer[1];
            msg.flags   = pkt_buffer[2];
            msg.ptype   = pkt_buffer[3];
            msg.rx_int  = MSG_RX_UART3;
            msg.tx_mask = MSG_TX_NONE;
            msg.len     = len-6;
            memcpy(&msg.data, &pkt_buffer[4], msg.len);
            xQueueSend(rx_msg_queue_handle, &msg, 0);
          }
          else
          {
            // bad CRC
            swo_printf("Bad CRC\n");
          }
        }
        usart3_pkt_buffer_ptr = 0;
      }
      else
      {
        if (usart3_pkt_buffer_ptr == sizeof(usart3_pkt_buffer))
          usart3_pkt_buffer_ptr = 0;
        usart3_pkt_buffer[usart3_pkt_buffer_ptr++] = ch;
      }
    }

    // USART 4 (GSE umbilical)
    while (uxQueueMessagesWaiting(usart4_rx_queue_handle))
    {
      xQueueReceive(usart4_rx_queue_handle, &ch, 0);

      if (ch == 0)
      {
        len = cobs_decode(usart4_pkt_buffer, usart4_pkt_buffer_ptr, pkt_buffer);

        // swo_printf("Got packet [UART4]\n");

        // for (int i = 0; i < usart4_pkt_buffer_ptr; i++)
        // {
        //   swo_printf("%02x ", usart4_pkt_buffer[i]);
        // }
        // swo_printf("\n");

        if (len > 6)
        {

          if (crc16_block(pkt_buffer, len) == 0)
          {
            // swo_printf("Got packet [UART4]\n");

            // for (int i = 0; i < len; i++)
            // {
            //   swo_printf("%02x ", pkt_buffer[i]);
            // }
            // swo_printf("\n");

            msg.dest    = pkt_buffer[0];
            msg.src     = pkt_buffer[1];
            msg.flags   = pkt_buffer[2];
            msg.ptype   = pkt_buffer[3];
            msg.rx_int  = MSG_RX_UART4;
            msg.tx_mask = MSG_TX_NONE;
            msg.len     = len-6;
            memcpy(&msg.data, &pkt_buffer[4], msg.len);
            xQueueSend(rx_msg_queue_handle, &msg, 0);
          }
          else
          {
            // bad CRC
            swo_printf("Bad CRC\n");
          }
        }
        usart4_pkt_buffer_ptr = 0;
      }
      else
      {
        if (usart4_pkt_buffer_ptr == sizeof(usart4_pkt_buffer))
          usart4_pkt_buffer_ptr = 0;
        usart4_pkt_buffer[usart4_pkt_buffer_ptr++] = ch;
      }
    }

    // USB CDC
    while (uxQueueMessagesWaiting(usb_cdc_rx_queue_handle))
    {
      xQueueReceive(usb_cdc_rx_queue_handle, &ch, 0);

      if (ch == 0)
      {
        len = cobs_decode(usb_cdc_pkt_buffer, usb_cdc_pkt_buffer_ptr, pkt_buffer);

        // swo_printf("Got packet [USB CDC]\n");

        // for (int i = 0; i < usb_cdc_pkt_buffer_ptr; i++)
        // {
        //   swo_printf("%02x ", usb_cdc_pkt_buffer[i]);
        // }
        // swo_printf("\n");

        if (len > 6)
        {

          if (crc16_block(pkt_buffer, len) == 0)
          {
            // swo_printf("Got packet [USB CDC]\n");

            // for (int i = 0; i < len; i++)
            // {
            //   swo_printf("%02x ", pkt_buffer[i]);
            // }
            // swo_printf("\n");

            msg.dest    = pkt_buffer[0];
            msg.src     = pkt_buffer[1];
            msg.flags   = pkt_buffer[2];
            msg.ptype   = pkt_buffer[3];
            msg.rx_int  = MSG_RX_USB;
            msg.tx_mask = MSG_TX_NONE;
            msg.len     = len-6;
            memcpy(&msg.data, &pkt_buffer[4], msg.len);
            xQueueSend(rx_msg_queue_handle, &msg, 0);
          }
          else
          {
            // bad CRC
            swo_printf("Bad CRC\n");
          }
        }
        usb_cdc_pkt_buffer_ptr = 0;
      }
      else
      {
        if (usb_cdc_pkt_buffer_ptr == sizeof(usb_cdc_pkt_buffer))
          usb_cdc_pkt_buffer_ptr = 0;
        usb_cdc_pkt_buffer[usb_cdc_pkt_buffer_ptr++] = ch;
      }
    }

    // Process received messages
    while (uxQueueMessagesWaiting(rx_msg_queue_handle))
    {
      xQueueReceive(rx_msg_queue_handle, &msg, 0);

      if (msg.dest == MSG_DEST_BCAST || msg.dest != dev_id)
      {
        // forward packet
        msg.tx_mask = MSG_INV_TX_MASK(msg.rx_int);

        xQueueSend(tx_msg_queue_handle, &msg, 0);
      }

      if (msg.dest == MSG_DEST_BCAST || msg.dest == dev_id)
      {
        // address check passed

        if (msg.ptype == MSG_TYPE_PING_REQ)
        {
          // echo request
          msg.dest = msg.src;
          msg.src = dev_id;
          msg.ptype = MSG_TYPE_PING_RESP;
          msg.tx_mask = MSG_TX_MASK(msg.rx_int); // loop it back

          xQueueSend(tx_msg_queue_handle, &msg, 0);
        }
        else if (msg.ptype == MSG_TYPE_COMMAND)
        {
          uint32_t cmd = 0;

          cmd = msg.data[0];
          cmd |= msg.data[1] << 8;
          cmd |= msg.data[2] << 16;
          cmd |= msg.data[3] << 24;

          // swo_printf("Cmd packet: 0x%08x\n", cmd);

          switch (cmd)
          {
            case 0x004001F0:
              // fire e-match command
              fire_cmd = 1;
              break;
            case 0x004001FF:
              // set arm state command
              arm_state = msg.data[4];
              break;
          }
        }
        else if (msg.ptype == MSG_TYPE_DIO_SET_BIT)
        {
          uint8_t bank = msg.data[0];
          uint16_t mask = 1 << msg.data[1];

          switch (bank)
          {
            case 0:
              if (msg.data[2])
                solenoid_state |= mask;
              else
                solenoid_state &= ~mask;
              break;
          }
        }
      }
    }

    // Transmit messages
    while (uxQueueMessagesWaiting(tx_msg_queue_handle))
    {
      xQueueReceive(tx_msg_queue_handle, &msg, 0);

      pkt_buffer[0] = msg.dest;
      pkt_buffer[1] = msg.src;
      pkt_buffer[2] = msg.flags;
      pkt_buffer[3] = msg.ptype;
      memcpy(&pkt_buffer[4], &msg.data, msg.len);

      crc = crc16_block(pkt_buffer, msg.len+4);

      pkt_buffer[msg.len+4] = crc & 0xff;
      pkt_buffer[msg.len+5] = crc >> 8;

      len = cobs_encode(pkt_buffer, msg.len+6, pkt_buffer_2);
      pkt_buffer_2[len++] = 0;

      // UART 3 (GSE control)
      if (msg.tx_mask & MSG_TX_UART3)
      {
        for (int i = 0; i < len; i++)
        {
          ch = pkt_buffer_2[i];
          xQueueSend(usart3_tx_queue_handle, &ch, 10);
          if (!usart3_active)
          {
            usart3_active = 1;
            xQueueReceive(usart3_tx_queue_handle, usart3_tx_hw_buffer, 0);
            HAL_UART_Transmit_IT(&huart3, usart3_tx_hw_buffer, 1);
          }
        }
      }

      // UART 4 (GSE umbilical)
      if (msg.tx_mask & MSG_TX_UART4)
      {
        for (int i = 0; i < len; i++)
        {
          ch = pkt_buffer_2[i];
          xQueueSend(usart4_tx_queue_handle, &ch, 10);
          if (!usart4_active)
          {
            usart4_active = 1;
            xQueueReceive(usart4_tx_queue_handle, usart4_tx_hw_buffer, 0);
            HAL_UART_Transmit_IT(&huart4, usart4_tx_hw_buffer, 1);
          }
        }
      }

      // USB CDC
      if (msg.tx_mask & MSG_TX_USB)
      {
        //usb_cdc_tx(pkt_buffer_2, len);
        if (usb_cdc_tx_buffer_ptr + len < sizeof(usb_cdc_tx_buffer))
        {
          memcpy(usb_cdc_tx_buffer+usb_cdc_tx_buffer_ptr, pkt_buffer_2, len);
          usb_cdc_tx_buffer_ptr += len;
        }
      }
    }

    // USB CDC
    if (usb_cdc_tx_buffer_ptr)
    {
      if (usb_cdc_tx(usb_cdc_tx_buffer, usb_cdc_tx_buffer_ptr) == 0)
      {
        usb_cdc_tx_buffer_ptr = 0;
      }
    }

    osDelay(1);
  }
  /* USER CODE END startMessageHandler */
}

/* USER CODE BEGIN Header_startSgRead */
/**
* @brief Function implementing the sgRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startSgRead */
void startSgRead(void const * argument)
{
  /* USER CODE BEGIN startSgRead */
  int32_t sg[3];

  while (1)
  {
    // read strain gauges
    sg[0] = 0;
    sg[1] = 0;
    sg[2] = 0;

    for (int i = 0; i < 24; i++)
    {
      // clock pulse
      HAL_GPIO_WritePin(SG1_CLK_GPIO_Port, SG1_CLK_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(SG2_CLK_GPIO_Port, SG2_CLK_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(SG3_CLK_GPIO_Port, SG3_CLK_Pin, GPIO_PIN_SET);

      //osDelay(1);
      for (int j = 0; j < 56; j++) __NOP();

      HAL_GPIO_WritePin(SG1_CLK_GPIO_Port, SG1_CLK_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SG2_CLK_GPIO_Port, SG2_CLK_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SG3_CLK_GPIO_Port, SG3_CLK_Pin, GPIO_PIN_RESET);

      sg[0] = (sg[0] << 1) | HAL_GPIO_ReadPin(SG1_DAT_GPIO_Port, SG1_DAT_Pin);
      sg[1] = (sg[1] << 1) | HAL_GPIO_ReadPin(SG2_DAT_GPIO_Port, SG2_DAT_Pin);
      sg[2] = (sg[2] << 1) | HAL_GPIO_ReadPin(SG3_DAT_GPIO_Port, SG3_DAT_Pin);

      //osDelay(1);
      for (int j = 0; j < 56; j++) __NOP();
    }

    // clock pulse
    HAL_GPIO_WritePin(SG1_CLK_GPIO_Port, SG1_CLK_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SG2_CLK_GPIO_Port, SG2_CLK_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SG3_CLK_GPIO_Port, SG3_CLK_Pin, GPIO_PIN_SET);

    //osDelay(1);
    for (int j = 0; j < 56; j++) __NOP();

    HAL_GPIO_WritePin(SG1_CLK_GPIO_Port, SG1_CLK_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SG2_CLK_GPIO_Port, SG2_CLK_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SG3_CLK_GPIO_Port, SG3_CLK_Pin, GPIO_PIN_RESET);

    // sign extend
    if (sg[0] & (1 << 23)) sg[0] |= 0xff000000;
    if (sg[1] & (1 << 23)) sg[1] |= 0xff000000;
    if (sg[2] & (1 << 23)) sg[2] |= 0xff000000;

    swo_printf("SG: %ld, %ld, %ld\n", sg[0], sg[1], sg[2]);

    osDelay(1000);
  }
  /* USER CODE END startSgRead */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
