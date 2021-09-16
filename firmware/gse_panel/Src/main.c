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
#include "message.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MSG_RX_UART1 0x00
#define MSG_TX_UART1 MSG_TX_MASK(MSG_RX_UART1)

#define MSG_RX_UART2 0x01
#define MSG_TX_UART2 MSG_TX_MASK(MSG_RX_UART2)

#define MSG_RX_USB 0x02
#define MSG_TX_USB MSG_TX_MASK(MSG_RX_USB)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId messageHandlerHandle;
uint32_t messageHandlerBuffer[ 512 ];
osStaticThreadDef_t messageHandlerControlBlock;
/* USER CODE BEGIN PV */

uint8_t usart1_rx_hw_buffer[1];
uint8_t usart1_rx_buffer[128];
StaticQueue_t usart1_rx_queue;
QueueHandle_t usart1_rx_queue_handle;
uint8_t usart1_active;
uint8_t usart1_tx_hw_buffer[1];
uint8_t usart1_tx_buffer[128];
StaticQueue_t usart1_tx_queue;
QueueHandle_t usart1_tx_queue_handle;

uint8_t usart2_rx_hw_buffer[1];
uint8_t usart2_rx_buffer[128];
StaticQueue_t usart2_rx_queue;
QueueHandle_t usart2_rx_queue_handle;
uint8_t usart2_active;
uint8_t usart2_tx_hw_buffer[1];
uint8_t usart2_tx_buffer[128];
StaticQueue_t usart2_tx_queue;
QueueHandle_t usart2_tx_queue_handle;

uint8_t usb_cdc_rx_buffer[128];
StaticQueue_t usb_cdc_rx_queue;
QueueHandle_t usb_cdc_rx_queue_handle;

uint8_t usb_cdc_tx_buffer[128];
int usb_cdc_tx_buffer_ptr = 0;

uint8_t usart1_pkt_buffer[128];
size_t usart1_pkt_buffer_ptr = 0;

uint8_t usart2_pkt_buffer[128];
size_t usart2_pkt_buffer_ptr = 0;

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


uint8_t dev_id = 0x8F;

uint8_t tx_seq = 0x00;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartMessageHandler(void const * argument);

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
  if (huart == &huart1)
  {
    // TX complete for USART1
    if (uxQueueMessagesWaitingFromISR(usart1_tx_queue_handle))
    {
      usart1_active = 1;
      xQueueReceiveFromISR(usart1_tx_queue_handle, usart1_tx_hw_buffer, NULL);
      HAL_UART_Transmit_IT(&huart1, usart1_tx_hw_buffer, 1);
    }
    else
    {
      usart1_active = 0;
    }
  }
  else if (huart == &huart2)
//  if (huart == &huart2)
  {
    // TX complete for USART2
    if (uxQueueMessagesWaitingFromISR(usart2_tx_queue_handle))
    {
      usart2_active = 1;
      xQueueReceiveFromISR(usart2_tx_queue_handle, usart2_tx_hw_buffer, NULL);
      HAL_UART_Transmit_IT(&huart2, usart2_tx_hw_buffer, 1);
    }
    else
    {
      usart2_active = 0;
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    // RX complete for USART1
    xQueueSendFromISR(usart1_rx_queue_handle, usart1_rx_hw_buffer, NULL);
    HAL_UART_Receive_IT(&huart1, usart1_rx_hw_buffer, 1);
  }
  else if (huart == &huart2)
  //if (huart == &huart2)
  {
    // RX complete for USART2
    xQueueSendFromISR(usart2_rx_queue_handle, usart2_rx_hw_buffer, NULL);
    HAL_UART_Receive_IT(&huart2, usart2_rx_hw_buffer, 1);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  usart1_rx_queue_handle = xQueueCreateStatic(sizeof(usart1_rx_buffer), 1, usart1_rx_buffer, &usart1_rx_queue);
  usart1_tx_queue_handle = xQueueCreateStatic(sizeof(usart1_tx_buffer), 1, usart1_tx_buffer, &usart1_tx_queue);

  HAL_UART_Receive_IT(&huart1, usart1_rx_hw_buffer, 1);

  usart2_rx_queue_handle = xQueueCreateStatic(sizeof(usart2_rx_buffer), 1, usart2_rx_buffer, &usart2_rx_queue);
  usart2_tx_queue_handle = xQueueCreateStatic(sizeof(usart2_tx_buffer), 1, usart2_tx_buffer, &usart2_tx_queue);

  HAL_UART_Receive_IT(&huart2, usart2_rx_hw_buffer, 1);

  usb_cdc_rx_queue_handle = xQueueCreateStatic(sizeof(usb_cdc_rx_buffer), 1, usb_cdc_rx_buffer, &usb_cdc_rx_queue);

  rx_msg_queue_handle = xQueueCreateStatic(RX_MSG_QUEUE_LEN, sizeof(struct message), rx_msg_buffer, &rx_msg_queue);
  tx_msg_queue_handle = xQueueCreateStatic(TX_MSG_QUEUE_LEN, sizeof(struct message), tx_msg_buffer, &tx_msg_queue);

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of messageHandler */
  osThreadStaticDef(messageHandler, StartMessageHandler, osPriorityNormal, 0, 512, messageHandlerBuffer, &messageHandlerControlBlock);
  messageHandlerHandle = osThreadCreate(osThread(messageHandler), NULL);

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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  HAL_GPIO_WritePin(GPIOE, LED2_Pin|LED3_Pin|GPIO_PIN_6|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED2_Pin LED3_Pin PE6 LED1_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED3_Pin|GPIO_PIN_6|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_Pin PB8 */
  GPIO_InitStruct.Pin = KEY_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

  uint32_t ref;
  uint32_t vals[8];

  while (1)
  {
    update = 0;

    msg.dest    = MSG_DEST_BCAST;
    msg.src     = dev_id;
    msg.flags   = 0x00;
    msg.rx_int  = MSG_RX_NONE;
    //msg.tx_mask = 0;
    msg.tx_mask = MSG_TX_UART1 | MSG_TX_USB;

    if (next_update <= osKernelSysTick())
    {
      update = 1;
      next_update += 100;
      //msg.tx_mask |= MSG_TX_UART1;
    }

    if (update)
    {
      if (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
      {
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
      }
      else
      {
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
      }

      msg.dest    = 0x40;
      msg.ptype   = MSG_TYPE_COMMAND;
      msg.len     = 0;
      msg.tx_mask = MSG_TX_UART1 | MSG_TX_UART2;
      msg.data[msg.len++] = 0xff;
      msg.data[msg.len++] = 0x01;
      msg.data[msg.len++] = 0x40;
      msg.data[msg.len++] = 0x00;
      msg.data[msg.len++] = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == 0;
      xQueueSend(tx_msg_queue_handle, &msg, 0);

    }

    osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartMessageHandler */
/**
* @brief Function implementing the messageHandler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMessageHandler */
void StartMessageHandler(void const * argument)
{
  /* USER CODE BEGIN StartMessageHandler */
  struct message msg;
  uint8_t pkt_buffer[128];
  uint8_t pkt_buffer_2[128];
  uint8_t ch;
  size_t len;

  uint8_t activity = 0;
  uint32_t next_act_update = 0;

  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  while(1)
  {
    // USART 1 (GSE control)
    while (uxQueueMessagesWaiting(usart1_rx_queue_handle))
    {
      xQueueReceive(usart1_rx_queue_handle, &ch, 0);

      if (ch == 0)
      {
        len = cobs_decode(usart1_pkt_buffer, usart1_pkt_buffer_ptr, pkt_buffer);

        // swo_printf("Got packet [UART1]\n");

        // for (int i = 0; i < usart1_pkt_buffer_ptr; i++)
        // {
        //   swo_printf("%02x ", usart1_pkt_buffer[i]);
        // }
        // swo_printf("\n");

        if (parse_message(&msg, pkt_buffer, len) == 0)
        {
          msg.rx_int  = MSG_RX_UART1;
          msg.tx_mask = MSG_TX_NONE;
          xQueueSend(rx_msg_queue_handle, &msg, 0);
        }
        else
        {
          swo_printf("Parse failed [UART1]\n");
        }
        usart1_pkt_buffer_ptr = 0;
      }
      else
      {
        if (usart1_pkt_buffer_ptr == sizeof(usart1_pkt_buffer))
          usart1_pkt_buffer_ptr = 0;
        usart1_pkt_buffer[usart1_pkt_buffer_ptr++] = ch;
      }
    }

    // USART 2 (GSE umbilical)
    while (uxQueueMessagesWaiting(usart2_rx_queue_handle))
    {
      xQueueReceive(usart2_rx_queue_handle, &ch, 0);

      if (ch == 0)
      {
        len = cobs_decode(usart2_pkt_buffer, usart2_pkt_buffer_ptr, pkt_buffer);

        // swo_printf("Got packet [UART2]\n");

        // for (int i = 0; i < usart2_pkt_buffer_ptr; i++)
        // {
        //   swo_printf("%02x ", usart2_pkt_buffer[i]);
        // }
        // swo_printf("\n");

        if (parse_message(&msg, pkt_buffer, len) == 0)
        {
          msg.rx_int  = MSG_RX_UART2;
          msg.tx_mask = MSG_TX_NONE;
          xQueueSend(rx_msg_queue_handle, &msg, 0);
        }
        else
        {
          swo_printf("Parse failed [UART2]\n");
        }
        usart2_pkt_buffer_ptr = 0;
      }
      else
      {
        if (usart2_pkt_buffer_ptr == sizeof(usart2_pkt_buffer))
          usart2_pkt_buffer_ptr = 0;
        usart2_pkt_buffer[usart2_pkt_buffer_ptr++] = ch;
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

        if (parse_message(&msg, pkt_buffer, len) == 0)
        {
          msg.rx_int  = MSG_RX_USB;
          msg.tx_mask = MSG_TX_NONE;
          xQueueSend(rx_msg_queue_handle, &msg, 0);
        }
        else
        {
          swo_printf("Parse failed [USB CDC]\n");
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

      if (is_duplicate_message(&msg))
        continue;

      register_message(&msg);

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
      }
    }

    // Transmit messages
    while (uxQueueMessagesWaiting(tx_msg_queue_handle))
    {
      xQueueReceive(tx_msg_queue_handle, &msg, 0);

      if (msg.rx_int == MSG_RX_NONE)
      {
        // locally generated traffic
        msg.seq = tx_seq;
        tx_seq = tx_seq + 1;

        register_message(&msg);
      }

      pack_message(&msg, pkt_buffer, sizeof(pkt_buffer));

      len = cobs_encode(pkt_buffer, msg.len+MSG_OVERHEAD, pkt_buffer_2);
      pkt_buffer_2[len++] = 0;

      // UART 3 (GSE control)
      if (msg.tx_mask & MSG_TX_UART1)
      {
        activity = 1;

        for (int i = 0; i < len; i++)
        {
          ch = pkt_buffer_2[i];
          xQueueSend(usart1_tx_queue_handle, &ch, 10);
          if (!usart1_active)
          {
            usart1_active = 1;
            xQueueReceive(usart1_tx_queue_handle, usart1_tx_hw_buffer, 0);
            HAL_UART_Transmit_IT(&huart1, usart1_tx_hw_buffer, 1);
          }
        }
      }

      // UART 4 (GSE umbilical)
      if (msg.tx_mask & MSG_TX_UART2)
      {
        activity = 1;

        for (int i = 0; i < len; i++)
        {
          ch = pkt_buffer_2[i];
          xQueueSend(usart2_tx_queue_handle, &ch, 10);
          if (!usart2_active)
          {
            usart2_active = 1;
            xQueueReceive(usart2_tx_queue_handle, usart2_tx_hw_buffer, 0);
            HAL_UART_Transmit_IT(&huart2, usart2_tx_hw_buffer, 1);
          }
        }
      }

      // USB CDC
      if (msg.tx_mask & MSG_TX_USB)
      {
        activity = 1;

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

    // power/act light
    if (next_act_update <= osKernelSysTick())
    {
      if (HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin))
      {
        if (activity)
        {
          activity = 0;
          HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
          next_act_update = osKernelSysTick() + 25;
        }
      }
      else
      {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        next_act_update = osKernelSysTick() + 25;
      }
    }

    osDelay(1);
  }
  /* USER CODE END StartMessageHandler */
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
