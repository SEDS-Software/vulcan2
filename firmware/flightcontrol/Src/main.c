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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

#include "cobs.h"
#include "message.h"
#include "ms56xx.h"
#include "lsm9ds1.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

struct imu_sample
{
  uint32_t time;
  int16_t temp;
  int16_t gx;
  int16_t gy;
  int16_t gz;
  int16_t ax;
  int16_t ay;
  int16_t az;
};

struct mag_sample
{
  uint32_t time;
  int16_t x;
  int16_t y;
  int16_t z;
};

struct baro_sample
{
  uint32_t time;
  int32_t p;
  int32_t temp;
  int32_t raw_p;
  int32_t raw_temp;
};

struct ain_record
{
  uint32_t time;
  uint8_t bank;
  uint8_t type;
  uint8_t count;
  uint16_t sample[8];
};

typedef enum
{
  FM_STATE_PAD,
  FM_STATE_ASCENT_1,
  FM_STATE_ASCENT_2,
  FM_STATE_ASCENT_3,
  FM_STATE_DESCENT_1,
  FM_STATE_DESCENT_2
} fm_state_t;

typedef enum
{
  FM_ORIENTATION_PLUS_X,
  FM_ORIENTATION_MINUS_X,
  FM_ORIENTATION_PLUS_Y,
  FM_ORIENTATION_MINUS_Y,
  FM_ORIENTATION_PLUS_Z,
  FM_ORIENTATION_MINUS_Z
} fm_orientation_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define AG_I2C_ADDR   0xd4
#define MAG_I2C_ADDR  0x38
#define BARO_I2C_ADDR 0xee


#define MSG_RX_UART1 0x00
#define MSG_TX_UART1 MSG_TX_MASK(MSG_RX_UART1)

#define MSG_RX_UART2 0x01
#define MSG_TX_UART2 MSG_TX_MASK(MSG_RX_UART2)

#define MSG_RX_UART3 0x02
#define MSG_TX_UART3 MSG_TX_MASK(MSG_RX_UART3)

#define MSG_RX_UART4 0x03
#define MSG_TX_UART4 MSG_TX_MASK(MSG_RX_UART4)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc3;

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

osThreadId loggingTaskHandle;
uint32_t loggingTaskBuffer[ 512 ];
osStaticThreadDef_t loggingTaskControlBlock;
osThreadId sensorTaskHandle;
uint32_t sensorTaskBuffer[ 512 ];
osStaticThreadDef_t sensorTaskControlBlock;
osThreadId monitorTaskHandle;
uint32_t monitorTaskBuffer[ 512 ];
osStaticThreadDef_t monitorTaskControlBlock;
osThreadId messageHandlerHandle;
uint32_t messageHandlerBuffer[ 512 ];
osStaticThreadDef_t messageHandlerControlBlock;
osThreadId IOTaskHandle;
uint32_t IOTaskBuffer[ 512 ];
osStaticThreadDef_t IOTaskControlBlock;
/* USER CODE BEGIN PV */


//uint8_t usart1_rx_hw_buffer[1];
//uint8_t usart1_rx_buffer[128];
//StaticQueue_t usart1_rx_queue;
//QueueHandle_t usart1_rx_queue_handle;
//uint8_t usart1_active;
//uint8_t usart1_tx_hw_buffer[1];
//uint8_t usart1_tx_buffer[128];
//StaticQueue_t usart1_tx_queue;
//QueueHandle_t usart1_tx_queue_handle;
//
//uint8_t usart2_rx_hw_buffer[1];
//uint8_t usart2_rx_buffer[128];
//StaticQueue_t usart2_rx_queue;
//QueueHandle_t usart2_rx_queue_handle;
//uint8_t usart2_active;
//uint8_t usart2_tx_hw_buffer[1];
//uint8_t usart2_tx_buffer[128];
//StaticQueue_t usart2_tx_queue;
//QueueHandle_t usart2_tx_queue_handle;
//
//uint8_t usart3_rx_hw_buffer[1];
//uint8_t usart3_rx_buffer[128];
//StaticQueue_t usart3_rx_queue;
//QueueHandle_t usart3_rx_queue_handle;
//uint8_t usart3_active;
//uint8_t usart3_tx_hw_buffer[1];
//uint8_t usart3_tx_buffer[128];
//StaticQueue_t usart3_tx_queue;
//QueueHandle_t usart3_tx_queue_handle;

uint8_t usart4_rx_hw_buffer[1];
uint8_t usart4_rx_buffer[128];
StaticQueue_t usart4_rx_queue;
QueueHandle_t usart4_rx_queue_handle;
uint8_t usart4_active;
uint8_t usart4_tx_hw_buffer[1];
uint8_t usart4_tx_buffer[128];
StaticQueue_t usart4_tx_queue;
QueueHandle_t usart4_tx_queue_handle;

uint8_t usart6_rx_hw_buffer[1];
uint8_t usart6_rx_buffer[128];
StaticQueue_t usart6_rx_queue;
QueueHandle_t usart6_rx_queue_handle;
uint8_t usart6_active;
uint8_t usart6_tx_hw_buffer[1];
uint8_t usart6_tx_buffer[128];
StaticQueue_t usart6_tx_queue;
QueueHandle_t usart6_tx_queue_handle;

uint8_t usart4_pkt_buffer[128];
size_t usart4_pkt_buffer_ptr = 0;

// message queues
#define RX_MSG_QUEUE_LEN 8
uint8_t rx_msg_buffer[RX_MSG_QUEUE_LEN*(sizeof(struct message))];
StaticQueue_t rx_msg_queue;
QueueHandle_t rx_msg_queue_handle;

#define TX_MSG_QUEUE_LEN 8
uint8_t tx_msg_buffer[TX_MSG_QUEUE_LEN*(sizeof(struct message))];
StaticQueue_t tx_msg_queue;
QueueHandle_t tx_msg_queue_handle;

// logging
#define IMU_LOG_BUFFER_SAMPLES 128
uint8_t imu_log_buffer[IMU_LOG_BUFFER_SAMPLES*sizeof(struct imu_sample)];
StaticQueue_t imu_log_queue;
QueueHandle_t imu_log_queue_handle;

#define MAG_LOG_BUFFER_SAMPLES 128
uint8_t mag_log_buffer[MAG_LOG_BUFFER_SAMPLES*sizeof(struct mag_sample)];
StaticQueue_t mag_log_queue;
QueueHandle_t mag_log_queue_handle;

#define BARO_LOG_BUFFER_SAMPLES 128
uint8_t baro_log_buffer[BARO_LOG_BUFFER_SAMPLES*sizeof(struct baro_sample)];
StaticQueue_t baro_log_queue;
QueueHandle_t baro_log_queue_handle;

#define GPS_LOG_BUFFER_SIZE 1024
uint8_t gps_log_buffer[GPS_LOG_BUFFER_SIZE];
StaticQueue_t gps_log_queue;
QueueHandle_t gps_log_queue_handle;

#define AIN_LOG_BUFFER_RECORDS 64
uint8_t ain_log_buffer[AIN_LOG_BUFFER_RECORDS*sizeof(struct ain_record)];
StaticQueue_t ain_log_queue;
QueueHandle_t ain_log_queue_handle;

#define MON_LOG_BUFFER_SIZE 1024
uint8_t mon_log_buffer[MON_LOG_BUFFER_SIZE];
StaticQueue_t mon_log_queue;
QueueHandle_t mon_log_queue_handle;

// IMU data to flight monitor
#define IMU_MON_BUFFER_SAMPLES 16
uint8_t imu_mon_buffer[IMU_MON_BUFFER_SAMPLES*sizeof(struct imu_sample)];
StaticQueue_t imu_mon_queue;
QueueHandle_t imu_mon_queue_handle;

#define BARO_MON_BUFFER_SAMPLES 16
uint8_t baro_mon_buffer[BARO_MON_BUFFER_SAMPLES*sizeof(struct baro_sample)];
StaticQueue_t baro_mon_queue;
QueueHandle_t baro_mon_queue_handle;




FIL baro_log;
FIL imu_log;
FIL mag_log;
FIL gps_log;

FIL ain_log;
FIL mon_log;


struct ms56xx baro;
uint8_t baro_cal_valid = 0;

uint8_t log_status = 0;
uint16_t log_index = 0;

uint16_t adc1_dma_buffer[1];
uint16_t adc3_dma_buffer[4];

uint8_t dev_id = 0x09;

uint8_t tx_seq = 0x00;

// flight monitoring

fm_state_t fm_state = FM_STATE_PAD;
fm_orientation_t fm_orientation = FM_ORIENTATION_MINUS_Y;
int fm_armed = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM14_Init(void);
void StartLoggingTask(void const * argument);
void StartSensorTask(void const * argument);
void StartMonitorTask(void const * argument);
void startMessageHandler(void const * argument);
void StartIOTask(void const * argument);

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
//  if (huart == &huart1)
//  {
//    // TX complete for UART1
//    if (uxQueueMessagesWaitingFromISR(usart1_tx_queue_handle))
//    {
//      usart1_active = 1;
//      xQueueReceiveFromISR(usart1_tx_queue_handle, usart1_tx_hw_buffer, NULL);
//      HAL_UART_Transmit_IT(&huart1, usart1_tx_hw_buffer, 1);
//    }
//    else
//    {
//      usart1_active = 0;
//    }
//  }
//  else if (huart == &huart2)
//  {
//    // TX complete for UART2
//    if (uxQueueMessagesWaitingFromISR(usart2_tx_queue_handle))
//    {
//      usart2_active = 1;
//      xQueueReceiveFromISR(usart2_tx_queue_handle, usart2_tx_hw_buffer, NULL);
//      HAL_UART_Transmit_IT(&huart2, usart2_tx_hw_buffer, 1);
//    }
//    else
//    {
//      usart2_active = 0;
//    }
//  }
//  else if (huart == &huart3)
//  {
//    // TX complete for UART3
//    if (uxQueueMessagesWaitingFromISR(usart3_tx_queue_handle))
//    {
//      usart3_active = 1;
//      xQueueReceiveFromISR(usart3_tx_queue_handle, usart3_tx_hw_buffer, NULL);
//      HAL_UART_Transmit_IT(&huart3, usart3_tx_hw_buffer, 1);
//    }
//    else
//    {
//      usart3_active = 0;
//    }
//  }
//  else if (huart == &huart4)
  if (huart == &huart4)
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
//  else if (huart == &huart6)
  if (huart == &huart6)
  {
    // TX complete for UART6
    if (uxQueueMessagesWaitingFromISR(usart6_tx_queue_handle))
    {
      usart6_active = 1;
      xQueueReceiveFromISR(usart6_tx_queue_handle, usart6_tx_hw_buffer, NULL);
      HAL_UART_Transmit_IT(&huart6, usart6_tx_hw_buffer, 1);
    }
    else
    {
      usart6_active = 0;
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//  if (huart == &huart1)
//  {
//    // RX complete for USART1
//    xQueueSendFromISR(usart1_rx_queue_handle, usart1_rx_hw_buffer, NULL);
//    HAL_UART_Receive_IT(&huart1, usart1_rx_hw_buffer, 1);
//  }
//  else if (huart == &huart2)
//  {
//    // RX complete for USART3
//    xQueueSendFromISR(usart2_rx_queue_handle, usart2_rx_hw_buffer, NULL);
//    HAL_UART_Receive_IT(&huart2, usart2_rx_hw_buffer, 1);
//  }
//  else if (huart == &huart3)
//  {
//    // RX complete for USART3
//    xQueueSendFromISR(usart3_rx_queue_handle, usart3_rx_hw_buffer, NULL);
//    HAL_UART_Receive_IT(&huart3, usart3_rx_hw_buffer, 1);
//  }
//  else if (huart == &huart4)
  if (huart == &huart4)
  {
    // RX complete for UART4
    xQueueSendFromISR(usart4_rx_queue_handle, usart4_rx_hw_buffer, NULL);
    HAL_UART_Receive_IT(&huart4, usart4_rx_hw_buffer, 1);
  }
  else if (huart == &huart6)
//  if (huart == &huart6)
  {
    // RX complete for USART6
    xQueueSendFromISR(usart6_rx_queue_handle, usart6_rx_hw_buffer, NULL);
    HAL_UART_Receive_IT(&huart6, usart6_rx_hw_buffer, 1);
  }
}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
//void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);
//void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc);
//void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc);

uint32_t adc_ref_acc;
uint32_t adc3_acc[4];

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc == &hadc1)
  {
    // exponential moving average

    // ref channel init
    if (adc_ref_acc == 0)
      adc_ref_acc = adc1_dma_buffer[0] << 16;

    adc_ref_acc = adc_ref_acc * 0.999 + (adc1_dma_buffer[0] << 16) * 0.001;
  }
  else if (hadc == &hadc3)
  {
    // exponential moving average

    for (int i = 0; i < 4; i++)
    {
      adc3_acc[i] = adc3_acc[i] * 0.95 + (adc3_dma_buffer[i] << 16) * 0.05;
    }
  }
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
//void HAL_TIM_PeriodElapsedHalfCpltCallback(TIM_HandleTypeDef *htim);
//void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
//void HAL_TIM_IC_CaptureHalfCpltCallback(TIM_HandleTypeDef *htim);
//void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
//void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim);
//void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim);
//void HAL_TIM_TriggerHalfCpltCallback(TIM_HandleTypeDef *htim);
//void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim14)
  {
    // 1 ms timer tick

    // sample ADC
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_dma_buffer, 1);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adc3_dma_buffer, 4);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_SDIO_SD_Init();
  MX_UART4_Init();
  MX_I2C3_Init();
  MX_FATFS_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_I2C2_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

  //  usart1_rx_queue_handle = xQueueCreateStatic(sizeof(usart1_rx_buffer), 1, usart1_rx_buffer, &usart1_rx_queue);
  //  usart1_tx_queue_handle = xQueueCreateStatic(sizeof(usart1_tx_buffer), 1, usart1_tx_buffer, &usart1_tx_queue);
  //
  //  HAL_UART_Receive_IT(&huart1, usart1_rx_hw_buffer, 1);
  //
  //  usart2_rx_queue_handle = xQueueCreateStatic(sizeof(usart2_rx_buffer), 1, usart2_rx_buffer, &usart2_rx_queue);
  //  usart2_tx_queue_handle = xQueueCreateStatic(sizeof(usart2_tx_buffer), 1, usart2_tx_buffer, &usart2_tx_queue);
  //
  //  HAL_UART_Receive_IT(&huart2, usart2_rx_hw_buffer, 1);
  //
  //  usart3_rx_queue_handle = xQueueCreateStatic(sizeof(usart3_rx_buffer), 1, usart3_rx_buffer, &usart3_rx_queue);
  //  usart3_tx_queue_handle = xQueueCreateStatic(sizeof(usart3_tx_buffer), 1, usart3_tx_buffer, &usart3_tx_queue);
  //
  //  HAL_UART_Receive_IT(&huart3, usart3_rx_hw_buffer, 1);

  usart4_rx_queue_handle = xQueueCreateStatic(sizeof(usart4_rx_buffer), 1, usart4_rx_buffer, &usart4_rx_queue);
  usart4_tx_queue_handle = xQueueCreateStatic(sizeof(usart4_tx_buffer), 1, usart4_tx_buffer, &usart4_tx_queue);

  HAL_UART_Receive_IT(&huart4, usart4_rx_hw_buffer, 1);

  usart6_rx_queue_handle = xQueueCreateStatic(sizeof(usart6_rx_buffer), 1, usart6_rx_buffer, &usart6_rx_queue);
  usart6_tx_queue_handle = xQueueCreateStatic(sizeof(usart6_tx_buffer), 1, usart6_tx_buffer, &usart6_tx_queue);

  HAL_UART_Receive_IT(&huart6, usart6_rx_hw_buffer, 1);

  rx_msg_queue_handle = xQueueCreateStatic(RX_MSG_QUEUE_LEN, sizeof(struct message), rx_msg_buffer, &rx_msg_queue);
  tx_msg_queue_handle = xQueueCreateStatic(TX_MSG_QUEUE_LEN, sizeof(struct message), tx_msg_buffer, &tx_msg_queue);

  HAL_TIM_Base_Start_IT(&htim14);

  imu_log_queue_handle = xQueueCreateStatic(IMU_LOG_BUFFER_SAMPLES, sizeof(struct imu_sample), imu_log_buffer, &imu_log_queue);
  mag_log_queue_handle = xQueueCreateStatic(MAG_LOG_BUFFER_SAMPLES, sizeof(struct mag_sample), mag_log_buffer, &mag_log_queue);
  baro_log_queue_handle = xQueueCreateStatic(BARO_LOG_BUFFER_SAMPLES, sizeof(struct baro_sample), baro_log_buffer, &baro_log_queue);

  imu_mon_queue_handle = xQueueCreateStatic(IMU_MON_BUFFER_SAMPLES, sizeof(struct imu_sample), imu_mon_buffer, &imu_mon_queue);
  baro_mon_queue_handle = xQueueCreateStatic(BARO_MON_BUFFER_SAMPLES, sizeof(struct baro_sample), baro_mon_buffer, &baro_mon_queue);

  gps_log_queue_handle = xQueueCreateStatic(sizeof(gps_log_buffer), 1, gps_log_buffer, &gps_log_queue);

  ain_log_queue_handle = xQueueCreateStatic(AIN_LOG_BUFFER_RECORDS, sizeof(struct ain_record), ain_log_buffer, &ain_log_queue);

  mon_log_queue_handle = xQueueCreateStatic(sizeof(mon_log_buffer), 1, mon_log_buffer, &mon_log_queue);

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
  /* definition and creation of loggingTask */
  osThreadStaticDef(loggingTask, StartLoggingTask, osPriorityLow, 0, 512, loggingTaskBuffer, &loggingTaskControlBlock);
  loggingTaskHandle = osThreadCreate(osThread(loggingTask), NULL);

  /* definition and creation of sensorTask */
  osThreadStaticDef(sensorTask, StartSensorTask, osPriorityNormal, 0, 512, sensorTaskBuffer, &sensorTaskControlBlock);
  sensorTaskHandle = osThreadCreate(osThread(sensorTask), NULL);

  /* definition and creation of monitorTask */
  osThreadStaticDef(monitorTask, StartMonitorTask, osPriorityNormal, 0, 512, monitorTaskBuffer, &monitorTaskControlBlock);
  monitorTaskHandle = osThreadCreate(osThread(monitorTask), NULL);

  /* definition and creation of messageHandler */
  osThreadStaticDef(messageHandler, startMessageHandler, osPriorityNormal, 0, 512, messageHandlerBuffer, &messageHandlerControlBlock);
  messageHandlerHandle = osThreadCreate(osThread(messageHandler), NULL);

  /* definition and creation of IOTask */
  osThreadStaticDef(IOTask, StartIOTask, osPriorityNormal, 0, 512, IOTaskBuffer, &IOTaskControlBlock);
  IOTaskHandle = osThreadCreate(osThread(IOTask), NULL);

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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
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
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 4;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_ENABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 83;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
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
  huart3.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED4_Pin|LED3_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BACKUP_GO_Pin|MAIN_GO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED4_Pin LED3_Pin LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED4_Pin|LED3_Pin|LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MAIN_SENSE_Pin BACKUP_SENSE_Pin */
  GPIO_InitStruct.Pin = MAIN_SENSE_Pin|BACKUP_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BACKUP_GO_Pin MAIN_GO_Pin */
  GPIO_InitStruct.Pin = BACKUP_GO_Pin|MAIN_GO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU_DEN_Pin IMU_INT2_Pin IMU_INT1_Pin IMU_INT_M_Pin
                           IMU_DRDY_M_Pin */
  GPIO_InitStruct.Pin = IMU_DEN_Pin|IMU_INT2_Pin|IMU_INT1_Pin|IMU_INT_M_Pin
                          |IMU_DRDY_M_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartLoggingTask */
/**
  * @brief  Function implementing the loggingTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLoggingTask */
void StartLoggingTask(void const * argument)
{
  /* USER CODE BEGIN 5 */

  char buffer[128];
  uint8_t ch;

  struct imu_sample imu_s;
  struct mag_sample mag_s;
  struct baro_sample baro_s;
  struct ain_record ain_r;

  /* Infinite loop */
  while (1)
  {
    osDelay(500);

    //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); // blue LED on

    swo_printf("Attempting to mount micro SD card...\n");

    retSD = f_mount(&SDFatFS, "", 1);
    if (retSD != FR_OK)
    {
      swo_printf("Mount failed; error code: %d\n", retSD);
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // red LED on

      if (retSD == FR_DISK_ERR)
      {
        // reinit card on disk error
        BSP_SD_Init();
      }

      continue;
    }

    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); // red LED off

    log_index = 0;
    while (1)
    {
      sprintf(buffer, "%04d", log_index);
      if (f_stat(buffer, 0) == FR_NO_FILE)
      {
        if (f_mkdir(buffer) == FR_OK)
        {
          break;
        }
      }
      log_index++;
    }

    swo_printf("Using path %s\n", buffer);
    f_chdir(buffer);

    swo_printf("Opening log files...\n");

    while (!baro_cal_valid)
    {
      osDelay(1);
    }

    if (f_open(&baro_log, "baro_cal.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) != FR_OK)
    {
      swo_printf("Failed to open baro_cal.txt\n");
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // red LED on
    }

    for (int i = 0; i < 8; i++)
    {
      f_printf(&baro_log, "%d\n", baro.c[i]);
    }

    f_sync(&baro_log);
    f_close(&baro_log);

    if (f_open(&baro_log, "baro_log.csv", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) != FR_OK)
    {
      swo_printf("Failed to open baro_log.csv\n");
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // red LED on
    }

    for (int i = 0; i < 8; i++)
    {
      f_printf(&baro_log, "#c%d=%d\n", i, baro.c[i]);
    }

    f_puts("time,pressure,temp,raw pressure,raw temp\n", &baro_log);

    f_sync(&baro_log);

    if (f_open(&imu_log, "imu_log.csv", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) != FR_OK)
    {
      swo_printf("Failed to open imu_log.csv\n");
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // red LED on
    }

    f_puts("time,temp,gx,gy,gz,ax,ay,az\n", &imu_log);

    f_sync(&imu_log);

    if (f_open(&mag_log, "mag_log.csv", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) != FR_OK)
    {
      swo_printf("Failed to open mag_log.csv\n");
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // red LED on
    }

    f_puts("time,mx,my,mz\n", &mag_log);

    f_sync(&mag_log);

    if (f_open(&gps_log, "gps_log.csv", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) != FR_OK)
    {
      swo_printf("Failed to open gps_log.csv\n");
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // red LED on
    }

    f_puts("gps nmea\n", &gps_log);

    f_sync(&gps_log);

    if (f_open(&ain_log, "ain_log.csv", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) != FR_OK)
    {
      swo_printf("Failed to open ain_log.csv\n");
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // red LED on
    }

    f_puts("time,bank,type,values\n", &ain_log);

    f_sync(&ain_log);

    if (f_open(&mon_log, "mon_log.csv", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) != FR_OK)
    {
      swo_printf("Failed to open mon_log.csv\n");
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // red LED on
    }

    f_puts("flight monitor\n", &mon_log);

    f_sync(&mon_log);

    //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET); // blue LED off

    // flush buffers
    while (uxQueueMessagesWaiting(usart6_rx_queue_handle))
    {
      xQueueReceive(usart6_rx_queue_handle, &ch, 0);
    }

    log_status = 1;

    swo_printf("Log init complete\n");

    while (log_status)
    {
      // store GPS data
      if (uxQueueMessagesWaiting(gps_log_queue_handle) > 128)
      {
        while (uxQueueMessagesWaiting(gps_log_queue_handle))
        {
          xQueueReceive(gps_log_queue_handle, &ch, 0);

          if (f_putc(ch, &gps_log) < 0)
          {
            swo_printf("Fail gps log write\n");
            log_status = 0;
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // red LED on
          }
        }
        if (f_sync(&gps_log))
        {
          swo_printf("Fail gps log sync\n");
          log_status = 0;
          HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // red LED on
        }
      }

      // store baro data
      if (uxQueueMessagesWaiting(baro_log_queue_handle) > 8)
      {
        while (uxQueueMessagesWaiting(baro_log_queue_handle))
        {
          xQueueReceive(baro_log_queue_handle, &baro_s, 0);

          if (f_printf(&baro_log, "%ld,%ld,%ld,%ld,%ld\n", baro_s.time, baro_s.p, baro_s.temp, baro_s.raw_p, baro_s.raw_temp) < 0)
          {
            swo_printf("Fail baro log write\n");
            log_status = 0;
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // red LED on
          }
        }
        if (f_sync(&baro_log))
        {
          swo_printf("Fail baro log sync\n");
          log_status = 0;
          HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // red LED on
        }
      }

      // store IMU data
      if (uxQueueMessagesWaiting(imu_log_queue_handle) > 32)
      {
        while (uxQueueMessagesWaiting(imu_log_queue_handle))
        {
          xQueueReceive(imu_log_queue_handle, &imu_s, 0);

          if (f_printf(&imu_log, "%ld,%d,%d,%d,%d,%d,%d,%d\n", imu_s.time, imu_s.temp, imu_s.gx, imu_s.gy, imu_s.gz, imu_s.ax, imu_s.ay, imu_s.az) < 0)
          {
            swo_printf("Fail imu log write\n");
            log_status = 0;
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // red LED on
          }
        }
        if (f_sync(&imu_log))
        {
          swo_printf("Fail imu log sync\n");
          log_status = 0;
          HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // red LED on
        }
      }

      // store magnetometer data
      if (uxQueueMessagesWaiting(mag_log_queue_handle) > 8)
      {
        while (uxQueueMessagesWaiting(mag_log_queue_handle))
        {
          xQueueReceive(mag_log_queue_handle, &mag_s, 0);

          if (f_printf(&mag_log, "%ld,%d,%d,%d\n", mag_s.time, mag_s.x, mag_s.y, mag_s.z) < 0)
          {
            swo_printf("Fail mag log write\n");
            log_status = 0;
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // red LED on
          }
        }
        if (f_sync(&mag_log))
        {
          swo_printf("Fail mag log sync\n");
          log_status = 0;
          HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // red LED on
        }
      }

      // store analog channel data
      if (uxQueueMessagesWaiting(ain_log_queue_handle) > 8)
      {
        while (uxQueueMessagesWaiting(ain_log_queue_handle))
        {
          xQueueReceive(ain_log_queue_handle, &ain_r, 0);

          f_printf(&ain_log, "%ld,%d,%d", ain_r.time, ain_r.bank, ain_r.type);
          for (int i = 0; i < ain_r.count; i++)
          {
            f_printf(&ain_log, ",%d", ain_r.sample[i]);
          }
          f_printf(&ain_log, "\n");
        }
        if (f_sync(&ain_log))
        {
          swo_printf("Fail AIN log sync\n");
          log_status = 0;
          HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // red LED on
        }
      }

      // store flight monitor data
      if (uxQueueMessagesWaiting(mon_log_queue_handle) > 128)
      {
        while (uxQueueMessagesWaiting(mon_log_queue_handle))
        {
          xQueueReceive(mon_log_queue_handle, &ch, 0);

          if (f_putc(ch, &mon_log) < 0)
          {
            swo_printf("Fail mon log write\n");
            log_status = 0;
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // red LED on
          }
        }
        if (f_sync(&mon_log))
        {
          swo_printf("Fail mon log sync\n");
          log_status = 0;
          HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // red LED on
        }
      }

      osDelay(10);
    }

    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); // red LED on
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the sensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartSensorTask */
  uint8_t data[32];

  char buffer[32];
  uint8_t ch;
  uint8_t gps_insert_ts = 1;

  I2C_HandleTypeDef *imu_i2c = &hi2c3;

  struct imu_sample imu_s;
  struct mag_sample mag_s;
  struct baro_sample baro_s;

  uint8_t baro_state = 0;
  uint32_t baro_next_sample = 0;
  uint32_t baro_next_op = 0;

  osDelay(200);

  swo_printf("Scan I2C bus (I2C3, main IMU)\n");

  for (int i = 0; i < 256; i+=2)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c3, i, 1, 100) == HAL_OK)
    {
      swo_printf("Found I2C device at 0x%02x\n", i);
    }
  }

  swo_printf("Scan I2C bus (I2C2, emulated IMU)\n");

  for (int i = 0; i < 256; i+=2)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c2, i, 1, 100) == HAL_OK)
    {
      swo_printf("Found I2C device at 0x%02x\n", i);

      if (i == BARO_I2C_ADDR)
      {
        // detected emulated IMU components; use emulated IMU
        imu_i2c = &hi2c2;
      }
    }
  }

  swo_printf("Init MS56XX...\n");

  // init barometric pressure sensor
  data[0] = MS56XX_RESET;
  HAL_I2C_Master_Transmit(imu_i2c, BARO_I2C_ADDR, data, 1, 100);
  data[0] = MS56XX_RESET;
  HAL_I2C_Master_Transmit(imu_i2c, BARO_I2C_ADDR, data, 1, 100);

  osDelay(20);

  for (int k = 0; k < 8; k++)
  {
    HAL_I2C_Mem_Read(imu_i2c, BARO_I2C_ADDR, MS56XX_PROM_READ+(k<<1), 1, data, 2, 100);
    baro.c[k] = data[0] << 8 | data[1];
    swo_printf("%d\n", baro.c[k]);
  }

  baro_cal_valid = 1;

  if (ms56xx_check_prom_crc(&baro))
  {
    swo_printf("Good CRC\n");
  }
  else
  {
    swo_printf("CRC check failed\n");
  }

  swo_printf("Init IMU...\n");

  // gryo and accelerometer
  data[0] = 0; // set gyro HPF reference to 0
  HAL_I2C_Mem_Write(imu_i2c, AG_I2C_ADDR, LSM9DS1_AG_REFERENCE_G, 1, data, 1, 100);
  data[0] = 0x78; // ODR_G = 3'b011 (119 sps), FS_G = 2'b11 (2000 dps), BW_G = 2'b00 (14 Hz)
  HAL_I2C_Mem_Write(imu_i2c, AG_I2C_ADDR, LSM9DS1_AG_CTRL_REG1_G, 1, data, 1, 100);
  data[0] = 0x00; // INT_SEL = 2'b00 (LPF1), OUT_SEL = 2'b00 (LPF1)
  HAL_I2C_Mem_Write(imu_i2c, AG_I2C_ADDR, LSM9DS1_AG_CTRL_REG2_G, 1, data, 1, 100);
  data[0] = 0x00; // LP_mode = 1'b0, HP_EN = 1'b0, HPCF_G = 4'b0000
  HAL_I2C_Mem_Write(imu_i2c, AG_I2C_ADDR, LSM9DS1_AG_CTRL_REG3_G, 1, data, 1, 100);
  data[0] = 0x00; // SignX_G = 1'b0, SignY_G = 1'b0, SignZ_G = 1'b0, Orient = 3'b000
  HAL_I2C_Mem_Write(imu_i2c, AG_I2C_ADDR, LSM9DS1_AG_ORIENT_CFG_G, 1, data, 1, 100);
  data[0] = 0x38; // Zen_G = 1'b1, Yen_G = 1'b1, Xen_G = 1'b1, LIR_XL1 = 1'b0, 4D_XL1 = 1'b0
  HAL_I2C_Mem_Write(imu_i2c, AG_I2C_ADDR, LSM9DS1_AG_CTRL_REG4, 1, data, 1, 100);
  data[0] = 0x38; // DEC = 2'b00, Zen_XL = 1'b1, Yen_XL = 1'b1, Zen_XL = 1'b1
  HAL_I2C_Mem_Write(imu_i2c, AG_I2C_ADDR, LSM9DS1_AG_CTRL_REG5_XL, 1, data, 1, 100);
  data[0] = 0x78; // ODR_XL = 3'b011 (119 sps), FS_XL = 2'b11 (8g), BW_SCAL_ODR = 1'b0, BW_XL = 2'b00 (408 Hz)
  HAL_I2C_Mem_Write(imu_i2c, AG_I2C_ADDR, LSM9DS1_AG_CTRL_REG6_XL, 1, data, 1, 100);
  data[0] = 0x00; // HR = 1'b0 (high res off), DCF = 2'b00 (ODR/50), FDS = 1'b0 (filter bypassed), HPIS1 = 1'b0 (filter bypassed)
  HAL_I2C_Mem_Write(imu_i2c, AG_I2C_ADDR, LSM9DS1_AG_CTRL_REG7_XL, 1, data, 1, 100);
  data[0] = 0x04; // BOOT = 1'b0, BDU = 1'b0, H_LACTIVE = 1'b0, PP_OD = 1'b0, SIM = 1'b0, IF_ADD_INC = 1'b1, BLE = 1'b0, SW_RESET = 1'b0
  HAL_I2C_Mem_Write(imu_i2c, AG_I2C_ADDR, LSM9DS1_AG_CTRL_REG8, 1, data, 1, 100);
  data[0] = 0x02; // SLEEP_G = 1'b0, FIFO_TEMP_EN = 1'b0, DRDY_mask_bit = 1'b0, I2C_DISABLE = 1'b0, FIFO_EN = 1'b1, STOP_ON_FTH = 1'b0
  HAL_I2C_Mem_Write(imu_i2c, AG_I2C_ADDR, LSM9DS1_AG_CTRL_REG9, 1, data, 1, 100);
  data[0] = 0x00; // ST_G = 1'b0, ST_XL = 1'b0
  HAL_I2C_Mem_Write(imu_i2c, AG_I2C_ADDR, LSM9DS1_AG_CTRL_REG10, 1, data, 1, 100);
  data[0] = 0xD0; // FMODE = 3'b110, FTH = 5'b10000
  HAL_I2C_Mem_Write(imu_i2c, AG_I2C_ADDR, LSM9DS1_AG_FIFO_CTRL, 1, data, 1, 100);

  // magnetometer
  data[0] = 0x00; // OFXM, OFYM, OFZM
  data[1] = 0x00;
  HAL_I2C_Mem_Write(imu_i2c, MAG_I2C_ADDR, LSM9DS1_MAG_OFFSET_X_REG_L_M, 1, data, 2, 100);
  HAL_I2C_Mem_Write(imu_i2c, MAG_I2C_ADDR, LSM9DS1_MAG_OFFSET_Y_REG_L_M, 1, data, 2, 100);
  HAL_I2C_Mem_Write(imu_i2c, MAG_I2C_ADDR, LSM9DS1_MAG_OFFSET_Z_REG_L_M, 1, data, 2, 100);
  data[0] = 0xf0; // TEMP_COMP = 1'b1, OM = 2'b11, DO = 3'b100 (10 sps), FAST_ODR = 1'b0, ST = 1'b0
  HAL_I2C_Mem_Write(imu_i2c, MAG_I2C_ADDR, LSM9DS1_MAG_CTRL_REG1_M, 1, data, 1, 100);
  data[0] = 0x00; // FS = 2'b00, REBOOT = 1'b0, SOFT_RST = 1'b0
  HAL_I2C_Mem_Write(imu_i2c, MAG_I2C_ADDR, LSM9DS1_MAG_CTRL_REG2_M, 1, data, 1, 100);
  data[0] = 0x00; // I2C_DISABLE = 1'b0, LP = 1'b0, SIM = 1'b0, MD = 2'b00
  HAL_I2C_Mem_Write(imu_i2c, MAG_I2C_ADDR, LSM9DS1_MAG_CTRL_REG3_M, 1, data, 1, 100);
  data[0] = 0x0C; // OMZ = 2'b11, BLE = 1'b0
  HAL_I2C_Mem_Write(imu_i2c, MAG_I2C_ADDR, LSM9DS1_MAG_CTRL_REG4_M, 1, data, 1, 100);
  data[0] = 0x00; // FAST_READ = 1'b0, BDU = 1'b0
  HAL_I2C_Mem_Write(imu_i2c, MAG_I2C_ADDR, LSM9DS1_MAG_CTRL_REG5_M, 1, data, 1, 100);

  // GPS
  while (uxQueueMessagesWaiting(usart6_rx_queue_handle))
  {
    xQueueReceive(usart6_rx_queue_handle, &ch, 0);
  }

  /* Infinite loop */
  while (1)
  {
    // check for IMU data
    // gyro and accelerometer
    HAL_I2C_Mem_Read(imu_i2c, AG_I2C_ADDR, LSM9DS1_AG_FIFO_SRC, 1, data, 1, 100); // FIFO status
    if (data[0] & 0x3f)
    {
      // data available in FIFO
      // read complete set of data
      imu_s.time = osKernelSysTick();
      HAL_I2C_Mem_Read(imu_i2c, AG_I2C_ADDR, LSM9DS1_AG_OUT_TEMP_L, 1, (uint8_t *)&imu_s.temp, 2, 100);
      imu_s.temp = ((imu_s.temp * 100) >> 4) + 2500; // rescale temp to deg C
      HAL_I2C_Mem_Read(imu_i2c, AG_I2C_ADDR, LSM9DS1_AG_OUT_X_L_G, 1, (uint8_t *)&imu_s.gx, 2*(3+3), 100);

      xQueueSend(imu_log_queue_handle, &imu_s, 0);
      xQueueSend(imu_mon_queue_handle, &imu_s, 0);
    }

    // magnetometer
    HAL_I2C_Mem_Read(imu_i2c, MAG_I2C_ADDR, LSM9DS1_MAG_STATUS_REG_M, 1, data, 1, 100); // status
    if (data[0] & 0x08)
    {
      // data available
      // read complete set of data
      mag_s.time = osKernelSysTick();
      HAL_I2C_Mem_Read(imu_i2c, MAG_I2C_ADDR, LSM9DS1_MAG_OUT_X_L_M, 1, (uint8_t *)&mag_s.x, 2*3, 100);

      xQueueSend(mag_log_queue_handle, &mag_s, 0);
    }

    // barometer
    if (baro_state == 0)
    {
      // sample pressure
      if (baro_next_sample <= osKernelSysTick())
      {
        baro_s.time = osKernelSysTick();
        baro_next_sample += 50;

        data[0] = MS56XX_CONV_D1_4096;
        HAL_I2C_Master_Transmit(imu_i2c, BARO_I2C_ADDR, data, 1, 100);

        baro_next_op = osKernelSysTick() + 10;
        baro_state = 1;
      }
    }
    else if (baro_state == 1)
    {
      // read pressure, sample temperature
      if (baro_next_op <= osKernelSysTick())
      {
        baro_s.raw_p = 0;
        HAL_I2C_Mem_Read(imu_i2c, BARO_I2C_ADDR, MS56XX_ADC_READ, 1, ((uint8_t *)&baro_s.raw_p)+1, 3, 100);
        data[0] = MS56XX_CONV_D2_4096;
        HAL_I2C_Master_Transmit(imu_i2c, BARO_I2C_ADDR, data, 1, 100);

        baro_next_op = osKernelSysTick() + 10;
        baro_state = 2;
      }
    }
    else if (baro_state == 2)
    {
      // read temperature
      if (baro_next_op <= osKernelSysTick())
      {
        baro_s.raw_temp = 0;
        HAL_I2C_Mem_Read(imu_i2c, BARO_I2C_ADDR, MS56XX_ADC_READ, 1, ((uint8_t *)&baro_s.raw_temp)+1, 3, 100);

        baro_s.raw_p = __REV(baro_s.raw_p);
        baro_s.raw_temp = __REV(baro_s.raw_temp);
        baro_s.p = baro_s.raw_p;
        baro_s.temp = baro_s.raw_temp;

        ms56xx_convert_2(&baro, &baro_s.p, &baro_s.temp);

        xQueueSend(baro_log_queue_handle, &baro_s, 0);
        xQueueSend(baro_mon_queue_handle, &baro_s, 0);

        baro_state = 0;
      }
    }

    // GPS
    while (uxQueueMessagesWaiting(usart6_rx_queue_handle))
    {
      if (gps_insert_ts)
      {
        sprintf(buffer, "%ld ", osKernelSysTick());
        for (char *ptr = buffer; *ptr; ptr++)
        {
          xQueueSend(gps_log_queue_handle, ptr, 0);
        }
        gps_insert_ts = 0;
      }
      xQueueReceive(usart6_rx_queue_handle, data, 0);
      xQueueSend(gps_log_queue_handle, data, 0);
      if (data[0] == '\n')
        gps_insert_ts = 1;
    }

    osDelay(1);
  }
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartMonitorTask */
/**
* @brief Function implementing the monitorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMonitorTask */
void StartMonitorTask(void const * argument)
{
  /* USER CODE BEGIN StartMonitorTask */

  struct imu_sample imu_s;
  struct baro_sample baro_s;

  uint32_t last_imu_s_time = 0;
  uint32_t last_baro_s_time = 0;

  int raw_imu_accel = 0;
  float g_scale = 4000;
  float imu_accel = 0;
  float imu_vel = 0;
  float imu_alt = 0;

  float baro_raw_alt = 0;
  float last_baro_raw_alt = 0;
  float baro_pad_alt = 0;
  float baro_alt = 0;
  float baro_vel = 0;

  float apogee_alt = 0;
  int32_t apogee_p = 0;

  int ts_ms = 10;
  long launch_t = 0;

  long update_t = 0;

  struct message msg;
  long xmit_t = 0;

  char buffer[64];

  uint32_t actuation_clear_time = 0;

  /* Infinite loop */
  while (1)
  {
    // process new IMU data
    while (uxQueueMessagesWaiting(imu_mon_queue_handle))
    {
      xQueueReceive(imu_mon_queue_handle, &imu_s, 0);

      if (fm_state == FM_STATE_PAD)
      {
        if (imu_s.ax > 2000)
        {
          fm_orientation = FM_ORIENTATION_PLUS_X;
        }
        else if (imu_s.ax < -2000)
        {
          fm_orientation = FM_ORIENTATION_MINUS_X;
        }
        else if (imu_s.ay > 2000)
        {
          fm_orientation = FM_ORIENTATION_PLUS_Y;
        }
        else if (imu_s.ay < -2000)
        {
          fm_orientation = FM_ORIENTATION_MINUS_Y;
        }
        else if (imu_s.az > 2000)
        {
          fm_orientation = FM_ORIENTATION_PLUS_Z;
        }
        else if (imu_s.az < -2000)
        {
          fm_orientation = FM_ORIENTATION_MINUS_Z;
        }
      }

      switch (fm_orientation)
      {
        case FM_ORIENTATION_PLUS_X:
          raw_imu_accel = imu_s.ax;
          break;
        case FM_ORIENTATION_MINUS_X:
          raw_imu_accel = -imu_s.ax;
          break;
        case FM_ORIENTATION_PLUS_Y:
          raw_imu_accel = imu_s.ay;
          break;
        case FM_ORIENTATION_MINUS_Y:
          raw_imu_accel = -imu_s.ay;
          break;
        case FM_ORIENTATION_PLUS_Z:
          raw_imu_accel = imu_s.az;
          break;
        case FM_ORIENTATION_MINUS_Z:
          raw_imu_accel = -imu_s.az;
          break;
      }

      imu_accel = ((raw_imu_accel/g_scale) - 1) * 9.81;

      switch (fm_state)
      {
        case FM_STATE_PAD:
          if (raw_imu_accel > 3600 && raw_imu_accel < 4400)
          {
            g_scale = g_scale * 0.999 + raw_imu_accel * 0.001;
          }
          break;
        default:
          imu_vel = imu_vel + imu_accel * ((imu_s.time - last_imu_s_time)*0.001);
          imu_alt = imu_alt + imu_vel * ((imu_s.time - last_imu_s_time)*0.001);
          break;
      }

      last_imu_s_time = imu_s.time;
    }

    // process new barometer data
    while (uxQueueMessagesWaiting(baro_mon_queue_handle))
    {
      xQueueReceive(baro_mon_queue_handle, &baro_s, 0);

      // p = 101325 (1 - 2.25577e-5 * h) ** 5.25588
      // h = (1 - 10 ** (log10(p / 101325) / 5.25588)) / 2.25577e-5
      baro_raw_alt = (1.0 - pow(10, log10(baro_s.p / 101325.0) / 5.25588)) / 2.25577e-5;
      baro_vel = baro_vel * 0.95 + ((baro_raw_alt - last_baro_raw_alt) / ((baro_s.time - last_baro_s_time)*0.001)) * 0.05;
      baro_alt = baro_raw_alt - baro_pad_alt;

      switch (fm_state)
      {
        case FM_STATE_PAD:
          baro_pad_alt = baro_pad_alt * 0.999 + baro_raw_alt * 0.001;

          if (abs(baro_pad_alt - baro_raw_alt) > 25)
          {
            baro_pad_alt = baro_raw_alt;
          }
          break;
        case FM_STATE_ASCENT_3:
        case FM_STATE_DESCENT_1:
          // capture max altitude

          if (baro_alt > apogee_alt)
          {
            apogee_p = baro_s.p;
            apogee_alt = baro_alt;
          }

          break;
        default:

          break;
      }

      last_baro_raw_alt = baro_raw_alt;
      last_baro_s_time = baro_s.time;
    }

    // monitor flight status
    switch (fm_state)
    {
      case FM_STATE_PAD:
        // on launch pad
        imu_vel = 0;
        imu_alt = 0;

        // detect launch
        if (fm_armed && imu_accel > 10 && baro_vel > 2)
        {
          launch_t = osKernelSysTick();
          fm_state = FM_STATE_ASCENT_1;

          sprintf(buffer, "%ld launch detected\n", osKernelSysTick());
          swo_printf("%s", buffer);
          for (char *ptr = buffer; *ptr; ptr++)
            xQueueSend(mon_log_queue_handle, ptr, 0);
          sprintf(buffer, "%ld ascent 1\n", osKernelSysTick());
          swo_printf("%s", buffer);
          for (char *ptr = buffer; *ptr; ptr++)
            xQueueSend(mon_log_queue_handle, ptr, 0);
        }
        break;
      case FM_STATE_ASCENT_1:
        // first ascent state, fixed delay
        if (osKernelSysTick() - launch_t > 2*1000) // TODO parameter
        {
          if (baro_alt > 20) // min alt to confirm launch
          {
            fm_state = FM_STATE_ASCENT_2;

            sprintf(buffer, "%ld ascent 2\n", osKernelSysTick());
            swo_printf("%s", buffer);
            for (char *ptr = buffer; *ptr; ptr++)
              xQueueSend(mon_log_queue_handle, ptr, 0);
          }
          else
          {
            fm_state = FM_STATE_PAD;

            sprintf(buffer, "%ld pad\n", osKernelSysTick());
            swo_printf("%s", buffer);
            for (char *ptr = buffer; *ptr; ptr++)
              xQueueSend(mon_log_queue_handle, ptr, 0);
          }
        }
        break;
      case FM_STATE_ASCENT_2:
        // second ascent state, wait for IMU velocity threshold
        if (imu_vel < 50) // TODO parameter
        {
          fm_state = FM_STATE_ASCENT_3;

          sprintf(buffer, "%ld ascent 3\n", osKernelSysTick());
          swo_printf("%s", buffer);
          for (char *ptr = buffer; *ptr; ptr++)
            xQueueSend(mon_log_queue_handle, ptr, 0);
        }
        break;
      case FM_STATE_ASCENT_3:
        // third ascent state, wait for barometric pressure sensor velocity threshold
        if (baro_vel < 1) // TODO parameter
        {
          // apogee actions here

          if (fm_armed && baro_alt > 100) // TODO parameter
          {
            // deploy drogue chute

            HAL_GPIO_WritePin(BACKUP_GO_GPIO_Port, BACKUP_GO_Pin, GPIO_PIN_SET);
            actuation_clear_time = osKernelSysTick() + 10000;

            sprintf(buffer, "%ld fire drogue\n", osKernelSysTick());
            swo_printf("%s", buffer);
            for (char *ptr = buffer; *ptr; ptr++)
              xQueueSend(mon_log_queue_handle, ptr, 0);
          }

          fm_state = FM_STATE_DESCENT_1;

          sprintf(buffer, "%ld descent 1\n", osKernelSysTick());
          swo_printf("%s", buffer);
          for (char *ptr = buffer; *ptr; ptr++)
            xQueueSend(mon_log_queue_handle, ptr, 0);
        }
        break;
      case FM_STATE_DESCENT_1:
        // first descent state, wait for barometric pressure sensor to show altitude less than threshold
        if (baro_alt < 308) // TODO parameter
        {
          // thousand foot actions here

          if (fm_armed && baro_alt > 100) // TODO parameter
          {
            // deploy main chute

            HAL_GPIO_WritePin(MAIN_GO_GPIO_Port, MAIN_GO_Pin, GPIO_PIN_SET);
            actuation_clear_time = osKernelSysTick() + 10000;

            sprintf(buffer, "%ld fire main\n", osKernelSysTick());
            swo_printf("%s", buffer);
            for (char *ptr = buffer; *ptr; ptr++)
              xQueueSend(mon_log_queue_handle, ptr, 0);
          }

          fm_state = FM_STATE_DESCENT_2;

          sprintf(buffer, "%ld descent 2\n", osKernelSysTick());
          swo_printf("%s", buffer);
          for (char *ptr = buffer; *ptr; ptr++)
            xQueueSend(mon_log_queue_handle, ptr, 0);
        }
        break;
      case FM_STATE_DESCENT_2:
        // second descent state, wait for recovery

        if (!fm_armed)
        {
          fm_state = FM_STATE_PAD;

          sprintf(buffer, "%ld pad\n", osKernelSysTick());
          swo_printf("%s", buffer);
          for (char *ptr = buffer; *ptr; ptr++)
            xQueueSend(mon_log_queue_handle, ptr, 0);
        }
        break;
    }

    if (actuation_clear_time <= osKernelSysTick())
    {
      HAL_GPIO_WritePin(BACKUP_GO_GPIO_Port, BACKUP_GO_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MAIN_GO_GPIO_Port, MAIN_GO_Pin, GPIO_PIN_RESET);
    }

    if (update_t <= osKernelSysTick())
    {
      update_t += 250;

      sprintf(buffer, "%ld flight monitor status\n", osKernelSysTick());
      //swo_printf("%s", buffer);
      for (char *ptr = buffer; *ptr; ptr++)
        xQueueSend(mon_log_queue_handle, ptr, 0);
      sprintf(buffer, "state %d\n", fm_state);
      //swo_printf("%s", buffer);
      for (char *ptr = buffer; *ptr; ptr++)
        xQueueSend(mon_log_queue_handle, ptr, 0);
      sprintf(buffer, "imu raw %d scale %d\n", raw_imu_accel, (int)g_scale);
      //swo_printf("%s", buffer);
      for (char *ptr = buffer; *ptr; ptr++)
        xQueueSend(mon_log_queue_handle, ptr, 0);
      sprintf(buffer, "imu accel %d vel %d alt %d\n", (int)imu_accel, (int)imu_vel, (int)imu_alt);
      //swo_printf("%s", buffer);
      for (char *ptr = buffer; *ptr; ptr++)
        xQueueSend(mon_log_queue_handle, ptr, 0);
      sprintf(buffer, "baro raw %ld raw alt %d pad alt %d\n", baro_s.p, (int)baro_raw_alt, (int)baro_pad_alt);
      //swo_printf("%s", buffer);
      for (char *ptr = buffer; *ptr; ptr++)
        xQueueSend(mon_log_queue_handle, ptr, 0);
      sprintf(buffer, "baro alt %d vel %d\n", (int)baro_alt, (int)baro_vel);
      //swo_printf("%s", buffer);
      for (char *ptr = buffer; *ptr; ptr++)
        xQueueSend(mon_log_queue_handle, ptr, 0);
    }

    if (xmit_t <= osKernelSysTick())
    {
      long l;

      xmit_t += 500;

      msg.dest    = MSG_DEST_BCAST;
      msg.src     = dev_id;
      msg.flags   = MSG_FLAG_RADIO;
      msg.rx_int  = MSG_RX_NONE;
      msg.tx_mask = MSG_TX_UART4 | MSG_TX_UART2;

      msg.ptype   = 0xf0;
      msg.len     = 0;
      l = osKernelSysTick();
      msg.data[msg.len++] = l & 0xff;
      msg.data[msg.len++] = (l >> 8) & 0xff;
      msg.data[msg.len++] = (l >> 16) & 0xff;
      msg.data[msg.len++] = (l >> 24) & 0xff;
      msg.data[msg.len++] = fm_state;
      l = 0;
      if (fm_armed) l |= (1 << 0);
      if (log_status) l |= (1 << 1);
      if (HAL_GPIO_ReadPin(BACKUP_SENSE_GPIO_Port, BACKUP_SENSE_Pin)) l |= (1 << 4);
      if (HAL_GPIO_ReadPin(MAIN_SENSE_GPIO_Port, MAIN_SENSE_Pin)) l |= (1 << 5);
      msg.data[msg.len++] = l;
      msg.data[msg.len++] = baro_s.p & 0xff;
      msg.data[msg.len++] = (baro_s.p >> 8) & 0xff;
      msg.data[msg.len++] = (baro_s.p >> 16) & 0xff;
      msg.data[msg.len++] = (baro_s.p >> 24) & 0xff;
      msg.data[msg.len++] = baro_s.temp & 0xff;
      msg.data[msg.len++] = (baro_s.temp >> 8) & 0xff;
      msg.data[msg.len++] = (baro_s.temp >> 16) & 0xff;
      msg.data[msg.len++] = (baro_s.temp >> 24) & 0xff;
      l = imu_accel * 100;
      msg.data[msg.len++] = l & 0xff;
      msg.data[msg.len++] = (l >> 8) & 0xff;
      msg.data[msg.len++] = (l >> 16) & 0xff;
      msg.data[msg.len++] = (l >> 24) & 0xff;
      l = baro_raw_alt * 100;
      msg.data[msg.len++] = l & 0xff;
      msg.data[msg.len++] = (l >> 8) & 0xff;
      msg.data[msg.len++] = (l >> 16) & 0xff;
      msg.data[msg.len++] = (l >> 24) & 0xff;
      l = baro_alt * 100;
      msg.data[msg.len++] = l & 0xff;
      msg.data[msg.len++] = (l >> 8) & 0xff;
      msg.data[msg.len++] = (l >> 16) & 0xff;
      msg.data[msg.len++] = (l >> 24) & 0xff;
      l = baro_pad_alt * 100;
      msg.data[msg.len++] = l & 0xff;
      msg.data[msg.len++] = (l >> 8) & 0xff;
      msg.data[msg.len++] = (l >> 16) & 0xff;
      msg.data[msg.len++] = (l >> 24) & 0xff;
      l = baro_vel * 100;
      msg.data[msg.len++] = l & 0xff;
      msg.data[msg.len++] = (l >> 8) & 0xff;
      msg.data[msg.len++] = (l >> 16) & 0xff;
      msg.data[msg.len++] = (l >> 24) & 0xff;
      l = imu_vel * 100;
      msg.data[msg.len++] = l & 0xff;
      msg.data[msg.len++] = (l >> 8) & 0xff;
      msg.data[msg.len++] = (l >> 16) & 0xff;
      msg.data[msg.len++] = (l >> 24) & 0xff;
      l = imu_alt * 100;
      msg.data[msg.len++] = l & 0xff;
      msg.data[msg.len++] = (l >> 8) & 0xff;
      msg.data[msg.len++] = (l >> 16) & 0xff;
      msg.data[msg.len++] = (l >> 24) & 0xff;
      xQueueSend(tx_msg_queue_handle, &msg, 0);
    }

    osDelay(ts_ms);
  }
  /* USER CODE END StartMonitorTask */
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

  while(1)
  {
    // receive new messages

    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

    // UART 4 (Telemetry board)
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

        if (parse_message(&msg, pkt_buffer, len) == 0)
        {
          msg.rx_int  = MSG_RX_UART4;
          msg.tx_mask = MSG_TX_NONE;
          xQueueSend(rx_msg_queue_handle, &msg, 0);
        }
        else
        {
          swo_printf("Parse failed [UART4]\n");
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

    // Process received messages
    while (uxQueueMessagesWaiting(rx_msg_queue_handle))
    {
      xQueueReceive(rx_msg_queue_handle, &msg, 0);

      if (is_duplicate_message(&msg))
        continue;

      register_message(&msg);

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

          switch (cmd)
          {
            case 0x000801F0:
              // set flight monitor state
              fm_state = msg.data[4];
              break;
            case 0x000801FF:
              // set flight monitor arm state command
              fm_armed = msg.data[4];
              break;
            case 0x000100F0:
              // restart logging
              if (msg.data[4])
                log_status = 0;
              break;
            case 0xff0008f0:
              // set address
              dev_id = msg.data[4];
              break;
          }
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

      // UART 4 (Telemetry board)
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
    }

    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

    osDelay(1);
  }
  /* USER CODE END startMessageHandler */
}

/* USER CODE BEGIN Header_StartIOTask */
/**
* @brief Function implementing the IOTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIOTask */
void StartIOTask(void const * argument)
{
  /* USER CODE BEGIN StartIOTask */
  struct message msg;

  struct ain_record ain_r;

  uint32_t next_gse_update = 0;
  uint32_t next_radio_update = 0;
  uint8_t update = 0;

  uint32_t ref;
  uint32_t vals[4];

  while (1)
  {

    update = 0;

    msg.dest    = MSG_DEST_BCAST;
    msg.src     = dev_id;
    msg.flags   = 0x00;
    msg.rx_int  = MSG_RX_NONE;
    msg.tx_mask = 0;

    if (next_gse_update <= osKernelSysTick())
    {
      update = 1;
      next_gse_update += 100;
      msg.tx_mask |= MSG_TX_UART4;
    }

    if (next_radio_update <= osKernelSysTick())
    {
      update = 1;
      next_radio_update += 500;
      msg.flags |= MSG_FLAG_RADIO;
      msg.tx_mask |= MSG_TX_UART2;
    }

    if (update)
    {
      // Read board voltages with ADC
      ref = adc_ref_acc;
      for (int i = 0; i < 4; i++)
      {
        // rescale to 10,000 counts per volt
        // assuming ref is 1.21 volts
        vals[i] = (((uint64_t)adc3_acc[i])*12100)/ref;
        ain_r.sample[i] = vals[i];
      }

      msg.ptype   = MSG_TYPE_ANALOG_VALUE;
      msg.len     = 0;
      msg.data[msg.len++] = 0; // bank
      msg.data[msg.len++] = 0; // type
      for (int i = 0; i < 4; i++)
      {
        msg.data[msg.len++] = vals[i] & 0xff;
        msg.data[msg.len++] = (vals[i] >> 8) & 0xff;
      }
      xQueueSend(tx_msg_queue_handle, &msg, 0);

      ain_r.time = osKernelSysTick();
      ain_r.bank = 0;
      ain_r.type = 0;
      ain_r.count = 4;
      xQueueSend(ain_log_queue_handle, &ain_r, 0);
    }

    osDelay(10);
  }
  /* USER CODE END StartIOTask */
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
