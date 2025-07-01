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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "liquidcrystal_i2c.h"
#include <string.h>
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

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* Definitions for readTempTask */
osThreadId_t readTempTaskHandle;
const osThreadAttr_t readTempTask_attributes = {
  .name = "readTempTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for updateLcdTask */
osThreadId_t updateLcdTaskHandle;
const osThreadAttr_t updateLcdTask_attributes = {
  .name = "updateLcdTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for sendTempTask */
osThreadId_t sendTempTaskHandle;
const osThreadAttr_t sendTempTask_attributes = {
  .name = "sendTempTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for processCmdTask */
osThreadId_t processCmdTaskHandle;
const osThreadAttr_t processCmdTask_attributes = {
  .name = "processCmdTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for temperatureQueue */
osMessageQueueId_t temperatureQueueHandle;
const osMessageQueueAttr_t temperatureQueue_attributes = {
  .name = "temperatureQueue"
};
/* Definitions for commandQueue */
osMessageQueueId_t commandQueueHandle;
const osMessageQueueAttr_t commandQueue_attributes = {
  .name = "commandQueue"
};
/* Definitions for uartMutex */
osMutexId_t uartMutexHandle;
const osMutexAttr_t uartMutex_attributes = {
  .name = "uartMutex"
};
/* USER CODE BEGIN PV */
uint8_t rxByte;
float g_currentTemperature = 0.0f; // Bien toan cuc nhiet do
uint32_t g_currentFrequency = 1000;
osMutexId_t tempMutexHandle;         // Mutex de bao ve bien nhiet do
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
void StartReadTempTask(void *argument);
void StartUpdateLcdTask(void *argument);
void StartSendTempTask(void *argument);
void StartProcessCmdTask(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HD44780_Init(2);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  HAL_ADC_Start(&hadc1);

  HAL_UART_Receive_IT(&huart1, &rxByte, 1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of uartMutex */
  uartMutexHandle = osMutexNew(&uartMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
    // Tao mutex de bao ve bien nhiet do
    tempMutexHandle = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of temperatureQueue */
  temperatureQueueHandle = osMessageQueueNew (1, sizeof(float), &temperatureQueue_attributes);

  /* creation of commandQueue */
  commandQueueHandle = osMessageQueueNew (32, sizeof(uint8_t), &commandQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of readTempTask */
  readTempTaskHandle = osThreadNew(StartReadTempTask, NULL, &readTempTask_attributes);

  /* creation of updateLcdTask */
  updateLcdTaskHandle = osThreadNew(StartUpdateLcdTask, NULL, &updateLcdTask_attributes);

  /* creation of sendTempTask */
  sendTempTaskHandle = osThreadNew(StartSendTempTask, NULL, &sendTempTask_attributes);

  /* creation of processCmdTask */
  processCmdTaskHandle = osThreadNew(StartProcessCmdTask, NULL, &processCmdTask_attributes);

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
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {

        osMessageQueuePut(commandQueueHandle, &rxByte, 0, 0);
        HAL_UART_Receive_IT(&huart1, &rxByte, 1);
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartReadTempTask */
/**
  * @brief  Function implementing the readTempTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartReadTempTask */
void StartReadTempTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  uint32_t xLastWakeTime = osKernelGetTickCount();
  const uint32_t xFrequency = 2000; // ms, 2 giây 1 lần

  for (;;)
  {
    // Đợi cho đến đúng thời điểm tiếp theo
    xLastWakeTime += xFrequency;
    osDelayUntil(xLastWakeTime);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    uint32_t adcVal = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    float temperature = ((1.43 - (float)adcVal * 3.3 / 4095.0) / 0.0043) + 25.0;

    if (osMutexAcquire(tempMutexHandle, osWaitForever) == osOK)
    {
        g_currentTemperature = temperature;
        osMutexRelease(tempMutexHandle);
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartUpdateLcdTask */
/**
* @brief Function implementing the updateLcdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUpdateLcdTask */
void StartUpdateLcdTask(void *argument)
{
  /* USER CODE BEGIN StartUpdateLcdTask */
	  float localTemp;
	  uint32_t localFreq;
	  char lcdBuffer[20];

  // Thêm vòng lặp delay cho task này
	  uint32_t xLastWakeTime = osKernelGetTickCount();
	  const uint32_t xFrequency = 1000; // 1 giây

  for(;;)
  {
    // Giữ khóa, đọc cả 2 giá trị nhiệt độ và tần số
	    if (osMutexAcquire(tempMutexHandle, osWaitForever) == osOK)
	    {
	        localTemp = g_currentTemperature;
	        localFreq = g_currentFrequency;
	        osMutexRelease(tempMutexHandle);
	    }

	    sprintf(lcdBuffer, "Temp: %.1fC", localTemp);
	    HD44780_SetCursor(0, 0);
	    HD44780_PrintStr(lcdBuffer);
	    HD44780_PrintStr("    ");

	    sprintf(lcdBuffer, "Freq: %luHz", localFreq);
	    HD44780_SetCursor(0, 1);
	    HD44780_PrintStr(lcdBuffer);
	    HD44780_PrintStr("    ");

	    xLastWakeTime += xFrequency;
	    osDelayUntil(xLastWakeTime);
  }
  /* USER CODE END StartUpdateLcdTask */
}

/* USER CODE BEGIN Header_StartSendTempTask */
/**
* @brief Function implementing the sendTempTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSendTempTask */
void StartSendTempTask(void *argument)
{
  /* USER CODE BEGIN StartSendTempTask */
	  float localTemp;
	  uint32_t localFreq;
	  char txBuffer[50];

	  uint32_t xLastWakeTime = osKernelGetTickCount();
	  const uint32_t xFrequency = 1000;

	  for(;;)
	  {
	    if (osMutexAcquire(tempMutexHandle, osWaitForever) == osOK)
	    {
	        localTemp = g_currentTemperature;
	        localFreq = g_currentFrequency;
	        osMutexRelease(tempMutexHandle);
	    }

	    if (osMutexAcquire(uartMutexHandle, osWaitForever) == osOK)
	    {
	      sprintf(txBuffer, "Temp: %.1f C, Freq: %lu Hz\r\n", localTemp, localFreq);
	      HAL_UART_Transmit(&huart1, (uint8_t*)txBuffer, strlen(txBuffer), 100);
	      osMutexRelease(uartMutexHandle);
	    }

	    xLastWakeTime += xFrequency;
	    osDelayUntil(xLastWakeTime);
	  }
  /* USER CODE END StartSendTempTask */
}

/* USER CODE BEGIN Header_StartProcessCmdTask */
/**
* @brief Function implementing the processCmdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartProcessCmdTask */
void StartProcessCmdTask(void *argument)
{
  char cmdBuffer[32];
  uint8_t cmdIndex = 0;
  uint8_t receivedChar;
  for(;;)
  {
    if (osMessageQueueGet(commandQueueHandle, &receivedChar, NULL, osWaitForever) == osOK)
    {

      if (receivedChar >= 32 && receivedChar <= 126 && cmdIndex < sizeof(cmdBuffer) - 1)
      {
        cmdBuffer[cmdIndex++] = receivedChar;
      }

      if (receivedChar == '\n' || receivedChar == '\r')
      {
        if(cmdIndex > 0)
        {
          cmdBuffer[cmdIndex] = '\0';
          int new_freq;
          if (sscanf(cmdBuffer, "FREQ=%d", &new_freq) == 1)
          {
            if (new_freq > 0 && new_freq <= 20000)
            {
              uint32_t new_arr = 72000000 / (72 * new_freq) - 1;
              __HAL_TIM_SET_AUTORELOAD(&htim2, new_arr);

              if (osMutexAcquire(tempMutexHandle, osWaitForever) == osOK)
              {
                  g_currentFrequency = new_freq;
                  osMutexRelease(tempMutexHandle);
              }

              if (osMutexAcquire(uartMutexHandle, osWaitForever) == osOK)
              {
                char txBuffer[50];
                sprintf(txBuffer, "OK! Frequency set to %d Hz\r\n", new_freq);
                HAL_UART_Transmit(&huart1, (uint8_t*)txBuffer, strlen(txBuffer), 100);
                osMutexRelease(uartMutexHandle);
              }
            }
            else
            {
              if (osMutexAcquire(uartMutexHandle, osWaitForever) == osOK)
              {
                char txBuffer[] = "Error: Invalid frequency value.\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t*)txBuffer, sizeof(txBuffer)-1, 100);
                osMutexRelease(uartMutexHandle);
              }
            }
          }
          else
          {
            if (osMutexAcquire(uartMutexHandle, osWaitForever) == osOK)
            {
              char txBuffer[] = "Error: Unknown command. Try 'FREQ=xxx' \r\n";
              HAL_UART_Transmit(&huart1, (uint8_t*)txBuffer, sizeof(txBuffer)-1, 100);
              osMutexRelease(uartMutexHandle);
            }
          }
        }
        cmdIndex = 0;
      }
      if (cmdIndex >= sizeof(cmdBuffer) - 1)
      {
        cmdIndex = 0;
      }
    }
  }
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
  if (htim->Instance == TIM4)
  {
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
