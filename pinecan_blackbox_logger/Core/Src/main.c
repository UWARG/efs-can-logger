/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "blackbox_logger.h"
#include "can_receiver.h"
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// FOR FUTURE: Could use g_fault_code to output viewable codes (LED blinks or something like that)
static volatile FaultCode g_fault_code = FAULT_NONE;
static volatile bool g_runtime_fault = false;

typedef enum {
  CAN_TEST_NOT_RUN = 0,
  CAN_TEST_WAITING_FOR_RX,
  CAN_TEST_PASSED,
  CAN_TEST_INIT_FAILED,
  CAN_TEST_TX_FAILED,
  CAN_TEST_RX_TIMEOUT,
  CAN_TEST_RX_OVERRUN,
  CAN_TEST_SD_FAILED
} CanTestStatus;

static volatile CanTestStatus g_can_test_status = CAN_TEST_NOT_RUN;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_TEST_FRAME_COUNT  1000U
#define CAN_TEST_PERIOD_MS    10U
#define CAN_TEST_TOTAL_TIMEOUT_MS (CAN_TEST_FRAME_COUNT * CAN_TEST_PERIOD_MS + 2000U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static volatile uint32_t g_test_tx_count = 0;
static volatile bool g_test_tx_failed = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
static void Recoverable_ErrorHandler(FaultCode fault);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void CanTest_StartTimer(void) {
    __HAL_RCC_TIM7_CLK_ENABLE();

    TIM7->CR1 = 0;
    TIM7->DIER = 0;

    // Current configuration: TIM7 clock = 80 MHz.
    // Divide to a 10 kHz counter, then interrupt every 10 ms.
    TIM7->PSC = 8000U - 1U;
    TIM7->ARR = CAN_TEST_PERIOD_MS * 10U - 1U;
    TIM7->CNT = 0;

    // Load the prescaler, then clear the resulting update flag.
    TIM7->EGR = TIM_EGR_UG;
    TIM7->SR = 0;

    // CAN RX currently has priority 0, so it can interrupt this handler.
    HAL_NVIC_SetPriority(TIM7_IRQn, 1, 0);
    HAL_NVIC_ClearPendingIRQ(TIM7_IRQn);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);

    TIM7->DIER = TIM_DIER_UIE;
    TIM7->CR1 = TIM_CR1_CEN;
}

static void CanTest_StopTimer(void) {
    TIM7->DIER = 0;
    TIM7->CR1 = 0;
    HAL_NVIC_DisableIRQ(TIM7_IRQn);
    HAL_NVIC_ClearPendingIRQ(TIM7_IRQn);
}

void TIM7_IRQHandler(void) {
    if ((TIM7->SR & TIM_SR_UIF) == 0U) {
        return;
    }

    TIM7->SR = 0;

    if (g_test_tx_failed ||
        g_test_tx_count >= CAN_TEST_FRAME_COUNT) {
        TIM7->DIER = 0;
        TIM7->CR1 = 0;
        return;
    }

    // Treat an unavailable mailbox as a test failure:
    // do not silently slow down the requested transmission rate.
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
        g_test_tx_failed = true;
        TIM7->DIER = 0;
        TIM7->CR1 = 0;
        return;
    }

    uint32_t sequence = g_test_tx_count;
    uint8_t data[8] = {
        (uint8_t)sequence,
        (uint8_t)(sequence >> 8),
        (uint8_t)(sequence >> 16),
        (uint8_t)(sequence >> 24),
        0, 0, 0, 0
    };

    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;

    header.StdId = 0x321U;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = sizeof(data);

    if (HAL_CAN_AddTxMessage(
            &hcan1, &header, data, &mailbox) != HAL_OK) {
        g_test_tx_failed = true;
        TIM7->DIER = 0;
        TIM7->CR1 = 0;
        return;
    }

    g_test_tx_count = sequence + 1U;
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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  if (!BlackboxLogger_Init("blackbox.bin")) {
    g_can_test_status = CAN_TEST_INIT_FAILED;
    Error_Handler();
  }

  if (!CanReceiver_Init()) {
    g_can_test_status = CAN_TEST_INIT_FAILED;
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  g_test_tx_count = 0;
  g_test_tx_failed = false;
  g_can_test_status = CAN_TEST_WAITING_FOR_RX;

  uint32_t test_started = HAL_GetTick();
  CanTest_StartTimer();

  while (g_test_tx_count < CAN_TEST_FRAME_COUNT ||
        CanReceiver_GetReceived() < CAN_TEST_FRAME_COUNT) {

      if (g_test_tx_failed) {
          g_can_test_status = CAN_TEST_TX_FAILED;
          break;
      }

      if (BlackboxLogger_HasStorageError() ||
          BlackboxLogger_GetDroppedRecordCount() != 0U) {
          g_can_test_status = CAN_TEST_SD_FAILED;
          break;
      }

      if (CanReceiver_GetOverrun() != 0U) {
          g_can_test_status = CAN_TEST_RX_OVERRUN;
          break;
      }

      if ((uint32_t)(HAL_GetTick() - test_started) >=
          CAN_TEST_TOTAL_TIMEOUT_MS) {
          g_can_test_status = CAN_TEST_RX_TIMEOUT;
          break;
      }

      // Timer and CAN interrupts continue during this blocking write.
      BlackboxLogger_Service();
  }

  CanTest_StopTimer();

  if (g_can_test_status == CAN_TEST_WAITING_FOR_RX) {
      if (g_test_tx_failed) {
        g_can_test_status = CAN_TEST_TX_FAILED;
      } else if (CanReceiver_GetOverrun() != 0U) {
          g_can_test_status = CAN_TEST_RX_OVERRUN;
      } else if (!BlackboxLogger_Flush() ||
                  BlackboxLogger_HasStorageError() ||
                  BlackboxLogger_GetDroppedRecordCount() != 0U) {
          g_can_test_status = CAN_TEST_SD_FAILED;
      } else {
          g_can_test_status = CAN_TEST_PASSED;
      }
  }

  while (1)
  {
    if (BlackboxLogger_HasStorageError() && !g_runtime_fault) {
      Recoverable_ErrorHandler(FAULT_STORAGE);
    }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_LOOPBACK;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin|SPI2_CS_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SMPS_EN_Pin SMPS_V1_Pin SMPS_SW_Pin SPI2_CS_PIN_Pin */
  GPIO_InitStruct.Pin = SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin|SPI2_CS_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SMPS_PG_Pin */
  GPIO_InitStruct.Pin = SMPS_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SMPS_PG_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_GPIO_WritePin(SPI2_CS_PIN_GPIO_Port, SPI2_CS_PIN_Pin, GPIO_PIN_SET);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Records a recoverable runtime fault without stopping the MCU.
  * @param  fault Fault code to record.
  * @retval None
  */
static void Recoverable_ErrorHandler(FaultCode fault)
{
  g_fault_code = fault;
  g_runtime_fault = true;
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
  // Temp value
  // Later we can add more definitions to the enum to specify errors/faults
  g_fault_code = FAULT_STARTUP;
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
