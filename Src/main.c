/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define SPI_REG_INT_STTS		0x06
#define SPI_REG_RX_DAT_LEN		0x02
#define SPI_REG_TX_BUFF_AVAIL	0x03
#define SPI_CMD_RX_DATA			0x10
#define SPI_CMD_TX_CMD			0x91
#define SPI_CMD_TX_DATA			0x90

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEIL //KEIL ,True_STD
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
enum {
TRANSFER_READY,
TRANSFER_WAIT,
TRANSFER_COMPLETE,
TRANSFER_ERROR
};

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
//uint8_t SPI_buffer_tx[10]={61, 62, 63, 64, 65, 66, 67, 68, 69, 70};
//uint8_t SPI_buffer_rx[10] = {0,};
/* transfer state */
__IO uint32_t wTransferState = TRANSFER_READY;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
#ifdef KEIL
      #ifdef __GNUC__
      //With GCC, small printf (option LD Linker->Libraries->Small printf
      //set to 'Yes') calls __io_putchar()
         #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
		  #else
				 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
		  #endif /* __GNUC__*/
    #if 1
    PUTCHAR_PROTOTYPE
    {
      HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, 0xFFFF);
      return ch;
    }
    #endif
  #endif			 

  #ifdef True_STD
  int _write(int fd, char *str, int len)
  {
    for(int i=0; i<len; i++)
    {
      HAL_UART_Transmit(&huart2, (uint8_t *)&str[i], 1, 0xFFFF);
    }
    return len;
  }
#endif
void Clear_SPI_CR2_SSOE(SPI_HandleTypeDef *hspi);
void Clear_SPI_MasterEnable(SPI_HandleTypeDef *hspi)
{
  SET_BIT(hspi->Instance->CR1, SPI_CR1_MSTR|SPI_CR1_SPE);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uart_buffer uart_2;
uart_buffer uart_6;

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  //while(1);
  printf(" HAL_SPI_TxRxCpltCallback  \r\n");
  //CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_SSOE);
  //SET_BIT(hspi->Instance->CR2, SPI_CR2_SSOE);
  wTransferState = TRANSFER_COMPLETE;
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  printf(" HAL_SPI_ERROR \r\n");
  wTransferState = TRANSFER_ERROR;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t temp_data[10] = {61, 62, 63, 64, 65, 66, 67, 68, 69, 70};
  uint8_t SPI_buffer_tx[10]={61, 62, 63, 64, 65, 66, 67, 68, 69, 70};
  uint8_t SPI_buffer_rx[10] = {0,};
  uint8_t temp_uart_buffer[512];
  uint8_t rxData;
  int temp_uart_index = 0;
  uint8_t temp_cmd = 0;
  uint8_t cmd_code = 0;;
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  
  uart_buffer_init(&uart_2);
  uart_buffer_init(&uart_6);
  //HAL_SPI_TransmitReceive_DMA(&hspi1, SPI_buffer_tx, SPI_buffer_tx, 10);
  //memcpy(SPI_buffer_tx, temp_data, 10);
  //HAL_SPI_TransmitReceive_DMA(&hspi1, SPI_buffer_tx, SPI_buffer_tx, 10);

  HAL_MspInit();
  HAL_SPI_MspInit(&hspi1);
  __HAL_SPI_ENABLE(&hspi1);
  //HAL_SPI_Receive_DMA_INIT(&hspi1,SPI_buffer_rx,2);

  HAL_UART_Receive_IT(&huart2, &rxData, 1);
  HAL_UART_Receive_IT(&huart6, &rxData, 1);
  //HAL_SPI_TransmitReceive_DMA(&hspi1, SPI_buffer_tx, SPI_buffer_rx, 2);
  //Clear_SPI_CR2_SSOE(&hspi1);
  
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("-------------------------------------\r\n");
  printf(" \t SPI DMA Test Program  \r\n");
  printf("-------------------------------------\r\n");
  while (1)
  {
  	if(get_uart_flag(&uart_2) == 1)
	{
		temp_uart_index = 0;
		while(uart_buffer_empty(&uart_2) == 0)
		{
			temp_uart_buffer[temp_uart_index++] = uart_Dequeue(&uart_2);
		}
		temp_uart_buffer[temp_uart_index++] = 0;
		printf("uart_2 : %s \n", temp_uart_buffer);
		uart_flag_reset(&uart_2);
	}
	
	if(get_uart_flag(&uart_6) == 1)
	{
		temp_uart_index = 0;
		while(uart_buffer_empty(&uart_6) == 0)
		{
			temp_uart_buffer[temp_uart_index++] = uart_Dequeue(&uart_6);
		}
		temp_uart_buffer[temp_uart_index++] = 0;
		printf("uart_6 : %s \n", temp_uart_buffer);
		uart_flag_reset(&uart_6);
		
		if(strncmp(temp_uart_buffer,"stts",4) == 0)
		{	//send
			SPI_buffer_tx[0] = SPI_REG_INT_STTS;
			SPI_buffer_tx[1] = 0xff;
			SPI_buffer_rx[2] = 0x00;
			cmd_code = SPI_REG_INT_STTS;
			wTransferState = TRANSFER_WAIT;
			if(HAL_SPI_TransmitReceive_DMA(&hspi1, SPI_buffer_tx, SPI_buffer_rx, 3) != HAL_OK)
			{
				Error_Handler();
			}
			//Clear_SPI_CR2_SSOE(&hspi1);
			#if 0
			printf("spi1 tx:%02x rx:%02x%02x \r\n", SPI_buffer_tx[0], SPI_buffer_rx[2], SPI_buffer_rx[1]);
			if(SPI_buffer_rx[1] == 0x01)
			{
				printf("stts = %02x\r\n", SPI_buffer_rx[1]);
			}
			printf("_spi1 tx:%02x rx:%02x%02x \r\n", SPI_REG_INT_STTS, SPI_buffer_rx[2], SPI_buffer_rx[1]);
			Clear_SPI_MasterEnable(&hspi1);
			#endif
		}
		else if(strncmp(temp_uart_buffer,"tbav",4) == 0)
		{	//send
			SPI_buffer_tx[0] = SPI_REG_TX_BUFF_AVAIL;
			SPI_buffer_tx[1] = 0xff;
			SPI_buffer_rx[2] = 0x00;
			cmd_code = SPI_REG_TX_BUFF_AVAIL;
			wTransferState = TRANSFER_WAIT;
			if(HAL_SPI_TransmitReceive_DMA(&hspi1, SPI_buffer_tx, SPI_buffer_rx, 3) != HAL_OK)
			{
				Error_Handler();
			}
			//Clear_SPI_CR2_SSOE(&hspi1);
			#if 0
			printf("spi1 tx:%02x rx:%02x%02x \r\n", SPI_buffer_tx[0], SPI_buffer_rx[2], SPI_buffer_rx[1]);
			if(SPI_buffer_rx[1] & 0x02)
			{
				printf("BUFF AVAIL = %02x\r\n", SPI_buffer_rx[1]);
			}
			printf("_spi1 tx:%02x rx:%02x%02x \r\n", SPI_REG_TX_BUFF_AVAIL, SPI_buffer_rx[2], SPI_buffer_rx[1]);
			#endif
		}
		else if(strncmp(temp_uart_buffer,"rlen",4) == 0)
		{	//send
			SPI_buffer_tx[0] = SPI_REG_RX_DAT_LEN;
			SPI_buffer_tx[1] = 0xff;
			SPI_buffer_rx[2] = 0x00;
			cmd_code = SPI_REG_RX_DAT_LEN;
			wTransferState = TRANSFER_WAIT;
			if(HAL_SPI_TransmitReceive_DMA(&hspi1, SPI_buffer_tx, SPI_buffer_rx, 3) != HAL_OK)
			{
				Error_Handler();
			}
			
			//Clear_SPI_CR2_SSOE(&hspi1);
			#if 0
			printf("spi1 tx:%02x rx:%02x%02x \r\n", SPI_buffer_tx[0], SPI_buffer_rx[2], SPI_buffer_rx[1]);
			if((SPI_buffer_rx[2]<<8 |(SPI_buffer_rx[1]&0x00FF)) > 0 )
			{
				printf("Recv len = %02x%02x\r\n", SPI_buffer_rx[2], SPI_buffer_rx[1]);
			}
			printf("_spi1 tx:%02x rx:%02x%02x \r\n", SPI_REG_RX_DAT_LEN, SPI_buffer_rx[2], SPI_buffer_rx[1]);
			Clear_SPI_MasterEnable(&hspi1);
			#endif
		}
	}
	switch(wTransferState)
	{
		case TRANSFER_READY:
			break;
		case TRANSFER_ERROR:
			printf("SPI ERROR\r\n");
			wTransferState = TRANSFER_READY;
			break;
		case TRANSFER_COMPLETE:
			printf("spi1 tx:%02x rx:%02x%02x \r\n", cmd_code, SPI_buffer_rx[2], SPI_buffer_rx[1]);
			wTransferState = TRANSFER_READY;
			#if 0
			switch(cmd_code)
			{
				case SPI_REG_INT_STTS:
					break;
				case SPI_REG_TX_BUFF_AVAIL:
					break;
				case SPI_REG_RX_DAT_LEN:
					break;
			}
			#endif
			break;
		case TRANSFER_WAIT:
			break;
		default :
			break;
	}
	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //memcpy(SPI_buffer_tx, temp_data, 10);
    //HAL_SPI_TransmitReceive_DMA(&hspi1, SPI_buffer_tx, SPI_buffer_tx, 10);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the WakeUp 
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
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
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
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
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
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
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Clear_SPI_CR2_SSOE(SPI_HandleTypeDef *hspi)
{
  CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_SSOE);
  SET_BIT(hspi->Instance->CR2, SPI_CR2_SSOE);

}

void uart2_enqueue(char data)
{
	uart_Enqueue(&uart_2, data);
}
void uart6_enqueue(char data)
{
	uart_Enqueue(&uart_6, data);
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
