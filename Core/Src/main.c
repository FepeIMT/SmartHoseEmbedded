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
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

enum status
{
	APAGADO = 0,
	ENCENDIDO,
	ROOM1_ON,
	ROOM1_OFF,
	ROOM2_ON,
	ROOM2_OFF,
	ROOM3_ON,
	ROOM3_OFF,
	BATHROOM1_ON,
	BATHROOM1_OFF,
	BATHROOM2_ON,
	BATHROOM2_OFF,
	KITCHEN_ON,
	KITCHEN_OFF,
	GARDEN_ON,
	GARDEN_OFF,
	LIVINGROOM_ON,
	LIVINGROOM_OFF,
	FAN_ON,
	FAN_OFF
};

enum function
{
	Update_Temp,
	Update_Leds,
	Data_Requested
};

enum info
{
	COMMAND,
	DATA
};

enum options
{
	Change_State,
	Request_FirstFloor,
	Request_SecondFloor,
	Automatic_Mode,
	Manual_Mode
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef hlpuart1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for UpdateScreen */
osThreadId_t UpdateScreenHandle;
const osThreadAttr_t UpdateScreen_attributes = {
  .name = "UpdateScreen",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for ReadLM35 */
osThreadId_t ReadLM35Handle;
const osThreadAttr_t ReadLM35_attributes = {
  .name = "ReadLM35",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */

uint8_t temp_LM35;
uint8_t buffer[2];
uint8_t buffer_in[2];
uint8_t current_mode = Automatic_Mode;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C3_Init(void);
static void MX_ADC1_Init(void);
void StartUpdate(void *argument);
void StartRead(void *argument);

/* USER CODE BEGIN PFP */
void UpdateFirstFloor(void);
void UpdateSecondFloor(void);
void ChangePinState(uint8_t data);
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
  MX_LPUART1_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&hlpuart1, buffer_in, 2);



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
  /* creation of UpdateScreen */
  UpdateScreenHandle = osThreadNew(StartUpdate, NULL, &UpdateScreen_attributes);

  /* creation of ReadLM35 */
  ReadLM35Handle = osThreadNew(StartRead, NULL, &ReadLM35_attributes);

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_I2C3
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c3.Init.Timing = 0x107075B0;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PF12 PF13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PF14 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE8 PE10 PE11
                           PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/*ISR routines */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	buffer[COMMAND] = Update_Leds;
	switch(GPIO_Pin)
	{
		case GPIO_PIN_8:
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
			if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11))
			{
				buffer[DATA] = ROOM1_ON;
			}
			else
			{
				buffer[DATA] = ROOM1_OFF;
			}
			break;
		case GPIO_PIN_9:
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14);
			if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_14))
			{
				buffer[DATA] = ROOM2_ON;
			}
			else
			{
				buffer[DATA] = ROOM2_OFF;
			}
			break;
		case GPIO_PIN_5:
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);
			if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13))
			{
				buffer[DATA] = ROOM3_ON;
			}
			else
			{
				buffer[DATA] = ROOM3_OFF;
			}
			break;
		case GPIO_PIN_6:
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_15);
			if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_15))
			{
				buffer[DATA] = BATHROOM1_ON;
			}
			else
			{
				buffer[DATA] = BATHROOM1_OFF;
			}
			break;
		case GPIO_PIN_7:
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_8);
			if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8))
			{
				buffer[DATA] = BATHROOM2_ON;
			}
			else
			{
				buffer[DATA] = BATHROOM2_OFF;
			}
			break;
		case GPIO_PIN_14:
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_9);
			if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9))
			{
				buffer[DATA] = KITCHEN_ON;
			}
			else
			{
				buffer[DATA] = KITCHEN_OFF;
			}
			break;
		case GPIO_PIN_15:
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
			if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8))
			{
				buffer[DATA] = GARDEN_ON;
			}
			else
			{
				buffer[DATA] = GARDEN_OFF;
			}
			break;
		case GPIO_PIN_12:
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_7);
			if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7))
			{
				buffer[DATA] = LIVINGROOM_ON;
			}
			else
			{
				buffer[DATA] = LIVINGROOM_OFF;
			}
			break;
		case GPIO_PIN_13:
			if(current_mode == Manual_Mode)
			{
				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
				if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10))
				{
					buffer[DATA] = FAN_ON;
				}
				else
				{
					buffer[DATA] = FAN_OFF;
				}
			}
			else
			{
				buffer[DATA] = 50;
			}
			break;
		case GPIO_PIN_11:
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
			if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8))
			{
				buffer[DATA] = GARDEN_ON;
			}
			else
			{
				buffer[DATA] = GARDEN_OFF;
			}
			break;
	}


	HAL_UART_Transmit(&hlpuart1, buffer, 2, HAL_MAX_DELAY);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{


	if(buffer_in[COMMAND] == Change_State)
	{
		ChangePinState(buffer_in[1]);
	}
	else if(buffer_in[COMMAND] == Request_FirstFloor)
	{
		UpdateFirstFloor();
	}
	else if(buffer_in[COMMAND] == Request_SecondFloor)
	{
		UpdateSecondFloor();
	}
	else if(buffer_in[COMMAND] == Automatic_Mode)
	{
		current_mode = Automatic_Mode;
	}
	else if(buffer_in[COMMAND] == Manual_Mode)
	{
		current_mode = Manual_Mode;
	}

	//The UART Port will be ready when arrive another data
	HAL_UART_Receive_IT(&hlpuart1, buffer_in, 2);
}


void UpdateFirstFloor(void)
{
	buffer[COMMAND] = Data_Requested;
	if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_15))
	{
		buffer[DATA] = BATHROOM1_ON;
		HAL_UART_Transmit(&hlpuart1, buffer, 2, HAL_MAX_DELAY);

	}

	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9))
	{
		buffer[DATA] = KITCHEN_ON;
		HAL_UART_Transmit(&hlpuart1, buffer, 2, HAL_MAX_DELAY);
	}

	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8))
	{
		buffer[DATA] = GARDEN_ON;
		HAL_UART_Transmit(&hlpuart1, buffer, 2, HAL_MAX_DELAY);
	}

	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7))
	{
		buffer[DATA] = LIVINGROOM_ON;
		HAL_UART_Transmit(&hlpuart1, buffer, 2, HAL_MAX_DELAY);
	}

}

void UpdateSecondFloor(void)
{
	buffer[COMMAND] = Data_Requested;

	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11))
	{
		buffer[DATA] = ROOM1_ON;
		HAL_UART_Transmit(&hlpuart1, buffer, 2, HAL_MAX_DELAY);
	}

	if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_14))
	{
		buffer[DATA] = ROOM2_ON;
		HAL_UART_Transmit(&hlpuart1, buffer, 2, HAL_MAX_DELAY);
	}

	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13))
	{
		buffer[DATA] = ROOM3_ON;
		HAL_UART_Transmit(&hlpuart1, buffer, 2, HAL_MAX_DELAY);
	}

	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8))
	{
		buffer[DATA] = BATHROOM2_ON;
		HAL_UART_Transmit(&hlpuart1, buffer, 2, HAL_MAX_DELAY);
	}

	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10))
	{
		buffer[DATA] = FAN_ON;
		HAL_UART_Transmit(&hlpuart1, buffer, 2, HAL_MAX_DELAY);
	}

}

void ChangePinState(uint8_t data)
{
	buffer[COMMAND] = Update_Leds;

	switch(data)
	{
		case ROOM1_ON:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, ENCENDIDO);
			buffer[DATA] = ROOM1_ON;
			break;
		case ROOM1_OFF:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, APAGADO);
			buffer[DATA] = ROOM1_OFF;
			break;
		case ROOM2_ON:
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, ENCENDIDO);
			buffer[DATA] = ROOM2_ON;
			break;
		case ROOM2_OFF:
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, APAGADO);
			buffer[DATA] = ROOM2_OFF;
			break;
		case ROOM3_ON:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, ENCENDIDO);
			buffer[DATA] = ROOM3_ON;
			break;
		case ROOM3_OFF:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, APAGADO);
			buffer[DATA] = ROOM3_OFF;
			break;
		case BATHROOM1_ON:
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, ENCENDIDO);
			buffer[DATA] = BATHROOM1_ON;
			break;
		case BATHROOM1_OFF:
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, APAGADO);
			buffer[DATA] = BATHROOM1_OFF;
			break;
		case BATHROOM2_ON:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, ENCENDIDO);
			buffer[DATA] = BATHROOM2_ON;
			break;
		case BATHROOM2_OFF:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, APAGADO);
			buffer[DATA] = BATHROOM2_OFF;
			break;
		case KITCHEN_ON:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, ENCENDIDO);
			buffer[DATA] = KITCHEN_ON;
			break;
		case KITCHEN_OFF:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, APAGADO);
			buffer[DATA] = KITCHEN_OFF;
			break;
		case GARDEN_ON:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, ENCENDIDO);
			buffer[DATA] = GARDEN_ON;
			break;
		case GARDEN_OFF:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, APAGADO);
			buffer[DATA] = GARDEN_OFF;
			break;
		case LIVINGROOM_ON:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, ENCENDIDO);
			buffer[DATA] = LIVINGROOM_ON;
			break;
		case LIVINGROOM_OFF:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, APAGADO);
			buffer[DATA] = LIVINGROOM_OFF;
			break;
		case FAN_ON:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, ENCENDIDO);
			buffer[DATA] = FAN_ON;
			break;
		case FAN_OFF:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, APAGADO);
			buffer[DATA] = FAN_OFF;
			break;
	}
	//Transimit the confirmation that the state change LED/FAN
	HAL_UART_Transmit(&hlpuart1, buffer, 2, HAL_MAX_DELAY);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartUpdate */
/**
  * @brief  Function implementing the UpdateScreen thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUpdate */
void StartUpdate(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

	  osDelay(1);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartRead */
/**
* @brief Function implementing the ReadLM35 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRead */
void StartRead(void *argument)
{
  /* USER CODE BEGIN StartRead */
  /* Infinite loop */
  for(;;)
  {
	//GET ADC Value
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	temp_LM35 = HAL_ADC_GetValue(&hadc1)/10;
	buffer[DATA] = temp_LM35;
	buffer[COMMAND] = Update_Temp;

	HAL_UART_Transmit_IT(&hlpuart1, buffer, 2);

	osDelay(250);
  }
  /* USER CODE END StartRead */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
