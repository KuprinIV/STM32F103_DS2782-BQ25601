/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bq25601.h"
#include "ds2782.h"
#include "queue.h"
#include "usbd_custom_hid_if.h"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
volatile uint8_t isUpdateEvent = 0;
volatile uint8_t ae_cnt_div = 0;
volatile uint8_t isLoadEnViaUsb = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static void checkPowerButton(void);
static void processHostCommands(uint8_t* report_data);
static uint8_t sendStatusTickHandle(void);
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
	uint8_t pg_state_prev = 0;
	BQ25601_Status bq25601_status;
	uint8_t ae_pwr_off_cntr = 0;
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
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  bq25601_drv->SetChargerEnabled(1);
  bq25601_drv->Init();
  ds2782_drv->Init();
  HAL_TIM_Base_Start_IT(&htim4); // start scanning timer
  LED_GPIO_Port->ODR |= LED_Pin; // indicate power on state
  // enable load 66 mA if no input charger source
  Load_66mA_Ctrl((PG_GPIO_Port->IDR & PG_Pin) != 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(isUpdateEvent)
	  {
		  isUpdateEvent = 0;

		  // check power button state
		  checkPowerButton();

		  // if active empty state and no charge input make power off
		  bq25601_drv->GetChargerState(&bq25601_status);
		  if((ds2782_drv->ReadStatus() & ACTIVE_EMPTY_FLAG) && (bq25601_status.vbus_status == NO_VBUS_INPUT))
		  {
			  if(ae_pwr_off_cntr++ > 5)
			  {
				  bq25601_drv->PowerOff();
			  }
		  }
		  else
		  {
			  ae_pwr_off_cntr = 0;
		  }

		  // detect active empty state
		  if((ds2782_drv->ReadStatus() & ACTIVE_EMPTY_FLAG) && ae_cnt_div == 3)
		  {
			  LED_GPIO_Port->ODR ^= LED_Pin;
		  }
		  else if(!(ds2782_drv->ReadStatus() & ACTIVE_EMPTY_FLAG))
		  {
			  LED_GPIO_Port->ODR |= LED_Pin;
		  }
		  ae_cnt_div = (ae_cnt_div+1) & 0x03;

		  /** send battery and DS2782 status data via USB. If status wasn't need to sent and commands queue isn't empty process
		   * Host commands */
		  if(!sendStatusTickHandle() && !commands_queue->IsEmpty())
		  {
			  processHostCommands(commands_queue->Poll());
		  }

		  // set load 66 mA state in depend from power good status, if it isn't enabled via USB
		  if(!isLoadEnViaUsb)
		  {
			  if(!(PG_GPIO_Port->IDR & PG_Pin) && pg_state_prev == 0)
			  {
				  pg_state_prev = 1;
				  Load_66mA_Ctrl(0);
			  }
			  else if((PG_GPIO_Port->IDR & PG_Pin) && pg_state_prev == 1)
			  {
				  pg_state_prev = 0;
				  Load_66mA_Ctrl(1);
			  }
		  }
	  }
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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 400000;
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
  htim4.Init.Prescaler = 4799;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LOAD_EN_GPIO_Port, LOAD_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|GHGEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LOAD_EN_Pin */
  GPIO_InitStruct.Pin = LOAD_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LOAD_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin GHGEN_Pin */
  GPIO_InitStruct.Pin = LED_Pin|GHGEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PWR_BTN_Pin */
  GPIO_InitStruct.Pin = PWR_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PWR_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QON_Pin */
  GPIO_InitStruct.Pin = QON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(QON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_Pin */
  GPIO_InitStruct.Pin = INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PG_Pin */
  GPIO_InitStruct.Pin = PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PG_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
/**
 * @brief Check power button state. If button was pressed more than 1 s make power off
 * @param: None
 * @return: None
 */
static void checkPowerButton(void)
{
  static uint8_t powerOffCntr;
  static uint8_t isPwrBtnPressed;

  if((PWR_BTN_GPIO_Port->IDR & PWR_BTN_Pin) == 0 && !isPwrBtnPressed)
  {
	  isPwrBtnPressed = 1;
	  powerOffCntr++;
  }
  else if((PWR_BTN_GPIO_Port->IDR & PWR_BTN_Pin) == 0 && isPwrBtnPressed)
  {
	  if(powerOffCntr++ >= 10)
	  {
		  powerOffCntr = 0;
		  bq25601_drv->PowerOff();
	  }
  }
  else if((PWR_BTN_GPIO_Port->IDR & PWR_BTN_Pin) && isPwrBtnPressed)
  {
	  powerOffCntr = 0;
	  isPwrBtnPressed = 0;
  }
}

/**
 * @brief Process commands, received via HID reports from Host
 * @param: report_data - HID report data with command (0 - report ID, 1 - command ID, 2 - command data length, 3...36 - command data)
 * @return: None
 */
static void processHostCommands(uint8_t* report_data)
{
	uint8_t outputData[USBD_CUSTOMHID_INREPORT_BUF_SIZE]= {0};
	BQ25601_Status bq25601_status;
	BQ25601_FaultType charger_fault_state;

	if(report_data[0] == COMMANDS_ID)
	{
		switch(report_data[1])
		{
			case READ_DS2782_REGISTERS_DATA:
				outputData[0] = COMMANDS_RESPONSE_ID;
				outputData[1] = READ_DS2782_REGISTERS_DATA;
				outputData[2] = report_data[4];

				ds2782_drv->ReadRegistersMap(report_data[3], outputData+3, report_data[4]);
				USBD_CUSTOM_HID_SendReport_FS(outputData, report_data[4]+3);
				break;

			case READ_DS2782_EEPROM_DATA:
				outputData[0] = COMMANDS_RESPONSE_ID;
				outputData[1] = READ_DS2782_EEPROM_DATA;
				outputData[2] = report_data[5];
				ds2782_drv->ReadEepromBlock(report_data[3], report_data[4], outputData+3, report_data[5]);
				USBD_CUSTOM_HID_SendReport_FS(outputData, report_data[5]+3);
				break;

			case WRITE_DS2782_EEPROM_DATA:
				ds2782_drv->WriteEepromBlock(report_data[3], report_data[4], report_data+5, report_data[2]-2);
				break;

			case LOCK_DS2782_EEPROM_BLOCK: // disable lock DS2782 EEPROM function for debug purposes
				//ds2782_drv->LockEepromBlock(report_data[3] & 0x01);
				break;

			case READ_DS2782_EEPROM_LOCK_STATUS:
				outputData[0] = COMMANDS_RESPONSE_ID;
				outputData[1] = READ_DS2782_EEPROM_LOCK_STATUS;
				outputData[2] = 1;
				outputData[3] = ds2782_drv->IsEepromBlockLocked(report_data[3] & 0x01);
				USBD_CUSTOM_HID_SendReport_FS(outputData, 4);
				break;

			case READ_BQ25601_STATUS:
				outputData[0] = COMMANDS_RESPONSE_ID;
				outputData[1] = READ_BQ25601_STATUS;
				outputData[2] = sizeof(bq25601_status);
				bq25601_drv->GetChargerState(&bq25601_status);
				memcpy(outputData+3, &bq25601_status, sizeof(bq25601_status));
				USBD_CUSTOM_HID_SendReport_FS(outputData, sizeof(bq25601_status)+3);
				break;

			case READ_BQ25601_FAULTS:
				outputData[0] = COMMANDS_RESPONSE_ID;
				outputData[1] = READ_BQ25601_FAULTS;
				outputData[2] = sizeof(charger_fault_state);
				bq25601_drv->GetChargerFault(&charger_fault_state);
				memcpy(outputData+3, &charger_fault_state, sizeof(charger_fault_state));
				USBD_CUSTOM_HID_SendReport_FS(outputData, sizeof(charger_fault_state)+3);
				break;

			case BQ25601_CHG_CTRL:
				bq25601_drv->SetChargerEnabled(report_data[3] & 0x01);
				break;

			case LOAD_66MA_CTRL:
				isLoadEnViaUsb = report_data[3] & 0x01;
				Load_66mA_Ctrl(report_data[3] & 0x01);
				break;

			default:
				break;
		}
	}
}

/**
 * @brief Send DS2782 status and battery parameters
 * @param: None
 * @return: 0 - data wasn't sent, 1 - data was sent
 */
static uint8_t sendStatusTickHandle(void)
{
	static uint8_t battery_status_cnt_div;
	uint8_t outputData[USBD_CUSTOMHID_INREPORT_BUF_SIZE]= {0};
	uint8_t result = 0;
	int16_t batteryVoltage = 0;
	int16_t batteryCurrent = 0;

	if(battery_status_cnt_div < 20)
	{
	  battery_status_cnt_div++;
	}
	else
	{
	  battery_status_cnt_div = 0;

	  outputData[0] = COMMANDS_RESPONSE_ID;
	  outputData[1] = READ_DS2782_REGISTERS_DATA;
	  outputData[2] = 32;
	  // read DS2782 registers map
	  ds2782_drv->ReadRegistersMap(STATUS_REG, outputData+3, 31);
	  // read battery voltage and current values
	  batteryVoltage = ds2782_drv->ReadBatteryVoltage();
	  batteryCurrent = ds2782_drv->ReadBatteryCurrent();
	  // change battery voltage and current data
	  outputData[VOLT_MSB_REG+3-STATUS_REG] = (uint8_t)(batteryVoltage>>8);
	  outputData[VOLT_LSB_REG+3-STATUS_REG] = (uint8_t)(batteryVoltage & 0xFF);
	  outputData[CURRENT_MSB_REG+3-STATUS_REG] = (uint8_t)(batteryCurrent>>8);
	  outputData[CURRENT_LSB_REG+3-STATUS_REG] = (uint8_t)(batteryCurrent & 0xFF);
	  // read RSNSP value
	  outputData[35] = ds2782_drv->ReadRegister8b(RSNSP_MB);

	  USBD_CUSTOM_HID_SendReport_FS(outputData, sizeof(outputData));
	  result = 1;
	}
	return result;
}

/**
 * @brief Load 66 mA control
 * @param: is_enabled: 0 - load isn't connected, 1 - load is connected
 * @return: None
 */
void Load_66mA_Ctrl(uint8_t is_enabled)
{
	if(is_enabled)
	{
		LOAD_EN_GPIO_Port->ODR |= LOAD_EN_Pin;
	}
	else
	{
		LOAD_EN_GPIO_Port->ODR &= ~LOAD_EN_Pin;
	}
}

/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	isUpdateEvent = 1;
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

