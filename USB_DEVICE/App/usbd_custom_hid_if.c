/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_custom_hid_if.c
  * @version        : v2.0_Cube
  * @brief          : USB Device Custom HID interface file.
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
#include "usbd_custom_hid_if.h"

/* USER CODE BEGIN INCLUDE */
#include "bq25601.h"
#include "ds2782.h"
#include "main.h"
#include "queue.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @addtogroup USBD_CUSTOM_HID
  * @{
  */

/** @defgroup USBD_CUSTOM_HID_Private_TypesDefinitions USBD_CUSTOM_HID_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Defines USBD_CUSTOM_HID_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Macros USBD_CUSTOM_HID_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Variables USBD_CUSTOM_HID_Private_Variables
  * @brief Private variables.
  * @{
  */

/** Usb HID report descriptor. */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
  /* USER CODE BEGIN 0 */
		0x06, 0x00, 0xff,              // USAGE_PAGE (Vendor Defined Page 1)
		0x09, 0x01,                    // USAGE (Vendor Usage 1)
		0xa1, 0x01,                    // COLLECTION (Application)

		/** read battery data state: (bytes: 0 - report ID (0x01), 1,2 - battery voltage (in volts*100), 3,4 - battery current (in mA),
		 5 - battery relative capacity in %) */
		0x09, 0x01,                    //   USAGE (Vendor Usage 1)
		0x85, 0x01,               	   //   REPORT_ID (1)
		0x95, 0x05,                    //   REPORT_COUNT (5)
		0x75, 0x08,                    //   REPORT_SIZE (8)
		0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
		0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
		0x81, 0x82,                    //   INPUT (Data,Var,Abs)

		/** read DS2782 status register: (bytes: 0 - report ID (0x02), 1 - status register value) */
		0x09, 0x01,                    //   USAGE (Vendor Usage 1)
		0x85, 0x02,               	   //   REPORT_ID (2)
		0x95, 0x01,                    //   REPORT_COUNT (1)
		0x75, 0x08,                    //   REPORT_SIZE (8)
		0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
		0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
		0x81, 0x82,                    //   INPUT (Data,Var,Abs)

		/** read DS2782 EEPROM block 1 lock status: (bytes: 0 - report ID (0x03), 1 - lock state (0 - isn't locked, 1 - is locked)) */
		0x09, 0x01,                    //   USAGE (Vendor Usage 1)
		0x85, 0x03,               	   //   REPORT_ID (3)
		0x95, 0x01,                    //   REPORT_COUNT (1)
		0x75, 0x08,                    //   REPORT_SIZE (8)
		0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
		0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
		0x81, 0x82,                    //   INPUT (Data,Var,Abs)

		/** read DS2782 EEPROM block 1 data: (bytes: 0 - report ID (0x04), 1 - start address, 2 - data length) */
		0x09, 0x01,                    //   USAGE (Vendor Usage 1)
		0x85, 0x04,               	   //   REPORT_ID (4)
		0x95, 0x02,                    //   REPORT_COUNT (2)
		0x75, 0x08,                    //   REPORT_SIZE (8)
		0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
		0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
		0x91, 0x82,                    //   OUTPUT (Data,Var,Abs)

		/** send DS2782 EEPROM block 1 data: (bytes: 0 - report ID (0x05), 1...32 - EEPROM block 1 data) */
		0x09, 0x01,                    //   USAGE (Vendor Usage 1)
		0x85, 0x05,               	   //   REPORT_ID (5)
		0x95, 0x20,                    //   REPORT_COUNT (32)
		0x75, 0x08,                    //   REPORT_SIZE (8)
		0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
		0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
		0x81, 0x82,                    //   INPUT (Data,Var,Abs)

		/** write DS2782 EEPROM block 1 data: (bytes: 0 - report ID (0x06), 1 - start address, 2 - data length,  3...34 - EEPROM block 1 data) */
		0x09, 0x01,                    //   USAGE (Vendor Usage 1)
		0x85, 0x06,               	   //   REPORT_ID (6)
		0x95, 0x22,                    //   REPORT_COUNT (34)
		0x75, 0x08,                    //   REPORT_SIZE (8)
		0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
		0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
		0x91, 0x82,                    //   OUTPUT (Data,Var,Abs)

		/** read BQ25601 status data: (bytes: 0 - report ID (0x07), 1...4 - BQ25601 status register value) */
		0x09, 0x01,                    //   USAGE (Vendor Usage 1)
		0x85, 0x07,               	   //   REPORT_ID (7)
		0x95, 0x04,                    //   REPORT_COUNT (4)
		0x75, 0x08,                    //   REPORT_SIZE (8)
		0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
		0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
		0x81, 0x82,                    //   INPUT (Data,Var,Abs)

		/** read BQ25601 fault state data: (bytes: 0 - report ID (0x08), 1...5 - BQ25601 fault register value) */
		0x09, 0x01,                    //   USAGE (Vendor Usage 1)
		0x85, 0x08,               	   //   REPORT_ID (8)
		0x95, 0x05,                    //   REPORT_COUNT (5)
		0x75, 0x08,                    //   REPORT_SIZE (8)
		0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
		0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
		0x81, 0x82,                    //   INPUT (Data,Var,Abs)

		/** Command report: (bytes: 0 - report ID (0x09), 1 - BQ25601 charger enabled state (0 - charger disabled, 1 - charger enabled),
		 * 2 - DS2782 EEPROM block 1 lock (1 - lock memory block), 3 - read EEPROM block 1 lock status, 4 - read BQ25601 status data),
		 * 5 - load 66 mA ctrl (0 - disabled, 1 - enabled) */
		0x09, 0x01,                    //   USAGE (Vendor Usage 1)
		0x85, 0x09,               	   //   REPORT_ID (9)
		0x95, 0x05,                    //   REPORT_COUNT (5)
		0x75, 0x08,                    //   REPORT_SIZE (8)
		0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
		0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
		0x91, 0x82,                    //   OUTPUT (Data,Var,Abs)
  /* USER CODE END 0 */
  0xC0    /*     END_COLLECTION	             */
};

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Exported_Variables USBD_CUSTOM_HID_Exported_Variables
  * @brief Public variables.
  * @{
  */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */
/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_FunctionPrototypes USBD_CUSTOM_HID_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CUSTOM_HID_Init_FS(void);
static int8_t CUSTOM_HID_DeInit_FS(void);
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state);

/**
  * @}
  */

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS =
{
  CUSTOM_HID_ReportDesc_FS,
  CUSTOM_HID_Init_FS,
  CUSTOM_HID_DeInit_FS,
  CUSTOM_HID_OutEvent_FS
};

/** @defgroup USBD_CUSTOM_HID_Private_Functions USBD_CUSTOM_HID_Private_Functions
  * @brief Private functions.
  * @{
  */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_Init_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  DeInitializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_DeInit_FS(void)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Send the report to the Host
  * @param  report: The report to be sent
  * @param  len: The report length
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */

uint8_t USBD_CUSTOM_HID_SendReport_FS(uint8_t *report, uint16_t len)
{
	uint8_t report_data[33] = {0};
	memcpy(report_data, report, len);
	return USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report_data, sizeof(report_data));
}


/**
  * @brief  Manage the CUSTOM HID class events
  * @param  event_idx: Event index
  * @param  state: Event state
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state)
{
  /* USER CODE BEGIN 6 */
	USBD_CUSTOM_HID_HandleTypeDef  *hhid = (USBD_CUSTOM_HID_HandleTypeDef*)hUsbDeviceFS.pClassData;
	commands_queue->Insert(hhid->Report_buf);
	return (USBD_OK);
  /* USER CODE END 6 */
}

/* USER CODE BEGIN 7 */

/* USER CODE END 7 */

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

