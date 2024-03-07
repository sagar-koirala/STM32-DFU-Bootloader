/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdbool.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define myCanAddress 0xAF
#define destinationMCUaddress 0xAB    // uint8_t device address of the destination MCU
#define responseWaitTime 5000
uint32_t flashData;
uint32_t appStoredAddress = 0x08140000;

typedef struct {
	uint8_t senderAddress;
	uint8_t receiverAddress;
	uint8_t packet_type;
	uint8_t packet_subType;  // only 5 bits will be used
} CAN_Ext_ID;

CAN_Ext_ID rxPCKT_ExtID;

typedef struct {
	uint8_t FW_version;
	uint32_t FW_size;
	uint16_t FW_chunkCount_total;
	uint32_t FW_crc;
} FW_mData_t;

FW_mData_t FW_mData_h = { 1, 6096, 1524, 3906866583 };

typedef enum {
	subType_version, subType_size, subType_chunkCount, subType_crc,
} FW_mData_subType;

typedef enum {
	OTA_PCKT_TYPE_CMD,      // Command
	OTA_PCKT_TYPE_fwDATA,   // Data
	OTA_PCKT_TYPE_mData,    // Header
	OTA_PCKT_TYPE_RESPONSE, // Response
} OTA_PCKT_TYPE;

typedef enum {
	OTA_CMD_START = 0, // OTA Start command
	OTA_CMD_END = 1,   // OTA End command
	OTA_CMD_ABORT = 2, // OTA Abort command
} OTA_CMD;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OTA_PCKT_ACK 0x00 // ACK
#define OTA_PCKT_NACK 0x01 // NACK

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t RxData[8];
uint8_t TxData[8] = { 0 };
uint32_t TxMailbox;

bool ackReceived = 0;
bool nackReceived = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}

void process_packet();
void handle_packet_type(uint8_t firmwareUpdatePacketType);
uint32_t packCAN_Ext_ID(CAN_Ext_ID id);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	process_packet();
}

uint32_t packCAN_Ext_ID(CAN_Ext_ID id) {
	uint32_t packedID = 0;
	packedID |= (uint32_t) id.senderAddress;
	packedID |= (uint32_t) id.receiverAddress << 8;
	packedID |= (uint32_t) id.packet_type << 16;
	packedID |= (uint32_t) id.packet_subType << 24;
	return packedID;
}

uint32_t htonl(uint32_t net) {
	return __builtin_bswap32(net);
}

void process_packet() {
	memcpy(&rxPCKT_ExtID, &RxHeader.ExtId, sizeof(CAN_Ext_ID));
	if (rxPCKT_ExtID.receiverAddress == myCanAddress) {
		// handle message with correct CAN address here
		handle_packet_type(rxPCKT_ExtID.packet_type);
	}
}

void handle_packet_type(uint8_t firmwareUpdatePacketType) {
	switch (firmwareUpdatePacketType) {
	case (OTA_PCKT_TYPE_CMD):
		// Handle cmd type data
		break;
	case (OTA_PCKT_TYPE_mData):
		// handle_metaData_type_packet
		break;
	case (OTA_PCKT_TYPE_RESPONSE):
		// handle_response_type_packet
		if (RxData[0] == OTA_PCKT_ACK)
			ackReceived = 1;
		else if (RxData[0] == OTA_PCKT_NACK)
			nackReceived = 1;
		break;
	case (OTA_PCKT_TYPE_fwDATA):
		// handle_fwDATA_type_packet
		break;
	default:
		// handle incorrect packet type here
		break;
	}
}
void send_cmd(uint8_t receiverAddress, uint8_t cmd) {
	CAN_Ext_ID txPCKT_extID =
			{ .senderAddress = myCanAddress, .receiverAddress = receiverAddress,
					.packet_type = OTA_PCKT_TYPE_CMD };
	TxHeader.ExtId = packCAN_Ext_ID(txPCKT_extID);
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.DLC = sizeof(cmd);
	TxHeader.RTR = CAN_RTR_DATA;
	TxData[0] = cmd;
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

void send_FWmData(uint8_t receiverAddress, uint8_t pcktType,
		uint8_t pckt_subType, void *data, uint8_t dataSize) {

	CAN_Ext_ID txPCKT_extID = { .senderAddress = myCanAddress,
			.receiverAddress = receiverAddress, .packet_type = pcktType,
			.packet_subType = pckt_subType };

	TxHeader.ExtId = packCAN_Ext_ID(txPCKT_extID);
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.DLC = dataSize;
	TxHeader.RTR = CAN_RTR_DATA;

	uint8_t *byteData = (uint8_t*) data;
	for (int i = 0; i < dataSize; i++) {
		TxData[i] = byteData[i];
	}

	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

void wait_response() {
	uint32_t startTime = HAL_GetTick();
	while (!ackReceived && !nackReceived
			&& HAL_GetTick() - startTime < responseWaitTime)
		;
}

void Flash_Read_Data(uint32_t StartSectorAddress, uint32_t *RxBuf,
		uint16_t numberofwords) {
	while (1) {

		*RxBuf = *(__IO uint32_t*) StartSectorAddress;
		StartSectorAddress += 4;
		RxBuf++;
		if (!(numberofwords--))
			break;
	}
}

void send_FWdata(uint8_t receiverAddress, uint8_t pcktType,
                                  uint8_t pckt_subType, uint32_t chunkNumber,
                                  uint32_t StartSectorAddress) {

    CAN_Ext_ID txPCKT_extID = { .senderAddress = myCanAddress,
                                .receiverAddress = receiverAddress,
                                .packet_type = pcktType,
                                .packet_subType = pckt_subType };

    TxHeader.ExtId = packCAN_Ext_ID(txPCKT_extID);
    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.DLC = 8; // 4 bytes for chunk number and 4 bytes for data
    TxHeader.RTR = CAN_RTR_DATA;

    // Store chunk number in the first 4 bytes of TxData
    for (int i = 0; i < 4; i++) {
        TxData[i] = (chunkNumber >> (8 * i)) & 0xFF;
    }

    // Read data from flash
    Flash_Read_Data(StartSectorAddress, &flashData, 1);

    // Store flash data in the last 4 bytes of TxData
    for (int i = 0; i < 4; i++) {
        TxData[4 + i] = (flashData >> (8 * i)) & 0xFF;
    }

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
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
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
	printf("Application Started..\n ");
	HAL_CAN_Start(&hcan1);  //start can
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //activate rx fifo0 full notification
	HAL_GPIO_TogglePin(blueLED_GPIO_Port, blueLED_Pin);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		do {
			send_cmd(destinationMCUaddress, OTA_CMD_START);
			wait_response();
		} while (!ackReceived);
		ackReceived = 0;
		HAL_GPIO_TogglePin(blueLED_GPIO_Port, blueLED_Pin);
		HAL_Delay(100);
		do {
			send_FWmData(destinationMCUaddress, OTA_PCKT_TYPE_mData,
					subType_version, &FW_mData_h.FW_version,
					sizeof(FW_mData_h.FW_version));
			wait_response();
		} while (!ackReceived);
		ackReceived = 0;
		HAL_GPIO_TogglePin(blueLED_GPIO_Port, blueLED_Pin);
		HAL_Delay(100);
		do {
			send_FWmData(destinationMCUaddress, OTA_PCKT_TYPE_mData,
					subType_size, &FW_mData_h.FW_size,
					sizeof(FW_mData_h.FW_size));
			wait_response();
		} while (!ackReceived);
		ackReceived = 0;
		HAL_GPIO_TogglePin(blueLED_GPIO_Port, blueLED_Pin);
		HAL_Delay(100);
		do {
			send_FWmData(destinationMCUaddress, OTA_PCKT_TYPE_mData,
					subType_chunkCount, &FW_mData_h.FW_chunkCount_total,
					sizeof(FW_mData_h.FW_chunkCount_total));
			wait_response();
		} while (!ackReceived);
		ackReceived = 0;
		HAL_GPIO_TogglePin(blueLED_GPIO_Port, blueLED_Pin);
		do {
			send_FWmData(destinationMCUaddress, OTA_PCKT_TYPE_mData,
					subType_crc, &FW_mData_h.FW_crc,
					sizeof(FW_mData_h.FW_crc));
			wait_response();
		} while (!ackReceived);
		ackReceived = 0;
		HAL_GPIO_TogglePin(blueLED_GPIO_Port, blueLED_Pin);
		HAL_Delay(100);
		for (int chunkNumber = 0; chunkNumber < FW_mData_h.FW_chunkCount_total; chunkNumber++) {
			do {
				send_FWdata(destinationMCUaddress, OTA_PCKT_TYPE_fwDATA, 0x00, chunkNumber, appStoredAddress+(chunkNumber*4));
				wait_response();
			} while (!ackReceived);
			ackReceived = 0;
			HAL_GPIO_TogglePin(blueLED_GPIO_Port, blueLED_Pin);
		}
		HAL_GPIO_WritePin(greenLED_GPIO_Port, greenLED_Pin, 1);
		HAL_GPIO_WritePin(blueLED_GPIO_Port, blueLED_Pin, 1);
		HAL_GPIO_WritePin(redLed_GPIO_Port, redLed_Pin, 1);
		while(1); // process complete
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hcan1.Init.Prescaler = 1;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_14TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 10; // which filter bank to use from the assigned ones
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterIdHigh = 0;  //will just receive 0x103 address mssg
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = 0;
	canfilterconfig.FilterMaskIdLow = 0;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 0; // how many filters to assign to the CAN1 (master can)

	HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
  /* USER CODE END CAN1_Init 2 */

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
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, greenLED_Pin|redLed_Pin|blueLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : usr_btn_Pin */
  GPIO_InitStruct.Pin = usr_btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(usr_btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : greenLED_Pin redLed_Pin blueLED_Pin */
  GPIO_InitStruct.Pin = greenLED_Pin|redLed_Pin|blueLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
	while (1) {
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
