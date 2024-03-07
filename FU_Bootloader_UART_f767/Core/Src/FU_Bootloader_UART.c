/*
 * OTA_Bootloader.c
 *
 *  Created on: Feb 15, 2024
 *      Author: Sagar
 */

#include <FU_Bootloader_UART.h>

uint16_t received_pckt_len;

static uint32_t nFW_received_bytes;
static uint16_t nFW_received_chunk;

uint16_t firmwareUpdatePacketDataLength;

bool bChunkMismatch = 0;
/* Buffer to hold the received data */
extern uint8_t Rx_Buffer[OTA_PCKT_MAX_SIZE];

extern uint8_t appMdata[200];

FU_STAT FU_stat = FU_STAT_Idle;
FW_version curentApp;

/* static function prototypes */
static void send_resp(uint8_t type);
static HAL_StatusTypeDef unlock_flash();
static HAL_StatusTypeDef lock_flash();
static HAL_StatusTypeDef erase_flash_sectors(uint32_t sector,
		uint32_t num_sectors);
//static HAL_StatusTypeDef write_to_flash(uint8_t *data, uint16_t data_len);
static HAL_StatusTypeDef write_to_flash(uint32_t flash_address, uint8_t *data,
		uint16_t data_len);
//static void deinit_peripherals();
static void process_packet();
void handle_packet_type(uint8_t firmwareUpdatePacketType);
void handle_command_type_packet(uint8_t CMD);
void handle_metaData_type_packet();
void handle_response_type_packet();
void handle_fwDATA_type_packet();

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == USART2) {
		received_pckt_len = Size;
	}
	HAL_GPIO_TogglePin(blueLED_GPIO_Port, blueLED_Pin);
}

void OTA_update() {
	memset(Rx_Buffer, 0, OTA_PCKT_MAX_SIZE);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Rx_Buffer, OTA_PCKT_MAX_SIZE); // ENABLE UART2 in DMA mode
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	while (FU_stat != FU_STAT_completed) {
		// check if packet data is received
		if (received_pckt_len > 0) {
			process_packet();
			received_pckt_len = 0;
			memset(Rx_Buffer, 0, OTA_PCKT_MAX_SIZE);
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Rx_Buffer, OTA_PCKT_MAX_SIZE); // ENABLE UART2 in DMA mode
			__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
		}
	}
	HAL_GPIO_WritePin(blueLED_GPIO_Port, blueLED_Pin, 0);
}

static void process_packet() {
	const bool isFirmwareUpdatePacket = Rx_Buffer[0] == OTA_PCKT_SOF
			&& Rx_Buffer[received_pckt_len - 1] == OTA_PCKT_EOF;
	firmwareUpdatePacketDataLength = ((uint16_t) Rx_Buffer[3] << 8)
			| Rx_Buffer[2];
	const uint32_t firmwareUpdatePacketCRC =
			((uint32_t) Rx_Buffer[received_pckt_len - 2] << 24)
					| (Rx_Buffer[received_pckt_len - 3] << 16)
					| (Rx_Buffer[received_pckt_len - 4] << 8)
					| Rx_Buffer[received_pckt_len - 5];
	const uint32_t calculatedCRC = HAL_CRC_Calculate(&hcrc,
			(uint32_t*) &Rx_Buffer[received_pckt_len
					- firmwareUpdatePacketDataLength - 5],
			firmwareUpdatePacketDataLength);
	const bool isCRCcorrect = firmwareUpdatePacketCRC == calculatedCRC;

	const uint8_t firmwareUpdatePacketType = Rx_Buffer[1];

	if (!isFirmwareUpdatePacket || !isCRCcorrect) {
		send_resp(OTA_PCKT_NACK);
	} else {
		handle_packet_type(firmwareUpdatePacketType);
	}
}

void handle_packet_type(uint8_t firmwareUpdatePacketType) {
	switch (firmwareUpdatePacketType) {
	case (OTA_PCKT_TYPE_CMD):
		handle_command_type_packet(Rx_Buffer[4]);
		break;
	case (OTA_PCKT_TYPE_mData):
		handle_metaData_type_packet();
		break;
	case (OTA_PCKT_TYPE_RESPONSE):
		handle_response_type_packet();
		break;
	case (OTA_PCKT_TYPE_fwDATA):
		handle_fwDATA_type_packet();
		break;
	default:
		send_resp(OTA_PCKT_NACK);
		break;
	}
}

void handle_command_type_packet(uint8_t CMD) {
	switch (CMD) {
	case (OTA_CMD_START):
		FU_stat = FU_STAT_startCMD_received;
		send_resp(OTA_PCKT_ACK);
		break;
	case (OTA_CMD_END):
		// handle end command here
		break;
	default:
		send_resp(OTA_PCKT_NACK);
		break;
	}
}

void handle_metaData_type_packet() {
	FU_stat = FU_STAT_mData_received;
	FW_mData_t FW_mData_ph;
	memcpy(&FW_mData_ph, &Rx_Buffer[4], sizeof(FW_mData_t));
	send_resp(OTA_PCKT_ACK);
}

void handle_response_type_packet() {
	// handle response type packet here
}

void handle_fwDATA_type_packet() {
	FU_stat = FU_STAT_FWdownload_started;
//	bool bChunkMismatch = 0;
	if ((((uint16_t) Rx_Buffer[5] << 8) | Rx_Buffer[4])
			!= (nFW_received_chunk + 1)) {
		bChunkMismatch = 1;
		send_resp(OTA_PCKT_NACK);
	} else {
		nFW_received_chunk = ((uint16_t) Rx_Buffer[5] << 8) | Rx_Buffer[4];
		bool writeSuccess;
		do {
			writeSuccess = unlock_flash();
			if (writeSuccess != HAL_OK)
				break;

			if (nFW_received_bytes == 0) {
				writeSuccess = erase_flash_sectors(9U, 2);
			}

			if (writeSuccess != HAL_OK)
				break;

			writeSuccess = write_to_flash(
			APP1_FLASH_ADDR + nFW_received_bytes, &Rx_Buffer[6],
					firmwareUpdatePacketDataLength);
			if (writeSuccess != HAL_OK)
				break;
			else {
				nFW_received_bytes += firmwareUpdatePacketDataLength;
			}
			writeSuccess = lock_flash();
		} while (false);
		if (writeSuccess != HAL_OK) {
			send_resp(OTA_PCKT_NACK);
		} else {
			send_resp(OTA_PCKT_ACK);
		}
	}
}

static void send_resp(uint8_t type) {
	OTA_RESP rsp = { .sof = OTA_PCKT_SOF, .packet_type = OTA_PCKT_TYPE_RESPONSE,
			.data_len = 1u, .status = type, .eof =
			OTA_PCKT_EOF };
	rsp.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) &rsp.status, 1);
	//send response
	HAL_UART_Transmit(&huart2, (uint8_t*) &rsp, sizeof(OTA_RESP), 100);
}

//static void deinit_peripherals() {
//	// Di-initialize all the peripherals or reset to default state
//	// Reverse order as they are initialized in main file
//	HAL_CRC_DeInit(&hcrc);
//	HAL_UART_DeInit(&huart2);
//	HAL_DMA_DeInit(&hdma_usart2_rx);
//	HAL_GPIO_DeInit(greenLED_GPIO_Port, greenLED_Pin);
//	HAL_RCC_DeInit();
//	HAL_DeInit();
//	SysTick->CTRL = 0;
//	SysTick->LOAD = 0;
//	SysTick->VAL = 0;
//}

static HAL_StatusTypeDef unlock_flash() {
	HAL_StatusTypeDef ret = HAL_FLASH_Unlock();
	return ret;
}

static HAL_StatusTypeDef lock_flash() {
	HAL_StatusTypeDef ret = HAL_FLASH_Lock();
	return ret;
}

static HAL_StatusTypeDef erase_flash_sectors(uint32_t sector,
		uint32_t num_sectors) {
	HAL_StatusTypeDef ret = HAL_OK;

	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SectorError;

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.Sector = sector;
	EraseInitStruct.NbSectors = num_sectors;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	ret = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);

	return ret;
}

// Write data to flash at a specified address
static HAL_StatusTypeDef write_to_flash(uint32_t flash_address, uint8_t *data,
		uint16_t data_len) {
	HAL_StatusTypeDef ret = HAL_OK;

	for (int i = 0; i < data_len; i++) {
		ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, flash_address + i,
				data[i]);
		if (ret != HAL_OK) {
			break;
		}
	}

	return ret;
}
