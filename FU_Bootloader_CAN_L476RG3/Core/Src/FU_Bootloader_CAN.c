/*
 * OTA_Bootloader.c
 *
 *  Created on: Feb 23, 2024
 *      Author: Sagar
 */

#include "FU_Bootloader_CAN.h"

uint16_t received_pckt_len;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
uint8_t TxData[8];
uint32_t TxMailbox;
uint8_t rxCount;
bool CANpcktReceived = 0;

CAN_Ext_ID receivedExtID;
FW_mData_t receivedFW_mData;

static uint32_t pcktChunkNumber;
static uint32_t totalChunkReceived;
static uint32_t prevWordReceived;

/* Buffer to hold the received data */

FU_STAT FU_stat = FU_STAT_Idle;

/* static function prototypes */
static void send_resp(uint8_t type);
static HAL_StatusTypeDef erase_flash_pages(uint32_t startAddress,
		uint32_t endAddress);
static HAL_StatusTypeDef write_to_flash(uint32_t address, uint64_t Data);
static void process_packet();
void handle_packet_type(uint8_t packetType, uint8_t subType);
void handle_command_type_packet(uint8_t CMD);
void handle_metaData_type_packet(uint8_t subType);
void handle_response_type_packet();
void handle_fwDATA_type_packet();

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	CANpcktReceived = 1;
	rxCount ++;
	process_packet();
	CANpcktReceived = 0;

}

void OTA_update() {
	HAL_CAN_Start(&hcan1);  //start can
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //activate rx fifo0 full notification
	while (FU_stat != FU_STAT_completed) {
		if (CANpcktReceived) {
			process_packet();
		}
	}
}

void process_packet() {
	memcpy(&receivedExtID, &RxHeader.ExtId, sizeof(CAN_Ext_ID));
	if (receivedExtID.receiverAddress == myCanAddress) {
		// handle message with correct CAN address here
		handle_packet_type(receivedExtID.packet_type,
				receivedExtID.packet_subType);
	}
}

void handle_packet_type(uint8_t packetType, uint8_t subType) {
	switch (packetType) {
	case (OTA_PCKT_TYPE_CMD):
		handle_command_type_packet(RxData[0]);
		break;
	case (OTA_PCKT_TYPE_mData):
		handle_metaData_type_packet(subType);
		break;
	case (OTA_PCKT_TYPE_RESPONSE):
//		handle_response_type_packet();
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
//		send_resp(OTA_PCKT_NACK);
		break;
	}
}

void handle_metaData_type_packet(uint8_t subType) {
	FU_stat = FU_STAT_mData_received;
	switch (subType) {
	case (subType_version):
		receivedFW_mData.FW_version = RxData[0];
		send_resp(OTA_PCKT_ACK);
		break;
	case (subType_size):
		memcpy(&receivedFW_mData.FW_size, &RxData,
				sizeof(receivedFW_mData.FW_size));
		send_resp(OTA_PCKT_ACK);
		break;
	case (subType_chunkCount):
		memcpy(&receivedFW_mData.FW_chunkCount_total, &RxData,
				sizeof(receivedFW_mData.FW_chunkCount_total));
		send_resp(OTA_PCKT_ACK);
		break;
	case (subType_crc):
		memcpy(&receivedFW_mData.FW_crc, &RxData,
				sizeof(receivedFW_mData.FW_crc));
		send_resp(OTA_PCKT_ACK);
		break;
	default:
		send_resp(OTA_PCKT_NACK);
		break;
	}
}

void handle_fwDATA_type_packet() {
	FU_stat = FU_STAT_FWdownload_started;
	uint32_t receivedWord = 0;
	memcpy(&pcktChunkNumber, &RxData[0], 4);
	memcpy(&receivedWord, &RxData[4], 4);
	volatile uint64_t Data64 = ((uint64_t)receivedWord << 32) | prevWordReceived;

	//erase flash on first chunk received
	if (pcktChunkNumber == 0) {
		if(erase_flash_pages(APP1_FLASH_ADDR,
		APP1_FLASH_ADDR + receivedFW_mData.FW_size) != HAL_OK){
			send_resp(OTA_PCKT_NACK);
		}
	}
	if(pcktChunkNumber % 2 == 1){
		if (write_to_flash(APP1_FLASH_ADDR + (totalChunkReceived-1) * 4, Data64)
					== HAL_OK) {
				send_resp(OTA_PCKT_ACK);
				prevWordReceived = 0;
			} else {
				send_resp(OTA_PCKT_NACK);
			}
	}
	else{
		prevWordReceived = receivedWord;
		send_resp(OTA_PCKT_ACK);
	}
	if(totalChunkReceived+1 == receivedFW_mData.FW_chunkCount_total)FU_stat = FU_STAT_completed;
	totalChunkReceived++;
}

uint32_t packCAN_Ext_ID(CAN_Ext_ID id) {
	uint32_t packedID = 0;
	packedID |= (uint32_t) id.senderAddress;
	packedID |= (uint32_t) id.receiverAddress << 8;
	packedID |= (uint32_t) id.packet_type << 16;
	return packedID;
}

static void send_resp(uint8_t type) {
	CAN_Ext_ID txPCKT_extID = { .senderAddress = myCanAddress,
			.receiverAddress = receivedExtID.senderAddress, .packet_type =
					OTA_PCKT_TYPE_RESPONSE };
	TxHeader.ExtId = packCAN_Ext_ID(txPCKT_extID);
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.DLC = sizeof(type);
	TxHeader.RTR = CAN_RTR_DATA;
	TxData[0] = type;
	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

uint32_t GetPage(uint32_t Addr) {
	uint32_t page = 0;
	page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;

	return page;
}

static HAL_StatusTypeDef erase_flash_pages(uint32_t startAddress,
		uint32_t endAddress) {
	HAL_StatusTypeDef ret = HAL_OK;

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	uint32_t StartPage = GetPage(APP1_FLASH_ADDR);
	uint32_t EndPageAdress = APP1_FLASH_ADDR + receivedFW_mData.FW_size;
	uint32_t EndPage = GetPage(EndPageAdress);

	for(int page = StartPage; page <= EndPage; page ++){
		HAL_FLASH_Unlock();
		FLASH_PageErase(page & 0xFF, (page & 0x100) == 0 ? FLASH_BANK_1 : FLASH_BANK_2);
		FLASH_WaitForLastOperation((uint32_t) FLASH_TIMEOUT_VALUE);
		CLEAR_BIT(FLASH->CR, (FLASH_CR_PER | FLASH_CR_PNB));
		HAL_FLASH_Lock();
	}
	HAL_FLASH_Lock();
	return ret;
}



static HAL_StatusTypeDef write_to_flash(uint32_t address, uint64_t Data) {
	HAL_StatusTypeDef ret = HAL_OK;
	HAL_FLASH_Unlock();
	ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, Data);

	HAL_FLASH_Lock();
	return ret;
}
