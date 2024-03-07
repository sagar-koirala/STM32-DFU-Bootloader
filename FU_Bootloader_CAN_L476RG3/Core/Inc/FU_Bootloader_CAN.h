/*
 * OTA_Bootloader.h
 *
 *  Created on: Feb 23, 2024
 *      Author: Sagar
 */

#ifndef INC_FU_BOOTLOADER_CAN_H_
#define INC_FU_BOOTLOADER_CAN_H_

#include "main.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "stm32l4xx_hal.h"

#define myCanAddress 0xAB

extern CAN_HandleTypeDef hcan1;

// Constants
#define OTA_PCKT_SOF 0xAA // Start of Frame
#define OTA_PCKT_EOF 0xBB // End of frame
#define OTA_PCKT_ACK 0x00 // ACK
#define OTA_PCKT_NACK 0x01 // NACK

#define APP1_FLASH_ADDR 0x08019000 // First App Flash Address
#define APP2_FLASH_ADDR 0x08087000 // Second App flash address

#define FW_DATA_MAX_SIZE 1024 // Maximum Firmware Size
#define FW_DATA_OVERHEAD 9 // Data overhead
#define OTA_PCKT_MAX_SIZE (FW_DATA_MAX_SIZE + FW_DATA_OVERHEAD)

typedef enum {
    FU_STAT_Idle,
    FU_STAT_startCMD_received,
    FU_STAT_mData_received,
    FU_STAT_FWdownload_started,
    FU_STAT_FWdownload_aborted,
    FU_STAT_FWdownload_completed,
    FU_STAT_completed,
} FU_STAT;

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

#pragma pack(push, 1)
typedef struct {
	uint8_t FW_version;
	uint32_t FW_size;
	uint16_t FW_chunkCount_total;
	uint32_t FW_crc;
} FW_mData_t;
#pragma pack(pop)

typedef enum {
	subType_version, subType_size, subType_chunkCount, subType_crc,
} FW_mData_subType;

// 29 bit extended CAN ID
#pragma pack(push, 1)
typedef struct {
	uint8_t senderAddress;
    uint8_t receiverAddress;
    uint8_t packet_type;
    uint8_t packet_subType;  // only 5 bits are used
} CAN_Ext_ID;
#pragma pack(pop)

// Global Function Prototypes
void OTA_update();
uint32_t packCAN_Ext_ID(CAN_Ext_ID id);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);


#endif /* INC_FU_BOOTLOADER_CAN_H_ */
