/*
 * OTA_Bootloader.h
 *
 *  Created on: Feb 15, 2024
 *      Author: Sagar
 */

#ifndef INC_FU_BOOTLOADER_UART_H_
#define INC_FU_BOOTLOADER_UART_H_

#include "main.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stm32f7xx_hal.h>

extern CRC_HandleTypeDef hcrc;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
// Constants
#define OTA_PCKT_SOF 0xAA // Start of Frame
#define OTA_PCKT_EOF 0xBB // End of frame
#define OTA_PCKT_ACK 0x00 // ACK
#define OTA_PCKT_NACK 0x01 // NACK

#define APP1_FLASH_ADDR 0x08140000 // First App Flash Address
#define APP2_FLASH_ADDR 0x08040000 // Second App flash address

#define FW_DATA_MAX_SIZE 1024 // Maximum Firmware Size
#define FW_DATA_OVERHEAD 9 // Data overhead
#define OTA_PCKT_MAX_SIZE (FW_DATA_MAX_SIZE + FW_DATA_OVERHEAD)

// Enumerations
typedef enum {
    APP1,
    APP2
} FW_version;

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

// Data Structures
#pragma pack(push, 1)
typedef struct {
    uint8_t FW_version;
    uint32_t FW_size;
    uint16_t FW_chunkCount_total;
    uint32_t FW_crc;
} FW_mData_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint8_t sof;
    uint8_t packet_type;
    uint16_t data_len;
    uint8_t status;
    uint32_t crc;
    uint8_t eof;
} OTA_RESP;
#pragma pack(pop)

// Global Function Prototypes
void OTA_update();
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);


#endif /* INC_FU_BOOTLOADER_UART_H_ */
