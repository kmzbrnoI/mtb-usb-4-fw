#pragma once

#include "stm32.h"
#include "usb.h"

#define CDC_EP0_SIZE 0x08
#define CDC_MAIN_RXD_EP 0x01
#define CDC_MAIN_TXD_EP 0x81
#define CDC_MAIN_NTF_EP 0x82
#define CDC_DEBUG_RXD_EP 0x02
#define CDC_DEBUG_TXD_EP 0x83
#define CDC_DEBUG_NTF_EP 0x84

#define CDC_DATA_SZ 0x40
#define CDC_NTF_SZ 0x08
#define CDC_MTBUSB_BUF_SIZE 0x80

extern void (*cdc_main_received)(uint8_t command_code, uint8_t *data, size_t data_size);

void cdc_init();
bool cdc_is_debug_ep_enabled();
bool cdc_main_can_send(void);
bool cdc_main_send(uint8_t command_code, uint8_t *data, size_t datasize);

void cdc_send_ack(void);
void cdc_send_error(uint8_t error_code, uint8_t command_code, uint8_t module);

#define MTBUSB_CMD_PM_FORWARD 0x10
#define MTBUSB_CMD_PM_INFO_REQ 0x20
#define MTBUSB_CMD_PM_CHANGE_SPEED 0x21
#define MTBUSB_CMD_PM_ACTIVE_MODULES_REQ 0x22

#define MTBUSB_CMD_MP_ACK 0x01
#define MTBUSB_CMD_MP_ERROR 0x02
#define MTBUSB_CMD_MP_FORWARD 0x10
#define MTBUSB_CMD_MP_INFO 0x20
#define MTBUSB_CMD_MP_ACTIVE_MODULES_LIST 0x22
#define MTBUSB_CMD_MP_NEW_MODULE 0x23
#define MTBUSB_CMD_MP_MODULE_FAILED 0x24
