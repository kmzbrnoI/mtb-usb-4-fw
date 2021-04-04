#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool mtbbus_init(void);
bool mtbbus_can_send(void);
bool mtbbus_send(uint8_t addr, uint8_t command_code, uint8_t *data, size_t datalen);

void mtbbus_module_inquiry(uint8_t module_addr);

#define MTBBUS_CMD_MOSI_MODULE_INQUIRY 0x01

#define MTBBUS_CMD_MISO_ACK 0x01
#define MTBBUS_CMD_MISO_ERROR 0x02

#define MTBBUS_ERROR_UNKNOWN_COMMAND 0x01
#define MTBBUS_ERROR_UNSUPPORTED_COMMAND 0x02
#define MTBBUS_ERROR_BAD_ADDRESS 0x03
#define MTBBUS_ERROR_BUSY 0x04
