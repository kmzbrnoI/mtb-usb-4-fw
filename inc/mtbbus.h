/* MTBbus communiation API
 * This library reports events in ‹mtbbus_rx_flags› union. User should poll
 * these flags and reset them when events are processed. Before flags are cleared,
 * ‹mtbbus_can_send› returns false, so no new commands could be sent to MTBbus
 * (so no new events can occur, cause all events are in direct response to
 * some packet sent).
 * This library sets flags in ‹modules.h› and flags too. If user checks for flags,
 * it could read updated ‹modules_*› values.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MTBBUS_OUT_BUF_SIZE 128
#define MTBBUS_IN_BUF_SIZE 128

typedef union {
	size_t all;
	struct {
		bool received :1;
		bool timeout_pc :1;
		bool timeout_inquiry :1;
		bool discovered :1;
	} sep;
} MtbBusRxFlags;

extern MtbBusRxFlags mtbbus_rx_flags;
extern uint16_t mtbbus_received_data[MTBBUS_IN_BUF_SIZE];
extern size_t mtbbus_addr;
extern size_t mtbbus_command_code;


bool mtbbus_init(void);
bool mtbbus_can_send(void);
bool mtbbus_send(uint8_t addr, uint8_t command_code, uint8_t *data, size_t datalen);
void mtbbus_update_50us(void);

void mtbbus_module_inquiry(uint8_t module_addr);
void mtbbus_modules_inquiry(void);

#define MTBBUS_CMD_MOSI_MODULE_INQUIRY 0x01

#define MTBBUS_CMD_MISO_ACK 0x01
#define MTBBUS_CMD_MISO_ERROR 0x02

#define MTBBUS_ERROR_UNKNOWN_COMMAND 0x01
#define MTBBUS_ERROR_UNSUPPORTED_COMMAND 0x02
#define MTBBUS_ERROR_BAD_ADDRESS 0x03
#define MTBBUS_ERROR_BUSY 0x04
