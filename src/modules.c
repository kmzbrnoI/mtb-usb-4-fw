#include <stddef.h>
#include <stdint.h>
#include "modules.h"

uint32_t modules_active[8];
uint32_t modules_timeout_counters[32];
uint32_t modules_changed[32];

void modules_init(void) {
	for (size_t i = 0; i < 8; i++)
		modules_active[i] = 0;
	for (size_t i = 0; i < 32; i++)
		modules_timeout_counters[i] = MODULE_ATTEMPTS_MAX | (MODULE_ATTEMPTS_MAX << 4);
}

bool module_active(uint8_t addr) {
	return (modules_active[addr/32] >> (addr%32)) & 0x1;
}

void module_set_active(uint8_t addr, bool state) {
	if (state)
		modules_active[addr/32] |= (1 << (addr%32));
	else
		modules_active[addr/32] &= ~(1 << (addr%32));
}

size_t module_dec_attempts(uint8_t addr) {
	size_t value = module_get_attempts(addr);
	if (value > 0) {
		module_set_attempts(addr, value-1);
		return value-1;
	}
	return 0;
}

void module_reset_attempts(uint8_t addr) {
	module_set_attempts(addr, MODULE_ATTEMPTS_MAX);
}

size_t module_get_attempts(uint8_t addr) {
	return (modules_timeout_counters[addr/8] >> (4*(addr%8))) & 0xF;
}

void module_set_attempts(uint8_t addr, size_t value) {
	if (value > 0xF)
		value = 0xF;

	size_t i = addr/8;
	uint32_t mask = ~(0xF << (4*(addr%8)));
	modules_timeout_counters[i] = (modules_timeout_counters[i] & mask) | (value << (4*(addr%8)));
}

uint8_t module_next_active_addr(uint8_t addr) {
	uint32_t module_active = modules_active[addr/32];
	module_active >>= addr % 32;

	do {
		addr++;
		if (addr == 0)
			return 0;
		if ((addr%32) > 0)
			module_active >>= 1;
		else
			module_active = modules_active[addr/32];
	} while ((module_active&1) == 0);

	return addr;
}

bool module_changed(uint8_t addr) {
	return (modules_changed[addr/32] >> (addr%32)) & 0x1;
}

void module_set_changed(uint8_t addr, bool state) {
	if (state)
		modules_changed[addr/32] |= (1 << (addr%32));
	else
		modules_changed[addr/32] &= ~(1 << (addr%32));
}
