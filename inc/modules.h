#pragma once

#include <stdbool.h>

#define MODULE_ATTEMPTS_MAX 4

void modules_init(void);
bool module_active(uint8_t addr);
void module_set_active(uint8_t addr, bool state);

size_t module_dec_attempts(uint8_t addr);
void module_reset_attempts(uint8_t addr);
size_t module_get_attempts(uint8_t addr);
void module_set_attempts(uint8_t addr, size_t value);

// Returns 0 in case when no activa address in range addr+1–255 found.
uint8_t module_next_active_addr(uint8_t addr);
