#pragma once

#include <stdbool.h>

#define MODULE_ATTEMPTS_MAX 4

void modules_init(void);
bool module_active(uint8_t addr);
void module_set_active(uint8_t addr, bool state);

void module_dec_attempts(uint8_t addr);
void module_reset_attempts(uint8_t addr);
size_t module_get_attempts(uint8_t addr);
void module_set_attempts(uint8_t addr, size_t value);