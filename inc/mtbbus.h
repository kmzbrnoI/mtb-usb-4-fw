#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool mtbbus_init(void);
void mtbbus_send(uint8_t *data, size_t length);
bool mtbbus_can_send(void);
