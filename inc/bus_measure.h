/* Measuring of voltage & current through RS485 driver (ADM2483) via
   INA219 and galvanically-separated I2C. */

#pragma once

#include <stdbool.h>

bool bus_measure_init(void);
