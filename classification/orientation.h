#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <stdint.h>
#include "../commons/data_formats.h"
#include "classify.h"

ActionType classify_orientation(BMI2SensAxisData data[], uint32_t length);

#endif