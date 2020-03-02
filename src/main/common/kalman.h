#pragma once

#include "sensors/gyro.h"
#include "filter.h"

extern void kalman_init(void);
extern void kalman_update(float* input, float* output);