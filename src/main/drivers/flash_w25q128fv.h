#pragma once

#include "flash_impl.h"

#ifdef USE_FLASH_W25Q128FV

bool w25q128fv_detect(flashDevice_t *fdevice, uint32_t jedecID);

#endif
