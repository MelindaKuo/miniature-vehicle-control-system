#pragma once

#include <stdint.h>


bool faultStoreInit();


uint16_t faultStoreLoad();


void faultStoreSave(uint16_t latchedFaults);


uint32_t faultStoreWriteCount();
