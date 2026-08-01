#pragma once



#include <stdint.h>
#include <driver/twai.h>


bool canBusInit();

bool canSend(uint32_t id, const uint8_t* data, uint8_t dlc);


bool canReceive(twai_message_t* outMsg, uint32_t timeoutMs);

void canGetStatus(twai_status_info_t* outStatus);
