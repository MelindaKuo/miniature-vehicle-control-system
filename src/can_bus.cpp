
//twai CAN

#include <Arduino.h>
#include "can_bus.h"
#include "pins.h"
#include <cstring>

bool canBusInit() {

    twai_general_config_t gConfig = TWAI_GENERAL_CONFIG_DEFAULT(PIN_TWAI_TX, PIN_TWAI_RX, TWAI_MODE_NO_ACK);
    twai_timing_config_t tConfig = TWAI_TIMING_CONFIG_500KBITS();

    twai_filter_config_t fConfig = {};
    fConfig.single_filter = true;
    fConfig.acceptance_code = (uint32_t)0b001 << 29;
    uint32_t careBits = (uint32_t)0b111 << 29;
    fConfig.acceptance_mask = ~careBits;

    if(twai_driver_install (&gConfig, &tConfig, &fConfig) != ESP_OK) {
        return false;
    }

    if(twai_start() != ESP_OK){
        return false;
    }

    return true;
}


bool canSend(uint32_t id, const uint8_t* data, uint8_t dlc) {

    twai_message_t msg = {0};
    msg.identifier = id;
    msg.data_length_code = dlc;
    msg.self = 1;  //...

    memcpy(msg.data, data, dlc);

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(40));
    bool ok = err == ESP_OK;


    Serial.printf("CANTX,%lu,%03X,%d", millis(), id, ok ? 1 : 0);
    for (uint8_t i = 0; i < dlc; i++) {
        Serial.printf(",%02X", data[i]);
    }
    Serial.println();

    return ok;

}

bool canReceive(twai_message_t* outMsg, uint32_t timeoutMs) {
    esp_err_t err = twai_receive(outMsg, pdMS_TO_TICKS(timeoutMs));

    return err == ESP_OK;
}

void canGetStatus(twai_status_info_t* outStatus) {
    twai_get_status_info(outStatus);
}
