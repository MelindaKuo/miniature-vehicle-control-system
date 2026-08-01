
#include <Arduino.h>

#include "pins.h"
#include "project_config.h"
#include "can_bus.h"
#include "dim.h"
#include "vcu.h"
#include <Wire.h> 

void setup() {
    Serial.begin(115200);
    delay(300);                       
    Serial.println();
    Serial.println(F("=== EV control system: boot ==="));

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    analogReadResolution(12);
    canBusInit(); 


    dimStart();
    vcuStart();

    Serial.println(F("=== tasks started ==="));
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
