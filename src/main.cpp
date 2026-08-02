
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

    canBusInit();


    //potentiometers, MPU6050, buttons
    #ifdef NODE_ROLE_DIM
        Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
        analogReadResolution(12);
        dimStart();

    #endif


    //servo + buzzer

    #ifdef NODE_ROLE_VCU
        vcuStart();
    #endif


    

    


    
    

    Serial.println(F("=== tasks started ==="));
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
