//sensor node
#include <Arduino.h>


#include "dim.h"
#include "pins.h"
#include "project_config.h"
#include "can_dictionary.h"
#include "can_bus.h"
#include "mpu6050.h"


static DimState s_dimState = DimState::BOOT;


static uint8_t s_counterPedal     = 0;
static uint8_t s_counterImu       = 0;
static uint8_t s_counterHeartbeat = 0;

static bool s_dimLastBusOff = false;


static volatile uint32_t s_brakeEdgeCount = 0;
static volatile uint32_t s_startEdgeCount = 0;

static volatile bool s_brakeDebounced = false; 
static volatile uint32_t s_lastBrakeTime = 0; 


static volatile bool s_startDebounced = false;
static volatile uint32_t s_lastStartTime = 0; 


static void IRAM_ATTR onBrakeChange() {

    s_brakeEdgeCount++;

    uint32_t timeNow = millis(); 

    if(timeNow - s_lastBrakeTime >= 20){
        s_brakeDebounced = (digitalRead(PIN_BTN_BRAKE)== LOW);
        s_lastBrakeTime = timeNow;


    }
}

static void IRAM_ATTR onStartChange() {
    s_startEdgeCount++;

    uint32_t timeNow = millis(); 

    if(timeNow - s_lastStartTime >=20){
        s_startDebounced = (digitalRead(PIN_BTN_START) == LOW);
        s_lastStartTime = timeNow; 
    }
}


static void dimPedalTask(void* arg) {
    (void)arg;
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(PERIOD_DIM_PEDAL_MS);

    for (;;) {
        
        uint16_t potA = analogRead(PIN_POT_A);
        uint16_t potB = analogRead(PIN_POT_B);

        uint8_t flags = 0; 

        if(s_brakeDebounced){
            flags |= PedalFlag::BRAKE_PRESSED;
        }

        if(s_startDebounced){
            flags |= PedalFlag::START_PRESSED;
            
        }

        uint8_t bytes[CAN_DLC] = {};

        bytes[0] = potA & 0xFF;
        bytes[1] = (potA >> 8) & 0xFF;
        bytes[2] = potB & 0xFF; 
        bytes[3] = (potB >> 8) & 0xFF; 
        bytes[4] = flags; 

        bytes[OFF_COUNTER] = s_counterPedal++; 

        uint8_t sum = 0; 

        for(int i = 0; i< 7; i++){
            sum += bytes[i];
        }

        bytes[OFF_CHECKSUM] = sum; 
        
        canSend(CanId::DIM_PEDAL, bytes, CAN_DLC);

        vTaskDelayUntil(&lastWake, period);
    }
}

static void dimImuTask(void* arg) {
    (void)arg;
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(PERIOD_DIM_IMU_MS);

    for (;;) {
        int16_t ax, ay, az; 

        if(mpu6050ReadAccelRaw(ax, ay, az)){
            s_dimState = DimState::SENSOR_OK;
            int16_t mgX = (int32_t)ax * ImuMsg::MILLI_G_PER_G/16384;
            int16_t mgY = (int32_t)ay * ImuMsg::MILLI_G_PER_G/16384;
            int16_t mgZ = (int32_t)az * ImuMsg::MILLI_G_PER_G/16384;

            
            uint8_t bytes[CAN_DLC] = {};

            bytes[0] = (mgX ) & 0xFF; 
            bytes[1] = (mgX >> 8) & 0xFF;
            bytes[2] = (mgY) & 0xFF; 
            bytes[3] = (mgY >> 8) & 0xFF; 
            bytes[4] = (mgZ ) & 0xFF;
            bytes[5] = (mgZ >> 8) & 0xFF;

            bytes[OFF_COUNTER] = s_counterImu++; 

            uint8_t sum = 0; 

            for(int i = 0; i< 7; i++){
                sum += bytes[i];

            }

            bytes[OFF_CHECKSUM] = sum; 

            canSend(CanId::DIM_IMU, bytes, CAN_DLC);

        }
        else{
            s_dimState = DimState::SENSOR_ERR;
        }

        vTaskDelayUntil(&lastWake, period);
    }
}

static void dimHeartbeatTask(void* arg) {
    (void)arg;
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(PERIOD_DIM_HEARTBEAT_MS);

    for (;;) {

        twai_status_info_t busStatus;
        canGetStatus(&busStatus);

        Serial.printf("DIMBUS,%lu,%d,%lu,%lu,%lu\n", millis(), (int)busStatus.state,
                      busStatus.tx_error_counter, busStatus.rx_error_counter,
                      busStatus.msgs_to_tx);

        if(busStatus.state == TWAI_STATE_BUS_OFF){
            if(!s_dimLastBusOff){
                twai_initiate_recovery();
            }
            s_dimLastBusOff = true;
        }
        else{
            s_dimLastBusOff = false;
        }

        uint32_t time = millis();

        uint8_t bytes[CAN_DLC] = {};

        bytes[0] = time & 0xFF; 
        bytes[1] =  (time >> 8) & 0xFF; 
        bytes[2] = ((time >> 16) & 0xFF); 
        bytes[3] = ((time >> 24) & 0xFF); 

        bytes[4] = (uint8_t)(s_dimState);

        bytes[OFF_COUNTER] = s_counterHeartbeat++; 

        uint8_t sum = 0; 
        for(int i=  0; i< 7; i++){
            sum+= bytes[i]; 
        }

        bytes[OFF_CHECKSUM] = sum; 

        canSend(CanId::DIM_HEARTBEAT, bytes, CAN_DLC);

        vTaskDelayUntil(&lastWake, period);
    }
}


void dimStart() {
    pinMode(PIN_BTN_BRAKE, INPUT_PULLUP);
    pinMode(PIN_BTN_START, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_BTN_BRAKE), onBrakeChange, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_BTN_START), onStartChange, CHANGE);

    s_dimState = DimState::BOOT;

    bool mpuStart = mpu6050Init(); 

    if(mpuStart){
        s_dimState = DimState::SENSOR_OK; 
    }

    else{
        s_dimState = DimState::SENSOR_ERR; 
    }
    BaseType_t okP = xTaskCreatePinnedToCore(dimPedalTask, "dim_pedal",
                            STACK_DIM_PEDAL, nullptr,
                            PRIO_DIM_PEDAL, nullptr, CORE_DIM);

    BaseType_t okI = xTaskCreatePinnedToCore(dimImuTask, "dim_imu",
                            STACK_DIM_IMU, nullptr,
                            PRIO_DIM_IMU, nullptr, CORE_DIM);

    BaseType_t okH = xTaskCreatePinnedToCore(dimHeartbeatTask, "dim_heartbeat",
                            STACK_DIM_HEARTBEAT, nullptr,
                            PRIO_DIM_HEARTBEAT, nullptr, CORE_DIM);

    if(okP != pdPASS){
        Serial.println("DIM Pedal Fail");
    }
    if(okI != pdPASS){
        Serial.println("DIM IMU Fail");
    }
    if(okH != pdPASS){
        Serial.println("DIM Heartbeat Fail");
    }


}
