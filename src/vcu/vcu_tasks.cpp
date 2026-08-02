#include <Arduino.h>
#include <esp_task_wdt.h>

#include "vcu.h"
#include "pins.h"
#include "project_config.h"
#include "can_dictionary.h"
#include "can_bus.h"
#include "fault_store.h"


static QueueHandle_t s_rxQueue = nullptr;

static uint8_t s_counterStatus = 0;
static uint8_t s_counterFault  = 0;
static int MAX_STEP = 7; 


static uint16_t s_potA_raw    = 0;
static uint16_t s_potB_raw    = 0;
static uint8_t  s_buttonFlags = 0;
static int16_t  s_accelX_mg   = 0;
static int16_t  s_accelY_mg   = 0;
static int16_t  s_accelZ_mg   = 0;
static uint32_t s_uptime = 0;
static uint8_t s_dimState = 0;   


static uint8_t s_lastPedalCounter     = 0;
static uint8_t s_lastImuCounter       = 0;
static uint8_t s_lastHeartbeatCounter = 0;
static bool    s_pedalCounterValid     = false;
static bool    s_imuCounterValid       = false;
static bool    s_heartbeatCounterValid = false;


static uint32_t s_pedalDroppedFrames     = 0;
static uint32_t s_imuDroppedFrames       = 0;
static uint32_t s_heartbeatDroppedFrames = 0;


static uint32_t s_rxDropCount = 0;


static DriveState s_driveState       = DriveState::BOOT;
static uint8_t    s_lastTorquePct    = 0;
static uint8_t    s_prevButtonFlags  = 0;   // for edge-detecting start presses
static uint32_t   s_readyEnteredAtMs = 0;  //buzzer timer 


static uint32_t s_lastPedalRxMs     = 0;   
static uint32_t s_lastImuRxMs       = 0;   
static uint32_t s_lastHeartbeatRxMs = 0;   

static uint16_t  s_activeFaults        = 0;
static uint16_t  s_latchedFaults       = 0;
static uint16_t  s_lastPublishedFaults = 0xFFFF;
static uint32_t  s_lastFaultPublishMs  = 0;

static uint32_t s_eraseMs = 0; 
static bool s_erasedHeldLast = false;

static bool s_appsBrakeCheck = false;

static uint32_t s_pedalMismatchStartMs = 0;
static constexpr uint32_t PEDAL_MISMATCH_PERSIST_MS = 100;

static bool checkPedalPlausible(uint16_t potA_raw, uint16_t potB_raw, uint8_t* outPedalPct) {

    bool rangeCheckA = (potA_raw <= POT_A_RAW_MAX && potA_raw >= POT_A_RAW_MIN);
    bool rangeCheckB = (potB_raw <= POT_B_RAW_MAX && potB_raw >= POT_B_RAW_MIN);

    if (!rangeCheckA || !rangeCheckB) {
        *outPedalPct = 0;
        s_pedalMismatchStartMs = 0;
        return false;
    }

    int potA_pct = (potA_raw - POT_A_RAW_MIN) * 100 / (POT_A_RAW_MAX - POT_A_RAW_MIN);
    int potB_pct = (potB_raw - POT_B_RAW_MIN) * 100 / (POT_B_RAW_MAX - POT_B_RAW_MIN);
    int pctDiff = abs(potB_pct - potA_pct);
    uint8_t avgPct = (potA_pct + potB_pct) / 2;

    if (pctDiff <= 10) {
        s_pedalMismatchStartMs = 0;
        *outPedalPct = avgPct;
        return true;
    }

    uint32_t now = millis();
    if (s_pedalMismatchStartMs == 0) {
        s_pedalMismatchStartMs = now;  
    }

    if (now - s_pedalMismatchStartMs >= PEDAL_MISMATCH_PERSIST_MS) {
        *outPedalPct = 0;
        return false; 
    }

    *outPedalPct = avgPct; 
    return true;
}

static bool checkBrakePedalPlausible(uint8_t pedalPCT, bool brakePressed){
    if(brakePressed && pedalPCT > 25){
        s_appsBrakeCheck = true;
        return true;
    }
    else if(s_appsBrakeCheck && pedalPCT < 5){
        s_appsBrakeCheck = false;
    }

    return s_appsBrakeCheck;

}

static bool checkImuPlausible(int16_t ax, int16_t ay, int16_t az) {

    int32_t tSquared = (ax*ax) + (ay*ay) + (az*az);
    int32_t lThreash = 900*900; 
    int32_t hThreash = 1100*1100; 

    if(ax < -2000 || ax > 2000){
        return false; 
    }

    if(ay < -2000 || ay > 2000){
        return false; 
    }

    if(az < -2000 || az > 2000){
        return false; 
    }

    if(tSquared <= lThreash || tSquared >=hThreash){
        return false;
    }

    return true; 
}

static DriveState stepDriveState(DriveState current, bool pedalOk, bool imuOk,
                                  uint8_t pedalPct, uint8_t buttonFlags,
                                  uint8_t prevButtonFlags, uint16_t activeFaults,
                                  uint16_t latchedFaults) {

    bool brakePressed = buttonFlags & PedalFlag::BRAKE_PRESSED;
    bool startPressed = buttonFlags & PedalFlag::START_PRESSED;
    bool startJustPressed = (buttonFlags & PedalFlag::START_PRESSED) &&
                             !(prevButtonFlags & PedalFlag::START_PRESSED);

    switch(current){
        case DriveState::BOOT:
            if(pedalOk && imuOk){
                current = DriveState::IDLE; 
            }
            break;
        case DriveState::IDLE:
            
            if(brakePressed && startPressed && pedalPct == 0 && pedalOk && activeFaults == 0 && latchedFaults==0){
                current = DriveState::READY;
                s_readyEnteredAtMs = millis();
            }
            break;
        case DriveState::READY: {
            uint32_t elapsed = millis() - s_readyEnteredAtMs;
            if(elapsed >=2000){
                current = DriveState::DRIVE;
            }
            break;
        }
        case DriveState::DRIVE:
            if(!pedalOk || !imuOk || activeFaults != 0){
                current = DriveState::FAULT;
            }
            else if(startJustPressed){
                current = DriveState::IDLE;
            }
            break;
        case DriveState::FAULT:
            bool bothHeldNow = brakePressed && startPressed;
            if(bothHeldNow && !s_erasedHeldLast){
                //just pressed. 
                s_eraseMs = millis(); 
            }

            else if(bothHeldNow && s_erasedHeldLast && (millis() - s_eraseMs) > 3000){
                //latched NVS disappear
                if(s_latchedFaults != 0 ){
                    s_latchedFaults = 0; 
                    faultStoreSave(s_latchedFaults);
                }
                
            }

            s_erasedHeldLast = bothHeldNow;

            if(pedalOk && imuOk && latchedFaults == 0 && activeFaults==0){
                current = DriveState::IDLE;
            }


            break;  
    }

    return current;
}


static uint8_t computeTorquePct(DriveState state, uint8_t pedalPct) {

    uint8_t torqueTarget = (pedalPct * 7) / 10;

    if(state != DriveState::DRIVE){
        return 0;
    }

    uint8_t newTorque = s_lastTorquePct;
    int change = (int) torqueTarget - (int) newTorque;

    if (change > MAX_STEP) {
        newTorque += MAX_STEP;
    } 
    else if (change < -MAX_STEP) {
        newTorque -= MAX_STEP;
    } 
    else {
        newTorque += change;
    }

    return newTorque;
}

static void applyTorqueOutput(uint8_t torquePct) {
    uint32_t pulseUs = 1000 + ((uint32_t)torquePct * 1000) / 100;  // 1000..2000us
    uint32_t duty = (pulseUs * 4096UL) / 20000UL;                  // 20ms period, 12-bit res
    ledcWrite(LEDC_CH_TORQUE, duty);
}

static void publishVcuStatus(DriveState state, uint8_t torquePct, uint8_t pedalPct) {
    uint8_t bytes[CAN_DLC] = {};
    bytes[StatusMsg::OFF_DRIVE_STATE] = (uint8_t)state;
    bytes[StatusMsg::OFF_TORQUE_PCT] = torquePct;
    bytes[StatusMsg::OFF_ACTIVE_FAULTS]   = s_activeFaults & 0xFF;
    bytes[StatusMsg::OFF_ACTIVE_FAULTS+1] = (s_activeFaults >> 8) & 0xFF;
    bytes[StatusMsg::OFF_PEDAL_PCT] = pedalPct;
    bytes[StatusMsg::OFF_RSVD] = 0; 
    bytes[OFF_COUNTER] = s_counterStatus++;

    uint8_t sum = 0; 
    for(int i = 0; i< 7; i++){
        sum+= bytes[i];
    }
    bytes[OFF_CHECKSUM] = sum;

    canSend(CanId::VCU_STATUS, bytes, CAN_DLC);

}

static void updateTimeoutFaults() {

    uint32_t now = millis(); 
    if(now - s_lastHeartbeatRxMs > TIMEOUT_DIM_MS){
        s_activeFaults |= FaultBit::DIM_TIMEOUT; 
        s_activeFaults |= FaultBit::PEDAL_TIMEOUT;
        s_activeFaults |= FaultBit::IMU_TIMEOUT;
    }
    else{
        s_activeFaults &= ~FaultBit::DIM_TIMEOUT; 
        if(now - s_lastPedalRxMs > TIMEOUT_PEDAL_MS){
            s_activeFaults |= FaultBit::PEDAL_TIMEOUT;
        }
        else{
            s_activeFaults &= ~FaultBit::PEDAL_TIMEOUT;
        }

        if(now-s_lastImuRxMs > TIMEOUT_IMU_MS){
            s_activeFaults |= FaultBit::IMU_TIMEOUT;
        }
        else{
            s_activeFaults &= ~FaultBit::IMU_TIMEOUT;
        }
    }
    


}


static void updateLatchedFaults() {

    uint16_t newLatched = s_latchedFaults | (s_activeFaults & FaultBit::DIM_TIMEOUT);
    if (newLatched != s_latchedFaults) {
        s_latchedFaults = newLatched;
        faultStoreSave(s_latchedFaults); 
  }


}

static FaultCode mostSevereActiveFault(uint16_t activeFaults) {
    //heartbeat most severe 

    if(activeFaults & FaultBit::DIM_TIMEOUT){
        return FaultCode::DIM_TIMEOUT;
    }

    if(activeFaults & FaultBit::PEDAL_IMPLAUSIBLE){
        return FaultCode::PEDAL_IMPLAUSIBLE;
    }

    if(activeFaults & FaultBit::PEDAL_TIMEOUT){
        return FaultCode::PEDAL_TIMEOUT;
    }

    if(activeFaults & FaultBit::IMU_IMPLAUSIBLE){
        return FaultCode::IMU_IMPLAUSIBLE;
    }

    if(activeFaults & FaultBit::IMU_TIMEOUT){
        return FaultCode::IMU_TIMEOUT;
    }

    return FaultCode::NONE;
}

static void publishVcuFault() {
    if(s_activeFaults == s_lastPublishedFaults){
        return ;
    }

    if(millis() - s_lastFaultPublishMs <200){
        return; 
    }

    uint8_t bytes[CAN_DLC] = {};
    bytes[FaultMsg::OFF_FAULT_CODE] = (uint8_t) mostSevereActiveFault (s_activeFaults);
    uint8_t flags = 0; 
    if(s_latchedFaults !=0){
        flags |= FaultMsgFlag::LATCHED;
    }

    uint16_t newer = s_activeFaults &~ s_lastPublishedFaults; 

    if(newer != 0){
        flags |= FaultMsgFlag::NEWLY_SET;
    }

    bytes[FaultMsg::OFF_FLAGS] = flags;

    bytes[FaultMsg::OFF_LATCHED_FAULTS]     = s_latchedFaults & 0xFF;
    bytes[FaultMsg::OFF_LATCHED_FAULTS + 1] = (s_latchedFaults >> 8) & 0xFF;
    bytes[FaultMsg::OFF_NVS_WRITES] = (uint8_t)faultStoreWriteCount(); 
    bytes[FaultMsg::OFF_RSVD] = 0;
    bytes[OFF_COUNTER]  = s_counterFault++;

    uint8_t sum  = 0; 
    for(int i = 0; i< 7; i++){
        sum += bytes[i];
    }
    bytes[OFF_CHECKSUM] = sum; 
    canSend(CanId::VCU_FAULT, bytes, CAN_DLC);

    s_lastPublishedFaults = s_activeFaults;
    s_lastFaultPublishMs  = millis();

}

static void vcuRxTask(void* arg) {
    (void)arg;
    twai_message_t msg;

    for (;;) {
        if (canReceive(&msg, 50)) {
            if (xQueueSend(s_rxQueue, &msg, 0) != pdTRUE) {
                s_rxDropCount++;
            }
        }
    }
}

static void vcuControlTask(void* arg) {
    (void)arg;
    esp_task_wdt_add(NULL);  
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(PERIOD_VCU_CONTROL_MS);
    twai_message_t msg;

    for (;;) {
        esp_task_wdt_reset();

        while (xQueueReceive(s_rxQueue, &msg, 0) == pdTRUE) {
            uint8_t sum = 0;
            for(int i = 0; i < 7; i++){
                sum += msg.data[i];
            }
            if(sum != msg.data[OFF_CHECKSUM]){
                continue;  
            }

            switch(msg.identifier){
                case CanId::DIM_PEDAL: {
                    s_potA_raw = msg.data[PedalMsg::OFF_POT_A] | msg.data[PedalMsg::OFF_POT_A +1] <<8;
                    s_potB_raw = msg.data[PedalMsg::OFF_POT_B] | msg.data[PedalMsg::OFF_POT_B +1] <<8;
                    s_buttonFlags = msg.data[PedalMsg::OFF_FLAGS];
                    uint8_t newCounter = msg.data[OFF_COUNTER];
                    if (s_pedalCounterValid) {
                        uint8_t skipped = newCounter - s_lastPedalCounter - 1;
                        s_pedalDroppedFrames += skipped;
                    }
                    s_lastPedalCounter = newCounter;
                    s_pedalCounterValid = true;
                    s_lastPedalRxMs = millis();
                    break;
                }
                case CanId::DIM_IMU: {
                    s_accelX_mg = msg.data[ImuMsg::OFF_ACCEL_X] | msg.data[ImuMsg::OFF_ACCEL_X +1] << 8;
                    s_accelY_mg = msg.data[ImuMsg::OFF_ACCEL_Y] | msg.data[ImuMsg::OFF_ACCEL_Y+1] << 8;
                    s_accelZ_mg = msg.data[ImuMsg::OFF_ACCEL_Z] | msg.data[ImuMsg::OFF_ACCEL_Z +1] <<8;
                    uint8_t newCounter = msg.data[OFF_COUNTER];
                    if (s_imuCounterValid) {
                        uint8_t skipped = newCounter - s_lastImuCounter - 1;
                        s_imuDroppedFrames += skipped;
                    }
                    s_lastImuCounter = newCounter;
                    s_imuCounterValid = true;
                    s_lastImuRxMs = millis();
                    break;
                }
                case CanId::DIM_HEARTBEAT: {
                    s_uptime = msg.data[HeartbeatMsg::OFF_UPTIME_MS] | msg.data[HeartbeatMsg::OFF_UPTIME_MS+1]<<8 | msg.data[HeartbeatMsg::OFF_UPTIME_MS+2] << 16 | msg.data[HeartbeatMsg::OFF_UPTIME_MS+3] <<24;
                    s_dimState = msg.data[HeartbeatMsg::OFF_DIM_STATE];
                    uint8_t newCounter = msg.data[OFF_COUNTER];
                    if (s_heartbeatCounterValid) {
                        uint8_t skipped = newCounter - s_lastHeartbeatCounter - 1;
                        s_heartbeatDroppedFrames += skipped;
                    }
                    s_lastHeartbeatCounter = newCounter;
                    s_heartbeatCounterValid = true;
                    s_lastHeartbeatRxMs = millis();
                    break;
                }
                default:
                    break;

            }

        }

        uint8_t pedalPct = 0;

        bool brakePressed = s_buttonFlags & PedalFlag::BRAKE_PRESSED;


        bool pedalOk = checkPedalPlausible(s_potA_raw, s_potB_raw, &pedalPct);
        bool imuOk   = checkImuPlausible(s_accelX_mg, s_accelY_mg, s_accelZ_mg);
        

        updateTimeoutFaults();

        if (pedalOk) {
            s_activeFaults &= ~FaultBit::PEDAL_IMPLAUSIBLE;
        }
        else{
            s_activeFaults |=  FaultBit::PEDAL_IMPLAUSIBLE;
        }         

        if (imuOk) {
            s_activeFaults &= ~FaultBit::IMU_IMPLAUSIBLE;
        }
        else{   
            s_activeFaults |=  FaultBit::IMU_IMPLAUSIBLE;
        }

        updateLatchedFaults();

        s_driveState = stepDriveState(s_driveState, pedalOk, imuOk, pedalPct,
                                       s_buttonFlags, s_prevButtonFlags,
                                       s_activeFaults, s_latchedFaults);
        s_prevButtonFlags = s_buttonFlags;

        digitalWrite(PIN_BUZZER, s_driveState == DriveState::READY ? HIGH : LOW);

        uint8_t torquePct = computeTorquePct(s_driveState, pedalPct);
        bool brakePedalActive = checkBrakePedalPlausible(pedalPct, brakePressed);
        

        if(brakePedalActive){
            torquePct = 0; 
        }
        s_lastTorquePct = torquePct;
        applyTorqueOutput(torquePct);

        publishVcuStatus(s_driveState, torquePct, pedalPct);
        publishVcuFault();

        vTaskDelayUntil(&lastWake, period);
    }
}

void vcuStart() {
    esp_err_t wdtErr = esp_task_wdt_init(3, true);  
    if (wdtErr != ESP_OK && wdtErr != ESP_ERR_INVALID_STATE) {
        Serial.println(F("esp_task_wdt_init() failed"));
    }

    if (!faultStoreInit()) {
        Serial.println(F("faultStoreInit() failed -- latched faults won't persist across reboots"));
    }
    s_latchedFaults = faultStoreLoad();

    if(s_latchedFaults !=0){
        s_driveState = DriveState::FAULT;
    }

    s_rxQueue = xQueueCreate(QLEN_VCU_RX, sizeof(twai_message_t));

    xTaskCreatePinnedToCore(vcuRxTask, "vcu_rx",
                            STACK_VCU_RX, nullptr,
                            PRIO_VCU_RX, nullptr, CORE_VCU);

    xTaskCreatePinnedToCore(vcuControlTask, "vcu_control",
                            STACK_VCU_CONTROL, nullptr,
                            PRIO_VCU_CONTROL, nullptr, CORE_VCU);


    ledcSetup(LEDC_CH_TORQUE, 50, 12);
    ledcAttachPin(PIN_PWM_TORQUE, LEDC_CH_TORQUE);
    ledcWrite(LEDC_CH_TORQUE, 0);

    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);

    (void)s_counterFault;
}
