#pragma once



#include <stdint.h>

enum class DriveState : uint8_t {
    BOOT  = 0,  
    IDLE  = 1, 
    READY = 2,  
    DRIVE = 3,  
    FAULT = 4,  
};


enum class FaultCode : uint8_t {
    NONE               = 0,
    PEDAL_IMPLAUSIBLE  = 1,
    IMU_IMPLAUSIBLE    = 2,
    PEDAL_TIMEOUT      = 3,
    IMU_TIMEOUT        = 4,
    DIM_TIMEOUT        = 5,
    CAN_BUS_OFF        = 6, 
};


void vcuStart();
