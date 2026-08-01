#pragma once

#include <stdint.h>


enum class DimState : uint8_t {
    BOOT      = 0,  
    SENSOR_OK = 1,  
    SENSOR_ERR= 2,   
};


void dimStart();
