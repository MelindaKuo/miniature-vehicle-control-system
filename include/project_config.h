#pragma once


#include <stdint.h>

//ms
constexpr uint32_t PERIOD_DIM_PEDAL_MS     = 20;   // 50 Hz
constexpr uint32_t PERIOD_DIM_IMU_MS       = 50;   // 20 Hz
constexpr uint32_t PERIOD_DIM_HEARTBEAT_MS = 100;  // 10 Hz
constexpr uint32_t PERIOD_VCU_CONTROL_MS   = 20;   // 50 Hz, matched to pedal
constexpr uint32_t PERIOD_VCU_STATUS_MS    = 50;   // 20 Hz status broadcast
constexpr uint32_t PERIOD_TELEMETRY_MS     = 100;  // 10 Hz serial log


constexpr uint32_t PRIO_VCU_RX        = 5;
constexpr uint32_t PRIO_VCU_CONTROL   = 4;
constexpr uint32_t PRIO_DIM_PEDAL     = 3;
constexpr uint32_t PRIO_DIM_IMU       = 2;
constexpr uint32_t PRIO_DIM_HEARTBEAT = 1;
constexpr uint32_t PRIO_TELEMETRY     = 1;


constexpr int CORE_DIM = 1;
constexpr int CORE_VCU = 0;


constexpr uint32_t STACK_DIM_PEDAL     = 3072;
constexpr uint32_t STACK_DIM_IMU       = 3072;
constexpr uint32_t STACK_DIM_HEARTBEAT = 2560;
constexpr uint32_t STACK_VCU_RX        = 3072;
constexpr uint32_t STACK_VCU_CONTROL   = 4096;
constexpr uint32_t STACK_TELEMETRY     = 4096;


constexpr uint32_t QLEN_VCU_RX = 16; 


constexpr uint32_t TIMEOUT_PEDAL_MS = 100;  // 5 missed pedal frames
constexpr uint32_t TIMEOUT_IMU_MS   = 250;  // 5 missed IMU frames
constexpr uint32_t TIMEOUT_DIM_MS   = 400;  // 4 missed heartbeats


constexpr uint16_t ADC_MAX_COUNTS = 4095;  

constexpr uint16_t POT_A_RAW_MIN = 0;
constexpr uint16_t POT_A_RAW_MAX = 4095;
constexpr uint16_t POT_B_RAW_MIN = 0;
constexpr uint16_t POT_B_RAW_MAX = 4095;
