#pragma once


#include <stdint.h>

namespace CanId {
    constexpr uint32_t DIM_PEDAL     = 0x100;
    constexpr uint32_t DIM_IMU       = 0x101;
    constexpr uint32_t DIM_HEARTBEAT = 0x102;
    constexpr uint32_t DIM_RESERVED  = 0x103;

    constexpr uint32_t VCU_STATUS    = 0x200;
    constexpr uint32_t VCU_FAULT     = 0x201;
}

constexpr uint8_t CAN_DLC = 8; 


constexpr uint8_t OFF_COUNTER  = 6;
constexpr uint8_t OFF_CHECKSUM = 7;

// 0x100 DIM_PEDAL -- 20 ms

//   [0..1] u16  pot_a_raw     0..4095   raw ADC, APPS1
//   [2..3] u16  pot_b_raw     0..4095   raw ADC, APPS2
//   [4]    u8   button_flags  bitfield, see PedalFlag below
//   [5]    u8   reserved      transmit as 0x00
//   [6]    u8   counter
//   [7]    u8   checksum

namespace PedalMsg {
    constexpr uint8_t OFF_POT_A  = 0;
    constexpr uint8_t OFF_POT_B  = 2;
    constexpr uint8_t OFF_FLAGS  = 4;
    constexpr uint8_t OFF_RSVD   = 5;
}

namespace PedalFlag {
    constexpr uint8_t BRAKE_PRESSED = 0x01;  // bit 0
    constexpr uint8_t START_PRESSED = 0x02;  // bit 1
    // bits 2..7 reserved, transmit as 0
}


// 0x101 DIM_IMU -- 50 ms

//   [0..1] i16  accel_x   milli-g   (1000 = 1.0 g)
//   [2..3] i16  accel_y   milli-g
//   [4..5] i16  accel_z   milli-g
//   [6]    u8   counter
//   [7]    u8   checksum
namespace ImuMsg {
    constexpr uint8_t OFF_ACCEL_X = 0;
    constexpr uint8_t OFF_ACCEL_Y = 2;
    constexpr uint8_t OFF_ACCEL_Z = 4;

    constexpr int16_t MILLI_G_PER_G = 1000;
}


// 0x102 DIM_HEARTBEAT -- 100 ms
//   [0..3] u32  uptime_ms   DIM's millis() at transmit
//   [4]    u8   dim_state   see DimState (Step 3)
//   [5]    u8   reserved
//   [6]    u8   counter
//   [7]    u8   checksum
//
namespace HeartbeatMsg {
    constexpr uint8_t OFF_UPTIME_MS = 0;
    constexpr uint8_t OFF_DIM_STATE = 4;
    constexpr uint8_t OFF_RSVD      = 5;
}


// 0x200 VCU_STATUS -- 50 ms
//   [0]    u8   drive_state   
//   [1]    u8   torque_pct     0..100, the value actually commanded to PWM
//   [2..3] u16  active_faults  
//   [4]    u8   pedal_pct      0..100, post-plausibility validated pedal
//   [5]    u8   reserved
//   [6]    u8   counter
//   [7]    u8   checksum
//

namespace StatusMsg {
    constexpr uint8_t OFF_DRIVE_STATE   = 0;
    constexpr uint8_t OFF_TORQUE_PCT    = 1;
    constexpr uint8_t OFF_ACTIVE_FAULTS = 2;
    constexpr uint8_t OFF_PEDAL_PCT     = 4;
    constexpr uint8_t OFF_RSVD          = 5;
}


namespace FaultBit {
    constexpr uint16_t PEDAL_IMPLAUSIBLE = 0x0001;  // dual-sensor mismatch or out-of-range
    constexpr uint16_t IMU_IMPLAUSIBLE   = 0x0002;  // accel reading failed its sanity check
    constexpr uint16_t PEDAL_TIMEOUT     = 0x0004;  // no DIM_PEDAL frame recently
    constexpr uint16_t IMU_TIMEOUT       = 0x0008;  // no DIM_IMU frame recently
    constexpr uint16_t DIM_TIMEOUT       = 0x0010;  // no DIM_HEARTBEAT frame recently -- the whole node looks dead
    constexpr uint16_t CAN_BUS_OFF       = 0x0020; // bus 
    // bits 5-15 reserved for expansion
}


// 0x201 VCU_FAULT -- on change, rate limited to 200 ms

//   [0]    u8   fault_code                         
//   [1]    u8   flags           bit0 = latched, bit1 = newly set this frame
//   [2..3] u16  latched_faults  bitmask (FaultBit) of everything latched since
//                               boot
//   [4]    u8   nvs_write_count  
//   [5]    u8   reserved
//   [6]    u8   counter
//   [7]    u8   checksum


namespace FaultMsg {
    constexpr uint8_t OFF_FAULT_CODE     = 0;
    constexpr uint8_t OFF_FLAGS          = 1;
    constexpr uint8_t OFF_LATCHED_FAULTS = 2;
    constexpr uint8_t OFF_NVS_WRITES     = 4;
    constexpr uint8_t OFF_RSVD           = 5;
}

namespace FaultMsgFlag {
    constexpr uint8_t LATCHED    = 0x01;  // bit 0
    constexpr uint8_t NEWLY_SET  = 0x02;  // bit 1
}
