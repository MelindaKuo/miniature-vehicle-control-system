#pragma once
// ---------------------------------------------------------------------------


#include <stdint.h>

constexpr uint8_t MPU6050_I2C_ADDR = 0x68;  // AD0 tied/floating low
constexpr uint8_t REG_WHO_AM_I     = 0x75;  // should read back 0x68
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;  // device boots in SLEEP; must clear bit 6
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;  // 6 bytes follow: X,Y,Z, hi byte first


bool mpu6050Init();

bool mpu6050ReadAccelRaw(int16_t& ax, int16_t& ay, int16_t& az);
