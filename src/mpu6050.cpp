//i2c

#include <Wire.h>
#include "mpu6050.h"

bool mpu6050Init() {
    delay(100);

    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x00);
    if(Wire.endTransmission() != 0){
        return false;
    }

    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(REG_WHO_AM_I);
    Wire.endTransmission(false);
    if(Wire.requestFrom(MPU6050_I2C_ADDR, (uint8_t)1) != 1){
        return false;
    }

    uint8_t who = Wire.read();
    return who == 0x68;
}

bool mpu6050ReadAccelRaw(int16_t& ax, int16_t& ay, int16_t& az) {

    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    uint8_t rec = Wire.requestFrom(MPU6050_I2C_ADDR, (uint8_t)6);
    if(rec != 6){
        return false;
    }

    uint8_t xh = Wire.read(); 
    uint8_t xl = Wire.read(); 
    uint8_t yh = Wire.read(); 
    uint8_t yl = Wire.read(); 
    uint8_t zh = Wire.read();
    uint8_t zl = Wire.read(); 

    ax = (int16_t)((xh <<8) | xl);
    ay = (int16_t)((yh << 8)| yl);
    az = (int16_t)((zh << 8)| zl);

    return true;

}
